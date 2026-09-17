// ROS 2 参数服务器桥实现。
//
// 三部分：
//  1) 修改回调（executor 上下文）：校验 + 构造 mailbox config 请求。设备写
//     是异步事务，set 只登记 outstanding，同步返回仅表示「已受理」；
//  2) on_config_result 钩子（motor_bridge 事件循环内同任务调用）：CONFIG_RESULT
//     收敛 outstanding——失败回滚旧值并写 last_error；
//  3) poll（10ms 桥定时器）：动态注册新电机参数、100ms 影子同步、save/refresh
//     脉冲执行、错误码与堆余量回写。
//
// 线程边界：回调与 poll 都在 micro_ros 任务（executor/spin 与定时器同线程），
// 彼此不并发；影响电机只经 motor_rt_submit_config，影响 Flash 只经 storage
// 任务通知，本文件不直接发 FDCAN、不擦写 Flash（ARD §28.1）。

#include "param_bridge.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <string.h>

#include <rcl/error_handling.h>
#include <rclc_parameter/rclc_parameter.h>

#include "motor_rt_task.h"
#include "param_name.h"
#include "param_units.h"
#include "storage_task.h"

// 错误码（system.last_error）：0=无；1..6 对齐 CONFIG 结果七码（reason 1..6）；
// 10=本地校验拒绝；11=mailbox 提交失败（FIFO 满/断链）；12=未绑定/未知电机；
// 20=save 失败；30=refresh 失败；40=设备参数与固件记录不一致（未初始化，该台
// 不可运动）；41=设备参数回读不成功（未核对，同样不可运动）。
// 40/41 由内核结论驱动（dev_init），与请求回告无关：设备真值说了算。
#define ERR_NONE 0
#define ERR_LOCAL_VALIDATION 10
#define ERR_SUBMIT_FAILED 11
#define ERR_UNKNOWN_MOTOR 12
#define ERR_SAVE_FAIL 20
#define ERR_REFRESH_FAIL 30
#define ERR_DEV_INIT_MISMATCH 40
#define ERR_DEV_INIT_UNVERIFIED 41

static const char *const kSystemSave = "system.save";
static const char *const kSystemRefresh = "system.refresh";
static const char *const kSystemLastError = "system.last_error";
static const char *const kSystemLastErrorMotor = "system.last_error_motor";
static const char *const kSystemHeapFree = "system.heap_free";
// 回读完成序号（只读、单调递增）：上位机"点回读 → 等结果"的**唯一可靠判据**。
// 只看 last_error(_motor) 不行——上一轮成功留下的 (0, id) 与本轮无法区分，
// 页面会把"还没读"当成功。每次 param 桥发起的回读收尾（无论成败）都 +1。
static const char *const kSystemRefreshSeq = "system.refresh_seq";
// 设零点（写 = 电机 id，触发）：CERB 写 0x2531=1，仅失能态受理（已使能一律被
// 内核拒绝）。set_zero_seq 是只读完成序号，判据同 refresh_seq。
static const char *const kSystemSetZero = "system.set_zero";
static const char *const kSystemSetZeroSeq = "system.set_zero_seq";
#define kSystemParamCount 8U

static const uint32_t kSyncPeriodMs = 100U;  // 影子同步周期
static const uint32_t kRegisterPerPoll = 2U; // 每拍最多注册电机数
static const uint32_t kMaxMotors =
    (PARAM_BRIDGE_MAX_PARAMS - kSystemParamCount) / PARAM_FIELD_COUNT;

// param 请求的 request_id 标签位：与内核身份读取伪 id（bit31）错开。
#define PARAM_REQUEST_TAG 0x40000000U
// 位置双限暂存标记（同伴字段尚未设置，未提交内核；不匹配任何真实结果事件）。
#define PARAM_REQUEST_STAGED 1U

static rclc_parameter_server_t g_server;
static bool g_created = false;
static bool g_internal_write = false; // 内部 set 重入守卫

struct MotorSlot {
  bool used;
  bool bound;
  uint16_t motor_id;
  // 每个字段在飞请求的 request_id（0=无；STAGED=位置单限暂存）；
  // 成对字段（kp/kd、pos_min/max）共享同一真实 id。
  uint32_t outstanding[PARAM_FIELD_COUNT];
  double old_value[PARAM_FIELD_COUNT]; // 受理时旧值（回滚用，enable 存 0/1）
  double staged_pos[2]; // 位置双限：先到的一侧（索引 = field-POS_MIN）
  float torque_per_amp; // 关节侧 N·m/A（参数力矩列 A ↔ 内核 N·m 的换算系数）
  uint8_t reported_dev_init; // 已上报的设备参数结论（变化才报，避免刷码）
};

static MotorSlot g_motors[MOTOR_CONTROL_MAX_MOTORS];
static motor_control_param_view_t g_views[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_view_count = 0;

static int64_t g_last_error = 0;
static int64_t g_last_error_motor = 0xFFFF;
static bool g_error_pending = false;
static bool g_save_pending = false;
static bool g_refresh_pending = false;
static uint16_t g_refresh_motor = 0;
static uint32_t g_refresh_id = 0;
static uint32_t g_next_request_id = 1U; // 跨会话不复位（防重连后旧事件串台）
static int64_t g_refresh_seq = 0;       // 回读完成序号（只增；重连不复位，页面自比对）
static bool g_set_zero_pending = false;
static uint16_t g_set_zero_motor = 0;
static uint32_t g_set_zero_id = 0;
static int64_t g_set_zero_seq = 0;      // 设零点完成序号（同 refresh_seq 口径）
static uint32_t g_last_sync_ms = 0;
static uint32_t g_last_heap_ms = 0;
static bool g_force_resync = false;

typedef char param_name_buf_t[PARAM_NAME_MAX_LEN];

static MotorSlot *find_slot(uint16_t motor_id) {
  for (size_t i = 0; i < MOTOR_CONTROL_MAX_MOTORS; ++i) {
    if (g_motors[i].used && g_motors[i].motor_id == motor_id) {
      return &g_motors[i];
    }
  }
  return nullptr;
}

static void report_error(int64_t code, uint16_t motor_id) {
  g_last_error = code;
  g_last_error_motor = motor_id;
  g_error_pending = true; // 回调内禁 set，poll 里统一写回
}

static uint32_t next_request_id() {
  g_next_request_id++;
  if (g_next_request_id == 0U) {
    g_next_request_id = 1U;
  }
  return PARAM_REQUEST_TAG | (g_next_request_id & 0x3FFFFFFFU);
}

// 内部写回（同步/回滚/脉冲复位）：必然再进修改回调，守卫直接放行。
static void set_bool_internal(const char *name, bool value) {
  g_internal_write = true;
  (void)rclc_parameter_set_bool(&g_server, name, value);
  g_internal_write = false;
}
static void set_int_internal(const char *name, int64_t value) {
  g_internal_write = true;
  (void)rclc_parameter_set_int(&g_server, name, value);
  g_internal_write = false;
}
static void set_double_internal(const char *name, double value) {
  g_internal_write = true;
  (void)rclc_parameter_set_double(&g_server, name, value);
  g_internal_write = false;
}

static void get_double_or_nan(const char *name, double *out) {
  if (rclc_parameter_get_double(&g_server, name, out) != RCL_RET_OK) {
    *out = NAN;
  }
}

static control_mailbox_config_kind_t field_config_kind(param_field_t field) {
  switch (field) {
    case PARAM_FIELD_ENABLE:
      return CONTROL_MAILBOX_CONFIG_SET_ENABLE;
    case PARAM_FIELD_KP:
    case PARAM_FIELD_KD:
      return CONTROL_MAILBOX_CONFIG_SET_GAINS;
    case PARAM_FIELD_POS_MIN:
    case PARAM_FIELD_POS_MAX:
      return CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
    case PARAM_FIELD_VEL_LIMIT:
      return CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
    case PARAM_FIELD_TORQUE_LIMIT:
      return CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT;
    default:
      return (control_mailbox_config_kind_t)0;
  }
}

// 登记在飞字段（成对），旧值已由调用方写入 slot->old_value。
static void mark_outstanding(MotorSlot *slot, uint32_t request_id,
                             const param_field_t *fields, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    slot->outstanding[fields[i]] = request_id;
  }
}

// 单个限值（速度/力矩）提交。
static bool submit_single_limit(MotorSlot *slot, param_field_t field,
                                double value) {
  if (!param_value_ok(field, (float)value, 0.0f, nullptr)) {
    report_error(ERR_LOCAL_VALIDATION, slot->motor_id);
    return false;
  }
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = slot->motor_id;
  request.kind = field_config_kind(field);
  // 参数服务器对外是工程单位：速度 deg/s、力矩 A（见 param_units.h）。
  if (field == PARAM_FIELD_VEL_LIMIT) {
    request.value.max_velocity_rad_s = param_units_deg_to_rad((float)value);
  } else {
    request.value.max_torque_nm = param_units_amp_to_nm((float)value,
                                                        slot->torque_per_amp);
    if (!isfinite(request.value.max_torque_nm)) {
      report_error(ERR_LOCAL_VALIDATION, slot->motor_id); // 系数未知：不拿瞎猜的换算下发
      return false;
    }
  }
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, slot->motor_id);
    return false;
  }
  param_name_buf_t name;
  (void)param_name_build_motor(name, sizeof(name), slot->motor_id, field);
  get_double_or_nan(name, &slot->old_value[field]);
  mark_outstanding(slot, request.request_id, &field, 1);
  return true;
}

// 增益成对提交。
static bool submit_gains(MotorSlot *slot, param_field_t changed,
                         double new_value, double old_value) {
  param_name_buf_t name;
  double kp = 0.0, kd = 0.0;
  (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                               PARAM_FIELD_KP);
  get_double_or_nan(name, &kp);
  (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                               PARAM_FIELD_KD);
  get_double_or_nan(name, &kd);
  if (changed == PARAM_FIELD_KP) {
    kp = new_value;
  } else {
    kd = new_value;
  }
  if (!param_value_ok(PARAM_FIELD_KP, (float)kp, 0.0f, nullptr) ||
      !param_value_ok(PARAM_FIELD_KD, (float)kd, 0.0f, nullptr)) {
    report_error(ERR_LOCAL_VALIDATION, slot->motor_id);
    return false;
  }
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = slot->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_GAINS;
  request.value.gains.kp = (float)kp;
  request.value.gains.kd = (float)kd;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, slot->motor_id);
    return false;
  }
  const param_field_t companion =
      changed == PARAM_FIELD_KP ? PARAM_FIELD_KD : PARAM_FIELD_KP;
  (void)param_name_build_motor(name, sizeof(name), slot->motor_id, companion);
  double companion_value = 0.0;
  get_double_or_nan(name, &companion_value);
  slot->old_value[changed] = old_value;
  slot->old_value[companion] = companion_value;
  const param_field_t fields[2] = {PARAM_FIELD_KP, PARAM_FIELD_KD};
  mark_outstanding(slot, request.request_id, fields, 2);
  return true;
}

// 位置双限：同伴仍为 NaN（Vaxor 未配置过）时先暂存单侧；两侧到齐再提交。
static bool submit_position_limits(MotorSlot *slot, param_field_t changed,
                                   double new_value, double old_value) {
  const param_field_t other = changed == PARAM_FIELD_POS_MIN
                                  ? PARAM_FIELD_POS_MAX
                                  : PARAM_FIELD_POS_MIN;
  const int self_idx = changed - PARAM_FIELD_POS_MIN;
  const int other_idx = other - PARAM_FIELD_POS_MIN;

  // 另一侧此前暂存？以暂存值为准（两次单设的正常顺序）。
  double other_value = NAN;
  bool other_staged = slot->outstanding[other] == PARAM_REQUEST_STAGED;
  if (other_staged) {
    other_value = slot->staged_pos[other_idx];
  } else {
    param_name_buf_t name;
    (void)param_name_build_motor(name, sizeof(name), slot->motor_id, other);
    get_double_or_nan(name, &other_value);
  }

  if (isnan(other_value)) {
    // 单侧暂存（另一侧从未配置），不提交内核。
    slot->staged_pos[self_idx] = new_value;
    slot->outstanding[changed] = PARAM_REQUEST_STAGED;
    slot->old_value[changed] = old_value;
    return true;
  }

  const double lo = changed == PARAM_FIELD_POS_MIN ? new_value : other_value;
  const double hi = changed == PARAM_FIELD_POS_MAX ? new_value : other_value;
  if (!param_position_limits_ok((float)lo, (float)hi)) {
    report_error(ERR_LOCAL_VALIDATION, slot->motor_id);
    return false;
  }
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = slot->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
  // 成对校验在工程单位（deg）里做，只在提交边界折成内核 SI。
  request.value.position_limits.min_position_rad = param_units_deg_to_rad((float)lo);
  request.value.position_limits.max_position_rad = param_units_deg_to_rad((float)hi);
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, slot->motor_id);
    return false;
  }
  slot->old_value[changed] = old_value;
  slot->old_value[other] = other_value;
  if (other_staged) {
    slot->outstanding[other] = 0U;
  }
  const param_field_t fields[2] = {PARAM_FIELD_POS_MIN, PARAM_FIELD_POS_MAX};
  mark_outstanding(slot, request.request_id, fields, 2);
  return true;
}

// 参数修改回调。返回 false 时 rclc 自动保留旧值（同步拒绝无需手动回滚）。
static bool param_on_modification(const Parameter *old_param,
                                  const Parameter *new_param, void *context) {
  (void)context;
  if (g_internal_write || new_param == nullptr || old_param == nullptr) {
    return true;
  }
  const char *name = new_param->name.data;

  if (strcmp(name, kSystemSave) == 0) {
    if (new_param->value.type == RCLC_PARAMETER_BOOL &&
        new_param->value.bool_value) {
      g_save_pending = true; // busy 期重复脉冲合并
    }
    return true;
  }
  if (strcmp(name, kSystemRefresh) == 0) {
    if (new_param->value.type != RCLC_PARAMETER_INT ||
        new_param->value.integer_value < 0) {
      return false;
    }
    const uint16_t motor_id = (uint16_t)new_param->value.integer_value;
    MotorSlot *slot = find_slot(motor_id);
    if (slot == nullptr || !slot->bound) {
      report_error(ERR_UNKNOWN_MOTOR, motor_id);
      return false;
    }
    g_refresh_pending = true;
    g_refresh_motor = motor_id;
    return true;
  }
  if (strcmp(name, kSystemSetZero) == 0) {
    if (new_param->value.type != RCLC_PARAMETER_INT ||
        new_param->value.integer_value < 0) {
      return false;
    }
    const uint16_t motor_id = (uint16_t)new_param->value.integer_value;
    MotorSlot *slot = find_slot(motor_id);
    if (slot == nullptr || !slot->bound) {
      report_error(ERR_UNKNOWN_MOTOR, motor_id);
      return false;
    }
    g_set_zero_pending = true;
    g_set_zero_motor = motor_id;
    return true;
  }

  uint16_t motor_id = 0;
  param_field_t field = PARAM_FIELD_KP;
  if (param_name_parse(name, &motor_id, &field) != PARAM_NAME_MOTOR) {
    return false;
  }
  MotorSlot *slot = find_slot(motor_id);
  if (slot == nullptr) {
    report_error(ERR_UNKNOWN_MOTOR, motor_id);
    return false;
  }

  if (field == PARAM_FIELD_ENABLE) {
    if (new_param->value.type != RCLC_PARAMETER_BOOL) {
      return false;
    }
    control_mailbox_config_request_t request = {};
    request.request_id = next_request_id();
    request.received_tick_ms = millis();
    request.motor_id = motor_id;
    request.kind = CONTROL_MAILBOX_CONFIG_SET_ENABLE;
    request.value.enable = new_param->value.bool_value;
    if (!motor_rt_submit_config(&request)) {
      report_error(ERR_SUBMIT_FAILED, motor_id);
      return false;
    }
    slot->old_value[PARAM_FIELD_ENABLE] =
        old_param->value.bool_value ? 1.0 : 0.0;
    slot->outstanding[PARAM_FIELD_ENABLE] = request.request_id;
    return true;
  }

  if (new_param->value.type != RCLC_PARAMETER_DOUBLE) {
    return false;
  }
  const double new_value = new_param->value.double_value;
  const double old_value = old_param->value.double_value;

  switch (field_config_kind(field)) {
    case CONTROL_MAILBOX_CONFIG_SET_GAINS:
      return submit_gains(slot, field, new_value, old_value);
    case CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS:
      return submit_position_limits(slot, field, new_value, old_value);
    case CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT:
    case CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT:
      slot->old_value[field] = old_value;
      return submit_single_limit(slot, field, new_value);
    default:
      return false;
  }
}

// ---- poll：注册 / 同步 / 脉冲 ----

static void register_system_params() {
  (void)rclc_add_parameter(&g_server, kSystemSave, RCLC_PARAMETER_BOOL);
  (void)rclc_add_parameter(&g_server, kSystemRefresh, RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemRefreshSeq, RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemSetZero, RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemSetZeroSeq, RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemLastError, RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemLastErrorMotor,
                           RCLC_PARAMETER_INT);
  (void)rclc_add_parameter(&g_server, kSystemHeapFree, RCLC_PARAMETER_INT);
  g_internal_write = true;
  (void)rclc_parameter_set_bool(&g_server, kSystemSave, false);
  (void)rclc_parameter_set_int(&g_server, kSystemRefresh, -1);
  (void)rclc_parameter_set_int(&g_server, kSystemLastError, 0);
  (void)rclc_parameter_set_int(&g_server, kSystemLastErrorMotor, 0xFFFF);
  (void)rclc_parameter_set_int(&g_server, kSystemHeapFree, 0);
  (void)rclc_parameter_set_int(&g_server, kSystemRefreshSeq, g_refresh_seq);
  (void)rclc_parameter_set_int(&g_server, kSystemSetZero, -1);
  (void)rclc_parameter_set_int(&g_server, kSystemSetZeroSeq, g_set_zero_seq);
  g_internal_write = false;
  (void)rclc_set_parameter_read_only(&g_server, kSystemLastError, true);
  (void)rclc_set_parameter_read_only(&g_server, kSystemLastErrorMotor, true);
  (void)rclc_set_parameter_read_only(&g_server, kSystemHeapFree, true);
  (void)rclc_set_parameter_read_only(&g_server, kSystemRefreshSeq, true);
  (void)rclc_set_parameter_read_only(&g_server, kSystemSetZeroSeq, true);
}

static double view_field(const motor_control_param_view_t *v,
                         param_field_t field) {
  switch (field) {
    case PARAM_FIELD_ENABLE:
      return v->enable ? 1.0 : 0.0;
    case PARAM_FIELD_KP:
      return v->kp;
    case PARAM_FIELD_KD:
      return v->kd;
    // 对外工程单位：deg / deg/s / A（见 param_units.h，换算只在这一层做一次）。
    case PARAM_FIELD_POS_MIN:
      return param_units_rad_to_deg(v->pos_min_rad);
    case PARAM_FIELD_POS_MAX:
      return param_units_rad_to_deg(v->pos_max_rad);
    case PARAM_FIELD_VEL_LIMIT:
      return param_units_rad_to_deg(v->velocity_limit_rad_s);
    case PARAM_FIELD_TORQUE_LIMIT:
      return param_units_nm_to_amp(v->torque_limit_nm, v->torque_per_amp);
    default:
      return NAN;
  }
}

static MotorSlot *alloc_slot() {
  for (size_t i = 0; i < MOTOR_CONTROL_MAX_MOTORS; ++i) {
    if (!g_motors[i].used) {
      return &g_motors[i];
    }
  }
  return nullptr;
}

static size_t used_slot_count() {
  size_t n = 0;
  for (size_t i = 0; i < MOTOR_CONTROL_MAX_MOTORS; ++i) {
    if (g_motors[i].used) {
      n++;
    }
  }
  return n;
}

static void register_motor_params(const motor_control_param_view_t *v) {
  MotorSlot *slot = find_slot(v->motor_id);
  if (slot != nullptr) {
    slot->bound = v->bound;
    return;
  }
  if (used_slot_count() >= kMaxMotors) {
    report_error(ERR_UNKNOWN_MOTOR, v->motor_id); // 参数槽位耗尽
    return;
  }
  slot = alloc_slot();
  if (slot == nullptr) {
    return; // used_slot_count 已限上界，理论不可达；防御空指针
  }
  memset(slot, 0, sizeof(*slot));
  slot->used = true;
  slot->motor_id = v->motor_id;
  slot->bound = v->bound;
  slot->torque_per_amp = v->torque_per_amp;

  param_name_buf_t name;
  (void)param_name_build_motor(name, sizeof(name), v->motor_id,
                               PARAM_FIELD_ENABLE);
  (void)rclc_add_parameter(&g_server, name, RCLC_PARAMETER_BOOL);
  for (int f = PARAM_FIELD_KP; f < PARAM_FIELD_COUNT; ++f) {
    (void)param_name_build_motor(name, sizeof(name), v->motor_id,
                                 (param_field_t)f);
    (void)rclc_add_parameter(&g_server, name, RCLC_PARAMETER_DOUBLE);
  }
  // 初值（内部写，不触发业务动作）。NaN 原值透传给 double 参数。
  g_internal_write = true;
  (void)param_name_build_motor(name, sizeof(name), v->motor_id,
                               PARAM_FIELD_ENABLE);
  (void)rclc_parameter_set_bool(&g_server, name, v->enable);
  for (int f = PARAM_FIELD_KP; f < PARAM_FIELD_COUNT; ++f) {
    (void)param_name_build_motor(name, sizeof(name), v->motor_id,
                                 (param_field_t)f);
    (void)rclc_parameter_set_double(&g_server, name,
                                    view_field(v, (param_field_t)f));
  }
  g_internal_write = false;
}

static bool doubles_differ(double a, double b) {
  if (isnan(a)) {
    return !isnan(b);
  }
  return isnan(b) || fabs(a - b) > 1e-6;
}

// 内核视图 → rcl 影子同步（有在飞/暂存请求的字段跳过）。
static void sync_slot_from_view(MotorSlot *slot,
                                const motor_control_param_view_t *v) {
  slot->bound = v->bound;
  slot->torque_per_amp = v->torque_per_amp; // 绑定型号变化时换算系数随之更新
  param_name_buf_t name;
  if (slot->outstanding[PARAM_FIELD_ENABLE] == 0U) {
    bool current = false;
    (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                                 PARAM_FIELD_ENABLE);
    (void)rclc_parameter_get_bool(&g_server, name, &current);
    if (current != v->enable) {
      set_bool_internal(name, v->enable);
    }
  }
  for (int f = PARAM_FIELD_KP; f < PARAM_FIELD_COUNT; ++f) {
    if (slot->outstanding[f] != 0U) {
      continue;
    }
    double current = NAN;
    (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                                 (param_field_t)f);
    (void)rclc_parameter_get_double(&g_server, name, &current);
    const double target = view_field(v, (param_field_t)f);
    if (doubles_differ(current, target)) {
      set_double_internal(name, target);
    }
  }
}

static void do_set_zero_pulse() {
  g_set_zero_pending = false;
  set_int_internal(kSystemSetZero, -1);
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = g_set_zero_motor;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_ZERO;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, g_set_zero_motor);
    return;
  }
  g_set_zero_id = request.request_id;
}

static void do_refresh_pulse() {
  g_refresh_pending = false;
  set_int_internal(kSystemRefresh, -1);
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = g_refresh_motor;
  request.kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, g_refresh_motor);
    return;
  }
  g_refresh_id = request.request_id;
}

void param_bridge_poll(void) {
  if (!g_created) {
    return;
  }
  const uint32_t now = millis();
  g_view_count = motor_rt_copy_param_views(g_views, MOTOR_CONTROL_MAX_MOTORS);

  // 注册新电机（每拍有界）。
  uint32_t registered = 0;
  for (size_t i = 0; i < g_view_count && registered < kRegisterPerPoll; ++i) {
    if (find_slot(g_views[i].motor_id) == nullptr) {
      register_motor_params(&g_views[i]);
      registered++;
    }
  }

  // 脉冲/错误动作（回调内只置位，在 on_callback 之外执行）。
  if (g_error_pending) {
    g_error_pending = false;
    set_int_internal(kSystemLastError, g_last_error);
    set_int_internal(kSystemLastErrorMotor, g_last_error_motor);
  }
  if (g_refresh_pending) {
    do_refresh_pulse();
  }
  if (g_set_zero_pending) {
    do_set_zero_pulse();
  }
  if (g_save_pending) {
    g_save_pending = false;
    storage_task_request_save();
    set_bool_internal(kSystemSave, false);
  }
  if (storage_take_result() == STORAGE_SAVE_FAIL) {
    report_error(ERR_SAVE_FAIL, 0xFFFF);
  }

  // 设备参数结论（未初始化）：按"变化才报"推 system.last_error(_motor)。
  // 结论由内核上电回读比对产生，不随任何请求回告走。
  for (size_t i = 0; i < g_view_count; ++i) {
    MotorSlot *slot = find_slot(g_views[i].motor_id);
    if (slot == nullptr || slot->reported_dev_init == g_views[i].dev_init) {
      continue;
    }
    slot->reported_dev_init = g_views[i].dev_init;
    if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH) {
      report_error(ERR_DEV_INIT_MISMATCH, g_views[i].motor_id);
    } else if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED) {
      report_error(ERR_DEV_INIT_UNVERIFIED, g_views[i].motor_id);
    } else if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_OK ||
               g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_NA) {
      // 结论转好也上报（last_error=0 + motor_id）：上位机据此确认"这次回读
      // 成功"，否则一个陈旧的 40 会永远挂在那里，分不清新旧。
      report_error(ERR_NONE, g_views[i].motor_id);
    }
  }

  // 周期影子同步。
  if (g_force_resync || (uint32_t)(now - g_last_sync_ms) >= kSyncPeriodMs) {
    g_last_sync_ms = now;
    g_force_resync = false;
    for (size_t i = 0; i < g_view_count; ++i) {
      MotorSlot *slot = find_slot(g_views[i].motor_id);
      if (slot != nullptr) {
        sync_slot_from_view(slot, &g_views[i]);
      }
    }
  }

  // 堆余量（1s，台架观测）。
  if ((uint32_t)(now - g_last_heap_ms) >= 1000U) {
    g_last_heap_ms = now;
    set_int_internal(kSystemHeapFree, (int64_t)xPortGetFreeHeapSize());
  }
}

void param_bridge_on_config_result(const motor_control_event_t *event) {
  if (!g_created || event == nullptr ||
      event->type != MOTOR_CONTROL_EVENT_CONFIG_RESULT) {
    return;
  }
  const uint32_t id = event->request_id;
  if ((id & PARAM_REQUEST_TAG) == 0U) {
    return; // 非 param 桥发起
  }
  const bool ok = event->reason == MOTOR_PROTOCOL_CONFIG_OK;

  if (id == g_set_zero_id) {
    g_set_zero_id = 0;
    if (!ok) {
      report_error((int64_t)event->reason, event->motor_id);
    }
    g_set_zero_seq++;
    set_int_internal(kSystemSetZeroSeq, g_set_zero_seq);
    return;
  }
  if (id == g_refresh_id) {
    g_refresh_id = 0;
    if (!ok) {
      report_error(ERR_REFRESH_FAIL, event->motor_id);
    }
    // 收尾序号：上位机据此确认"这次回读已经结束"，再去读结论码
    // （0=一致 / 40=与记录不一致 / 41=回读不成功 / 其余=事务失败码）。
    g_refresh_seq++;
    set_int_internal(kSystemRefreshSeq, g_refresh_seq);
    g_force_resync = true; // 成功：下一拍吸收新设备影子
    return;
  }

  for (size_t i = 0; i < MOTOR_CONTROL_MAX_MOTORS; ++i) {
    MotorSlot *slot = &g_motors[i];
    if (!slot->used || slot->motor_id != event->motor_id) {
      continue;
    }
    bool touched = false;
    for (int f = 0; f < PARAM_FIELD_COUNT; ++f) {
      if (slot->outstanding[f] != id) {
        continue;
      }
      touched = true;
      slot->outstanding[f] = 0U;
      if (!ok) {
        report_error((int64_t)event->reason, event->motor_id);
        param_name_buf_t name;
        (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                                     (param_field_t)f);
        if (f == PARAM_FIELD_ENABLE) {
          set_bool_internal(name, slot->old_value[f] != 0.0);
        } else {
          set_double_internal(name, slot->old_value[f]);
        }
      }
    }
    if (touched && ok) {
      g_force_resync = true; // 吸收内核钳制后的实际值
    }
  }
}

// ---- 生命周期 ----

static_assert(
    PARAM_BRIDGE_EXECUTOR_HANDLES == RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
    "PARAM_BRIDGE_EXECUTOR_HANDLES 与 rclc 的 handle 数不一致");

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) {
    return true; // 幂等
  }
  if (node == nullptr || executor == nullptr) {
    return false;
  }

  rclc_parameter_options_t options = {};
  options.notify_changed_over_dds = false; // 结果回告由 motor_events 承载
  options.max_params = PARAM_BRIDGE_MAX_PARAMS;
  options.allow_undeclared_parameters = false;
  options.low_mem_mode = true; // DTCM newlib 堆预算（约 0.3KB/槽）

  if (rclc_parameter_server_init_with_option(&g_server, node, &options) !=
      RCL_RET_OK) {
    return false;
  }
  g_created = true;

  if (rclc_executor_add_parameter_server(executor, &g_server,
                                         param_on_modification) != RCL_RET_OK) {
    param_bridge_destroy(node);
    return false;
  }
  register_system_params();
  return true;
}

void param_bridge_destroy(rcl_node_t *node) {
  if (!g_created) {
    return;
  }
  g_created = false;
  memset(g_motors, 0, sizeof(g_motors));
  g_view_count = 0;
  g_save_pending = false;
  g_refresh_pending = false;
  g_refresh_id = 0;
  g_error_pending = false;
  g_force_resync = false;
  if (node != nullptr) {
    (void)rclc_parameter_server_fini(&g_server, node);
  }
}

bool param_bridge_created(void) { return g_created; }
