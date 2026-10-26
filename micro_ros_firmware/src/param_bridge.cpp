// 配置回调只提交 mailbox；结果与保存终态在本任务中关联，设备不做原子回滚。
#include "param_bridge.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <can_bridge_msgs/msg/config_request.h>
#include <can_bridge_msgs/msg/system_command.h>
#include <can_bridge_msgs/msg/system_status.h>
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>
#include <rcl/error_handling.h>
#include <rmw_microros/time_sync.h>

#include "motor_rt_task.h"
#include "motor_bridge.h"
#include "config_transaction.hpp"
#include "param_field.h"
#include "param_units.h"
#include "ros_qos.h"
#include "save_transaction.h"
#include "storage_task.h"
#include "wire_codec.h"

static const char *kConfigTopic =
    can_bridge_msgs__msg__WireContract__TOPIC_CONFIG;

// 错误码（system_status.last_error）：0=无；1..6 对齐 CONFIG 结果七码；10=本地校验
// 拒绝；11=mailbox 提交失败（FIFO 满/断链）；12=未绑定/未知电机；20=save 失败；
// 30=refresh 失败；40=设备参数与固件记录不一致；41=回读失败；42=本机无该电机记录。
// 40/41/42 由 dev_init 上报，不参与运动授权；须人工核对设备参数。
#define ERR_NONE 0
#define ERR_LOCAL_VALIDATION 10
#define ERR_SUBMIT_FAILED 11
#define ERR_UNKNOWN_MOTOR 12
#define ERR_SAVE_FAIL 20
#define ERR_REFRESH_FAIL 30
#define ERR_DEV_INIT_MISMATCH 40
#define ERR_DEV_INIT_UNVERIFIED 41
#define ERR_DEV_INIT_NO_RECORD 42

#define kMotorIdNone 0xFFFFU

// 线协议目标上界取自生成常量。
#define kParamSetMaxTargets \
  can_bridge_msgs__msg__ConfigRequest__targets__MAX_SIZE

// 每目标（位置限位对算一次）提交一次；消息上界不得超过 FIFO 深度，避免部分入队。
static_assert(kParamSetMaxTargets <= CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY,
    "ConfigRequest 上界超过配置 FIFO 深度");

// executor 占一条订阅句柄，发布不占句柄。
static_assert(PARAM_BRIDGE_EXECUTOR_HANDLES == 1U,
    "PARAM_BRIDGE_EXECUTOR_HANDLES 必须等于本文件的订阅数");
static_assert(MOTOR_CONTROL_MAX_MOTORS == SAVE_TRANSACTION_MAX_TARGETS,
    "SAVE 事务容量必须与电机视图容量一致");

static_assert(kParamSetMaxTargets <= config_transaction::kMaxTargets,
    "配置内容缓存必须容纳全部 wire targets");
static_assert(can_bridge_msgs__msg__ConfigResult__STATUS_PENDING == config_transaction::kPending &&
    can_bridge_msgs__msg__ConfigResult__STATUS_SUCCEEDED == config_transaction::kSucceeded &&
    can_bridge_msgs__msg__ConfigResult__STATUS_FAILED == config_transaction::kFailed &&
    can_bridge_msgs__msg__ConfigResult__STATUS_BUSY == config_transaction::kBusy &&
    can_bridge_msgs__msg__ConfigResult__STATUS_CONFLICT == config_transaction::kConflict,
    "配置结果状态必须与 wire 契约一致");

// ---- 线协议 field 表 ----
// 下标 = ParamSetTarget.field = param_device_slot_t
// （0=pos_min 1=pos_max 2=vel_limit 3=torque_limit 4=limit_protect 5=direction）。
//
// wire field 与 param_field_t 序号不同，混用会写错槽位。
// 可操作集合由本表显式定义；新增枚举不会自动开放操作。
typedef struct {
  param_field_t param_field;            // Flash blob / 视图槽位序
  control_mailbox_config_kind_t kind;   // 内核动作
} param_field_desc_t;

static const param_field_desc_t kFields[PARAM_DEVICE_SLOT_COUNT] = {
  {PARAM_FIELD_POS_MIN, CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS},
  {PARAM_FIELD_POS_MAX, CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS},
  {PARAM_FIELD_VEL_LIMIT, CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT},
  {PARAM_FIELD_TORQUE_LIMIT, CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT},
  {PARAM_FIELD_LIMIT_PROTECT, CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT},
  {PARAM_FIELD_DIRECTION, CONTROL_MAILBOX_CONFIG_SET_DIRECTION},
};

static_assert(sizeof(kFields) / sizeof(kFields[0]) == PARAM_DEVICE_SLOT_COUNT,
    "线协议 field 表长度必须等于设备槽位数：加槽位要同时改 .msg");
// 线协议 field 的取值范围就是设备槽序，两端同序由这条断言与 host 测试共同保证。
static_assert(PARAM_DEVICE_SLOT_COUNT == 6U,
    "设备槽位数变了：同步改 .msg 的 ParamSetTarget 常量与线协议表");

// 内核操作使用 bit30 命名空间；外部 request_id 只在事务表内关联。
#define PARAM_INTERNAL_ID_TAG 0x40000000U

// ---- 状态 ----
static rcl_subscription_t g_config_sub;
static can_bridge_msgs__msg__ConfigRequest g_config_msg;
static can_bridge_msgs__msg__ParamSetTarget g_config_targets_buf[kParamSetMaxTargets];
static can_bridge_msgs__msg__SystemStatus g_system_status_msg;
static bool g_sub_up = false;
static bool g_sub_added = false;
static bool g_created = false;

// 最近一次 poll 的参数视图，供 motor_bridge 发布 device_param_report。
static motor_control_param_view_t g_views[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_view_count = 0;

static uint16_t g_last_error = ERR_NONE;
static uint16_t g_last_error_motor = kMotorIdNone;

static uint32_t g_next_request_id = 1U; // 跨会话不复位（防重连后旧事件串台）

// 完成序号保留给旧 facade；新调用方消费带原请求 ID 的 ConfigResult。
static int64_t g_refresh_seq = 0;

static int64_t g_set_zero_seq = 0;

static int64_t g_clear_fault_seq = 0;


// COMMAND_REPORT 的一次性请求：回调置位，motor_bridge 下一拍 poll 取走并发一帧
// device_param_report。不进 mailbox、不触发 SDO——没在线电机时帧里就是板内表。
static bool g_report_pending = false;

static save_transaction_t g_save_transaction;
static int64_t g_save_seq = 0;

static config_transaction::Coordinator g_transactions;
static int g_active_slot = -1;
static uint64_t g_observed_session = 0;
static int g_save_slot = -1;
static uint16_t g_save_total = 0, g_save_completed = 0, g_save_error = 0;
static bool g_sequence_pending[config_transaction::kCapacity] = {};

static void update_completion_sequences() {
  for (size_t i = 0; i < config_transaction::kCapacity; ++i) {
    const auto *r = g_transactions.result(static_cast<int>(i));
    if (!g_sequence_pending[i] || r == nullptr ||
        r->status == config_transaction::kPending) {
      continue;
    }
    g_sequence_pending[i] = false;
    switch (r->command) {
    case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH: ++g_refresh_seq; break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO: ++g_set_zero_seq; break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT: ++g_clear_fault_seq; break;
    default: break;
    }
  }
}

static void sync_config_session() {
  const uint64_t session = motor_bridge_session_id();
  if (session == g_observed_session) {
    return;
  }
  // 新会话不继承旧 SAVE 的游标，也不让旧结果占住当前结果槽。
  save_transaction_disconnect(&g_save_transaction);
  if (g_save_slot >= 0) { ++g_save_seq; }
  g_save_slot = -1;
  g_report_pending = false;
  g_transactions.disconnect();
  update_completion_sequences();
  g_transactions.set_session(session);
  memset(g_sequence_pending, 0, sizeof(g_sequence_pending));
  g_observed_session = session;
}

// 已上报的设备参数结论（下标 = motor_id；0xFF = 从未上报）。
// 用电机 id 做 key 而不是视图下标：视图顺序不是契约，内核表项只增不减。
// 当前协议节点号为 1..127；ID 范围独立于注册数量上限。
static uint8_t g_reported_dev_init[128U];

static const motor_control_param_view_t *find_view(uint16_t motor_id) {
  for (size_t i = 0; i < g_view_count; ++i) {
    if (g_views[i].motor_id == motor_id) {
      return &g_views[i];
    }
  }
  return nullptr;
}

static void report_error(uint16_t code, uint16_t motor_id) {
  g_last_error = code;
  g_last_error_motor = motor_id;
}

// 内部操作 ID 跨重连递增，避免迟到事件关联到新请求。
static uint32_t next_internal_id() {
  g_next_request_id++;
  if (g_next_request_id == 0U) {
    g_next_request_id = 1U;
  }
  return PARAM_INTERNAL_ID_TAG | (g_next_request_id & 0x3FFFFFFFU);
}

static uint32_t save_next_request_id(void *context) {
  (void)context;
  return next_internal_id();
}

static bool save_submit_device(void *context, uint32_t request_id,
    uint16_t motor_id, uint8_t dirty_slots) {
  (void)context;
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;
  request.value.save_slots = dirty_slots;
  return motor_rt_submit_config(&request);
}

static bool save_request_storage(void *context, uint32_t request_id) {
  (void)context;
  storage_task_request_save(request_id);
  return true;
}

static save_transaction_storage_result_t save_take_storage_result(
    void *context, uint32_t *request_id) {
  (void)context;
  switch (storage_take_result(request_id)) {
  case STORAGE_SAVE_OK:
    return SAVE_TRANSACTION_STORAGE_OK;
  case STORAGE_SAVE_FAIL:
    return SAVE_TRANSACTION_STORAGE_FAIL;
  case STORAGE_SAVE_IDLE:
  default:
    return SAVE_TRANSACTION_STORAGE_IDLE;
  }
}

static void save_report_error(void *context,
    save_transaction_error_t reason,
    uint16_t motor_id,
    uint16_t device_error_code) {
  (void)context;
  g_save_error = device_error_code ? device_error_code : ERR_SAVE_FAIL;
  switch (reason) {
  case SAVE_TRANSACTION_ERROR_SUBMIT:
    report_error(ERR_SUBMIT_FAILED, motor_id);
    break;
  case SAVE_TRANSACTION_ERROR_DEVICE:
    report_error(device_error_code, motor_id);
    break;
  case SAVE_TRANSACTION_ERROR_TIMEOUT:
  case SAVE_TRANSACTION_ERROR_STORAGE:
  default:
    report_error(ERR_SAVE_FAIL, kMotorIdNone);
    break;
  }
}

static const save_transaction_ports_t kSaveTransactionPorts = {
  .context = nullptr,
  .next_request_id = save_next_request_id,
  .submit_device_save = save_submit_device,
  .request_storage_save = save_request_storage,
  .take_storage_result = save_take_storage_result,
  .report_error = save_report_error,
};

// ---- 提交 ----

// 公共头的请求 ID 在提交点换成独立操作 ID。
static control_mailbox_config_request_t make_config_request(
    const motor_control_param_view_t *v, control_mailbox_config_kind_t kind,
    uint32_t request_id) {
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = v->motor_id;
  request.kind = kind;
  return request;
}

// 提交并记账：此处失败只可能是 FIFO 满/断链（值域已在整条预校验阶段过完）。
static bool submit_config_request(const control_mailbox_config_request_t *request,
    uint16_t motor_id) {
  control_mailbox_config_request_t correlated = *request;
  correlated.request_id = next_internal_id();
  if (!motor_rt_submit_config(&correlated)) {
    report_error(ERR_SUBMIT_FAILED, motor_id);
    return false;
  }
  // 一条消息至多 16 次提交；事件由本任务稍后消费，不会先于记账到达。
  if (!g_transactions.track(g_active_slot, correlated.request_id)) {
    report_error(ERR_SUBMIT_FAILED, motor_id);
    return false;
  }
  return true;
}

// 速度/力矩限值提交；值域须提前整条校验，避免部分生效。失败表示 FIFO 满或断链。
static bool submit_single_limit(const motor_control_param_view_t *v,
    const param_field_desc_t *desc, float value,
    uint32_t request_id) {
  control_mailbox_config_request_t request =
      make_config_request(v, desc->kind, request_id);
  // 单位口径见 param_units.h：速度对外就是 rad/s（内核同单位，直通），力矩对外
  // 是 A（设备对象 0x2538 存的就是电流）。换算只在这一处做。
  if (desc->param_field == PARAM_FIELD_VEL_LIMIT) {
    request.value.max_velocity_rad_s = value;
  } else {
    request.value.max_torque_nm = param_units_amp_to_nm(value, v->torque_per_amp);
  }
  return submit_config_request(&request, v->motor_id);
}

// 位置限位须预校验成对；本层不暂存或补齐。
static bool submit_position_limits(const motor_control_param_view_t *v, float lo,
    float hi, uint32_t request_id) {
  control_mailbox_config_request_t request = make_config_request(
      v, CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS, request_id);
  // 位置对外就是 rad（内核同单位，直通、不折算角度）。
  request.value.position_limits.min_position_rad = lo;
  request.value.position_limits.max_position_rad = hi;
  return submit_config_request(&request, v->motor_id);
}

// 保护开关唯一反转点：对外 1=保护生效，厂家原值 0=保护开；下层只接收原值。
static bool submit_safety_protect(const motor_control_param_view_t *v,
    float value, uint32_t request_id) {
  control_mailbox_config_request_t request = make_config_request(
      v, CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT, request_id);
  request.value.safety_protect_raw = (value == 1.0f) ? 0U : 1U;
  return submit_config_request(&request, v->motor_id);
}

// 方向按厂家原值直通，不做语义反转。
static bool submit_direction(const motor_control_param_view_t *v, float value,
    uint32_t request_id) {
  control_mailbox_config_request_t request =
      make_config_request(v, CONTROL_MAILBOX_CONFIG_SET_DIRECTION, request_id);
  request.value.direction_raw = (value == 1.0f) ? 1U : 0U;
  return submit_config_request(&request, v->motor_id);
}

// 单目标分发（位置两项走成对路径，不到这里）。返回 false 时 last_error 已置。
static bool dispatch_target(const motor_control_param_view_t *v, uint8_t field,
    float value, uint32_t request_id) {
  switch (field) {
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_VEL_LIMIT:
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_TORQUE_LIMIT:
    return submit_single_limit(v, &kFields[field], value, request_id);
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_LIMIT_PROTECT:
    return submit_safety_protect(v, value, request_id);
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_DIRECTION:
    return submit_direction(v, value, request_id);
  default:
    return false;   // POS_MIN/POS_MAX 走成对路径；越界 field 已被预校验拒掉
  }
}

// ---- 整条校验 ----
// 整条预校验，不提交 mailbox、不发布；失败设置 last_error 并整条拒绝。
static bool validate_param_set(
    const can_bridge_msgs__msg__ParamSetTarget *targets, uint16_t count) {
  if (count > kParamSetMaxTargets) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    return false;
  }
  for (uint16_t i = 0; i < count; ++i) {
    const can_bridge_msgs__msg__ParamSetTarget &t = targets[i];
    if (t.field >= PARAM_DEVICE_SLOT_COUNT) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    const motor_control_param_view_t *v = find_view(t.motor_id);
    if (v == nullptr) {
      report_error(ERR_UNKNOWN_MOTOR, t.motor_id);
      return false;
    }
    // 上界参数传 0：内核侧还会按型号额定再钳一次，本层只做纯值域判定。
    if (!param_value_ok(kFields[t.field].param_field, t.value, 0.0f, nullptr)) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    // 力矩 A→N·m 的系数未知时换算非有限，不能下发；放进整条预校验避免部分生效。
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_TORQUE_LIMIT &&
        !isfinite(param_units_amp_to_nm(t.value, v->torque_per_amp))) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    // 同一 (motor_id, field) 出现两次：静默取一个会掩盖调用方 bug。
    for (uint16_t j = 0; j < i; ++j) {
      if (targets[j].motor_id == t.motor_id && targets[j].field == t.field) {
        report_error(ERR_LOCAL_VALIDATION, t.motor_id);
        return false;
      }
    }
  }
  // 位置限位成对：同一条消息内该台必须两边都给，且 min < max。
  const uint8_t pos_bits = (uint8_t)((1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) |
      (1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX));
  for (uint16_t i = 0; i < count; ++i) {
    const uint16_t id = targets[i].motor_id;
    uint8_t mask = 0U;
    float lo = NAN;
    float hi = NAN;
    for (uint16_t j = 0; j < count; ++j) {
      const can_bridge_msgs__msg__ParamSetTarget &p = targets[j];
      if (p.motor_id != id) {
        continue;
      }
      mask |= (uint8_t)(1U << p.field);
      if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) {
        lo = p.value;
      } else if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
        hi = p.value;
      }
    }
    if ((mask & pos_bits) == 0U) {
      continue; // 本台没写位置限位
    }
    if ((mask & pos_bits) != pos_bits) {
      report_error(ERR_LOCAL_VALIDATION, id);
      return false;
    }
    if (!param_position_limits_ok(lo, hi)) {
      report_error(ERR_LOCAL_VALIDATION, id);
      return false;
    }
  }
  return true;
}

static void apply_param_set(
    const can_bridge_msgs__msg__ParamSetTarget *targets, uint16_t count,
    uint32_t request_id) {
  uint16_t total = count;
  for (uint16_t i = 0; i < count; ++i) {
    if (targets[i].field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
      --total;
    }
  }
  bool consumed[kParamSetMaxTargets] = {};
  for (uint16_t i = 0; i < count; ++i) {
    if (consumed[i]) {
      continue;
    }
    const can_bridge_msgs__msg__ParamSetTarget &t = targets[i];
    const motor_control_param_view_t *v = find_view(t.motor_id);
    if (v == nullptr) {
      continue; // 预校验已确认存在；防御
    }
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN ||
        t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
      float lo = NAN;
      float hi = NAN;
      for (uint16_t j = 0; j < count; ++j) {
        const can_bridge_msgs__msg__ParamSetTarget &p = targets[j];
        if (p.motor_id != t.motor_id) {
          continue;
        }
        if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) {
          lo = p.value;
          consumed[j] = true;
        } else if (p.field ==
            can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
          hi = p.value;
          consumed[j] = true;
        }
      }
      if (!submit_position_limits(v, lo, hi, request_id)) {
        g_transactions.submitted(g_active_slot, total, ERR_SUBMIT_FAILED);
        return;
      }
      continue;
    }
    consumed[i] = true;
    if (!dispatch_target(v, t.field, t.value, request_id)) {
      g_transactions.submitted(g_active_slot, total, ERR_SUBMIT_FAILED);
      return;
    }
  }
  g_transactions.submitted(g_active_slot, total, 0);
}

static void handle_system_command(uint8_t command, uint16_t motor_id,
    uint32_t request_id) {
  if (command == can_bridge_msgs__msg__SystemCommand__COMMAND_SAVE) {
    if (motor_id != can_bridge_msgs__msg__SystemCommand__MOTOR_ID_NONE) {
      report_error(ERR_LOCAL_VALIDATION, motor_id);
      g_transactions.complete(g_active_slot, 0, 0, ERR_LOCAL_VALIDATION);
      return;
    }
    // 保存快照与参数写互斥；不能把在飞写入的一半投影到 W25。
    if (g_transactions.pending_except(g_active_slot) ||
        save_transaction_in_progress(&g_save_transaction)) {
      g_transactions.busy(g_active_slot);
      return;
    }
    g_save_slot = g_active_slot;
    g_save_total = g_save_completed = g_save_error = 0;
    g_transactions.external(g_save_slot);
    save_transaction_request(&g_save_transaction);
    return;
  }
  if (command == can_bridge_msgs__msg__SystemCommand__COMMAND_REPORT) {
    g_report_pending = true;
    return;
  }
  control_mailbox_config_kind_t kind;
  switch (command) {
  case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH:
    kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS; break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO:
    kind = CONTROL_MAILBOX_CONFIG_SET_ZERO; break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT:
    kind = CONTROL_MAILBOX_CONFIG_CLEAR_FAULT; break;
  default:
    report_error(ERR_LOCAL_VALIDATION, motor_id);
    g_transactions.complete(g_active_slot, 0, 0, ERR_LOCAL_VALIDATION);
    return;
  }
  g_sequence_pending[g_active_slot] = true;
  const motor_control_param_view_t *v = find_view(motor_id);
  if (v == nullptr || !v->bound) {
    report_error(ERR_UNKNOWN_MOTOR, motor_id);
    g_transactions.complete(g_active_slot, 0, 1, ERR_UNKNOWN_MOTOR);
    return;
  }
  auto request = make_config_request(v, kind, request_id);
  const bool ok = submit_config_request(&request, motor_id);
  g_transactions.submitted(g_active_slot, 1, ok ? 0 : ERR_SUBMIT_FAILED);
}

static void config_cb(const void *msgin) {
  const auto *msg = static_cast<const can_bridge_msgs__msg__ConfigRequest *>(msgin);
  if (msg == nullptr) {
    return;
  }
  sync_config_session();
  update_completion_sequences();
  config_transaction::Request request{};
  request.session_id = msg->session_id;
  request.sync_time_ns = msg->sync_time_ns;
  request.request_id = msg->request_id;
  request.kind = msg->kind;
  request.command = msg->command;
  request.motor_id = msg->motor_id;
  request.count = msg->count;
  request.size = msg->targets.size;
  if (request.size <= config_transaction::kMaxTargets) {
    for (size_t i = 0; i < request.size; ++i) {
      request.targets[i] = {msg->targets.data[i].motor_id,
                            msg->targets.data[i].field, msg->targets.data[i].value};
    }
  }
  g_active_slot = g_transactions.begin(request, motor_bridge_session_id(), millis());
  if (g_active_slot < 0) {
    return;
  }
  if (!wire_config_groups_exclusive(msg->kind, msg->count,
      msg->targets.size, msg->command, msg->motor_id)) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    g_transactions.complete(g_active_slot, 0, 0, ERR_LOCAL_VALIDATION);
    return;
  }
  if (save_transaction_in_progress(&g_save_transaction) &&
      !(msg->kind == WIRE_CONFIG_KIND_SYSTEM_COMMAND &&
      msg->command == can_bridge_msgs__msg__SystemCommand__COMMAND_REPORT)) {
    g_transactions.busy(g_active_slot);
    return;
  }
  if (msg->kind == WIRE_CONFIG_KIND_PARAM_SET) {
    if (!validate_param_set(msg->targets.data, msg->count)) {
      g_transactions.complete(g_active_slot, 0, 0, g_last_error);
      return;
    }
    apply_param_set(msg->targets.data, msg->count, msg->request_id);
  } else {
    handle_system_command(msg->command, msg->motor_id, msg->request_id);
  }
  update_completion_sequences();
}

// ---- 内核事件回告 ----

void param_bridge_on_config_result(const motor_control_event_t *event) {
  if (!g_created || event == nullptr ||
      event->type != MOTOR_CONTROL_EVENT_CONFIG_RESULT) {
    return;
  }
  const uint32_t id = event->request_id;
  const bool ok = event->reason == MOTOR_PROTOCOL_CONFIG_OK;

  if (save_transaction_on_device_result(
      &g_save_transaction, id, ok, event->motor_id,
      (uint16_t)event->reason, &kSaveTransactionPorts)) {
    if (ok) {
      ++g_save_completed;
    }
    return;
  }

  const bool matched = g_transactions.on_result(id, ok, (uint16_t)event->reason);
  if (!ok) {
    report_error((uint16_t)event->reason, event->motor_id);
  }
  if (matched) {
    update_completion_sequences();
  }
}

// ---- 状态快照 ----

const can_bridge_msgs__msg__SystemStatus *param_bridge_system_status(void) {
  g_system_status_msg.sync_time_ns = rmw_uros_epoch_nanos();
  g_system_status_msg.last_error = g_last_error;
  g_system_status_msg.last_error_motor = g_last_error_motor;
  g_system_status_msg.refresh_seq = (uint32_t)g_refresh_seq;
  g_system_status_msg.set_zero_seq = (uint32_t)g_set_zero_seq;
  g_system_status_msg.clear_fault_seq = (uint32_t)g_clear_fault_seq;
  g_system_status_msg.save_seq = (uint32_t)g_save_seq;
  g_system_status_msg.save_in_progress =
      save_transaction_in_progress(&g_save_transaction);
  return &g_system_status_msg;
}

// SAVE 状态机独立于 ROS 消息、motor_control 视图和 storage_task 实现；本桥只
// 转换视图快照、提供设备/存储端口，维护状态序号。
static void poll_save(uint32_t now) {
  save_transaction_target_t targets[SAVE_TRANSACTION_MAX_TARGETS] = {};
  size_t target_count = g_view_count;
  if (target_count > SAVE_TRANSACTION_MAX_TARGETS) {
    target_count = SAVE_TRANSACTION_MAX_TARGETS;
  }
  for (size_t i = 0U; i < target_count; ++i) {
    targets[i].motor_id = g_views[i].motor_id;
    targets[i].dirty_slots = g_views[i].record_dirty_slots;
  }

  if (g_save_transaction.request_pending &&
      g_save_transaction.phase == SAVE_TRANSACTION_PHASE_IDLE) {
    g_save_total = 0;
    for (size_t i = 0; i < target_count; ++i) {
      if (targets[i].dirty_slots != 0) { ++g_save_total; }
    }
  }
  const save_transaction_effects_t effects = save_transaction_poll(
      &g_save_transaction, now, targets, target_count,
      &kSaveTransactionPorts);
  if (effects.completed) {
    // 完成序号是动作结束序号，不是成功计数；失败也必须推进，避免上层永久等待。
    ++g_save_seq;
    if (g_save_slot >= 0) {
      g_transactions.complete(g_save_slot, g_save_completed,
          g_save_total - g_save_completed, g_save_error);
      g_save_slot = -1;
    }
  }
  if (effects.status_changed) {
  }
}

// 内核上电回读与固件记录比对的结论，仅在变化时更新 last_error，与请求回告无关。
static void report_dev_init_changes(void) {
  for (size_t i = 0; i < g_view_count; ++i) {
    const motor_control_param_view_t *v = &g_views[i];
    if (v->motor_id >= sizeof(g_reported_dev_init) ||
        g_reported_dev_init[v->motor_id] == v->dev_init) {
      continue;
    }
    g_reported_dev_init[v->motor_id] = v->dev_init;
    switch (v->dev_init) {
    case MOTOR_CONTROL_DEV_INIT_MISMATCH:
      report_error(ERR_DEV_INIT_MISMATCH, v->motor_id);
      break;
    case MOTOR_CONTROL_DEV_INIT_UNVERIFIED:
      report_error(ERR_DEV_INIT_UNVERIFIED, v->motor_id);
      break;
    case MOTOR_CONTROL_DEV_INIT_NO_RECORD:
      report_error(ERR_DEV_INIT_NO_RECORD, v->motor_id);
      break;
    case MOTOR_CONTROL_DEV_INIT_OK:
    case MOTOR_CONTROL_DEV_INIT_NA:
      // 恢复正常也上报，清除上一轮错误。
      report_error(ERR_NONE, v->motor_id);
      break;
    default:
      break;   // PENDING 不是结论，不报
    }
  }
}

void param_bridge_poll(void) {
  if (!g_created) {
    return;
  }
  sync_config_session();
  const uint32_t now = millis();
  g_view_count = motor_rt_copy_param_views(g_views, MOTOR_CONTROL_MAX_MOTORS);

  poll_save(now);
  report_dev_init_changes();

  g_transactions.poll(now);
  update_completion_sequences();
}

uint32_t param_bridge_publish_fail_count(void) {
  return 0U;
}

const motor_control_param_view_t *param_bridge_views(size_t *out_count) {
  if (out_count != nullptr) {
    *out_count = g_view_count;
  }
  return g_views;
}

bool param_bridge_take_report_request(void) {
  bool pending = false;
  const bool requested = g_report_pending;
  g_report_pending = false;
  if (requested) {
    for (size_t i = 0; i < config_transaction::kCapacity; ++i) {
      const auto *r = g_transactions.result(static_cast<int>(i));
      if (r != nullptr && r->status == config_transaction::kPending &&
          r->kind == WIRE_CONFIG_KIND_SYSTEM_COMMAND &&
          r->command == can_bridge_msgs__msg__SystemCommand__COMMAND_REPORT) {
        g_transactions.complete(static_cast<int>(i), 0, 0, 0);
        pending = true;
      }
    }
  }
  return pending;
}

const motor_control_stored_params_t *param_bridge_stored_records(
    size_t *out_count) {
  return storage_boot_params(out_count);
}

// ---- 生命周期 ----

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) {
    return true; // 幂等
  }
  if (node == nullptr || executor == nullptr) {
    return false;
  }

  save_transaction_init(&g_save_transaction);

  // 反序列化写入静态序列缓冲；初始化前必须设置 data/capacity，rclc 不代为分配。
  g_config_msg.targets.capacity =
      sizeof(g_config_targets_buf) / sizeof(g_config_targets_buf[0]);
  g_config_msg.targets.data = g_config_targets_buf;
  if (g_config_msg.targets.capacity < kParamSetMaxTargets) {
    return false;
  }

  memset(g_reported_dev_init, 0xFF, sizeof(g_reported_dev_init));
  g_view_count = 0;

  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  // 配置/系统命令面是低频状态声明，reliable；< 512 B MTU，不分片。
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_config_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, ConfigRequest),
      kConfigTopic, &sub_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_up = true;

  if (rclc_executor_add_subscription(executor, &g_config_sub,
      &g_config_msg, config_cb, ON_NEW_DATA) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_added = true;

  g_created = true;
  return true;
}

void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (!g_created && !g_sub_up) {
    return;
  }
  g_created = false;
  g_view_count = 0;
  memset(g_reported_dev_init, 0xFF, sizeof(g_reported_dev_init));
  // 重建后视图会重填，保存事务不得沿用旧游标。
  save_transaction_disconnect(&g_save_transaction);
  if (g_save_slot >= 0) {
    g_transactions.complete(g_save_slot, g_save_completed,
        g_save_total - g_save_completed, ERR_SAVE_FAIL);
    ++g_save_seq;
    g_save_slot = -1;
  }
  g_transactions.disconnect();
  update_completion_sequences();
  g_report_pending = false;

  // fini 不会移除 executor 句柄；订阅须先移除，避免重连后句柄残留。
  if (executor != nullptr) {
    if (g_sub_added) {
      (void)rclc_executor_remove_subscription(executor, &g_config_sub);
    }
  }
  g_sub_added = false;
  // fini 的 warn_unused_result 要求接收返回值；清理失败无补救动作。
  rcl_ret_t rc = RCL_RET_OK;
  if (g_sub_up) {
    rc = rcl_subscription_fini(&g_config_sub, node);
    g_sub_up = false;
  }
  (void)rc;
}

bool param_bridge_result_peek(can_bridge_msgs__msg__ConfigResult *out) {
  if (out == nullptr) { return false; }
  config_transaction::Result result{};
  if (!g_transactions.peek(result)) { return false; }
  out->session_id = result.session_id;
  out->request_id = result.request_id;
  out->kind = result.kind;
  out->command = result.command;
  out->status = result.status;
  out->error_code = result.error_code;
  out->completed_targets = result.completed_targets;
  out->failed_targets = result.failed_targets;
  return true;
}

void param_bridge_result_ack(void) {
  g_transactions.ack();
}

uint32_t param_bridge_request_high_water(void) {
  return g_transactions.request_high_water();
}
