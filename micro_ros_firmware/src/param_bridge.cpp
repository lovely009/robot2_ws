// 参数与系统命令桥实现（**话题面**，取代 2026-09-21 退役的 rclc 参数服务器）。
//
// 三部分：
//  1) 订阅回调（executor 上下文）：整条校验 + 提交 mailbox。设备写是异步事务，
//     回调返回只表示「已受理」；最终成败经内核 CONFIG_RESULT 事件异步给出；
//  2) on_config_result 钩子（motor_bridge 事件循环内同任务调用）：收敛三条系统
//     命令的完成序号，并把其余失败报进 last_error；
//  3) poll（10ms 桥定时器）：发布 system_status、分批下发保存、上报设备参数结论。
//
// 线程边界：回调与 poll 都在 micro_ros 任务（executor/spin 与定时器同线程），
// 彼此不并发；影响电机只经 motor_rt_submit_config，影响 Flash 只经 storage
// 任务通知，本文件不直接发 FDCAN、不擦写 Flash（ARD §28.1）。

#include "param_bridge.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <can_bridge_msgs/msg/param_set_array.h>
#include <can_bridge_msgs/msg/system_command.h>
#include <can_bridge_msgs/msg/system_status.h>
#include <rcl/error_handling.h>
#include <rmw_microros/time_sync.h>

#include "motor_rt_task.h"
#include "param_name.h"
#include "param_units.h"
#include "ros_qos.h"
#include "storage_task.h"

static const char *kParamSetTopic = "micro_ros_do/param_set";
static const char *kSystemCommandTopic = "micro_ros_do/system_command";
static const char *kSystemStatusTopic = "micro_ros_up/system_status";

// 错误码（system_status.last_error）：0=无；1..6 对齐 CONFIG 结果七码（reason
// 1..6）；10=本地校验拒绝；11=mailbox 提交失败（FIFO 满/断链）；12=未绑定/未知
// 电机；20=save 失败；30=refresh 失败；40=设备参数与固件记录不一致（未初始化，
// 该台不可运动）；41=设备参数回读不成功（未核对，同样不可运动）；42=本机对该台
// **无记录**（首次上电/新板，同样不可运动，须人工核对后写参数）。
// 40/41/42 由内核结论驱动（dev_init），与请求回告无关：固件那份记录说了算。
// 码表逐字沿用参数面时期，操作员侧零迁移。
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

// 线协议 field 上界 = .msg 的 ParamSetTarget[<=16]。直接引生成的常量，不手抄：
// 谁把 .msg 的上界改大，下面的 static_assert 立刻编译失败。
#define kParamSetMaxTargets \
  can_bridge_msgs__msg__ParamSetArray__targets__MAX_SIZE

// 每个目标（每对位置限位算一次）一次 motor_rt_submit_config，而配置 FIFO 只有
// 这么深。上界必须 <= 它，否则一条消息自己就能把队列打满——被拒的台设备侧没落
// 盘、W25 却落了整份视图 ⇒ 下次上电是"记录新、设备旧"的假不一致。
static_assert(kParamSetMaxTargets <= CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY,
              "ParamSetArray 上界超过配置 FIFO 深度：一条消息能自己把队列打满");

// executor 句柄数：两条订阅（发布不占 handle）。少算了会在 add_subscription 上
// 失败，且只在运行期以 WAIT_AGENT 无限重连的形式暴露。
static_assert(PARAM_BRIDGE_EXECUTOR_HANDLES == 2U,
              "PARAM_BRIDGE_EXECUTOR_HANDLES 必须等于本文件的订阅数");

// ---- 线协议 field 表 ----
//
// 下标 = ParamSetTarget.field = param_device_slot_t
// （0=pos_min 1=pos_max 2=vel_limit 3=torque_limit 4=limit_protect 5=direction）。
//
// ⚠️ 右列 param_field **不是**左列 wire field 的镜像：两套编号数值上不同
// （wire 3 = 力矩，param_field_t 3 = 位置下限），互用不会被任何校验拒绝，只会
// **静默写错槽位**。所以映射必须逐行写出来，并由下面的断言钉住长度。
//
// 表与枚举分离是 2026-09-19 那次事故留下的规矩：当时有人在 param_field_t 尾部
// 追加 6 个 PID 字段，注册循环以枚举边界为上界，把它们自动带进了参数树（恒
// NaN、纯占槽、第 37 台起静默失败）。教训是"枚举全集 ≠ 可操作集合"，加字段
// 必须显式改这张表。事故完整记录见 decision-log。
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

// 固件自发请求的 id 命名空间（bit30）。调用方给的 request_id 逐字进内核，只有
// 落在这一位上的会被本地拒绝——bit31 已被内核身份读取的伪 id 占用。
#define PARAM_INTERNAL_ID_TAG 0x40000000U

// ---- 状态 ----
static rcl_subscription_t g_param_set_sub;
static rcl_subscription_t g_system_cmd_sub;
static rcl_publisher_t g_system_status_pub;
static can_bridge_msgs__msg__ParamSetArray g_param_set_msg;
static can_bridge_msgs__msg__ParamSetTarget g_param_set_buf[kParamSetMaxTargets];
static can_bridge_msgs__msg__SystemCommand g_system_cmd_msg;
static can_bridge_msgs__msg__SystemStatus g_system_status_msg;
static bool g_sub_up = false;
static bool g_cmd_sub_up = false;
static bool g_pub_up = false;
static bool g_sub_added = false;
static bool g_cmd_sub_added = false;
static bool g_created = false;

// 最近一次 poll 的参数视图快照。param_bridge_views() 把它交给 device_param_report
// （motor_bridge），是**唯一不得变的外部契约**。
static motor_control_param_view_t g_views[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_view_count = 0;

static uint16_t g_last_error = ERR_NONE;
static uint16_t g_last_error_motor = kMotorIdNone;
static bool g_status_dirty = true; // 有变化 → 下一拍立即发一帧（不用等心跳）
static uint32_t g_status_publish_fail = 0U;
static uint32_t g_last_status_ms = 0U;
// 5 Hz（不是 1 Hz）：1 Hz 会让"发命令 → 轮询 seq"每次多等最多 1 s，而 refresh
// 本身只要 ~30 ms，完成信号反成瓶颈。last_error 是**电平**不是一次性事件，
// 丢一帧只损失 200 ms。
#define kStatusPeriodMs 200U

static uint32_t g_next_request_id = 1U; // 跨会话不复位（防重连后旧事件串台）

// 三条系统命令的完成序号与在飞 id。序号是只读、单调递增的**唯一可靠完成判据**：
// 只看 last_error 不行——上一轮成功留下的 (0, motor_id) 与本轮无法区分，页面会
// 把"还没读到"当成功。
static int64_t g_refresh_seq = 0;
static uint32_t g_refresh_id = 0;
static int64_t g_set_zero_seq = 0;
static uint32_t g_set_zero_id = 0;
static int64_t g_clear_fault_seq = 0;
static uint32_t g_clear_fault_id = 0;

// 保存脉冲的进度游标：一次保存可能要下发几十台，而配置 FIFO 只有 16 深、消费侧
// 每 5ms 边界只抽 4 条。一次 poll 里全推出去必然溢出，而被拒的台设备侧没落盘、
// W25 却已经无条件落了盘（storage_task_request_save 是整份视图快照）⇒ 下次上电
// 就是"记录新、设备旧"的假不一致，操作者得逐台重核。故按拍分批、全部下发完才
// 落 W25。
// **不能拿脏位当游标**：被运动禁写门拒掉的台（ACTIVE/SAFE_STOP）脏位永远不清，
// 用"还有脏台吗"判"扫完了吗"会永远不收敛，W25 就永远不落。
// 游标下标对 g_views 稳定：内核表项只增不减，copy_param_views 按注册顺序输出。
static uint16_t g_save_cursor = 0U;
static bool g_save_requested = false;     // 收到 SAVE，尚未起航
static bool g_save_active = false;        // 设备侧分批下发中
static bool g_save_flash_pending = false; // 已请 W25 落盘，等终态
static uint32_t g_save_phase_ms = 0U;     // 等落盘阶段的起点（看门狗用）
static int64_t g_save_seq = 0;
// 每拍最多下发几台。必须**低于**消费侧的抽取速率（每 5ms 边界 4 条 = 每 10ms
// 边界 8 条），否则队列仍会累积。
#define kSaveSubmitsPerPoll 4U
// W25 落盘看门狗：v3 要擦 2 个扇区（量级 ~1s/扇区），但存储任务卡死时
// save_in_progress 会永久为真——那是**电平**，卡住就没有第二次机会了。与
// 48-motor-scale §5.3 点名的 outstanding 永久残留同类，故必须兜底。
#define kSaveTimeoutMs 5000U

// 已上报的设备参数结论（下标 = motor_id；0xFF = 从未上报）。
// 用电机 id 做 key 而不是视图下标：视图顺序不是契约，内核表项只增不减。
static uint8_t g_reported_dev_init[MOTOR_CONTROL_MAX_MOTORS];

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
  g_status_dirty = true; // 回调内不发布，poll 里统一发
}

// 代分配一个内部 id（调用方给 0 时）。恒带 PARAM_INTERNAL_ID_TAG，与调用方
// 逐字透传的 id 天然不撞（那种 id 的 bit30 已被本地拒绝）。
static uint32_t next_internal_id() {
  g_next_request_id++;
  if (g_next_request_id == 0U) {
    g_next_request_id = 1U;
  }
  return PARAM_INTERNAL_ID_TAG | (g_next_request_id & 0x3FFFFFFFU);
}

// ---- 提交 ----

// 单个限值（速度/力矩）。
// 返回 false = 提交失败（FIFO 满/断链）。**值域校验不在这里**：它必须在整条
// 预校验阶段过完，否则中途失败会留下"部分生效"。
static bool submit_single_limit(const motor_control_param_view_t *v,
                                const param_field_desc_t *desc, float value,
                                uint32_t request_id) {
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = v->motor_id;
  request.kind = desc->kind;
  // 单位口径见 param_units.h：速度对外就是 rad/s（内核同单位，直通），力矩对外
  // 是 A（设备对象 0x2538 存的就是电流）。换算只在这一处做。
  if (desc->param_field == PARAM_FIELD_VEL_LIMIT) {
    request.value.max_velocity_rad_s = value;
  } else {
    request.value.max_torque_nm = param_units_amp_to_nm(value, v->torque_per_amp);
  }
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, v->motor_id);
    return false;
  }
  return true;
}

// 位置双限提交。成对值由调用方算好——不成对的半边在预校验阶段就被整条拒了，
// 这里没有暂存/补齐逻辑（参数面时期的 staged_pos 是 rclc 一次只能 set 一个标量
// 的产物，一条消息能带两边，它没有存在理由）。
static bool submit_position_limits(const motor_control_param_view_t *v, float lo,
                                   float hi, uint32_t request_id) {
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = v->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
  // 位置对外就是 rad（内核同单位，直通、不折算角度）。
  request.value.position_limits.min_position_rad = lo;
  request.value.position_limits.max_position_rad = hi;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, v->motor_id);
    return false;
  }
  return true;
}

// 保护开关提交。**唯一反转点**：对外 1 = 保护生效 ↔ 厂家原值 0 = 保护开。
// 反转只在这一处发生，protocol 层与 mailbox 只认原值（control_mailbox.h 的
// SET_SAFETY_PROTECT 注释即此约定）。别为了"对称"在别处再加一层。
static bool submit_safety_protect(const motor_control_param_view_t *v,
                                  float value, uint32_t request_id) {
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = v->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT;
  request.value.safety_protect_raw = (value == 1.0f) ? 0U : 1U;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, v->motor_id);
    return false;
  }
  return true;
}

// 方向提交：**原值直通，不做语义反转**——0x2541 没有 0x2650 那种"1 = 关"的
// 反向陷阱，别为了与保护开关对称而多加一层。
static bool submit_direction(const motor_control_param_view_t *v, float value,
                             uint32_t request_id) {
  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = v->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_DIRECTION;
  request.value.direction_raw = (value == 1.0f) ? 1U : 0U;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, v->motor_id);
    return false;
  }
  return true;
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
      return false; // POS_MIN/POS_MAX 走成对路径；越界 field 已被预校验拒掉
  }
}

// ---- 整条校验 ----
//
// 任一不成立 → false（last_error 已置），调用方直接返回、一个字节都不下发。
// **纯函数式**：不碰 mailbox、不发布，所以"整条受理或整条拒绝"是能兑现的
// 承诺而不是尽力而为——凡是能在这里判定的失败，都不会在分发阶段才冒出来。
static bool validate_param_set(const can_bridge_msgs__msg__ParamSetArray *msg) {
  // count 是 targets.size 的冗余声明（口径同 MotorPolicyArray.count）：对不上
  // 就说明发送方对契约的理解与固件不同，宁可整条拒。
  if (msg->count != msg->targets.size || msg->count > kParamSetMaxTargets) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    return false;
  }
  for (uint16_t i = 0; i < msg->count; ++i) {
    const can_bridge_msgs__msg__ParamSetTarget &t = msg->targets.data[i];
    if (t.field >= PARAM_DEVICE_SLOT_COUNT ||
        t.motor_id >= MOTOR_CONTROL_MAX_MOTORS) {
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
    // 力矩列要过一次 A→N·m，系数未知（未绑定型号）时不能拿瞎猜的系数下发。
    // 放在预校验而不是提交时：提交中途失败会留下部分生效，破坏整条语义。
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_TORQUE_LIMIT &&
        !isfinite(param_units_amp_to_nm(t.value, v->torque_per_amp))) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    // 同一 (motor_id, field) 出现两次：静默取一个会掩盖调用方 bug。
    for (uint16_t j = 0; j < i; ++j) {
      if (msg->targets.data[j].motor_id == t.motor_id &&
          msg->targets.data[j].field == t.field) {
        report_error(ERR_LOCAL_VALIDATION, t.motor_id);
        return false;
      }
    }
  }
  // 位置限位成对：同一条消息内该台必须两边都给，且 min < max。
  const uint8_t pos_bits = (uint8_t)((1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) |
                                     (1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX));
  for (uint16_t i = 0; i < msg->count; ++i) {
    const uint16_t id = msg->targets.data[i].motor_id;
    uint8_t mask = 0U;
    float lo = NAN;
    float hi = NAN;
    for (uint16_t j = 0; j < msg->count; ++j) {
      const can_bridge_msgs__msg__ParamSetTarget &p = msg->targets.data[j];
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
      // 只给一边：不做半边暂存。另一边从 device_param_report 的 rec_pos_* 读回
      // 一起发——半边状态正是本设计要消掉的东西。
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

// 逐目标分发。此刻预校验已全部通过，除提交失败（FIFO 满/断链）外不会再失败。
static void apply_param_set(const can_bridge_msgs__msg__ParamSetArray *msg,
                            uint32_t request_id) {
  bool consumed[kParamSetMaxTargets] = {};
  for (uint16_t i = 0; i < msg->count; ++i) {
    if (consumed[i]) {
      continue;
    }
    const can_bridge_msgs__msg__ParamSetTarget &t = msg->targets.data[i];
    const motor_control_param_view_t *v = find_view(t.motor_id);
    if (v == nullptr) {
      continue; // 预校验已确认存在；防御
    }
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN ||
        t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
      // 成对提交：一趟把所有位置元素找齐并标记消费（**不是静默丢弃**——它们
      // 就在这次成对提交里一起下发）。预校验已保证恰好两个且 lo < hi。
      float lo = NAN;
      float hi = NAN;
      for (uint16_t j = 0; j < msg->count; ++j) {
        const can_bridge_msgs__msg__ParamSetTarget &p = msg->targets.data[j];
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
        return; // last_error 已置；继续下发只会扩大不一致
      }
      continue;
    }
    consumed[i] = true;
    if (!dispatch_target(v, t.field, t.value, request_id)) {
      return;
    }
  }
}

static void param_set_array_cb(const void *msgin) {
  const can_bridge_msgs__msg__ParamSetArray *msg =
      (const can_bridge_msgs__msg__ParamSetArray *)msgin;
  if (msg == nullptr) {
    return;
  }
  if (!validate_param_set(msg)) {
    return;
  }

  // request_id：0 = 固件代分配（配置 FIFO 拒 0）；落在固件自发命名空间 → 整条
  // 拒绝；其余**逐字**进内核并随 CONFIG_RESULT 原样回告（先例
  // motor_bridge.cpp 的 snapshot.request_id 直通）。不做去重——全仓库无去重
  // 先例，且做它要重新引入本次正要删掉的状态。
  uint32_t request_id = msg->request_id;
  if ((request_id & PARAM_INTERNAL_ID_TAG) != 0U) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    return;
  }
  if (request_id == 0U) {
    request_id = next_internal_id();
  }
  apply_param_set(msg, request_id);
}

// 系统命令：refresh / set_zero / clear_fault 直接提交（与 motor_bridge 的三个
// 回调同构——校验与提交都在回调里做）；save 只置请求，实际的设备侧分批下发与
// W25 落盘摊到 poll 里（见 g_save_cursor 的注释）。
static void system_command_cb(const void *msgin) {
  const can_bridge_msgs__msg__SystemCommand *msg =
      (const can_bridge_msgs__msg__SystemCommand *)msgin;
  if (msg == nullptr) {
    return;
  }

  if (msg->command == can_bridge_msgs__msg__SystemCommand__COMMAND_SAVE) {
    // 只接受整机保存：W25 那一半必须整体落盘，按台保存只能省设备侧一半，语义
    // 含混（见 .msg）。给了具体 motor_id 就是调用方误解了契约。
    if (msg->motor_id != can_bridge_msgs__msg__SystemCommand__MOTOR_ID_NONE) {
      report_error(ERR_LOCAL_VALIDATION, msg->motor_id);
      return;
    }
    g_save_requested = true; // 进行中的重复请求合并（保存是幂等的整体动作）
    return;
  }

  control_mailbox_config_kind_t kind;
  switch (msg->command) {
    case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH:
      kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
      break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO:
      kind = CONTROL_MAILBOX_CONFIG_SET_ZERO;
      break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT:
      kind = CONTROL_MAILBOX_CONFIG_CLEAR_FAULT;
      break;
    default:
      report_error(ERR_LOCAL_VALIDATION, msg->motor_id);
      return;
  }

  const motor_control_param_view_t *v = (msg->motor_id < MOTOR_CONTROL_MAX_MOTORS)
                                            ? find_view(msg->motor_id)
                                            : nullptr;
  // 未绑定型号的台没有设备事务可做（内核会回 UNSUPPORTED），在本地就拦掉，
  // 省一次往返并把错误码归到"这台不能用"这一类。
  if (v == nullptr || !v->bound) {
    report_error(ERR_UNKNOWN_MOTOR, msg->motor_id);
    return;
  }

  uint32_t request_id = msg->request_id;
  if ((request_id & PARAM_INTERNAL_ID_TAG) != 0U) {
    report_error(ERR_LOCAL_VALIDATION, msg->motor_id);
    return;
  }
  if (request_id == 0U) {
    request_id = next_internal_id();
  }

  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = msg->motor_id;
  request.kind = kind;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, msg->motor_id);
    return;
  }
  // 同一命令重复下发：**后来的取代先前的**（旧的在飞事件到达时 id 已不匹配，
  // 自然被忽略）。三条命令各自只有一份在飞状态，这是参数面时期的既有语义。
  switch (msg->command) {
    case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH:
      g_refresh_id = request_id;
      break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO:
      g_set_zero_id = request_id;
      break;
    case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT:
      g_clear_fault_id = request_id;
      break;
    default:
      break;
  }
}

// ---- 内核事件回告 ----

void param_bridge_on_config_result(const motor_control_event_t *event) {
  if (!g_created || event == nullptr ||
      event->type != MOTOR_CONTROL_EVENT_CONFIG_RESULT) {
    return;
  }
  const uint32_t id = event->request_id;
  const bool ok = event->reason == MOTOR_PROTOCOL_CONFIG_OK;

  // 判据是**逐字等于**在飞 id 之一，不是"带 bit30 就算我的"——后者会把调用方
  // 恰好在 bit30 置位的 request_id 误认成自己的命令（现存的反向隐患）。
  if (id != 0U && id == g_refresh_id) {
    g_refresh_id = 0U;
    if (!ok) {
      report_error(ERR_REFRESH_FAIL, event->motor_id);
    }
    // 收尾序号：上位机据此确认"这次回读已经结束"，再去读 last_error 与
    // device_param_report（0=一致 / 40=与记录不一致 / 41=回读不成功）。
    g_refresh_seq++;
    g_status_dirty = true;
    return;
  }
  if (id != 0U && id == g_set_zero_id) {
    g_set_zero_id = 0U;
    if (!ok) {
      report_error((uint16_t)event->reason, event->motor_id);
    }
    g_set_zero_seq++;
    g_status_dirty = true;
    return;
  }
  if (id != 0U && id == g_clear_fault_id) {
    g_clear_fault_id = 0U;
    if (!ok) {
      report_error((uint16_t)event->reason, event->motor_id);
    }
    g_clear_fault_seq++;
    g_status_dirty = true;
    return;
  }

  // 其余 CONFIG_RESULT：参数写、策略、轨迹等的回告。成功无需处理——没有影子
  // 要同步，视图会在下一拍 poll 里重新拍；失败报进 last_error。**不区分来源**：
  // 参数面时期 policy 路径的失败操作员在参数页上看不到，这是改进。若台架发现
  // 高频重发导致刷屏，退路是加 16 项 in-flight 表恢复"只报自己发的"。
  if (!ok) {
    report_error((uint16_t)event->reason, event->motor_id);
  }
}

// ---- 发布 ----

static void publish_system_status() {
  g_system_status_msg.sync_time_ns = rmw_uros_epoch_nanos();
  g_system_status_msg.last_error = g_last_error;
  g_system_status_msg.last_error_motor = g_last_error_motor;
  g_system_status_msg.refresh_seq = (uint32_t)g_refresh_seq;
  g_system_status_msg.set_zero_seq = (uint32_t)g_set_zero_seq;
  g_system_status_msg.clear_fault_seq = (uint32_t)g_clear_fault_seq;
  g_system_status_msg.save_seq = (uint32_t)g_save_seq;
  g_system_status_msg.save_in_progress =
      g_save_active || g_save_flash_pending;
  if (rcl_publish(&g_system_status_pub, &g_system_status_msg, nullptr) !=
      RCL_RET_OK) {
    // best-effort 超传输上限时 rcl_publish 静默丢；计数由 motor_bridge 合并进
    // MotorDiagnostics.publish_fail（§3.0 的教训：失败必须可见，不要 (void)）。
    ++g_status_publish_fail;
  }
}

// ---- 保存三段状态机 ----
//
// **save_in_progress 覆盖全程**：
//   ① g_save_requested     → 起航（游标归零）
//   ② g_save_active        → 设备侧分批下发 0x2539
//   ③ g_save_flash_pending → W25 落盘，等 storage 终态
// 旧实现把 ② 扫完就直接报"保存完成"（清布尔与落盘请求同一拍），而 v3 的 W25 要
// 擦 2 个扇区（量级 ~1s/扇区）——"完成"比真正落盘早了整整一段，此时断电就是真
// 丢数据。现在必须 ③ 出终态才算完。
static void poll_save(uint32_t now) {
  if (g_save_requested && !g_save_active && !g_save_flash_pending) {
    g_save_requested = false;
    g_save_cursor = 0U;
    g_save_active = true;
    g_status_dirty = true;
  }
  if (g_save_active) {
    uint32_t submitted = 0U;
    while (g_save_cursor < g_view_count && submitted < kSaveSubmitsPerPoll) {
      const motor_control_param_view_t *view = &g_views[g_save_cursor++];
      // 只处理脏的台：没写过的台一个字节都不发（驱动器写 flash 期间会阻塞自身
      // 控制循环，白存一次就是白等一截控制环），也不占本拍配额。
      // 只脏易失槽（保护开关/方向）的台由内核在保存事务成功回 OK 时清脏位——
      // 那种台只欠 W25 那一半，扫完一轮落盘就是它的全部。
      if (view->record_dirty_slots == 0U) {
        continue;
      }
      control_mailbox_config_request_t request = {};
      request.request_id = next_internal_id();
      request.received_tick_ms = millis();
      request.motor_id = view->motor_id;
      request.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;
      request.value.save_slots = view->record_dirty_slots;
      if (!motor_rt_submit_config(&request)) {
        // 保存是幂等的整体动作：这一台漏了，重复发一次 SAVE 即可补上，不中断
        // 整轮（中断会让后面的台永远轮不到）。
        report_error(ERR_SUBMIT_FAILED, view->motor_id);
      }
      submitted++;
    }
    if (g_save_cursor >= g_view_count) {
      // 设备侧扫完 ≠ 保存完成：现在才请 W25 落盘，并把 save_in_progress 交给
      // 第三段。**不发 save_seq、不清 in_progress**。
      g_save_active = false;
      g_save_cursor = 0U;
      g_save_flash_pending = true;
      g_save_phase_ms = now;
      storage_task_request_save();
    }
    return;
  }
  if (!g_save_flash_pending) {
    return;
  }
  // 关于 W25 结果：save 失败自有其报告路径（下面这条），不回退设备侧脏位——
  // 脏位是内核在保存事务成功回 OK 时清的（只脏易失槽的台受理即清），到这一步
  // 已经没了。"未保存"标记消失后，last_error 是唯一剩下的信号；不要试图从这里
  // 重新推导或重设脏位（内核那份记录说了算）。
  const storage_save_status_t result = storage_take_result();
  if (result == STORAGE_SAVE_IDLE) {
    if ((uint32_t)(now - g_save_phase_ms) < kSaveTimeoutMs) {
      return; // 还在落盘
    }
    // 看门狗：存储任务卡死（或从未启动）时不能永远停在"保存中"。
    report_error(ERR_SAVE_FAIL, kMotorIdNone);
  } else if (result == STORAGE_SAVE_FAIL) {
    report_error(ERR_SAVE_FAIL, kMotorIdNone);
  }
  g_save_flash_pending = false;
  // 无论成败都推进序号：这是"保存动作收尾"的计数不是成功数。失败也 +1 是刻意
  // 的——否则等序号的人会一直等下去（口径同 refresh_seq）。
  g_save_seq++;
  g_status_dirty = true;
}

// 设备参数结论（未初始化）：按"变化才报"推 last_error。结论由内核上电回读比对
// 产生，不随任何请求回告走——固件那份记录说了算。
static void report_dev_init_changes(void) {
  for (size_t i = 0; i < g_view_count; ++i) {
    const motor_control_param_view_t *v = &g_views[i];
    if (v->motor_id >= MOTOR_CONTROL_MAX_MOTORS ||
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
        // 结论转好也上报（last_error=0 + motor_id）：上位机据此确认"这次回读
        // 成功"，否则一个陈旧的 40 会永远挂在那里，分不清新旧。
        report_error(ERR_NONE, v->motor_id);
        break;
      default:
        break; // PENDING 不是结论，不报
    }
  }
}

void param_bridge_poll(void) {
  if (!g_created) {
    return;
  }
  const uint32_t now = millis();
  g_view_count = motor_rt_copy_param_views(g_views, MOTOR_CONTROL_MAX_MOTORS);

  poll_save(now);
  report_dev_init_changes();

  // 5 Hz 心跳 + 变化即发：等序号的调用方不必等满一个周期。
  if (g_status_dirty || (uint32_t)(now - g_last_status_ms) >= kStatusPeriodMs) {
    g_last_status_ms = now;
    g_status_dirty = false;
    publish_system_status();
  }
}

uint32_t param_bridge_publish_fail_count(void) { return g_status_publish_fail; }

const motor_control_param_view_t *param_bridge_views(size_t *out_count) {
  if (out_count != nullptr) {
    *out_count = g_view_count;
  }
  return g_views;
}

// ---- 生命周期 ----

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) {
    return true; // 幂等
  }
  if (node == nullptr || executor == nullptr) {
    return false;
  }

  // 序列静态存储：rosidl C 序列是 data/size/capacity 三元组，反序列化会写进来，
  // 必须在 init 之前接好（漏了就是往 NULL 写）。rclc/std 都不分配。
  g_param_set_msg.targets.capacity =
      sizeof(g_param_set_buf) / sizeof(g_param_set_buf[0]);
  g_param_set_msg.targets.data = g_param_set_buf;
  if (g_param_set_msg.targets.capacity < kParamSetMaxTargets) {
    return false;
  }

  memset(g_reported_dev_init, 0xFF, sizeof(g_reported_dev_init));
  g_view_count = 0;

  // 上行：best-effort。29 B 的**电平**状态，丢一帧只损失 200 ms（与 diagnostics
  // / trajectory_state 同判据）。它远小于 512 B MTU，不需要 reliable 分片。
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_system_status_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     SystemStatus),
                         kSystemStatusTopic, &pub_opts) != RCL_RET_OK) {
    return false;
  }
  g_pub_up = true;

  // 下行两条：reliable。低频**状态声明**（配置/命令），不允许静默丢——与
  // MotorPolicyArray 同一判据。两条都 < 512 B MTU（ParamSetArray 最大
  // 16×8+14 ≈ 142 B），不分片。
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_param_set_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        ParamSetArray),
                            kParamSetTopic, &sub_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_up = true;

  if (rcl_subscription_init(&g_system_cmd_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        SystemCommand),
                            kSystemCommandTopic, &sub_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_cmd_sub_up = true;

  if (rclc_executor_add_subscription(executor, &g_param_set_sub,
                                     &g_param_set_msg, param_set_array_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_added = true;

  if (rclc_executor_add_subscription(executor, &g_system_cmd_sub,
                                     &g_system_cmd_msg, system_command_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_cmd_sub_added = true;

  g_created = true;
  g_status_dirty = true; // 建桥后立刻发第一帧（页面不必等满一个心跳周期）
  return true;
}

void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (!g_created && !g_sub_up && !g_cmd_sub_up && !g_pub_up) {
    return;
  }
  g_created = false;
  g_view_count = 0;
  memset(g_reported_dev_init, 0xFF, sizeof(g_reported_dev_init));
  // 进行中的保存不能在重建后带着旧游标续跑：g_views 会重填，游标下标已无意义
  // （会跳过靠前的脏台、并触发一次没被要求的 W25 落盘）。
  g_save_requested = false;
  g_save_active = false;
  g_save_flash_pending = false;
  g_save_cursor = 0U;
  g_refresh_id = 0U;
  g_set_zero_id = 0U;
  g_clear_fault_id = 0U;

  // **订阅必须先自己移除再 fini**：rclc 的 executor 不感知 fini，残留句柄会在
  // 重连后顶掉后面话题的 init（表现为第 6 条订阅失败 → WAIT_AGENT 无限重连，
  // 编译期零提示）。发布不占 handle，直接 fini。
  if (executor != nullptr) {
    if (g_sub_added) {
      (void)rclc_executor_remove_subscription(executor, &g_param_set_sub);
    }
    if (g_cmd_sub_added) {
      (void)rclc_executor_remove_subscription(executor, &g_system_cmd_sub);
    }
  }
  g_sub_added = false;
  g_cmd_sub_added = false;
  // rcl_*_fini 带 warn_unused_result，(void) 消不掉——必须真接住返回值；
  // 这里只关心"清干净了"，失败也没有补救动作。
  rcl_ret_t rc = RCL_RET_OK;
  if (g_sub_up) {
    rc = rcl_subscription_fini(&g_param_set_sub, node);
    g_sub_up = false;
  }
  if (g_cmd_sub_up) {
    rc = rcl_subscription_fini(&g_system_cmd_sub, node);
    g_cmd_sub_up = false;
  }
  if (g_pub_up) {
    rc = rcl_publisher_fini(&g_system_status_pub, node);
    g_pub_up = false;
  }
  (void)rc;
}
