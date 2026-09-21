// 电机域 ROS 桥实现：ID 化消息面（ARD-0007 §15）。
//
// 下行 3 条（订阅）：motor_command_array（100Hz 完整快照）、motor_trajectory（多点
// 轨迹）、motor_policy（逐电机 enable/kp/kd）。
// 上行 7 条（发布）：motion_states 100Hz、motor_status_array 20Hz、motor_topology 1s、
// motor_events、trajectory_state 10Hz、diagnostics 1Hz、device_param_report 1Hz + 变化。
// 全部用相对名，见下。
//
// 轨迹与快照的仲裁在内核：轨迹声明集合内的电机由轨迹独占（快照对它不下发也不软停），
// 集合外仍归快照（缺失即软停，ARD §13.1）。策略与二者正交（ARD §15.4 禁止 kp/kd/enable
// 进 100Hz 快照）。型号绑定由内核经 READ_IDENTITY 事务完成，本桥不经手下行配置。
//
// 本桥不做：joint_name 语义、协议/CAN 字节、直接 RX 访问（ARD §28.1）。

#include "motor_bridge.h"

#include <Arduino.h>

#include <string.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <can_bridge_msgs/msg/device_param_report.h>
#include <can_bridge_msgs/msg/motor_command_array.h>
#include <can_bridge_msgs/msg/motor_event.h>
#include <can_bridge_msgs/msg/motor_diagnostics.h>
#include <can_bridge_msgs/msg/motor_motion_state_array.h>
#include <can_bridge_msgs/msg/motor_policy_array.h>
#include <can_bridge_msgs/msg/motor_status_array.h>
#include <can_bridge_msgs/msg/motor_topology.h>
#include <can_bridge_msgs/msg/motor_trajectory.h>
#include <can_bridge_msgs/msg/motor_trajectory_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/time_sync.h>

#include "motor_rt_task.h"
#include "param_bridge.h"
#include "param_units.h"
#include "ros_qos.h"

// 序列静态存储（rosidl C 序列为 data/size/capacity 三元组；容量预分配，
// 禁止反序列化路径上的动态分配）。
// 各 ROS 数组容量对齐内核注册表（跨库一致性由 motor_rt_task.cpp
// 顶部 static_assert 锁定；勿在本文件写裸 64）。
static constexpr size_t kRosArrayCapacity = MOTOR_CONTROL_MAX_MOTORS;
static can_bridge_msgs__msg__MotorMotionStateTarget
    g_motion_states_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorStatusTarget g_status_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorTopologyEntry g_topology_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorCommandTarget g_command_targets_buf[kRosArrayCapacity];
// 轨迹两层有界序列各自预分配（点数容量独立于电机数，见 .msg 契约）。
static constexpr size_t kRosTrajectoryPointCapacity =
    MOTOR_CONTROL_MAX_TRAJECTORY_POINTS;
static can_bridge_msgs__msg__MotorTrajectoryPoint
    g_trajectory_points_buf[kRosTrajectoryPointCapacity];
static uint16_t g_trajectory_target_ids_buf[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
static can_bridge_msgs__msg__MotorPolicyTarget g_policy_targets_buf[kRosArrayCapacity];

// ---- 话题名 ----
// 统一用**相对名**：整条固件接口可被 launch 重映射，且"哪些是固件私有接口"一眼可辨
// ——根命名空间留给标准话题（/joint_states 等，由上层适配桥产出）。
// 改名必须同步消费者：web_host/web/app.js 的 TOPICS。
static const char *kCommandTopic = "micro_ros_do/motor_command_array";
static const char *kTrajectoryTopic = "micro_ros_do/motor_trajectory";
static const char *kPolicyTopic = "micro_ros_do/motor_policy";
static const char *kMotionStatesTopic = "micro_ros_up/motion_states";
static const char *kStatusTopic = "micro_ros_up/motor_status_array";
static const char *kTopologyTopic = "micro_ros_up/motor_topology";
static const char *kEventsTopic = "micro_ros_up/motor_events";
static const char *kDiagnosticsTopic = "micro_ros_up/diagnostics";
static const char *kTrajectoryStateTopic = "micro_ros_up/trajectory_state";
static const char *kDeviceParamReportTopic = "micro_ros_up/device_param_report";

// 消息面契约版本：随字段**语义**变更递增（字段只增不改序，故不是每次加字段
// 都动）。上层据此检测漂移，而不是静默按旧布局解析。
// 2 = 上行按「拓扑=身份 / 状态=状态与电气 / 运动=运动学」三分去重：
//     lifecycle 只留 status、state_age_ms 只留 motion、topology 加版本号。
static constexpr uint16_t kMotorMsgSchemaVersion = 2U;

// ---- 订阅（2）----
static rcl_subscription_t g_command_sub;
static can_bridge_msgs__msg__MotorCommandArray g_command_msg;
static bool g_command_up = false;
static bool g_command_added = false;

static rcl_subscription_t g_trajectory_sub;
static can_bridge_msgs__msg__MotorTrajectory g_trajectory_msg;
static bool g_trajectory_up = false;
static bool g_trajectory_added = false;

static rcl_subscription_t g_policy_sub;
static can_bridge_msgs__msg__MotorPolicyArray g_policy_msg;
static bool g_policy_up = false;
static bool g_policy_added = false;

// ---- 发布（7）----
static rcl_publisher_t g_motion_states_pub;
static can_bridge_msgs__msg__MotorMotionStateArray g_motion_states_msg;
static bool g_motion_states_up = false;

static rcl_publisher_t g_status_pub;
static can_bridge_msgs__msg__MotorStatusArray g_status_msg;
static bool g_status_up = false;

static rcl_publisher_t g_topology_pub;
static can_bridge_msgs__msg__MotorTopology g_topology_msg;
static bool g_topology_up = false;

static rcl_publisher_t g_events_pub;
static can_bridge_msgs__msg__MotorEvent g_event_msg;
static bool g_events_up = false;

static rcl_publisher_t g_diagnostics_pub;
static can_bridge_msgs__msg__MotorDiagnostics g_diagnostics_msg;
// 诊断消息里三个 [<=3] 序列各有自己的静态缓冲，**必须逐条接线**：漏接的序列
// .data 为 NULL，写入会落到地址 0（STM32H7 的 ITCM 在那儿，写不报错），两个漏接
// 的序列于是互相覆盖——rx_hw_lost 曾整整一版都在报 tx 的数。
static uint32_t g_diagnostics_ring_buf[MOTOR_CONTROL_MAX_BUSES];
static uint32_t g_diagnostics_rx_lost_buf[MOTOR_CONTROL_MAX_BUSES];
static uint32_t g_diagnostics_tx_count_buf[MOTOR_CONTROL_MAX_BUSES];
static bool g_diagnostics_up = false;

static rcl_publisher_t g_trajectory_state_pub;
static can_bridge_msgs__msg__MotorTrajectoryState g_trajectory_state_msg;
static uint16_t g_trajectory_state_ids_buf[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
static bool g_trajectory_state_up = false;

static rcl_publisher_t g_device_param_pub;
static can_bridge_msgs__msg__DeviceParamReport g_device_param_msg;
// 每台 12 个 float + 若干小字段 ≈ 60 B；48 台约 3 KB。**静态而非栈上**：
// 发布在 micro_ros 任务栈上跑，栈放不下。
static can_bridge_msgs__msg__DeviceParamEntry
    g_device_param_entries[MOTOR_CONTROL_MAX_MOTORS];
static bool g_device_param_up = false;
// 事件触发的"马上发一帧"标志（见 publish_events）。
static bool g_device_param_pending = false;

size_t motor_bridge_executor_handle_count(void) {
  // command + trajectory + policy 三个订阅（下行配置话题已删；型号绑定在
  // 内核内自动完成）。加订阅必须同步本计数，否则 executor 句柄不足会静默
  // 丢回调。
  return 3U;
}

// ---------------------------------------------------------------------------
// 回调：消息校验 → mailbox 写入
// ---------------------------------------------------------------------------

static void command_array_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorCommandArray *)msgin;
  if (msg->count > msg->targets.size) {
    return; // 有界数组外的声明：拒绝
  }
  control_command_snapshot_t snapshot = {0};
  snapshot.sequence = msg->sequence;
  snapshot.received_tick_ms = millis(); // 本地接收 tick（ARD §12.2）
  const uint8_t count =
      msg->count < CONTROL_COMMAND_MAILBOX_MAX_TARGETS
          ? msg->count
          : CONTROL_COMMAND_MAILBOX_MAX_TARGETS;
  for (uint8_t i = 0; i < count; ++i) {
    snapshot.targets[i].motor_id = msg->targets.data[i].motor_id;
    snapshot.targets[i].position = msg->targets.data[i].position;
    snapshot.targets[i].velocity = msg->targets.data[i].velocity;
    snapshot.targets[i].effort = msg->targets.data[i].effort;
  }
  snapshot.target_count = count;
  (void)motor_rt_submit_command(&snapshot); // 非法快照由 mailbox 拒绝并计数
}

static void trajectory_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorTrajectory *)msgin;
  // 声明计数必须在有界数组之内：超界一律拒绝，不做静默截断（截断会把
  // 一条轨迹悄悄变成另一条）。
  if (msg->count > msg->points.size ||
      msg->target_count > msg->target_ids.size ||
      msg->count > CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS ||
      msg->target_count > CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS) {
    return;
  }
  // 5KB 级别，静态而非栈上（回调在 micro_ros 任务栈上跑）。
  static control_trajectory_t trajectory;
  memset(&trajectory, 0, sizeof(trajectory));
  trajectory.sequence = msg->sequence;
  trajectory.trajectory_id = msg->trajectory_id;
  trajectory.received_tick_ms = millis(); // 本地接收 tick（ARD §12.2）
  trajectory.point_count = msg->count;
  trajectory.target_count = msg->target_count;
  for (uint16_t i = 0; i < msg->target_count; ++i) {
    trajectory.target_ids[i] = msg->target_ids.data[i];
  }
  for (uint16_t i = 0; i < msg->count; ++i) {
    trajectory.points[i].time_from_start_ms =
        msg->points.data[i].time_from_start_ms;
    trajectory.points[i].motor_id = msg->points.data[i].motor_id;
    trajectory.points[i].position = msg->points.data[i].position;
    trajectory.points[i].velocity = msg->points.data[i].velocity;
    trajectory.points[i].effort = msg->points.data[i].effort;
  }
  (void)motor_rt_submit_trajectory(&trajectory); // 非法轨迹由 mailbox 拒绝并计数
}

static void policy_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorPolicyArray *)msgin;
  // 声明计数必须在有界数组之内：超界一律拒绝，不做静默截断。
  if (msg->count > msg->targets.size ||
      msg->count > CONTROL_POLICY_MAILBOX_MAX_TARGETS) {
    return;
  }
  // 约 0.8KB，静态而非栈上（回调在 micro_ros 任务栈上跑）。
  static control_policy_snapshot_t snapshot;
  snapshot.received_tick_ms = millis(); // 本地接收 tick
  snapshot.request_id = msg->request_id;
  snapshot.policy_generation = msg->policy_generation;
  snapshot.target_count = msg->count;
  for (uint8_t i = 0; i < msg->count; ++i) {
    snapshot.targets[i].motor_id = msg->targets.data[i].motor_id;
    snapshot.targets[i].enabled = msg->targets.data[i].enabled;
    snapshot.targets[i].kp = msg->targets.data[i].kp;
    snapshot.targets[i].kd = msg->targets.data[i].kd;
    snapshot.targets[i].confirm_unprotected =
        msg->targets.data[i].confirm_unprotected;
  }
  (void)motor_rt_submit_policy(&snapshot); // 非法策略由 mailbox 整条拒绝并计数
}

// ---------------------------------------------------------------------------
// 发布填充
// ---------------------------------------------------------------------------

// 出站发布失败计数：best-effort 单条超传输上限时 rcl_publish 返回错误而消息
// **静默丢弃**（48 台时 motion_states / motor_status_array 都会中招）。此前所有
// 调用点都把返回值 (void) 掉了，故障在外部完全不可见。
static uint32_t g_publish_fail = 0U;

static void note_publish(rcl_ret_t rc) {
  if (rc != RCL_RET_OK) {
    ++g_publish_fail;
  }
}

static void publish_motion_states() {
  // 50 Hz（10ms tick ×2）：与设备的 20ms 反馈对齐；控制边界独立为 5ms/200Hz。
  // 100Hz 里有一半只是重复发同一份测量值——与 publish_statuses 降到 20Hz 是同一条理由
  // （那边注释写的正是「100Hz 里绝大部分只是 state_age 自增的重复内容」）。
  static uint8_t s_motion_phase = 0U;
  if ((++s_motion_phase % 2U) != 0U) {
    return;
  }
  motor_control_motion_state_t states[kRosArrayCapacity];
  const size_t count =
      motor_rt_copy_motion_states(states, sizeof(states) / sizeof(states[0]));
  const uint8_t out_count = (uint8_t)(count < kRosArrayCapacity ? count : kRosArrayCapacity);
  g_motion_states_msg.states.size = out_count;
  for (uint8_t i = 0; i < out_count; ++i) {
    auto &out = g_motion_states_msg.states.data[i];
    out.motor_id = states[i].motor_id;
    out.position = states[i].position_rad;
    out.velocity = states[i].velocity_rad_s;
    out.effort = states[i].effort_nm;
    out.state_age_ms = states[i].state_age_ms;
    out.target_source = states[i].target_source;
  }
  g_motion_states_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_motion_states_pub, &g_motion_states_msg, nullptr);
  note_publish(publish_rc);
}

static void publish_statuses() {
  // 状态是电平视图：故障/生命周期变化的即时通知由 motor_events（reliable）
  // 承担，本话题按 .msg 契约保持低频（20Hz：10ms tick ×5）；100Hz 里
  // 绝大部分只是 state_age 自增的重复内容，运动量新鲜度以 motion_states 为准。
  static uint8_t s_phase = 0U;
  if (++s_phase < 5U) {
    return;
  }
  s_phase = 0U;
  motor_control_status_t statuses[kRosArrayCapacity];
  const size_t count =
      motor_rt_copy_statuses(statuses, sizeof(statuses) / sizeof(statuses[0]));
  const uint8_t out_count = (uint8_t)(count < kRosArrayCapacity ? count : kRosArrayCapacity);
  g_status_msg.statuses.size = out_count;
  for (uint8_t i = 0; i < out_count; ++i) {
    auto &out = g_status_msg.statuses.data[i];
    out.motor_id = statuses[i].motor_id;
    out.temperature = statuses[i].temperature_c;
    out.current = statuses[i].current_a;
    out.voltage = statuses[i].voltage_v;
    out.drive_mode = statuses[i].drive_mode;
    out.unified_fault_code = statuses[i].unified_fault_code;
    out.vendor_fault_code = statuses[i].vendor_fault_code;
    out.protection_flags = statuses[i].protection_flags;
    out.limit_flags = statuses[i].limit_flags;
    out.online = statuses[i].online;
    out.lifecycle = statuses[i].lifecycle;
    // 设备侧安全事实（正向语义；known=false 时值无意义，页面显示 --）。
    out.device_protect_known = statuses[i].device_protect_known;
    out.device_protect_enabled = statuses[i].device_protect_enabled;
    out.device_direction_known = statuses[i].device_direction_known;
    out.device_direction = statuses[i].device_direction;
  }
  g_status_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_status_pub, &g_status_msg, nullptr);
  note_publish(publish_rc);
}

static uint32_t g_topology_tick = 0U; // 1s 节流的相位

static void publish_topology(uint32_t now_ms) {
  if ((uint32_t)(now_ms - g_topology_tick) < 1000U) {
    return;
  }
  g_topology_tick = now_ms;
  motor_rt_topology_entry_t entries[kRosArrayCapacity];
  const size_t count =
      motor_rt_copy_topology(entries, sizeof(entries) / sizeof(entries[0]));
  const uint8_t out_count = (uint8_t)(count < kRosArrayCapacity ? count : kRosArrayCapacity);
  g_topology_msg.entries.size = out_count;
  for (uint8_t i = 0; i < out_count; ++i) {
    auto &out = g_topology_msg.entries.data[i];
    out.motor_id = entries[i].motor_id;
    out.bus_index = entries[i].bus_index;
    out.protocol = entries[i].protocol;
    (void)rosidl_runtime_c__String__assignn(&out.model_name,
                                            entries[i].model_name,
                                            strlen(entries[i].model_name));
    out.capability_flags = entries[i].capability_flags;
    out.online = entries[i].online;
    out.conflict = entries[i].conflict;
  }
  g_topology_msg.schema_version = kMotorMsgSchemaVersion;
  g_topology_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_topology_pub, &g_topology_msg, nullptr);
  note_publish(publish_rc);
}

// 诊断：1Hz。这些计数器此前全在芯片里，除 heap_free 一个都不对外——实时性与
// 总线容量的问题因此只能靠 JTAG 定位。全部是单调递增的生命周期累计。
static void publish_diagnostics(uint32_t now_ms) {
  static uint32_t s_phase = 0U;
  if ((uint32_t)(now_ms - s_phase) < 1000U) {
    return;
  }
  s_phase = now_ms;
  motor_rt_diagnostics_t d = {};
  motor_rt_copy_diagnostics(&d);
  g_diagnostics_msg.sync_time_ns = rmw_uros_epoch_nanos();
  g_diagnostics_msg.rx_frame_count = d.kernel.rx_frame_count;
  g_diagnostics_msg.unknown_frame_count = d.kernel.unknown_frame_count;
  g_diagnostics_msg.decode_error_count = d.kernel.decode_error_count;
  g_diagnostics_msg.snapshot_accepted = d.kernel.snapshot_accepted;
  g_diagnostics_msg.snapshot_rejected = d.kernel.snapshot_rejected;
  g_diagnostics_msg.config_rejected = d.kernel.config_rejected;
  g_diagnostics_msg.event_dropped = d.kernel.event_dropped;
  g_diagnostics_msg.tx_dropped = d.kernel.tx_dropped;
  g_diagnostics_msg.p0_retry = d.kernel.p0_retry;
  g_diagnostics_msg.trajectory_accepted = d.kernel.trajectory_accepted;
  g_diagnostics_msg.trajectory_rejected = d.kernel.trajectory_rejected;
  g_diagnostics_msg.policy_accepted = d.kernel.policy_accepted;
  g_diagnostics_msg.policy_rejected = d.kernel.policy_rejected;
  g_diagnostics_msg.ring_drop.size = MOTOR_CONTROL_MAX_BUSES;
  for (uint32_t i = 0U; i < MOTOR_CONTROL_MAX_BUSES; ++i) {
    g_diagnostics_msg.ring_drop.data[i] = d.ring_drop[i];
  }
  g_diagnostics_msg.heap_free = d.heap_free;
  g_diagnostics_msg.mailbox_rejected_command = d.mailbox_rejected_command;
  g_diagnostics_msg.mailbox_rejected_trajectory = d.mailbox_rejected_trajectory;
  g_diagnostics_msg.mailbox_rejected_policy = d.mailbox_rejected_policy;
  // 合并本模块与 param_bridge 的出站失败：同一根 USB CDC 链路上的同类故障，
  // 分两个计数对外毫无意义（param_bridge 的 system_status 也是 best-effort）。
  g_diagnostics_msg.publish_fail =
      g_publish_fail + param_bridge_publish_fail_count();
  g_diagnostics_msg.rx_hw_lost.size = MOTOR_CONTROL_MAX_BUSES;
  g_diagnostics_msg.tx_frame_count.size = MOTOR_CONTROL_MAX_BUSES;
  for (uint32_t i = 0U; i < MOTOR_CONTROL_MAX_BUSES; ++i) {
    g_diagnostics_msg.rx_hw_lost.data[i] = d.rx_hw_lost[i];
    g_diagnostics_msg.tx_frame_count.data[i] = d.tx_frame_count[i];
  }
  const rcl_ret_t publish_rc =
      rcl_publish(&g_diagnostics_pub, &g_diagnostics_msg, nullptr);
  note_publish(publish_rc);
}

// 轨迹执行状态：10Hz。补上「我发的轨迹受理了吗、跑到哪了」——轨迹下行是 reliable
// 分片传输，但执行状态回执是 best-effort；没有它就只能靠猜。被拒的细节另见事件流
// 的 TRAJECTORY_REJECTED。
static void publish_trajectory_state(uint32_t now_ms) {
  static uint32_t s_phase = 0U;
  if ((uint32_t)(now_ms - s_phase) < 100U) {
    return;
  }
  s_phase = now_ms;
  motor_control_trajectory_state_t st = {};
  motor_rt_copy_trajectory_state(&st);
  g_trajectory_state_msg.sync_time_ns = rmw_uros_epoch_nanos();
  g_trajectory_state_msg.trajectory_id = st.trajectory_id;
  g_trajectory_state_msg.active = st.active;
  g_trajectory_state_msg.completed = st.completed;
  g_trajectory_state_msg.elapsed_ms = st.elapsed_ms;
  g_trajectory_state_msg.total_ms = st.total_ms;
  g_trajectory_state_msg.target_count = st.target_count;
  g_trajectory_state_msg.target_ids.size = st.target_count;
  for (uint16_t i = 0U; i < st.target_count; ++i) {
    g_trajectory_state_msg.target_ids.data[i] = st.target_ids[i];
  }
  const rcl_ret_t publish_rc =
      rcl_publish(&g_trajectory_state_pub, &g_trajectory_state_msg, nullptr);
  note_publish(publish_rc);
}

// 设备参数核对快照：1 Hz 心跳 + 状态变化即时一帧（变化由内核 CONFIG_RESULT /
// DEV_INIT 事件触发）。视图复用 param_bridge 那一份，不拷第二份。
// 本函数在 param_bridge_poll **之前**跑，读的是上一个 10ms 周期的快照（滞后一拍），
// 对 1 Hz 心跳与"写完一项看结论变绿"无影响——勿为消除这一拍去调换两个 poll。
static void publish_device_param_report(uint32_t now_ms) {
  static uint32_t s_phase = 0U;
  const bool heartbeat = (uint32_t)(now_ms - s_phase) >= 1000U;
  if (!heartbeat && !g_device_param_pending) {
    return;
  }
  if (heartbeat) {
    s_phase = now_ms;
  }
  g_device_param_pending = false;
  size_t view_count = 0U;
  const motor_control_param_view_t *views = param_bridge_views(&view_count);
  if (views == nullptr) {
    return;
  }
  const uint16_t count = (uint16_t)(view_count < kRosArrayCapacity
                                        ? view_count
                                        : kRosArrayCapacity);
  g_device_param_msg.entries.size = count;
  for (uint16_t i = 0U; i < count; ++i) {
    auto &out = g_device_param_msg.entries.data[i];
    const motor_control_param_view_t &v = views[i];
    out.motor_id = v.motor_id;
    out.dev_init_state = v.dev_init;
    out.record_dirty = v.record_dirty_slots != 0U;
    out.mismatch_mask = v.dev_mismatch_mask;
    out.rec_pos_min = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.rec_pos_max = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.rec_vel_limit = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    // 力矩列按 **A** 呈现（设备对象 0x2538 存的就是电流），与 param_set 的 field=3
    // 同口径；record_si/dev_si 是关节侧 N·m，这里过一次换算——系数只有内核有
    // （v.torque_per_amp），页面不碰 mA。系数缺失时回 NaN，守住"缺失量一律 NaN"
    // （绝不落 0）。位置/速度列与内核同为 rad / rad/s，直通不换算。
    out.rec_torque_limit = param_units_nm_to_amp(
        v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX], v.torque_per_amp);
    out.rec_safety = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.rec_direction = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
    out.dev_pos_min = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.dev_pos_max = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.dev_vel_limit = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    out.dev_torque_limit =
        param_units_nm_to_amp(v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX],
                              v.torque_per_amp);
    out.dev_safety = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.dev_direction = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  }
  // .msg 同时声明 count 与 entries（两者都上线），口径同
  // MotorTrajectoryState.target_count：不设它消费者会读到 0 条。
  g_device_param_msg.count = count;
  g_device_param_msg.sync_time_ns = rmw_uros_epoch_nanos();
  note_publish(rcl_publish(&g_device_param_pub, &g_device_param_msg, nullptr));
}

static void publish_events() {
  motor_control_event_t events[16];
  size_t n;
  while ((n = motor_rt_drain_events(events, 16)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      // 参数桥先收敛 outstanding（同任务顺序调用，无线程问题），再发布。
      param_bridge_on_config_result(&events[i]);
      // 参数面结论会变的两类事件：请求落地、比对定论。报告是低频电平视图，
      // 只在有必要时才不等到下一个心跳。
      if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT ||
          events[i].type == MOTOR_CONTROL_EVENT_DEV_INIT) {
        g_device_param_pending = true;
      }
      g_event_msg.tick_ms = events[i].tick_ms;
      g_event_msg.event_type = (uint8_t)events[i].type;
      g_event_msg.motor_id = events[i].motor_id;
      g_event_msg.bus_index = events[i].bus_index;
      g_event_msg.reason = events[i].reason;
      g_event_msg.request_id = events[i].request_id;
      g_event_msg.event_seq = events[i].seq;
      const rcl_ret_t publish_rc = rcl_publish(&g_events_pub, &g_event_msg, nullptr);
      note_publish(publish_rc);
    }
  }
}

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (node == nullptr || executor == nullptr) {
    return false;
  }

  g_command_msg.targets.capacity = kRosArrayCapacity;
  g_command_msg.targets.data = g_command_targets_buf;
  g_trajectory_msg.points.capacity = kRosTrajectoryPointCapacity;
  g_trajectory_msg.points.data = g_trajectory_points_buf;
  g_trajectory_msg.target_ids.capacity = MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS;
  g_trajectory_msg.target_ids.data = g_trajectory_target_ids_buf;
  g_policy_msg.targets.capacity = kRosArrayCapacity;
  g_policy_msg.targets.data = g_policy_targets_buf;
  g_diagnostics_msg.ring_drop.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_diagnostics_msg.ring_drop.data = g_diagnostics_ring_buf;
  g_diagnostics_msg.rx_hw_lost.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_diagnostics_msg.rx_hw_lost.data = g_diagnostics_rx_lost_buf;
  g_diagnostics_msg.tx_frame_count.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_diagnostics_msg.tx_frame_count.data = g_diagnostics_tx_count_buf;
  // 序列接线没有类型层面的保护，漏一条就是往 NULL 写。把它变成启动期硬失败，
  // 而不是等读到"两个字段一模一样"才发现。
  {
    const rosidl_runtime_c__uint32__Sequence *buses[] = {
        &g_diagnostics_msg.ring_drop,
        &g_diagnostics_msg.rx_hw_lost,
        &g_diagnostics_msg.tx_frame_count,
    };
    for (size_t i = 0U; i < sizeof(buses) / sizeof(buses[0]); ++i) {
      if (buses[i]->data == NULL ||
          buses[i]->capacity < MOTOR_CONTROL_MAX_BUSES) {
        return false;
      }
    }
  }
  g_trajectory_state_msg.target_ids.capacity =
      MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS;
  g_trajectory_state_msg.target_ids.data = g_trajectory_state_ids_buf;
  g_motion_states_msg.states.capacity = kRosArrayCapacity;
  g_motion_states_msg.states.data = g_motion_states_buf;
  g_status_msg.statuses.capacity = kRosArrayCapacity;
  g_status_msg.statuses.data = g_status_buf;
  g_topology_msg.entries.capacity = kRosArrayCapacity;
  g_topology_msg.entries.data = g_topology_buf;
  g_device_param_msg.entries.capacity = kRosArrayCapacity;
  g_device_param_msg.entries.data = g_device_param_entries;

  // model_name 是全桥唯一 String 成员：静态缓冲零值时 data==NULL，首个设备
  // 注册后的拓扑发布即向 NULL memcpy（BusFault）。static 守卫一次性初始化，
  // 不随会话重连反复分配；容量由 String__assignn 首拍按需增长，稳态零分配。
  static bool s_topology_strings_ready = false;
  if (!s_topology_strings_ready) {
    for (size_t i = 0; i < sizeof(g_topology_buf) / sizeof(g_topology_buf[0]);
         ++i) {
      (void)rosidl_runtime_c__String__init(&g_topology_buf[i].model_name);
    }
    s_topology_strings_ready = true;
  }

  // ---- 订阅 ----
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_stream();
  if (rcl_subscription_init(&g_command_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(
                                can_bridge_msgs, msg, MotorCommandArray),
                            kCommandTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_command_up = true;

  sub_opts = rcl_subscription_get_default_options();
  // 轨迹必须 reliable：它允许跨 MTU 分片，而 best-effort 流不分片，>512 B 会被静默
  // 丢弃。当前 STREAM_HISTORY=16（输入/输出各 8 KB）足以承载 256 点上限（约 5.2 KB），
  // 网页对更长示教按 256 点分段。发布侧 rosbridge 也必须 reliable 才配得上。
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_trajectory_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        MotorTrajectory),
                            kTrajectoryTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_trajectory_up = true;

  sub_opts = rcl_subscription_get_default_options();
  // 策略是低频状态声明，允许重发；用 reliable 小深度避免"没生效"被静默。
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_policy_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        MotorPolicyArray),
                            kPolicyTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_policy_up = true;

  // ---- 发布 ----
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_motion_states_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(
                             can_bridge_msgs, msg, MotorMotionStateArray),
                         kMotionStatesTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_motion_states_up = true;

  pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_status_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorStatusArray),
                         kStatusTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_status_up = true;

  pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_topology_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorTopology),
                         kTopologyTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_topology_up = true;

  pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_reliable(8);
  if (rcl_publisher_init(&g_events_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorEvent),
                         kEventsTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_events_up = true;

  pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_diagnostics_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorDiagnostics),
                         kDiagnosticsTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_diagnostics_up = true;

  pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_trajectory_state_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorTrajectoryState),
                         kTrajectoryStateTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_trajectory_state_up = true;

  pub_opts = rcl_publisher_get_default_options();
  // **可靠**：48 台时本消息约 3 KB，超过一个 MTU(512 B)；best-effort 流不分片，
  // 会被静默丢弃（与 motor_topology 同一处理，那里也是 48 台量级）。
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_device_param_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(
                             can_bridge_msgs, msg, DeviceParamReport),
                         kDeviceParamReportTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_device_param_up = true;

  // ---- executor ----
  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg,
                                     command_array_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_command_added = true;
  if (rclc_executor_add_subscription(executor, &g_trajectory_sub,
                                     &g_trajectory_msg, trajectory_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_trajectory_added = true;
  if (rclc_executor_add_subscription(executor, &g_policy_sub, &g_policy_msg,
                                     policy_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_policy_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (executor != nullptr && g_command_added) {
    rclc_executor_remove_subscription(executor, &g_command_sub);
    g_command_added = false;
  }
  if (executor != nullptr && g_trajectory_added) {
    rclc_executor_remove_subscription(executor, &g_trajectory_sub);
    g_trajectory_added = false;
  }
  if (g_command_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_command_sub, node);
    (void)rc;
    g_command_up = false;
  }
  if (executor != nullptr && g_policy_added) {
    rclc_executor_remove_subscription(executor, &g_policy_sub);
    g_policy_added = false;
  }
  if (g_trajectory_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_trajectory_sub, node);
    (void)rc;
    g_trajectory_up = false;
  }
  if (g_policy_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_policy_sub, node);
    (void)rc;
    g_policy_up = false;
  }
  if (g_motion_states_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_motion_states_pub, node);
    (void)rc;
    g_motion_states_up = false;
  }
  if (g_status_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_status_pub, node);
    (void)rc;
    g_status_up = false;
  }
  if (g_topology_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_topology_pub, node);
    (void)rc;
    g_topology_up = false;
  }
  if (g_events_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_events_pub, node);
    (void)rc;
    g_events_up = false;
  }
  if (g_diagnostics_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_diagnostics_pub, node);
    (void)rc;
    g_diagnostics_up = false;
  }
  if (g_trajectory_state_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_trajectory_state_pub, node);
    (void)rc;
    g_trajectory_state_up = false;
  }
  if (g_device_param_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_device_param_pub, node);
    (void)rc;
    g_device_param_up = false;
  }
}

void motor_bridge_poll(void) {
  const uint32_t now_ms = millis();
  publish_motion_states();
  publish_statuses();
  publish_topology(now_ms);
  publish_events();
  publish_diagnostics(now_ms);
  publish_trajectory_state(now_ms);
  publish_device_param_report(now_ms);
}
