// 电机域 ROS 桥实现：ID 化消息面（ARD-0007 §15；下行已精简）。
//
// 订阅（1 句柄）：
//   /motor_command_array（100Hz 完整快照）→ 校验 → mailbox 快照
// 发布（4）：
//   micro_ros/motion_states（100Hz 运动状态，只读副本）
//   micro_ros/motor_status_array（20Hz 诊断）
//   micro_ros/motor_topology（1s 全量拓扑）
//   micro_ros/motor_events（事件转发）
//
// 下行无配置通道：型号绑定由内核在发现建档/重连时经 READ_IDENTITY 事务读
// 设备自报身份并查型号目录完成；授权/增益/限位取内核安全默认（建档
// policy_enable 默认 true、全局授权 mailbox 引导豁免、增益回退
// profile default_kp/kd、CERB 限位在驱动器侧配置）。
//
// 本桥不做：joint_name 语义、协议/CAN 字节、直接 RX 访问（ARD §28.1）。
// QoS（TBD-ROS-05 默认表）：命令/状态流 best-effort depth1（latest-wins、
// 无历史积压，ARD §34）；事件/拓扑 reliable 小深度。

#include "motor_bridge.h"

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <can_bridge_msgs/msg/motor_command_array.h>
#include <can_bridge_msgs/msg/motor_event.h>
#include <can_bridge_msgs/msg/motor_motion_state_array.h>
#include <can_bridge_msgs/msg/motor_status_array.h>
#include <can_bridge_msgs/msg/motor_topology.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/time_sync.h>

#include "motor_rt_task.h"
#include "param_bridge.h"

// 序列静态存储（rosidl C 序列为 data/size/capacity 三元组；容量预分配，
// 禁止反序列化路径上的动态分配）。
static can_bridge_msgs__msg__MotorMotionStateTarget
    g_motion_states_buf[64];
static can_bridge_msgs__msg__MotorStatusTarget g_status_buf[64];
static can_bridge_msgs__msg__MotorTopologyEntry g_topology_buf[64];
static can_bridge_msgs__msg__MotorCommandTarget g_command_targets_buf[64];

// ---- 话题名 ----
static const char *kCommandTopic = "/motor_command_array";
static const char *kMotionStatesTopic = "micro_ros/motion_states";
static const char *kStatusTopic = "micro_ros/motor_status_array";
static const char *kTopologyTopic = "micro_ros/motor_topology";
static const char *kEventsTopic = "micro_ros/motor_events";

// ---- 订阅（1）----
static rcl_subscription_t g_command_sub;
static can_bridge_msgs__msg__MotorCommandArray g_command_msg;
static bool g_command_up = false;
static bool g_command_added = false;

// ---- 发布（4）----
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

size_t motor_bridge_executor_handle_count(void) {
  return 1U; // command 订阅（下行配置通道已删；型号绑定在内核内自动完成）
}

// QoS 助手（TBD-ROS-05 默认表）。
static rmw_qos_profile_t qos_stream() {
  // 高频流：best-effort + depth 1（latest-wins，无历史积压，ARD §34）。
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.depth = 1;
  return qos;
}

static rmw_qos_profile_t qos_reliable(size_t depth) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.depth = depth;
  return qos;
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

// ---------------------------------------------------------------------------
// 发布填充
// ---------------------------------------------------------------------------

static void publish_motion_states() {
  motor_control_motion_state_t states[64];
  const size_t count =
      motor_rt_copy_motion_states(states, sizeof(states) / sizeof(states[0]));
  const uint8_t out_count = (uint8_t)(count < 64U ? count : 64U);
  g_motion_states_msg.states.size = out_count;
  for (uint8_t i = 0; i < out_count; ++i) {
    auto &out = g_motion_states_msg.states.data[i];
    out.motor_id = states[i].motor_id;
    out.position = states[i].position_rad;
    out.velocity = states[i].velocity_rad_s;
    out.effort = states[i].effort_nm;
    out.state_age_ms = states[i].state_age_ms;
  }
  g_motion_states_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_motion_states_pub, &g_motion_states_msg, nullptr);
  (void)publish_rc;
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
  motor_control_status_t statuses[64];
  const size_t count =
      motor_rt_copy_statuses(statuses, sizeof(statuses) / sizeof(statuses[0]));
  const uint8_t out_count = (uint8_t)(count < 64U ? count : 64U);
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
    out.state_age_ms = statuses[i].state_age_ms;
  }
  g_status_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_status_pub, &g_status_msg, nullptr);
  (void)publish_rc;
}

static uint32_t g_topology_tick = 0U; // 1s 节流的相位

static void publish_topology(uint32_t now_ms) {
  if ((uint32_t)(now_ms - g_topology_tick) < 1000U) {
    return;
  }
  g_topology_tick = now_ms;
  motor_rt_topology_entry_t entries[64];
  const size_t count =
      motor_rt_copy_topology(entries, sizeof(entries) / sizeof(entries[0]));
  const uint8_t out_count = (uint8_t)(count < 64U ? count : 64U);
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
    out.lifecycle = entries[i].lifecycle;
  }
  g_topology_msg.sync_time_ns = rmw_uros_epoch_nanos();
  const rcl_ret_t publish_rc = rcl_publish(&g_topology_pub, &g_topology_msg, nullptr);
  (void)publish_rc;
}

static void publish_events() {
  motor_control_event_t events[16];
  size_t n;
  while ((n = motor_rt_drain_events(events, 16)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      // 参数桥先收敛 outstanding（同任务顺序调用，无线程问题），再发布。
      param_bridge_on_config_result(&events[i]);
      g_event_msg.tick_ms = events[i].tick_ms;
      g_event_msg.event_type = (uint8_t)events[i].type;
      g_event_msg.motor_id = events[i].motor_id;
      g_event_msg.bus_index = events[i].bus_index;
      g_event_msg.reason = events[i].reason;
      g_event_msg.request_id = events[i].request_id;
      const rcl_ret_t publish_rc = rcl_publish(&g_events_pub, &g_event_msg, nullptr);
      (void)publish_rc;
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

  g_command_msg.targets.capacity = 64;
  g_command_msg.targets.data = g_command_targets_buf;
  g_motion_states_msg.states.capacity = 64;
  g_motion_states_msg.states.data = g_motion_states_buf;
  g_status_msg.statuses.capacity = 64;
  g_status_msg.statuses.data = g_status_buf;
  g_topology_msg.entries.capacity = 64;
  g_topology_msg.entries.data = g_topology_buf;

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

  // ---- executor ----
  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg,
                                     command_array_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_command_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (executor != nullptr && g_command_added) {
    rclc_executor_remove_subscription(executor, &g_command_sub);
    g_command_added = false;
  }
  if (g_command_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_command_sub, node);
    (void)rc;
    g_command_up = false;
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
}

void motor_bridge_poll(void) {
  const uint32_t now_ms = millis();
  publish_motion_states();
  publish_statuses();
  publish_topology(now_ms);
  publish_events();
}
