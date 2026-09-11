// 电机域 ROS 桥实现：ID 化消息面（ARD-0007 §15-20）。
//
// 订阅（5 句柄）：
//   /motor_command_array（100Hz 完整快照）→ 校验 → mailbox 快照
//   micro_ros/motor_policy（Enable/Kp/Kd/软限幅）→ mailbox 配置
//   micro_ros/motor_config（模式/设备侧限位事务）→ mailbox 配置
//   micro_ros/motor_enable（全局授权电平）
// 发布（5）：
//   micro_ros/motion_states（100Hz 目标，只读副本）
//   micro_ros/motor_status_array（低频诊断）
//   micro_ros/motor_topology（1s 全量拓扑）
//   micro_ros/motor_events（事件转发）
//   micro_ros/motor_config_result（事务结果，request_id 关联）
//
// 本桥不做：joint_name 语义、协议/CAN 字节、直接 RX 访问（ARD §28.1）。
// QoS（TBD-ROS-05 默认表）：命令/状态流 best-effort depth1（latest-wins、
// 无历史积压，ARD §34）；策略/配置/结果/事件/拓扑 reliable 小深度。

#include "motor_bridge.h"

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#include <can_bridge_msgs/msg/motor_bind.h>
#include <can_bridge_msgs/msg/motor_command_array.h>
#include <can_bridge_msgs/msg/motor_config.h>
#include <can_bridge_msgs/msg/motor_config_result.h>
#include <can_bridge_msgs/msg/motor_event.h>
#include <can_bridge_msgs/msg/motor_motion_state_array.h>
#include <can_bridge_msgs/msg/motor_policy.h>
#include <can_bridge_msgs/msg/motor_status_array.h>
#include <can_bridge_msgs/msg/motor_topology.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/time_sync.h>

#include "motor_rt_task.h"

// 序列静态存储（rosidl C 序列为 data/size/capacity 三元组；容量预分配，
// 禁止反序列化路径上的动态分配）。
static can_bridge_msgs__msg__MotorMotionStateTarget
    g_motion_states_buf[64];
static can_bridge_msgs__msg__MotorStatusTarget g_status_buf[64];
static can_bridge_msgs__msg__MotorTopologyEntry g_topology_buf[64];
static can_bridge_msgs__msg__MotorCommandTarget g_command_targets_buf[64];

// ---- 话题名 ----
static const char *kCommandTopic = "/motor_command_array";
static const char *kPolicyTopic = "micro_ros/motor_policy";
static const char *kConfigTopic = "micro_ros/motor_config";
static const char *kBindTopic = "micro_ros/motor_bind";
static const char *kEnableTopic = "micro_ros/motor_enable";
static const char *kMotionStatesTopic = "micro_ros/motion_states";
static const char *kStatusTopic = "micro_ros/motor_status_array";
static const char *kTopologyTopic = "micro_ros/motor_topology";
static const char *kEventsTopic = "micro_ros/motor_events";
static const char *kConfigResultTopic = "micro_ros/motor_config_result";

// ---- 订阅（5）----
static rcl_subscription_t g_command_sub;
static can_bridge_msgs__msg__MotorCommandArray g_command_msg;
static bool g_command_up = false;
static bool g_command_added = false;

static rcl_subscription_t g_policy_sub;
static can_bridge_msgs__msg__MotorPolicy g_policy_msg;
static bool g_policy_up = false;
static bool g_policy_added = false;

static rcl_subscription_t g_config_sub;
static can_bridge_msgs__msg__MotorConfig g_config_msg;
static bool g_config_up = false;
static bool g_config_added = false;

static rcl_subscription_t g_bind_sub;
static can_bridge_msgs__msg__MotorBind g_bind_msg;
static bool g_bind_up = false;
static bool g_bind_added = false;

static rcl_subscription_t g_enable_sub;
static std_msgs__msg__Bool g_enable_msg;
static bool g_enable_up = false;
static bool g_enable_added = false;

// ---- 发布（5）----
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

static rcl_publisher_t g_config_result_pub;
static can_bridge_msgs__msg__MotorConfigResult g_config_result_msg;
static bool g_config_result_up = false;

static uint32_t g_policy_request_id = 1U; // Policy 无 request_id，桥代生成

// Policy 请求 id 发放：0 为契约保留值（config_cb 与内核 submit_config 均拒
// 0），回绕时跳过。
static uint32_t next_policy_request_id(void) {
  const uint32_t id = g_policy_request_id++;
  if (g_policy_request_id == 0U) {
    g_policy_request_id = 1U;
  }
  return id;
}

size_t motor_bridge_executor_handle_count(void) {
  return 5U; // command/policy/config/bind/enable 订阅
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

static void policy_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorPolicy *)msgin;

  control_mailbox_config_request_t request = {0};
  request.received_tick_ms = millis();
  request.motor_id = msg->motor_id;

  // Enable 先于增益/限位，以先表达运动许可变化；每条请求使用独立
  // request_id，使各自的 CONFIG_RESULT 可关联。
  request.kind = CONTROL_MAILBOX_CONFIG_SET_ENABLE;
  request.value.enable = msg->enable;
  request.request_id = next_policy_request_id();
  (void)motor_rt_submit_config(&request);

  if (msg->set_gains) {
    request.kind = CONTROL_MAILBOX_CONFIG_SET_GAINS;
    request.value.gains.kp = msg->kp;
    request.value.gains.kd = msg->kd;
    request.request_id = next_policy_request_id();
    (void)motor_rt_submit_config(&request);
  }
  if (msg->set_limits) {
    request.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
    request.value.position_limits.min_position_rad = msg->min_position_rad;
    request.value.position_limits.max_position_rad = msg->max_position_rad;
    request.request_id = next_policy_request_id();
    (void)motor_rt_submit_config(&request);
  }
}

static void config_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorConfig *)msgin;
  if (msg->request_id == 0U) {
    return; // request_id 契约：非 0
  }
  control_mailbox_config_request_t request = {0};
  request.request_id = msg->request_id;
  request.received_tick_ms = millis();
  request.motor_id = msg->motor_id;
  if (msg->kind == can_bridge_msgs__msg__MotorConfig__KIND_SET_MODE) {
    request.kind = CONTROL_MAILBOX_CONFIG_SET_MODE;
    request.value.mode = msg->mode;
  } else if (msg->kind ==
             can_bridge_msgs__msg__MotorConfig__KIND_SET_LIMITS) {
    request.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
    request.value.position_limits.min_position_rad = msg->min_position_rad;
    request.value.position_limits.max_position_rad = msg->max_position_rad;
  } else {
    return; // 未知 kind
  }
  (void)motor_rt_submit_config(&request);
}

// 绑定回调（TBD-PROFILE-01 过渡通道）：校验后入 mailbox bind FIFO；
// 结果经 motor_events 的 PROFILE_BOUND/PROFILE_REJECTED 事件回告。
static void bind_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorBind *)msgin;
  if (msg->motor_id == 0U || msg->model_name.size == 0U ||
      msg->model_name.size >= CONTROL_MAILBOX_BIND_NAME_CAPACITY ||
      msg->model_name.data == nullptr) {
    return; // 非法载荷静默丢弃（事件层无法回告无 bus 上下文的绑定）
  }
  char name[CONTROL_MAILBOX_BIND_NAME_CAPACITY];
  memcpy(name, msg->model_name.data, msg->model_name.size);
  name[msg->model_name.size] = '\0';
  (void)motor_rt_submit_bind(msg->motor_id, name);
}

static void enable_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const std_msgs__msg__Bool *)msgin;
  motor_rt_request_authorization(msg->data);
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

static void publish_events_and_results() {
  motor_control_event_t events[16];
  size_t n;
  while ((n = motor_rt_drain_events(events, 16)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT) {
        g_config_result_msg.request_id = events[i].request_id;
        g_config_result_msg.motor_id = events[i].motor_id;
        g_config_result_msg.result =
            (uint8_t)(events[i].reason & 0xFFU);
        const rcl_ret_t publish_rc = rcl_publish(&g_config_result_pub, &g_config_result_msg, nullptr);
  (void)publish_rc;
      }
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

size_t motor_bridge_executor_handle_count(void); // 见头文件

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

  sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_reliable(4);
  if (rcl_subscription_init(&g_policy_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        MotorPolicy),
                            kPolicyTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_policy_up = true;

  sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_reliable(4);
  if (rcl_subscription_init(&g_config_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        MotorConfig),
                            kConfigTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_config_up = true;

  sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_reliable(4);
  if (rcl_subscription_init(&g_bind_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                        MotorBind),
                            kBindTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_bind_up = true;
  // 有界字符串（<=31+NUL）反序列化要求预置容量：直接挂静态缓冲，零堆分配
  //（与序列缓冲同一静态存储口径，见文件头注释）。
  static char s_bind_name_buf[CONTROL_MAILBOX_BIND_NAME_CAPACITY];
  g_bind_msg.model_name.data = s_bind_name_buf;
  g_bind_msg.model_name.size = 0U;
  g_bind_msg.model_name.capacity = sizeof(s_bind_name_buf);

  sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_enable_sub, node,
                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                            kEnableTopic, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_enable_up = true;

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
  pub_opts.qos = qos_reliable(4);
  if (rcl_publisher_init(&g_config_result_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg,
                                                     MotorConfigResult),
                         kConfigResultTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_config_result_up = true;

  // ---- executor ----
  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg,
                                     command_array_cb,
                                     ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(executor, &g_policy_sub, &g_policy_msg,
                                     policy_cb, ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(executor, &g_config_sub, &g_config_msg,
                                     config_cb, ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(executor, &g_bind_sub, &g_bind_msg,
                                     bind_cb, ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(executor, &g_enable_sub, &g_enable_msg,
                                     enable_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_command_added = g_policy_added = g_config_added = g_bind_added =
      g_enable_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (executor != nullptr) {
    if (g_command_added) {
      rclc_executor_remove_subscription(executor, &g_command_sub);
      g_command_added = false;
    }
    if (g_policy_added) {
      rclc_executor_remove_subscription(executor, &g_policy_sub);
      g_policy_added = false;
    }
    if (g_config_added) {
      rclc_executor_remove_subscription(executor, &g_config_sub);
      g_config_added = false;
    }
    if (g_bind_added) {
      rclc_executor_remove_subscription(executor, &g_bind_sub);
      g_bind_added = false;
    }
    if (g_enable_added) {
      rclc_executor_remove_subscription(executor, &g_enable_sub);
      g_enable_added = false;
    }
  }
  if (g_command_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_command_sub, node);
    (void)rc;
    g_command_up = false;
  }
  if (g_policy_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_policy_sub, node);
    (void)rc;
    g_policy_up = false;
  }
  if (g_config_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_config_sub, node);
    (void)rc;
    g_config_up = false;
  }
  if (g_bind_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_bind_sub, node);
    (void)rc;
    g_bind_up = false;
  }
  if (g_enable_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_enable_sub, node);
    (void)rc;
    g_enable_up = false;
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
  if (g_config_result_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_config_result_pub, node);
    (void)rc;
    g_config_result_up = false;
  }
}

void motor_bridge_poll(void) {
  const uint32_t now_ms = millis();
  publish_motion_states();
  publish_statuses();
  publish_topology(now_ms);
  publish_events_and_results();
}
