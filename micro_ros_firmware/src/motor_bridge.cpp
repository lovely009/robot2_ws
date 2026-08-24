// 电机域 ROS 桥接实现。
//
// ROS 回调只校验/转换消息并写 control task mailbox；20 ms 定时器只复制
// control task 维护的最新关节快照。协议、总线与正常发送路径均不在本模块。

#include "motor_bridge.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>

#include "control_task.h"
#include "time_utils.h"

#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE

static const char *kJointCommandsTopic = "/joint_commands";
static const char *kJointFeedbackTopic = "micro_ros/JointState";
static const char *kMotorEnableTopic = "micro_ros/motor_enable";
static const size_t kJointCommandMaxMotors = 2;  // 当前默认两台测试电机

static rcl_subscription_t g_joint_state_sub;
static rcl_subscription_t g_motor_enable_sub;
static rcl_publisher_t g_joint_feedback_pub;
static sensor_msgs__msg__JointState g_joint_state_msg;
static sensor_msgs__msg__JointState g_joint_feedback_msg;
static std_msgs__msg__Bool g_motor_enable_msg;

// micro-ROS 不会为 NULL 序列/字符串分配空间：全部静态预分配。
static rosidl_runtime_c__String g_joint_name_strs[kJointCommandMaxMotors];
static char g_joint_name_buf[kJointCommandMaxMotors][32];
static double g_joint_position_buf[kJointCommandMaxMotors];
static double g_joint_velocity_buf[kJointCommandMaxMotors];
static double g_joint_effort_buf[kJointCommandMaxMotors];
static rosidl_runtime_c__String g_joint_fb_name_strs[kJointCommandMaxMotors];
static char g_joint_fb_name_buf[kJointCommandMaxMotors][32];
static double g_joint_fb_position_buf[kJointCommandMaxMotors];
static double g_joint_fb_velocity_buf[kJointCommandMaxMotors];
static double g_joint_fb_effort_buf[kJointCommandMaxMotors];

// 调试计数（GDB 可读）。
volatile uint32_t g_joint_feedback_pub_count = 0;
volatile int32_t g_joint_feedback_pub_rc = 0;

static bool g_ready = false;
static bool g_joint_sub_up = false;
static bool g_enable_sub_up = false;
static bool g_publisher_up = false;
static bool g_joint_sub_added = false;
static bool g_enable_sub_added = false;

static void joint_state_sub_cb(const void *msg) {
  const sensor_msgs__msg__JointState *state =
      (const sensor_msgs__msg__JointState *)msg;
  if (state == nullptr || state->name.data == nullptr) {
    return;
  }

  size_t count = state->name.size;
  if (count > state->name.capacity) {
    count = state->name.capacity;
  }
  if (count > kJointCommandMaxMotors) {
    count = kJointCommandMaxMotors;
  }
  uint32_t received_ms = millis();
  for (size_t index = 0; index < count; ++index) {
    motor_control_command_t command = {};
    const rosidl_runtime_c__String *name = &state->name.data[index];
    if (name->data == nullptr || name->size == 0U ||
        name->size >= name->capacity ||
        name->size >= sizeof(command.joint_name)) {
      continue;
    }
    memcpy(command.joint_name, name->data, name->size);
    command.joint_name[name->size] = '\0';

    // 恰好只有 position 或 velocity 时选择对应 vaxor 模式；其余保持 MIT。
    const bool has_position =
        state->position.data != nullptr && state->position.size > index &&
        state->position.capacity > index;
    const bool has_velocity =
        state->velocity.data != nullptr && state->velocity.size > index &&
        state->velocity.capacity > index;
    const bool has_effort =
        state->effort.data != nullptr && state->effort.size > index &&
        state->effort.capacity > index;
    command.mode = MOTOR_PROTOCOL_WRIST_MODE_MIT;
    if (has_position && !has_velocity) {
      command.mode = MOTOR_PROTOCOL_WRIST_MODE_POSITION;
    } else if (has_velocity && !has_position) {
      command.mode = MOTOR_PROTOCOL_WRIST_MODE_VELOCITY;
    }
    if (has_position) {
      command.position = state->position.data[index];
    }
    if (has_velocity) {
      command.velocity = state->velocity.data[index];
    }
    if (has_effort) {
      command.effort = state->effort.data[index];
    }
    if (!isfinite(command.position) || !isfinite(command.velocity) ||
        !isfinite(command.effort)) {
      continue;
    }
    (void)control_task_submit_command(&command, received_ms);
  }
}

static void motor_enable_sub_cb(const void *msg) {
  const std_msgs__msg__Bool *enabled = (const std_msgs__msg__Bool *)msg;
  if (enabled != nullptr) {
    control_task_set_authorization(enabled->data);
  }
}

static void setup_command_msg_buffers() {
  g_joint_state_msg.name.data = g_joint_name_strs;
  g_joint_state_msg.name.size = 0;
  g_joint_state_msg.name.capacity = kJointCommandMaxMotors;
  for (size_t index = 0; index < kJointCommandMaxMotors; ++index) {
    g_joint_name_strs[index].data = g_joint_name_buf[index];
    g_joint_name_strs[index].size = 0;
    g_joint_name_strs[index].capacity = sizeof(g_joint_name_buf[index]);
  }
  g_joint_state_msg.position.data = g_joint_position_buf;
  g_joint_state_msg.position.size = 0;
  g_joint_state_msg.position.capacity = kJointCommandMaxMotors;
  g_joint_state_msg.velocity.data = g_joint_velocity_buf;
  g_joint_state_msg.velocity.size = 0;
  g_joint_state_msg.velocity.capacity = kJointCommandMaxMotors;
  g_joint_state_msg.effort.data = g_joint_effort_buf;
  g_joint_state_msg.effort.size = 0;
  g_joint_state_msg.effort.capacity = kJointCommandMaxMotors;
}

static void setup_feedback_msg_buffers() {
  g_joint_feedback_msg.name.data = g_joint_fb_name_strs;
  g_joint_feedback_msg.name.size = 0;
  g_joint_feedback_msg.name.capacity = kJointCommandMaxMotors;
  for (size_t index = 0; index < kJointCommandMaxMotors; ++index) {
    g_joint_fb_name_strs[index].data = g_joint_fb_name_buf[index];
    g_joint_fb_name_strs[index].size = 0;
    g_joint_fb_name_strs[index].capacity = sizeof(g_joint_fb_name_buf[index]);
  }
  g_joint_feedback_msg.position.data = g_joint_fb_position_buf;
  g_joint_feedback_msg.position.size = 0;
  g_joint_feedback_msg.position.capacity = kJointCommandMaxMotors;
  g_joint_feedback_msg.velocity.data = g_joint_fb_velocity_buf;
  g_joint_feedback_msg.velocity.size = 0;
  g_joint_feedback_msg.velocity.capacity = kJointCommandMaxMotors;
  g_joint_feedback_msg.effort.data = g_joint_fb_effort_buf;
  g_joint_feedback_msg.effort.size = 0;
  g_joint_feedback_msg.effort.capacity = kJointCommandMaxMotors;
  g_joint_feedback_msg.header.frame_id.data = (char *)"joints";
  g_joint_feedback_msg.header.frame_id.size = 6;
  g_joint_feedback_msg.header.frame_id.capacity = 7;
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (rclc_subscription_init_default(
          &g_joint_state_sub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointCommandsTopic) != RCL_RET_OK) {
    return false;
  }
  g_joint_sub_up = true;

  if (rclc_subscription_init_default(
          &g_motor_enable_sub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          kMotorEnableTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_enable_sub_up = true;

  if (rclc_publisher_init_default(
          &g_joint_feedback_pub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointFeedbackTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_publisher_up = true;
  setup_command_msg_buffers();
  setup_feedback_msg_buffers();
  g_motor_enable_msg.data = false;

  if (rclc_executor_add_subscription(executor, &g_joint_state_sub,
                                     &g_joint_state_msg, joint_state_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_joint_sub_added = true;
  if (rclc_executor_add_subscription(executor, &g_motor_enable_sub,
                                     &g_motor_enable_msg, motor_enable_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_enable_sub_added = true;
  g_ready = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_ready = false;
  if (g_enable_sub_added) {
    rcl_ret_t rc =
        rclc_executor_remove_subscription(executor, &g_motor_enable_sub);
    (void)rc;
    g_enable_sub_added = false;
  }
  if (g_joint_sub_added) {
    rcl_ret_t rc =
        rclc_executor_remove_subscription(executor, &g_joint_state_sub);
    (void)rc;
    g_joint_sub_added = false;
  }
  if (g_publisher_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_joint_feedback_pub, node);
    (void)rc;
    g_publisher_up = false;
  }
  if (g_enable_sub_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_motor_enable_sub, node);
    (void)rc;
    g_enable_sub_up = false;
  }
  if (g_joint_sub_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_joint_state_sub, node);
    (void)rc;
    g_joint_sub_up = false;
  }
}

void motor_bridge_poll(void) {
  if (!g_ready) {
    return;
  }

  size_t output_count = 0;
  for (size_t index = 0; index < kJointCommandMaxMotors; ++index) {
    control_joint_state_t state = {};
    if (!control_task_get_joint_state(index, &state)) {
      continue;
    }
    rosidl_runtime_c__String *name = &g_joint_fb_name_strs[output_count];
    size_t name_size = strlen(state.joint_name);
    if (name_size >= name->capacity) {
      continue;
    }
    memcpy(name->data, state.joint_name, name_size + 1U);
    name->size = name_size;
    g_joint_fb_position_buf[output_count] = state.position;
    g_joint_fb_velocity_buf[output_count] = state.velocity;
    g_joint_fb_effort_buf[output_count] = state.effort;
    ++output_count;
  }
  if (output_count == 0U) {
    return;
  }

  g_joint_feedback_msg.name.size = output_count;
  g_joint_feedback_msg.position.size = output_count;
  g_joint_feedback_msg.velocity.size = output_count;
  g_joint_feedback_msg.effort.size = output_count;
  // 时间戳：epoch 优先，未同步回退 millis
  fill_header_stamp(&g_joint_feedback_msg.header.stamp);
  rcl_ret_t rc =
      rcl_publish(&g_joint_feedback_pub, &g_joint_feedback_msg, nullptr);
  ++g_joint_feedback_pub_count;
  g_joint_feedback_pub_rc = (int32_t)rc;
  (void)rc;
}

#else

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  (void)node;
  (void)executor;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  (void)node;
  (void)executor;
}

void motor_bridge_poll(void) {}

#endif  // ENABLE_MOTOR_COMMAND_BRIDGE
