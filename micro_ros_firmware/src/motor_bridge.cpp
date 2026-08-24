// 电机域 ROS 桥接实现。
//
// 职责：订阅 /joint_commands（sensor_msgs/JointState）展开为
// control_motor_command() 下发；周期消费 FDCAN1 反馈，经 control_task
// 解码后发布 micro_ros/JointState。协议编解码与电机表在 control_task，
// 本模块不接触 can_bsp 内部缓冲。

#include "motor_bridge.h"

#include <Arduino.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

#include "can_bsp.h"
#include "control_task.h"

#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE

static const char *kJointCommandsTopic = "/joint_commands";    // 单点控制命令
static const char *kJointFeedbackTopic = "micro_ros/JointState";  // 关节反馈
static const size_t kJointCommandMaxMotors = 2;  // 当前默认两台测试电机

static rcl_subscription_t g_joint_state_sub;
static rcl_publisher_t g_joint_feedback_pub;
static sensor_msgs__msg__JointState g_joint_state_msg;
static sensor_msgs__msg__JointState g_joint_feedback_msg;

// micro-ROS 不会为 NULL 序列/字符串分配空间：全部静态预分配
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

// 调试计数（GDB 可读）
volatile uint32_t g_joint_feedback_decode_count = 0;
volatile uint32_t g_joint_feedback_decode_ok = 0;
volatile uint32_t g_joint_feedback_pub_count = 0;
volatile int32_t g_joint_feedback_pub_rc = 0;

static uint32_t g_last_rx_seq = 0;
static bool g_ready = false;
static bool g_sub_added = false;

// ROS 侧 JointState -> 默认测试电机（RP/vaxor）协议帧
static void joint_state_sub_cb(const void *msg) {
  const sensor_msgs__msg__JointState *state =
      (const sensor_msgs__msg__JointState *)msg;
  size_t count = state->name.size;
  if (count > kJointCommandMaxMotors) {
    count = kJointCommandMaxMotors;
  }
  for (size_t i = 0; i < count; ++i) {
    control_motor_command_t out = {};
    const rosidl_runtime_c__String *name = &state->name.data[i];
    size_t len = name->size;
    if (len > sizeof(out.joint_name) - 1U) {
      len = sizeof(out.joint_name) - 1U;
    }
    memcpy(out.joint_name, name->data, len);
    out.joint_name[len] = '\0';
    // vaxor 模式严格校验：恰好只有 position 或恰好只有 velocity 才切模式；
    // 两者同发、仅 effort 或均未发一律保持 MIT。
    const bool has_position = state->position.size > i;
    const bool has_velocity = state->velocity.size > i;
    out.mode = CONTROL_MOTOR_MODE_MIT;
    if (has_position && !has_velocity) {
      out.mode = CONTROL_MOTOR_MODE_POSITION;
    } else if (has_velocity && !has_position) {
      out.mode = CONTROL_MOTOR_MODE_VELOCITY;
    }
    if (has_position) {
      out.position = state->position.data[i];
    }
    if (has_velocity) {
      out.velocity = state->velocity.data[i];
    }
    if (state->effort.size > i) {
      out.effort = state->effort.data[i];
    }
    control_motor_command(&out);
  }
}

// 找到已登记关节或登记新关节；满 2 台后返回 -1
static int find_or_add_feedback_joint(const char *joint_name) {
  for (size_t i = 0; i < kJointCommandMaxMotors; ++i) {
    if (g_joint_fb_name_strs[i].size > 0U &&
        strcmp(g_joint_fb_name_strs[i].data, joint_name) == 0) {
      return (int)i;
    }
  }
  for (size_t i = 0; i < kJointCommandMaxMotors; ++i) {
    if (g_joint_fb_name_strs[i].size == 0U) {
      size_t len = strlen(joint_name);
      if (len >= g_joint_fb_name_strs[i].capacity) {
        len = g_joint_fb_name_strs[i].capacity - 1U;
      }
      memcpy(g_joint_fb_name_strs[i].data, joint_name, len);
      g_joint_fb_name_strs[i].data[len] = '\0';
      g_joint_fb_name_strs[i].size = len;
      g_joint_feedback_msg.name.size = i + 1U;
      g_joint_feedback_msg.position.size = i + 1U;
      g_joint_feedback_msg.velocity.size = i + 1U;
      g_joint_feedback_msg.effort.size = i + 1U;
      return (int)i;
    }
  }
  return -1;
}

// 尝试把 FDCAN1 新帧解析为默认电机反馈并发布 micro_ros/JointState
static void publish_joint_feedback(uint32_t id, const uint8_t *data,
                                   size_t size) {
  g_joint_feedback_decode_count++;
  control_joint_state_t state = {};
  if (!control_decode_feedback(id, data, size, &state)) {
    return;
  }
  g_joint_feedback_decode_ok++;
  int index = find_or_add_feedback_joint(state.joint_name);
  if (index < 0) {
    return;
  }
  g_joint_fb_position_buf[index] = state.position;
  g_joint_fb_velocity_buf[index] = state.velocity;
  g_joint_fb_effort_buf[index] = state.effort;

  uint32_t ms = millis();
  g_joint_feedback_msg.header.stamp.sec = (int32_t)(ms / 1000u);
  g_joint_feedback_msg.header.stamp.nanosec =
      (uint32_t)((ms % 1000u) * 1000000u);
  g_joint_feedback_msg.header.frame_id.data = (char *)"joints";
  g_joint_feedback_msg.header.frame_id.size = 6;
  g_joint_feedback_msg.header.frame_id.capacity = 7;
  rcl_ret_t rc = rcl_publish(&g_joint_feedback_pub, &g_joint_feedback_msg,
                             nullptr);
  g_joint_feedback_pub_count++;
  g_joint_feedback_pub_rc = (int32_t)rc;
  (void)rc;
}

static void setup_command_msg_buffers() {
  g_joint_state_msg.name.data = g_joint_name_strs;
  g_joint_state_msg.name.size = 0;
  g_joint_state_msg.name.capacity = kJointCommandMaxMotors;
  for (size_t i = 0; i < kJointCommandMaxMotors; ++i) {
    g_joint_name_strs[i].data = g_joint_name_buf[i];
    g_joint_name_strs[i].size = 0;
    g_joint_name_strs[i].capacity = sizeof(g_joint_name_buf[i]);
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
  for (size_t i = 0; i < kJointCommandMaxMotors; ++i) {
    g_joint_fb_name_strs[i].data = g_joint_fb_name_buf[i];
    g_joint_fb_name_strs[i].size = 0;
    g_joint_fb_name_strs[i].capacity = sizeof(g_joint_fb_name_buf[i]);
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
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (rclc_subscription_init_default(
          &g_joint_state_sub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointCommandsTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_default(
          &g_joint_feedback_pub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointFeedbackTopic) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_subscription_fini(&g_joint_state_sub, node);
    (void)rc;
    return false;
  }
  setup_command_msg_buffers();
  setup_feedback_msg_buffers();

  if (rclc_executor_add_subscription(executor, &g_joint_state_sub,
                                     &g_joint_state_msg, joint_state_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_publisher_fini(&g_joint_feedback_pub, node);
    rc = rcl_subscription_fini(&g_joint_state_sub, node);
    (void)rc;
    return false;
  }
  g_sub_added = true;
  g_ready = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (!g_ready && !g_sub_added) {
    return;
  }
  g_ready = false;
  if (g_sub_added) {
    rcl_ret_t rc = rclc_executor_remove_subscription(executor, &g_joint_state_sub);
    (void)rc;
    g_sub_added = false;
  }
  rcl_ret_t rc = rcl_publisher_fini(&g_joint_feedback_pub, node);
  rc = rcl_subscription_fini(&g_joint_state_sub, node);
  (void)rc;
}

void motor_bridge_poll(void) {
  if (!g_ready) {
    return;
  }
  can_rx_frame_t frame = {0};
  uint32_t seq = can1_rx_snapshot(&frame);
  if (seq == g_last_rx_seq) {
    return;
  }
  g_last_rx_seq = seq;
  publish_joint_feedback(frame.id, frame.data, frame.dlc);
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
