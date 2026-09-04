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
#include <can_bridge_msgs/msg/motor_gains.h>
#include <can_bridge_msgs/msg/motor_mode.h>
#include <can_bridge_msgs/msg/motor_status.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>

#include "control_task.h"
#include "time_utils.h"

// 电机域话题分工：控制目标、运动授权、模式和增益分别输入，反馈与诊断分别输出。
// 控制目标不携带模式或增益，避免普通控制消息隐式改变电机配置。
static const char *kJointCommandsTopic = "/joint_commands";
static const char *kJointFeedbackTopic = "micro_ros/JointState";
static const char *kMotorStatusTopic = "micro_ros/motor_status";
static const char *kMotorEnableTopic = "micro_ros/motor_enable";
static const char *kMotorModeTopic = "micro_ros/motor_mode";
static const char *kMotorGainsTopic = "micro_ros/motor_gains";
static const size_t kJointCommandMaxMotors = 4;  // 当前默认四台测试电机

// rcl 实体和对应消息必须具有静态生命周期，不能使用回调栈上的临时对象。
static rcl_subscription_t g_joint_state_sub;
static rcl_subscription_t g_motor_enable_sub;
static rcl_subscription_t g_motor_mode_sub;
static rcl_subscription_t g_motor_gains_sub;
static rcl_publisher_t g_joint_feedback_pub;
static rcl_publisher_t g_motor_status_pub;
static sensor_msgs__msg__JointState g_joint_state_msg;
static sensor_msgs__msg__JointState g_joint_feedback_msg;
static std_msgs__msg__Bool g_motor_enable_msg;
static can_bridge_msgs__msg__MotorMode g_motor_mode_msg;
static can_bridge_msgs__msg__MotorGains g_motor_gains_msg;
static can_bridge_msgs__msg__MotorStatus g_motor_status_msg;

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
static char g_motor_status_name_buf[32];
static char g_motor_mode_name_buf[32];
static char g_motor_gains_name_buf[32];
static uint32_t g_motor_status_published_sequence[kJointCommandMaxMotors];

// 调试计数（GDB 可读）。
volatile uint32_t g_joint_feedback_pub_count = 0;
volatile int32_t g_joint_feedback_pub_rc = 0;
volatile uint32_t g_motor_status_pub_count = 0;
volatile int32_t g_motor_status_pub_rc = 0;

// 分阶段记录创建结果：建链中途失败或 agent 断连时，只销毁已经成功初始化、
// 已经加入 executor 的实体，避免对未初始化的 rcl 对象调用 fini/remove。
static bool g_ready = false;
static bool g_joint_sub_up = false;
static bool g_enable_sub_up = false;
static bool g_mode_sub_up = false;
static bool g_gains_sub_up = false;
static bool g_joint_publisher_up = false;
static bool g_status_publisher_up = false;
static bool g_joint_sub_added = false;
static bool g_enable_sub_added = false;
static bool g_mode_sub_added = false;
static bool g_gains_sub_added = false;

static bool copy_request_joint_name(const rosidl_runtime_c__String *source,
                                    char *destination,
                                    size_t destination_capacity) {
  // ROS 字符串必须同时满足 size/capacity 约束；内部请求固定使用 C 字符串。
  if (source == nullptr || source->data == nullptr || source->size == 0U ||
      source->size >= source->capacity ||
      source->size >= destination_capacity) {
    return false;
  }
  memcpy(destination, source->data, source->size);
  destination[source->size] = '\0';
  return true;
}

static void joint_state_sub_cb(const void *msg) {
  const sensor_msgs__msg__JointState *state =
      (const sensor_msgs__msg__JointState *)msg;
  if (state == nullptr || state->name.data == nullptr) {
    return;
  }

  // name 决定命令数量；异常 size 先收紧到实际 capacity 和固件固定容量。
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

    // JointState 的三个数值数组允许省略；缺失字段保持零值。每个已提供字段都要
    // 同时检查 data、size 和 capacity，防止畸形序列导致越界读取。
    const bool has_position =
        state->position.data != nullptr && state->position.size > index &&
        state->position.capacity > index;
    const bool has_velocity =
        state->velocity.data != nullptr && state->velocity.size > index &&
        state->velocity.capacity > index;
    const bool has_effort =
        state->effort.data != nullptr && state->effort.size > index &&
        state->effort.capacity > index;
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
    // 回调只写 latest-wins mailbox；FDCAN 发送由 control task 独占完成。
    (void)control_task_submit_command(&command, received_ms);
  }
}

static void motor_enable_sub_cb(const void *msg) {
  const std_msgs__msg__Bool *enabled = (const std_msgs__msg__Bool *)msg;
  if (enabled != nullptr) {
    control_task_set_authorization(enabled->data);
  }
}

static void motor_mode_sub_cb(const void *msg) {
  const can_bridge_msgs__msg__MotorMode *mode =
      (const can_bridge_msgs__msg__MotorMode *)msg;
  motor_control_mode_request_t request = {};
  if (mode == nullptr ||
      !copy_request_joint_name(&mode->joint_name, request.joint_name,
                               sizeof(request.joint_name))) {
    return;
  }
  // 协议支持范围由 control mailbox 再按关节类型校验；模式配置会建立一个
  // 控制周期屏障，切换完成前不会消费新的运动目标。
  request.mode = mode->mode;
  (void)control_task_submit_mode(&request);
}

static void motor_gains_sub_cb(const void *msg) {
  const can_bridge_msgs__msg__MotorGains *gains =
      (const can_bridge_msgs__msg__MotorGains *)msg;
  motor_control_gains_request_t request = {};
  if (gains == nullptr || !isfinite(gains->kp) || !isfinite(gains->kd) ||
      !copy_request_joint_name(&gains->joint_name, request.joint_name,
                               sizeof(request.joint_name))) {
    return;
  }
  // 此处先拒绝 NaN/Inf；非负和协议上限由 control mailbox 统一校验。
  // 新增益只会被随后接收的控制命令复制，不会自行刷新命令或触发运动。
  request.kp = gains->kp;
  request.kd = gains->kd;
  (void)control_task_submit_gains(&request);
}

static void setup_command_msg_buffers() {
  // 订阅消息的序列存储由应用提供；executor 反序列化时只写这些固定缓冲区。
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
  // 发布消息复用同一组固定缓冲区，每次 poll 仅调整有效元素的 size。
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

static void setup_status_msg_buffers() {
  // sequence 用于区分新状态快照，重连后清零以重新发布当前有效状态。
  memset(g_motor_status_published_sequence, 0,
         sizeof(g_motor_status_published_sequence));
  g_motor_status_msg.joint_name.data = g_motor_status_name_buf;
  g_motor_status_msg.joint_name.size = 0U;
  g_motor_status_msg.joint_name.capacity = sizeof(g_motor_status_name_buf);
  g_motor_status_msg.header.frame_id.data = (char *)"joints";
  g_motor_status_msg.header.frame_id.size = 6U;
  g_motor_status_msg.header.frame_id.capacity = 7U;
}

static void setup_configuration_msg_buffers() {
  // 两个配置话题每次只承载一个关节名，各使用独立缓冲区避免互相覆盖。
  g_motor_mode_msg.joint_name.data = g_motor_mode_name_buf;
  g_motor_mode_msg.joint_name.size = 0U;
  g_motor_mode_msg.joint_name.capacity = sizeof(g_motor_mode_name_buf);
  g_motor_gains_msg.joint_name.data = g_motor_gains_name_buf;
  g_motor_gains_msg.joint_name.size = 0U;
  g_motor_gains_msg.joint_name.capacity = sizeof(g_motor_gains_name_buf);
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  // 先初始化所有 subscription/publisher，再绑定静态消息并加入 executor。
  // 任一步失败都走统一 destroy，保证下次 agent 重连可以重新创建。
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

  if (rclc_subscription_init_default(
          &g_motor_mode_sub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorMode),
          kMotorModeTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_mode_sub_up = true;

  if (rclc_subscription_init_default(
          &g_motor_gains_sub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorGains),
          kMotorGainsTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_gains_sub_up = true;

  if (rclc_publisher_init_default(
          &g_joint_feedback_pub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointFeedbackTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_joint_publisher_up = true;

  if (rclc_publisher_init_default(
          &g_motor_status_pub, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorStatus),
          kMotorStatusTopic) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_status_publisher_up = true;
  setup_command_msg_buffers();
  setup_feedback_msg_buffers();
  setup_status_msg_buffers();
  setup_configuration_msg_buffers();
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
  if (rclc_executor_add_subscription(executor, &g_motor_mode_sub,
                                     &g_motor_mode_msg, motor_mode_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_mode_sub_added = true;
  if (rclc_executor_add_subscription(executor, &g_motor_gains_sub,
                                     &g_motor_gains_msg, motor_gains_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_gains_sub_added = true;
  // 只有所有实体和 executor handle 均就绪后，定时发布路径才允许运行。
  g_ready = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  // 先阻止 poll 使用实体，再按“executor handle → rcl 实体”的顺序释放。
  g_ready = false;
  if (g_gains_sub_added) {
    rcl_ret_t rc =
        rclc_executor_remove_subscription(executor, &g_motor_gains_sub);
    (void)rc;
    g_gains_sub_added = false;
  }
  if (g_mode_sub_added) {
    rcl_ret_t rc =
        rclc_executor_remove_subscription(executor, &g_motor_mode_sub);
    (void)rc;
    g_mode_sub_added = false;
  }
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
  if (g_status_publisher_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_motor_status_pub, node);
    (void)rc;
    g_status_publisher_up = false;
  }
  if (g_joint_publisher_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_joint_feedback_pub, node);
    (void)rc;
    g_joint_publisher_up = false;
  }
  if (g_enable_sub_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_motor_enable_sub, node);
    (void)rc;
    g_enable_sub_up = false;
  }
  if (g_gains_sub_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_motor_gains_sub, node);
    (void)rc;
    g_gains_sub_up = false;
  }
  if (g_mode_sub_up) {
    rcl_ret_t rc = rcl_subscription_fini(&g_motor_mode_sub, node);
    (void)rc;
    g_mode_sub_up = false;
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

  // 将 control task 中已经收到反馈的关节压紧写入 JointState；尚无反馈的关节
  // 不占序列元素，确保 name/position/velocity/effort 始终等长。
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
  // 没有有效关节反馈时仅跳过本次 JointState 发布，不发布空消息。
  rcl_ret_t rc = RCL_RET_OK;
  if (output_count > 0U) {
    g_joint_feedback_msg.name.size = output_count;
    g_joint_feedback_msg.position.size = output_count;
    g_joint_feedback_msg.velocity.size = output_count;
    g_joint_feedback_msg.effort.size = output_count;
    // 时间戳：epoch 优先，未同步回退 millis
    fill_header_stamp(&g_joint_feedback_msg.header.stamp);
    rc = rcl_publish(&g_joint_feedback_pub, &g_joint_feedback_msg, nullptr);
    ++g_joint_feedback_pub_count;
    g_joint_feedback_pub_rc = (int32_t)rc;
  }

  // MotorStatus 与 JointState 独立处理，不依赖 JointState 是否有效；每台已收到
  // 20 字节状态帧的电机发布一条消息。
  for (size_t index = 0; index < kJointCommandMaxMotors; ++index) {
    control_motor_status_t status = {};
    if (!control_task_get_motor_status(index, &status)) {
      continue;
    }
    // 只发布尚未成功发送过的新快照；失败时不推进序号，下次 poll 自动重试。
    if (status.sequence == g_motor_status_published_sequence[index]) {
      continue;
    }
    size_t name_size = strlen(status.joint_name);
    if (name_size >= g_motor_status_msg.joint_name.capacity) {
      continue;
    }
    memcpy(g_motor_status_msg.joint_name.data, status.joint_name,
           name_size + 1U);
    g_motor_status_msg.joint_name.size = name_size;
    g_motor_status_msg.voltage = status.voltage;
    g_motor_status_msg.fault = status.fault;
    g_motor_status_msg.motor_state = status.motor_state;
    g_motor_status_msg.calibration_state = status.calibration_state;
    g_motor_status_msg.mode = status.mode;
    g_motor_status_msg.motor_type = status.motor_type;
    fill_header_stamp(&g_motor_status_msg.header.stamp);
    rc = rcl_publish(&g_motor_status_pub, &g_motor_status_msg, nullptr);
    ++g_motor_status_pub_count;
    g_motor_status_pub_rc = (int32_t)rc;
    if (rc == RCL_RET_OK) {
      g_motor_status_published_sequence[index] = status.sequence;
    }
  }
}
