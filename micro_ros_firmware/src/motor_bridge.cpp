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

// ---------------------------------------------------------------------------
// 参数回调：解析电机拓扑参数并触发电机表重建
// ---------------------------------------------------------------------------
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE

#include <stdlib.h>
#include <string.h>

#include "motor_control.h"

// 临时配置缓冲区（参数收集完成后一次性应用）
static motor_control_motor_config_t g_pending_motors[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_pending_motor_count = 0;
static bool g_config_pending = false;

// 参数解析辅助函数
static bool param_get_int(const Parameter *param, int64_t *value) {
  if (param == NULL || value == NULL) return false;
  if (param->value.type != RCLC_PARAMETER_INT) return false;
  *value = param->value.integer_value;
  return true;
}

static bool param_get_double(const Parameter *param, double *value) {
  if (param == NULL || value == NULL) return false;
  if (param->value.type != RCLC_PARAMETER_DOUBLE) return false;
  *value = param->value.double_value;
  return true;
}

static bool param_get_string(const Parameter *param, const char **value) {
  if (param == NULL || value == NULL) return false;
  if (param->value.type != RCLC_PARAMETER_NOT_SET &&
      param->value.string_value.data != NULL) {
    *value = param->value.string_value.data;
    return true;
  }
  return false;
}

// 总线名到索引的映射（由参数 bus_mapping_{name} 动态设置）
static uint8_t g_bus_name_to_index[8][16];  // 最多 8 条总线，每条总线名最长 15 字符
static uint8_t g_bus_name_to_index_count = 0;

static uint8_t find_bus_index(const char *bus_name) {
  if (bus_name == NULL) return 0;
  for (uint8_t i = 0; i < g_bus_name_to_index_count; ++i) {
    if (strcmp((const char *)g_bus_name_to_index[i], bus_name) == 0) {
      return i;
    }
  }
  return 0;  // 默认返回总线 0
}

static void register_bus_mapping(const char *bus_name, uint8_t bus_index) {
  if (bus_name == NULL || g_bus_name_to_index_count >= 8) return;
  // 检查是否已存在
  for (uint8_t i = 0; i < g_bus_name_to_index_count; ++i) {
    if (strcmp((const char *)g_bus_name_to_index[i], bus_name) == 0) {
      return;  // 已存在
    }
  }
  // 注册新映射
  size_t name_len = strlen(bus_name);
  if (name_len >= 16) name_len = 15;
  memcpy(g_bus_name_to_index[g_bus_name_to_index_count], bus_name, name_len);
  g_bus_name_to_index[g_bus_name_to_index_count][name_len] = '\0';
  g_bus_name_to_index_count++;
}

// 协议名到枚举的映射
static motor_control_protocol_t parse_protocol(const char *protocol_str) {
  if (protocol_str == NULL) return MOTOR_CONTROL_RP;
  if (strcmp(protocol_str, "vaxor") == 0 || strcmp(protocol_str, "VAXOR") == 0) {
    return MOTOR_CONTROL_VAXOR;
  }
  return MOTOR_CONTROL_RP;  // 默认 RP
}

bool motor_bridge_param_callback(const Parameter *old_param,
                                 const Parameter *new_param,
                                 void *context) {
  (void)old_param;
  (void)context;

  if (new_param == NULL) return false;

  // 处理 motor_count 参数
  if (strcmp(new_param->name.data, "motor_count") == 0) {
    int64_t count = 0;
    if (param_get_int(new_param, &count) && count > 0 && count <= MOTOR_CONTROL_MAX_MOTORS) {
      g_pending_motor_count = (size_t)count;
      g_config_pending = true;
      return true;
    }
    return false;
  }

  // 处理 bus_mapping_{name} 参数
  if (strncmp(new_param->name.data, "bus_mapping_", 12) == 0) {
    const char *bus_name = new_param->name.data + 12;
    int64_t bus_index = 0;
    if (param_get_int(new_param, &bus_index) && bus_index >= 0 && bus_index < 8) {
      register_bus_mapping(bus_name, (uint8_t)bus_index);
      return true;
    }
    return false;
  }

  // 处理电机参数 motor_{i}_{field}
  if (strncmp(new_param->name.data, "motor_", 6) == 0) {
    const char *rest = new_param->name.data + 6;

    // 解析索引
    char *end_ptr = NULL;
    long index = strtol(rest, &end_ptr, 10);
    if (end_ptr == rest || index < 0 || (size_t)index >= MOTOR_CONTROL_MAX_MOTORS) {
      return false;
    }

    // 确保 pending 数组足够大
    if ((size_t)index >= g_pending_motor_count) {
      g_pending_motor_count = (size_t)index + 1;
    }

    motor_control_motor_config_t *motor = &g_pending_motors[index];
    const char *field = end_ptr + 1;  // 跳过下划线

    if (strcmp(field, "joint_name") == 0) {
      const char *value = NULL;
      if (param_get_string(new_param, &value)) {
        size_t len = strlen(value);
        if (len >= MOTOR_CONTROL_JOINT_NAME_CAPACITY) len = MOTOR_CONTROL_JOINT_NAME_CAPACITY - 1;
        memcpy((void *)motor->joint_name, (const void *)value, len);
        ((char *)motor->joint_name)[len] = '\0';
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "protocol") == 0) {
      const char *value = NULL;
      if (param_get_string(new_param, &value)) {
        motor->protocol = parse_protocol(value);
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "id") == 0) {
      int64_t value = 0;
      if (param_get_int(new_param, &value) && value >= 1 && value <= 60) {
        motor->id = (uint8_t)value;
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "bus") == 0) {
      const char *value = NULL;
      if (param_get_string(new_param, &value)) {
        motor->bus_index = find_bus_index(value);
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "gear_ratio") == 0) {
      double value = 0;
      if (param_get_double(new_param, &value) && value > 0) {
        motor->gear_ratio = value;
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "gear_efficiency") == 0) {
      double value = 0;
      if (param_get_double(new_param, &value) && value > 0 && value <= 1.0) {
        motor->gear_efficiency = value;
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "torque_constant") == 0) {
      double value = 0;
      if (param_get_double(new_param, &value) && value >= 0) {
        motor->torque_constant_nm_per_amp = value;
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "default_kp") == 0) {
      double value = 0;
      if (param_get_double(new_param, &value) && value >= 0) {
        motor->default_kp = value;
        g_config_pending = true;
        return true;
      }
    } else if (strcmp(field, "default_kd") == 0) {
      double value = 0;
      if (param_get_double(new_param, &value) && value >= 0) {
        motor->default_kd = value;
        g_config_pending = true;
        return true;
      }
    }
  }

  return false;  // 未知参数，拒绝
}

// 应用挂起的配置（由 ros_bridge_poll 调用）
void motor_bridge_apply_pending_config(void) {
  if (!g_config_pending || g_pending_motor_count == 0) return;

  // 检查所有电机配置是否完整
  for (size_t i = 0; i < g_pending_motor_count; ++i) {
    motor_control_motor_config_t *motor = &g_pending_motors[i];
    if (motor->joint_name[0] == '\0') return;  // 关节名为空，配置不完整
    if (motor->id == 0) return;  // ID 无效
  }

  // 调用 control_task 重建电机表
  if (control_task_reconfigure(g_pending_motors, g_pending_motor_count)) {
    g_config_pending = false;
    g_pending_motor_count = 0;
  }
}

#else

// 禁用电机桥接时的空实现
bool motor_bridge_param_callback(const Parameter *old_param,
                                 const Parameter *new_param,
                                 void *context) {
  (void)old_param;
  (void)new_param;
  (void)context;
  return false;
}

void motor_bridge_apply_pending_config(void) {}

#endif  // ENABLE_MOTOR_COMMAND_BRIDGE
