// micro-ROS 桥接层（IMU/温度发布）
//
// 职责：node + executor 的生命周期管理，定时器读取 sensor_task 更新的最新样本，
// 分别发布 sensor_msgs/Imu（imu/data）与 std_msgs/Float32（imu/temp）。
//
// 注释规范：中文写意图，micro-ROS / ROS 2 API 保留英文。

#include "ros_bridge.h"
#include "control_task.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
#include "can_bsp.h"
#include <can_bridge_msgs/msg/can_frame.h>
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
#include <sensor_msgs/msg/joint_state.h>
#endif

// ---------------------------------------------------------------------------
// 配置（单点可改，节点名/话题只在这里改）
// ---------------------------------------------------------------------------
static const char *kNodeName = "micro_ros_node";
static const char *kImuTopic = "micro_ros/imu/data";
static const char *kTempTopic = "micro_ros/imu/temp";
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
static const char *kCan1SendTopic = "micro_ros/can1/send";
static const char *kCan1ReceiveTopic = "micro_ros/can1/receive";
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
static const char *kJointCommandsTopic = "/joint_commands";  // 单点控制命令
static const char *kJointFeedbackTopic = "micro_ros/JointState";  // 关节反馈
#endif

// micro-ROS 实体（agent 断连后整体重建）
static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_publisher_t g_imu_pub;
static rcl_publisher_t g_temp_pub;
static std_msgs__msg__Float32 g_temp_msg;
static sensor_msgs__msg__Imu g_imu_msg;
static rcl_timer_t g_timer;
static rcl_allocator_t g_allocator;
static const imu_data_t *g_imu_source = nullptr;  // 业务任务写入的最新样本
static bool g_entities_up = false;

#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
static rcl_publisher_t g_can1_rx_pub;
static rcl_subscription_t g_can1_tx_sub;
static can_bridge_msgs__msg__CANFrame g_can1_rx_msg;
static can_bridge_msgs__msg__CANFrame g_can1_tx_msg;
static char g_can1_tx_frame_id_buf[8];  // 订阅反序列化需要预分配字符串
static uint32_t g_last_can1_rx_counter = 0;
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
static const size_t kJointCommandMaxMotors = 2;  // 当前默认两台测试电机
static rcl_subscription_t g_joint_state_sub;
static rcl_publisher_t g_joint_feedback_pub;
static sensor_msgs__msg__JointState g_joint_state_msg;
// micro-ROS 不会为 NULL 序列/字符串分配空间：全部静态预分配
static rosidl_runtime_c__String g_joint_name_strs[kJointCommandMaxMotors];
static char g_joint_name_buf[kJointCommandMaxMotors][32];
static double g_joint_position_buf[kJointCommandMaxMotors];
static double g_joint_velocity_buf[kJointCommandMaxMotors];
static double g_joint_effort_buf[kJointCommandMaxMotors];
static sensor_msgs__msg__JointState g_joint_feedback_msg;
static rosidl_runtime_c__String g_joint_fb_name_strs[kJointCommandMaxMotors];
static char g_joint_fb_name_buf[kJointCommandMaxMotors][32];
static double g_joint_fb_position_buf[kJointCommandMaxMotors];
static double g_joint_fb_velocity_buf[kJointCommandMaxMotors];
static double g_joint_fb_effort_buf[kJointCommandMaxMotors];
volatile uint32_t g_joint_feedback_decode_count = 0;
volatile uint32_t g_joint_feedback_decode_ok = 0;
volatile uint32_t g_joint_feedback_pub_count = 0;
volatile int32_t g_joint_feedback_pub_rc = 0;
#endif

static void fill_imu_msg(const imu_data_t *data) {
  // 时间戳（sensor_msgs 惯例，sec + nanosec）
  uint32_t ms = millis();
  g_imu_msg.header.stamp.sec = (int32_t)(ms / 1000u);
  g_imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000u) * 1000000u);
  g_imu_msg.header.frame_id.data = (char *)"imu_link";
  g_imu_msg.header.frame_id.size = 8;
  g_imu_msg.header.frame_id.capacity = 9;

  // 四元数（w,x,y,z → x,y,z,w）
  g_imu_msg.orientation.x = data->quat[1];
  g_imu_msg.orientation.y = data->quat[2];
  g_imu_msg.orientation.z = data->quat[3];
  g_imu_msg.orientation.w = data->quat[0];
  g_imu_msg.angular_velocity.x = data->gyro[0];
  g_imu_msg.angular_velocity.y = data->gyro[1];
  g_imu_msg.angular_velocity.z = data->gyro[2];
  g_imu_msg.linear_acceleration.x = data->accel[0];
  g_imu_msg.linear_acceleration.y = data->accel[1];
  g_imu_msg.linear_acceleration.z = data->accel[2];

  // 协方差全 0：表示未知/未标定，符合 sensor_msgs 惯例
  memset(g_imu_msg.orientation_covariance, 0,
         sizeof(g_imu_msg.orientation_covariance));
  memset(g_imu_msg.angular_velocity_covariance, 0,
         sizeof(g_imu_msg.angular_velocity_covariance));
  memset(g_imu_msg.linear_acceleration_covariance, 0,
         sizeof(g_imu_msg.linear_acceleration_covariance));

  g_temp_msg.data = data->temp;
}

#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
static void fill_can1_rx_msg() {
  // 时间戳沿用 IMU 的 millis 方案；frame_id 标记来源总线
  uint32_t ms = millis();
  g_can1_rx_msg.header.stamp.sec = (int32_t)(ms / 1000u);
  g_can1_rx_msg.header.stamp.nanosec = (uint32_t)((ms % 1000u) * 1000000u);
  g_can1_rx_msg.header.frame_id.data = (char *)"can1";
  g_can1_rx_msg.header.frame_id.size = 4;
  g_can1_rx_msg.header.frame_id.capacity = 5;

  g_can1_rx_msg.id = (uint16_t)can1_rx_meta.id;
  g_can1_rx_msg.dlc = can1_rx_meta.dlc;
  memset(g_can1_rx_msg.data, 0, sizeof(g_can1_rx_msg.data));
  memcpy(g_can1_rx_msg.data, rx_data1, can1_rx_meta.dlc);
  g_can1_rx_msg.is_extended = can1_rx_meta.is_extended;
  g_can1_rx_msg.is_fd = can1_rx_meta.is_fd;
}

// 收到 ROS 侧 CANFrame 后写入 FDCAN1 TX FIFO（micro-ROS 任务上下文，可阻塞）
static void can1_tx_sub_cb(const void *msg) {
  const can_bridge_msgs__msg__CANFrame *frame =
      (const can_bridge_msgs__msg__CANFrame *)msg;
  fdcanx_send_frame(&hfdcan1, frame->id, frame->data, frame->dlc,
                    frame->is_extended, frame->is_fd);
}
#endif

#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
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
    // vaxor 模式由字段决定：position -> 位置，velocity -> 速度，effort -> MIT
    out.mode = CONTROL_MOTOR_MODE_MIT;
    if (state->position.size > i) {
      out.position = state->position.data[i];
      out.mode = CONTROL_MOTOR_MODE_POSITION;
    }
    if (state->velocity.size > i) {
      out.velocity = state->velocity.data[i];
      if (state->position.size <= i) {
        out.mode = CONTROL_MOTOR_MODE_VELOCITY;
      }
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
#endif

// 定时器回调：周期发布最新 IMU 样本；同时服务 XRCE 会话（保活）
static void imu_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  if (g_imu_source != nullptr) {
    ros_bridge_publish_imu(g_imu_source);
  }
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
  if (can1_rx_counter != g_last_can1_rx_counter) {
    g_last_can1_rx_counter = can1_rx_counter;
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
    // 先尝试按默认电机协议解析并发布关节反馈
    publish_joint_feedback(can1_rx_meta.id, rx_data1, can1_rx_meta.dlc);
#endif
    fill_can1_rx_msg();
    rcl_ret_t rc = rcl_publish(&g_can1_rx_pub, &g_can1_rx_msg, nullptr);
    (void)rc;
  }
#endif
}

// ---------------------------------------------------------------------------
// 实体生命周期（agent 断连后可重建，全部可重入）
// ---------------------------------------------------------------------------
static bool create_entities() {
  g_entities_up = false;
  // rclc_support/executor 保存 allocator 指针而非拷贝，必须使用静态存储，
  // 不能放在 create_entities 的栈上（函数返回后即悬空，spin 时触发 HardFault）。
  g_allocator = rcl_get_default_allocator();

  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_node_init_default(&g_node, kNodeName, "", &g_support) != RCL_RET_OK) {
    return false;
  }

  // IMU + 温度话题（publisher 不占用 executor 句柄）
  if (rclc_publisher_init_default(&g_imu_pub, &g_node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                  kImuTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_default(&g_temp_pub, &g_node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                  kTempTopic) != RCL_RET_OK) {
    return false;
  }
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
  if (rclc_publisher_init_default(&g_can1_rx_pub, &g_node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, CANFrame),
                                  kCan1ReceiveTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(&g_can1_tx_sub, &g_node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, CANFrame),
                                     kCan1SendTopic) != RCL_RET_OK) {
    return false;
  }
  // micro-ROS 不会为 NULL 字符串分配空间：订阅缓冲须预分配
  g_can1_tx_msg.header.frame_id.data = g_can1_tx_frame_id_buf;
  g_can1_tx_msg.header.frame_id.size = 0;
  g_can1_tx_msg.header.frame_id.capacity = sizeof(g_can1_tx_frame_id_buf);
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
  if (rclc_subscription_init_default(
          &g_joint_state_sub, &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointCommandsTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_default(
          &g_joint_feedback_pub, &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          kJointFeedbackTopic) != RCL_RET_OK) {
    return false;
  }
  // micro-ROS 不会为 NULL 序列/字符串分配空间：订阅缓冲须预分配
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
#endif

  // 周期定时器：发布 IMU 并维持微-ROS 会话（推荐的最小发布模型）
  if (rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(20),
                              imu_timer_cb) != RCL_RET_OK) {
    return false;
  }

  // 3 个句柄：定时器 + CAN1 发送订阅 + JointState 订阅（后两者宏控制）
  if (rclc_executor_init(&g_executor, &g_support.context, 3, &g_allocator) !=
      RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_timer(&g_executor, &g_timer) != RCL_RET_OK) {
    return false;
  }
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
  if (rclc_executor_add_subscription(&g_executor, &g_can1_tx_sub, &g_can1_tx_msg,
                                     can1_tx_sub_cb, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
  if (rclc_executor_add_subscription(&g_executor, &g_joint_state_sub,
                                     &g_joint_state_msg, joint_state_sub_cb,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
#endif

  g_entities_up = true;
  return true;
}

static void destroy_entities() {
  g_entities_up = false;
  // 析构失败无需处理：实体即将整体销毁，agent 断连后整体重建
  rcl_ret_t rc = RCL_RET_OK;
  rc = rclc_executor_remove_timer(&g_executor, &g_timer);
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
  rc = rclc_executor_remove_subscription(&g_executor, &g_can1_tx_sub);
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
  rc = rclc_executor_remove_subscription(&g_executor, &g_joint_state_sub);
#endif
  rc = rcl_timer_fini(&g_timer);
  rc = rclc_executor_fini(&g_executor);
  rc = rcl_publisher_fini(&g_temp_pub, &g_node);
  rc = rcl_publisher_fini(&g_imu_pub, &g_node);
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
  rc = rcl_subscription_fini(&g_can1_tx_sub, &g_node);
  rc = rcl_publisher_fini(&g_can1_rx_pub, &g_node);
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
  rc = rcl_subscription_fini(&g_joint_state_sub, &g_node);
  rc = rcl_publisher_fini(&g_joint_feedback_pub, &g_node);
#endif
  rc = rcl_node_fini(&g_node);
  rc = rclc_support_fini(&g_support);
  (void)rc;
}

// ---------------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------------
bool ros_bridge_create() { return create_entities(); }

void ros_bridge_destroy() { destroy_entities(); }

void ros_bridge_set_imu_source(const imu_data_t *src) { g_imu_source = src; }

void ros_bridge_spin() {
  // 处理 executor 回调（运行定时器发布，并服务微-ROS 会话）
  rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100));
}

void ros_bridge_publish_imu(const imu_data_t *data) {
  // 实体未建立（agent 断连/启动期）时静默跳过，避免空指针
  if (!g_entities_up) {
    return;
  }
  fill_imu_msg(data);
  rcl_ret_t rc = rcl_publish(&g_imu_pub, &g_imu_msg, nullptr);
  rc = rcl_publish(&g_temp_pub, &g_temp_msg, nullptr);
  (void)rc;
}
