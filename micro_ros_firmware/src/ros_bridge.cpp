// micro-ROS 桥接层（空工程骨架）
//
// 职责：node + executor 的生命周期管理。当前为空白脚手架（无发布/订阅），
// 后续业务在此创建 publisher/subscription 并在 executor 注册即可。
//
// 注释规范：中文写意图，micro-ROS / ROS 2 API 保留英文。

#include "ros_bridge.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

// ---------------------------------------------------------------------------
// 配置（单点可改，节点名/话题只在这里改）
// ---------------------------------------------------------------------------
static const char *kNodeName = "micro_ros_node";
static const char *kImuTopic = "imu/data";
static const char *kTempTopic = "imu/temp";

// micro-ROS 实体（agent 断连后整体重建）
static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_publisher_t g_imu_pub;
static rcl_publisher_t g_temp_pub;
static std_msgs__msg__Float32 g_temp_msg;
static sensor_msgs__msg__Imu g_imu_msg;
static rcl_timer_t g_timer;
static const imu_data_t *g_imu_source = nullptr;  // 业务任务写入的最新样本
static bool g_entities_up = false;

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

// 定时器回调：周期发布最新 IMU 样本；同时服务 XRCE 会话（保活）
static void imu_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  if (g_imu_source != nullptr) {
    ros_bridge_publish_imu(g_imu_source);
  }
}

// ---------------------------------------------------------------------------
// 实体生命周期（agent 断连后可重建，全部可重入）
// ---------------------------------------------------------------------------
static bool create_entities() {
  g_entities_up = false;
  rcl_allocator_t allocator = rcl_get_default_allocator();

  if (rclc_support_init(&g_support, 0, nullptr, &allocator) != RCL_RET_OK) {
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

  // 周期定时器：发布 IMU 并维持微-ROS 会话（推荐的最小发布模型）
  if (rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(20),
                              imu_timer_cb) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&g_executor, &g_support.context, 1, &allocator) !=
      RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_timer(&g_executor, &g_timer) != RCL_RET_OK) {
    return false;
  }

  g_entities_up = true;
  return true;
}

static void destroy_entities() {
  g_entities_up = false;
  // 析构失败无需处理：实体即将整体销毁，agent 断连后整体重建
  rcl_ret_t rc = RCL_RET_OK;
  rc = rclc_executor_remove_timer(&g_executor, &g_timer);
  rc = rcl_timer_fini(&g_timer);
  rc = rclc_executor_fini(&g_executor);
  rc = rcl_publisher_fini(&g_temp_pub, &g_node);
  rc = rcl_publisher_fini(&g_imu_pub, &g_node);
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
