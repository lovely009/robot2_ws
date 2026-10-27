// IMU 域 ROS 桥接实现。
//
// 职责：维护 IMU/温度消息缓冲，定时器触发时发布 sensor_task 写入的最新样本。
// 实体创建/销毁由 ros_bridge 编排，本模块只处理自己的 publisher。

#include "imu_bridge.h"
#include "ros_qos.h"
#include "serial_transport.h"

#include <Arduino.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>
#include <can_bridge_msgs/msg/agent_wire_contract.h>

#include "sensor_task.h"
#include "time_utils.h"

static uint32_t g_publish_fail = 0U;
uint32_t imu_bridge_publish_fail_count(void) { return g_publish_fail; }
static rcl_publisher_t g_imu_pub;
static rcl_publisher_t g_temp_pub;
static sensor_msgs__msg__Imu g_imu_msg;
static sensor_msgs__msg__Temperature g_temp_msg;
static bool g_ready = false;
static char kImuFrameId[] = "imu_link";

static uint32_t g_imu_sequence = 0U;
static uint32_t g_temp_sequence = 0U;
static bool g_have_imu_sequence = false;
static bool g_have_temp_sequence = false;
static uint32_t g_temp_tick = 0U;

static void fill_imu_msg(const imu_data_t *data,
    const sensor_sample_metadata_t *metadata) {
  fill_epoch_stamp(&g_imu_msg.header.stamp, metadata->epoch_ns);
  g_imu_msg.header.frame_id.data = (char *)kImuFrameId;
  g_imu_msg.header.frame_id.size = sizeof(kImuFrameId) - 1U;
  g_imu_msg.header.frame_id.capacity = sizeof(kImuFrameId);

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
}

bool imu_bridge_create(rcl_node_t *node) {
  rcl_publisher_options_t opts = rcl_publisher_get_default_options();
  opts.qos = qos_stream();
  if (rcl_publisher_init(&g_imu_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_IMU, &opts) != RCL_RET_OK) return false;
  if (rcl_publisher_init(&g_temp_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_TEMPERATURE, &opts) != RCL_RET_OK) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_imu_pub, node); (void)rc;
    return false;
  }
  g_have_imu_sequence = false;
  g_have_temp_sequence = false;
  g_temp_tick = millis();
  g_ready = true;
  return true;
}

void imu_bridge_destroy(rcl_node_t *node) {
  if (!g_ready) {
    return;
  }
  g_ready = false;
  rcl_ret_t rc = rcl_publisher_fini(&g_temp_pub, node);
  rc = rcl_publisher_fini(&g_imu_pub, node);
  (void)rc;
}

void imu_bridge_poll(void) {
  if (!g_ready || !time_clock_valid()) return;
  imu_data_t snapshot = {};
  sensor_sample_metadata_t metadata = {};
  if (sensor_task_get_imu_sample(&snapshot, &metadata) && metadata.clock_valid &&
      (!g_have_imu_sequence || metadata.sequence != g_imu_sequence)) {
    fill_imu_msg(&snapshot, &metadata);
    serial_transport_send_begin();
    const rcl_ret_t rc = rcl_publish(&g_imu_pub, &g_imu_msg, nullptr);
    serial_transport_send_end();
    if (rc != RCL_RET_OK) ++g_publish_fail;
    g_imu_sequence = metadata.sequence;
    g_have_imu_sequence = true;
  }
  const uint32_t now = millis();
  if ((uint32_t)(now - g_temp_tick) < 500U) return;
  g_temp_tick = now;
  float temperature = 0;
  if (sensor_task_get_temperature(&temperature, &metadata) && metadata.clock_valid &&
      (!g_have_temp_sequence || metadata.sequence != g_temp_sequence)) {
    // 温度来自独立读取快照，绝不借姿态发布时间刷新旧温度。
    fill_epoch_stamp(&g_temp_msg.header.stamp, metadata.epoch_ns);
    g_temp_msg.header.frame_id.data = kImuFrameId;
    g_temp_msg.header.frame_id.size = sizeof(kImuFrameId) - 1U;
    g_temp_msg.header.frame_id.capacity = sizeof(kImuFrameId);
    g_temp_msg.temperature = temperature;
    g_temp_msg.variance = 0.0;
    serial_transport_send_begin();
    const rcl_ret_t rc = rcl_publish(&g_temp_pub, &g_temp_msg, nullptr);
    serial_transport_send_end();
    if (rc != RCL_RET_OK) ++g_publish_fail;
    g_temp_sequence = metadata.sequence;
    g_have_temp_sequence = true;
  }
}
