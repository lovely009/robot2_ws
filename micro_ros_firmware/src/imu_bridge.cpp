// IMU 域 ROS 桥接实现。
//
// 职责：维护 IMU/温度消息缓冲，定时器触发时发布 sensor_task 写入的最新样本。
// 实体创建/销毁由 ros_bridge 编排，本模块只处理自己的 publisher。

#include "imu_bridge.h"

#include <Arduino.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

#include "sensor_task.h"
#include "time_utils.h"

static const char *kImuTopic = "micro_ros/imu/data";
static const char *kTempTopic = "micro_ros/imu/temp";

static rcl_publisher_t g_imu_pub;
static rcl_publisher_t g_temp_pub;
static sensor_msgs__msg__Imu g_imu_msg;
static std_msgs__msg__Float32 g_temp_msg;
static bool g_ready = false;

static void fill_imu_msg(const imu_data_t *data) {
  // 时间戳（sensor_msgs 惯例，sec + nanosec）：epoch 优先，未同步回退 millis
  fill_header_stamp(&g_imu_msg.header.stamp);
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

bool imu_bridge_create(rcl_node_t *node) {
  if (rclc_publisher_init_default(&g_imu_pub, node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                  kImuTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_default(&g_temp_pub, node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                  kTempTopic) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_publisher_fini(&g_imu_pub, node);
    (void)rc;
    return false;
  }
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
  if (!g_ready) {
    return;
  }
  imu_data_t snapshot = {};
  if (!sensor_task_get_imu_snapshot(&snapshot, nullptr)) {
    return;
  }
  fill_imu_msg(&snapshot);
  rcl_ret_t rc = rcl_publish(&g_imu_pub, &g_imu_msg, nullptr);
  rc = rcl_publish(&g_temp_pub, &g_temp_msg, nullptr);
  (void)rc;
}
