// IMU 域 ROS 桥接接口（agent 通信层下的 IMU 发布模块）。
// 负责 sensor_msgs/Imu（micro_ros_up/imu/data）与 std_msgs/Float32
// （micro_ros_up/imu/temp）的缓冲、填充与发布；生命周期由 ros_bridge 统一管理。

#ifndef IMU_BRIDGE_H
#define IMU_BRIDGE_H

#include <stdbool.h>

#include <rcl/rcl.h>

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建 IMU/温度 publisher（节点已初始化；失败时自行清理已建实体）
bool imu_bridge_create(rcl_node_t *node);

// 销毁 IMU/温度 publisher（可重入；未创建时为空操作）
void imu_bridge_destroy(rcl_node_t *node);

// 周期发布最新样本（由 ros_bridge 定时器调用；未连接时静默跳过）
void imu_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif  // IMU_BRIDGE_H
