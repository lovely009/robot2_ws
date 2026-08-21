// micro-ROS 桥接层接口（空工程骨架）
//
// 职责：node/executor 生命周期管理。main.cpp 只负责 FreeRTOS 任务编排与
// 串口初始化，本模块封装全部 ROS 实体。
//
// 注释规范：中文写意图，micro-ROS / ROS 2 API 保留英文。

#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建 ROS 实体（node/publisher/timer），agent 连接建立后调用。成功返回 true
bool ros_bridge_create(void);

// 销毁 ROS 实体，agent 断连后调用（可重入，实体整体重建）
void ros_bridge_destroy(void);

// 周期处理：executor（心跳定时器）
void ros_bridge_spin(void);

// 发布 IMU 数据（sensor_msgs/Imu + std_msgs/Float32 温度）
// agent 未连接或实体未建立时静默跳过。
void ros_bridge_publish_imu(const imu_data_t *data);

// 设置最新 IMU 样本源；定时器会周期读取并发布（在 app 任务更新后调用一次）
void ros_bridge_set_imu_source(const imu_data_t *src);

#ifdef __cplusplus
}
#endif

#endif /* ROS_BRIDGE_H */
