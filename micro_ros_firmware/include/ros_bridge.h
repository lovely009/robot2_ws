// micro-ROS 桥接层接口（agent 通信生命周期）。
//
// 职责：node/executor/timer 生命周期与断连重建。main.cpp 只负责 FreeRTOS
// 任务编排与串口初始化；各业务域的 ROS 实体在 imu_bridge、motor_bridge 中
// 分别维护，本模块按固定顺序编排创建/销毁与定时轮询。

#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 创建 ROS 实体（node/publisher/timer），agent 连接建立后调用。成功返回 true
bool ros_bridge_create(void);

// 销毁 ROS 实体，agent 断连后调用（可重入，实体整体重建）
void ros_bridge_destroy(void);

// 周期处理：spin executor（服务 10ms 周期定时器，驱动各域轮询发布）
void ros_bridge_spin(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_BRIDGE_H */
