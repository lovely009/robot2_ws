// 电机域 ROS 桥接接口（agent 通信层下的电机模块）。
// 受 ENABLE_MOTOR_COMMAND_BRIDGE 宏控制：订阅 /joint_commands 下发默认
// 测试电机，并消费 FDCAN1 反馈发布 micro_ros/JointState 与 motor_status。

#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 创建电机话题（节点/executor 已初始化；失败时自行清理已建实体）
bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

// 销毁电机话题（可重入；未创建时为空操作）
void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);

// 周期消费 FDCAN1 反馈并发布 JointState（由 ros_bridge 定时器调用）
void motor_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_BRIDGE_H
