// 电机域 ROS 桥接口（agent 通信层下的电机模块）。
//
// Phase B 中间态：旧 /joint_commands(JointState)、motor_mode、motor_gains
// 订阅与 JointState/motor_status 发布全部为 joint_name 依赖面，已随任务层
// 重写移除（ARD AR-003：MCU 不携带关节语义；HANDOFF 阶段 C）。命令/状态
// 话题面将在 Phase C 随 ID 化消息（MotorCommandArray / MotorMotionStateArray
// / MotorStatusArray 等，ARD §15-20）冻结后以 motor_ros_bridge 重建。
// 本阶段仅保留无名字依赖的 motor_enable → 运行时授权门。

#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 本桥占用 executor 句柄数（ros_bridge 汇总用，避免魔法数截断）。
size_t motor_bridge_executor_handle_count(void);

// 创建电机话题（节点/executor 已初始化；失败时自行清理已建实体）
bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

// 销毁电机话题（可重入；未创建时为空操作）
void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);

// 周期轮询（由 ros_bridge 定时器调用；Phase C 在此发布 ID 化状态副本）
void motor_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_BRIDGE_H
