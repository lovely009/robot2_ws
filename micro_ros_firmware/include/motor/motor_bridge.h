// 电机域 ROS 桥接口：使用 ID 化有界消息，不携带 joint_name，不直接访问
// 协议字节或 CAN RX。命令经 mailbox 进入 motor_rt_task，状态从只读副本发布。

#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 本桥占用 executor 句柄数（ros_bridge 汇总用，避免魔法数截断）。
size_t motor_bridge_executor_handle_count(void);
uint64_t motor_bridge_session_id(void);
bool motor_bridge_begin_session(uint64_t epoch_ns);

// 创建电机话题（节点/executor 已初始化；失败时自行清理已建实体）
bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

// 销毁电机话题（可重入；未创建时为空操作）
void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);

// 周期轮询（由 ros_bridge 定时器调用）：先把内核配置结果事件转发给
// param_bridge（顺序契约见 ros_bridge.h），再发布 ID 化状态副本。
void motor_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_BRIDGE_H
