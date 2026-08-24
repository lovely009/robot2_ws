// CAN1 调试桥接接口（agent 通信层下的 CAN 调试模块）。
// 受 ENABLE_CAN1_DEBUG_BRIDGE 宏控制：开启时发布 micro_ros/can1/receive
// （固件收到）并订阅 micro_ros/can1/send（固件发出）。

#ifndef CAN_DEBUG_BRIDGE_H
#define CAN_DEBUG_BRIDGE_H

#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 创建 CAN1 调试话题（节点/executor 已初始化；失败时自行清理已建实体）
bool can_debug_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

// 销毁 CAN1 调试话题（可重入；未创建时为空操作）
void can_debug_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);

// 周期消费 CAN1 最新接收帧并发布（由 ros_bridge 定时器调用）
void can_debug_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif  // CAN_DEBUG_BRIDGE_H
