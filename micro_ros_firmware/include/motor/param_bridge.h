// 配置桥只持有快照与事务结果；发布由 motor_bridge 统一调度。
#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <can_bridge_msgs/msg/agent_status.h>
#include <can_bridge_msgs/msg/agent_info.h>
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_BRIDGE_EXECUTOR_HANDLES 1U
bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_poll(void);
void param_bridge_on_config_result(const motor_control_event_t *event);
// 调用方为 results 提供 8 个静态元素；发布不回收结果。
void param_bridge_fill_status(can_bridge_msgs__msg__AgentStatus *out);
// 单页待发数据，发送失败仍保留；无待发页返回 NULL。
const can_bridge_msgs__msg__AgentInfo *param_bridge_info(void);
void param_bridge_info_sent(bool success);
#ifdef __cplusplus
}
#endif
#endif
