// 原生配置/策略/设备信息服务桥（Schema 6）。
#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <can_bridge_msgs/msg/motor_system_state.h>
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_BRIDGE_EXECUTOR_HANDLES 3U
bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_poll(void);
void param_bridge_on_config_result(const motor_control_event_t *event);
void param_bridge_fill_system_state(can_bridge_msgs__msg__MotorSystemState *out);

#ifdef __cplusplus
}
#endif
#endif
