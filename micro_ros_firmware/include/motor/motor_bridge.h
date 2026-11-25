// 原生电机域 ROS 桥接口（Schema 6）：直发 /motor/motion, /motor/state, /motor/events，
// 直收 /motor/command, /control/stop, /client_count。
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

size_t motor_bridge_executor_handle_count(void);
uint64_t motor_bridge_session_id(void);
uint64_t motor_bridge_ensure_lease(uint32_t now_ms);
bool motor_bridge_begin_session(uint64_t epoch_ns);
bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);
void motor_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_BRIDGE_H
