// 配置桥只持有快照与事务结果；发布由 motor_bridge 统一调度。
#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <can_bridge_msgs/msg/config_result.h>
#include <can_bridge_msgs/msg/system_status.h>
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_BRIDGE_EXECUTOR_HANDLES 1U

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);
// 同一任务内先消费内核事件，再刷新视图并驱动事务。
void param_bridge_poll(void);
void param_bridge_on_config_result(const motor_control_event_t *event);
const can_bridge_msgs__msg__SystemStatus *param_bridge_system_status(void);
// 发布成功才 ack；失败后下一轮可再次 peek。
bool param_bridge_result_peek(can_bridge_msgs__msg__ConfigResult *out);
void param_bridge_result_ack(void);
uint32_t param_bridge_request_high_water(void);
// 兼容旧诊断入口；本桥不再发布，恒为 0。
uint32_t param_bridge_publish_fail_count(void);
const motor_control_param_view_t *param_bridge_views(size_t *out_count);
// REPORT 被 facade 取走即完成；此入口保持合并报告请求的语义。
bool param_bridge_take_report_request(void);
const motor_control_stored_params_t *param_bridge_stored_records(size_t *out_count);

#ifdef __cplusplus
}
#endif
#endif
