#ifndef FIRMWARE_PARAMETERS_H
#define FIRMWARE_PARAMETERS_H
#include "firmware_settings.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#ifdef __cplusplus
extern "C" {
#endif
#define FIRMWARE_PARAMETER_HANDLES 5U
bool firmware_parameters_create(rcl_node_t *node, rclc_executor_t *executor);
void firmware_parameters_destroy(rcl_node_t *node, rclc_executor_t *executor);
void firmware_parameters_poll(void);
bool firmware_parameters_set_value(unsigned field, int64_t value);
bool firmware_parameters_unsaved(void);
void firmware_parameters_load(const uint32_t words[3]);
firmware_settings_t firmware_parameters_snapshot(void);
void firmware_parameters_saved(const firmware_settings_t *settings);
#ifdef __cplusplus
}
#endif
#endif
