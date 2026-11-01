#include "firmware_parameters.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <rclc_parameter/rclc_parameter.h>
#include <string.h>
#include "motor_rt_task.h"
#include "param_bridge.h"
#include "time_utils.h"

static firmware_settings_t g_current = FIRMWARE_SETTINGS_DEFAULT;
static firmware_settings_t g_saved = FIRMWARE_SETTINGS_DEFAULT;
static rclc_parameter_server_t g_server;
static bool g_created, g_added, g_internal;
static motor_control_status_t g_motor_status[MOTOR_CONTROL_MAX_MOTORS];
static const char *const kReadOnly[] = {
  "system.can_nominal_kbps", "system.can_data_kbps", "system.unsaved"};
static_assert(FIRMWARE_PARAMETER_HANDLES == RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
    "参数服务 executor 句柄数漂移");

firmware_settings_t firmware_parameters_snapshot(void) {
  taskENTER_CRITICAL();
  const auto settings = g_current;
  taskEXIT_CRITICAL();
  return settings;
}
void firmware_parameters_load(const uint32_t words[3]) {
  firmware_settings_t settings = FIRMWARE_SETTINGS_DEFAULT;
  (void)firmware_settings_unpack(words, &settings);
  taskENTER_CRITICAL();
  g_current = g_saved = settings;
  taskEXIT_CRITICAL();
}
void firmware_parameters_saved(const firmware_settings_t *settings) {
  taskENTER_CRITICAL();
  g_saved = *settings;
  taskEXIT_CRITICAL();
}

bool firmware_parameters_unsaved(void) {
  taskENTER_CRITICAL();
  const bool dirty = !firmware_settings_equal(&g_current, &g_saved);
  taskEXIT_CRITICAL();
  return dirty;
}
bool firmware_parameters_set_value(unsigned field, int64_t value) {
  if (field >= FIRMWARE_SETTINGS_COUNT || !time_clock_valid() || param_bridge_save_in_progress()) return false;
  auto candidate = firmware_parameters_snapshot();
  if (!firmware_settings_set(&candidate, field, value)) return false;
  taskENTER_CRITICAL();
  const size_t count = motor_rt_copy_statuses(g_motor_status, MOTOR_CONTROL_MAX_MOTORS);
  bool enabled = false;
  for (size_t j = 0; j < count; ++j)
    enabled |= g_motor_status[j].policy_enabled || g_motor_status[j].motion_enabled;
  agent_trajectory_status_t trajectory{};
  motor_rt_copy_agent_trajectory(&trajectory);
  if (enabled || (trajectory.state >= AGENT_TRAJECTORY_LOADING && trajectory.state <= AGENT_TRAJECTORY_RUNNING)) {
    taskEXIT_CRITICAL(); return false;
  }
  g_current = candidate;
  taskEXIT_CRITICAL();
  return true;
}
static bool on_parameter(const Parameter *old_value, const Parameter *value, void *) {
  if (g_internal) return true;
  if (!old_value || !value || value->value.type != RCLC_PARAMETER_INT) return false;
  for (unsigned i = 0; i < FIRMWARE_SETTINGS_COUNT; ++i)
    if (!strcmp(value->name.data, firmware_setting_names[i]))
      return firmware_parameters_set_value(i, value->value.integer_value);
  return false;
}
void firmware_parameters_poll(void) {
  if (!g_created) return;
  taskENTER_CRITICAL();
  const bool dirty = !firmware_settings_equal(&g_current, &g_saved);
  taskEXIT_CRITICAL();
  g_internal = true;
  const auto settings = firmware_parameters_snapshot();
  for (unsigned i = 0; i < FIRMWARE_SETTINGS_COUNT; ++i) {
    const auto rc = rclc_parameter_set_int(&g_server, firmware_setting_names[i], firmware_settings_get(&settings, i));
    (void)rc;
  }
  const rcl_ret_t rc = rclc_parameter_set_bool(&g_server, kReadOnly[2], dirty);
  (void)rc;
  g_internal = false;
}
bool firmware_parameters_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) return true;
  const rclc_parameter_options_t options = {false, FIRMWARE_SETTINGS_COUNT + 3U, false, true};
  const rcl_ret_t rc = rclc_parameter_server_init_with_option(&g_server, node, &options);
  // 初始化即使失败也可能已创建服务；由统一 destroy 收回。
  g_created = true;
  if (rc != RCL_RET_OK) return false;
  const auto settings = firmware_parameters_snapshot();
  for (unsigned i = 0; i < FIRMWARE_SETTINGS_COUNT + 3U; ++i) {
    const char *name = i < FIRMWARE_SETTINGS_COUNT ? firmware_setting_names[i] : kReadOnly[i - FIRMWARE_SETTINGS_COUNT];
    const auto type = i == FIRMWARE_SETTINGS_COUNT + 2U ? RCLC_PARAMETER_BOOL : RCLC_PARAMETER_INT;
    if (rclc_add_parameter(&g_server, name, type) != RCL_RET_OK) return false;
    if (type == RCLC_PARAMETER_BOOL) continue;
    const int64_t value = i < FIRMWARE_SETTINGS_COUNT ? firmware_settings_get(&settings, i) :
        i == FIRMWARE_SETTINGS_COUNT ? 1000 : 5000;
    if (rclc_parameter_set_int(&g_server, name, value) != RCL_RET_OK) return false;
  }
  if (rclc_executor_add_parameter_server(executor, &g_server, on_parameter) != RCL_RET_OK) return false;
  g_added = true;
  firmware_parameters_poll();
  return true;
}
void firmware_parameters_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (!g_created) return;
  if (g_added) {
    rcl_service_t *services[] = {&g_server.get_service, &g_server.get_types_service,
      &g_server.set_service, &g_server.list_service, &g_server.describe_service};
    for (auto *service : services) {
      const rcl_ret_t rc = rclc_executor_remove_service(executor, service); (void)rc;
    }
  }
  const rcl_ret_t rc = rclc_parameter_server_fini(&g_server, node); (void)rc;
  g_server = rclc_parameter_server_t{};
  g_created = g_added = g_internal = false;
}
