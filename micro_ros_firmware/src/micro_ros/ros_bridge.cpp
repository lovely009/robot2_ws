// micro-ROS 桥接层（agent 通信生命周期 + rclc_lifecycle + 原生话题/服务分发）。
#include "ros_bridge.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_lifecycle/rclc_lifecycle.h>
#include <lifecycle_msgs/msg/state.h>
#include <lifecycle_msgs/msg/transition.h>
#include <rmw_microros/time_sync.h>
#include <rmw_microros/timing.h>

#include "imu_bridge.h"
#include "motor_bridge.h"
#include "motor_rt_task.h"
#include "param_bridge.h"
#include "time_utils.h"

static const char *kNodeName = "micro_ros_node";

static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_timer_t g_timer;
static rcl_allocator_t g_allocator;
static rcl_lifecycle_state_machine_t g_state_machine;
static rclc_lifecycle_node_t g_lifecycle_node;
static rclc_lifecycle_service_context_t g_lifecycle_ctx;

static bool g_support_up = false;
static bool g_node_up = false;
static bool g_lifecycle_up = false;
static bool g_executor_up = false;
static bool g_timer_up = false;

static size_t executor_handle_count() {
  // 1 timer + motor_bridge subs + param_bridge services + 3 lifecycle services
  return 1U + motor_bridge_executor_handle_count() +
         PARAM_BRIDGE_EXECUTOR_HANDLES + 3U;
}

static int on_lifecycle_configure(void) {
  return RCL_RET_OK;
}

static int on_lifecycle_activate(void) {
  motor_rt_set_agent_connected(true);
  return RCL_RET_OK;
}

static int on_lifecycle_deactivate(void) {
  motor_rt_request_stop_all();
  return RCL_RET_OK;
}

static int on_lifecycle_cleanup(void) {
  motor_rt_request_stop_all();
  motor_rt_set_agent_connected(false);
  return RCL_RET_OK;
}

bool ros_bridge_is_active(void) {
  if (!g_lifecycle_up || g_state_machine.current_state == nullptr) {
    return false;
  }
  return g_state_machine.current_state->id ==
         lifecycle_msgs__msg__State__PRIMARY_STATE_ACTIVE;
}

static void bridge_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  imu_bridge_poll();
  motor_bridge_poll();
}

void ros_bridge_destroy() {
  motor_rt_set_agent_connected(false);
  motor_rt_set_clock_valid(false);
  time_sync_reset();
  if (g_support_up) {
    const rmw_ret_t rc = rmw_uros_set_context_entity_destroy_session_timeout(
        rcl_context_get_rmw_context(&g_support.context), 0);
    (void)rc;
  }
  imu_bridge_destroy(&g_node);
  motor_bridge_destroy(&g_node, &g_executor);
  param_bridge_destroy(&g_node, &g_executor);

  if (g_timer_up) {
    (void)rclc_executor_remove_timer(&g_executor, &g_timer);
    (void)rcl_timer_fini(&g_timer);
    g_timer_up = false;
  }
  if (g_executor_up) {
    (void)rclc_executor_fini(&g_executor);
    g_executor_up = false;
  }
  if (g_lifecycle_up) {
    (void)rclc_lifecycle_node_fini(&g_lifecycle_node, &g_allocator);
    g_lifecycle_up = false;
  }
  if (g_node_up) {
    (void)rcl_node_fini(&g_node);
    g_node_up = false;
  }
  if (g_support_up) {
    (void)rclc_support_fini(&g_support);
    g_support_up = false;
  }
}

bool ros_bridge_create() {
  ros_bridge_destroy();

  g_allocator = rcl_get_default_allocator();
  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) {
    return false;
  }
  g_support_up = true;

  if (rclc_node_init_default(&g_node, kNodeName, "", &g_support) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  g_node_up = true;

  g_state_machine = rcl_lifecycle_get_zero_initialized_state_machine();
  if (rclc_make_node_a_lifecycle_node(&g_lifecycle_node, &g_node,
      &g_state_machine, &g_allocator, true) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  g_lifecycle_up = true;
  g_lifecycle_ctx.lifecycle_node = &g_lifecycle_node;
  (void)rclc_lifecycle_register_on_configure(&g_lifecycle_node, on_lifecycle_configure);
  (void)rclc_lifecycle_register_on_activate(&g_lifecycle_node, on_lifecycle_activate);
  (void)rclc_lifecycle_register_on_deactivate(&g_lifecycle_node, on_lifecycle_deactivate);
  (void)rclc_lifecycle_register_on_cleanup(&g_lifecycle_node, on_lifecycle_cleanup);

  if (rclc_executor_init(&g_executor, &g_support.context, executor_handle_count(),
      &g_allocator) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  g_executor_up = true;

  if (rclc_lifecycle_init_get_state_server(&g_lifecycle_ctx, &g_executor) != RCL_RET_OK ||
      rclc_lifecycle_init_get_available_states_server(&g_lifecycle_ctx, &g_executor) != RCL_RET_OK ||
      rclc_lifecycle_init_change_state_server(&g_lifecycle_ctx, &g_executor) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }

  if (!imu_bridge_create(&g_node) ||
      !motor_bridge_create(&g_node, &g_executor) ||
      !param_bridge_create(&g_node, &g_executor)) {
    ros_bridge_destroy();
    return false;
  }

  if (rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(10),
      bridge_timer_cb) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  g_timer_up = true;

  if (rclc_executor_add_timer(&g_executor, &g_timer) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }

  const bool sync_ok = rmw_uros_sync_session(1000) == RMW_RET_OK;
  const uint64_t epoch_ns = sync_ok ? rmw_uros_epoch_nanos() : 0;
  time_sync_record(sync_ok, epoch_ns);
  if (!time_clock_valid() || !motor_bridge_begin_session(epoch_ns)) {
    ros_bridge_destroy();
    return false;
  }

  motor_rt_set_clock_valid(true);
  if (rclc_lifecycle_change_state(&g_lifecycle_node,
      lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE, true) != RCL_RET_OK ||
      rclc_lifecycle_change_state(&g_lifecycle_node,
      lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE, true) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  return true;
}

void ros_bridge_spin() {
  if (!g_executor_up) {
    return;
  }
  rclc_executor_spin_some(&g_executor, 0U);
}
