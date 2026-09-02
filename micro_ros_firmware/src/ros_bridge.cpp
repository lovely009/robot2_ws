// micro-ROS 桥接层（agent 通信生命周期）。
//
// 职责：node + executor + 周期定时器生命周期管理，agent 断连后整体重建。
// 业务域实体由 imu_bridge / can_debug_bridge / motor_bridge 各自维护，
// 本模块只负责创建/销毁顺序与定时轮询分发。
//
// 注释规范：中文写意图，micro-ROS / ROS 2 API 保留英文。

#include "ros_bridge.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
#include <rclc_parameter/rclc_parameter.h>
#endif
#include <rmw_microros/time_sync.h>

#include "can_debug_bridge.h"
#include "control_task.h"
#include "imu_bridge.h"
#include "motor_bridge.h"

static const char *kNodeName = "micro_ros_node";

static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_timer_t g_timer;
static rcl_allocator_t g_allocator;
#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
static rclc_parameter_server_t g_param_server;
#endif

// executor 最大句柄数：定时器 + 各宏开关下的订阅/参数服务器
static const size_t kExecutorHandles = 1
#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE
    + 1
#endif
#if defined(ENABLE_MOTOR_COMMAND_BRIDGE) && ENABLE_MOTOR_COMMAND_BRIDGE
    + 2
#endif
#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
    + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES
#endif
    ;

// 按初始化阶段跟踪，保证失败/断连时只清理已成功创建的实体
static bool g_support_up = false;
static bool g_node_up = false;
static bool g_executor_up = false;
static bool g_timer_up = false;
#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
static bool g_param_server_up = false;
#endif

// 周期定时器：驱动各业务域轮询（IMU 发布、CAN1 调试帧、电机反馈）
static void bridge_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  imu_bridge_poll();
  can_debug_bridge_poll();
  motor_bridge_poll();
#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
  motor_bridge_apply_pending_config();
#endif
}

static void destroy_entities() {
  // 先通知控制任务失联，使旧命令在 ROS 实体清理期间即失效。
  control_task_set_agent_connected(false);
  imu_bridge_destroy(&g_node);
  can_debug_bridge_destroy(&g_node, &g_executor);
  motor_bridge_destroy(&g_node, &g_executor);

#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
  if (g_param_server_up) {
    rcl_ret_t rc = rclc_parameter_server_fini(&g_param_server, &g_node);
    (void)rc;
    g_param_server_up = false;
  }
#endif
  if (g_timer_up) {
    rcl_ret_t rc = rclc_executor_remove_timer(&g_executor, &g_timer);
    rc = rcl_timer_fini(&g_timer);
    (void)rc;
    g_timer_up = false;
  }
  if (g_executor_up) {
    rcl_ret_t rc = rclc_executor_fini(&g_executor);
    (void)rc;
    g_executor_up = false;
  }
  if (g_node_up) {
    rcl_ret_t rc = rcl_node_fini(&g_node);
    (void)rc;
    g_node_up = false;
  }
  if (g_support_up) {
    rcl_ret_t rc = rclc_support_fini(&g_support);
    (void)rc;
    g_support_up = false;
  }
}

static bool create_entities() {
  // 先清残留，保证可重入（agent 断连后整体重建）
  destroy_entities();

  // rclc_support/executor 保存 allocator 指针而非拷贝，必须使用静态存储
  g_allocator = rcl_get_default_allocator();
  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) {
    return false;
  }
  g_support_up = true;

  if (rclc_node_init_default(&g_node, kNodeName, "", &g_support) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
  g_node_up = true;

  if (rclc_executor_init(&g_executor, &g_support.context, kExecutorHandles,
                         &g_allocator) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
  g_executor_up = true;

  if (!imu_bridge_create(&g_node)) {
    destroy_entities();
    return false;
  }
  if (!can_debug_bridge_create(&g_node, &g_executor)) {
    destroy_entities();
    return false;
  }
  if (!motor_bridge_create(&g_node, &g_executor)) {
    destroy_entities();
    return false;
  }

#if defined(ENABLE_PARAMETER_SERVER) && ENABLE_PARAMETER_SERVER
  // 初始化参数服务器（允许未声明参数，支持动态电机拓扑）
  rclc_parameter_options_t param_options;
  param_options.notify_changed_over_dds = true;
  param_options.max_params = 64;
  param_options.allow_undeclared_parameters = true;
  param_options.low_mem_mode = false;
  if (rclc_parameter_server_init_with_option(&g_param_server, &g_node, &param_options) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
  g_param_server_up = true;

  if (rclc_executor_add_parameter_server_with_context(&g_executor, &g_param_server, motor_bridge_param_callback, nullptr) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
#endif

  if (rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(20),
                              bridge_timer_cb) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
  g_timer_up = true;

  if (rclc_executor_add_timer(&g_executor, &g_timer) != RCL_RET_OK) {
    destroy_entities();
    return false;
  }
  // 会话已建立：发起一次 NTP 时间同步，使 rmw_uros_epoch_nanos() 与主机对齐。
  // 失败不致命（各桥在未同步时回退 millis()），故忽略返回值。
  rmw_uros_sync_session(1000);
  // 仅在所有实体和 executor handle 均就绪后允许接收新电机命令。
  control_task_set_agent_connected(true);
  return true;
}

// ---------------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------------
bool ros_bridge_create() {
  return create_entities();
}

void ros_bridge_destroy() {
  destroy_entities();
}

void ros_bridge_spin() {
  if (!g_executor_up) {
    return;
  }
  // 处理 executor 回调（运行定时器发布，并服务微-ROS 会话）
  rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100));
}
