// micro-ROS 桥接层（agent 通信生命周期）：node + executor + 周期定时器，断连后整体
// 重建。业务域实体由 imu_bridge / motor_bridge / param_bridge 各自维护，本模块只管
// 创建/销毁顺序与定时轮询分发。

#include "ros_bridge.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/time_sync.h>
#include <rmw_microros/timing.h>

#include "imu_bridge.h"
#include "motor_bridge.h"
#include "motor_rt_task.h"
#include "param_bridge.h"
#include "firmware_parameters.h"
#include "time_utils.h"

static const char *kNodeName = "micro_ros_node";

static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_timer_t g_timer;
static rcl_allocator_t g_allocator;

// executor 最大句柄数：定时器 + 各业务桥订阅数（桥自报，避免魔法数截断）。
// 只有订阅/定时器/服务占 handle，发布不占。少算一个就在 add_subscription 上
// 失败，且只在运行期以 WAIT_AGENT 无限重连的形式暴露。
static size_t executor_handle_count() {
  return 1U + motor_bridge_executor_handle_count() +
         PARAM_BRIDGE_EXECUTOR_HANDLES + FIRMWARE_PARAMETER_HANDLES;
}

// 按初始化阶段跟踪，保证失败/断连时只清理已成功创建的实体
static bool g_support_up = false;
static bool g_node_up = false;
static bool g_executor_up = false;
static bool g_timer_up = false;

// 周期定时器：驱动各业务域轮询（IMU、电机反馈与状态发布）
static void bridge_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;
  imu_bridge_poll();
  motor_bridge_poll();
  firmware_parameters_poll();
  // motor_bridge_poll 内先转发事件、更新参数快照，再统一发送。
}

// ---------------------------------------------------------------------------
// 对外接口（micro_ros_task 调用；可重入：断连后整体重建）
// ---------------------------------------------------------------------------
void ros_bridge_destroy() {
  // 先通知实时任务失联，使旧命令在 ROS 实体清理期间即失效。
  motor_rt_set_agent_connected(false);
  motor_rt_set_clock_valid(false);
  time_sync_reset();
  if (g_support_up) {
    // Agent 已失联，逐实体等待远端删除确认只会延迟下一次建链。
    const rmw_ret_t rc = rmw_uros_set_context_entity_destroy_session_timeout(
        rcl_context_get_rmw_context(&g_support.context), 0);
    (void)rc;
  }
  imu_bridge_destroy(&g_node);
  motor_bridge_destroy(&g_node, &g_executor);
  param_bridge_destroy(&g_node, &g_executor);
  firmware_parameters_destroy(&g_node, &g_executor);

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

bool ros_bridge_create() {
  // 先清残留，保证可重入（agent 断连后整体重建）
  ros_bridge_destroy();

  // rclc_support/executor 保存 allocator 指针而非拷贝，必须使用静态存储
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

  if (rclc_executor_init(&g_executor, &g_support.context, executor_handle_count(),
      &g_allocator) != RCL_RET_OK) {
    ros_bridge_destroy();
    return false;
  }
  g_executor_up = true;

  if (!imu_bridge_create(&g_node)) {
    ros_bridge_destroy();
    return false;
  }
  if (!motor_bridge_create(&g_node, &g_executor)) {
    ros_bridge_destroy();
    return false;
  }
  if (!param_bridge_create(&g_node, &g_executor)) {
    ros_bridge_destroy();
    return false;
  }
  if (!firmware_parameters_create(&g_node, &g_executor)) {
    ros_bridge_destroy();
    return false;
  }

  // 10ms 周期：IMU 最多100Hz，motion 每20ms一代，可靠流每轮最多一条；
  // status 基础10Hz、重要变化最小20ms，温度仅新样本2Hz。
  // USB CDC 最大负载与可靠分片阻塞仍需台架验证。
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
  // 会话已建立：发起一次 NTP 时间同步，使 rmw_uros_epoch_nanos() 与主机对齐。
  // 同步成功后分配会话标识；未建立会话时禁止接收运动授权。
  const bool sync_ok = rmw_uros_sync_session(1000) == RMW_RET_OK;
  const uint64_t epoch_ns = sync_ok ? rmw_uros_epoch_nanos() : 0;
  time_sync_record(sync_ok, epoch_ns);
  if (!time_clock_valid() || !motor_bridge_begin_session(epoch_ns)) {
    ros_bridge_destroy();
    return false;
  }
  // 仅在所有实体和 executor handle 均就绪后允许接收新电机命令。
  motor_rt_set_clock_valid(true);
  motor_rt_set_agent_connected(true);
  return true;
}

void ros_bridge_spin() {
  if (!g_executor_up) {
    return;
  }
  // 处理 executor 回调（运行定时器发布，并服务微-ROS 会话）
  rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100));
}
