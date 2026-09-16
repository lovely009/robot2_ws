// ROS 2 参数服务器桥（rclc_parameter）——骨架实现。
//
// 本文件只做三件事：建服务器对象、接入 executor、按生命周期收尾。参数集合
// 为空（契约未定），因此这里没有参数注册、没有修改门禁、没有影子值同步。
//
// 安全边界（写参数契约时不得放宽，先记在这里）：修改回调运行在 micro_ros
// 任务（executor）上下文，**不得直接发 FDCAN**——需要影响电机的动作一律经
// mailbox 投给 motor_rt 任务，由内核门禁裁决（ARD §28.1）。

#include "param_bridge.h"

#include <rcl/error_handling.h>
#include <rclc_parameter/rclc_parameter.h>

// executor 配额必须与 rclc 的内部定义一致；不一致时
// rclc_executor_add_parameter_server 会因 handle 不足而失败，且只在运行期暴露。
static_assert(
    PARAM_BRIDGE_EXECUTOR_HANDLES == RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
    "PARAM_BRIDGE_EXECUTOR_HANDLES 与 rclc 的 handle 数不一致");

static rclc_parameter_server_t g_server;
static bool g_created = false;

// 参数修改回调。骨架期没有参数，回调不可达；接线先立在这里，参数契约落地时
// 在同一处加门禁与拒绝原因码（rclc 不给 reason 字段，只能用只读参数回告）。
static bool param_on_modification(const Parameter *old_param,
                                  const Parameter *new_param, void *context) {
  (void)old_param;
  (void)new_param;
  (void)context;
  return true;
}

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) {
    return true; // 幂等：调用方重复 create 不应重建（重建须先 destroy）
  }
  if (node == nullptr || executor == nullptr) {
    return false;
  }

  rclc_parameter_options_t options = {};
  // 不发 /parameter_events：结果类回告由 motor_events 承载，省一个 publisher
  // 槽位与常驻缓冲（rmw 侧 MAX_PUBLISHERS 是稀缺资源）。
  options.notify_changed_over_dds = false;
  options.max_params = PARAM_BRIDGE_MAX_PARAMS;
  options.allow_undeclared_parameters = false; // 只服务已声明的参数集
  options.low_mem_mode = false;

  if (rclc_parameter_server_init_with_option(&g_server, node, &options) !=
      RCL_RET_OK) {
    return false; // 多半是 MAX_SERVICES 配置不符，见 param_bridge.h 的库级前提
  }
  g_created = true;

  if (rclc_executor_add_parameter_server(executor, &g_server,
                                         param_on_modification) != RCL_RET_OK) {
    param_bridge_destroy(node);
    return false;
  }
  return true;
}

void param_bridge_destroy(rcl_node_t *node) {
  if (!g_created) {
    return;
  }
  g_created = false;
  if (node != nullptr) {
    // rclc 没有 "remove_parameter_server"：service handle 留在 executor 里，
    // 由紧随其后的 rclc_executor_fini 一并回收（ros_bridge 的销毁顺序）。
    (void)rclc_parameter_server_fini(&g_server, node);
  }
}

bool param_bridge_created(void) { return g_created; }
