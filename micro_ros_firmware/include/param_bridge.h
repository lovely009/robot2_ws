// ROS 2 参数服务器桥（rclc_parameter）——骨架。
//
// 现状（骨架期）：只有生命周期与 executor 接线——服务器对象、5 个 service
// （get/get_types/set/list/describe）与容量预留，**参数契约尚未落地**（参数
// 集合为空）。主机侧验证：`ros2 service list` 能看到该节点的 5 个 parameters
// service，`ros2 param list` 返回空。参数集合的设计与逐条落地是下一步。
//
// 生命周期由 ros_bridge 编排（agent 断连时整体重建，与 imu/motor 桥同款）。
//
// 库级前提（不满足会静默失败）：rclc 参数服务器固定要建 5 个 service，要求
// libmicroros.a 以 RMW_UXRCE_MAX_SERVICES>=5 构建（见仓库根的
// microros_user.meta 与 tools/check_microros_config.py）。配置不符时 init 会在
// 第二个 service 上失败 → 本模块返回 false → ros_bridge 整体回滚 → 节点在
// WAIT_AGENT 无限重连；编译期不会有任何提示。
//
// rclc_parameter 的三个硬约束（落参数契约时必须遵守）：
//   1) 只有 BOOL/INT/DOUBLE，没有 string —— 字符串语义（型号名等）只能编码
//      成整数索引；
//   2) 修改回调只能返回 true/false，SetParametersResult.reason 由 rclc 填空串，
//      用户写不进去 —— "为什么被拒"要另开只读参数给原因码；
//   3) rclc_add_parameter 在用户回调内被禁用 —— 动态参数只能在 executor spin
//      之外（bridge timer 轮询）补齐。

#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#ifdef __cplusplus
extern "C" {
#endif

// 参数服务器占用的 executor handle 数。必须与 rclc 的
// RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES 一致：少了 rclc_executor_add_
// parameter_server 直接失败（编译期锁在 param_bridge.cpp）。
#define PARAM_BRIDGE_EXECUTOR_HANDLES 5U

// 参数集合容量（编译期固定，rclc 不动态扩容）。骨架期无参数，留出余量；
// 参数契约落地后按实际条目数收敛。
#define PARAM_BRIDGE_MAX_PARAMS 32U

// 创建参数服务器并接入 executor（node/executor 由 ros_bridge 提供）。
// 失败返回 false，调用方按既有失败路径整体回滚。
bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

// 销毁参数服务器（只 fini 自己的实体；executor/node 由 ros_bridge 收尾）。
void param_bridge_destroy(rcl_node_t *node);

bool param_bridge_created(void);

#ifdef __cplusplus
}
#endif

#endif // PARAM_BRIDGE_H
