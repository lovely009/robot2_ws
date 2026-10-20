// 参数与系统命令桥（**话题面**）。下行 WireContract 的 TOPIC_CONFIG（kind 区分稀疏参数写
// 与系统命令，整条受理或整条拒绝）；上行 /micro_ros_up/system_status（错误电平 + 四个完成序号，
// 5 Hz）。参数核对的**双行快照由 motor_bridge 的 device_param_report 承载**，它消费本桥的
// param_bridge_views()，两侧槽位序同一套。参数写是**异步事务**：回调只做「整条校验 + 提交
// mailbox」，成败经内核 CONFIG_RESULT 异步给出、由 system_status.last_error 呈现，没有影子
// 副本也没有回滚。回调在 micro-ROS 任务上下文，不得直接发 FDCAN 或擦 Flash；enable/kp/kd
// 走 WireContract 的 TOPIC_POLICY，本桥从不经手。

#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// executor 句柄数：一条订阅（ConfigRequest 合并了参数写与系统命令）。**发布不占
// handle**（rclc 只为订阅/定时器/服务分配），所以加话题时只有订阅要动这个数——
// 少算了会在 add_subscription 上失败。有 static_assert 与 .msg 上界共同守着（见 .cpp）。
#define PARAM_BRIDGE_EXECUTOR_HANDLES 1U

// 历史：本桥曾承载 rclc 参数服务器（72 槽一次性预分配，常驻 23,616 B DTCM 堆）。
// 那段推导保留在 docs/architecture/motor-runtime-decision-log.md 与
// docs/plans/active/48-motor-scale-design.md §5.2/§5.3，不在活注释里复制第二份
// ——已经漂移过一次。话题面不预分配，容量随 .msg 的有界数组走。

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
// 必须把 executor 传进来：订阅要在 fini 之前从 executor 移除，否则重连后
// 残留句柄会顶掉后面话题的 init（micro-ROS 的典型静默失败）。
void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor);

// 10ms 桥定时器调用（排在 motor_bridge_poll 之后）：发布 system_status、
// 分批下发保存、上报设备参数结论。
void param_bridge_poll(void);

// motor_bridge publish_events 的逐条事件钩子（同任务顺序调用）：
// 收敛 system_command 的完成序号，并把非本桥命令的失败也报进 last_error。
void param_bridge_on_config_result(const motor_control_event_t *event);

// 本桥的出站发布失败计数（best-effort 超传输上限时 rcl_publish 静默丢弃）。
// 由 motor_bridge 合并进 MotorDiagnostics.publish_fail——**不要 (void) 掉**。
uint32_t param_bridge_publish_fail_count(void);

// 最近一次 param_bridge_poll 的参数视图快照（只读，条数写入 *out_count）。
// 设备参数报告（motor_bridge）复用这一份，而不是自己再拷一份：视图数组
// 64 台约 6 KB，拷两份就是白占一份 RAM_D1。两者同在本任务里顺序调用，无并发。
const motor_control_param_view_t *param_bridge_views(size_t *out_count);

// 取出并清除一次性的「上报参数视图」请求（SystemCommand COMMAND_REPORT 置位）。
// motor_bridge 每拍 poll 调用：为 true 就发一帧 device_param_report。请求不进
// mailbox、不访问设备——REPORT 与 REFRESH 的区别就在这里。
bool param_bridge_take_report_request(void);

// W25 boot 参数记录（直通 storage_task 的只读快照，条数写入 *out_count）。
// motor_bridge 据此给「未发现但有板内记录」的电机补行：没有在线电机时
// device_param_report 呈现的就是这份板内参数表。
const motor_control_stored_params_t *param_bridge_stored_records(size_t *out_count);

#ifdef __cplusplus
}
#endif

#endif // PARAM_BRIDGE_H
