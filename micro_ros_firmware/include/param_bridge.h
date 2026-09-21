// 参数与系统命令桥（**话题面**）：取代 2026-09-21 退役的 rclc 参数服务器。
//
// 下行两条（订阅）：
//   /micro_ros_do/param_set       ParamSetArray   稀疏参数写（整条受理/整条拒绝）
//   /micro_ros_do/system_command  SystemCommand   save/refresh/set_zero/clear_fault
// 上行一条（发布）：
//   /micro_ros_up/system_status   SystemStatus    错误电平 + 四个完成序号，5 Hz
//
// **只读侧不在这里**：参数核对的双行快照（固件记录 vs 驱动板回读）由
// motor_bridge 的 /micro_ros_up/device_param_report 承载，它消费本桥的
// param_bridge_views()。写侧与读侧的槽位序是同一套（param_device_slot_t）。
//
// 语义：参数写是**异步事务**——回调只做「整条校验 + 提交 mailbox」，同步返回
// 不代表成功；最终成败经内核 CONFIG_RESULT 事件异步给出（on_config_result），
// 并由 system_status.last_error 呈现。回调运行在 micro-ROS 任务上下文，不得
// 直接发 FDCAN / 擦 Flash——一律经 mailbox 或 storage 任务通知（ARD §28.1）。
//
// 与参数面时期的差异（都是简化，不是新增能力）：
//   - 没有影子副本、没有失败回滚：话题是**命令**不是状态副本，失败 = last_error
//     + 脏位不翻转 + record_dirty 仍为真，由操作员重发。
//   - 完成判据从"读 system.<x>_seq"变成"读 system_status.<x>_seq"，语义不变。
//   - enable/kp/kd 仍在 /micro_ros_do/motor_policy（ARD §19.4 禁止它们进命令
//     快照），本桥从不经手。

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

// executor 句柄数：两条订阅。**发布不占 handle**（rclc 只为订阅/定时器/服务
// 分配），所以加话题时只有订阅要动这个数——少算了会在 add_subscription 上失败。
// 有 static_assert 与 .msg 上界共同守着（见 .cpp）。
#define PARAM_BRIDGE_EXECUTOR_HANDLES 2U

// 历史：本桥曾承载 rclc 参数服务器，容量按 72 槽（10 system + 10 电机 × 6 项）
// 一次性预分配，常驻 23,616 B 的 DTCM 堆。那段推导（328 B/槽的实测斜率、
// 416 槽必然分配失败、rclc 不检查分配结果）保留在
// docs/architecture/motor-runtime-decision-log.md 与
// docs/plans/active/48-motor-scale-design.md §5.2/§5.3，不在活注释里复制第二份
// ——已经漂移过一次。话题面不预分配，容量随 .msg 的有界数组走。

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
// 必须把 executor 传进来：两条订阅要在 fini 之前从 executor 移除，否则重连后
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

#ifdef __cplusplus
}
#endif

#endif // PARAM_BRIDGE_H
