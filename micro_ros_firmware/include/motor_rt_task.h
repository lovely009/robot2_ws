// 唯一电机实时业务所有者（ARD-0007 AR-005 / §11 / §28）。
//
// 节拍：
//  - 1ms 服务：三路 RX ring 批量弹出（每总线有界配额，TBD-CAN-01）→
//    motor_control_on_raw_frame → motor_control_service（OFFLINE/新鲜度级联/
//    配置事务步进）→ 每总线 can_tx_scheduler pump → can_bsp_submit_tx。
//  - 20ms 控制边界：mailbox 会话/授权边沿同步到内核 → 完整命令快照 →
//    apply_command_snapshot → 配置 FIFO 逐条（每边界有界）→ submit_config。
//
// 线程边界：mailbox 是本任务与 ROS 桥之间的唯一通道。外部只允许通过本头
// 文件的写入口（临界区内转发 mailbox）与只读状态副本交互；内核上下文仅由
// 本任务写（ARD §28.1 单一所有者）。
//
// 无 raw CAN 调试出口，不转发旧 control_task_* 符号（禁止项 10）。

#ifndef MOTOR_RT_TASK_H
#define MOTOR_RT_TASK_H

#include <stdbool.h>
#include <stddef.h>

#include "control_mailbox.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建实时任务（幂等；mailbox/内核初始化完成后任务延迟自启动 FDCAN）。
// 返回 false = 任务创建失败（调用方应 Error_Handler）。
bool motor_rt_task_start(void);

// ---- mailbox 写入口（ROS 桥/会话层调用；内部短临界区）----

// 提交一份完整命令快照（latest-wins；非法快照由 mailbox 拒绝并计数）。
bool motor_rt_submit_command(const control_command_snapshot_t *snapshot);

// 低频配置提交（暂无 ROS 生产者；契约见 control_mailbox 配置 FIFO）。
bool motor_rt_submit_config(const control_mailbox_config_request_t *request);

// 会话状态请求（电平；内核经任务边界沿同步）。
void motor_rt_set_agent_connected(bool connected);

// ---- 只读状态副本（ROS 桥轮询发布用；内部短临界区）----

// 返回实际拷贝的条数（= 当前注册表条目数，受 capacity 截断）。
size_t motor_rt_copy_motion_states(motor_control_motion_state_t *out,
                                   size_t capacity);
size_t motor_rt_copy_statuses(motor_control_status_t *out, size_t capacity);

// 拓扑条目副本（协议族枚举与 MotorTopologyEntry.msg 对齐）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  uint8_t protocol; // 0=VAXOR 1=CERB
  char model_name[32];
  uint32_t capability_flags;
  bool online;
  bool conflict;
  uint8_t lifecycle;
} motor_rt_topology_entry_t;
size_t motor_rt_copy_topology(motor_rt_topology_entry_t *out, size_t capacity);

// 参数视图副本（param_bridge 注册/同步与存储任务 save 快照共用；短临界区）。
size_t motor_rt_copy_param_views(motor_control_param_view_t *out,
                                 size_t capacity);

// 取走内核事件环（motor_control 内，深度 MOTOR_CONTROL_MAX_EVENTS）中的事件：
// 本任务生产、ROS 桥在轮询里消费，两侧共用临界区。满则环内丢最旧并计数
// （ctx.event_dropped_count）。类型复用内核事件（无 ROS 依赖）。
size_t motor_rt_drain_events(motor_control_event_t *out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_RT_TASK_H
