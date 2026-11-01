// 电机实时任务是内核唯一写者：1ms 有界 RX/TX 服务，50..1000Hz 控制边界（默认200Hz）。
// ROS 桥仅通过 mailbox 提交输入、通过只读副本取状态；无 raw CAN 调试出口。

#ifndef MOTOR_RT_TASK_H
#define MOTOR_RT_TASK_H

#include <stdbool.h>
#include <stddef.h>

#include "control_mailbox.h"
#include "motor_control.h"
#include "agent_trajectory.h"
#include "control_lease.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建实时任务（幂等；mailbox/内核初始化完成后任务延迟自启动 FDCAN）。
// 返回 false = 任务创建失败（调用方应 Error_Handler）。
bool motor_rt_task_start(void);

// ---- mailbox 写入口（ROS 桥/会话层调用；内部短临界区）----

// 提交一份完整命令快照（latest-wins；非法快照由 mailbox 拒绝并计数）。
bool motor_rt_submit_command(const control_command_snapshot_t *snapshot);

// 轨迹入口见 agent_trajectory.h；旧全量分段邮箱不再用于生产。

// 提交一条批量策略（latest-wins；非法由 mailbox 整条拒绝并计数）。
// 不走 config FIFO——那条队列深 16，装不下最多 FIRMWARE_MAX_MOTORS 台的策略消息。
bool motor_rt_submit_policy(const control_policy_snapshot_t *snapshot);

// 低频配置提交，契约见 control_mailbox 配置 FIFO。
bool motor_rt_submit_config(const control_mailbox_config_request_t *request);

// 会话状态请求（电平；内核经任务边界沿同步）。
void motor_rt_set_agent_connected(bool connected);
void motor_rt_begin_agent_session(uint64_t session_id);
void motor_rt_request_stop_all(void);
bool motor_rt_submit_control_lease(uint64_t session_id, uint64_t lease_id,
    uint32_t sequence, uint32_t deadline, bool active);
void motor_rt_copy_control_lease(control_lease_state_t *out);
void motor_rt_request_disable(uint64_t motor_mask);
void motor_rt_set_clock_valid(bool valid);

typedef struct {
  uint32_t command_sequence;
  uint32_t stop_generation;
  uint32_t required_policy_generation;
  bool lease_valid;
  bool authorized;
} motor_rt_link_status_t;
void motor_rt_copy_link_status(motor_rt_link_status_t *out);

// ---- 只读状态副本（ROS 桥轮询发布用；内部短临界区）----

// 返回实际拷贝的条数（= 当前注册表条目数，受 capacity 截断）。
size_t motor_rt_copy_motion_states(motor_control_motion_state_t *out,
    size_t capacity);
size_t motor_rt_copy_statuses(motor_control_status_t *out, size_t capacity);

// 拓扑条目副本（协议族枚举与 MotorTopologyEntry.msg 对齐）。
typedef struct {
  uint16_t motor_id;
  uint8_t bus_index;
  uint8_t protocol; // adapter 注册序：0=VAXOR，1=CERB；新协议编号须追加。
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

// 诊断快照：内核计数器 + 任务层补齐的每总线 RX ring 溢出与堆余量。
// 堆余量由本任务采集（内核不感知堆），ring 溢出由 lib/fdcan 持有。
typedef struct {
  motor_control_diagnostics_t kernel;
  // mailbox 拒绝计数：授权/会话不满足或载荷非法，未进入内核。
  uint32_t mailbox_rejected_command;
  uint32_t mailbox_rejected_trajectory;
  uint32_t mailbox_rejected_policy;
  uint32_t ring_drop[MOTOR_CONTROL_MAX_BUSES];
  uint32_t rx_hw_lost[MOTOR_CONTROL_MAX_BUSES]; // 硬件 FIFO0 MESSAGE_LOST
  uint32_t tx_frame_count[MOTOR_CONTROL_MAX_BUSES]; // 已提交硬件的发送帧数
  uint32_t heap_free;
} motor_rt_diagnostics_t;
void motor_rt_copy_diagnostics(motor_rt_diagnostics_t *out);

// 轨迹执行状态副本（内核类型直接暴露：无协议指针，无需镜像）。
void motor_rt_copy_trajectory_state(motor_control_trajectory_state_t *out);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_RT_TASK_H
