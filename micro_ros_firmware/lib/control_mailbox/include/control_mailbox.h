// 固定容量的跨任务控制输入交换层。
// 不依赖 HAL、RTOS、Arduino、ROS 或任何电机协议。
//
// 并发契约：本层无内部锁、无原子字段。submit/take/clear/request_*/
// take_events 的每一次调用都必须位于调用方提供的临界区内，且写侧与
// 读侧必须共用同一互斥域（单核下 FreeRTOS taskENTER_CRITICAL 或等效
// 关中断即满足；"两侧各自加锁"不构成互斥，会产生数据竞争）。仅支持
// 任务上下文调用（ISR 内 taskENTER_CRITICAL 非法，本层不做检测）；
// 违反契约（如无锁双写者）不属于受支持用法。

#ifndef CONTROL_MAILBOX_H
#define CONTROL_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "control_command_mailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY 16U

// 低频设备配置的语义类型。具体型号/协议能力由运行时和协议适配器判断。
typedef enum {
  CONTROL_MAILBOX_CONFIG_SET_MODE = 1U,
  CONTROL_MAILBOX_CONFIG_SET_GAINS = 2U,
  CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS = 3U,
  CONTROL_MAILBOX_CONFIG_SET_ENABLE = 4U, // 逐电机 Enable（ARD §19 MotorPolicy）
} control_mailbox_config_kind_t;

typedef struct {
  float kp;
  float kd;
} control_mailbox_gains_t;

typedef struct {
  float min_position_rad;
  float max_position_rad;
} control_mailbox_position_limits_t;

typedef struct {
  uint32_t request_id;
  uint32_t received_tick_ms;
  uint16_t motor_id;
  control_mailbox_config_kind_t kind;
  union {
    uint8_t mode;
    bool enable;
    control_mailbox_gains_t gains;
    control_mailbox_position_limits_t position_limits;
  } value;
} control_mailbox_config_request_t;

typedef struct {
  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_edge;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_edge;
} control_mailbox_events_t;

typedef struct {
  control_command_mailbox_t command;
  control_mailbox_config_request_t
      config_queue[CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY];
  uint8_t config_head;
  uint8_t config_count;

  // 拒绝诊断（拒绝必可见，且类别可区分；为生命周期累计，clear 不清零）。
  uint32_t config_invalid_count;   // 请求非法（request_id==0/字段非法/未知 kind）
  uint32_t config_overflow_count;  // 配置 FIFO 满（消费慢于生产）
  uint32_t config_session_count;   // Agent 未连接期的配置提交
  uint32_t command_rejected_count; // 被授权/会话门拒绝的命令提交
                                   //（快照本身非法走 command.invalid_snapshot_count）

  bool authorization_requested;     // 授权电平（init 默认 true = 引导豁免）
  bool authorization_pending;       // 电平变化边沿（一次性，take_events 读后清）
  bool authorization_disable_edge;  // 授权撤销边沿（一次性）
  bool agent_state_seen;            // 是否收到过首次 Agent 状态
  bool agent_connected_requested;   // Agent 连接电平（未见状态前视为连接）
  bool agent_state_pending;         // 连接电平变化边沿（一次性）
  bool agent_disconnect_edge;       // 连接后断开边沿（一次性）
} control_mailbox_t;

void control_mailbox_init(control_mailbox_t *mailbox);

// 高频输入：完整快照 latest-wins。未授权或 Agent 已断开时拒绝新命令
// （计入 command_rejected_count；快照本身非法由 command.invalid_snapshot_count
// 计数，二者不重复）。
bool control_mailbox_submit_command(
    control_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot);
bool control_mailbox_take_command(control_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot);
// 只撤销"未取走"的输入；诊断计数保留为生命周期统计。
void control_mailbox_clear_command(control_mailbox_t *mailbox);

// 低频输入：有界 FIFO，不覆盖、不丢弃已经入队的旧配置请求。
// 配置不属于运动授权：可在禁能/未授权状态下排队（会话门仍生效，
// 且会话门先于数据校验——与 submit_command 同序，断链期拒绝统一
// 计入 config_session_count）。
bool control_mailbox_submit_config(
    control_mailbox_t *mailbox,
    const control_mailbox_config_request_t *request);
bool control_mailbox_take_config(
    control_mailbox_t *mailbox,
    control_mailbox_config_request_t *request);
void control_mailbox_clear_config(control_mailbox_t *mailbox);

// 连接/授权是状态请求：同状态重复调用幂等（不产生新边沿）；
// 断链（曾连接后断开）清命令+配置并撤销授权；会话恢复不恢复授权
// ——恢复必须显式 request_authorization(true)（ARD §33）。
void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized);
void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                             bool connected);
// 返回值 = 自上次调用以来是否出现未读边沿；返回 false 时仍带货当前电平
// ——任务层启动时应先调用一次以同步初始电平。
bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MAILBOX_H
