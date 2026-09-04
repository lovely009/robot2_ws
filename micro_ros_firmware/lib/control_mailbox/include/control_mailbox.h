// 固定容量的跨任务控制输入交换层。
// 不依赖 HAL、RTOS、Arduino、ROS 或任何电机协议；调用方负责提供短临界区。

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
  uint32_t config_rejected_count;

  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_edge;
  bool agent_state_seen;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_edge;
} control_mailbox_t;

void control_mailbox_init(control_mailbox_t *mailbox);

// 高频输入：完整快照 latest-wins。未授权或 Agent 已断开时拒绝新命令。
bool control_mailbox_submit_command(
    control_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot);
bool control_mailbox_take_command(control_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot);
void control_mailbox_clear_command(control_mailbox_t *mailbox);

// 低频输入：有界 FIFO，不覆盖、不丢弃已经入队的旧配置请求。
bool control_mailbox_submit_config(
    control_mailbox_t *mailbox,
    const control_mailbox_config_request_t *request);
bool control_mailbox_take_config(
    control_mailbox_t *mailbox,
    control_mailbox_config_request_t *request);
void control_mailbox_clear_config(control_mailbox_t *mailbox);

// 连接/授权是状态请求；take_events 返回状态电平和一次性边沿。
void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized);
void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                             bool connected);
bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MAILBOX_H
