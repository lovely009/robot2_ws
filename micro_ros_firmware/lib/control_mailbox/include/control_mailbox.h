// 固定容量控制输入 mailbox：不依赖 HAL、RTOS、Arduino 或 ROS。
// 调用方负责在并发访问时提供短 critical section。

#ifndef CONTROL_MAILBOX_H
#define CONTROL_MAILBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  motor_control_command_t command;
  uint32_t received_ms;
  uint32_t sequence;
  bool pending;
} control_mailbox_command_slot_t;

typedef struct {
  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_edge;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_edge;
} control_mailbox_events_t;

typedef struct {
  char joint_names[MOTOR_CONTROL_MAX_MOTORS]
                  [MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  control_mailbox_command_slot_t command_slots[MOTOR_CONTROL_MAX_MOTORS];
  motor_control_tx_action_t debug_frame;
  size_t joint_count;
  uint32_t next_sequence;
  uint32_t command_overwrite_count;
  uint32_t invalid_command_count;
  uint32_t debug_submit_count;
  uint32_t debug_overwrite_count;
  uint32_t invalid_debug_frame_count;
  uint32_t debug_sequence;
  bool debug_pending;
  bool authorization_requested;
  bool authorization_pending;
  bool authorization_disable_edge;
  bool agent_state_seen;
  bool agent_connected_requested;
  bool agent_state_pending;
  bool agent_disconnect_edge;
} control_mailbox_t;

void control_mailbox_init(control_mailbox_t *mailbox,
                          const motor_control_motor_config_t *motors,
                          size_t motor_count);
bool control_mailbox_submit_command(control_mailbox_t *mailbox,
                                    const motor_control_command_t *command,
                                    uint32_t received_ms);
bool control_mailbox_take_command(control_mailbox_t *mailbox, size_t index,
                                  motor_control_command_t *command,
                                  uint32_t *received_ms, uint32_t *sequence);
void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized);
void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                              bool connected);
bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events);
bool control_mailbox_submit_debug_frame(
    control_mailbox_t *mailbox, const motor_control_tx_action_t *frame);
bool control_mailbox_take_debug_frame(control_mailbox_t *mailbox,
                                      motor_control_tx_action_t *frame,
                                      uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MAILBOX_H
