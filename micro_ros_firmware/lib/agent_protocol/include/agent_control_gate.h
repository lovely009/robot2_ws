#ifndef AGENT_CONTROL_GATE_H
#define AGENT_CONTROL_GATE_H

#include "agent_protocol.h"
#include "control_mailbox.h"
#include "motor_control.h"
#include "wire_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

// 与 mailbox 共用调用方临界区；step/service 只由内核唯一写任务调用。
typedef struct {
  agent_link_t link;
  wire_command_guard_t command_guard;
  bool kernel_authorized;
  bool session_pending;
  uint64_t disable_pending_mask;
} agent_control_gate_t;

void agent_gate_begin_session(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, uint64_t session_id);
void agent_gate_request_stop(agent_control_gate_t *gate);
void agent_gate_request_disable(agent_control_gate_t *gate, uint64_t motor_mask);
bool agent_gate_submit_command(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, const control_command_snapshot_t *snapshot,
    uint32_t now_ms);
bool agent_gate_submit_policy(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, const control_policy_snapshot_t *snapshot,
    uint32_t now_ms);
uint8_t agent_gate_service(agent_control_gate_t *gate, control_mailbox_t *mailbox,
    motor_control_context_t *ctx, uint32_t now_ms, bool clock_valid);
void agent_gate_step(agent_control_gate_t *gate, control_mailbox_t *mailbox,
    motor_control_context_t *ctx, uint32_t now_ms, bool clock_valid);

#ifdef __cplusplus
}
#endif
#endif
