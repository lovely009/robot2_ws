#include "agent_control_gate.h"
#include "session_sync.h"
#include <string.h>

void agent_gate_begin_session(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, uint64_t session_id) {
  memset(gate, 0, sizeof(*gate));
  wire_command_guard_reset(&gate->command_guard, session_id);
  control_mailbox_clear_command(mailbox);
  control_mailbox_clear_policy(mailbox);
  control_mailbox_clear_trajectory(mailbox);
  control_mailbox_clear_config(mailbox);
  gate->session_pending = true;
}

void agent_gate_request_stop(agent_control_gate_t *gate) {
  agent_link_request_stop(&gate->link);
}

void agent_gate_request_disable(agent_control_gate_t *gate, uint64_t motor_mask) {
  gate->disable_pending_mask |= motor_mask;
}

static void reject_policy(motor_control_context_t *ctx,
    const control_policy_snapshot_t *policy, uint32_t now_ms) {
  for (uint8_t i = 0; i < policy->target_count; ++i)
    motor_control_report_config_result(ctx, policy->request_id,
        policy->targets[i].motor_id, MOTOR_PROTOCOL_CONFIG_INVALID, now_ms);
}

static void discard_policy(control_mailbox_t *mailbox,
    motor_control_context_t *ctx, uint32_t now_ms) {
  static control_policy_snapshot_t pending;
  if (control_mailbox_take_policy(mailbox, &pending))
    reject_policy(ctx, &pending, now_ms);
}

bool agent_gate_submit_command(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, const control_command_snapshot_t *snapshot,
    uint32_t now_ms) {
  if (snapshot == NULL || !gate->link.connected || !gate->link.clock_valid ||
      gate->link.stop_pending || !agent_deadline_valid(now_ms, snapshot->valid_until_tick_ms) ||
      !wire_command_guard_allows(&gate->command_guard, snapshot->session_id, snapshot->sequence)) {
    ++mailbox->command_rejected_count;
    return false;
  }
  // 心跳必须在撤权状态仍可受理，但它不改变运动授权。
  if (!control_command_mailbox_publish(&mailbox->command, snapshot)) return false;
  wire_command_guard_commit(&gate->command_guard, snapshot->sequence);
  agent_link_accept_command(&gate->link, snapshot->sequence, now_ms);
  return true;
}

bool agent_gate_submit_policy(agent_control_gate_t *gate,
    control_mailbox_t *mailbox, const control_policy_snapshot_t *snapshot,
    uint32_t now_ms) {
  if (snapshot == NULL) return false;
  const uint32_t remaining = snapshot->valid_until_tick_ms - now_ms;
  agent_link_t candidate = gate->link;
  if (mailbox->policy.pending || gate->disable_pending_mask != 0U ||
      remaining == 0U || remaining > 100U ||
      !agent_link_accept_policy(&candidate, snapshot->policy_generation, now_ms)) {
    ++mailbox->policy_rejected_count;
    return false;
  }
  if (!control_policy_mailbox_publish(&mailbox->policy, snapshot)) return false;
  bool enables = false;
  for (uint8_t i = 0; i < snapshot->target_count; ++i) enables |= snapshot->targets[i].enabled;
  if (!enables) candidate.authorized = gate->link.authorized;
  // 只在成功入邮箱后推进最高受理代数；包括还没执行的策略。
  gate->link = candidate;
  return true;
}

uint8_t agent_gate_service(agent_control_gate_t *gate, control_mailbox_t *mailbox,
    motor_control_context_t *ctx, uint32_t now_ms, bool clock_valid) {
  control_mailbox_events_t events = {0};
  (void)control_mailbox_take_events(mailbox, &events);
  if (events.agent_disconnect_edge) agent_gate_request_stop(gate);
  if (gate->session_pending) {
    motor_control_stop_all(ctx, now_ms);
    gate->session_pending = false;
  }
  const bool connected = mailbox->agent_state_seen && mailbox->agent_connected_requested;
  const uint8_t action = agent_link_poll(&gate->link, now_ms, connected, clock_valid);
  if (action & AGENT_LINK_REVOKE) {
    motor_control_stop_all(ctx, now_ms);
    gate->kernel_authorized = false;
    control_mailbox_clear_command(mailbox);
    discard_policy(mailbox, ctx, now_ms);
    control_mailbox_clear_trajectory(mailbox);
  }
  if (gate->disable_pending_mask != 0U) {
    motor_control_disable_mask(ctx, gate->disable_pending_mask, now_ms);
    gate->disable_pending_mask = 0U;
    gate->link.barrier_generation = gate->link.highest_policy_generation;
    ++gate->link.stop_generation;
    // 禁用前已排队的策略不得在下一拍重新武装。
    discard_policy(mailbox, ctx, now_ms);
  }
  // session_sync 的授权参数来自显式策略，而不是 Agent 在线电平。
  session_sync_apply(ctx, &events,
      gate->kernel_authorized && gate->link.authorized, now_ms);
  if (action & AGENT_LINK_SOFT_STOP) {
    motor_control_soft_stop_all(ctx, now_ms);
    control_mailbox_clear_trajectory(mailbox);
  }
  return action;
}

void agent_gate_step(agent_control_gate_t *gate, control_mailbox_t *mailbox,
    motor_control_context_t *ctx, uint32_t now_ms, bool clock_valid) {
  (void)agent_gate_service(gate, mailbox, ctx, now_ms, clock_valid);
  // 单一实时写者复用静态转换区，避免 4KB 任务栈上出现两份全容量快照。
  static control_policy_snapshot_t policy_in;
  static motor_control_policy_snapshot_t policy;
  if (control_mailbox_take_policy(mailbox, &policy_in)) {
    const uint32_t remaining = policy_in.valid_until_tick_ms - now_ms;
    if (remaining > 0U && remaining <= 100U && gate->link.lease_valid &&
        policy_in.policy_generation > gate->link.barrier_generation) {
      memset(&policy, 0, sizeof(policy));
      policy.received_tick_ms = policy_in.received_tick_ms;
      policy.request_id = policy_in.request_id;
      policy.policy_generation = policy_in.policy_generation;
      policy.target_count = policy_in.target_count;
      bool enables = false;
      for (uint8_t i = 0; i < policy.target_count; ++i) {
        policy.targets[i].motor_id = policy_in.targets[i].motor_id;
        policy.targets[i].enabled = policy_in.targets[i].enabled;
        policy.targets[i].kp = policy_in.targets[i].kp;
        policy.targets[i].kd = policy_in.targets[i].kd;
        policy.targets[i].confirm_unprotected = policy_in.targets[i].confirm_unprotected;
        enables |= policy.targets[i].enabled;
      }
      if (enables && gate->link.authorized) {
        gate->kernel_authorized = true;
        (void)motor_control_set_authorization(ctx, true, now_ms);
      }
      if (!motor_control_apply_policy(ctx, &policy, now_ms) && enables) {
        gate->kernel_authorized = false;
        (void)motor_control_set_authorization(ctx, false, now_ms);
      }
    } else {
      reject_policy(ctx, &policy_in, now_ms);
    }
  }
  static control_command_snapshot_t command_in;
  static motor_control_command_snapshot_t command;
  if (control_mailbox_take_command(mailbox, &command_in) &&
      agent_deadline_valid(now_ms, command_in.valid_until_tick_ms)) {
    memset(&command, 0, sizeof(command));
    command.sequence = command_in.sequence;
    command.received_tick_ms = command_in.received_tick_ms;
    command.valid_until_tick_ms = command_in.valid_until_tick_ms;
    command.target_count = command_in.target_count;
    for (uint8_t i = 0; i < command.target_count; ++i) {
      command.targets[i].motor_id = command_in.targets[i].motor_id;
      command.targets[i].position_rad = command_in.targets[i].position;
      command.targets[i].velocity_rad_s = command_in.targets[i].velocity;
      command.targets[i].effort_nm = command_in.targets[i].effort;
    }
    (void)motor_control_apply_command_snapshot(ctx, &command, now_ms);
  }
}
