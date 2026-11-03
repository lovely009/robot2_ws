#include "agent_protocol.h"
#include "agent_control_gate.h"
#include "motor_control_internal.h"
#include <cassert>
#include <cstdint>
#include <cstring>

// Explicit fixture authority; empty command heartbeats never create it.
static void fixture_lease(agent_control_gate_t &gate, uint32_t now = 100) {
  const auto session = gate.authority.session_id;
  const auto id = gate.authority.lease_id ? gate.authority.lease_id + 1 : session * 1000000ULL + 1;
  assert(control_lease_submit(&gate.authority, session, id, 1, now + 250, true, now));
  // Fixture starts with no old kernel intent; production services this handoff.
  (void)control_lease_take_revocation(&gate.authority);
}

static void test_unarmed_gate_preserves_segmented_identity_read() {
  const motor_protocol_adapter_t *adapters[] = {&motor_protocol_cerb_adapter};
  motor_control_context_t ctx{};
  control_mailbox_t mailbox{};
  agent_control_gate_t gate{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, adapters, 1);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  agent_gate_service(&gate, &mailbox, &ctx, 99, false);

  motor_protocol_raw_frame_t frame{};
  frame.bus_index = 0; frame.can_id = 0x301; frame.dlc = 12;
  frame.is_fd = true; frame.is_brs = true; frame.rx_tick_ms = 100;
  motor_control_on_raw_frame(&ctx, &frame);
  auto step = [&](uint32_t tick, uint8_t expected) {
    agent_gate_service(&gate, &mailbox, &ctx, tick, false);
    motor_control_service(&ctx, tick);
    motor_protocol_tx_frame_t tx[4]{};
    assert(motor_control_collect_tx(&ctx, tx, 4) == 1);
    assert(tx[0].can_id == 0x601 && tx[0].data[0] == expected);
    assert(!ctx.authorized && !ctx.entries[0].motion_enabled);
  };
  step(101, 0x40);

  // 显式停止仍中止身份事务；普通未授权电平不应反复执行停止。
  auto stopped = ctx;
  auto stopped_gate = gate;
  auto stopped_mailbox = mailbox;
  agent_gate_request_stop(&stopped_gate);
  assert(agent_gate_service(&stopped_gate, &stopped_mailbox, &stopped, 102, false)
      & AGENT_LINK_REVOKE);
  assert(!stopped.entries[0].tx_active && !stopped.authorized && !stopped.entries[0].policy_enable);

  frame.can_id = 0x581; frame.dlc = 8; frame.rx_tick_ms = 102;
  const uint8_t initiate[] = {0x41, 0x08, 0x10, 0, 10, 0, 0, 0};
  std::memcpy(frame.data, initiate, 8);
  motor_control_on_raw_frame(&ctx, &frame);
  step(103, 0x60);
  frame.rx_tick_ms = 104;
  const uint8_t first[] = {0, 'R', '4', '8', 'L', 'I', 'T', 'E'};
  std::memcpy(frame.data, first, 8);
  motor_control_on_raw_frame(&ctx, &frame);
  step(105, 0x70);
  frame.rx_tick_ms = 106;
  const uint8_t last[] = {0x19, '_', 'V', '2', 0, 0, 0, 0};
  std::memcpy(frame.data, last, 8);
  motor_control_on_raw_frame(&ctx, &frame);
  assert(ctx.entries[0].profile != nullptr);
  assert(std::strcmp(ctx.entries[0].profile->model_name, "cerb_r48_101") == 0);
  assert(!ctx.authorized && !gate.kernel_authorized && !gate.link.authorized);
  assert(!ctx.entries[0].motion_enabled);
}

static void test_sparse_enable_does_not_authorize_unselected_discovery() {
  const motor_protocol_adapter_t *adapters[] = {&motor_protocol_cerb_adapter};
  motor_control_context_t ctx{};
  control_mailbox_t mailbox{};
  agent_control_gate_t gate{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, adapters, 1);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  for (uint16_t id = 1; id <= 2; ++id) {
    motor_protocol_raw_frame_t frame{};
    frame.can_id = 0x300 + id; frame.dlc = 12;
    frame.is_fd = true; frame.rx_tick_ms = 100;
    motor_control_on_raw_frame(&ctx, &frame);
    auto &entry = ctx.entries[id - 1];
    entry.profile = motor_control_profile_at(1);
    entry.dev_raw_present |= (1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX);
    entry.dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX] = 300;
    entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
  }
  control_command_snapshot_t command{};
  command.session_id = 123; command.sequence = 1;
  command.received_tick_ms = 100; command.valid_until_tick_ms = 130;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  control_policy_snapshot_t policy{};
  policy.policy_generation = 1; policy.valid_until_tick_ms = 200;
  policy.target_count = 1; policy.targets[0].motor_id = 1;
  policy.targets[0].enabled = true;
  policy.targets[0].confirm_unprotected = true;
  policy.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &policy, 100));
  agent_gate_step(&gate, &mailbox, &ctx, 101, true);
  assert(ctx.authorized && ctx.entries[0].policy_enable);
  command.sequence = 2; command.target_count = 1;
  command.targets[0].motor_id = 2;
  command.targets[0].position = 0.5f;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 102));
  agent_gate_step(&gate, &mailbox, &ctx, 102, true);
  assert(!ctx.entries[1].policy_enable);
  assert(!ctx.entries[1].has_target && !ctx.entries[1].motion_enabled);
  motor_protocol_tx_frame_t tx[8]{};
  const size_t count = motor_control_collect_tx(&ctx, tx, 8);
  for (size_t i = 0; i < count; ++i) assert(tx[i].tx_class != MOTOR_PROTOCOL_TX_CLASS_P1 ||
      tx[i].can_id != (MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE | 2U));
}

static void test_deadline_window_and_request_ttl() {
  assert(!agent_deadline_valid(100, 100));
  assert(agent_deadline_valid(100, 101));
  assert(agent_deadline_valid(100, 130));
  assert(!agent_deadline_valid(100, 131));
  assert(agent_deadline_valid(UINT32_MAX - 4, 5));
  uint32_t deadline = 0;
  assert(agent_request_deadline(1000000000, 100, 100, 1030000000, 500, &deadline));
  assert(deadline == 570);
  assert(!agent_request_deadline(1000000000, 100, 100, 1100000000, 500, &deadline));
  assert(!agent_request_deadline(1021000000, 100, 100, 1000000000, 500, &deadline));
  assert(agent_request_deadline(1020000000, 100, 100, 1000000000, 500, &deadline));
  assert(deadline == 600); // 未来容差不额外延长本地 TTL
}
static void test_stop_barrier_includes_queued_policy() {
  agent_link_t link{};
  agent_link_reset(&link);
  agent_link_poll(&link, 0, true, true);
  agent_link_accept_command(&link, 1, 0);
  assert(agent_link_accept_policy(&link, 5, 0));
  assert(agent_link_accept_policy(&link, 99, 0)); // 已受理但尚未执行
  agent_link_request_stop(&link);
  assert(!agent_link_accept_policy(&link, 100, 0)); // 停止锁存不能被抢过
  assert(agent_link_poll(&link, 5, true, true) & AGENT_LINK_REVOKE);
  assert(link.barrier_generation == 99 && link.stop_generation == 1);
  agent_link_accept_command(&link, 2, 5);
  assert(!link.authorized);
  assert(!agent_link_accept_policy(&link, 99, 5));
  assert(agent_link_accept_policy(&link, 100, 5));
}
static void test_lease_soft_stop_then_revoke_and_clock_loss() {
  agent_link_t link{};
  agent_link_reset(&link);
  agent_link_poll(&link, 0, true, true);
  agent_link_accept_command(&link, 1, 0);
  assert(agent_link_accept_policy(&link, 1, 0));
  assert(agent_link_poll(&link, 30, true, true) == 0);
  assert(agent_link_poll(&link, 31, true, true) & AGENT_LINK_SOFT_STOP);
  assert(link.authorized);
  assert(agent_link_poll(&link, 101, true, true) & AGENT_LINK_REVOKE);
  agent_link_accept_command(&link, 2, 102);
  assert(!link.authorized);
  assert(agent_link_accept_policy(&link, 2, 102));
  assert(agent_link_poll(&link, 103, true, false) & AGENT_LINK_REVOKE);
  assert(!link.authorized);
}
static void test_real_gate_stop_bypasses_full_fifo_and_retired_policy() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  control_command_snapshot_t command{}; // 空帧也过完全相同的会话/序号门
  command.session_id = 122;
  command.sequence = 1;
  command.received_tick_ms = 100;
  command.valid_until_tick_ms = 130;
  command.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 100));
  command.session_id = 123;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  command.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 100));
  control_policy_snapshot_t policy{};
  policy.request_id = 1;
  policy.received_tick_ms = 100;
  policy.valid_until_tick_ms = 200;
  policy.policy_generation = 900;
  policy.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &policy, 100));
  for (unsigned i = 0; i < CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY; ++i) {
    control_mailbox_config_request_t request{};
    request.request_id = 10 + i;
    request.kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
    request.motor_id = 1;
    assert(control_mailbox_submit_config(&mailbox, &request));
  }
  ctx.entry_count = 1;
  ctx.entries[0].used = true;
  ctx.entries[0].motor_id = 1;
  ctx.entries[0].policy_enable = true;
  agent_gate_request_stop(&gate);
  agent_gate_step(&gate, &mailbox, &ctx, 105, true);

  assert(!ctx.entries[0].policy_enable);
  assert(!ctx.authorized && !mailbox.policy.pending);
  assert(mailbox.config_count == CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY);
  assert(gate.link.stop_generation == 1 && gate.link.barrier_generation == 900);
  command.sequence = 2;
  command.received_tick_ms = 106;
  command.valid_until_tick_ms = 136;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 106));
  policy.policy_generation = 900;
  policy.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_policy(&gate, &mailbox, &policy, 106));
  agent_gate_step(&gate, &mailbox, &ctx, 110, true);
  assert(!ctx.authorized); // 连接电平与合法心跳均不得复权
  fixture_lease(gate, 110);
  policy.policy_generation = 901;
  ctx.entry_count = 1;
  ctx.entries[0].used = true;
  ctx.entries[0].motor_id = 1;
  policy.target_count = 1;
  policy.targets[0].motor_id = 1;
  policy.targets[0].enabled = true;
  policy.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &policy, 110));
  agent_gate_step(&gate, &mailbox, &ctx, 115, true);
  assert(ctx.authorized);
  agent_gate_service(&gate, &mailbox, &ctx, 116, false);
  assert(!ctx.authorized && gate.link.barrier_generation == 901);
}

static void test_expired_mailbox_input_does_not_refresh_kernel_lease() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  control_command_snapshot_t command{};
  command.session_id = 123;
  command.sequence = 1;
  command.received_tick_ms = 100;
  command.valid_until_tick_ms = 101;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  agent_gate_step(&gate, &mailbox, &ctx, 105, true);
  assert(!ctx.have_snapshot && ctx.snapshot_accepted_count == 0);
  agent_gate_begin_session(&gate, &mailbox, 124);
  fixture_lease(gate);
  command.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 105));
  assert(mailbox.config_count == 0 && !mailbox.policy.pending);
}

static void test_disconnect_reconnect_between_service_ticks_still_revokes() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  agent_link_accept_command(&gate.link, 1, 100);
  assert(agent_link_accept_policy(&gate.link, 7, 100));
  gate.kernel_authorized = true;
  assert(motor_control_set_authorization(&ctx, true, 100));
  control_mailbox_request_agent_connected(&mailbox, false);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 101, true);
  assert(!ctx.authorized);
  assert(gate.link.barrier_generation == 7);
}

static void test_policy_queue_never_overwrites_and_expiry_reports_failure() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 9);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  agent_link_accept_command(&gate.link, 1, 100);
  control_policy_snapshot_t p{};
  p.request_id = 1; p.policy_generation = 1;
  p.valid_until_tick_ms = 101; p.target_count = 1; p.targets[0].motor_id = 1;
  p.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &p, 100));
  p.request_id = 2; p.policy_generation = 2;
  p.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_policy(&gate, &mailbox, &p, 100));
  agent_gate_step(&gate, &mailbox, &ctx, 105, true);
  motor_control_event_t event[4]{};
  assert(motor_control_drain_events(&ctx, event, 4) == 1);
  assert(event[0].request_id == 1 && event[0].reason != MOTOR_PROTOCOL_CONFIG_OK);
  ctx.entry_count = 1; ctx.entries[0].used = true;
  ctx.entries[0].motor_id = 1; ctx.entries[0].policy_enable = true;
  agent_gate_request_disable(&gate, 1);
  agent_gate_service(&gate, &mailbox, &ctx, 106, true);
  assert(!ctx.entries[0].policy_enable);
}

// 独立稀疏失能回归（spec §6：全禁用/失能走独立停止路径，不占配置事务槽）：
// config FIFO 已满、且存在失能前已受理但未执行的更高代使能策略时，disable 只
// 关闭目标电机，不清空 FIFO，并丢弃那条待执行使能；旧策略代数与心跳都不能复活
// 该电机，只有严格高于 barrier 的新策略在有效租约内才能重新使能。
static void test_sparse_disable_full_fifo_discards_pending_enable() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);

  // 两个测试电机走真实型号目录（内核策略门禁要求已绑定 profile）。
  const motor_protocol_model_profile_t *profile = motor_control_profile_at(1);
  assert(profile != nullptr);
  ctx.entry_count = 2;
  for (size_t i = 0; i < ctx.entry_count; ++i) {
    motor_control_entry_t &entry = ctx.entries[i];
    entry.used = true;
    entry.motor_id = (uint16_t)(i + 1U);
    entry.node = (uint16_t)(i + 1U);
    entry.adapter = profile->protocol;
    entry.profile = profile;
    entry.dev_raw_present |= (1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX);
    entry.dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX] = 300;
    entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    entry.online = true;
    entry.policy_enable = true;
    entry.motion_authorized = true;
  }

  // 建立运动租约；随后用现有的 config 提交接口把 16 槽 FIFO 填满。
  control_command_snapshot_t command{};
  command.session_id = 123;
  command.sequence = 1;
  command.received_tick_ms = 100;
  command.valid_until_tick_ms = 130;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  for (unsigned i = 0; i < CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY; ++i) {
    control_mailbox_config_request_t request{};
    request.request_id = 10 + i;
    request.kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
    request.motor_id = 1;
    assert(control_mailbox_submit_config(&mailbox, &request));
  }
  assert(mailbox.config_count == CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY);

  // 更高代使能策略先被受理入邮箱，但尚未执行；紧接着请求只失能 motor1。
  control_policy_snapshot_t pending_enable{};
  pending_enable.request_id = 600;
  pending_enable.received_tick_ms = 100;
  pending_enable.valid_until_tick_ms = 200;
  pending_enable.policy_generation = 500;
  pending_enable.target_count = 1;
  pending_enable.targets[0].motor_id = 1;
  pending_enable.targets[0].enabled = true;
  pending_enable.targets[0].kp = 1.0f;
  pending_enable.targets[0].kd = 0.1f;
  pending_enable.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &pending_enable, 100));
  assert(mailbox.policy.pending);
  agent_gate_request_disable(&gate, UINT64_C(1)); // motor_id 1 = bit 0
  agent_gate_step(&gate, &mailbox, &ctx, 101, true);

  // 稀疏失能：motor1 关闭、motor2 不变；FIFO 原样保留；停止代数前进，
  // barrier 覆盖已受理的最高策略代数；待执行使能策略被丢弃并回 INVALID 结果。
  assert(!ctx.entries[0].policy_enable);
  assert(ctx.entries[1].policy_enable);
  assert(!mailbox.policy.pending);
  assert(mailbox.config_count == CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY);
  assert(mailbox.config_queue[mailbox.config_head].request_id == 10);
  assert(gate.link.stop_generation == 1);
  assert(gate.link.highest_policy_generation == 500);
  assert(gate.link.barrier_generation == 500);
  motor_control_event_t events[16]{};
  size_t event_count = motor_control_drain_events(&ctx, events, 16);
  bool discarded = false;
  for (size_t i = 0; i < event_count; ++i) {
    if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT &&
        events[i].request_id == 600 && events[i].motor_id == 1 &&
        events[i].reason == MOTOR_PROTOCOL_CONFIG_INVALID) {
      discarded = true;
    }
  }
  assert(discarded);

  // 旧队列策略（代数等于 barrier）+ 心跳：都不得复活 motor1。
  command.sequence = 2;
  command.received_tick_ms = 103;
  command.valid_until_tick_ms = 133;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 103));
  control_policy_snapshot_t old_policy = pending_enable;
  old_policy.request_id = 601;
  old_policy.received_tick_ms = 103;
  old_policy.valid_until_tick_ms = 203;
  old_policy.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &old_policy, 103));
  assert(mailbox.policy.pending);
  agent_gate_step(&gate, &mailbox, &ctx, 104, true);
  assert(!ctx.entries[0].policy_enable);
  assert(ctx.entries[1].policy_enable);
  assert(!mailbox.policy.pending);
  assert(gate.link.stop_generation == 1 && gate.link.barrier_generation == 500);

  // 租约过 30 ms 后，严格更高代的新策略在提交阶段即被拒（不得靠复活绕租约）。
  control_policy_snapshot_t fresh_enable = pending_enable;
  fresh_enable.request_id = 602;
  fresh_enable.received_tick_ms = 134;
  fresh_enable.valid_until_tick_ms = 234;
  fresh_enable.policy_generation = 501;
  fresh_enable.lease_id = gate.authority.lease_id;
  assert(!agent_gate_submit_policy(&gate, &mailbox, &fresh_enable, 134));
  assert(!mailbox.policy.pending);
  agent_gate_step(&gate, &mailbox, &ctx, 134, true);
  assert(!gate.link.lease_valid);
  assert(!ctx.entries[0].policy_enable);

  // 新心跳重建租约后，同一代 501 策略才被受理并在执行拍重新武装 motor1。
  command.sequence = 3;
  command.received_tick_ms = 135;
  command.valid_until_tick_ms = 165;
  command.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 135));
  fresh_enable.received_tick_ms = 136;
  fresh_enable.valid_until_tick_ms = 236;
  fresh_enable.lease_id = gate.authority.lease_id;
  assert(agent_gate_submit_policy(&gate, &mailbox, &fresh_enable, 136));
  assert(mailbox.policy.pending);
  agent_gate_step(&gate, &mailbox, &ctx, 137, true);
  assert(gate.link.lease_valid);
  assert(ctx.entries[0].policy_enable);
  assert(ctx.entries[0].policy_generation == 501);
  assert(ctx.entries[1].policy_enable);
  assert(!mailbox.policy.pending);
  // 重新使能不是新的停止事件：barrier/停止代数保持失能时的值。
  assert(gate.link.stop_generation == 1 && gate.link.barrier_generation == 500);
}

static void test_snapshot_renews_only_existing_authority_atomically() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  agent_gate_begin_session(&gate, &mailbox, 123);
  fixture_lease(gate);
  gate.link.connected = gate.link.clock_valid = true;
  control_command_snapshot_t command{};
  command.session_id = 123;
  command.lease_id = gate.authority.lease_id;
  command.sequence = 1;
  command.valid_until_tick_ms = 130;
  command.lease_sequence = 2;
  command.lease_valid_until_tick_ms = 300;
  command.target_count = 1;
  command.targets[0].motor_id = 2;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  assert(gate.authority.sequence == 2);
  assert(gate.authority.valid_until_tick_ms == 300);
  assert(mailbox.command.pending);
  // A replay with a new lease sequence must not refresh either deadline.
  command.lease_sequence = 3;
  command.lease_valid_until_tick_ms = 400;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 110));
  assert(gate.authority.valid_until_tick_ms == 300);
  command.sequence = 2;
  command.targets[0].position = __builtin_nanf("");
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 110));
  assert(gate.authority.sequence == 2);
  command.targets[0].position = 0;
  command.lease_valid_until_tick_ms = 500;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 110));
  command.lease_valid_until_tick_ms = 300;
  ++command.lease_id;
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 110));
  --command.lease_id;
  control_lease_revoke(&gate.authority);
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 110));
}

static void test_authority_heartbeat_does_not_refresh_motion_deadline() {
  agent_control_gate_t gate{};
  control_mailbox_t mailbox{};
  motor_control_context_t ctx{};
  control_mailbox_init(&mailbox);
  motor_control_init(&ctx, nullptr, 0);
  agent_gate_begin_session(&gate, &mailbox, 123);
  control_mailbox_request_agent_connected(&mailbox, true);
  agent_gate_service(&gate, &mailbox, &ctx, 100, true);
  fixture_lease(gate);
  control_command_snapshot_t command{};
  command.session_id = 123; command.sequence = 1;
  command.valid_until_tick_ms = 130;
  assert(agent_gate_submit_command(&gate, &mailbox, &command, 100));
  agent_gate_admit_command(&gate, &mailbox, &ctx, 100);
  const auto deadline = ctx.snapshot_deadline_tick_ms;
  assert(control_lease_submit(&gate.authority, 123, gate.authority.lease_id,
      2, 350, true, 120));
  assert(agent_gate_service(&gate, &mailbox, &ctx, 131, true) & AGENT_LINK_SOFT_STOP);
  assert(ctx.snapshot_deadline_tick_ms == deadline);
  assert(gate.link.command_tick_ms == 100);
  // Losing the link revokes authority; incoming old snapshots cannot restore it.
  control_mailbox_request_agent_connected(&mailbox, false);
  agent_gate_service(&gate, &mailbox, &ctx, 132, true);
  assert(!gate.authority.active);
  assert(!agent_gate_submit_command(&gate, &mailbox, &command, 133));
}

int main() {
  test_snapshot_renews_only_existing_authority_atomically();
  test_authority_heartbeat_does_not_refresh_motion_deadline();
  for (unsigned op = 0; op < 256; ++op) {
    const bool supported = op == 0 || op == 2 || op == 3 || op == 4 || op == 6 || op == 7 || op == 8;
    assert(agent_config_operation_supported((uint8_t)op) == supported);
  }
  test_unarmed_gate_preserves_segmented_identity_read();
  test_sparse_enable_does_not_authorize_unselected_discovery();
  test_sparse_disable_full_fifo_discards_pending_enable();
  test_policy_queue_never_overwrites_and_expiry_reports_failure();
  test_disconnect_reconnect_between_service_ticks_still_revokes();
  test_real_gate_stop_bypasses_full_fifo_and_retired_policy();
  test_expired_mailbox_input_does_not_refresh_kernel_lease();
  test_deadline_window_and_request_ttl();
  test_stop_barrier_includes_queued_policy();
  test_lease_soft_stop_then_revoke_and_clock_loss();
}
