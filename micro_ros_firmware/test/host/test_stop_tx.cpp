// Cross-layer regression: a stop must invalidate old commands at both queue
// stages, including CERB commands whose control word would re-enable the drive.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

#include "motor_control_internal.h"
#include "motor_control_profile.h"
#include "motor_tx_router.h"

static can_tx_frame_t to_can(const motor_protocol_tx_frame_t &in) {
  can_tx_frame_t out = {};
  out.can_id = in.can_id;
  out.tx_slot = in.tx_slot;
  out.is_extended = in.is_extended;
  out.is_fd = in.is_fd;
  out.is_brs = in.is_brs;
  out.size = in.size;
  out.node = in.node;
  out.aggregate_only = in.aggregate_only;
  std::memcpy(out.data, in.data, in.size);
  return out;
}

struct Fixture {
  motor_control_context_t kernel = {};
  can_tx_scheduler_t scheduler = {};
  motor_tx_router_t router = {};
  std::vector<can_tx_frame_t> sent;
  const motor_protocol_adapter_t *adapters[1] = {&motor_protocol_cerb_adapter};

  Fixture() {
    motor_control_init(&kernel, adapters, 1U);
    can_tx_scheduler_init(&scheduler);
    motor_tx_router_init(&router);
    motor_control_set_agent_connected(&kernel, true, 100U);
    assert(motor_control_set_authorization(&kernel, true, 100U));
  }

  motor_control_entry_t &add_motor(uint16_t node) {
    auto &entry = kernel.entries[kernel.entry_count++];
    entry.used = entry.online = true;
    entry.motor_id = entry.node = node;
    entry.adapter = &motor_protocol_cerb_adapter;
    for (size_t i = 0U; i < motor_control_profile_count(); ++i) {
      const auto *profile = motor_control_profile_at(i);
      if (std::strcmp(profile->model_name, "cerb_r48_101") == 0) {
        entry.profile = profile;
      }
    }
    assert(entry.profile != nullptr);
    entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
    entry.motion_enabled = entry.motion_authorized = entry.policy_enable = true;
    entry.last_rx_tick_ms = entry.last_motion_rx_tick_ms = 100U;
    entry.identity_read_done = entry.dev_read_done = entry.boot_record_offered = true;
    return entry;
  }

  void command(uint32_t now = 100U) {
    motor_control_command_snapshot_t snapshot = {};
    snapshot.received_tick_ms = now;
    snapshot.sequence = now;
    snapshot.target_count = (uint8_t)kernel.entry_count;
    for (size_t i = 0U; i < kernel.entry_count; ++i) {
      snapshot.targets[i].motor_id = kernel.entries[i].motor_id;
      snapshot.targets[i].position_rad = 1.0f;
    }
    assert(motor_control_apply_command_snapshot(&kernel, &snapshot, now));
  }

  void disable(uint16_t motor_id, bool policy = false) {
    motor_protocol_config_request_t request = {};
    request.motor_id = motor_id;
    request.request_id = 1U;
    request.kind = policy ? MOTOR_PROTOCOL_CONFIG_SET_POLICY
                          : MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
    assert(motor_control_submit_config(&kernel, &request, 101U));
  }

  void arm(uint16_t motor_id) {
    motor_protocol_config_request_t request{};
    request.motor_id = motor_id;
    request.request_id = 2;
    request.kind = MOTOR_PROTOCOL_CONFIG_SET_POLICY;
    request.policy_generation = 1;
    request.enable = true;
    assert(motor_control_submit_config(&kernel, &request, 102));
  }

  // The motor_rt task owns both queues: apply latched cancellations before
  // moving its bounded batch, then pump. No hardware or ROS is involved.
  void route() {
    can_tx_scheduler_cancel_slots(&scheduler,
        motor_control_take_tx_cancellations(&kernel));
    motor_protocol_tx_frame_t frames[MOTOR_TX_ROUTER_LOGICAL_CAPACITY];
    const size_t count = motor_control_collect_tx(&kernel, frames,
        MOTOR_TX_ROUTER_LOGICAL_CAPACITY);
    for (size_t i = 0U; i < count; ++i) {
      const can_tx_frame_t frame = to_can(frames[i]);
      if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0) {
        if (!can_tx_scheduler_submit_p0(&scheduler, &frame)) {
          (void)motor_control_requeue_p0(&kernel, &frames[i]);
        }
      } else if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P1) {
        assert(can_tx_scheduler_submit_p1(&scheduler, frames[i].tx_slot, &frame));
      } else {
        (void)can_tx_scheduler_submit_p2(&scheduler, &frame);
      }
    }
  }

  static bool capture(void *context, uint8_t, const can_tx_frame_t *frame) {
    static_cast<Fixture *>(context)->sent.push_back(*frame);
    return true;
  }

  void pump() { (void)motor_tx_router_pump(&router, &scheduler, 0U, capture, this); }

  void assert_single_disable() const {
    assert(sent.size() == 1U);
    assert(sent[0].can_id == 0x111U);
    assert(sent[0].data[0] == 0x0CU);
  }
};

enum class Stop { Revoke, Disconnect, Disable, Policy, Fault, Offline, Conflict };

static void apply_stop(Fixture &f, Stop stop) {
  switch (stop) {
  case Stop::Revoke:
    assert(motor_control_set_authorization(&f.kernel, false, 101U));
    break;
  case Stop::Disconnect:
    motor_control_set_agent_connected(&f.kernel, false, 101U);
    break;
  case Stop::Disable:
  case Stop::Policy:
    f.disable(1U, stop == Stop::Policy);
    break;
  case Stop::Fault:
  case Stop::Conflict: {
    motor_protocol_raw_frame_t frame = {};
    frame.can_id = 0x301U;
    frame.is_fd = true;
    frame.dlc = 12U;
    frame.rx_tick_ms = 101U;
    frame.bus_index = stop == Stop::Conflict ? 1U : 0U;
    frame.data[11] = stop == Stop::Fault ? 0x20U : 0U;
    motor_control_on_raw_frame(&f.kernel, &frame);
    break;
  }
  case Stop::Offline:
    motor_control_service(&f.kernel, 201U);
    break;
  }
  assert(!f.kernel.entries[0].motion_enabled);
}

static void test_stop_cancels_kernel_and_scheduler_commands() {
  for (const auto stop : {Stop::Revoke, Stop::Disconnect, Stop::Disable,
       Stop::Policy, Stop::Fault, Stop::Offline, Stop::Conflict}) {
    for (const bool already_routed : {false, true}) {
      Fixture f;
      f.add_motor(1U);
      f.command();
      if (already_routed) {
        f.route();
        assert(f.scheduler.p1.dirty[0]);
        f.command(); // Old commands now exist at both stages simultaneously.
      }
      apply_stop(f, stop);
      f.route();
      f.pump(); // P0 disable is sent first.
      f.pump(); // Before the fix this sent stale 0xCC and re-enabled CERB.
      f.pump();
      f.assert_single_disable();
      assert(!f.scheduler.p1.dirty[0]);
      assert(motor_control_take_tx_cancellations(&f.kernel) == 0U);
    }
  }
}

static void test_stop_also_cancels_an_unsent_enable() {
  for (const bool already_routed : {false, true}) {
    Fixture f;
    auto &entry = f.add_motor(1U);
    entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    entry.motion_enabled = false;
    f.command(); // Real CERB enable with its 100ms settling gate.
    assert(entry.enable_pending);
    if (already_routed) {
      f.route();
      assert(f.scheduler.p0.count == 1U);
    }
    f.disable(1U);
    f.route();
    f.pump();
    f.pump();
    f.assert_single_disable();
  }
}

static void test_soft_stop_discards_old_position_target() {
  Fixture f;
  f.add_motor(1U);
  f.command();
  f.route();
  motor_control_service(&f.kernel, 131U); // 30ms snapshot timeout.
  assert(f.kernel.entries[0].lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  f.route();
  f.pump();
  f.pump();
  assert(f.sent.size() == 1U);
  motor_protocol_tx_frame_t expected = {};
  const auto &entry = f.kernel.entries[0];
  assert(entry.adapter->encode_soft_stop(entry.profile, entry.node, 0U, &expected));
  assert(std::memcmp(f.sent[0].data, expected.data, expected.size) == 0);
}

static void test_full_p0_queue_cannot_preserve_the_old_target() {
  Fixture f;
  auto &first = f.add_motor(1U);
  auto &other = f.add_motor(2U);
  f.command();
  f.route();
  motor_protocol_tx_frame_t unrelated = {};
  assert(other.adapter->encode_disable(other.profile, other.node, 0U, &unrelated));
  unrelated.tx_slot = 1U;
  const auto queued = to_can(unrelated);
  for (size_t i = 0U; i < CAN_TX_P0_CAPACITY; ++i) {
    assert(can_tx_scheduler_submit_p0(&f.scheduler, &queued));
  }
  f.disable(first.motor_id);
  f.route(); // New disable cannot enter P0 yet, but cancellation must take effect.
  assert(!f.scheduler.p1.dirty[0]);
  assert(f.kernel.tx_count == 1U); // Disable is requeued, not dropped.
  for (size_t i = 0U; i < 5U; ++i) {
    f.pump();
    f.route();
  }
  size_t disables = 0U;
  for (const auto &frame : f.sent) {
    if (frame.can_id == 0x111U) {
      assert(frame.data[0] == 0x0CU);
      ++disables;
    }
  }
  assert(disables == 1U);
}

static void test_cancel_even_when_disable_encoding_fails() {
  Fixture f;
  auto &entry = f.add_motor(1U);
  f.command();
  f.route();
  motor_protocol_adapter_t failing = motor_protocol_cerb_adapter;
  failing.encode_disable = nullptr;
  entry.adapter = &failing;
  f.disable(1U);
  assert(entry.p0_pending);
  f.route();
  f.pump();
  f.pump();
  assert(f.sent.empty());
  assert(!f.scheduler.p1.dirty[0]);
}

static void test_cerb_high_node_aggregation_excludes_stopped_axis() {
  Fixture f;
  f.add_motor(20U);
  f.add_motor(21U);
  f.add_motor(22U);
  f.command();
  f.route();
  f.disable(20U);
  f.route();
  f.pump(); // High-node disable is a single-slot 0x210 frame.
  f.pump(); // Remaining two nodes may still aggregate and move.
  f.pump();
  assert(f.sent.size() == 2U);
  assert(f.sent[0].can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(f.sent[0].data[56] == 20U && f.sent[0].data[0] == 0x0CU);
  assert(f.sent[1].can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(f.sent[1].data[56] == 21U && f.sent[1].data[57] == 22U);
  assert(f.sent[1].data[58] == 0U);
  assert(f.sent[1].data[0] == 0xCCU && f.sent[1].data[9] == 0xCCU);
}

static void test_explicit_recovery_can_send_a_fresh_enable() {
  Fixture f;
  f.add_motor(1U);
  f.command();
  f.route();
  apply_stop(f, Stop::Revoke);
  f.route();
  f.pump();
  f.pump();
  f.assert_single_disable();
  assert(motor_control_set_authorization(&f.kernel, true, 102U));
  f.arm(1U);
  f.command(102U);
  f.route();
  f.pump();
  assert(f.sent.size() == 2U);
  assert(f.sent[1].can_id == 0x111U && f.sent[1].data[0] == 0xCCU);
}

static void test_fresh_enable_after_stop_before_route_is_preserved() {
  Fixture f;
  f.add_motor(1U);
  f.command();
  f.route();
  apply_stop(f, Stop::Revoke);
  assert(motor_control_set_authorization(&f.kernel, true, 102U));
  f.arm(1U);
  f.command(102U); // New evidence arrives before the task drains cancellation.
  f.route();
  f.pump();
  f.pump();
  assert(f.sent.size() == 2U);
  assert(f.sent[0].data[0] == 0x0CU); // New stop precedes the deliberate enable.
  assert(f.sent[1].data[0] == 0xCCU);
  assert(!f.scheduler.p1.dirty[0]); // No pre-stop position target survives.
}

int main() {
  test_stop_cancels_kernel_and_scheduler_commands();
  test_stop_also_cancels_an_unsent_enable();
  test_soft_stop_discards_old_position_target();
  test_full_p0_queue_cannot_preserve_the_old_target();
  test_cancel_even_when_disable_encoding_fails();
  test_cerb_high_node_aggregation_excludes_stopped_axis();
  test_explicit_recovery_can_send_a_fresh_enable();
  test_fresh_enable_after_stop_before_route_is_preserved();
  std::puts("test_stop_tx: all passed");
}
