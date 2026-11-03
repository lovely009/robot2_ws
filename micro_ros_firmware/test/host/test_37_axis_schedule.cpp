// Kernel/CAN scheduling regression, not a JTC/USB integration acceptance test.
// All IDs, bus assignments and device responses in this fixture are fictional.
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "motor_control_internal.h"
#include "motor_tx_router.h"

namespace {
constexpr unsigned axes = 37;
struct Axis {
  double position = 0., target = 0.;
  std::vector<unsigned> commands;
};
struct Bench {
  motor_control_context_t kernel{};
  motor_tx_router_t router{};
  std::array<can_tx_scheduler_t, CAN_TX_BUS_COUNT> schedulers{};
  std::array<Axis, axes> devices;
  unsigned tick = 100;
  const motor_protocol_adapter_t *adapters[1] = {&motor_protocol_cerb_adapter};
  Bench() {
    motor_control_init(&kernel, adapters, 1);
    motor_tx_router_init(&router);
    for (auto &scheduler : schedulers) can_tx_scheduler_init(&scheduler);
    motor_control_set_agent_connected(&kernel, true, tick);
    assert(motor_control_set_authorization(&kernel, true, tick));
    kernel.entry_count = axes;
    for (unsigned i = 0; i < axes; ++i) {
      auto &entry = kernel.entries[i];
      entry.used = entry.online = entry.has_measured = true;
      entry.motor_id = entry.node = i + 1;
      entry.bus_index = i % CAN_TX_BUS_COUNT;
      entry.adapter = &motor_protocol_cerb_adapter;
      for (size_t p = 0; p < motor_control_profile_count(); ++p) {
        auto profile = motor_control_profile_at(p);
        if (std::strcmp(profile->model_name, "cerb_r48_101") == 0) entry.profile = profile;
      }
      assert(entry.profile);
      entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
      entry.policy_enable = entry.motion_authorized = entry.motion_enabled = true;
      entry.identity_read_done = entry.dev_read_done = entry.boot_record_offered = true;
      entry.dev_raw_present = 1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX;
      entry.dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX] = 300;
      entry.last_motion_rx_tick_ms = entry.last_rx_tick_ms = tick;
    }
  }
  void feedback() {
    for (unsigned i = 0; i < axes; ++i) {
      // Stateful bounded-speed device; encode actual CERB measurement bytes.
      auto &device = devices[i];
      device.position += std::fmax(-.02, std::fmin(.02, device.target - device.position));
      const auto code = static_cast<int16_t>(std::lround(device.position * 180. / MOTOR_PROTOCOL_PI * 182.));
      motor_protocol_raw_frame_t frame{};
      frame.bus_index = i % CAN_TX_BUS_COUNT;
      frame.can_id = MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE + i + 1;
      frame.is_fd = true;
      frame.dlc = 12;
      frame.rx_tick_ms = tick;
      frame.data[0] = static_cast<uint8_t>(static_cast<uint16_t>(code) >> 8);
      frame.data[1] = static_cast<uint8_t>(code);
      motor_control_on_raw_frame(&kernel, &frame);
    }
  }
  void snapshot(unsigned source) {
    motor_control_command_snapshot_t command{};
    command.sequence = source;
    command.received_tick_ms = tick;
    command.valid_until_tick_ms = source + 30;
    command.target_count = axes;
    for (unsigned i = 0; i < axes; ++i) {
      command.targets[i].motor_id = i + 1;
      command.targets[i].position_rad = .1f + static_cast<float>(i) * .001f;
    }
    assert(motor_control_apply_command_snapshot(&kernel, &command, tick));
  }
  void receive(uint8_t bus, unsigned node, const uint8_t *data) {
    assert(node >= 1 && node <= axes);
    assert((node - 1) % CAN_TX_BUS_COUNT == bus);
    assert(data[0] == 0xCC);  // An enabled MIT output, not an inferred send count.
    auto &device = devices[node - 1];
    device.commands.push_back(tick);
    const auto raw = static_cast<unsigned>(data[1]) * 256 + data[2];
    const double limit = kernel.entries[node - 1].profile->mit_position_limit_rad;
    device.target = (raw / 65535. * 2. - 1.) * limit;
  }
  static bool submit(void *context, uint8_t bus, const can_tx_frame_t *frame) {
    auto &bench = *static_cast<Bench *>(context);
    if (frame->can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID) {
      for (unsigned i = 0; i < 6 && frame->data[56 + i]; ++i)
        bench.receive(bus, frame->data[56 + i], frame->data + i * 9);
    } else {
      bench.receive(bus, frame->can_id - 0x110U, frame->data);
    }
    return true;
  }
  void route() {
    const auto canceled = motor_control_take_tx_cancellations(&kernel);
    for (auto &scheduler : schedulers) can_tx_scheduler_cancel_slots(&scheduler, canceled);
    motor_protocol_tx_frame_t frames[MOTOR_TX_ROUTER_LOGICAL_CAPACITY];
    const auto count = motor_control_collect_tx(&kernel, frames, MOTOR_TX_ROUTER_LOGICAL_CAPACITY);
    for (size_t i = 0; i < count; ++i) {
      const auto &input = frames[i];
      can_tx_frame_t output{};
      output.can_id = input.can_id;
      output.tx_slot = input.tx_slot;
      output.is_extended = input.is_extended;
      output.is_fd = input.is_fd;
      output.is_brs = input.is_brs;
      output.node = input.node;
      output.aggregate_only = input.aggregate_only;
      output.size = input.size;
      std::memcpy(output.data, input.data, input.size);
      assert(input.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
      assert(can_tx_scheduler_submit_p1(&schedulers[input.bus_index], input.tx_slot, &output));
    }
    for (unsigned bus = 0; bus < CAN_TX_BUS_COUNT; ++bus)
      motor_tx_router_pump(&router, &schedulers[bus], bus, submit, this);
  }
};

void test_fair_50_50_200() {
  Bench bench;
  for (bench.tick = 100; bench.tick < 1100; ++bench.tick) {
    if (bench.tick % 20 == 0) bench.feedback(); // 50 Hz feedback, real decoder.
    motor_control_service(&bench.kernel, bench.tick);
    if (bench.tick % 5 == 0) {
      if (bench.tick % 20 == 0) bench.snapshot(bench.tick); // 50 Hz source.
      motor_control_hold_step(&bench.kernel, bench.tick); // 200 Hz boundary.
    }
    bench.route();
  }
  for (unsigned i = 0; i < axes; ++i) {
    const auto &device = bench.devices[i];
    assert(device.commands.size() == 200);
    for (size_t n = 1; n < device.commands.size(); ++n)
      assert(device.commands[n] - device.commands[n - 1] == 5);
    assert(std::fabs(device.position - (.1 + i * .001)) < .002);
    assert(bench.kernel.entries[i].has_measured);
  }
  std::puts("37 fictional axes / 3 buses: every axis 200 outputs, every interval 5 ms; feedback decoded");
}

void test_deadline_is_not_renewed_by_hold() {
  for (unsigned delay = 0; delay <= 10; ++delay) {
    Bench bench;
    bench.tick = 100 + delay;
    bench.snapshot(100); // Transport delay consumes the source's 30ms window.
    motor_control_hold_step(&bench.kernel, 129);
    motor_control_service(&bench.kernel, 130);
    assert(!motor_control_target_valid(&bench.kernel, 0, 130));
    assert(bench.kernel.entries[0].lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
    assert(bench.kernel.snapshot_deadline_tick_ms == 130); // Hold cannot restamp.
  }
  std::puts("0..10 ms receive delay: original 30ms deadline preserved, timeout soft-stops");
}

void test_receive_and_control_phase_margin() {
  for (unsigned receive_period : {10U, 1U}) {
  unsigned cases_with_gaps = 0, longest_gap = 0;
  // All millisecond phases: 20ms host source, 10ms receive task, 5ms control.
  // The source deadline remains source+30 throughout. A gap is an acceptance
  // blocker for smooth 50/50/200 operation, even though safe rejection is correct.
  for (unsigned host = 0; host < 20; ++host)
    for (unsigned receive = 0; receive < receive_period; ++receive)
      for (unsigned control = 0; control < 5; ++control) {
        Bench bench;
        unsigned latest = 0, consumed = 0, pending = 0, gap = 0;
        bool had_gap = false;
        for (bench.tick = 100; bench.tick < 360; ++bench.tick) {
          if (bench.tick % 20 == host) latest = bench.tick;
          if (bench.tick % 20 == 0) bench.feedback();
          motor_control_service(&bench.kernel, bench.tick);
          if (bench.tick % receive_period == receive && latest != consumed) {
            pending = latest;
            consumed = latest;
          }
          if (bench.tick % 5 == control && pending) {
            bench.snapshot(pending);
            pending = 0;
            motor_control_hold_step(&bench.kernel, bench.tick);
          }
          if (bench.tick >= 160 && bench.kernel.have_snapshot &&
              !motor_control_target_valid(&bench.kernel, 0, bench.tick)) {
            had_gap = true;
            longest_gap = std::max(longest_gap, ++gap);
          } else gap = 0;
          // Drain bounded output to isolate phase behavior from queue pressure.
          motor_protocol_tx_frame_t output[MOTOR_CONTROL_MAX_TX_PENDING];
          motor_control_collect_tx(&bench.kernel, output, MOTOR_CONTROL_MAX_TX_PENDING);
        }
        cases_with_gaps += had_gap;
      }
  if (receive_period == 1) { assert(cases_with_gaps == 0); }
  else assert(cases_with_gaps > 0); // Known timing margin deficiency must stay visible.
  assert(cases_with_gaps < 1000);
  assert(longest_gap <= 4);
  std::printf("receive=%ums: %u/%u phase cases have deadline gaps, maximum %u ms; no timeout widened\n",
      receive_period, cases_with_gaps, 100 * receive_period, longest_gap);
  }
}
} // namespace

int main() {
  test_fair_50_50_200();
  test_deadline_is_not_renewed_by_hold();
  test_receive_and_control_phase_margin();
}
