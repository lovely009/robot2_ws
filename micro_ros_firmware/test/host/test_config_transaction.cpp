#include "config_transaction.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>

using namespace config_transaction;

static Request request(uint32_t id = 1) {
  Request r{};
  r.session_id = 9;
  r.request_id = id;
  r.valid_until_tick_ms = 1000;
  r.motor_id = UINT16_MAX;
  r.policy_generation = 1;
  r.policy_count = 1;
  r.policies[0] = {2, true, false, 3.0f, 1.0f};
  return r;
}

static Result result(const Coordinator &c, uint32_t id = 1) {
  Result rows[kCapacity]{};
  const auto count = c.copy_results(rows, kCapacity);
  for (size_t i = 0; i < count; ++i) if (rows[i].request_id == id) return rows[i];
  assert(false && "missing retained result");
  return {};
}
static Result rejection(const Coordinator &c) {
  Result out{};
  assert(c.copy_rejection(out));
  return out;
}

static void test_exact_content_and_sessions() {
  Coordinator c;
  auto r = request();
  r.valid_until_tick_ms = 100;
  int slot = c.begin(r, 9, UINT32_MAX - 10);
  assert(slot >= 0 && c.track(slot, 1, 2));
  c.submitted(slot, 1, 0);
  auto changed = r;
  changed.sync_time_ns = 1;
  assert(c.begin(changed, 9, 0) < 0);
  assert(result(c).status == kPending);
  changed = r;
  uint32_t bits = 0x7fc00001;
  std::memcpy(&changed.policies[0].kp, &bits, sizeof(bits));
  assert(c.begin(changed, 9, 0) < 0);
  assert(rejection(c).status == kConflict);
  c.poll(kTimeoutMs - 12);
  assert(result(c).status == kPending);
  c.poll(kTimeoutMs - 11);
  assert(result(c).status == kFailed);
  assert(c.begin(request(0), 9, 0) < 0 && rejection(c).status == kFailed);
  changed = request(8); changed.session_id = 10;
  assert(c.begin(changed, 9, 0) < 0 && rejection(c).status == kFailed);

  Coordinator disconnect;
  slot = disconnect.begin(request(), 9, 0);
  assert(disconnect.track(slot, 8, 2)); disconnect.submitted(slot, 1, 0);
  disconnect.disconnect();
  assert(result(disconnect).status == kFailed);
  assert(!disconnect.on_result(8, true, 0));
}

static void test_acknowledged_request_cannot_replay() {
  Coordinator c;
  for (uint32_t id = 1; id <= 12; ++id) {
    const int slot = c.begin(request(id), 9, id);
    assert(slot >= 0);
    c.complete(slot, 1, 0, 0);
    assert(result(c, id).status == kSucceeded);
    assert(c.ack_result(9, id));
  }
  auto old = request(); old.sync_time_ns = 100;
  assert(c.request_high_water() == 12);
  assert(c.begin(old, 9, 100) < 0);
  assert(rejection(c).status == kRetired && rejection(c).error_code == kExpired);
  old.request_id = 11;
  assert(c.begin(old, 9, 100) < 0 && rejection(c).status == kRetired);
  auto next_session = request(); next_session.session_id = 10;
  assert(c.begin(next_session, 10, 0) >= 0);
  assert(c.begin(old, 10, 0) < 0 && rejection(c).status == kFailed);
}

static void test_new_session_discards_old_results() {
  Coordinator c;
  for (uint32_t id = 1; id <= kCapacity; ++id) assert(c.begin(request(id), 9, 0) >= 0);
  assert(c.begin(request(9), 9, 0) < 0 && rejection(c).status == kBusy);
  c.set_session(10);
  Result rows[kCapacity]{};
  assert(c.copy_results(rows, kCapacity) == 0 && !c.copy_rejection(rows[0]));
  assert(c.request_high_water() == 0);
  auto r = request(); r.session_id = 10;
  const int slot = c.begin(r, 10, 1);
  assert(slot >= 0 && c.request_high_water() == 1);
  c.complete(slot, 1, 0, 0);
  assert(result(c).session_id == 10 && result(c).status == kSucceeded);
  c.set_session(10);
  assert(c.request_high_water() == 1 && result(c).status == kSucceeded);
}

static void test_results_require_explicit_ack_and_never_lower_watermark() {
  Coordinator c;
  Result rows[kCapacity]{};
  for (uint32_t id = 1; id <= kCapacity; ++id) {
    const int slot = c.begin(request(id), 9, 0);
    assert(slot >= 0);
    c.complete(slot, 1, 0, 0);
  }
  assert(c.copy_results(rows, kCapacity) == kCapacity);
  assert(c.copy_results(rows, kCapacity) == kCapacity);
  assert(c.begin(request(9), 9, 1) < 0);
  assert(c.request_high_water() == 8);
  assert(!c.ack_result(10, 1));
  assert(c.ack_result(9, 1) && !c.ack_result(9, 1));
  assert(c.request_high_water() == 8);
  assert(c.begin(request(1), 9, 2) < 0 && rejection(c).status == kRetired);
  const int ninth = c.begin(request(9), 9, 2);
  assert(ninth >= 0);
  for (uint32_t id = 2; id <= 8; ++id) assert(c.ack_result(9, id));
  c.busy(ninth);
  assert(c.request_high_water() == 8);
  assert(c.begin(request(9), 9, 3) >= 0);
}

static void test_retry_never_extends_start_deadline() {
  Coordinator c;
  auto r = request(); r.valid_until_tick_ms = 10;
  const int slot = c.begin(r, 9, 0);
  assert(slot >= 0);
  auto retry = r; retry.sync_time_ns = 999; retry.valid_until_tick_ms = 100;
  assert(c.begin(retry, 9, 5) < 0);
  assert(c.can_start(slot, 9) && !c.can_start(slot, 10));
  assert(result(c).status == kFailed && result(c).error_code == kExpired);
  assert(!c.ack_result(10, r.request_id) && c.ack_result(9, r.request_id));
  Coordinator wrapping;
  r.valid_until_tick_ms = 3;
  const int wrap_slot = wrapping.begin(r, 9, UINT32_MAX - 3);
  assert(wrap_slot >= 0 && wrapping.can_start(wrap_slot, 2));
  assert(!wrapping.can_start(wrap_slot, 3));
}

static void test_masks_partial_results_and_late_completion() {
  Coordinator c;
  auto r = request(); r.policy_count = 2; r.policies[1] = {3, true, false, 4.0f, 1.0f};
  const int slot = c.begin(r, 9, 0);
  assert(c.track(slot, 100, 2) && c.track(slot, 101, 3));
  c.submitted(slot, 2, 0);
  assert(!c.ack_result(9, 1));
  assert(c.on_result(100, true, 0, 2) && c.on_result(101, false, 7, 3));
  const auto out = result(c);
  assert(out.status == kPartial && out.completed_motor_mask == 2 && out.failed_motor_mask == 4);
  Detail details[2]{};
  assert(c.copy_details(9, 1, details, 2) == 2 && details[1].error == 7);
  assert(!c.on_result(101, true, 0, 3));
  c.complete(slot, 2, 0, 0);
  assert(result(c).status == kPartial && result(c).failed_motor_mask == 4);
}

static void test_full_queue_times_out_and_keeps_terminal_results() {
  Coordinator full;
  for (uint32_t id = 1; id <= kCapacity; ++id) {
    const int slot = full.begin(request(id), 9, 0);
    assert(slot >= 0 && full.track(slot, 100 + id, 2));
    full.submitted(slot, 1, 0);
  }
  assert(full.begin(request(9), 9, 0) < 0 && rejection(full).status == kBusy);
  full.poll(kTimeoutMs);
  for (uint32_t id = 1; id <= kCapacity; ++id) {
    assert(result(full, id).status == kFailed && result(full, id).failed_targets == 1);
  }
  assert(!full.on_result(101, true, 0));
  Coordinator partial;
  const int slot = partial.begin(request(), 9, 0);
  assert(partial.track(slot, 77, 2));
  partial.submitted(slot, 3, 11);
  assert(partial.on_result(77, true, 0));
  const auto out = result(partial);
  assert(out.status == kPartial && out.completed_targets == 1 && out.failed_targets == 2 && out.error_code == 11);
  Coordinator unfinished;
  const int pending = unfinished.begin(request(), 9, 0);
  unfinished.poll(kTimeoutMs * 3);
  assert(result(unfinished).status == kFailed);
  unfinished.complete(pending, 2, 1, 20);
  assert(result(unfinished).error_code == kTimeout);
}

static void test_policy_results_share_operation_id_without_cross_matching() {
  Coordinator c;
  auto r = request(); r.policy_count = 2; r.policies[1] = {3, true, false, 4.0f, 1.0f};
  const int slot = c.begin(r, 9, 0);
  assert(c.track(slot, 77, 2) && c.track(slot, 77, 3));
  assert(!c.track(slot, 77, 2));
  c.submitted(slot, 2, 0);
  assert(!c.on_result(77, true, 0, 4));
  assert(c.on_result(77, false, 7, 3));
  assert(result(c).status == kPending && !c.ack_result(9, 1));
  assert(!c.on_result(77, true, 0, 3));
  assert(c.on_result(77, true, 0, 2));
  assert(result(c).status == kPartial);
  assert(result(c).completed_motor_mask == 2 && result(c).failed_motor_mask == 4);
  assert(result(c).completed_targets == 1 && result(c).failed_targets == 1);
  assert(c.ack_result(9, 1));
}

int main() {
  test_policy_results_share_operation_id_without_cross_matching();
  test_exact_content_and_sessions();
  test_acknowledged_request_cannot_replay();
  test_new_session_discards_old_results();
  test_results_require_explicit_ack_and_never_lower_watermark();
  test_retry_never_extends_start_deadline();
  test_masks_partial_results_and_late_completion();
  test_full_queue_times_out_and_keeps_terminal_results();
  Coordinator reserved;
  assert(reserved.begin(request(0x40000000U), 9, 0) < 0);
  assert(reserved.begin(request(0x80000000U), 9, 0) < 0);
  assert(reserved.request_high_water() == 0);
  auto too_many = request(); too_many.policy_count = kMaxOperations + 1;
  assert(reserved.begin(too_many, 9, 0) < 0 && rejection(reserved).error_code == kInvalid);
  std::printf("config_transaction: all assertions passed; coordinator=%zu bytes\n", sizeof(Coordinator));
}
