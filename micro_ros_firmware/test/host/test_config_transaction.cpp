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
  r.count = r.size = 1;
  r.targets[0] = {2, 2, 3.0f};
  return r;
}

static void test_exact_content_and_sessions() {
  Coordinator c;
  auto r = request();
  r.valid_until_tick_ms = 100;
  int slot = c.begin(r, 9, UINT32_MAX - 10);
  assert(slot >= 0 && c.track(slot, 1));
  c.submitted(slot, 1, 0);
  auto changed = r;
  changed.sync_time_ns = 1;
  assert(c.begin(changed, 9, 0) < 0);
  Result out{};
  assert(c.peek(out) && out.status == kPending); c.ack();
  changed = r;
  uint32_t bits = 0x7fc00001;
  std::memcpy(&changed.targets[0].value, &bits, sizeof(bits));
  assert(c.begin(changed, 9, 0) < 0);
  assert(c.peek(out) && out.status == kConflict); c.ack();
  c.poll(kTimeoutMs - 12);
  assert(c.result(slot)->status == kPending);
  c.poll(kTimeoutMs - 11);
  assert(c.result(slot)->status == kFailed);
  changed = request(0);
  assert(c.begin(changed, 9, 0) < 0);
  assert(c.peek(out) && out.status == kFailed); c.ack();
  changed = request(8); changed.session_id = 10;
  assert(c.begin(changed, 9, 0) < 0);
  assert(c.peek(out) && out.status == kFailed); c.ack();

  Coordinator disconnect;
  slot = disconnect.begin(request(), 9, 0);
  assert(disconnect.track(slot, 8)); disconnect.submitted(slot, 1, 0);
  disconnect.disconnect();
  assert(disconnect.result(slot)->status == kFailed);
  assert(!disconnect.on_result(8, true, 0));
}

static void test_evicted_request_cannot_replay() {
  Coordinator c;
  Result out{};
  auto old_save = request();
  old_save.kind = 1; old_save.count = old_save.size = 0;
  int slot = c.begin(old_save, 9, 0);
  assert(slot >= 0);
  c.complete(slot, 1, 0, 0);
  assert(c.peek(out)); c.ack();
  assert(c.ack_result(9, 1));
  for (uint32_t id = 2; id <= 12; ++id) {
    auto r = old_save; r.request_id = id;
    slot = c.begin(r, 9, id);
    assert(slot >= 0);
    c.complete(slot, 1, 0, 0);
    assert(c.peek(out)); c.ack();
    assert(c.ack_result(9, id));
  }
  old_save.sync_time_ns = 100;
  assert(c.request_high_water() == 12);
  assert(c.begin(old_save, 9, 100) < 0);
  assert(c.peek(out) && out.status == kRetired && out.error_code == 22);
  c.ack();
  auto gap = old_save; gap.request_id = 11;
  assert(c.begin(gap, 9, 100) < 0); // 仍缓存的旧请求返回原成功。
  assert(c.peek(out) && out.status == kRetired); c.ack();
  auto next_session = old_save; next_session.session_id = 10;
  assert(c.begin(next_session, 10, 0) >= 0);
  assert(c.begin(old_save, 10, 0) < 0);
  assert(c.peek(out) && out.status == kFailed); c.ack();
}

static void test_new_session_discards_old_results() {
  Coordinator c;
  for (uint32_t id = 1; id <= 8; ++id) {
    int slot = c.begin(request(id), 9, 0);
    assert(slot >= 0);
    c.external(slot);
  }
  assert(c.begin(request(9), 9, 0) < 0);
  Result stale{};
  assert(c.peek(stale) && stale.status == kBusy);
  c.set_session(10); // 连旧会话尚未 ack 的拒绝也不能阻塞新会话。
  assert(c.request_high_water() == 0);
  auto r = request(); r.session_id = 10;
  int slot = c.begin(r, 10, 1);
  assert(slot >= 0 && c.request_high_water() == 1);
  c.complete(slot, 1, 0, 0);
  Result out{};
  assert(c.peek(out) && out.session_id == 10 && out.status == kSucceeded);
  c.ack();
  c.set_session(10);
  assert(c.request_high_water() == 1);
  assert(!c.peek(out));
}

static void test_results_require_explicit_ack_and_never_lower_watermark() {
  Coordinator c;
  Result rows[kCapacity]{};
  for (uint32_t id = 1; id <= kCapacity; ++id) {
    const int slot = c.begin(request(id), 9, 0);
    assert(slot >= 0);
    c.complete(slot, 1, 0, 0);
    Result published{};
    assert(c.peek(published)); c.ack(); // 发布成功不能回收终态
  }
  assert(c.copy_results(rows, kCapacity) == kCapacity);
  assert(c.copy_results(rows, kCapacity) == kCapacity);
  assert(c.begin(request(9), 9, 1) < 0);
  assert(c.request_high_water() == 8);
  assert(!c.ack_result(10, 1));
  assert(c.ack_result(9, 1));
  assert(!c.ack_result(9, 1));
  assert(c.request_high_water() == 8);
  assert(c.begin(request(1), 9, 2) < 0);
  Result rejected{};
  assert(c.copy_rejection(rejected) && rejected.status == kRetired);
  const int ninth = c.begin(request(9), 9, 2);
  assert(ninth >= 0); // BUSY 没有提前退休该 ID
  for (uint32_t id = 2; id <= 8; ++id) assert(c.ack_result(9, id));
  c.busy(ninth);
  assert(c.request_high_water() == 8); // 即使此前槽已 ACK，水位也不能退到零
}

static void test_retry_never_extends_start_deadline() {
  Coordinator c;
  auto r = request();
  r.valid_until_tick_ms = 10;
  const int slot = c.begin(r, 9, 0);
  assert(slot >= 0);
  auto retry = r;
  retry.sync_time_ns = 999;
  retry.valid_until_tick_ms = 100;
  assert(c.begin(retry, 9, 5) < 0);
  assert(c.can_start(slot, 9));
  assert(!c.can_start(slot, 10));
  assert(c.result(slot)->status == kFailed && c.result(slot)->error_code == kExpired);
  assert(!c.ack_result(10, r.request_id));
  assert(c.ack_result(9, r.request_id));
  Coordinator wrapping;
  r.valid_until_tick_ms = 3;
  const int wrap_slot = wrapping.begin(r, 9, UINT32_MAX - 3);
  assert(wrap_slot >= 0 && wrapping.can_start(wrap_slot, 2));
  assert(!wrapping.can_start(wrap_slot, 3));
}

static void test_target_masks_partial_and_external_timeout() {
  Coordinator c;
  auto r = request();
  r.count = r.size = 2;
  r.targets[1] = {3, 2, 4.0f};
  const int slot = c.begin(r, 9, 0);
  assert(c.track(slot, 100, 2, 2) && c.track(slot, 101, 3, 2));
  c.submitted(slot, 2, 0);
  assert(!c.ack_result(9, 1)); // 未完成不能被 ACK 丢弃
  assert(c.on_result(100, true, 0, 2));
  assert(c.on_result(101, false, 7, 3));
  const auto *out = c.result(slot);
  assert(out->status == kPartial);
  assert(out->completed_motor_mask == 2 && out->failed_motor_mask == 4);
  Detail details[kMaxTargets]{};
  assert(c.copy_details(9, 1, details, kMaxTargets) == 2);
  assert(details[1].error == 7);
  Coordinator external;
  const int s = external.begin(request(), 9, 0);
  external.external(s);
  external.poll(kTimeoutMs);
  assert(external.result(s)->status == kFailed);
}

int main() {
  test_target_masks_partial_and_external_timeout();
  test_results_require_explicit_ack_and_never_lower_watermark();
  test_retry_never_extends_start_deadline();
  {
    Coordinator reserved;
    assert(reserved.begin(request(0x40000000U), 9, 0) < 0);
    assert(reserved.begin(request(0x80000000U), 9, 0) < 0);
    assert(reserved.request_high_water() == 0U);
  }
  test_new_session_discards_old_results();
  test_evicted_request_cannot_replay();
  test_exact_content_and_sessions();
  Coordinator c;
  auto r = request();
  int slot = c.begin(r, 9, 0);
  assert(slot >= 0);
  assert(c.track(slot, 101));
  c.submitted(slot, 1, 0);
  assert(c.begin(r, 9, 1) < 0);
  assert(c.on_result(101, true, 0));
  assert(!c.on_result(101, true, 0));
  Result out{};
  assert(c.peek(out) && out.status == kSucceeded && out.completed_targets == 1);
  assert(c.peek(out)); // 未 ack 的结果必须保留。
  c.ack();
  assert(!c.peek(out));
  assert(c.begin(r, 9, 2) < 0);
  assert(c.peek(out) && out.status == kSucceeded);
  c.ack();
  r.targets[0].value = 4;
  assert(c.begin(r, 9, 3) < 0);
  assert(c.peek(out) && out.status == kConflict);
  c.ack();
  assert(c.begin(request(2), 0, 0) < 0);
  assert(c.peek(out) && out.status == kFailed);
  c.ack();

  Coordinator full;
  for (uint32_t i = 1; i <= 8; ++i) {
    int s = full.begin(request(i), 9, 0);
    assert(s >= 0);
    assert(full.track(s, 100 + i));
    full.submitted(s, 1, 0);
  }
  assert(full.begin(request(9), 9, 0) < 0);
  assert(full.peek(out) && out.status == kBusy);
  full.ack();
  full.poll(kTimeoutMs);
  for (int i = 0; i < 8; ++i) {
    assert(full.peek(out) && out.status == kFailed && out.failed_targets == 1);
    full.ack();
  }
  assert(!full.on_result(101, true, 0));

  Coordinator partial;
  slot = partial.begin(request(), 9, 0);
  assert(partial.track(slot, 77));
  partial.submitted(slot, 3, 11); // 已提交一项，剩余两项未提交。
  assert(partial.on_result(77, true, 0));
  assert(partial.peek(out) && out.status == kPartial);
  assert(out.completed_targets == 1 && out.failed_targets == 2 && out.error_code == 11);

  Coordinator race;
  slot = race.begin(request(), 9, 0);
  assert(race.track(slot, 78));
  race.submitted(slot, 1, 0);
  assert(race.peek(out) && out.status == kPending);
  assert(race.on_result(78, true, 0));
  race.ack(); // ack 旧 PENDING 不得抹掉新终态。
  assert(race.peek(out) && out.status == kSucceeded);
  race.ack();

  Coordinator save;
  r = request();
  r.kind = 1; r.command = 0; r.count = r.size = 0;
  slot = save.begin(r, 9, 0);
  save.external(slot);
  save.poll(kTimeoutMs * 3); // 所有请求都有本地五秒总超时。
  assert(save.peek(out) && out.status == kFailed);
  save.ack();
  save.complete(slot, 2, 1, 20); // 迟到完成不得改写已结算终态。
  assert(!save.peek(out));
  std::printf("config_transaction: all assertions passed; coordinator=%zu bytes\n", sizeof(Coordinator));
}
