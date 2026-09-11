// motor_event_relay host 单测（cassert 风格）：FIFO 序、部分取走、满队丢最旧
// 保最新，以及 head 非零时的跨回绕；固定所有 head 状态下的严格 FIFO 契约。

#include <cassert>
#include <cstdint>
#include <cstdio>

#include "motor_event_relay.h"

namespace {

motor_control_event_t make_event(uint16_t seq) {
  motor_control_event_t e = {};
  e.type = MOTOR_CONTROL_EVENT_DISCOVERED;
  e.tick_ms = seq; // 用 tick_ms 携带写入序号，drain 后按序断言
  e.motor_id = seq;
  return e;
}

// 断言 out[0..n) 的写入序号恰为 first_seq 起的连续递增序列。
void assert_sequence(const motor_control_event_t *out, size_t n,
                     uint16_t first_seq) {
  for (size_t i = 0; i < n; ++i) {
    assert(out[i].tick_ms == (uint32_t)(first_seq + i));
  }
}

void test_fifo_order_and_partial_drain() {
  motor_event_relay_t relay;
  motor_event_relay_init(&relay);
  motor_control_event_t out[4];
  assert(motor_event_relay_drain(&relay, out, 4) == 0); // 空环

  for (uint16_t i = 0; i < 5; ++i) {
    motor_control_event_t e = make_event(i);
    motor_event_relay_push(&relay, &e);
  }
  assert(motor_event_relay_drain(&relay, out, 3) == 3);
  assert_sequence(out, 3, 0);
  assert(motor_event_relay_drain(&relay, out, 4) == 2);
  assert_sequence(out, 2, 3);
  assert(motor_event_relay_drain(&relay, out, 4) == 0);
  assert(motor_event_relay_dropped_count(&relay) == 0);
}

void test_full_drops_oldest_keeps_newest() {
  motor_event_relay_t relay;
  motor_event_relay_init(&relay);
  for (uint16_t i = 0; i < MOTOR_EVENT_RELAY_CAPACITY + 3; ++i) {
    motor_control_event_t e = make_event(i);
    motor_event_relay_push(&relay, &e);
  }
  assert(motor_event_relay_dropped_count(&relay) == 3);

  motor_control_event_t out[MOTOR_EVENT_RELAY_CAPACITY];
  assert(motor_event_relay_drain(&relay, out, MOTOR_EVENT_RELAY_CAPACITY) ==
         MOTOR_EVENT_RELAY_CAPACITY);
  assert_sequence(out, MOTOR_EVENT_RELAY_CAPACITY, 3); // 序号 3..66，最新保留

  // 排空后可继续使用：环不是一次性容器。
  motor_control_event_t e = make_event(100);
  motor_event_relay_push(&relay, &e);
  assert(motor_event_relay_drain(&relay, out, 1) == 1);
  assert(out[0].tick_ms == 100);
}

void test_wraparound_with_nonzero_head() {
  motor_event_relay_t relay;
  motor_event_relay_init(&relay);
  // 先制造 head≠0：入 5 取 3 → head=3，count=2（残留序号 3、4）。
  for (uint16_t i = 0; i < 5; ++i) {
    motor_control_event_t e = make_event(i);
    motor_event_relay_push(&relay, &e);
  }
  motor_control_event_t scratch[3];
  assert(motor_event_relay_drain(&relay, scratch, 3) == 3);

  // 从 head≠0 起入 64 条（先填满后跨下标回绕再溢出 2 条）：
  // 溢出丢的是最旧（序号 3、4），环形序仍严格 FIFO。
  for (uint16_t i = 5; i < MOTOR_EVENT_RELAY_CAPACITY + 5; ++i) {
    motor_control_event_t e = make_event(i);
    motor_event_relay_push(&relay, &e);
  }
  assert(motor_event_relay_dropped_count(&relay) == 2);

  motor_control_event_t out[MOTOR_EVENT_RELAY_CAPACITY];
  assert(motor_event_relay_drain(&relay, out, MOTOR_EVENT_RELAY_CAPACITY) ==
         MOTOR_EVENT_RELAY_CAPACITY);
  assert_sequence(out, MOTOR_EVENT_RELAY_CAPACITY, 5); // 序号 5..68
}

void test_null_and_zero_capacity_defense() {
  motor_event_relay_t relay;
  motor_event_relay_init(&relay);
  motor_control_event_t e = make_event(1);
  motor_event_relay_push(&relay, &e);
  motor_control_event_t out[1];
  assert(motor_event_relay_drain(&relay, out, 0) == 0);  // capacity 0
  assert(motor_event_relay_drain(&relay, nullptr, 4) == 0); // out NULL
  assert(motor_event_relay_drain(nullptr, out, 4) == 0);    // relay NULL
  motor_event_relay_push(&relay, nullptr);                  // 不崩溃
  motor_event_relay_push(nullptr, &e);
  assert(motor_event_relay_dropped_count(&relay) == 0);
  assert(motor_event_relay_dropped_count(nullptr) == 0);
}

} // namespace

int main() {
  test_fifo_order_and_partial_drain();
  test_full_drops_oldest_keeps_newest();
  test_wraparound_with_nonzero_head();
  test_null_and_zero_capacity_defense();
  std::puts("motor_event_relay: all tests passed");
  return 0;
}
