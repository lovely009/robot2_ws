// can_tx_scheduler host test（ARD-0007 §22）。
// 验证：P0 > P1 > P2 出队顺序、P1 latest-wins、P0 去重由上层负责（本层有界
// FIFO 满丢新并计数）、P2 每次 pump 配额、容量受限时的优先级保持、
// 多总线实例独立、硬件拒绝记账。

#include <cassert>
#include <cstdio>
#include <cstring>

#include "can_tx_scheduler.h"

static can_tx_frame_t make_frame(uint32_t id, uint8_t marker) {
  can_tx_frame_t f = {};
  f.can_id = id;
  f.is_fd = true;
  f.size = 4;
  f.data[0] = marker;
  return f;
}

// 空 pump、P1 槽发送一次后变干净。
static void test_p1_send_once() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  can_tx_frame_t out[8];

  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == 0);

  can_tx_frame_t f = make_frame(0x515, 1);
  assert(can_tx_scheduler_submit_p1(&sched, 5, &f));
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == 1);
  assert(out[0].can_id == 0x515 && out[0].data[0] == 1);
  assert(sched.p1_sent == 1);
  // 已发送的槽不再重发（无历史积压，禁止项 5）。
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == 0);
}

// P1 latest-wins：脏槽被覆盖 → replaced 计数，只发最新。
static void test_p1_latest_wins() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  can_tx_frame_t old_f = make_frame(0x515, 0xAA);
  can_tx_frame_t new_f = make_frame(0x515, 0xBB);
  assert(can_tx_scheduler_submit_p1(&sched, 3, &old_f));
  assert(can_tx_scheduler_submit_p1(&sched, 3, &new_f)); // 覆盖未发送旧目标
  assert(sched.p1.replaced_count == 1);

  can_tx_frame_t out[8];
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == 1);
  assert(out[0].data[0] == 0xBB); // 只发最新
}

// 优先级顺序：P0 → P1 → P2（ARD §22 / §43.3）。
static void test_priority_order() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  can_tx_frame_t p1 = make_frame(0x511, 0x11);
  can_tx_frame_t p2 = make_frame(0x522, 0x22);
  can_tx_frame_t p0 = make_frame(0x500, 0x00);
  assert(can_tx_scheduler_submit_p1(&sched, 1, &p1));
  assert(can_tx_scheduler_submit_p2(&sched, &p2));
  assert(can_tx_scheduler_submit_p0(&sched, &p0)); // 最后入队但最先出队

  can_tx_frame_t out[8];
  uint32_t p0_count = 0U;
  uint32_t p1_count = 0U;
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, &p0_count, &p1_count) == 3);
  assert(p0_count == 1U && p1_count == 1U);
  assert(out[0].data[0] == 0x00);
  assert(out[1].data[0] == 0x11);
  assert(out[2].data[0] == 0x22);
  assert(sched.p0_sent == 1 && sched.p1_sent == 1 && sched.p2_sent == 1);
}

// P2 配额：一次 pump 最多出队 CAN_TX_P2_PUMP_QUOTA 帧，分多轮排空。
static void test_p2_quota() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  for (uint8_t i = 0; i < 5; ++i) {
    can_tx_frame_t f = make_frame(0x520 + i, i);
    assert(can_tx_scheduler_submit_p2(&sched, &f));
  }
  can_tx_frame_t out[8];
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == CAN_TX_P2_PUMP_QUOTA);
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == CAN_TX_P2_PUMP_QUOTA);
  assert(can_tx_scheduler_pump(&sched, out, 8, 8, nullptr, nullptr) == 5 - 2 * CAN_TX_P2_PUMP_QUOTA);
}

// 控制槽持续填满时，调用方可保留物理发送名额给 SYNC/配置 P2。
static void test_p1_limit_leaves_room_for_p2() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  for (uint16_t slot = 0; slot < 16U; ++slot) {
    can_tx_frame_t f = make_frame(0x110U + slot, (uint8_t)slot);
    assert(can_tx_scheduler_submit_p1(&sched, slot, &f));
  }
  can_tx_frame_t sync = make_frame(0x080U, 0U);
  assert(can_tx_scheduler_submit_p2(&sched, &sync));
  can_tx_frame_t out[8];
  uint32_t p0_count = 0U;
  uint32_t p1_count = 0U;
  assert(can_tx_scheduler_pump(&sched, out, 8U, 6U,
      &p0_count, &p1_count) == 7U);
  assert(p0_count == 0U && p1_count == 6U);
  assert(out[6].can_id == 0x080U);
  assert(sched.p1.dirty[6] && sched.p1.dirty[15]);
}

static void test_sixteen_single_axis_commands_and_sync_drain_in_two_pumps() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  for (uint16_t slot = 0; slot < 16U; ++slot) {
    can_tx_frame_t f = make_frame(0x501U + 4U * slot, (uint8_t)slot);
    assert(can_tx_scheduler_submit_p1(&sched, slot, &f));
  }
  can_tx_frame_t sync = make_frame(0x080U, 0U);
  assert(can_tx_scheduler_submit_p2(&sched, &sync));
  can_tx_frame_t out[18];
  uint32_t p0_count = 0U;
  uint32_t p1_count = 0U;
  assert(can_tx_scheduler_pump(&sched, out, 18U, 15U,
      &p0_count, &p1_count) == 16U);
  assert(p1_count == 15U && out[15].can_id == 0x080U);
  assert(can_tx_scheduler_pump(&sched, out, 18U, 16U,
      &p0_count, &p1_count) == 1U);
  assert(p1_count == 1U && out[0].data[0] == 15U);
}

// P0/P2 有界：满则丢新并计数（ARD §22.3；P0 上层已去重）。
static void test_bounded_queues() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  for (uint8_t i = 0; i < CAN_TX_P0_CAPACITY; ++i) {
    can_tx_frame_t f = make_frame(0x500 + i, i);
    assert(can_tx_scheduler_submit_p0(&sched, &f));
  }
  can_tx_frame_t overflow = make_frame(0x5FF, 0xFF);
  assert(!can_tx_scheduler_submit_p0(&sched, &overflow));
  assert(sched.p0.dropped_count == 1);

  for (uint8_t i = 0; i < CAN_TX_P2_CAPACITY; ++i) {
    can_tx_frame_t f = make_frame(0x520 + i, i);
    assert(can_tx_scheduler_submit_p2(&sched, &f));
  }
  can_tx_frame_t p2_overflow = make_frame(0x5EF, 0xEE);
  assert(!can_tx_scheduler_submit_p2(&sched, &p2_overflow));
  assert(sched.p2.dropped_count == 1);
}

// 容量受限的 pump：P0 绝不被跳过，P1 次之，P2 留队等下一轮。
static void test_pump_small_capacity_keeps_priority() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  can_tx_frame_t p1a = make_frame(0x511, 0x11);
  can_tx_frame_t p1b = make_frame(0x512, 0x12);
  can_tx_frame_t p2 = make_frame(0x522, 0x22);
  can_tx_frame_t p0a = make_frame(0x500, 0xA0);
  can_tx_frame_t p0b = make_frame(0x501, 0xA1);
  assert(can_tx_scheduler_submit_p1(&sched, 1, &p1a));
  assert(can_tx_scheduler_submit_p1(&sched, 2, &p1b));
  assert(can_tx_scheduler_submit_p2(&sched, &p2));
  assert(can_tx_scheduler_submit_p0(&sched, &p0a));
  assert(can_tx_scheduler_submit_p0(&sched, &p0b));

  can_tx_frame_t out[2];
  uint32_t p0_count = 0U;
  assert(can_tx_scheduler_pump(&sched, out, 2, 2, &p0_count, nullptr) == 2);
  assert(p0_count == 2); // P0 前缀计数供硬件拒收回插识别
  assert(out[0].data[0] == 0xA0 && out[1].data[0] == 0xA1); // P0 先出
  assert(can_tx_scheduler_pump(&sched, out, 2, 2, &p0_count, nullptr) == 2);
  assert(p0_count == 0); // 本轮无 P0
  assert(out[0].data[0] == 0x11 && out[1].data[0] == 0x12); // P1 次之
  assert(can_tx_scheduler_pump(&sched, out, 2, 2, nullptr, nullptr) == 1); // P2 最后（配额 2 但仅剩 1）
  assert(out[0].data[0] == 0x22);
}

// 多总线独立：两个调度器实例互不影响（ARD §22 每总线独立）。
static void test_multi_bus_independence() {
  can_tx_scheduler_t bus0, bus1;
  can_tx_scheduler_init(&bus0);
  can_tx_scheduler_init(&bus1);
  can_tx_frame_t f0 = make_frame(0x510, 0xB0);
  can_tx_frame_t f1 = make_frame(0x511, 0xB1);
  assert(can_tx_scheduler_submit_p1(&bus0, 0, &f0));
  assert(can_tx_scheduler_submit_p1(&bus1, 0, &f1));

  can_tx_frame_t out[4];
  assert(can_tx_scheduler_pump(&bus0, out, 4, 4, nullptr, nullptr) == 1);
  assert(out[0].data[0] == 0xB0);
  assert(can_tx_scheduler_pump(&bus1, out, 4, 4, nullptr, nullptr) == 1);
  assert(out[0].data[0] == 0xB1);
}

// 硬件拒绝记账（ARD §32.1 tx_hw_reject）。
static void test_hw_reject_accounting() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  can_tx_scheduler_note_hw_reject(&sched);
  can_tx_scheduler_note_hw_reject(&sched);
  assert(sched.tx_hw_reject_count == 2);
}

// 取消必须跨过环形 FIFO 的回绕点，并覆盖 32 位以上的槽位；其他轴和
// P2 不得被误删。同轴旧使能也是 P0，必须随旧 P1 一起失效。
static void test_cancel_slots_preserves_unrelated_fifo_order() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  sched.p0.head = CAN_TX_P0_CAPACITY - 2U;
  const uint16_t high_slot = CAN_TX_P1_SLOTS - 1U;
  assert(high_slot >= 32U);
  const uint16_t slots[] = {1U, 2U, high_slot, 3U};
  for (const uint16_t slot : slots) {
    auto frame = make_frame(0x500U + slot, (uint8_t)slot);
    frame.tx_slot = slot;
    assert(can_tx_scheduler_submit_p0(&sched, &frame));
    assert(can_tx_scheduler_submit_p1(&sched, slot, &frame));
  }
  const auto management = make_frame(0x080U, 0xAAU);
  assert(can_tx_scheduler_submit_p2(&sched, &management));
  can_tx_scheduler_cancel_slots(&sched,
      (UINT64_C(1) << 1U) | (UINT64_C(1) << high_slot));
  assert(!sched.p1.dirty[1U] && !sched.p1.dirty[high_slot]);
  can_tx_frame_t out[8];
  uint32_t p0_count = 0U, p1_count = 0U;
  assert(can_tx_scheduler_pump(&sched, out, 8U, 8U,
      &p0_count, &p1_count) == 5U);
  assert(p0_count == 2U && p1_count == 2U);
  assert(out[0].data[0] == 2U && out[1].data[0] == 3U);
  assert(out[2].data[0] == 2U && out[3].data[0] == 3U);
  assert(out[4].can_id == management.can_id);
}

static void test_expiring_zero_cannot_escape_p0_backpressure_or_cancellation() {
  can_tx_scheduler_t sched;
  can_tx_scheduler_init(&sched);
  auto zero = make_frame(0x607, 1);
  zero.tx_slot = 2; zero.valid_until_tick_ms = 150;
  auto management = make_frame(0x080, 2);
  auto stop = make_frame(0x107, 3);
  assert(can_tx_scheduler_submit_p2(&sched, &zero));
  assert(can_tx_scheduler_submit_p2(&sched, &management));
  assert(can_tx_scheduler_submit_p0(&sched, &stop));
  can_tx_frame_t out[8];
  assert(can_tx_scheduler_pump(&sched, out, 1, 0, nullptr, nullptr) == 1);
  can_tx_scheduler_expire_p2(&sched, 149);
  assert(sched.p2.count == 2);
  can_tx_scheduler_expire_p2(&sched, 150);
  assert(can_tx_scheduler_pump(&sched, out, 8, 0, nullptr, nullptr) == 1);
  assert(out[0].can_id == 0x080);
  assert(can_tx_scheduler_submit_p2(&sched, &zero));
  assert(can_tx_scheduler_submit_p2(&sched, &management));
  stop.tx_slot = 2;
  assert(can_tx_scheduler_submit_p0(&sched, &stop));
  can_tx_scheduler_cancel_timed_p2(&sched, UINT64_C(1) << 2);
  assert(can_tx_scheduler_pump(&sched, out, 8, 0, nullptr, nullptr) == 2);
  assert(out[0].can_id == stop.can_id && out[1].can_id == 0x080);
  zero.valid_until_tick_ms = 2;
  assert(can_tx_scheduler_submit_p2(&sched, &zero));
  can_tx_scheduler_expire_p2(&sched, UINT32_MAX);
  assert(sched.p2.count == 1);
  can_tx_scheduler_expire_p2(&sched, 2);
  assert(sched.p2.count == 0);
}

int main() {
  test_expiring_zero_cannot_escape_p0_backpressure_or_cancellation();
  test_p1_send_once();
  test_p1_latest_wins();
  test_priority_order();
  test_p2_quota();
  test_p1_limit_leaves_room_for_p2();
  test_sixteen_single_axis_commands_and_sync_drain_in_two_pumps();
  test_bounded_queues();
  test_pump_small_capacity_keeps_priority();
  test_multi_bus_independence();
  test_hw_reject_accounting();
  test_cancel_slots_preserves_unrelated_fifo_order();
  printf("test_can_tx_scheduler: all passed\n");
  return 0;
}
