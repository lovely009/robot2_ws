// can_rx_ring host test（ARD-0007 §21）。
// 验证：SPSC 环的 FIFO 顺序、字段保真、容量 64、满时丢新帧并计数、
// head/tail 回绕、部分弹出后的 pending 语义。

#include <cassert>
#include <cstdio>
#include <cstring>

#include "can_rx_ring.h"

static can_rx_frame_t make_frame(uint8_t bus, uint32_t id, uint8_t dlc,
    uint32_t tick) {
  can_rx_frame_t f = {};
  f.bus_index = bus;
  f.can_id = id;
  f.is_extended = false;
  f.is_fd = true;
  f.is_brs = true;
  f.dlc = dlc;
  f.rx_tick_ms = tick;
  for (uint8_t i = 0; i < dlc; ++i) {
    f.data[i] = static_cast<uint8_t>(id + i);
  }
  return f;
}

// FIFO 顺序与字段保真。
static void test_fifo_order_and_fields() {
  can_rx_ring_t ring;
  can_rx_ring_init(&ring);
  assert(can_rx_ring_pending(&ring) == 0);

  can_rx_frame_t in = make_frame(1, 0x709, 12, 555);
  assert(can_rx_ring_push_from_isr(&ring, &in));
  assert(can_rx_ring_pending(&ring) == 1);
  assert(ring.rx_count == 1);

  can_rx_frame_t out = {};
  assert(can_rx_ring_pop(&ring, &out, 1) == 1);
  assert(out.bus_index == 1);
  assert(out.can_id == 0x709);
  assert(out.dlc == 12);
  assert(out.rx_tick_ms == 555);
  assert(out.is_fd && out.is_brs && !out.is_extended);
  assert(memcmp(out.data, in.data, 12) == 0);
  assert(can_rx_ring_pending(&ring) == 0);

  // 空环弹出返回 0。
  assert(can_rx_ring_pop(&ring, &out, 1) == 0);
}

// 三帧批量弹出保持顺序；部分弹出后 pending 正确。
static void test_batch_pop_order() {
  can_rx_ring_t ring;
  can_rx_ring_init(&ring);
  for (uint32_t i = 0; i < 3; ++i) {
    can_rx_frame_t f = make_frame(0, 0x100 + i, 8, i);
    assert(can_rx_ring_push_from_isr(&ring, &f));
  }
  assert(can_rx_ring_pending(&ring) == 3);

  can_rx_frame_t out[2];
  assert(can_rx_ring_pop(&ring, out, 2) == 2);
  assert(out[0].can_id == 0x100);
  assert(out[1].can_id == 0x101);
  assert(can_rx_ring_pending(&ring) == 1);
  assert(can_rx_ring_pop(&ring, out, 2) == 1);
  assert(out[0].can_id == 0x102);
}

// 容量 64：第 65 帧被丢（丢新帧），旧帧完整保留（ARD §21.3/§51.1）。
static void test_capacity_64_and_drop_new() {
  can_rx_ring_t ring;
  can_rx_ring_init(&ring);
  for (uint32_t i = 0; i < CAN_RX_RING_CAPACITY; ++i) {
    can_rx_frame_t f = make_frame(0, 0x200 + i, 8, i);
    assert(can_rx_ring_push_from_isr(&ring, &f));
  }
  assert(can_rx_ring_pending(&ring) == CAN_RX_RING_CAPACITY);

  can_rx_frame_t extra = make_frame(0, 0x2FF, 8, 999);
  assert(!can_rx_ring_push_from_isr(&ring, &extra)); // 满丢新
  assert(ring.overflow_count == 1);
  assert(can_rx_ring_pending(&ring) == CAN_RX_RING_CAPACITY);

  // 弹出顺序 = 入队顺序（最旧帧存活，新帧被丢弃）。
  can_rx_frame_t out[CAN_RX_RING_CAPACITY];
  assert(can_rx_ring_pop(&ring, out, CAN_RX_RING_CAPACITY) == CAN_RX_RING_CAPACITY);
  assert(out[0].can_id == 0x200);
  assert(out[CAN_RX_RING_CAPACITY - 1].can_id == 0x200 + CAN_RX_RING_CAPACITY - 1);
  assert(ring.rx_count == CAN_RX_RING_CAPACITY + 1);
}

// head/tail 回绕：跨容量多轮推弹，顺序与计数保持。
static void test_wraparound() {
  can_rx_ring_t ring;
  can_rx_ring_init(&ring);
  can_rx_frame_t out[4];
  uint32_t total = 0;
  for (uint32_t round = 0; round < CAN_RX_RING_CAPACITY * 3 + 7; ++round) {
    can_rx_frame_t f = make_frame(2, 0x300 + round, 4, round);
    assert(can_rx_ring_push_from_isr(&ring, &f));
    total++;
    uint32_t popped;
    while ((popped = can_rx_ring_pop(&ring, out, 4)) > 0) {
      for (uint32_t i = 0; i < popped; ++i) {
        assert(out[i].can_id == 0x300 + (total - popped) + i); // 严格 FIFO
      }
    }
    assert(can_rx_ring_pending(&ring) == 0);
  }
  assert(ring.rx_count == total);
  assert(ring.overflow_count == 0);
}

int main() {
  test_fifo_order_and_fields();
  test_batch_pop_order();
  test_capacity_64_and_drop_new();
  test_wraparound();
  printf("test_can_rx_ring: all passed\n");
  return 0;
}
