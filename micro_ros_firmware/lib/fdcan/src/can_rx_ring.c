// 每总线静态 RX 环实现（SPSC：ISR 生产 / 任务消费，见 can_rx_ring.h）。
// head/tail 为单调递增序号：差值 (head - tail) 即在环帧数（uint32 回绕安全），
// 槽位取模；环满判定 head - tail >= CAPACITY，不牺牲槽位。

#include "can_rx_ring.h"

#include <string.h>

void can_rx_ring_init(can_rx_ring_t *ring) {
  if (ring == NULL) {
    return;
  }
  memset(ring, 0, sizeof(*ring));
}

bool can_rx_ring_push_from_isr(can_rx_ring_t *ring, const can_rx_frame_t *frame) {
  if (ring == NULL || frame == NULL) {
    return false;
  }
  if ((uint32_t)(ring->head - ring->tail) >= CAN_RX_RING_CAPACITY) {
    // 环满：丢新帧（ISR 不动 tail——SPSC 单写者约定），计数可见（ARD §21.3）。
    ring->overflow_count++;
    ring->rx_count++;
    return false;
  }
  ring->slots[ring->head % CAN_RX_RING_CAPACITY] = *frame;
  ring->head++;
  ring->rx_count++;
  return true;
}

uint32_t can_rx_ring_pop(can_rx_ring_t *ring, can_rx_frame_t *out,
                         uint32_t max_count) {
  if (ring == NULL || out == NULL || max_count == 0U) {
    return 0U;
  }
  uint32_t popped = 0U;
  while (popped < max_count) {
    if (ring->tail == ring->head) {
      break; // 空
    }
    out[popped] = ring->slots[ring->tail % CAN_RX_RING_CAPACITY];
    ring->tail++;
    popped++;
  }
  return popped;
}

uint32_t can_rx_ring_pending(const can_rx_ring_t *ring) {
  if (ring == NULL) {
    return 0U;
  }
  return (uint32_t)(ring->head - ring->tail);
}
