// 每总线 TX 软件调度器实现（P0 > P1 latest-wins > P2 有界配额，见 can_tx_scheduler.h）。

#include "can_tx_scheduler.h"

#include <string.h>

void can_tx_scheduler_init(can_tx_scheduler_t *sched) {
  if (sched == NULL) {
    return;
  }
  memset(sched, 0, sizeof(*sched));
}

static bool fifo_push(can_tx_frame_t *queue, uint8_t capacity, uint8_t *head,
                      uint8_t *count, uint32_t *dropped,
                      const can_tx_frame_t *frame) {
  if (*count >= capacity) {
    // 满即丢新帧（P0 丢弃安全网：上层内核已带 p0_pending 重试）。
    (*dropped)++;
    return false;
  }
  const uint8_t index = (uint8_t)((*head + *count) % capacity);
  queue[index] = *frame;
  (*count)++;
  return true;
}

static bool fifo_pop(can_tx_frame_t *queue, uint8_t capacity, uint8_t *head,
                     uint8_t *count, can_tx_frame_t *out) {
  if (*count == 0U) {
    return false;
  }
  *out = queue[*head];
  *head = (uint8_t)((*head + 1U) % capacity);
  (*count)--;
  return true;
}

bool can_tx_scheduler_submit_p0(can_tx_scheduler_t *sched,
                                const can_tx_frame_t *frame) {
  if (sched == NULL || frame == NULL) {
    return false;
  }
  return fifo_push(sched->p0.queue, CAN_TX_P0_CAPACITY, &sched->p0.head,
                   &sched->p0.count, &sched->p0.dropped_count, frame);
}

bool can_tx_scheduler_submit_p1(can_tx_scheduler_t *sched, uint16_t slot,
                                const can_tx_frame_t *frame) {
  if (sched == NULL || frame == NULL || slot >= CAN_TX_P1_SLOTS) {
    return false;
  }
  if (sched->p1.dirty[slot]) {
    // latest-wins：覆盖未发送旧目标（ARD §22.2，禁止项 5 的反面实现）。
    sched->p1.replaced_count++;
  }
  sched->p1.slots[slot] = *frame;
  sched->p1.dirty[slot] = true;
  return true;
}

bool can_tx_scheduler_submit_p2(can_tx_scheduler_t *sched,
                                const can_tx_frame_t *frame) {
  if (sched == NULL || frame == NULL) {
    return false;
  }
  return fifo_push(sched->p2.queue, CAN_TX_P2_CAPACITY, &sched->p2.head,
                   &sched->p2.count, &sched->p2.dropped_count, frame);
}

uint32_t can_tx_scheduler_pump(can_tx_scheduler_t *sched, can_tx_frame_t *out,
                               uint32_t capacity, uint32_t *p0_count) {
  if (sched == NULL || out == NULL) {
    return 0U;
  }
  uint32_t count = 0U;

  // 1) P0：全部出队（绝不因容量/积压被跳过——放不下的留在队首，下轮先出）。
  while (count < capacity &&
         fifo_pop(sched->p0.queue, CAN_TX_P0_CAPACITY, &sched->p0.head,
                  &sched->p0.count, &out[count])) {
    sched->p0_sent++;
    count++;
  }
  if (p0_count != NULL) {
    *p0_count = count; // out 前缀中的 P0 帧数，供硬件拒收时回插
  }

  // 2) P1：发全部脏槽位一次（容量不足的脏槽保留到下轮）。
  for (uint16_t slot = 0; slot < CAN_TX_P1_SLOTS && count < capacity; ++slot) {
    if (sched->p1.dirty[slot]) {
      out[count] = sched->p1.slots[slot];
      sched->p1.dirty[slot] = false;
      sched->p1_sent++;
      count++;
    }
  }

  // 3) P2：受每次 pump 配额约束（TBD-CAN-01），绝不挤占 P0/P1。
  uint8_t quota = CAN_TX_P2_PUMP_QUOTA;
  while (quota > 0U && count < capacity &&
         fifo_pop(sched->p2.queue, CAN_TX_P2_CAPACITY, &sched->p2.head,
                  &sched->p2.count, &out[count])) {
    sched->p2_sent++;
    quota--;
    count++;
  }
  return count;
}

bool can_tx_scheduler_requeue_p0_front(can_tx_scheduler_t *sched,
                                       const can_tx_frame_t *frame) {
  if (sched == NULL || frame == NULL || sched->p0.count >= CAN_TX_P0_CAPACITY) {
    return false;
  }
  sched->p0.head = (uint8_t)((sched->p0.head + CAN_TX_P0_CAPACITY - 1U) %
                             CAN_TX_P0_CAPACITY);
  sched->p0.queue[sched->p0.head] = *frame;
  sched->p0.count++;
  return true;
}

void can_tx_scheduler_note_hw_reject(can_tx_scheduler_t *sched) {
  if (sched != NULL) {
    sched->tx_hw_reject_count++;
  }
}
