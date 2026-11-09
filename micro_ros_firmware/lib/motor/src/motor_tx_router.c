// One bounded scheduler pump plus protocol batching and HAL submission.

#include "motor_tx_router.h"

#include <string.h>

_Static_assert(sizeof(motor_tx_router_t) <= 4096U,
    "motor_tx_router static scratch must stay below 4 KiB");

static uint32_t min_u32(uint32_t left, uint32_t right) {
  return left < right ? left : right;
}

static uint32_t subtract_floor_u32(uint32_t value, uint32_t amount) {
  return value > amount ? value - amount : 0U;
}

static void to_protocol_frame(uint8_t bus_index, uint16_t tx_slot,
    const can_tx_frame_t *input,
    motor_protocol_tx_frame_t *output) {
  memset(output, 0, sizeof(*output));
  output->bus_index = bus_index;
  output->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  output->tx_slot = tx_slot;
  output->can_id = input->can_id;
  output->is_extended = input->is_extended;
  output->is_fd = input->is_fd;
  output->is_brs = input->is_brs;
  output->size = input->size;
  output->node = input->node;
  output->aggregate_only = input->aggregate_only;
  const size_t copy_size = input->size <= MOTOR_PROTOCOL_MAX_FRAME_BYTES
                               ? input->size
                               : MOTOR_PROTOCOL_MAX_FRAME_BYTES;
  memcpy(output->data, input->data, copy_size);
}

static bool to_can_frame(const motor_protocol_tx_frame_t *input,
    can_tx_frame_t *output) {
  if (input == NULL || output == NULL ||
      input->size > CAN_TX_FRAME_BYTES) {
    return false;
  }
  memset(output, 0, sizeof(*output));
  output->can_id = input->can_id;
  output->is_extended = input->is_extended;
  output->is_fd = input->is_fd;
  output->is_brs = input->is_brs;
  output->size = input->size;
  memcpy(output->data, input->data, input->size);
  return true;
}

static size_t load_scheduler_p1_lookahead(
    motor_tx_router_t *router, const can_tx_scheduler_t *scheduler,
    uint8_t bus_index, uint16_t first_slot, size_t capacity) {
  size_t count = 0U;
  for (uint16_t slot = first_slot;
      slot < CAN_TX_P1_SLOTS && count < capacity; ++slot) {
    if (!scheduler->p1.dirty[slot]) {
      continue;
    }
    to_protocol_frame(bus_index, slot, &scheduler->p1.slots[slot],
        &router->p1_lookahead[count]);
    ++count;
  }
  return count;
}

static uint32_t select_p1_logical_limit(
    motor_tx_router_t *router, const can_tx_scheduler_t *scheduler,
    uint8_t bus_index, uint32_t logical_capacity,
    uint32_t p0_logical_count) {
  const uint32_t p2_reserve =
      min_u32(scheduler->p2.count, CAN_TX_P2_PUMP_QUOTA);
  const uint32_t physical_p1_limit = subtract_floor_u32(
      subtract_floor_u32(MOTOR_TX_ROUTER_PHYSICAL_BUDGET, p0_logical_count),
      p2_reserve);
  const uint32_t logical_p1_limit = subtract_floor_u32(
      subtract_floor_u32(logical_capacity, p0_logical_count), p2_reserve);
  const size_t batch_capacity = motor_protocol_p1_batch_max();

  uint32_t physical_count = 0U;
  uint32_t logical_count = 0U;
  uint16_t first_slot = 0U;
  while (physical_count < physical_p1_limit &&
      logical_count < logical_p1_limit) {
    size_t available = load_scheduler_p1_lookahead(
        router, scheduler, bus_index, first_slot, batch_capacity);
    if (available == 0U) {
      break;
    }
    const uint32_t logical_remaining = logical_p1_limit - logical_count;
    if (available > logical_remaining) {
      available = logical_remaining;
    }

    motor_protocol_tx_frame_t combined = {0};
    size_t consumed = motor_protocol_aggregate_p1_frames(
        router->p1_lookahead, available, &combined);
    if (consumed == 0U || consumed > available) {
      consumed = 1U;
    }
    if (logical_count + consumed > logical_p1_limit) {
      break;
    }
    logical_count += (uint32_t)consumed;
    ++physical_count;
    first_slot = (uint16_t)(router->p1_lookahead[consumed - 1U].tx_slot + 1U);
  }
  return logical_count;
}

static bool submit_physical_frame(motor_tx_router_submit_fn submit,
    void *context, uint8_t bus_index,
    const can_tx_frame_t *frame,
    motor_tx_router_stats_t *stats) {
  ++stats->physical_attempts;
  if (submit(context, bus_index, frame)) {
    return true;
  }
  ++stats->hardware_rejections;
  return false;
}

void motor_tx_router_init(motor_tx_router_t *router) {
  if (router != NULL) {
    memset(router, 0, sizeof(*router));
  }
}

motor_tx_router_stats_t motor_tx_router_pump(
    motor_tx_router_t *router, can_tx_scheduler_t *scheduler,
    uint8_t bus_index, motor_tx_router_submit_fn submit, void *context) {
  motor_tx_router_stats_t stats = {0};
  if (router == NULL || scheduler == NULL || submit == NULL ||
      bus_index >= CAN_TX_BUS_COUNT) {
    return stats;
  }

  const bool p0_pending = scheduler->p0.count != 0U;
  const uint32_t p0_logical_count =
      min_u32(scheduler->p0.count, MOTOR_TX_ROUTER_PHYSICAL_BUDGET);
  const uint32_t logical_capacity =
      p0_pending ? p0_logical_count
                 : MOTOR_TX_ROUTER_LOGICAL_CAPACITY;
  const uint32_t p1_limit = select_p1_logical_limit(
      router, scheduler, bus_index, logical_capacity, p0_logical_count);

  uint32_t p0_count = 0U;
  uint32_t p1_count = 0U;
  const uint32_t count = can_tx_scheduler_pump(
      scheduler, router->logical_frames, logical_capacity, p1_limit,
      &p0_count, &p1_count);
  stats.p0_logical_frames = p0_count;
  stats.p1_logical_frames = p1_count;
  stats.p2_logical_frames = count - p0_count - p1_count;

  uint32_t retry_p0_from = p0_count;
  for (uint32_t i = 0U; i < p0_count; ++i) {
    if (!submit_physical_frame(submit, context, bus_index,
        &router->logical_frames[i], &stats)) {
      retry_p0_from = i;
      break;
    }
  }

  // Stop at the first rejected P0. Reinsert it and every unattempted P0 in
  // reverse order so the next pump preserves the original safety FIFO.
  for (uint32_t i = p0_count; i > retry_p0_from; --i) {
    const uint32_t index = i - 1U;
    if (can_tx_scheduler_requeue_p0_front(
        scheduler, &router->logical_frames[index])) {
      ++stats.p0_requeued;
    }
  }

  const uint32_t p1_begin = p0_count;
  const uint32_t p1_end = p0_count + p1_count;
  const size_t batch_capacity = motor_protocol_p1_batch_max();
  uint32_t i = p1_begin;
  while (i < p1_end) {
    size_t available = (size_t)(p1_end - i);
    if (available > batch_capacity) {
      available = batch_capacity;
    }
    for (size_t axis = 0U; axis < available; ++axis) {
      to_protocol_frame(bus_index, UINT16_MAX,
          &router->logical_frames[i + axis],
          &router->p1_lookahead[axis]);
    }

    motor_protocol_tx_frame_t physical = {0};
    size_t consumed = motor_protocol_aggregate_p1_frames(
        router->p1_lookahead, available, &physical);
    if (consumed == 0U || consumed > available) {
      consumed = 1U;
    }
    can_tx_frame_t can_frame = {0};
    const bool aggregated = consumed > 1U ||
        physical.can_id != router->logical_frames[i].can_id ||
        physical.size != router->logical_frames[i].size;
    if (aggregated && to_can_frame(&physical, &can_frame)) {
      (void)submit_physical_frame(submit, context, bus_index, &can_frame,
          &stats);
      i += (uint32_t)consumed;
      continue;
    }

    // 兜底：只允许聚合上线的帧绝不能被原样发出（其 ID 在线上有歧义）。
    if (router->logical_frames[i].aggregate_only) {
      ++stats.aggregate_dropped;
      ++i;
      continue;
    }

    (void)submit_physical_frame(submit, context, bus_index,
        &router->logical_frames[i], &stats);
    ++i;
  }

  for (uint32_t p2 = p1_end; p2 < count; ++p2) {
    (void)submit_physical_frame(submit, context, bus_index,
        &router->logical_frames[p2], &stats);
  }
  return stats;
}
