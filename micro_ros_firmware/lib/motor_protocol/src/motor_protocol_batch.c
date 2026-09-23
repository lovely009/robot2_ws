// Protocol adapter dispatch for optional P1 logical-to-physical batching.

#include "motor_protocol_adapter.h"

size_t motor_protocol_aggregate_p1_frames(
    const motor_protocol_tx_frame_t *frames, size_t available,
    motor_protocol_tx_frame_t *out) {
  if (frames == NULL || available == 0U || out == NULL) {
    return 0U;
  }
  *out = frames[0];
  if (frames[0].tx_class != MOTOR_PROTOCOL_TX_CLASS_P1) {
    return 1U;
  }

  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  for (size_t i = 0U; i < adapter_count; ++i) {
    const motor_protocol_adapter_t *adapter = adapters[i];
    if (adapter->aggregate_p1_frames == NULL) {
      continue;
    }
    size_t limit = adapter->p1_batch_max;
    if (limit < 2U) {
      continue;
    }
    if (limit > MOTOR_PROTOCOL_P1_BATCH_MAX) {
      limit = MOTOR_PROTOCOL_P1_BATCH_MAX;
    }
    if (limit > available) {
      limit = available;
    }

    motor_protocol_tx_frame_t candidate = {0};
    const size_t consumed =
        adapter->aggregate_p1_frames(frames, limit, &candidate);
    if (consumed > 1U && consumed <= limit &&
        candidate.bus_index == frames[0].bus_index &&
        candidate.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1 &&
        candidate.size <= MOTOR_PROTOCOL_MAX_FRAME_BYTES) {
      *out = candidate;
      return consumed;
    }
  }
  return 1U;
}

size_t motor_protocol_p1_batch_max(void) {
  size_t max_batch = 1U;
  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  for (size_t i = 0U; i < adapter_count; ++i) {
    const size_t adapter_max = adapters[i]->p1_batch_max;
    if (adapter_max > max_batch) {
      max_batch = adapter_max;
    }
  }
  return max_batch > MOTOR_PROTOCOL_P1_BATCH_MAX
             ? MOTOR_PROTOCOL_P1_BATCH_MAX
             : max_batch;
}
