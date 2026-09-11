#include "motor_event_relay.h"

void motor_event_relay_init(motor_event_relay_t *relay) {
  if (relay == NULL) {
    return;
  }
  relay->head = 0U;
  relay->count = 0U;
  relay->dropped_count = 0U;
  // slots 无需清零：count==0 即逻辑空，槽内残留不可达。
}

void motor_event_relay_push(motor_event_relay_t *relay,
                            const motor_control_event_t *event) {
  if (relay == NULL || event == NULL) {
    return;
  }
  if (relay->count == MOTOR_EVENT_RELAY_CAPACITY) {
    // 满队时覆盖最旧的 head 槽，再前移 head，保持严格 FIFO。
    relay->slots[relay->head] = *event;
    relay->head = (relay->head + 1U) % MOTOR_EVENT_RELAY_CAPACITY;
    relay->dropped_count++;
    return;
  }
  relay->slots[(relay->head + relay->count) % MOTOR_EVENT_RELAY_CAPACITY] = *event;
  relay->count++;
}

size_t motor_event_relay_drain(motor_event_relay_t *relay,
                               motor_control_event_t *out, size_t capacity) {
  if (relay == NULL || out == NULL || capacity == 0U) {
    return 0U;
  }
  const size_t count = relay->count < capacity ? relay->count : capacity;
  for (size_t i = 0; i < count; ++i) {
    out[i] = relay->slots[(relay->head + i) % MOTOR_EVENT_RELAY_CAPACITY];
  }
  relay->head = (relay->head + count) % MOTOR_EVENT_RELAY_CAPACITY;
  relay->count -= count;
  return count;
}

uint32_t motor_event_relay_dropped_count(const motor_event_relay_t *relay) {
  return relay != NULL ? relay->dropped_count : 0U;
}
