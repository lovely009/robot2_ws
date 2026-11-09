#ifndef AGENT_LIVENESS_H
#define AGENT_LIVENESS_H
#include <stdbool.h>
#include <stdint.h>
#include "firmware_config.h"

#define AGENT_PING_PERIOD_MS (1000U / FIRMWARE_AGENT_PING_HZ)
#define AGENT_PING_TIMEOUT_MS 5U
#define AGENT_PING_FAILURE_LIMIT 3U
#define AGENT_SYNC_PERIOD_MS 10000U
#define AGENT_SYNC_TIMEOUT_MS 2U

// 只由 micro-ROS 任务及其回调访问，运动新鲜度另由实时任务判定。
typedef struct {
  uint32_t received_ms;
  uint32_t probed_ms;
  uint8_t failures;
} agent_liveness_t;

static inline void agent_liveness_init(agent_liveness_t *state, uint32_t now) {
  state->received_ms = state->probed_ms = now - AGENT_PING_PERIOD_MS;
  state->failures = 0U;
}
static inline void agent_liveness_received(agent_liveness_t *state, uint32_t now) {
  state->received_ms = now;
  state->failures = 0U;
}
static inline bool agent_liveness_probe_due(const agent_liveness_t *state, uint32_t now) {
  return (uint32_t)(now - state->received_ms) >= AGENT_PING_PERIOD_MS &&
      (uint32_t)(now - state->probed_ms) >= AGENT_PING_PERIOD_MS;
}
static inline bool agent_liveness_probe_result(agent_liveness_t *state,
    uint32_t now, bool success) {
  state->probed_ms = now;
  if (success) agent_liveness_received(state, now);
  else if (state->failures < AGENT_PING_FAILURE_LIMIT) ++state->failures;
  return state->failures < AGENT_PING_FAILURE_LIMIT;
}
#endif
