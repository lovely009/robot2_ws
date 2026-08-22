#include "supervisor_heartbeat.h"

bool supervisor_heartbeat_evaluate(supervisor_heartbeat_state_t *state,
                                   uint32_t heartbeat, uint32_t now_ms,
                                   uint32_t timeout_ms) {
  if (state == 0) {
    return true;
  }

  if (!state->initialized || state->last_heartbeat != heartbeat) {
    state->last_heartbeat = heartbeat;
    state->last_progress_ms = now_ms;
    state->initialized = true;
    return false;
  }

  return (uint32_t)(now_ms - state->last_progress_ms) >= timeout_ms;
}
