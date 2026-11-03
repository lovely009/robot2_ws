#include "control_lease.h"

#include <stddef.h>
#include <string.h>

void control_lease_init(control_lease_state_t *state) {
  if (state != NULL) memset(state, 0, sizeof(*state));
}

void control_lease_revoke(control_lease_state_t *state) {
  if (state == NULL) return;
  state->active = false;
  state->revoke_pending = true;
}

void control_lease_set_session(control_lease_state_t *state, uint64_t session_id) {
  if (state == NULL || state->session_id == session_id) return;
  control_lease_init(state);
  state->session_id = session_id;
  state->revoke_pending = true;
}

static bool deadline_valid(uint32_t now_ms, uint32_t deadline_ms) {
  const uint32_t remaining = deadline_ms - now_ms;
  return remaining > 0U && remaining <= CONTROL_LEASE_MAX_MS;
}

bool control_lease_allows(const control_lease_state_t *state,
    uint64_t lease_id, uint32_t now_ms) {
  return state != NULL && state->active && lease_id != 0U &&
      lease_id == state->lease_id &&
      deadline_valid(now_ms, state->valid_until_tick_ms);
}

void control_lease_service(control_lease_state_t *state, uint32_t now_ms) {
  if (state != NULL && state->active &&
      !deadline_valid(now_ms, state->valid_until_tick_ms)) {
    control_lease_revoke(state);
  }
}

bool control_lease_submit(control_lease_state_t *state, uint64_t session_id,
    uint64_t lease_id, uint32_t sequence, uint32_t valid_until_tick_ms,
    bool active, uint32_t now_ms) {
  if (state == NULL) return false;
  control_lease_service(state, now_ms);
  if (session_id == 0U || session_id != state->session_id || sequence == 0U ||
      session_id > UINT64_MAX / CONTROL_LEASE_SESSION_STRIDE ||
      lease_id / CONTROL_LEASE_SESSION_STRIDE != session_id ||
      lease_id % CONTROL_LEASE_SESSION_STRIDE == 0U) return false;

  if (!active) {
    if (lease_id != state->lease_id || sequence <= state->sequence) return false;
    state->sequence = sequence;
    control_lease_revoke(state);
    return true;
  }
  if (!deadline_valid(now_ms, valid_until_tick_ms) || lease_id < state->lease_id)
    return false;
  if (lease_id == state->lease_id) {
    if (!state->active || sequence <= state->sequence) return false;
  } else {
    // The consumer must discard every old pending command and reset the kernel
    // before consuming intent bearing this newly accepted id.
    state->revoke_pending = true;
  }
  state->lease_id = lease_id;
  state->sequence = sequence;
  state->valid_until_tick_ms = valid_until_tick_ms;
  state->active = true;
  return true;
}

bool control_lease_take_revocation(control_lease_state_t *state) {
  if (state == NULL) return false;
  const bool pending = state->revoke_pending;
  state->revoke_pending = false;
  return pending;
}
