// MCU-local expiring control authority. No ROS, RTOS or dynamic allocation.
// Caller must serialize every access with the same critical section.
#ifndef CONTROL_LEASE_H
#define CONTROL_LEASE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_LEASE_MAX_MS 300U
#define CONTROL_LEASE_SESSION_STRIDE UINT64_C(1000000)

typedef struct {
  uint64_t session_id;
  uint64_t lease_id;
  uint32_t sequence;
  uint32_t valid_until_tick_ms;
  bool active;
  // A new id must revoke old kernel intent even if it arrives before expiry.
  bool revoke_pending;
} control_lease_state_t;

void control_lease_init(control_lease_state_t *state);
void control_lease_set_session(control_lease_state_t *state, uint64_t session_id);
bool control_lease_submit(control_lease_state_t *state, uint64_t session_id,
    uint64_t lease_id, uint32_t sequence, uint32_t valid_until_tick_ms,
    bool active, uint32_t now_ms);
bool control_lease_allows(const control_lease_state_t *state,
    uint64_t lease_id, uint32_t now_ms);
// Expiry is latched: no later packet for this id can resurrect it.
void control_lease_service(control_lease_state_t *state, uint32_t now_ms);
void control_lease_revoke(control_lease_state_t *state);
bool control_lease_take_revocation(control_lease_state_t *state);

#ifdef __cplusplus
}
#endif
#endif
