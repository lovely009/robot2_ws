#include "publish_schedule.h"

publish_schedule_output_t publish_schedule_step(bool control_due, bool status_due,
    bool result_pending, bool event_pending, bool topology_due,
    bool device_pending, publish_schedule_state_t *state) {
  publish_schedule_output_t out = {PUBLISH_RELIABLE_NONE, state->last_spare};
  const bool spare_pending = result_pending || event_pending || topology_due || device_pending;
  if ((control_due || status_due) && (state->periodic_streak < 2U || !spare_pending)) {
    const bool control = control_due && (!status_due ||
        state->last_periodic != PUBLISH_RELIABLE_CONTROL_STATES);
    out.reliable = control ? PUBLISH_RELIABLE_CONTROL_STATES : PUBLISH_RELIABLE_MOTOR_STATUS;
    state->last_periodic = out.reliable;
    if (state->periodic_streak < 2U) ++state->periodic_streak;
    return out;
  }
  const bool pending[] = {result_pending, event_pending, topology_due, device_pending};
  const uint8_t start = state->last_spare >= PUBLISH_RELIABLE_CONFIG_RESULT &&
      state->last_spare <= PUBLISH_RELIABLE_DEVICE_PARAM
      ? (uint8_t)((state->last_spare - PUBLISH_RELIABLE_CONFIG_RESULT + 1U) % 4U) : 0U;
  for (uint8_t i = 0; i < 4U; ++i) {
    const uint8_t index = (start + i) % 4U;
    if (pending[index]) {
      out.reliable = (publish_reliable_t)(PUBLISH_RELIABLE_CONFIG_RESULT + index);
      out.spare_served = out.reliable;
      state->last_spare = out.reliable;
      state->periodic_streak = 0U;
      return out;
    }
  }
  state->periodic_streak = 0U;
  return out;
}
