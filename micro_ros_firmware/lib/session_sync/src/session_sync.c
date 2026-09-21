#include "session_sync.h"

void session_sync_apply(motor_control_context_t *ctx,
                        const control_mailbox_events_t *events,
                        bool session_authorized, uint32_t now_ms) {
  if (ctx == NULL || events == NULL) {
    return;
  }
  if (events->agent_state_pending) {
    motor_control_set_agent_connected(ctx, events->agent_connected_requested,
                                      now_ms);
  }
  motor_control_set_authorization(ctx, session_authorized, now_ms);
}
