#include "session_sync.h"

void session_sync_apply(motor_control_context_t *ctx,
    const control_mailbox_events_t *events,
    bool motion_authorized, uint32_t now_ms) {
  if (ctx == NULL || events == NULL) {
    return;
  }
  if (events->agent_state_pending) {
    motor_control_set_agent_connected(ctx, events->agent_connected_requested,
        now_ms);
  }
  // 在线电平只保留既有授权，绝不创建授权；只有已执行的新策略可以复权。
  // 未授权电平必须幂等，否则每拍撤权会中止自动身份读取/参数回读事务。
  if (!motion_authorized && motor_control_is_authorized(ctx)) {
    (void)motor_control_set_authorization(ctx, false, now_ms);
  }
}
