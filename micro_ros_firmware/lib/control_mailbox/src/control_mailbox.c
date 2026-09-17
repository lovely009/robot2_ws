#include "control_mailbox.h"

#include <math.h>
#include <string.h>

static bool config_request_is_valid(
    const control_mailbox_config_request_t *request) {
  if (request == NULL || request->request_id == 0U) {
    return false;
  }

  switch (request->kind) {
    case CONTROL_MAILBOX_CONFIG_SET_MODE:
      return true;
    case CONTROL_MAILBOX_CONFIG_SET_ENABLE:
      return true; // bool 无可校验域
    case CONTROL_MAILBOX_CONFIG_SET_GAINS:
      return isfinite(request->value.gains.kp) &&
             isfinite(request->value.gains.kd) &&
             request->value.gains.kp >= 0.0F &&
             request->value.gains.kd >= 0.0F;
    case CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS:
      return isfinite(request->value.position_limits.min_position_rad) &&
             isfinite(request->value.position_limits.max_position_rad) &&
             request->value.position_limits.min_position_rad <
                 request->value.position_limits.max_position_rad;
    case CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT:
      return isfinite(request->value.max_velocity_rad_s) &&
             request->value.max_velocity_rad_s > 0.0F;
    case CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT:
      return isfinite(request->value.max_torque_nm) &&
             request->value.max_torque_nm > 0.0F;
    case CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS:
    case CONTROL_MAILBOX_CONFIG_SET_ZERO:
      return true; // 无载荷，只凭 request_id/motor_id
    default:
      return false;
  }
}

static uint8_t config_queue_index(uint8_t head, uint8_t offset) {
  return (uint8_t)((head + offset) % CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY);
}

static void clear_live_inputs(control_mailbox_t *mailbox) {
  control_mailbox_clear_command(mailbox);
  control_mailbox_clear_config(mailbox);
}

bool control_mailbox_session_authorized(const control_mailbox_t *mailbox) {
  return mailbox != NULL && (!mailbox->agent_state_seen ||
                             mailbox->agent_connected_requested);
}

static bool agent_is_connected_or_unknown(const control_mailbox_t *mailbox) {
  return control_mailbox_session_authorized(mailbox);
}

static void revoke_authorization(control_mailbox_t *mailbox) {
  if (mailbox->authorization_requested) {
    mailbox->authorization_requested = false;
    mailbox->authorization_pending = true;
    mailbox->authorization_disable_edge = true;
  }
  control_mailbox_clear_command(mailbox);
}

void control_mailbox_init(control_mailbox_t *mailbox) {
  if (mailbox == NULL) {
    return;
  }
  memset(mailbox, 0, sizeof(*mailbox));
  control_command_mailbox_init(&mailbox->command);
  // 在首次收到 Agent 状态前允许启动阶段写入；真正的安全裁决仍在 runtime。
  mailbox->authorization_requested = true;
}

bool control_mailbox_submit_command(
    control_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot) {
  if (mailbox == NULL) {
    return false;
  }
  // 授权/会话门拒绝计数可见（快照本身非法由 publish 侧 invalid_snapshot_count
  // 计数——两处互不重复：门拒绝发生在数据校验之前）。
  if (!mailbox->authorization_requested || !agent_is_connected_or_unknown(mailbox)) {
    ++mailbox->command_rejected_count;
    return false;
  }
  return control_command_mailbox_publish(&mailbox->command, snapshot);
}

bool control_mailbox_take_command(control_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot) {
  if (mailbox == NULL) {
    return false;
  }
  return control_command_mailbox_take(&mailbox->command, snapshot);
}

void control_mailbox_clear_command(control_mailbox_t *mailbox) {
  if (mailbox != NULL) {
    control_command_mailbox_clear(&mailbox->command);
  }
}

bool control_mailbox_submit_config(
    control_mailbox_t *mailbox,
    const control_mailbox_config_request_t *request) {
  if (mailbox == NULL) {
    return false;
  }
  // 拒绝类别可区分：会话门 / 非法请求 / FIFO 满（诊断语义见头文件）。
  // 与 submit_command 同序：门先于数据校验——断链期的非法载荷统一
  // 归因到会话门，两条提交路径的诊断口径一致。
  if (!agent_is_connected_or_unknown(mailbox)) {
    ++mailbox->config_session_count;
    return false;
  }
  if (!config_request_is_valid(request)) {
    ++mailbox->config_invalid_count;
    return false;
  }
  if (mailbox->config_count >= CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY) {
    ++mailbox->config_overflow_count;
    return false;
  }

  const uint8_t index = config_queue_index(
      mailbox->config_head, mailbox->config_count);
  mailbox->config_queue[index] = *request;
  ++mailbox->config_count;
  return true;
}

bool control_mailbox_take_config(
    control_mailbox_t *mailbox,
    control_mailbox_config_request_t *request) {
  if (mailbox == NULL || request == NULL || mailbox->config_count == 0U) {
    return false;
  }

  *request = mailbox->config_queue[mailbox->config_head];
  mailbox->config_head = config_queue_index(mailbox->config_head, 1U);
  --mailbox->config_count;
  return true;
}

void control_mailbox_clear_config(control_mailbox_t *mailbox) {
  if (mailbox != NULL) {
    mailbox->config_head = 0U;
    mailbox->config_count = 0U;
  }
}

void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized) {
  if (mailbox == NULL) {
    return;
  }

  const bool changed = mailbox->authorization_requested != authorized;
  if (!changed) {
    return;
  }

  mailbox->authorization_requested = authorized;
  mailbox->authorization_pending = true;
  if (!authorized) {
    mailbox->authorization_disable_edge = true;
    control_mailbox_clear_command(mailbox);
  }
}

void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                             bool connected) {
  if (mailbox == NULL) {
    return;
  }

  const bool disconnected_after_connection =
      mailbox->agent_state_seen && mailbox->agent_connected_requested &&
      !connected;
  const bool state_changed =
      !mailbox->agent_state_seen ||
      mailbox->agent_connected_requested != connected;

  mailbox->agent_state_seen = true;
  mailbox->agent_connected_requested = connected;
  if (state_changed) {
    mailbox->agent_state_pending = true;
  }

  if (!connected) {
    clear_live_inputs(mailbox);
    if (disconnected_after_connection) {
      revoke_authorization(mailbox);
      mailbox->agent_disconnect_edge = true;
    }
  } else if (!mailbox->authorization_requested) {
    // 会话恢复即重立授权电平（motor_enable 话题已删，授权由会话状态派生）。
    // 内核侧仍按生命周期冻结 FAULT/OFFLINE/CONFLICT 表项，安全语义不放宽。
    mailbox->authorization_requested = true;
    mailbox->authorization_pending = true;
  }
}

bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events) {
  if (mailbox == NULL || events == NULL) {
    return false;
  }

  const bool pending =
      mailbox->authorization_pending || mailbox->authorization_disable_edge ||
      mailbox->agent_state_pending || mailbox->agent_disconnect_edge;
  memset(events, 0, sizeof(*events));
  events->authorization_requested = mailbox->authorization_requested;
  events->authorization_pending = mailbox->authorization_pending;
  events->authorization_disable_edge = mailbox->authorization_disable_edge;
  events->agent_connected_requested = mailbox->agent_connected_requested;
  events->agent_state_pending = mailbox->agent_state_pending;
  events->agent_disconnect_edge = mailbox->agent_disconnect_edge;

  mailbox->authorization_pending = false;
  mailbox->authorization_disable_edge = false;
  mailbox->agent_state_pending = false;
  mailbox->agent_disconnect_edge = false;
  return pending;
}
