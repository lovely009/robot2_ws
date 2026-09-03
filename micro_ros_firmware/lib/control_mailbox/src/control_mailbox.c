#include "control_mailbox.h"

#include <math.h>
#include <string.h>

static void clear_commands(control_mailbox_t *mailbox) {
  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    mailbox->command_slots[index].pending = false;
  }
}

static void clear_configuration_requests(control_mailbox_t *mailbox) {
  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    mailbox->mode_slots[index].pending = false;
    mailbox->gains_slots[index].pending = false;
  }
}

static bool is_supported_frame_size(uint8_t size) {
  return size <= 8U || size == 12U || size == 16U || size == 20U ||
         size == 24U || size == 32U || size == 48U || size == 64U;
}

static bool is_agent_connected_allowed(const control_mailbox_t *mailbox) {
  return mailbox == NULL || !mailbox->agent_state_seen ||
         mailbox->agent_connected_requested;
}

static void drop_live_mailbox_state(control_mailbox_t *mailbox) {
  if (mailbox == NULL) {
    return;
  }
  clear_commands(mailbox);
  clear_configuration_requests(mailbox);
  mailbox->debug_pending = false;
}

static bool is_valid_command(const motor_control_command_t *command) {
  return command != NULL && command->joint_name[0] != '\0' &&
         memchr(command->joint_name, '\0', sizeof(command->joint_name)) != NULL;
}

static bool valid_joint_name(const char *joint_name) {
  return joint_name != NULL && joint_name[0] != '\0' &&
         memchr(joint_name, '\0', MOTOR_CONTROL_JOINT_NAME_CAPACITY) != NULL;
}

static bool is_valid_debug_frame(const motor_control_tx_action_t *frame) {
  return frame != NULL && frame->bus_index < MOTOR_CONTROL_MAX_BUSES &&
         is_supported_frame_size(frame->size) &&
         (!frame->is_fd && frame->size <= 8U || frame->is_fd);
}

void control_mailbox_init(control_mailbox_t *mailbox,
                          const motor_control_motor_config_t *motors,
                          size_t motor_count) {
  if (mailbox == NULL) {
    return;
  }
  memset(mailbox, 0, sizeof(*mailbox));
  mailbox->authorization_requested = true;
  if (motors == NULL) {
    return;
  }

  for (size_t source = 0;
       source < motor_count && mailbox->joint_count < MOTOR_CONTROL_MAX_MOTORS;
       ++source) {
    const char *name = motors[source].joint_name;
    if (name == NULL || name[0] == '\0') {
      continue;
    }
    const char *terminator =
        memchr(name, '\0', MOTOR_CONTROL_JOINT_NAME_CAPACITY);
    if (terminator == NULL) {
      continue;
    }
    size_t name_size = (size_t)(terminator - name) + 1U;
    memcpy(mailbox->joint_names[mailbox->joint_count], name, name_size);
    mailbox->protocols[mailbox->joint_count] = motors[source].protocol;
    ++mailbox->joint_count;
  }
}

bool control_mailbox_submit_command(control_mailbox_t *mailbox,
                                    const motor_control_command_t *command,
                                    uint32_t received_ms) {
  if (mailbox == NULL || !is_valid_command(command)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_command_count;
    }
    return false;
  }
  if (!mailbox->authorization_requested || !is_agent_connected_allowed(mailbox)) {
    return false;
  }

  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    if (strcmp(mailbox->joint_names[index], command->joint_name) != 0) {
      continue;
    }
    control_mailbox_command_slot_t *slot = &mailbox->command_slots[index];
    if (slot->pending) {
      ++mailbox->command_overwrite_count;
    }
    slot->command = *command;
    slot->received_ms = received_ms;
    slot->sequence = ++mailbox->next_sequence;
    slot->pending = true;
    return true;
  }
  ++mailbox->invalid_command_count;
  return false;
}

bool control_mailbox_take_command(control_mailbox_t *mailbox, size_t index,
                                  motor_control_command_t *command,
                                  uint32_t *received_ms, uint32_t *sequence) {
  if (mailbox == NULL || command == NULL || index >= mailbox->joint_count ||
      !mailbox->command_slots[index].pending) {
    return false;
  }
  const control_mailbox_command_slot_t *slot =
      &mailbox->command_slots[index];
  *command = slot->command;
  if (received_ms != NULL) {
    *received_ms = slot->received_ms;
  }
  if (sequence != NULL) {
    *sequence = slot->sequence;
  }
  mailbox->command_slots[index].pending = false;
  return true;
}

bool control_mailbox_submit_mode(
    control_mailbox_t *mailbox, const motor_control_mode_request_t *request) {
  if (mailbox == NULL || request == NULL ||
      !valid_joint_name(request->joint_name)) {
    return false;
  }
  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    if (strcmp(mailbox->joint_names[index], request->joint_name) == 0) {
      if ((mailbox->protocols[index] == MOTOR_CONTROL_VAXOR &&
           request->mode > MOTOR_PROTOCOL_WRIST_MODE_POSITION) ||
          (mailbox->protocols[index] == MOTOR_CONTROL_CREB &&
           request->mode != MOTOR_PROTOCOL_WRIST_MODE_MIT)) {
        return false;
      }
      mailbox->command_slots[index].pending = false;
      mailbox->mode_slots[index].request = *request;
      mailbox->mode_slots[index].pending = true;
      return true;
    }
  }
  return false;
}

bool control_mailbox_take_mode(control_mailbox_t *mailbox, size_t index,
                               motor_control_mode_request_t *request) {
  if (mailbox == NULL || request == NULL || index >= mailbox->joint_count ||
      !mailbox->mode_slots[index].pending) {
    return false;
  }
  *request = mailbox->mode_slots[index].request;
  mailbox->mode_slots[index].pending = false;
  return true;
}

bool control_mailbox_submit_gains(
    control_mailbox_t *mailbox, const motor_control_gains_request_t *request) {
  if (mailbox == NULL || request == NULL ||
      !valid_joint_name(request->joint_name) || !isfinite(request->kp) ||
      !isfinite(request->kd) || request->kp < 0.0 || request->kd < 0.0) {
    return false;
  }
  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    if (strcmp(mailbox->joint_names[index], request->joint_name) == 0) {
      if ((mailbox->protocols[index] == MOTOR_CONTROL_VAXOR &&
           (request->kp > 0.65535 || request->kd > 655.35)) ||
          (mailbox->protocols[index] == MOTOR_CONTROL_CREB &&
           (request->kp > 500.0 || request->kd > 5.0))) {
        return false;
      }
      mailbox->command_slots[index].pending = false;
      mailbox->gains_slots[index].request = *request;
      mailbox->gains_slots[index].pending = true;
      return true;
    }
  }
  return false;
}

bool control_mailbox_take_gains(control_mailbox_t *mailbox, size_t index,
                                motor_control_gains_request_t *request) {
  if (mailbox == NULL || request == NULL || index >= mailbox->joint_count ||
      !mailbox->gains_slots[index].pending) {
    return false;
  }
  *request = mailbox->gains_slots[index].request;
  mailbox->gains_slots[index].pending = false;
  return true;
}

bool control_mailbox_has_configuration(const control_mailbox_t *mailbox,
                                       size_t index) {
  return mailbox != NULL && index < mailbox->joint_count &&
         (mailbox->mode_slots[index].pending ||
          mailbox->gains_slots[index].pending);
}

void control_mailbox_request_authorization(control_mailbox_t *mailbox,
                                           bool authorized) {
  if (mailbox == NULL) {
    return;
  }
  mailbox->authorization_requested = authorized;
  mailbox->authorization_pending = true;
  if (!authorized) {
    mailbox->authorization_disable_edge = true;
    clear_commands(mailbox);
  }
}

void control_mailbox_request_agent_connected(control_mailbox_t *mailbox,
                                              bool connected) {
  if (mailbox == NULL) {
    return;
  }
  bool disconnected_after_connection =
      !connected && mailbox->agent_state_seen &&
      mailbox->agent_connected_requested;
  mailbox->agent_state_seen = true;
  mailbox->agent_connected_requested = connected;
  mailbox->agent_state_pending = true;
  if (!connected) {
    mailbox->agent_disconnect_edge = true;
    drop_live_mailbox_state(mailbox);
    if (disconnected_after_connection) {
      mailbox->authorization_requested = false;
    }
  }
}

bool control_mailbox_take_events(control_mailbox_t *mailbox,
                                 control_mailbox_events_t *events) {
  if (mailbox == NULL || events == NULL) {
    return false;
  }
  bool pending = mailbox->authorization_pending ||
                 mailbox->authorization_disable_edge ||
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

bool control_mailbox_submit_debug_frame(
    control_mailbox_t *mailbox, const motor_control_tx_action_t *frame) {
  if (mailbox == NULL || !is_valid_debug_frame(frame)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_debug_frame_count;
    }
    return false;
  }
  if (!is_agent_connected_allowed(mailbox)) {
    return false;
  }
  if (mailbox->debug_pending) {
    ++mailbox->debug_overwrite_count;
  }
  mailbox->debug_frame = *frame;
  mailbox->debug_sequence = ++mailbox->next_sequence;
  mailbox->debug_pending = true;
  ++mailbox->debug_submit_count;
  return true;
}

bool control_mailbox_take_debug_frame(control_mailbox_t *mailbox,
                                      motor_control_tx_action_t *frame,
                                      uint32_t *sequence) {
  if (mailbox == NULL || frame == NULL || !mailbox->debug_pending) {
    return false;
  }
  *frame = mailbox->debug_frame;
  if (sequence != NULL) {
    *sequence = mailbox->debug_sequence;
  }
  mailbox->debug_pending = false;
  return true;
}
