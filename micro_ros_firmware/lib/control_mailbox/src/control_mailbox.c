#include "control_mailbox.h"

#include <string.h>

static void clear_commands(control_mailbox_t *mailbox) {
  for (size_t index = 0; index < mailbox->joint_count; ++index) {
    mailbox->command_slots[index].pending = false;
  }
}

static bool is_supported_frame_size(uint8_t size) {
  return size <= 8U || size == 12U || size == 16U || size == 20U ||
         size == 24U || size == 32U || size == 48U || size == 64U;
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
    ++mailbox->joint_count;
  }
}

bool control_mailbox_submit_command(control_mailbox_t *mailbox,
                                    const motor_control_command_t *command,
                                    uint32_t received_ms) {
  if (mailbox == NULL || command == NULL || command->joint_name[0] == '\0' ||
      memchr(command->joint_name, '\0', sizeof(command->joint_name)) == NULL) {
    if (mailbox != NULL) {
      ++mailbox->invalid_command_count;
    }
    return false;
  }
  if (!mailbox->authorization_requested ||
      (mailbox->agent_state_seen &&
       !mailbox->agent_connected_requested)) {
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
    clear_commands(mailbox);
    mailbox->debug_pending = false;
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
  events->authorization_disable_edge =
      mailbox->authorization_disable_edge;
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
  if (mailbox == NULL || frame == NULL ||
      frame->bus_index >= MOTOR_CONTROL_MAX_BUSES ||
      !is_supported_frame_size(frame->size) ||
      (!frame->is_fd && frame->size > 8U)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_debug_frame_count;
    }
    return false;
  }
  if (mailbox->agent_state_seen && !mailbox->agent_connected_requested) {
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
