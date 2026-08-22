#include "control_mailbox.h"

#include <cassert>
#include <cstring>

static const motor_control_motor_config_t kMotors[] = {
    {"joint_1", MOTOR_CONTROL_RP, 1, 0,
     1.0, 1.0, 0.0, 30.0, 1.0},
    {"joint_2", MOTOR_CONTROL_VAXOR, 2, 0,
     50.0, 0.65, 0.042, 0.008, 1.0},
};

static motor_control_command_t make_command(const char *joint_name,
                                            double position) {
  motor_control_command_t command{};
  std::strcpy(command.joint_name, joint_name);
  command.position = position;
  return command;
}

static void test_per_joint_overwrite_preserves_other_timestamp() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox, kMotors, 2);
  motor_control_command_t joint_1 = make_command("joint_1", 1.0);
  motor_control_command_t joint_2 = make_command("joint_2", 2.0);

  assert(control_mailbox_submit_command(&mailbox, &joint_1, 10U));
  assert(control_mailbox_submit_command(&mailbox, &joint_2, 20U));
  joint_1.position = 3.0;
  assert(control_mailbox_submit_command(&mailbox, &joint_1, 30U));
  assert(mailbox.command_overwrite_count == 1U);

  motor_control_command_t out{};
  uint32_t received_ms = 0;
  uint32_t sequence = 0;
  assert(control_mailbox_take_command(
      &mailbox, 1, &out, &received_ms, &sequence));
  assert(received_ms == 20U);
  assert(out.position == 2.0);
  assert(control_mailbox_take_command(
      &mailbox, 0, &out, &received_ms, &sequence));
  assert(received_ms == 30U);
  assert(out.position == 3.0);
}

static void test_invalid_joint_does_not_block_later_valid_joint() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox, kMotors, 2);
  const motor_control_command_t invalid = make_command("unknown", 1.0);
  const motor_control_command_t valid = make_command("joint_2", 2.0);

  assert(!control_mailbox_submit_command(&mailbox, &invalid, 40U));
  assert(control_mailbox_submit_command(&mailbox, &valid, 40U));
  assert(mailbox.invalid_command_count == 1U);
  motor_control_command_t out{};
  uint32_t received_ms = 0;
  assert(control_mailbox_take_command(
      &mailbox, 1, &out, &received_ms, nullptr));
  assert(received_ms == 40U);
}

static void test_disable_enable_edge_discards_old_command() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox, kMotors, 2);
  const motor_control_command_t command = make_command("joint_1", 1.0);
  assert(control_mailbox_submit_command(&mailbox, &command, 10U));

  control_mailbox_request_authorization(&mailbox, false);
  control_mailbox_request_authorization(&mailbox, true);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.authorization_disable_edge);
  assert(events.authorization_pending);
  assert(events.authorization_requested);
  motor_control_command_t out{};
  assert(!control_mailbox_take_command(
      &mailbox, 0, &out, nullptr, nullptr));
  assert(control_mailbox_submit_command(&mailbox, &command, 11U));
}

static void test_disconnect_reconnect_edge_requires_reauthorization() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox, kMotors, 2);
  control_mailbox_request_agent_connected(&mailbox, true);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mailbox, &events));

  const motor_control_command_t command = make_command("joint_1", 1.0);
  assert(control_mailbox_submit_command(&mailbox, &command, 10U));
  motor_control_tx_action_t debug{};
  debug.bus_index = 0;
  debug.can_id = 0x123U;
  debug.size = 8U;
  assert(control_mailbox_submit_debug_frame(&mailbox, &debug));

  control_mailbox_request_agent_connected(&mailbox, false);
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_disconnect_edge);
  assert(events.agent_state_pending);
  assert(events.agent_connected_requested);
  assert(!events.authorization_requested);
  motor_control_command_t out{};
  assert(!control_mailbox_take_command(
      &mailbox, 0, &out, nullptr, nullptr));
  assert(!control_mailbox_take_debug_frame(&mailbox, &debug, nullptr));
  assert(!control_mailbox_submit_command(&mailbox, &command, 11U));

  control_mailbox_request_authorization(&mailbox, true);
  assert(control_mailbox_submit_command(&mailbox, &command, 12U));
}

static void test_debug_frame_overwrites_and_validates_format_dlc() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox, kMotors, 2);
  motor_control_tx_action_t frame{};
  frame.bus_index = 0;
  frame.can_id = 0x123U;
  frame.size = 9U;
  assert(!control_mailbox_submit_debug_frame(&mailbox, &frame));
  assert(mailbox.invalid_debug_frame_count == 1U);

  frame.size = 8U;
  frame.is_fd = false;
  frame.data[0] = 1U;
  assert(control_mailbox_submit_debug_frame(&mailbox, &frame));

  frame.size = 12U;
  frame.data[0] = 2U;
  assert(!control_mailbox_submit_debug_frame(&mailbox, &frame));
  assert(mailbox.invalid_debug_frame_count == 2U);

  motor_control_tx_action_t out{};
  uint32_t sequence = 0;
  assert(control_mailbox_take_debug_frame(&mailbox, &out, &sequence));
  assert(!out.is_fd);
  assert(out.size == 8U);
  assert(out.data[0] == 1U);

  frame.is_fd = true;
  assert(control_mailbox_submit_debug_frame(&mailbox, &frame));
  frame.size = 8U;
  frame.data[0] = 2U;
  assert(control_mailbox_submit_debug_frame(&mailbox, &frame));
  assert(mailbox.debug_overwrite_count == 1U);

  assert(control_mailbox_take_debug_frame(&mailbox, &out, &sequence));
  assert(out.is_fd);
  assert(out.size == 8U);
  assert(out.data[0] == 2U);
  assert(sequence > 0U);
  assert(!control_mailbox_take_debug_frame(&mailbox, &out, &sequence));
}

int main() {
  test_per_joint_overwrite_preserves_other_timestamp();
  test_invalid_joint_does_not_block_later_valid_joint();
  test_disable_enable_edge_discards_old_command();
  test_disconnect_reconnect_edge_requires_reauthorization();
  test_debug_frame_overwrites_and_validates_format_dlc();
  return 0;
}
