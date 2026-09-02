#include "motor_control.h"

#include <cassert>
#include <cmath>
#include <cstring>

static bool nearly_equal(double actual, double expected, double tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

static void assert_vaxor_control_action(
    const motor_control_tx_action_t &action, unsigned node, bool run,
    uint8_t mode) {
  assert(action.can_id == motor_protocol_wrist_control_id(node));
  assert(action.size == 8U);
  assert(!action.is_fd);
  assert(!action.is_extended);
  motor_protocol_wrist_control_t control{};
  assert(motor_protocol_unpack_wrist_control(action.data, action.size,
                                             &control));
  assert(control.run == run);
  assert(control.mode == mode);
}

static motor_protocol_joint_command_t unpack_vaxor_command_action(
    const motor_control_tx_action_t &action, unsigned node) {
  assert(action.can_id == motor_protocol_wrist_command_id(node));
  assert(action.size == 12U);
  assert(action.is_fd);
  assert(!action.is_extended);
  motor_protocol_joint_command_t command{};
  assert(motor_protocol_unpack_wrist_command(action.data, action.size,
                                             &command));
  return command;
}

static void test_timeout_boundaries() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 0));
  assert(motor_control_step(&ctx, 99, actions, 16) == 3U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
  motor_control_step(&ctx, 100, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_control_step(&ctx, 299, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_control_step(&ctx, 300, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
}

static void test_invalid_command_does_not_refresh_freshness() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t valid = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_command_t invalid = valid;
  invalid.position = NAN;
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &valid, 0));
  assert(!motor_control_accept_command(&ctx, &invalid, 90));
  assert(ctx.invalid_command_count == 1U);
  motor_control_step(&ctx, 100, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
}

static void test_infinity_is_rejected() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.0, INFINITY, 0.0};

  motor_control_init(&ctx, &motor, 1);
  assert(!motor_control_accept_command(&ctx, &command, 0));
  assert(ctx.invalid_command_count == 1U);
}

static void test_agent_disconnect_requires_authorization_and_new_command() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  motor_control_set_agent_connected(&ctx, false);
  motor_control_step(&ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);

  motor_control_set_agent_connected(&ctx, true);
  assert(motor_control_accept_command(&ctx, &command, 1));
  motor_control_step(&ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);

  motor_control_set_agent_connected(&ctx, false);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);
  assert(!ctx.runtime[0].command_valid);
  size_t count = motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert(!motor_control_set_authorization(&ctx, true));
  assert(!motor_control_accept_command(&ctx, &command, 2));

  motor_control_set_agent_connected(&ctx, true);
  assert(motor_control_set_authorization(&ctx, true));
  motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(motor_control_accept_command(&ctx, &command, 3));
  motor_control_step(&ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
}

static void test_vaxor_enable_mode_switch_and_gear_ratio_conversion() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 1,
      50.0, 0.65, 0.042, 0.008, 1.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  motor_control_command_t command = {"wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.2, 0.3};
  assert(motor_control_accept_command(&ctx, &command, 0));
  size_t count = motor_control_step(&ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  motor_protocol_joint_command_t encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, 5.0, 0.0001));
  assert(nearly_equal(encoded.velocity, 10.0, 0.01));

  command.mode = MOTOR_PROTOCOL_WRIST_MODE_POSITION;
  command.position = -0.2;
  command.velocity = 0.0;
  assert(motor_control_accept_command(&ctx, &command, 2));
  count = motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert_vaxor_control_action(actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, -10.0, 0.0001));

  command.mode = MOTOR_PROTOCOL_WRIST_MODE_VELOCITY;
  command.position = 0.0;
  command.velocity = -0.3;
  assert(motor_control_accept_command(&ctx, &command, 3));
  count = motor_control_step(&ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_VELOCITY);
  assert_vaxor_control_action(actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_VELOCITY);
  encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.velocity, -15.0, 0.01));
}

static void test_bus_fault_and_manual_disable_are_bus_level() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      {"bus_0_joint", MOTOR_CONTROL_VAXOR, 2, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
      {"bus_1_joint", MOTOR_CONTROL_VAXOR, 3, 1, 50.0, 0.65, 0.042, 0.008, 1.0},
  };
  const motor_control_command_t commands[] = {
      {"bus_0_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.0, 0.0},
      {"bus_1_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.2, 0.0, 0.0},
  };
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 2);
  assert(motor_control_accept_command(&ctx, &commands[0], 0));
  assert(motor_control_accept_command(&ctx, &commands[1], 0));
  size_t count = motor_control_step(&ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 6U);
  motor_control_set_bus_fault(&ctx, 0, true);
  count = motor_control_step(&ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == motor_protocol_wrist_control_id(2));
  assert(actions[0].data[0] == 0U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);

  motor_control_set_bus_fault(&ctx, 0, false);
  assert(motor_control_set_authorization(&ctx, true));
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  assert(motor_control_accept_command(&ctx, &commands[0], 2));
  count = motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == motor_protocol_wrist_control_id(2));
  assert(actions[0].data[0] == 0U);
}

static void test_vaxor_invalid_mode_is_rejected() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {"wrist", 3U, 0.0, 0.0, 0.0};

  motor_control_init(&ctx, &motor, 1);
  assert(!motor_control_accept_command(&ctx, &command, 0));
  assert(ctx.invalid_command_count == 1U);
  assert(!ctx.runtime[0].command_valid);
}

static void test_hard_stop_invalidates_command_across_full_millis_wrap() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {"wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 100U));
  assert(motor_control_step(&ctx, 100U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);
  assert(motor_control_step(&ctx, 400U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(!ctx.runtime[0].command_valid);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
}

static void test_invalid_config_names_are_skipped() {
  motor_control_context_t ctx{};
  char unterminated_name[32];
  std::memset(unterminated_name, 'x', sizeof(unterminated_name));
  const motor_control_motor_config_t motors[] = {
      {nullptr, MOTOR_CONTROL_VAXOR, 2, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
      {"", MOTOR_CONTROL_VAXOR, 3, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
      {unterminated_name, MOTOR_CONTROL_VAXOR, 4, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
      {"valid", MOTOR_CONTROL_VAXOR, 5, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
  };
  const motor_control_command_t command = {"valid", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.0, 0.0};

  motor_control_init(&ctx, motors, 4);
  assert(ctx.motor_count == 1U);
  assert(motor_control_accept_command(&ctx, &command, 0));
}

static void test_config_name_is_deep_copied() {
  motor_control_context_t ctx{};
  char caller_name[32] = "stable_joint";
  const motor_control_motor_config_t motor = {
      caller_name, MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t original_command = {"stable_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.0, 0.0};
  const motor_control_command_t mutated_command = {"mutated", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.2, 0.0, 0.0};

  motor_control_init(&ctx, &motor, 1);
  std::strcpy(caller_name, "mutated");
  assert(motor_control_accept_command(&ctx, &original_command, 0));
  assert(!motor_control_accept_command(&ctx, &mutated_command, 1));
}

static void test_fixed_capacities_are_enforced() {
  motor_control_context_t ctx{};
  motor_control_motor_config_t motors[MOTOR_CONTROL_MAX_MOTORS + 1U]{};
  for (size_t index = 0; index < MOTOR_CONTROL_MAX_MOTORS + 1U; ++index) {
    motors[index] = {"joint", MOTOR_CONTROL_VAXOR,
                     static_cast<uint8_t>(index + 1U), 0,
                     50.0, 0.65, 0.042, 0.008, 1.0};
  }
  motor_control_tx_action_t sentinel{};
  sentinel.can_id = 0x123U;

  motor_control_init(&ctx, motors, MOTOR_CONTROL_MAX_MOTORS + 1U);
  assert(ctx.motor_count == MOTOR_CONTROL_MAX_MOTORS);
  assert(motor_control_step(&ctx, 0, &sentinel, 0) == 0U);
  assert(sentinel.can_id == 0x123U);

  motor_control_context_t vaxor_ctx{};
  const motor_control_motor_config_t vaxor = {"wrist", MOTOR_CONTROL_VAXOR, 2, 0, 50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {"wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[3]{};
  motor_control_init(&vaxor_ctx, &vaxor, 1);
  assert(motor_control_accept_command(&vaxor_ctx, &command, 0));
  assert(motor_control_step(&vaxor_ctx, 0, actions, 2) == 0U);
  assert(!vaxor_ctx.runtime[0].vaxor_enabled);
  assert(motor_control_step(&vaxor_ctx, 0, actions, 3) == 3U);
  assert(vaxor_ctx.runtime[0].vaxor_enabled);
}

static void test_feedback_is_decoded_by_motor_control() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      {"wrist", MOTOR_CONTROL_VAXOR, 2, 0, 50.0, 0.65, 0.042, 0.008, 1.0},
  };
  motor_control_init(&ctx, motors, 1U);

  const motor_protocol_joint_state_t wrist_input{5.0, -10.0, 2.0};
  motor_control_rx_frame_t wrist_frame{};
  wrist_frame.bus_index = 0U;
  wrist_frame.can_id = motor_protocol_wrist_reply_id(2U);
  wrist_frame.size = 12U;
  motor_protocol_pack_wrist_reply(2U, &wrist_input, wrist_frame.data);
  assert(motor_control_accept_feedback(&ctx, &wrist_frame));

  motor_control_joint_state_t state{};
  assert(motor_control_get_joint_state(&ctx, 0U, &state));
  assert(std::strcmp(state.joint_name, "wrist") == 0);
  assert(nearly_equal(state.position, wrist_input.position / 50.0, 0.0001));
  assert(nearly_equal(state.velocity, wrist_input.velocity / 50.0, 0.001));
  assert(nearly_equal(state.effort, wrist_input.effort * 0.042 * 50.0 * 0.65, 0.01));

  motor_control_rx_frame_t unmatched{};
  unmatched.bus_index = 1U;
  unmatched.can_id = 1U;
  unmatched.size = 12U;
  assert(!motor_control_accept_feedback(&ctx, &unmatched));
  assert(ctx.valid_feedback_count == 1U);
  assert(ctx.invalid_feedback_count == 1U);
}

int main() {
  test_timeout_boundaries();
  test_invalid_command_does_not_refresh_freshness();
  test_infinity_is_rejected();
  test_agent_disconnect_requires_authorization_and_new_command();
  test_vaxor_enable_mode_switch_and_gear_ratio_conversion();
  test_bus_fault_and_manual_disable_are_bus_level();
  test_vaxor_invalid_mode_is_rejected();
  test_hard_stop_invalidates_command_across_full_millis_wrap();
  test_invalid_config_names_are_skipped();
  test_config_name_is_deep_copied();
  test_fixed_capacities_are_enforced();
  test_feedback_is_decoded_by_motor_control();
  return 0;
}
