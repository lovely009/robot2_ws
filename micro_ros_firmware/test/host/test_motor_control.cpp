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

static motor_control_motor_config_t creb_config(
    const char *joint_name, uint8_t id, uint8_t bus_index,
    double torque_constant, double torque_limit) {
  return {joint_name, MOTOR_CONTROL_CREB, id, bus_index,
          101.0, 1.0, torque_constant, 30.0, 1.0,
          MOTOR_PROTOCOL_PI, MOTOR_PROTOCOL_PI, torque_limit};
}

static void assert_creb_action(const motor_control_tx_action_t &action,
                               unsigned node, bool enabled) {
  uint32_t command_id = 0U;
  assert(motor_protocol_creb_command_id(node, &command_id));
  assert(action.can_id == command_id);
  assert(action.size == 12U);
  assert(action.is_fd);
  assert(!action.is_extended);
  assert((action.data[0] & 0xc0U) == (enabled ? 0xc0U : 0x00U));
  assert((action.data[0] & 0x1eU) == 0x0cU);
}

static void assert_sync_action(const motor_control_tx_action_t &action,
                               uint8_t bus_index) {
  assert(action.bus_index == bus_index);
  assert(action.can_id == 0x080U);
  assert(action.size == 0U);
  assert(!action.is_fd);
  assert(!action.is_extended);
}

static void test_creb_waits_100_ms_after_enable_then_runs_and_times_out() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor =
      creb_config("creb", 1U, 0U, 0.086, 45.0);
  const motor_control_command_t command = {
      "creb", 0.5, 0.25, 2.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_accept_command(&ctx, &command, 0U));
  size_t count = motor_control_step(
      &ctx, 0U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert_creb_action(actions[0], 1U, true);
  assert_sync_action(actions[1], 0U);

  assert(motor_control_accept_command(&ctx, &command, 99U));
  count = motor_control_step(&ctx, 99U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert_sync_action(actions[0], 0U);

  assert(motor_control_accept_command(&ctx, &command, 100U));
  count = motor_control_step(&ctx, 100U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert_creb_action(actions[0], 1U, true);
  assert_sync_action(actions[1], 0U);

  count = motor_control_step(&ctx, 200U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert_creb_action(actions[0], 1U, true);
  assert_sync_action(actions[1], 0U);

  count = motor_control_step(&ctx, 400U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert_creb_action(actions[0], 1U, false);
  assert_sync_action(actions[1], 0U);

  count = motor_control_step(&ctx, 410U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert_sync_action(actions[0], 0U);
}

static void test_creb_sync_is_once_per_configured_bus() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      creb_config("creb_1", 1U, 0U, 0.086, 45.0),
      creb_config("creb_2", 2U, 0U, 0.077, 11.0),
      creb_config("creb_3", 3U, 1U, 0.077, 11.0),
      {"wrist", MOTOR_CONTROL_VAXOR, 1U, 2U,
       50.0, 0.65, 0.042, 0.008, 1.0},
  };
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 4U);
  size_t count = motor_control_step(
      &ctx, 0U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert_sync_action(actions[0], 0U);
  assert_sync_action(actions[1], 1U);
}

static void test_creb_feedback_converts_motor_speed_and_current() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor =
      creb_config("creb", 2U, 0U, 0.086, 45.0);
  motor_control_init(&ctx, &motor, 1U);

  motor_control_rx_frame_t frame{};
  frame.bus_index = 0U;
  assert(motor_protocol_creb_feedback_id(2U, &frame.can_id));
  frame.size = 12U;
  frame.is_fd = true;
  const uint8_t payload[12]{0x07U, 0x1cU, 0x02U, 0x58U,
                            0x05U, 0xdcU, 0U, 0U,
                            0U, 0U, 6U, 0xc0U};
  std::memcpy(frame.data, payload, sizeof(payload));
  assert(motor_control_accept_feedback(&ctx, &frame));

  motor_control_joint_state_t state{};
  assert(motor_control_get_joint_state(&ctx, 0U, &state));
  assert(nearly_equal(state.position, 10.0 * MOTOR_PROTOCOL_PI / 180.0,
                      1e-12));
  assert(nearly_equal(state.velocity,
                      600.0 * 2.0 * MOTOR_PROTOCOL_PI / 60.0 / 101.0,
                      1e-12));
  assert(nearly_equal(state.effort, 1.5 * 0.086 * 101.0, 1e-12));

  frame.data[11] |= 0x20U;
  assert(!motor_control_accept_feedback(&ctx, &frame));
}

static void test_creb_command_uses_mit_wire_mode() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor =
      creb_config("creb", 1U, 0U, 0.086, 45.0);
  const motor_control_command_t command = {
      "creb", 0.5, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_accept_command(&ctx, &command, 0U));
  assert(motor_control_step(&ctx, 0U, actions,
                            MOTOR_CONTROL_MAX_TX_ACTIONS) == 2U);
  assert_creb_action(actions[0], 1U, true);
}

static void test_gains_apply_only_to_subsequent_commands() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", 0.1, 0.2, 0.3};

  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_accept_command(&ctx, &command, 0U));
  assert(nearly_equal(ctx.runtime[0].command.kp, 0.008, 1e-12));
  assert(nearly_equal(ctx.runtime[0].command.kd, 1.0, 1e-12));

  assert(motor_control_set_gains(&ctx, "wrist", 0.02, 2.0));
  assert(nearly_equal(ctx.runtime[0].command.kp, 0.008, 1e-12));
  assert(nearly_equal(ctx.runtime[0].command.kd, 1.0, 1e-12));
  assert(motor_control_accept_command(&ctx, &command, 1U));
  assert(nearly_equal(ctx.runtime[0].command.kp, 0.02, 1e-12));
  assert(nearly_equal(ctx.runtime[0].command.kd, 2.0, 1e-12));

  assert(!motor_control_set_gains(&ctx, "wrist", NAN, 1.0));
  assert(!motor_control_set_gains(&ctx, "wrist", -0.1, 1.0));
  assert(!motor_control_set_gains(&ctx, "unknown", 0.1, 1.0));
}

static void test_mode_change_stops_vaxor_and_requires_new_command() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", 0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_accept_command(&ctx, &command, 0U));
  assert(motor_control_step(&ctx, 0U, actions,
                            MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);
  assert(motor_control_set_mode(&ctx, "wrist",
                                MOTOR_PROTOCOL_WRIST_MODE_POSITION));
  assert(!ctx.runtime[0].command_valid);
  assert(motor_control_step(&ctx, 1U, actions,
                            MOTOR_CONTROL_MAX_TX_ACTIONS) == 2U);
  assert_vaxor_control_action(actions[0], 2U, false,
                              MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(actions[1], 2U, false,
                              MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert(ctx.runtime[0].active_mode == MOTOR_PROTOCOL_WRIST_MODE_POSITION);

  assert(motor_control_accept_command(&ctx, &command, 2U));
  assert(motor_control_step(&ctx, 2U, actions,
                            MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);
  assert_vaxor_control_action(actions[0], 2U, false,
                              MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert_vaxor_control_action(actions[1], 2U, true,
                              MOTOR_PROTOCOL_WRIST_MODE_POSITION);

  assert(!motor_control_set_mode(&ctx, "wrist", 3U));
  assert(!motor_control_set_mode(&ctx, "unknown",
                                 MOTOR_PROTOCOL_WRIST_MODE_MIT));
}

static void test_creb_rejects_non_mit_mode_change() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor =
      creb_config("creb", 1U, 0U, 0.086, 45.0);
  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_set_mode(&ctx, "creb",
                                MOTOR_PROTOCOL_WRIST_MODE_MIT));
  assert(!motor_control_set_mode(&ctx, "creb",
                                 MOTOR_PROTOCOL_WRIST_MODE_POSITION));
}

static void test_disabled_vaxor_changes_mode_before_any_command() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1U);
  assert(motor_control_set_mode(&ctx, "wrist",
                                MOTOR_PROTOCOL_WRIST_MODE_POSITION));
  assert(motor_control_step(&ctx, 0U, actions,
                            MOTOR_CONTROL_MAX_TX_ACTIONS) == 2U);
  assert_vaxor_control_action(actions[0], 2U, false,
                              MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(actions[1], 2U, false,
                              MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert(ctx.runtime[0].active_mode == MOTOR_PROTOCOL_WRIST_MODE_POSITION);
}

static void test_timeout_boundaries() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "joint", 0.25, 0.0, 0.0};
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
      "joint", 0.25, 0.0, 0.0};
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
      "joint", 0.0, INFINITY, 0.0};

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
      "joint", 0.25, 0.0, 0.0};
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
  motor_control_command_t command = {"wrist", 0.1, 0.2, 0.3};
  assert(motor_control_accept_command(&ctx, &command, 0));
  size_t count = motor_control_step(&ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  motor_protocol_joint_command_t encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, 5.0, 0.0001));
  assert(nearly_equal(encoded.velocity, 10.0, 0.01));

  assert(motor_control_set_mode(&ctx, "wrist",
                                MOTOR_PROTOCOL_WRIST_MODE_POSITION));
  command.position = -0.2;
  command.velocity = 0.0;
  assert(motor_control_accept_command(&ctx, &command, 2));
  count = motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert_vaxor_control_action(actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, -10.0, 0.0001));

  assert(motor_control_set_mode(&ctx, "wrist",
                                MOTOR_PROTOCOL_WRIST_MODE_VELOCITY));
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
      {"bus_0_joint", 0.1, 0.0, 0.0},
      {"bus_1_joint", 0.2, 0.0, 0.0},
  };
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 2);
  assert(motor_control_accept_command(&ctx, &commands[0], 0));
  assert(motor_control_accept_command(&ctx, &commands[1], 0));
  size_t count = motor_control_step(&ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 6U);
  motor_control_set_bus_fault(&ctx, 0, true);
  count = motor_control_step(&ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == motor_protocol_wrist_control_id(2));
  assert(actions[0].data[0] == 0U);
  assert(actions[1].bus_index == 1U);
  assert(actions[1].can_id == motor_protocol_wrist_command_id(3));
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);

  motor_control_set_bus_fault(&ctx, 0, false);
  assert(motor_control_set_authorization(&ctx, true));
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  assert(motor_control_accept_command(&ctx, &commands[0], 2));
  count = motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 4U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == motor_protocol_wrist_control_id(2));
  assert(actions[0].data[0] == 0U);
}

static void test_hard_stop_invalidates_command_across_full_millis_wrap() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {"wrist", 0.1, 0.2, 0.3};
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
  const motor_control_command_t command = {"valid", 0.1, 0.0, 0.0};

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
  const motor_control_command_t original_command = {"stable_joint", 0.1, 0.0, 0.0};
  const motor_control_command_t mutated_command = {"mutated", 0.2, 0.0, 0.0};

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
  const motor_control_command_t command = {"wrist", 0.1, 0.2, 0.3};
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

static void test_status_feedback_is_available_by_motor_index() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  motor_control_init(&ctx, &motor, 1U);

  const motor_protocol_joint_state_t input{5.0, -10.0, 2.0};
  motor_control_rx_frame_t frame{};
  frame.bus_index = 0U;
  frame.can_id = motor_protocol_wrist_status_id(2U);
  frame.size = 20U;
  motor_protocol_pack_wrist_status(2U, &input, 48.6, 0x12345678U,
                                   4U, 5U, 2U, 7U, frame.data);
  assert(motor_control_accept_feedback(&ctx, &frame));

  motor_control_motor_status_t status{};
  assert(motor_control_get_motor_status(&ctx, 0U, &status));
  assert(status.sequence == 1U);
  assert(std::strcmp(status.joint_name, "wrist") == 0);
  assert(nearly_equal(status.voltage_v, 48.6, 0.051));
  assert(status.fault == 0x12345678U);
  assert(status.motor_state == 4U);
  assert(status.calibration_state == 5U);
  assert(status.mode == 2U);
  assert(status.motor_type == 7U);

  assert(motor_control_accept_feedback(&ctx, &frame));
  assert(motor_control_get_motor_status(&ctx, 0U, &status));
  assert(status.sequence == 2U);
}

static void test_short_feedback_does_not_create_motor_status() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  motor_control_init(&ctx, &motor, 1U);

  const motor_protocol_joint_state_t input{5.0, -10.0, 2.0};
  motor_control_rx_frame_t frame{};
  frame.bus_index = 0U;
  frame.can_id = motor_protocol_wrist_reply_id(2U);
  frame.size = 12U;
  motor_protocol_pack_wrist_reply(2U, &input, frame.data);
  assert(motor_control_accept_feedback(&ctx, &frame));

  motor_control_motor_status_t status{};
  assert(!motor_control_get_motor_status(&ctx, 0U, &status));
}

int main() {
  test_creb_waits_100_ms_after_enable_then_runs_and_times_out();
  test_creb_sync_is_once_per_configured_bus();
  test_creb_feedback_converts_motor_speed_and_current();
  test_creb_command_uses_mit_wire_mode();
  test_gains_apply_only_to_subsequent_commands();
  test_mode_change_stops_vaxor_and_requires_new_command();
  test_creb_rejects_non_mit_mode_change();
  test_disabled_vaxor_changes_mode_before_any_command();
  test_timeout_boundaries();
  test_invalid_command_does_not_refresh_freshness();
  test_infinity_is_rejected();
  test_agent_disconnect_requires_authorization_and_new_command();
  test_vaxor_enable_mode_switch_and_gear_ratio_conversion();
  test_bus_fault_and_manual_disable_are_bus_level();
  test_hard_stop_invalidates_command_across_full_millis_wrap();
  test_invalid_config_names_are_skipped();
  test_config_name_is_deep_copied();
  test_fixed_capacities_are_enforced();
  test_feedback_is_decoded_by_motor_control();
  test_status_feedback_is_available_by_motor_index();
  test_short_feedback_does_not_create_motor_status();
  return 0;
}
