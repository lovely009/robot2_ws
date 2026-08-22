#include "motor_control.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <initializer_list>

static bool nearly_equal(double actual, double expected, double tolerance) {
  return std::fabs(actual - expected) <= tolerance;
}

static void test_timeout_boundaries() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};
  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 0));
  motor_control_step(&ctx, 99, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
  motor_control_step(&ctx, 100, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_protocol_joint_command_t slot{};
  assert(motor_protocol_unpack_rp_slot(actions[0].data, 8, &slot));
  assert(nearly_equal(slot.kp, 0.0, 0.001));
  assert(nearly_equal(slot.kd, 1.0, 0.01));
  assert(nearly_equal(slot.velocity, 0.0, 0.05));
  assert(nearly_equal(slot.feedforward, 0.0, 0.05));
  motor_control_step(&ctx, 299, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_control_step(&ctx, 300, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(motor_protocol_unpack_rp_slot(actions[0].data, 8, &slot));
  assert(nearly_equal(slot.kp, 0.0, 0.001));
  assert(nearly_equal(slot.kd, 0.0, 0.001));
}

static void test_timeout_uses_wrapping_unsigned_elapsed_time() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, UINT32_MAX - 50U));
  motor_control_step(&ctx, 48U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
  motor_control_step(&ctx, 49U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_control_step(&ctx, 249U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
}

static void test_invalid_command_does_not_refresh_freshness() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
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
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.0, INFINITY, 0.0};

  motor_control_init(&ctx, &motor, 1);
  assert(!motor_control_accept_command(&ctx, &command, 0));
  assert(ctx.invalid_command_count == 1U);
}

static void test_invalid_names_are_counted() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  motor_control_command_t command = {
      "unknown", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.0, 0.0, 0.0};

  motor_control_init(&ctx, &motor, 1);
  assert(!motor_control_accept_command(&ctx, &command, 0));
  command.joint_name[0] = '\0';
  assert(!motor_control_accept_command(&ctx, &command, 0));
  std::memset(command.joint_name, 'x', sizeof(command.joint_name));
  assert(!motor_control_accept_command(&ctx, &command, 0));
  assert(ctx.invalid_command_count == 3U);
}

static void test_agent_disconnect_requires_authorization_and_new_command() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
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
  size_t count = motor_control_step(
      &ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].size == 8U);
  assert(actions[0].data[0] == 0U);
  assert(!actions[0].is_fd);
  assert(!motor_control_set_authorization(&ctx, true));
  assert(!motor_control_accept_command(&ctx, &command, 2));
  motor_control_set_agent_connected(&ctx, true);
  motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);

  assert(motor_control_set_authorization(&ctx, true));
  motor_control_step(&ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(motor_control_accept_command(&ctx, &command, 3));
  motor_control_step(&ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
}

static void test_rp_commands_are_aggregated_per_bus_and_faults_are_per_slot() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      {"joint_1", MOTOR_CONTROL_RP, 1, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {"joint_2", MOTOR_CONTROL_RP, 2, 0,
       1.0, 1.0, 0.0, 40.0, 2.0},
  };
  const motor_control_command_t commands[] = {
      {"joint_1", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.25, 1.0, 2.0},
      {"joint_2", MOTOR_PROTOCOL_WRIST_MODE_MIT, -0.5, -2.0, -3.0},
  };
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 2);
  assert(motor_control_accept_command(&ctx, &commands[0], 0));
  assert(motor_control_accept_command(&ctx, &commands[1], 0));
  size_t count = motor_control_step(
      &ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_COMMAND_ID);
  assert(actions[0].size == 64U);
  assert(actions[0].is_fd);
  assert(!actions[0].is_extended);

  motor_protocol_joint_command_t slot_1{};
  motor_protocol_joint_command_t slot_2{};
  assert(motor_protocol_unpack_rp_slot(actions[0].data, 8, &slot_1));
  assert(motor_protocol_unpack_rp_slot(actions[0].data + 8, 8, &slot_2));
  assert(nearly_equal(slot_1.position, 0.25, 0.001));
  assert(nearly_equal(slot_1.velocity, 1.0, 0.05));
  assert(nearly_equal(slot_1.kp, 30.0, 0.2));
  assert(nearly_equal(slot_2.position, -0.5, 0.001));
  assert(nearly_equal(slot_2.velocity, -2.0, 0.05));
  assert(nearly_equal(slot_2.kp, 40.0, 0.2));

  motor_control_set_motor_fault(&ctx, 0, true);
  count = motor_control_step(
      &ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  assert(motor_protocol_unpack_rp_slot(actions[0].data, 8, &slot_1));
  assert(motor_protocol_unpack_rp_slot(actions[0].data + 8, 8, &slot_2));
  assert(nearly_equal(slot_1.position, 0.0, 0.001));
  assert(nearly_equal(slot_1.velocity, 0.0, 0.05));
  assert(nearly_equal(slot_1.kp, 0.0, 0.001));
  assert(nearly_equal(slot_1.kd, 0.0, 0.001));
  assert(nearly_equal(slot_1.feedforward, 0.0, 0.05));
  assert(nearly_equal(slot_2.position, -0.5, 0.001));
  assert(nearly_equal(slot_2.velocity, -2.0, 0.05));

  motor_control_set_motor_fault(&ctx, 0, false);
  assert(motor_control_set_authorization(&ctx, true));
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  assert(motor_control_accept_command(&ctx, &commands[0], 2));
  count = motor_control_step(
      &ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
}

static void test_rp_hard_timeout_only_neutralizes_its_slot() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      {"old", MOTOR_CONTROL_RP, 1, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {"fresh", MOTOR_CONTROL_RP, 2, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
  };
  const motor_control_command_t old_command = {
      "old", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.3, 0.0, 0.0};
  const motor_control_command_t fresh_command = {
      "fresh", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.4, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 2);
  assert(motor_control_accept_command(&ctx, &old_command, 0));
  assert(motor_control_accept_command(&ctx, &fresh_command, 250));
  assert(motor_control_step(
             &ctx, 300, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  motor_protocol_joint_command_t old_slot{};
  motor_protocol_joint_command_t fresh_slot{};
  assert(motor_protocol_unpack_rp_slot(actions[0].data, 8, &old_slot));
  assert(motor_protocol_unpack_rp_slot(actions[0].data + 8, 8, &fresh_slot));
  assert(nearly_equal(old_slot.kp, 0.0, 0.001));
  assert(nearly_equal(old_slot.kd, 0.0, 0.001));
  assert(nearly_equal(fresh_slot.position, 0.4, 0.001));

  assert(motor_control_accept_command(&ctx, &old_command, 301));
  motor_control_step(&ctx, 301, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
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

static void test_vaxor_enable_mode_switch_and_gear_ratio_conversion() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 1,
      50.0, 0.65, 0.042, 0.008, 1.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  motor_control_command_t command = {
      "wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.2, 0.3};
  assert(motor_control_accept_command(&ctx, &command, 0));
  size_t count = motor_control_step(
      &ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(
      actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(
      actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  motor_protocol_joint_command_t encoded =
      unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, 5.0, 0.0001));
  assert(nearly_equal(encoded.velocity, 10.0, 0.01));

  count = motor_control_step(
      &ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 1U);
  encoded = unpack_vaxor_command_action(actions[0], 2);
  assert(nearly_equal(encoded.position, 5.0, 0.0001));

  command.mode = MOTOR_PROTOCOL_WRIST_MODE_POSITION;
  command.position = -0.2;
  command.velocity = 0.0;
  assert(motor_control_accept_command(&ctx, &command, 2));
  count = motor_control_step(
      &ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(
      actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  assert_vaxor_control_action(
      actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_POSITION);
  encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.position, -10.0, 0.0001));

  command.mode = MOTOR_PROTOCOL_WRIST_MODE_VELOCITY;
  command.position = 0.0;
  command.velocity = -0.3;
  assert(motor_control_accept_command(&ctx, &command, 3));
  count = motor_control_step(
      &ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(
      actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_VELOCITY);
  assert_vaxor_control_action(
      actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_VELOCITY);
  encoded = unpack_vaxor_command_action(actions[2], 2);
  assert(nearly_equal(encoded.velocity, -15.0, 0.01));
}

static void test_bus_fault_and_manual_disable_are_bus_level() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motors[] = {
      {"bus_0_joint", MOTOR_CONTROL_RP, 1, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {"bus_1_joint", MOTOR_CONTROL_RP, 1, 1,
       1.0, 1.0, 0.0, 30.0, 1.0},
  };
  const motor_control_command_t commands[] = {
      {"bus_0_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.1, 0.0, 0.0},
      {"bus_1_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT, 0.2, 0.0, 0.0},
  };
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, motors, 2);
  assert(motor_control_accept_command(&ctx, &commands[0], 0));
  assert(motor_control_accept_command(&ctx, &commands[1], 0));
  assert(motor_control_step(
             &ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 2U);

  motor_control_set_bus_fault(&ctx, 0, true);
  assert(!motor_control_set_authorization(&ctx, true));
  size_t count = motor_control_step(
      &ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 0U);
  assert(actions[1].bus_index == 1U);
  assert(actions[1].can_id == MOTOR_PROTOCOL_RP_COMMAND_ID);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_FAULT_LATCHED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);

  motor_control_set_bus_fault(&ctx, 0, false);
  assert(motor_control_set_authorization(&ctx, true));
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(ctx.runtime[1].state == MOTOR_CONTROL_ACTIVE);
  assert(motor_control_accept_command(&ctx, &commands[0], 2));
  assert(motor_control_step(
             &ctx, 2, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 1U);
  assert(actions[1].bus_index == 0U);
  assert(actions[1].can_id == MOTOR_PROTOCOL_RP_COMMAND_ID);
  assert(actions[2].bus_index == 1U);
  assert(actions[2].can_id == MOTOR_PROTOCOL_RP_COMMAND_ID);

  assert(motor_control_set_authorization(&ctx, false));
  count = motor_control_step(
      &ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert(actions[0].bus_index == 0U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[1].bus_index == 1U);
  assert(actions[1].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
}

static void test_vaxor_invalid_mode_is_rejected() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", 3U, 0.0, 0.0, 0.0};

  motor_control_init(&ctx, &motor, 1);
  assert(!motor_control_accept_command(&ctx, &command, 0));
  assert(ctx.invalid_command_count == 1U);
  assert(!ctx.runtime[0].command_valid);
}

static void test_vaxor_disconnect_stop_survives_recovery_before_step() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  motor_control_set_agent_connected(&ctx, true);
  assert(motor_control_accept_command(&ctx, &command, 0));
  assert(motor_control_step(
             &ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);
  assert(ctx.runtime[0].vaxor_enabled);

  motor_control_set_agent_connected(&ctx, false);
  motor_control_set_agent_connected(&ctx, true);
  assert(motor_control_set_authorization(&ctx, true));
  assert(motor_control_accept_command(&ctx, &command, 1));

  size_t count = motor_control_step(
      &ctx, 1, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 3U);
  assert_vaxor_control_action(
      actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  assert_vaxor_control_action(
      actions[1], 2, true, MOTOR_PROTOCOL_WRIST_MODE_MIT);
  (void)unpack_vaxor_command_action(actions[2], 2);
}

static void test_hard_stop_invalidates_command_across_full_millis_wrap() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 100U));
  assert(motor_control_step(
             &ctx, 100U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 3U);

  assert(motor_control_step(
             &ctx, 400U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
  assert(!ctx.runtime[0].command_valid);
  assert_vaxor_control_action(
      actions[0], 2, false, MOTOR_PROTOCOL_WRIST_MODE_MIT);

  size_t count = motor_control_step(
      &ctx, 100U, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 0U);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
}

static void test_rp_disable_repeats_until_atomic_enable_and_command_recovery() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};

  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 0));
  assert(motor_control_step(
             &ctx, 0, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);

  motor_control_set_bus_fault(&ctx, 0, true);
  for (uint32_t now_ms : {1U, 2U}) {
    assert(motor_control_step(
               &ctx, now_ms, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
    assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
    assert(actions[0].data[0] == 0U);
  }

  motor_control_set_bus_fault(&ctx, 0, false);
  assert(motor_control_step(
             &ctx, 3, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 0U);

  assert(motor_control_set_authorization(&ctx, true));
  assert(motor_control_step(
             &ctx, 4, actions, MOTOR_CONTROL_MAX_TX_ACTIONS) == 1U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 0U);

  assert(motor_control_accept_command(&ctx, &command, 5));
  assert(motor_control_step(&ctx, 5, actions, 1) == 1U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 0U);

  size_t count = motor_control_step(
      &ctx, 5, actions, MOTOR_CONTROL_MAX_TX_ACTIONS);
  assert(count == 2U);
  assert(actions[0].can_id == MOTOR_PROTOCOL_RP_ENABLE_ID);
  assert(actions[0].data[0] == 1U);
  assert(actions[1].can_id == MOTOR_PROTOCOL_RP_COMMAND_ID);
  motor_protocol_joint_command_t slot{};
  assert(motor_protocol_unpack_rp_slot(actions[1].data, 8, &slot));
  assert(nearly_equal(slot.position, 0.25, 0.001));
}

static void test_invalid_config_names_are_skipped() {
  motor_control_context_t ctx{};
  char unterminated_name[32];
  std::memset(unterminated_name, 'x', sizeof(unterminated_name));
  const motor_control_motor_config_t motors[] = {
      {nullptr, MOTOR_CONTROL_RP, 1, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {"", MOTOR_CONTROL_RP, 2, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {unterminated_name, MOTOR_CONTROL_RP, 3, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
      {"valid", MOTOR_CONTROL_RP, 4, 0,
       1.0, 1.0, 0.0, 30.0, 1.0},
  };
  const motor_control_command_t command = {
      "valid", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.0, 0.0};

  motor_control_init(&ctx, motors, 4);
  assert(ctx.motor_count == 1U);
  assert(motor_control_accept_command(&ctx, &command, 0));
}

static void test_config_name_is_deep_copied() {
  motor_control_context_t ctx{};
  char caller_name[32] = "stable_joint";
  const motor_control_motor_config_t motor = {
      caller_name, MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t original_command = {
      "stable_joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.0, 0.0};
  const motor_control_command_t mutated_command = {
      "mutated", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.2, 0.0, 0.0};

  motor_control_init(&ctx, &motor, 1);
  std::strcpy(caller_name, "mutated");
  assert(motor_control_accept_command(&ctx, &original_command, 0));
  assert(!motor_control_accept_command(&ctx, &mutated_command, 1));
}

static void test_fixed_capacities_are_enforced() {
  motor_control_context_t ctx{};
  motor_control_motor_config_t motors[MOTOR_CONTROL_MAX_MOTORS + 1U]{};
  for (size_t index = 0; index < MOTOR_CONTROL_MAX_MOTORS + 1U; ++index) {
    motors[index] = {"joint", MOTOR_CONTROL_RP,
                     static_cast<uint8_t>(index + 1U), 0,
                     1.0, 1.0, 0.0, 30.0, 1.0};
  }
  motor_control_tx_action_t sentinel{};
  sentinel.can_id = 0x123U;

  motor_control_init(&ctx, motors, MOTOR_CONTROL_MAX_MOTORS + 1U);
  assert(ctx.motor_count == MOTOR_CONTROL_MAX_MOTORS);
  assert(motor_control_step(&ctx, 0, &sentinel, 0) == 0U);
  assert(sentinel.can_id == 0x123U);

  motor_control_context_t vaxor_ctx{};
  const motor_control_motor_config_t vaxor = {
      "wrist", MOTOR_CONTROL_VAXOR, 2, 0,
      50.0, 0.65, 0.042, 0.008, 1.0};
  const motor_control_command_t command = {
      "wrist", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.1, 0.2, 0.3};
  motor_control_tx_action_t actions[3]{};
  motor_control_init(&vaxor_ctx, &vaxor, 1);
  assert(motor_control_accept_command(&vaxor_ctx, &command, 0));
  assert(motor_control_step(&vaxor_ctx, 0, actions, 2) == 0U);
  assert(!vaxor_ctx.runtime[0].vaxor_enabled);
  assert(motor_control_step(&vaxor_ctx, 0, actions, 3) == 3U);
  assert(vaxor_ctx.runtime[0].vaxor_enabled);
}

int main() {
  test_timeout_boundaries();
  test_timeout_uses_wrapping_unsigned_elapsed_time();
  test_invalid_command_does_not_refresh_freshness();
  test_infinity_is_rejected();
  test_invalid_names_are_counted();
  test_agent_disconnect_requires_authorization_and_new_command();
  test_rp_commands_are_aggregated_per_bus_and_faults_are_per_slot();
  test_rp_hard_timeout_only_neutralizes_its_slot();
  test_vaxor_enable_mode_switch_and_gear_ratio_conversion();
  test_bus_fault_and_manual_disable_are_bus_level();
  test_vaxor_invalid_mode_is_rejected();
  test_vaxor_disconnect_stop_survives_recovery_before_step();
  test_hard_stop_invalidates_command_across_full_millis_wrap();
  test_rp_disable_repeats_until_atomic_enable_and_command_recovery();
  test_invalid_config_names_are_skipped();
  test_config_name_is_deep_copied();
  test_fixed_capacities_are_enforced();
  return 0;
}
