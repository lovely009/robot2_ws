#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>

#include "motor_protocol.h"

namespace {

constexpr double kVaxorPositionTolerance = 0.00001;
constexpr double kVaxorVelocityTolerance = 0.011;
constexpr double kVaxorKpTolerance = 0.00001;
constexpr double kVaxorKdTolerance = 0.01;
constexpr double kVaxorCurrentTolerance = 0.003;

void test_vaxor_12_byte_command_round_trip() {
  const motor_protocol_joint_command_t input{1.5, -2.0, 0.12345, 0.67, -1.1};
  uint8_t bytes[12]{};
  motor_protocol_pack_vaxor_command(&input, bytes);

  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_vaxor_command(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position - input.position) < kVaxorPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kVaxorVelocityTolerance);
  assert(std::abs(output.kp - input.kp) < kVaxorKpTolerance);
  assert(std::abs(output.kd - input.kd) < kVaxorKdTolerance);
  assert(std::abs(output.feedforward - input.feedforward) < kVaxorCurrentTolerance);
}

void test_vaxor_status_and_feedback_round_trip() {
  const motor_protocol_joint_state_t input{5.0, -10.0, 2.0};
  uint8_t reply[12]{};
  motor_protocol_pack_vaxor_reply(2U, &input, reply);

  motor_protocol_joint_state_t output{};
  assert(motor_protocol_unpack_vaxor_feedback(reply, sizeof(reply), &output));
  assert(std::abs(output.position - input.position) < kVaxorPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kVaxorVelocityTolerance);
  assert(std::abs(output.effort - input.effort) < kVaxorCurrentTolerance);
}

void test_vaxor_status_decodes_all_status_fields() {
  const uint8_t bytes[20]{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                          0xe6U, 0x01U, 0x78U, 0x56U, 0x34U, 0x12U,
                          4U, 5U, 2U, 7U};

  motor_protocol_vaxor_status_t status{};
  assert(motor_protocol_unpack_vaxor_status(bytes, sizeof(bytes), &status));
  assert(std::abs(status.voltage_v - 48.6) < 0.051);
  assert(status.fault == 0x12345678U);
  assert(status.motor_state == 4U);
  assert(status.calibration_state == 5U);
  assert(status.mode == 2U);
  assert(status.motor_type == 7U);
}

void test_vaxor_status_encodes_voltage_as_unbiased_signed_tenths() {
  const motor_protocol_joint_state_t input{};
  uint8_t bytes[20]{};
  motor_protocol_pack_vaxor_status(2U, &input, 48.6, 0U,
                                   0U, 0U, 0U, 0U, bytes);
  assert(bytes[10] == 0xe6U);
  assert(bytes[11] == 0x01U);
}

void test_vaxor_status_rejects_short_feedback() {
  uint8_t bytes[12]{};
  motor_protocol_vaxor_status_t status{};
  assert(!motor_protocol_unpack_vaxor_status(bytes, sizeof(bytes), &status));
}

void test_unpack_rejects_invalid_lengths() {
  uint8_t bytes[20]{};
  motor_protocol_joint_command_t command{};
  motor_protocol_joint_state_t state{};
  motor_protocol_vaxor_control_t control{};

  assert(!motor_protocol_unpack_vaxor_command(bytes, 11U, &command));
  assert(!motor_protocol_unpack_vaxor_control(bytes, 7U, &control));
  assert(!motor_protocol_unpack_vaxor_feedback(bytes, 11U, &state));
  assert(!motor_protocol_unpack_vaxor_feedback(bytes, 13U, &state));
}

void test_cerb_mit_frame_matches_new_protocol_layout() {
  const motor_protocol_joint_command_t command{12.5, -30.0, 500.0, 5.0,
                                                -10.0};
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));

  const uint8_t expected[12]{0xcc, 0xff, 0xff, 0x00, 0x0f, 0xff,
                             0xff, 0xf0, 0x00, 0x00, 0x00, 0x00};
  for (size_t index = 0; index < sizeof(expected); ++index) {
    assert(bytes[index] == expected[index]);
  }
}

void test_cerb_disabled_neutral_frame_is_safe() {
  const motor_protocol_joint_command_t command{};
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, false, bytes));

  const uint8_t expected[12]{0x0c, 0x7f, 0xff, 0x7f, 0xf0, 0x00,
                             0x00, 0x07, 0xff, 0x00, 0x00, 0x00};
  for (size_t index = 0; index < sizeof(expected); ++index) {
    assert(bytes[index] == expected[index]);
  }
}

void test_cerb_feedback_decodes_big_endian_units() {
  const uint8_t bytes[12]{0x07, 0x1c, 0xff, 0xc4, 0x05, 0xdc,
                          0x00, 0x00, 0x00, 0xf0, 0x06, 0xc0};
  motor_protocol_joint_state_t state{};
  assert(motor_protocol_unpack_cerb_feedback(bytes, sizeof(bytes), &state));
  assert(std::abs(state.position - 10.0 * MOTOR_PROTOCOL_PI / 180.0) < 1e-12);
  assert(std::abs(state.velocity + 2.0 * MOTOR_PROTOCOL_PI) < 1e-12);
  assert(state.effort == 1.5);
}

void test_cerb_rejects_invalid_or_faulted_data() {
  motor_protocol_joint_command_t command{};
  command.position = NAN;
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0};
  uint8_t bytes[12]{};
  assert(!motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));

  const motor_protocol_joint_command_t finite_command{};
  const motor_protocol_cerb_limits_t invalid_limits{0.0, 30.0, 10.0};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &invalid_limits, true, bytes));
  const motor_protocol_cerb_limits_t overflowing_limits{DBL_MAX, 30.0, 10.0};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &overflowing_limits, true, bytes));

  bytes[11] = 0x20;
  motor_protocol_joint_state_t state{};
  assert(!motor_protocol_unpack_cerb_feedback(bytes, sizeof(bytes), &state));
  assert(!motor_protocol_unpack_cerb_feedback(bytes, 11U, &state));
}

void test_cerb_large_finite_limit_quantizes_without_overflow() {
  const motor_protocol_cerb_limits_t limits{DBL_MAX / 4.0, 30.0, 10.0};
  const motor_protocol_joint_command_t command{limits.position_rad, 0.0, 0.0,
                                                0.0, 0.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, false, bytes));
  assert(bytes[1] == 0xff);
  assert(bytes[2] == 0xff);
}

void test_cerb_node_ids_cover_declared_range() {
  assert(!motor_protocol_cerb_node_valid(0U));
  assert(motor_protocol_cerb_node_valid(1U));
  assert(motor_protocol_cerb_node_valid(127U));
  assert(!motor_protocol_cerb_node_valid(128U));
  uint32_t id = 0U;
  assert(motor_protocol_cerb_command_id(1U, &id));
  assert(id == 0x111U);
  assert(motor_protocol_cerb_command_id(127U, &id));
  assert(id == 0x18fU);
  assert(!motor_protocol_cerb_command_id(0U, &id));
  assert(!motor_protocol_cerb_command_id(128U, &id));
  assert(!motor_protocol_cerb_command_id(1U, nullptr));
  assert(motor_protocol_cerb_feedback_id(1U, &id));
  assert(id == 0x301U);
}

}  // namespace

int main() {
  test_vaxor_12_byte_command_round_trip();
  test_vaxor_status_and_feedback_round_trip();
  test_vaxor_status_decodes_all_status_fields();
  test_vaxor_status_encodes_voltage_as_unbiased_signed_tenths();
  test_vaxor_status_rejects_short_feedback();
  test_unpack_rejects_invalid_lengths();
  test_cerb_mit_frame_matches_new_protocol_layout();
  test_cerb_disabled_neutral_frame_is_safe();
  test_cerb_feedback_decodes_big_endian_units();
  test_cerb_rejects_invalid_or_faulted_data();
  test_cerb_large_finite_limit_quantizes_without_overflow();
  test_cerb_node_ids_cover_declared_range();
  return 0;
}
