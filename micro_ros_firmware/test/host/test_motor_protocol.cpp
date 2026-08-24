#include <cassert>
#include <cmath>
#include <cstdint>

#include "motor_protocol.h"

namespace {

constexpr double kRpPositionTolerance = 0.01;
constexpr double kRpVelocityTolerance = 0.04;
constexpr double kRpGainTolerance = 0.13;
constexpr double kRpTorqueTolerance = 0.04;
constexpr double kWristPositionTolerance = 0.00001;
constexpr double kWristVelocityTolerance = 0.011;
constexpr double kWristKpTolerance = 0.00001;
constexpr double kWristKdTolerance = 0.01;
constexpr double kWristCurrentTolerance = 0.003;

void test_rp_zero_command_round_trip() {
  motor_protocol_joint_command_t input{};
  uint8_t bytes[8]{};
  motor_protocol_pack_rp_slot(&input, bytes);

  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_rp_slot(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position) < kRpPositionTolerance);
  assert(std::abs(output.velocity) < kRpVelocityTolerance);
  assert(std::abs(output.kp) < kRpGainTolerance);
  assert(std::abs(output.kd) < kRpGainTolerance / 50.0);
  assert(std::abs(output.feedforward) < kRpTorqueTolerance);
}

void test_rp_command_clamps_extreme_values() {
  const motor_protocol_joint_command_t input{
      100.0, -100.0, 1000.0, 100.0, -100.0};
  uint8_t bytes[8]{};
  motor_protocol_pack_rp_slot(&input, bytes);

  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_rp_slot(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position - 10.0) < kRpPositionTolerance);
  assert(std::abs(output.velocity + 60.0) < kRpVelocityTolerance);
  assert(std::abs(output.kp - 500.0) < kRpGainTolerance);
  assert(std::abs(output.kd - 10.0) < kRpGainTolerance / 50.0);
  assert(std::abs(output.feedforward + 60.0) < kRpTorqueTolerance);
}

void test_rp_state_round_trip() {
  const motor_protocol_joint_state_t input{1.5, -2.0, 3.0};
  uint8_t bytes[12]{};
  motor_protocol_pack_rp_state(&input, 25.0, 0U, 48.0, 25.0, bytes);

  motor_protocol_joint_state_t output{};
  assert(motor_protocol_unpack_rp_state(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position - input.position) < kRpPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kRpVelocityTolerance);
  assert(std::abs(output.effort - input.effort) < kRpTorqueTolerance);
}

void test_wrist_12_byte_command_round_trip() {
  const motor_protocol_joint_command_t input{1.5, -2.0, 0.12345, 0.67, -1.1};
  uint8_t bytes[12]{};
  motor_protocol_pack_wrist_command(&input, bytes);

  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_wrist_command(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position - input.position) < kWristPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kWristVelocityTolerance);
  assert(std::abs(output.kp - input.kp) < kWristKpTolerance);
  assert(std::abs(output.kd - input.kd) < kWristKdTolerance);
  assert(std::abs(output.feedforward - input.feedforward) < kWristCurrentTolerance);
}

void test_unpack_rejects_invalid_lengths() {
  uint8_t bytes[20]{};
  motor_protocol_joint_command_t command{};
  motor_protocol_joint_state_t state{};
  motor_protocol_wrist_control_t control{};

  assert(!motor_protocol_unpack_rp_slot(bytes, 7U, &command));
  assert(!motor_protocol_unpack_rp_state(bytes, 11U, &state));
  assert(!motor_protocol_unpack_wrist_command(bytes, 11U, &command));
  assert(!motor_protocol_unpack_wrist_control(bytes, 7U, &control));
  assert(!motor_protocol_unpack_wrist_feedback(bytes, 11U, &state));
  assert(!motor_protocol_unpack_wrist_feedback(bytes, 13U, &state));
}

}  // namespace

int main() {
  test_rp_zero_command_round_trip();
  test_rp_command_clamps_extreme_values();
  test_rp_state_round_trip();
  test_wrist_12_byte_command_round_trip();
  test_unpack_rejects_invalid_lengths();
  return 0;
}
