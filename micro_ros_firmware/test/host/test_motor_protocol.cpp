#include <cassert>
#include <cmath>
#include <cstdint>

#include "motor_protocol.h"

namespace {

constexpr double kWristPositionTolerance = 0.00001;
constexpr double kWristVelocityTolerance = 0.011;
constexpr double kWristKpTolerance = 0.00001;
constexpr double kWristKdTolerance = 0.01;
constexpr double kWristCurrentTolerance = 0.003;

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

void test_wrist_status_and_feedback_round_trip() {
  const motor_protocol_joint_state_t input{5.0, -10.0, 2.0};
  uint8_t reply[12]{};
  motor_protocol_pack_wrist_reply(2U, &input, reply);

  motor_protocol_joint_state_t output{};
  assert(motor_protocol_unpack_wrist_feedback(reply, sizeof(reply), &output));
  assert(std::abs(output.position - input.position) < kWristPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kWristVelocityTolerance);
  assert(std::abs(output.effort - input.effort) < kWristCurrentTolerance);
}

void test_unpack_rejects_invalid_lengths() {
  uint8_t bytes[20]{};
  motor_protocol_joint_command_t command{};
  motor_protocol_joint_state_t state{};
  motor_protocol_wrist_control_t control{};

  assert(!motor_protocol_unpack_wrist_command(bytes, 11U, &command));
  assert(!motor_protocol_unpack_wrist_control(bytes, 7U, &control));
  assert(!motor_protocol_unpack_wrist_feedback(bytes, 11U, &state));
  assert(!motor_protocol_unpack_wrist_feedback(bytes, 13U, &state));
}

}  // namespace

int main() {
  test_wrist_12_byte_command_round_trip();
  test_wrist_status_and_feedback_round_trip();
  test_unpack_rejects_invalid_lengths();
  return 0;
}
