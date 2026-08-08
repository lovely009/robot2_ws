#include "motor_canfd_drive/protocol.hpp"

#include <array>
#include <gtest/gtest.h>

namespace protocol = motor_canfd_drive::protocol;

TEST(RpProtocol, NeutralSlotMatchesManual) {
  const auto bytes = protocol::pack_rp_slot({});
  const std::array<uint8_t, 8> expected{
      {0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff}};
  EXPECT_EQ(bytes, expected);
}

TEST(RpProtocol, SlotRoundTripKeepsPhysicalValuesWithinQuantization) {
  protocol::JointCommand command;
  command.position = 1.25;
  command.velocity = -2.0;
  command.kp = 30.0;
  command.kd = 1.0;
  command.feedforward = 2.5;
  const auto bytes = protocol::pack_rp_slot(command);
  const auto decoded = protocol::unpack_rp_slot(bytes.data(), bytes.size());
  EXPECT_NEAR(decoded.position, command.position, 20.0 / 65535.0);
  EXPECT_NEAR(decoded.velocity, command.velocity, 120.0 / 4095.0);
  EXPECT_NEAR(decoded.kp, command.kp, 500.0 / 4095.0);
  EXPECT_NEAR(decoded.kd, command.kd, 10.0 / 4095.0);
  EXPECT_NEAR(decoded.feedforward, command.feedforward, 120.0 / 4095.0);
}

TEST(WristProtocol, CommandMatchesManualExample) {
  protocol::JointCommand command;
  command.position =
      65537.0 / protocol::kWristTicksPerRevolution * 2.0 * protocol::kPi;
  command.kp = 0.008;
  command.kd = 1.0;
  const auto bytes = protocol::pack_wrist_command(command);
  const std::array<uint8_t, 12> expected{
      {0x01, 0x00, 0x01, 0x80, 0x00, 0x80, 0x20, 0x03,
       0x64, 0x00, 0x00, 0x80}};
  EXPECT_EQ(bytes, expected);
}

TEST(WristProtocol, FeedbackKnownZero) {
  const std::array<uint8_t, 12> bytes{
      {0x05, 0x05, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80,
       0x00, 0x80, 0x00, 0x00}};
  const auto state = protocol::unpack_wrist_feedback(bytes.data(), bytes.size());
  EXPECT_DOUBLE_EQ(state.position, 0.0);
  EXPECT_DOUBLE_EQ(state.velocity, 0.0);
  EXPECT_DOUBLE_EQ(state.effort, 0.0);
}

TEST(WristProtocol, ReplyGeneratedBySimulatorCanBeReadByDriver) {
  protocol::JointState current_state;
  current_state.position = 0.5;
  current_state.velocity = -0.25;
  current_state.effort = 1.5; // Current in ampere on the wire-side helper.
  const auto bytes = protocol::pack_wrist_reply(1U, current_state);
  const auto decoded = protocol::unpack_wrist_feedback(bytes.data(), bytes.size());
  EXPECT_NEAR(decoded.position, current_state.position,
              2.0 * protocol::kPi / protocol::kWristTicksPerRevolution);
  EXPECT_NEAR(decoded.velocity, current_state.velocity,
              2.0 * protocol::kPi / 600.0);
  EXPECT_NEAR(decoded.effort, current_state.effort,
              protocol::kWristCurrentScaleA);
}
