#include "motor_sim/protocol/motor_protocol.hpp"
#include "motor_sim/protocol/rp_protocol.hpp"
#include "motor_sim/protocol/vaxor_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace protocol = motor_sim::protocol;

namespace {

protocol::CanFrame frame_from_bytes(
    const uint32_t id, const std::initializer_list<uint8_t> bytes,
    const bool fd = false) {
  protocol::CanFrame frame;
  frame.id = id;
  frame.size = bytes.size();
  frame.fd = fd;
  frame.brs = fd;
  std::copy(bytes.begin(), bytes.end(), frame.data.begin());
  return frame;
}

}  // namespace

TEST(ProtocolFactory, CreatesOnlySupportedMotorFamilies) {
  EXPECT_NE(protocol::make_motor_protocol("rp"), nullptr);
  EXPECT_NE(protocol::make_motor_protocol("motor_2530"), nullptr);
  EXPECT_THROW(protocol::make_motor_protocol("unknown"),
               std::invalid_argument);
}

TEST(RpProtocol, DecodesPositionCommandFromConfiguredSlot) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.fd = true;
  frame.brs = true;
  frame.size = 64U;
  const std::array<uint8_t, 8> neutral{
      0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff};
  for (std::size_t slot = 0U; slot < 8U; ++slot) {
    std::copy(neutral.begin(), neutral.end(), frame.data.begin() + slot * 8U);
  }
  const std::array<uint8_t, 8> slot_three{
      0x8f, 0xff, 0x7b, 0xb0, 0xf5, 0x19, 0x98, 0x54};
  std::copy(slot_three.begin(), slot_three.end(), frame.data.begin() + 16U);

  const protocol::RpProtocol rp;
  const auto command = rp.decode_command(frame, 3U);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->motor_id, 3U);
  EXPECT_EQ(command->mode, protocol::CommandMode::Mit);
  EXPECT_NEAR(command->position, 1.25, 20.0 / 65535.0);
  EXPECT_NEAR(command->velocity, -2.0, 120.0 / 4095.0);
  EXPECT_NEAR(command->feedforward, 2.5, 120.0 / 4095.0);
}

TEST(RpProtocol, DecodesEnableAndEncodesTwelveByteFeedback) {
  const protocol::RpProtocol rp;
  const auto enabled = rp.decode_command(
      frame_from_bytes(protocol::kRpEnableId, {1, 0, 0, 0, 0, 0, 0, 0}),
      1U);
  ASSERT_TRUE(enabled.has_value());
  EXPECT_EQ(enabled->mode, protocol::CommandMode::Enable);

  const protocol::MotorFeedback input{1U, 0.5, -0.25, 1.5};
  const auto frame = rp.encode_feedback(input, protocol::FeedbackKind::Reply);
  EXPECT_EQ(frame.id, 1U);
  EXPECT_EQ(frame.size, 12U);
  EXPECT_TRUE(frame.fd);
  const auto decoded = protocol::unpack_rp_state(frame.data.data(), frame.size);
  EXPECT_NEAR(decoded.position, input.position, 20.0 / 65535.0);
  EXPECT_NEAR(decoded.velocity, input.velocity, 120.0 / 4095.0);
  EXPECT_NEAR(decoded.effort, input.effort, 120.0 / 4095.0);
}

TEST(VaxorProtocol, DecodesManualMitCommandAndControlFrame) {
  const protocol::VaxorProtocol vaxor;
  const auto command = vaxor.decode_command(
      frame_from_bytes(protocol::vaxor_command_id(1U),
                       {0x01, 0x00, 0x01, 0x80, 0x00, 0x80,
                        0x20, 0x03, 0x64, 0x00, 0x00, 0x80},
                       true),
      1U);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->mode, protocol::CommandMode::Mit);
  EXPECT_NEAR(command->position,
              65537.0 / protocol::kVaxorTicksPerRevolution *
                  2.0 * protocol::kPi,
              2.0 * protocol::kPi / protocol::kVaxorTicksPerRevolution);
  EXPECT_NEAR(command->kp, 0.008, 1e-12);
  EXPECT_NEAR(command->kd, 1.0, 1e-12);

  const auto control = vaxor.decode_command(
      frame_from_bytes(protocol::vaxor_control_id(1U),
                       {1, 0, 2, 0, 0, 0, 0, 0}),
      1U);
  ASSERT_TRUE(control.has_value());
  EXPECT_EQ(control->mode, protocol::CommandMode::Position);
  EXPECT_TRUE(control->enabled);
}

TEST(VaxorProtocol, EncodesReplyAndStatusAtMotorSide) {
  const protocol::VaxorProtocol vaxor;
  protocol::MotorFeedback input{2U, 0.5, -0.25, 1.5};
  input.enabled = false;
  input.mode = protocol::CommandMode::Velocity;
  const auto reply = vaxor.encode_feedback(input, protocol::FeedbackKind::Reply);
  EXPECT_EQ(reply.id, protocol::vaxor_reply_id(2U));
  EXPECT_EQ(reply.size, 12U);
  EXPECT_TRUE(reply.fd);
  const auto decoded = protocol::unpack_vaxor_feedback(
      reply.data.data(), reply.size);
  EXPECT_NEAR(decoded.position, input.position,
              2.0 * protocol::kPi / protocol::kVaxorTicksPerRevolution);
  EXPECT_NEAR(decoded.velocity, input.velocity, 2.0 * protocol::kPi / 600.0);
  EXPECT_NEAR(decoded.effort, input.effort, protocol::kVaxorCurrentScaleA);

  const auto status =
      vaxor.encode_feedback(input, protocol::FeedbackKind::Status);
  EXPECT_EQ(status.id, protocol::vaxor_status_id(2U));
  EXPECT_EQ(status.size, 20U);
  EXPECT_EQ(status.data[18],
            static_cast<uint8_t>(protocol::CommandMode::Velocity));
  EXPECT_EQ(status.data[19], 0U);
}

TEST(MotorProtocols, RejectInvalidIdsAndIgnoreUnrelatedFrames) {
  protocol::RpProtocol rp;
  protocol::VaxorProtocol vaxor;
  EXPECT_THROW(rp.validate_motor_id(0U), std::out_of_range);
  EXPECT_THROW(rp.validate_motor_id(9U), std::out_of_range);
  EXPECT_THROW(vaxor.validate_motor_id(61U), std::out_of_range);
  EXPECT_FALSE(rp.decode_command(frame_from_bytes(0x123U, {}), 1U));
  EXPECT_FALSE(vaxor.decode_command(frame_from_bytes(0x123U, {}), 1U));
}
