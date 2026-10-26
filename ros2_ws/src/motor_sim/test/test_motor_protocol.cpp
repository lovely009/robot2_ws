#include "motor_sim/protocol/motor_protocol.hpp"
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
  EXPECT_NE(protocol::make_motor_protocol("vaxor"), nullptr);
  EXPECT_THROW(protocol::make_motor_protocol("unknown"),
               std::invalid_argument);
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
  // 手册 3.5：Data19 是电机类型枚举（0x01=30mm），与使能状态无关。
  EXPECT_EQ(status.data[19], 0x01U);
  // 手册 3.5：母线电压 = 码值 / 10 V，无 0x8000 偏移；120 -> 12.0 V。
  EXPECT_EQ(status.data[10], 120U);
  EXPECT_EQ(status.data[11], 0U);
}

TEST(MotorProtocols, RejectInvalidIdsAndIgnoreUnrelatedFrames) {
  protocol::VaxorProtocol vaxor;
  EXPECT_THROW(vaxor.validate_motor_id(61U), std::out_of_range);
  EXPECT_FALSE(vaxor.decode_command(frame_from_bytes(0x123U, {}), 1U));
}
