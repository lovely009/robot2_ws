#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "upper_body_canfd_hardware/wrist_2530.hpp"

namespace upper_body_canfd_hardware::wrist {
namespace {

TEST(TestWrist2530, ManualCommandExample)
{
  // Manual section 3.1 example: signed multi-turn position code = 65537.
  const double q =
    static_cast<double>(65537) / static_cast<double>(kTicksPerRevolution) * (2.0 * M_PI);
  const MitCommand command{q, 0.0, 0.008, 1.0, 0.0};
  const std::vector<uint8_t> expected{
    0x01, 0x00, 0x01, 0x80, 0x00, 0x80, 0x20, 0x03, 0x64, 0x00, 0x00, 0x80};
  EXPECT_EQ(PackMitCommand(command), expected);
}

TEST(TestWrist2530, CanIds)
{
  EXPECT_EQ(MitCommandId(1), 0x505u);
  EXPECT_EQ(MotorControlId(1), 0x506u);
  EXPECT_EQ(MitReplyId(3), 0x70Du);
  EXPECT_EQ(StatusId(3), 0x70Eu);
  EXPECT_THROW(MitCommandId(0), std::invalid_argument);
  EXPECT_THROW(MitReplyId(61), std::invalid_argument);
}

TEST(TestWrist2530, MitCommandRoundTrip)
{
  const MitCommand command{1.25, -0.5, 0.008, 1.0, 0.35};
  const auto packed = PackMitCommand(command);
  ASSERT_EQ(packed.size(), 12u);
  const auto decoded = UnpackMitCommand(packed);
  EXPECT_NEAR(decoded.position, 1.25, 5e-6);
  EXPECT_NEAR(decoded.velocity, -0.5, 0.01);
  EXPECT_NEAR(decoded.kp, 0.008, 1e-6);
  EXPECT_NEAR(decoded.kd, 1.0, 1e-6);
  EXPECT_NEAR(decoded.current_ff, 0.35, 0.003);
}

TEST(TestWrist2530, MitReplyRoundTrip)
{
  const auto packed = PackMitReply(2, 0.75, -0.25, 1.5);
  ASSERT_EQ(packed.size(), 12u);
  const auto reply = UnpackMitReply(packed);
  EXPECT_EQ(reply.command_can_id, MitCommandId(2));
  EXPECT_NEAR(reply.position, 0.75, 5e-6);
  EXPECT_NEAR(reply.velocity, -0.25, 0.01);
  EXPECT_NEAR(reply.current, 1.5, 0.003);
}

TEST(TestWrist2530, StatusRoundTrip)
{
  const auto packed = PackStatus(3, 0.3, 0.1, -0.2, 24.5, 0x8000, 4, 0, 0, 0x01);
  ASSERT_EQ(packed.size(), 20u);
  const auto status = UnpackStatus(packed);
  EXPECT_NEAR(status.position, 0.3, 5e-6);
  EXPECT_NEAR(status.velocity, 0.1, 0.01);
  EXPECT_NEAR(status.current, -0.2, 0.003);
  EXPECT_NEAR(status.voltage_v, 24.5, 0.05);
  EXPECT_EQ(status.fault, 0x8000u);
  EXPECT_EQ(status.motor_state, 4u);
  EXPECT_EQ(status.mode, 0u);
}

TEST(TestWrist2530, MotorControlAndGainConversions)
{
  EXPECT_EQ(PackMotorControl(true, false, kModeMit), (std::vector<uint8_t>{1, 0, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(PackMotorControl(false, true, kModeVelocity), (std::vector<uint8_t>{0, 1, 1, 0, 0, 0, 0, 0}));
  EXPECT_THROW(PackMotorControl(false, false, 7), std::invalid_argument);
  EXPECT_NEAR(KpToCurrentGain(0.008), 7.1749, 1e-3);
  EXPECT_NEAR(KdToCurrentGain(1.0), 0.01168, 1e-5);
}

}  // namespace
}  // namespace upper_body_canfd_hardware::wrist
