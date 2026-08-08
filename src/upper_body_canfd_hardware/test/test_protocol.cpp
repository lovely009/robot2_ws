#include "upper_body_canfd_hardware/protocol.hpp"
#include <gtest/gtest.h>
namespace p = upper_body_canfd_hardware::protocol;
TEST(RpProtocol, NeutralSlotMatchesManual) {
  const auto bytes = p::pack_rp_slot({});
  const std::array<uint8_t, 8> expected{
      {0x7f, 0xff, 0x7f, 0xf0, 0, 0, 0x07, 0xff}};
  EXPECT_EQ(bytes, expected);
}
TEST(WristProtocol, CommandMatchesManual) {
  p::JointCommand c;
  c.position = 65537.0 / p::kWristTicksPerRevolution * 2.0 * p::kPi;
  c.kp = 0.008;
  c.kd = 1.0;
  const auto bytes = p::pack_wrist_command(c);
  const std::array<uint8_t, 12> expected{
      {1, 0, 1, 0x80, 0, 0x80, 0x20, 3, 0x64, 0, 0, 0x80}};
  EXPECT_EQ(bytes, expected);
}
TEST(WristProtocol, FeedbackKnownZero) {
  const std::array<uint8_t, 12> bytes{
      {5, 5, 0, 0, 0, 0x80, 0, 0x80, 0, 0x80, 0, 0}};
  const auto s = p::unpack_wrist_feedback(bytes.data(), bytes.size());
  EXPECT_DOUBLE_EQ(s.position, 0.0);
  EXPECT_DOUBLE_EQ(s.velocity, 0.0);
  EXPECT_DOUBLE_EQ(s.effort, 0.0);
}
