#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "upper_body_canfd_hardware/rp_mit.hpp"

namespace upper_body_canfd_hardware::rp {
namespace {

constexpr double kPosTolerance = 5e-4;
constexpr double kFieldTolerance = 0.04;

TEST(TestRpMit, ManualNeutralExample)
{
  // The RP manual explicitly gives this slot for q=v=tau=kp=kd=0.
  const std::vector<uint8_t> expected{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  EXPECT_EQ(PackMitSlot(MitCommand{}), expected);
}

TEST(TestRpMit, BroadcastAndRoundTrip)
{
  const std::vector<MitCommand> commands{
    {0.4, -1.2, 25.0, 1.5, 2.0},
    {},
    {-0.7, 2.0, 40.0, 2.0, -3.0},
    {1.0, 0.0, 20.0, 1.0, 0.0},
  };
  const auto packed = PackBroadcast(commands);
  ASSERT_EQ(packed.size(), 64u);
  const auto decoded = UnpackBroadcast(packed);
  ASSERT_EQ(decoded.size(), 8u);
  EXPECT_NEAR(decoded[0].position, 0.4, kPosTolerance);
  EXPECT_NEAR(decoded[0].velocity, -1.2, kFieldTolerance);
  EXPECT_NEAR(decoded[0].torque_ff, 2.0, kFieldTolerance);
  EXPECT_NEAR(decoded[2].position, -0.7, kPosTolerance);
  // Slots 4..8 are neutral padding.
  for (int i = 4; i < 8; ++i) {
    EXPECT_NEAR(decoded[i].position, 0.0, kPosTolerance);
    EXPECT_NEAR(decoded[i].torque_ff, 0.0, kFieldTolerance);
  }
}

TEST(TestRpMit, StateRoundTrip)
{
  const auto payload = PackState(0.2, -0.8, 3.5, 42.0, 0x0001, 24.0, 33.0);
  ASSERT_EQ(payload.size(), 12u);
  const auto state = UnpackState(payload);
  EXPECT_NEAR(state.position, 0.2, kPosTolerance);
  EXPECT_NEAR(state.velocity, -0.8, kFieldTolerance);
  EXPECT_NEAR(state.torque, 3.5, kFieldTolerance);
  EXPECT_EQ(state.voltage_v, 24.0);
  EXPECT_EQ(state.mos_temperature_c, 42.0);
}

TEST(TestRpMit, EnableFrame)
{
  EXPECT_EQ(PackEnable(true), (std::vector<uint8_t>{1, 0, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(PackEnable(false), (std::vector<uint8_t>{0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(TestRpMit, UnpackInvalidDataThrows)
{
  EXPECT_THROW(UnpackMitSlot(std::vector<uint8_t>(7)), std::invalid_argument);
  EXPECT_THROW(UnpackBroadcast(std::vector<uint8_t>(8)), std::invalid_argument);
  EXPECT_THROW(UnpackState(std::vector<uint8_t>(11)), std::invalid_argument);
  EXPECT_THROW(PackBroadcast(std::vector<MitCommand>(9)), std::invalid_argument);
}

}  // namespace
}  // namespace upper_body_canfd_hardware::rp
