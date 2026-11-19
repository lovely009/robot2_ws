#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include "motor_hardware_interface/position_profile.hpp"

using motor_hardware_interface::PositionProfile;

TEST(PositionProfile, RetargetingPreservesPositionAndVelocityAndRespectsBounds) {
  PositionProfile profile(1.0, 2.0);
  profile.reset(0.0);
  ASSERT_TRUE(profile.target(0.8));
  double before_position = 0.0, before_velocity = 0.0;
  for (int i = 0; i < 300; ++i) {
    if (i == 8 || i == 11 || i == 19) {
      const double p = profile.position(), v = profile.velocity();
      ASSERT_TRUE(profile.target(i == 11 ? 0.9 : -0.4));
      EXPECT_DOUBLE_EQ(profile.position(), p);
      EXPECT_DOUBLE_EQ(profile.velocity(), v);
    }
    profile.advance(0.02);
    EXPECT_LE(std::abs(profile.velocity()), 1.0 + 1e-12);
    EXPECT_LE(std::abs(profile.velocity() - before_velocity), 0.04 + 1e-12);
    EXPECT_LE(std::abs(profile.position() - before_position), 0.02 + 1e-12);
    EXPECT_GE(profile.position(), -0.4 - 1e-12);
    EXPECT_LE(profile.position(), 0.9 + 1e-12);
    before_position = profile.position(); before_velocity = profile.velocity();
  }
  EXPECT_DOUBLE_EQ(profile.position(), -0.4);
  EXPECT_DOUBLE_EQ(profile.velocity(), 0.0);
}

TEST(PositionProfile, FinishesSmallTargetsAndResetForgetsPreviousMotion) {
  PositionProfile profile(3.0, 6.0);
  profile.reset(0.2);
  ASSERT_TRUE(profile.target(0.200001));
  profile.advance(0.02);
  EXPECT_DOUBLE_EQ(profile.position(), 0.200001);
  EXPECT_DOUBLE_EQ(profile.velocity(), 0.0);
  ASSERT_TRUE(profile.target(0.8));
  profile.advance(0.1);
  profile.reset(-0.1);
  profile.advance(1.0);
  EXPECT_DOUBLE_EQ(profile.position(), -0.1);
  EXPECT_DOUBLE_EQ(profile.velocity(), 0.0);
  EXPECT_FALSE(profile.target(NAN));
}

TEST(PositionProfile, IrregularRapidTargetsStayInsideMotionEnvelope) {
  std::mt19937 random(104);
  std::uniform_real_distribution<double> target(-1, 1), step(.001, .05);
  PositionProfile profile(3, 6);
  profile.reset(0);
  for (int i = 0; i < 20000; ++i) {
    if (i % 3 == 0) ASSERT_TRUE(profile.target(target(random)));
    const double before_p = profile.position(), before_v = profile.velocity(), dt = step(random);
    profile.advance(dt);
    ASSERT_LE(std::abs(profile.velocity()), 3 + 1e-11);
    ASSERT_LE(std::abs(profile.velocity() - before_v), 6 * dt + 1e-11);
    ASSERT_LE(std::abs(profile.position() - before_p), 3 * dt + 1e-11);
    ASSERT_LE(std::abs(profile.position()), 1 + 1e-11);
  }
}
