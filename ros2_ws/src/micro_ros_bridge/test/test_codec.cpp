// 与固件 lib/wire_codec 同一组 golden 向量（micro_ros_firmware/test/host/test_wire_codec.cpp）。
// 两边取值必须一致：不一致意味着固件与主机对同一个物理量的解释不同。
#include <gtest/gtest.h>

#include <cmath>

#include "micro_ros_bridge/codec.hpp"

using micro_ros_bridge::contract::kScalePosition;
using micro_ros_bridge::contract::kScaleVelocity;
using micro_ros_bridge::contract::kSentinelMissing;

TEST(Codec, GoldenVectors) {
  EXPECT_EQ(micro_ros_bridge::quantize(0.0, kScalePosition).q, 0);
  EXPECT_EQ(micro_ros_bridge::quantize(kScalePosition, kScalePosition).q, 32767);
  EXPECT_EQ(micro_ros_bridge::quantize(-kScalePosition, kScalePosition).q, -32767);
  EXPECT_EQ(micro_ros_bridge::quantize(0.00625, kScalePosition).q, 16);
  EXPECT_EQ(micro_ros_bridge::quantize(64.0, kScaleVelocity).q, 32767);
}

TEST(Codec, SaturatesInsteadOfRejecting) {
  const auto hi = micro_ros_bridge::quantize(1000.0, kScalePosition);
  EXPECT_EQ(hi.q, 32767);
  EXPECT_TRUE(hi.saturated);
  const auto lo = micro_ros_bridge::quantize(-1000.0, kScalePosition);
  EXPECT_EQ(lo.q, -32767);
  EXPECT_TRUE(lo.saturated);
  EXPECT_FALSE(micro_ros_bridge::quantize(kScalePosition, kScalePosition).saturated);
}

TEST(Codec, RoundTripWithinHalfStep) {
  const double step = kScalePosition / 32767.0;
  for (int i = -1000; i <= 1000; ++i) {
    const double x = i * step * 3.7;
    const double back = micro_ros_bridge::dequantize(
      micro_ros_bridge::quantize(x, kScalePosition).q, kScalePosition);
    EXPECT_LE(std::fabs(back - x), step / 2.0 + 1e-12);
  }
}

TEST(Codec, SentinelIsNotAValue) {
  EXPECT_TRUE(std::isnan(micro_ros_bridge::dequantize(kSentinelMissing, kScalePosition)));
  EXPECT_EQ(micro_ros_bridge::quantize(-kScalePosition, kScalePosition).q, -32767);
}

TEST(Codec, NonFiniteYieldsSentinel) {
  EXPECT_EQ(micro_ros_bridge::quantize(NAN, kScalePosition).q, kSentinelMissing);
  EXPECT_EQ(micro_ros_bridge::quantize(INFINITY, kScalePosition).q, kSentinelMissing);
}
