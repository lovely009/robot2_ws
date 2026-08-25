#include "humanoid_gazebo/motor_sim/safety_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace motor_sim = humanoid_gazebo::motor_sim;
using namespace std::chrono_literals;

namespace {

using TimePoint = motor_sim::SafetyMonitor::TimePoint;

TimePoint at_ms(const int64_t milliseconds) {
  return TimePoint{} + std::chrono::milliseconds(milliseconds);
}

TimePoint at_us(const int64_t microseconds) {
  return TimePoint{} + std::chrono::microseconds(microseconds);
}

}  // namespace

TEST(SafetyMonitor, TimeoutZerosEveryAxisAndLatches) {
  motor_sim::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command("left_arm", 1U, {0x01U}, at_ms(0));
  const auto state = safety.tick(at_ms(151));
  EXPECT_TRUE(state.latched);
  EXPECT_EQ(state.fault, motor_sim::Fault::CommandTimeout);
  EXPECT_EQ(state.efforts, std::vector<double>(30U, 0.0));
}

TEST(SafetyMonitor, DifferentDuplicatePayloadWithinWindowLatches) {
  motor_sim::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command("legs", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command("legs", 3U, {0x02U}, at_us(1500));
  EXPECT_EQ(safety.fault(), motor_sim::Fault::DuplicateController);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitor, IdenticalRetransmissionDoesNotLatch) {
  motor_sim::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command("legs", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command("legs", 3U, {0x01U}, at_us(1500));
  EXPECT_EQ(safety.fault(), motor_sim::Fault::None);
  EXPECT_FALSE(safety.latched());
}

TEST(SafetyMonitor, ReconnectDoesNotClearDisconnectFault) {
  motor_sim::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_disconnect("right_arm");
  safety.observe_reconnect("right_arm");
  EXPECT_EQ(safety.fault(), motor_sim::Fault::TransportDisconnected);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitor, LimitFaultIsFirstFaultAndCannotBeCleared) {
  motor_sim::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.latch(motor_sim::Fault::CommandLimit);
  safety.observe_disconnect("left_arm");
  EXPECT_EQ(safety.fault(), motor_sim::Fault::CommandLimit);
}
