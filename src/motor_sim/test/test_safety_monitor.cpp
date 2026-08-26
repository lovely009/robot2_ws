#include "motor_sim/core/safety_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace core = motor_sim::core;
using namespace std::chrono_literals;

namespace {

using TimePoint = core::SafetyMonitor::TimePoint;

TimePoint at_ms(const int64_t milliseconds) {
  return TimePoint{} + std::chrono::milliseconds(milliseconds);
}

TimePoint at_us(const int64_t microseconds) {
  return TimePoint{} + std::chrono::microseconds(microseconds);
}

}  // namespace

TEST(SafetyMonitor, TimeoutZerosEveryAxisAndLatches) {
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.set_enabled(0U, true, at_ms(0));
  safety.observe_valid_command(0U, "left_arm", 1U, {0x01U}, at_ms(0));
  const auto state = safety.tick(at_ms(151));
  EXPECT_TRUE(state.latched);
  EXPECT_EQ(state.fault, core::Fault::CommandTimeout);
  EXPECT_EQ(state.efforts, std::vector<double>(30U, 0.0));
}

TEST(SafetyMonitor, DisabledMotorsMayRemainIdle) {
  core::SafetyMonitor safety(2U, 150ms, 2ms);
  EXPECT_FALSE(safety.tick(TimePoint{} + 10s).latched);
}

TEST(SafetyMonitor, OneStaleEnabledMotorLatchesWholeRobot) {
  core::SafetyMonitor safety(2U, 150ms, 2ms);
  const auto start = TimePoint{} + 1s;
  safety.set_enabled(0U, true, start);
  safety.set_enabled(1U, true, start);
  safety.observe_valid_command(0U, "arm", 1U, {1U}, start + 100ms);
  safety.observe_valid_command(1U, "arm", 2U, {2U}, start + 10ms);
  const auto snapshot = safety.tick(start + 170ms);
  EXPECT_TRUE(snapshot.latched);
  EXPECT_EQ(snapshot.fault, core::Fault::CommandTimeout);
  EXPECT_TRUE(snapshot.motor_status[1].timed_out);
}

TEST(SafetyMonitor, DifferentDuplicatePayloadWithinWindowLatches) {
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command(0U, "legs", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command(0U, "legs", 3U, {0x02U}, at_us(1500));
  EXPECT_EQ(safety.fault(), core::Fault::DuplicateController);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitor, IdenticalRetransmissionDoesNotLatch) {
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command(0U, "legs", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command(0U, "legs", 3U, {0x01U}, at_us(1500));
  EXPECT_EQ(safety.fault(), core::Fault::None);
  EXPECT_FALSE(safety.latched());
}

TEST(SafetyMonitor, DifferentPayloadOutsideConflictWindowDoesNotLatch) {
  core::SafetyMonitor safety(2U, 150ms, 2ms);
  safety.observe_valid_command(0U, "arm", 1U, {0x01U}, at_us(0));
  safety.observe_valid_command(0U, "arm", 1U, {0x02U}, at_us(3000));
  EXPECT_EQ(safety.fault(), core::Fault::None);
  EXPECT_FALSE(safety.latched());
}

TEST(SafetyMonitor, ReconnectDoesNotClearDisconnectFault) {
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_disconnect("right_arm");
  safety.observe_reconnect("right_arm");
  EXPECT_EQ(safety.fault(), core::Fault::TransportDisconnected);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitor, LimitFaultIsFirstFaultAndCannotBeCleared) {
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.latch(core::Fault::CommandLimit);
  safety.observe_disconnect("left_arm");
  EXPECT_EQ(safety.fault(), core::Fault::CommandLimit);
}

// 调试模式（latching_enabled=false）：检测逻辑保留，但任何故障都不锁存、
// 不清零输出，方便单步调试时反复注入命令。
TEST(SafetyMonitorDebugMode, TimeoutDoesNotLatchOrZeroEfforts) {
  core::SafetyMonitor safety(30U, 150ms, 2ms, false);
  safety.set_enabled(0U, true, at_ms(0));
  safety.observe_valid_command(0U, "arm", 1U, {0x01U}, at_ms(0));
  const auto state = safety.tick(at_ms(151));
  EXPECT_FALSE(state.latched);
  EXPECT_EQ(state.fault, core::Fault::None);
  // 快照不携带清零 efforts；状态里仍能看到超时标记用于诊断。
  EXPECT_TRUE(state.efforts.empty());
  EXPECT_TRUE(state.motor_status[0].timed_out);
}

TEST(SafetyMonitorDebugMode, DuplicateControllerDoesNotLatch) {
  core::SafetyMonitor safety(30U, 150ms, 2ms, false);
  safety.observe_valid_command(0U, "legs", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command(0U, "legs", 3U, {0x02U}, at_us(1500));
  EXPECT_EQ(safety.fault(), core::Fault::None);
  EXPECT_FALSE(safety.latched());
}

TEST(SafetyMonitorDebugMode, DisconnectAndExplicitLatchAreIgnored) {
  core::SafetyMonitor safety(30U, 150ms, 2ms, false);
  safety.observe_disconnect("arm");
  safety.observe_reconnect("arm");
  safety.latch(core::Fault::WrongBus);
  EXPECT_EQ(safety.fault(), core::Fault::None);
  EXPECT_FALSE(safety.latched());
}
