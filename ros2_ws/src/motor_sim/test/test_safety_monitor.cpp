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
  // 单次冲突不锁存（可能是同一控制器的合法切换）；连续冲突才是双控制器。
  EXPECT_EQ(safety.fault(), core::Fault::None);
  safety.observe_valid_command(0U, "legs", 3U, {0x03U}, at_us(3000));
  EXPECT_EQ(safety.fault(), core::Fault::DuplicateController);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitor, SingleConflictingPayloadDoesNotLatch) {
  // 驱动器激活序列：零增益轮询帧后 0.1 ms 内发使能帧，payload 变化但只有一个
  // 控制器，不得锁存（canfd_system.cpp on_activate 的合法节奏）。
  core::SafetyMonitor safety(30U, 150ms, 2ms);
  safety.observe_valid_command(0U, "legs", 3U, {0x0CU}, at_us(0));
  safety.observe_valid_command(0U, "legs", 3U, {0xCCU}, at_us(100));
  EXPECT_EQ(safety.fault(), core::Fault::None);
  EXPECT_FALSE(safety.latched());
  // 窗口内的相同重发复位冲突计数，之后再来单次冲突也不锁存。
  safety.observe_valid_command(0U, "legs", 3U, {0xCCU}, at_us(1200));
  safety.observe_valid_command(0U, "legs", 3U, {0x0CU}, at_us(2300));
  EXPECT_FALSE(safety.latched());
  // 超出冲突窗口到达的命令同样复位冲突计数。
  safety.observe_valid_command(0U, "legs", 3U, {0xCCU}, at_us(100000));
  safety.observe_valid_command(0U, "legs", 3U, {0x0CU}, at_us(115000));
  EXPECT_FALSE(safety.latched());
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

TEST(SafetyMonitorReset, RejectsResetUntilEveryAxisWasExplicitlyDisabled) {
  core::SafetyMonitor safety(2U, 150ms, 2ms);
  safety.set_enabled(0U, true, at_ms(0));
  safety.set_enabled(1U, true, at_ms(0));
  safety.latch(core::Fault::CommandLimit);

  EXPECT_EQ(safety.reset_fault(), core::ResetFaultResult::AxesStillEnabled);
  safety.set_enabled(0U, false, at_ms(1));
  safety.set_enabled(1U, false, at_ms(1));
  EXPECT_EQ(safety.reset_fault(), core::ResetFaultResult::Cleared);
  EXPECT_FALSE(safety.latched());
  EXPECT_TRUE(safety.ready_for_new_enable_cycle());
}

TEST(SafetyMonitorReset, DisconnectFaultCannotBeReset) {
  core::SafetyMonitor safety(2U, 150ms, 2ms);
  safety.observe_disconnect("arm");
  safety.set_enabled(0U, false, at_ms(1));
  safety.set_enabled(1U, false, at_ms(1));

  EXPECT_EQ(safety.reset_fault(),
            core::ResetFaultResult::TransportDisconnected);
  EXPECT_TRUE(safety.latched());
}

TEST(SafetyMonitorReset, ClearedFaultDoesNotReuseOldCommandTimestamp) {
  core::SafetyMonitor safety(1U, 150ms, 2ms);
  safety.set_enabled(0U, true, at_ms(0));
  safety.observe_valid_command(0U, "arm", 1U, {0x01U}, at_ms(0));
  EXPECT_TRUE(safety.tick(at_ms(151)).latched);
  safety.set_enabled(0U, false, at_ms(152));
  ASSERT_EQ(safety.reset_fault(), core::ResetFaultResult::Cleared);

  safety.set_enabled(0U, true, at_ms(200));
  const auto state = safety.tick(at_ms(201));
  EXPECT_FALSE(state.motor_status[0].command_age.has_value());
  EXPECT_FALSE(state.motor_status[0].timed_out);
}

TEST(SafetyMonitorDebugMode, TimeoutRequiresAllAxisDisableBeforeRearm) {
  core::SafetyMonitor safety(2U, 150ms, 2ms, false);
  safety.set_enabled(0U, true, at_ms(0));
  safety.set_enabled(1U, true, at_ms(0));
  safety.observe_valid_command(0U, "arm", 1U, {1U}, at_ms(0));
  safety.observe_valid_command(1U, "arm", 2U, {2U}, at_ms(0));

  const auto fault = safety.tick(at_ms(151));
  EXPECT_FALSE(safety.latched());
  EXPECT_EQ(fault.fault, core::Fault::CommandTimeout);
  EXPECT_TRUE(fault.rearm_required);
  EXPECT_EQ(fault.efforts, std::vector<double>(2U, 0.0));

  safety.set_enabled(0U, false, at_ms(152));
  EXPECT_FALSE(safety.ready_for_new_enable_cycle());
  safety.set_enabled(1U, false, at_ms(152));
  EXPECT_TRUE(safety.ready_for_new_enable_cycle());
  EXPECT_FALSE(safety.snapshot().rearm_required);
}

TEST(SafetyMonitorDebugMode, DuplicateControllerBrakesUntilExplicitDisable) {
  core::SafetyMonitor safety(1U, 150ms, 2ms, false);
  safety.set_enabled(0U, true, at_us(0));
  safety.observe_valid_command(0U, "arm", 3U, {0x01U}, at_us(0));
  safety.observe_valid_command(0U, "arm", 3U, {0x02U}, at_us(1500));
  safety.observe_valid_command(0U, "arm", 3U, {0x03U}, at_us(3000));

  EXPECT_FALSE(safety.latched());
  EXPECT_EQ(safety.snapshot().fault, core::Fault::DuplicateController);
  EXPECT_TRUE(safety.rearm_required());
  safety.set_enabled(0U, false, at_us(1600));
  EXPECT_TRUE(safety.ready_for_new_enable_cycle());
}
