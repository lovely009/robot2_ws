#include "motor_sim/core/drive_lifecycle.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace core = motor_sim::core;

TEST(DriveLifecycle, RequiresEveryAxisFreshCommandInCurrentEnableCycle) {
  core::DriveLifecycle lifecycle(2U);
  EXPECT_EQ(lifecycle.state(), core::DriveState::BrakeEngaged);

  lifecycle.set_enabled(0U, true);
  lifecycle.observe_fresh_command(0U);
  lifecycle.set_enabled(1U, true);
  EXPECT_EQ(lifecycle.state(), core::DriveState::AwaitingFirstCommand);

  lifecycle.observe_fresh_command(1U);
  EXPECT_EQ(lifecycle.state(), core::DriveState::ReadyForControl);

  lifecycle.set_control_gate(true);
  EXPECT_EQ(lifecycle.state(), core::DriveState::MotorControlActive);
}

TEST(DriveLifecycle, DisableInvalidatesEveryOldCommandAndEngagesBrake) {
  core::DriveLifecycle lifecycle(2U);
  for (std::size_t index = 0U; index < 2U; ++index) {
    lifecycle.set_enabled(index, true);
    lifecycle.observe_fresh_command(index);
  }
  lifecycle.set_control_gate(true);

  lifecycle.set_enabled(0U, false);
  EXPECT_EQ(lifecycle.state(), core::DriveState::BrakeEngaged);
  lifecycle.set_enabled(0U, true);
  EXPECT_EQ(lifecycle.state(), core::DriveState::AwaitingFirstCommand);
}

TEST(DriveLifecycle, GateCannotActivateAnUnreadyArm) {
  core::DriveLifecycle lifecycle(2U);
  lifecycle.set_enabled(0U, true);
  lifecycle.observe_fresh_command(0U);
  lifecycle.set_control_gate(true);

  EXPECT_EQ(lifecycle.state(), core::DriveState::AwaitingFirstCommand);
}

TEST(DriveLifecycle, FaultsClearGateAndRequireFreshCommands) {
  core::DriveLifecycle lifecycle(1U);
  lifecycle.set_enabled(0U, true);
  lifecycle.observe_fresh_command(0U);
  lifecycle.set_control_gate(true);

  lifecycle.enter_fault(false);
  EXPECT_EQ(lifecycle.state(), core::DriveState::BrakeEngaged);
  lifecycle.reset_to_brake();
  lifecycle.set_enabled(0U, false);
  lifecycle.set_enabled(0U, true);
  EXPECT_EQ(lifecycle.state(), core::DriveState::AwaitingFirstCommand);

  lifecycle.enter_fault(true);
  EXPECT_EQ(lifecycle.state(), core::DriveState::FaultLatched);
  lifecycle.reset_to_brake();
  EXPECT_EQ(lifecycle.state(), core::DriveState::AwaitingFirstCommand);
}

TEST(DriveLifecycle, ReportsStableWireStateNames) {
  EXPECT_EQ(core::drive_state_name(core::DriveState::BrakeEngaged),
            "brake_engaged");
  EXPECT_EQ(core::drive_state_name(core::DriveState::AwaitingFirstCommand),
            "awaiting_first_command");
  EXPECT_EQ(core::drive_state_name(core::DriveState::ReadyForControl),
            "ready_for_control");
  EXPECT_EQ(core::drive_state_name(core::DriveState::MotorControlActive),
            "motor_control_active");
  EXPECT_EQ(core::drive_state_name(core::DriveState::FaultLatched),
            "fault_latched");
}

TEST(DriveLifecycle, RejectsZeroAxesAndInvalidIndices) {
  EXPECT_THROW(core::DriveLifecycle(0U), std::invalid_argument);
  core::DriveLifecycle lifecycle(1U);
  EXPECT_THROW(lifecycle.set_enabled(1U, true), std::out_of_range);
  EXPECT_THROW(lifecycle.observe_fresh_command(1U), std::out_of_range);
}
