#include "motor_sim/core/frame_router.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <vector>

namespace core = motor_sim::core;
namespace protocol = motor_sim::protocol;

namespace {

using TimePoint = core::FrameRouter::TimePoint;
using namespace std::chrono_literals;

core::FrameRouter make_router(const bool fault_latching_enabled = true) {
  const core::TopologyContract contract{
      {"arm"},
      {{"joint_a", "arm", "rp", 1U},
       {"joint_b", "arm", "vaxor", 4U}}};
  const auto path =
      std::string(TEST_SOURCE_DIR) + "/test/fixtures/two_axis.yaml";
  return core::FrameRouter(core::load_system_config(path, contract),
                           fault_latching_enabled);
}

protocol::CanFrame rp_enable(const bool enabled) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpEnableId;
  frame.size = 8U;
  frame.data[0] = enabled ? 1U : 0U;
  return frame;
}

protocol::CanFrame rp_command(const double position) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.fd = true;
  frame.brs = true;
  frame.size = 64U;
  protocol::MotorCommand neutral;
  const auto neutral_slot = protocol::pack_rp_slot(neutral);
  for (std::size_t slot = 0U; slot < protocol::kRpBroadcastSlots; ++slot) {
    std::copy(neutral_slot.begin(), neutral_slot.end(),
              frame.data.begin() + slot * neutral_slot.size());
  }
  protocol::MotorCommand command;
  command.position = position;
  command.kp = 20.0;
  command.kd = 1.0;
  const auto payload = protocol::pack_rp_slot(command);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_control(const bool enabled) {
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_control_id(4U);
  frame.size = 8U;
  const auto payload = protocol::pack_vaxor_control(
      enabled, false, protocol::CommandMode::Mit);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_command(const double motor_position) {
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_command_id(4U);
  frame.fd = true;
  frame.brs = true;
  frame.size = 12U;
  protocol::MotorCommand command;
  command.position = motor_position;
  command.kp = 0.01;
  command.kd = 1.0;
  const auto payload = protocol::pack_vaxor_command(command);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

void prepare_both_axes(core::FrameRouter &router, const TimePoint start) {
  ASSERT_EQ(router.handle("arm", rp_enable(true), start).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", vaxor_control(true), start + 3ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", rp_command(0.5), start + 6ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", vaxor_command(40.0), start + 9ms).status,
            core::RouterStatus::Applied);
}

}  // namespace

TEST(FrameRouterDriveState, StaysZeroUntilEveryAxisIsFreshAndGateIsOpen) {
  auto router = make_router();
  const std::vector<core::JointSample> states(2U);
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);
  EXPECT_EQ(router.compute_efforts(states, 0.01, TimePoint{}),
            std::vector<double>(2U, 0.0));

  ASSERT_EQ(router.handle("arm", rp_enable(true), TimePoint{}).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", rp_command(0.5), TimePoint{} + 3ms).status,
            core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::AwaitingFirstCommand);

  ASSERT_EQ(
      router.handle("arm", vaxor_control(true), TimePoint{} + 6ms).status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_command(40.0), TimePoint{} + 9ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::ReadyForControl);
  EXPECT_EQ(router.compute_efforts(states, 0.01, TimePoint{} + 10ms),
            std::vector<double>(2U, 0.0));

  router.set_control_gate(true);
  EXPECT_EQ(router.drive_state(), core::DriveState::MotorControlActive);
  const auto efforts =
      router.compute_efforts(states, 0.01, TimePoint{} + 11ms);
  EXPECT_NE(efforts[0], 0.0);
  EXPECT_NE(efforts[1], 0.0);
}

TEST(FrameRouterDriveState, TimeoutImmediatelyBrakesEveryAxis) {
  auto router = make_router();
  prepare_both_axes(router, TimePoint{});
  router.set_control_gate(true);

  const std::vector<core::JointSample> states(2U);
  const auto efforts = router.compute_efforts(states, 0.01, TimePoint{} + 160ms);
  EXPECT_EQ(efforts, std::vector<double>(2U, 0.0));
  EXPECT_EQ(router.drive_state(), core::DriveState::FaultLatched);
}

TEST(FrameRouterDriveState, LockedFaultAcceptsDisableForResetButNotMotion) {
  auto router = make_router();
  prepare_both_axes(router, TimePoint{});
  const std::vector<core::JointSample> states(2U);
  static_cast<void>(router.compute_efforts(states, 0.01, TimePoint{} + 160ms));

  EXPECT_EQ(router.handle("arm", rp_command(0.2), TimePoint{} + 170ms).status,
            core::RouterStatus::SafetyFault);
  EXPECT_EQ(router.handle("arm", rp_enable(false), TimePoint{} + 173ms).status,
            core::RouterStatus::Applied);
  EXPECT_EQ(
      router.handle("arm", vaxor_control(false), TimePoint{} + 176ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.reset_fault(), core::ResetFaultResult::Cleared);
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);

  ASSERT_EQ(router.handle("arm", rp_enable(true), TimePoint{} + 180ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(true), TimePoint{} + 183ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::AwaitingFirstCommand);
}

TEST(FrameRouterDriveState, DebugFaultNeedsDisableAndFreshReenableCycle) {
  auto router = make_router(false);
  prepare_both_axes(router, TimePoint{});
  router.set_control_gate(true);
  const std::vector<core::JointSample> states(2U);
  EXPECT_EQ(router.compute_efforts(states, 0.01, TimePoint{} + 160ms),
            std::vector<double>(2U, 0.0));
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);

  ASSERT_EQ(router.handle("arm", rp_enable(false), TimePoint{} + 170ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(false), TimePoint{} + 173ms).status,
      core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", rp_enable(true), TimePoint{} + 176ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(true), TimePoint{} + 179ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::AwaitingFirstCommand);
}
