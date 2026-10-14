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
      {{"joint_a", "arm", "vaxor", 1U},
       {"joint_b", "arm", "vaxor", 4U}}};
  const auto path =
      std::string(TEST_SOURCE_DIR) + "/test/fixtures/two_axis.yaml";
  return core::FrameRouter(core::load_system_config(path, contract),
                           fault_latching_enabled);
}

protocol::CanFrame vaxor_control(const unsigned id, const bool enabled) {
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_control_id(id);
  frame.size = 8U;
  const auto payload = protocol::pack_vaxor_control(
      enabled, false, protocol::CommandMode::Mit);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_command(const unsigned id,
                                 const double motor_position) {
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_command_id(id);
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
  ASSERT_EQ(router.handle("arm", vaxor_control(1U, true), start).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", vaxor_control(4U, true), start + 3ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", vaxor_command(1U, 40.0), start + 6ms).status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("arm", vaxor_command(4U, 40.0), start + 9ms).status,
            core::RouterStatus::Applied);
}

}  // namespace

TEST(FrameRouterDriveState, StaysZeroUntilEveryAxisIsFreshAndGateIsOpen) {
  auto router = make_router();
  const std::vector<core::JointSample> states(2U);
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);
  EXPECT_EQ(router.compute_efforts(states, 0.01, TimePoint{}),
            std::vector<double>(2U, 0.0));

  ASSERT_EQ(
      router.handle("arm", vaxor_control(1U, true), TimePoint{}).status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_command(1U, 40.0), TimePoint{} + 3ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::AwaitingFirstCommand);

  ASSERT_EQ(
      router.handle("arm", vaxor_control(4U, true), TimePoint{} + 6ms).status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_command(4U, 40.0), TimePoint{} + 9ms).status,
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

  EXPECT_EQ(
      router.handle("arm", vaxor_command(1U, 16.0), TimePoint{} + 170ms).status,
      core::RouterStatus::SafetyFault);
  EXPECT_EQ(
      router.handle("arm", vaxor_control(1U, false), TimePoint{} + 173ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(
      router.handle("arm", vaxor_control(4U, false), TimePoint{} + 176ms).status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.reset_fault(), core::ResetFaultResult::Cleared);
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);

  ASSERT_EQ(
      router.handle("arm", vaxor_control(1U, true), TimePoint{} + 180ms).status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(4U, true), TimePoint{} + 183ms).status,
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

  ASSERT_EQ(
      router.handle("arm", vaxor_control(1U, false), TimePoint{} + 170ms)
          .status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(4U, false), TimePoint{} + 173ms)
          .status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(1U, true), TimePoint{} + 176ms)
          .status,
      core::RouterStatus::Applied);
  ASSERT_EQ(
      router.handle("arm", vaxor_control(4U, true), TimePoint{} + 179ms)
          .status,
      core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::AwaitingFirstCommand);
}

// CERB 没有独立控制帧，使能/失能随 MIT 命令帧的使能位携带。锁存后必须仍能接收
// 带 enabled==false 的 CERB MIT 帧完成失能，否则该轴永远停在已使能、reset 永远
// 返回 AxesStillEnabled，整臂死锁只能重启节点恢复（F1 回归）。
core::FrameRouter make_cerb_router(const bool fault_latching_enabled = true) {
  const core::TopologyContract contract{{"arm"}, {{"joint_cerb", "arm", "cerb", 1U}}};
  const auto path =
      std::string(TEST_SOURCE_DIR) + "/test/fixtures/cerb_axis.yaml";
  return core::FrameRouter(core::load_system_config(path, contract),
                           fault_latching_enabled);
}

protocol::CanFrame cerb_command(const bool enabled, const double position,
                                const double feedforward_nm = 0.0,
                                const double torque_limit_nm = 11.0,
                                const unsigned id = 1U) {
  protocol::CanFrame frame;
  // CERB MIT 命令帧 ID：0x110 + 电机 ID。
  frame.id = 0x110U + id;
  frame.fd = true;
  frame.brs = true;
  frame.size = 12U;
  // Data0 控制/模式半字节：bit7=使能、bit6=抱闸释放、bit4:1=MIT(0x06)。
  frame.data[0] = (enabled ? 0x80U : 0x00U) | 0x40U | 0x0CU;
  // 与 cerb_protocol.cpp 的 MIT 解码布局一致：pos 12.5 rad/16bit、vel
  // 30 rad/s/12bit、kp 500/12bit、kd 5/12bit；τ_ff 的满量程**逐型号**，由调用方给出。
  const uint32_t pos = protocol::float_to_uint(position, -12.5, 12.5, 16);
  const uint32_t vel = protocol::float_to_uint(0.0, -30.0, 30.0, 12);
  const uint32_t kp = protocol::float_to_uint(0.5, 0.0, 500.0, 12);
  const uint32_t kd = protocol::float_to_uint(0.02, 0.0, 5.0, 12);
  const uint32_t tq =
      protocol::float_to_uint(feedforward_nm, -torque_limit_nm,
                              torque_limit_nm, 12);
  frame.data[1] = static_cast<uint8_t>((pos >> 8U) & 0xffU);
  frame.data[2] = static_cast<uint8_t>(pos & 0xffU);
  frame.data[3] = static_cast<uint8_t>((vel >> 4U) & 0xffU);
  frame.data[4] = static_cast<uint8_t>(((vel & 0x0fU) << 4U) | ((kp >> 8U) & 0x0fU));
  frame.data[5] = static_cast<uint8_t>(kp & 0xffU);
  frame.data[6] = static_cast<uint8_t>((kd >> 4U) & 0xffU);
  frame.data[7] = static_cast<uint8_t>(((kd & 0x0fU) << 4U) | ((tq >> 8U) & 0x0fU));
  frame.data[8] = static_cast<uint8_t>(tq & 0xffU);
  return frame;
}

void prepare_cerb(core::FrameRouter &router, const TimePoint start) {
  ASSERT_EQ(router.handle("arm", cerb_command(true, 0.0), start).status,
            core::RouterStatus::Applied);
}

TEST(FrameRouterDriveState, CerbAxisDisableAfterLatchAllowsFullReset) {
  auto router = make_cerb_router();
  prepare_cerb(router, TimePoint{});
  EXPECT_EQ(router.drive_state(), core::DriveState::ReadyForControl);
  router.set_control_gate(true);
  EXPECT_EQ(router.drive_state(), core::DriveState::MotorControlActive);

  // 命令超时锁存整臂。
  const std::vector<core::JointSample> states(1U);
  EXPECT_EQ(router.compute_efforts(states, 0.01, TimePoint{} + 200ms),
            std::vector<double>(1U, 0.0));
  EXPECT_EQ(router.drive_state(), core::DriveState::FaultLatched);

  // 锁存后普通带使能运动帧不能解锁。
  EXPECT_EQ(router.handle("arm", cerb_command(true, 1.0), TimePoint{} + 210ms)
                .status,
            core::RouterStatus::SafetyFault);
  // 但带失能位的 CERB MIT 帧必须被接受。
  EXPECT_EQ(router.handle("arm", cerb_command(false, 1.0), TimePoint{} + 213ms)
                .status,
            core::RouterStatus::Applied);
  EXPECT_EQ(router.reset_fault(), core::ResetFaultResult::Cleared);
  EXPECT_EQ(router.drive_state(), core::DriveState::BrakeEngaged);

  // 重新上使能并下发新鲜命令可恢复控制。
  ASSERT_EQ(router.handle("arm", cerb_command(true, 0.0), TimePoint{} + 220ms)
                .status,
            core::RouterStatus::Applied);
  EXPECT_EQ(router.drive_state(), core::DriveState::ReadyForControl);
  router.set_control_gate(true);
  EXPECT_EQ(router.drive_state(), core::DriveState::MotorControlActive);
}

// 整链贯通：YAML(profile) → MotorConfig → 工厂 → CerbProtocol → 关节力矩。
// 5.0 N·m 按 R58 的 45.0 满量程编码；若这条链上任何一环退回默认的 R48 11.0，
// 读出来就是 1.222 N·m。
TEST(FrameRouterMitTorque, DecodesR58FeedforwardWithConfiguredLimit) {
  const core::TopologyContract contract{
      {"arm"}, {{"joint_cerb58", "arm", "cerb", 3U}}};
  const auto path =
      std::string(TEST_SOURCE_DIR) + "/test/fixtures/cerb_r58_axis.yaml";
  core::FrameRouter router(core::load_system_config(path, contract), true);
  ASSERT_EQ(router.handle("arm", cerb_command(true, 0.0, 5.0, 45.0, 3U),
                          TimePoint{})
                .status,
            core::RouterStatus::Applied);
  router.set_control_gate(true);

  // 命令 position≈0、反馈全零，kp/kd 项可忽略，plant 输出即 τ_ff 本身。
  const std::vector<core::JointSample> states(1U);
  const auto efforts = router.compute_efforts(states, 0.01, TimePoint{} + 3ms);
  ASSERT_EQ(efforts.size(), 1U);
  EXPECT_NEAR(efforts[0], 5.0, 2.0 * 45.0 / 4095.0);
}
