#include "motor_sim/core/frame_router.hpp"
#include "humanoid_gazebo/topology.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <vector>

namespace core = motor_sim::core;
namespace protocol = ::motor_sim::protocol;
using namespace std::chrono_literals;

namespace {

const std::string kMotorsYaml =
    std::string(TEST_SOURCE_DIR) + "/config/motors.yaml";

core::FrameRouter::TimePoint at_ms(const int64_t milliseconds) {
  return core::FrameRouter::TimePoint{} +
         std::chrono::milliseconds(milliseconds);
}

protocol::CanFrame rp_enable_frame(const bool enabled) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpEnableId;
  frame.size = 8U;
  frame.data[0] = static_cast<uint8_t>(enabled);
  return frame;
}

protocol::CanFrame rp_command_frame(const double first_position) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.fd = true;
  frame.brs = true;
  frame.size = 64U;
  protocol::MotorCommand neutral;
  neutral.enabled = true;
  for (std::size_t slot = 0U; slot < protocol::kRpBroadcastSlots; ++slot) {
    const auto payload = protocol::pack_rp_slot(neutral);
    std::copy(payload.begin(), payload.end(), frame.data.begin() + slot * 8U);
  }
  // 左臂 ID 6 是 elbow_pitch，command margin 后 0 rad 不再是安全命令。
  auto elbow = neutral;
  elbow.position = 0.1;
  const auto elbow_payload = protocol::pack_rp_slot(elbow);
  std::copy(elbow_payload.begin(), elbow_payload.end(), frame.data.begin() + 40U);
  auto first = neutral;
  first.position = first_position;
  const auto payload = protocol::pack_rp_slot(first);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_command_frame(const unsigned node,
                                       const double motor_position,
                                       const double kp = 0.008,
                                       const double current = 0.0) {
  protocol::MotorCommand command;
  command.enabled = true;
  command.position = motor_position;
  command.kp = kp;
  command.kd = 0.0;
  command.feedforward = current;
  const auto payload = protocol::pack_vaxor_command(command);
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_command_id(node);
  frame.fd = true;
  frame.brs = true;
  frame.size = payload.size();
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_control_frame(const unsigned node, const bool enabled,
                                       const bool set_zero,
                                       const protocol::CommandMode mode) {
  const auto payload =
      protocol::pack_vaxor_control(enabled, set_zero, mode);
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_control_id(node);
  frame.size = payload.size();
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

}  // namespace

TEST(FrameRouter, RejectsKnownVaxorNodeOnWrongBus) {
  core::FrameRouter router(core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  const auto result =
      router.handle("left_arm", vaxor_command_frame(4U, 0.0), at_ms(0));
  EXPECT_EQ(result.status, core::RouterStatus::WrongBus);
  EXPECT_TRUE(router.safety_state().latched);
}

TEST(FrameRouter, OutOfCommandRangeLatchesWholeRobot) {
  core::FrameRouter router(core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  ASSERT_EQ(router.handle("left_arm", rp_enable_frame(true), at_ms(0)).status,
            core::RouterStatus::Applied);
  const auto result =
      router.handle("left_arm", rp_command_frame(1.0), at_ms(10));
  EXPECT_EQ(result.status, core::RouterStatus::SafetyFault);
  EXPECT_EQ(router.safety_state().fault, core::Fault::CommandLimit);
  const auto efforts = router.compute_efforts(
      std::vector<core::JointSample>(30U), 0.01, at_ms(10));
  EXPECT_EQ(efforts, std::vector<double>(30U, 0.0));
}

TEST(FrameRouter, ExplicitEnableAndValidCommandProduceIndependentEffort) {
  core::FrameRouter router(core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  EXPECT_EQ(router.handle("left_arm", rp_enable_frame(true), at_ms(0)).status,
            core::RouterStatus::Applied);
  EXPECT_EQ(router.handle("left_arm", rp_command_frame(0.1), at_ms(10)).status,
            core::RouterStatus::Applied);
  const auto efforts = router.compute_efforts(
      std::vector<core::JointSample>(30U), 0.01, at_ms(10));
  EXPECT_GT(efforts[0], 0.0);
  EXPECT_NEAR(efforts[1], 0.0, 0.05);
  EXPECT_DOUBLE_EQ(efforts[9], 0.0);
}

TEST(FrameRouter, CommandDoesNotAutoEnableMotor) {
  core::FrameRouter router(core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  EXPECT_EQ(router.handle("left_arm", rp_command_frame(0.1), at_ms(0)).status,
            core::RouterStatus::Applied);
  const auto efforts = router.compute_efforts(
      std::vector<core::JointSample>(30U), 0.01, at_ms(0));
  EXPECT_EQ(efforts, std::vector<double>(30U, 0.0));
}

TEST(FrameRouter, TimeoutAndDisconnectLatchZeroEffort) {
  core::FrameRouter timeout_router(
      core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  timeout_router.handle("left_arm", rp_enable_frame(true), at_ms(0));
  EXPECT_EQ(timeout_router.compute_efforts(
                std::vector<core::JointSample>(30U), 0.01, at_ms(151)),
            std::vector<double>(30U, 0.0));
  EXPECT_EQ(timeout_router.safety_state().fault,
            core::Fault::CommandTimeout);

  core::FrameRouter disconnect_router(
      core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  disconnect_router.observe_disconnect("legs");
  EXPECT_EQ(disconnect_router.safety_state().fault,
            core::Fault::TransportDisconnected);
}

TEST(FrameRouter, VaxorModeGainCurrentLimitAndSetZeroAreApplied) {
  core::FrameRouter router(core::load_system_config(kMotorsYaml, humanoid_gazebo::humanoid_topology_contract()));
  constexpr std::size_t kRightForearm = 15U;
  ASSERT_EQ(router.handle("right_arm",
                          vaxor_control_frame(
                              1U, true, false, protocol::CommandMode::Mit),
                          at_ms(0))
                .status,
            core::RouterStatus::Applied);
  ASSERT_EQ(router.handle("right_arm", vaxor_command_frame(1U, 3.6, 0.008, 11.0),
                          at_ms(10))
                .status,
            core::RouterStatus::Applied);
  std::vector<core::JointSample> states(30U);
  states[kRightForearm].position_rad = 0.0;
  const auto efforts = router.compute_efforts(states, 0.01, at_ms(10));
  EXPECT_GT(efforts[kRightForearm], 3.7);
  EXPECT_LE(efforts[kRightForearm], 3.82);

  states[kRightForearm].position_rad = 0.2;
  static_cast<void>(router.compute_efforts(states, 0.01, at_ms(11)));
  ASSERT_EQ(router.handle("right_arm",
                          vaxor_control_frame(
                              1U, true, true, protocol::CommandMode::Position),
                          at_ms(20))
                .status,
            core::RouterStatus::Applied);
  const auto feedback =
      router.motor_feedback(kRightForearm, states[kRightForearm], 0.0);
  EXPECT_NEAR(feedback.position, 0.0, 1e-12);
  EXPECT_TRUE(feedback.enabled);
  EXPECT_EQ(feedback.mode, protocol::CommandMode::Position);
}
