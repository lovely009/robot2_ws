#include "humanoid_gazebo/motor_sim/frame_router.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <vector>

namespace humanoid_sim = humanoid_gazebo::motor_sim;
namespace protocol = ::motor_sim::protocol;
using namespace std::chrono_literals;

namespace {

const std::string kMotorsYaml =
    std::string(TEST_SOURCE_DIR) + "/config/motors.yaml";

humanoid_sim::FrameRouter::TimePoint at_ms(const int64_t milliseconds) {
  return humanoid_sim::FrameRouter::TimePoint{} +
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
  humanoid_sim::FrameRouter router(humanoid_sim::load_system_config(kMotorsYaml));
  const auto result =
      router.handle("left_arm", vaxor_command_frame(4U, 0.0), at_ms(0));
  EXPECT_EQ(result.status, humanoid_sim::RouterStatus::WrongBus);
  EXPECT_TRUE(router.safety_state().latched);
}

TEST(FrameRouter, OutOfCommandRangeLatchesWholeRobot) {
  humanoid_sim::FrameRouter router(humanoid_sim::load_system_config(kMotorsYaml));
  ASSERT_EQ(router.handle("left_arm", rp_enable_frame(true), at_ms(0)).status,
            humanoid_sim::RouterStatus::Applied);
  const auto result =
      router.handle("left_arm", rp_command_frame(1.0), at_ms(10));
  EXPECT_EQ(result.status, humanoid_sim::RouterStatus::SafetyFault);
  EXPECT_EQ(router.safety_state().fault, humanoid_sim::Fault::CommandLimit);
  const auto efforts = router.compute_efforts(
      std::vector<humanoid_sim::JointSample>(30U), 0.01, at_ms(10));
  EXPECT_EQ(efforts, std::vector<double>(30U, 0.0));
}

TEST(FrameRouter, ExplicitEnableAndValidCommandProduceIndependentEffort) {
  humanoid_sim::FrameRouter router(humanoid_sim::load_system_config(kMotorsYaml));
  EXPECT_EQ(router.handle("left_arm", rp_enable_frame(true), at_ms(0)).status,
            humanoid_sim::RouterStatus::Applied);
  EXPECT_EQ(router.handle("left_arm", rp_command_frame(0.1), at_ms(10)).status,
            humanoid_sim::RouterStatus::Applied);
  const auto efforts = router.compute_efforts(
      std::vector<humanoid_sim::JointSample>(30U), 0.01, at_ms(10));
  EXPECT_GT(efforts[0], 0.0);
  EXPECT_NEAR(efforts[1], 0.0, 0.05);
  EXPECT_DOUBLE_EQ(efforts[9], 0.0);
}

TEST(FrameRouter, CommandDoesNotAutoEnableMotor) {
  humanoid_sim::FrameRouter router(humanoid_sim::load_system_config(kMotorsYaml));
  EXPECT_EQ(router.handle("left_arm", rp_command_frame(0.1), at_ms(0)).status,
            humanoid_sim::RouterStatus::Applied);
  const auto efforts = router.compute_efforts(
      std::vector<humanoid_sim::JointSample>(30U), 0.01, at_ms(0));
  EXPECT_EQ(efforts, std::vector<double>(30U, 0.0));
}

TEST(FrameRouter, TimeoutAndDisconnectLatchZeroEffort) {
  humanoid_sim::FrameRouter timeout_router(
      humanoid_sim::load_system_config(kMotorsYaml));
  timeout_router.handle("left_arm", rp_enable_frame(true), at_ms(0));
  EXPECT_EQ(timeout_router.compute_efforts(
                std::vector<humanoid_sim::JointSample>(30U), 0.01, at_ms(151)),
            std::vector<double>(30U, 0.0));
  EXPECT_EQ(timeout_router.safety_state().fault,
            humanoid_sim::Fault::CommandTimeout);

  humanoid_sim::FrameRouter disconnect_router(
      humanoid_sim::load_system_config(kMotorsYaml));
  disconnect_router.observe_disconnect("legs");
  EXPECT_EQ(disconnect_router.safety_state().fault,
            humanoid_sim::Fault::TransportDisconnected);
}

TEST(FrameRouter, VaxorModeGainCurrentLimitAndSetZeroAreApplied) {
  humanoid_sim::FrameRouter router(humanoid_sim::load_system_config(kMotorsYaml));
  constexpr std::size_t kRightForearm = 15U;
  ASSERT_EQ(router.handle("right_arm",
                          vaxor_control_frame(
                              1U, true, false, protocol::CommandMode::Mit),
                          at_ms(0))
                .status,
            humanoid_sim::RouterStatus::Applied);
  ASSERT_EQ(router.handle("right_arm", vaxor_command_frame(1U, 3.6, 0.008, 11.0),
                          at_ms(10))
                .status,
            humanoid_sim::RouterStatus::Applied);
  std::vector<humanoid_sim::JointSample> states(30U);
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
            humanoid_sim::RouterStatus::Applied);
  const auto feedback =
      router.motor_feedback(kRightForearm, states[kRightForearm], 0.0);
  EXPECT_NEAR(feedback.position, 0.0, 1e-12);
  EXPECT_TRUE(feedback.enabled);
  EXPECT_EQ(feedback.mode, protocol::CommandMode::Position);
}
