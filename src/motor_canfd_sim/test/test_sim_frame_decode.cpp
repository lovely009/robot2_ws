#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "motor_canfd_drive/motor_canfd_driver.hpp"
#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_sim/sim_frame_decode.hpp"

namespace motor_canfd_sim {
namespace {

namespace drive = motor_canfd_drive;
namespace protocol = motor_canfd_drive::protocol;

drive::DriverConfig make_config_with_bus(
    const std::vector<std::pair<std::string, std::string>> &motors) {
  drive::DriverConfig config;
  config.buses.push_back({"test_bus", "vcan0"});
  for (std::size_t index = 0; index < motors.size(); ++index) {
    drive::MotorConfig motor;
    motor.name = motors[index].first;
    motor.joint_name = motors[index].first + "_joint";
    motor.bus = "test_bus";
    motor.type = motors[index].second;
    motor.id = static_cast<unsigned>(index + 1U);
    config.motors.push_back(motor);
  }
  return config;
}

std::vector<std::size_t> all_motors(const drive::DriverConfig &config) {
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < config.motors.size(); ++index) {
    indices.push_back(index);
  }
  return indices;
}

// ---- decode_rp_frame ----

TEST(DecodeRpFrame, EnableFrameSetsEnabledForRpMotorsOnly) {
  const auto config = make_config_with_bus(
      {{"rp_motor", "rp"}, {"wrist_motor", "motor_2530"}, {"rp_motor_2", "rp"}});
  drive::CanFrame frame;
  frame.id = protocol::kRpEnableId;
  frame.size = 1U;
  frame.data[0] = 1U;

  const auto result = decode_rp_frame(config, all_motors(config), frame);
  ASSERT_TRUE(result.consumed);
  EXPECT_FALSE(result.needs_feedback);
  ASSERT_EQ(result.updates.size(), 2U);
  EXPECT_EQ(result.updates[0].motor, 0U);
  EXPECT_TRUE(result.updates[0].set_enabled);
  EXPECT_TRUE(result.updates[0].enabled);
  EXPECT_EQ(result.updates[1].motor, 2U);
  EXPECT_TRUE(result.updates[1].enabled);
}

TEST(DecodeRpFrame, CommandFrameUsesIdBasedSlotOffset) {
  auto config = make_config_with_bus({{"rp_1", "rp"}, {"rp_4", "rp"}});
  // 电机 id = 1 和 2（配置顺序），分别占用槽 0 和槽 1；这里验证偏移按 id 计算。
  config.motors[1].id = 4U;
  drive::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.size = 64U;
  const protocol::JointCommand command_a{0.5, 0.0, 30.0, 1.0, 0.0};
  const protocol::JointCommand command_b{-0.25, 0.0, 30.0, 1.0, 0.0};
  const auto slot_a = protocol::pack_rp_slot(command_a);
  const auto slot_b = protocol::pack_rp_slot(command_b);
  std::copy(slot_a.begin(), slot_a.end(), frame.data.begin());
  std::copy(slot_b.begin(), slot_b.end(), frame.data.begin() + 3U * 8U);

  const auto result = decode_rp_frame(config, all_motors(config), frame);
  ASSERT_TRUE(result.consumed);
  EXPECT_TRUE(result.needs_feedback);
  ASSERT_EQ(result.updates.size(), 2U);
  EXPECT_EQ(result.updates[0].motor, 0U);
  EXPECT_NEAR(result.updates[0].command.position, 0.5, 1e-3);
  EXPECT_EQ(result.updates[1].motor, 1U);
  EXPECT_NEAR(result.updates[1].command.position, -0.25, 1e-3);
}

TEST(DecodeRpFrame, CommandFrameWithoutRpMotorsIsConsumedWithoutFeedback) {
  const auto config =
      make_config_with_bus({{"wrist_motor", "motor_2530"}});
  drive::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.size = 64U;

  const auto result = decode_rp_frame(config, all_motors(config), frame);
  EXPECT_TRUE(result.consumed);
  EXPECT_FALSE(result.needs_feedback);
  EXPECT_TRUE(result.updates.empty());
}

TEST(DecodeRpFrame, UnknownFrameIsNotConsumed) {
  const auto config = make_config_with_bus({{"rp_motor", "rp"}});
  drive::CanFrame frame;
  frame.id = 0x123U;
  frame.size = 8U;

  const auto result = decode_rp_frame(config, all_motors(config), frame);
  EXPECT_FALSE(result.consumed);
  EXPECT_TRUE(result.updates.empty());
}

// ---- decode_motor_2530_frame ----

TEST(DecodeMotor2530Frame, ControlFrameParsesRunSetZeroAndMode) {
  const auto config = make_config_with_bus(
      {{"rp_motor", "rp"}, {"wrist_2", "motor_2530"}, {"wrist_3", "motor_2530"}});
  drive::CanFrame frame;
  frame.id = protocol::wrist_control_id(2U);
  frame.size = 8U;
  const auto payload = protocol::pack_wrist_control(
      true, true, protocol::WristMode::Velocity);
  std::copy(payload.begin(), payload.end(), frame.data.begin());

  const auto result = decode_motor_2530_frame(config, all_motors(config), frame);
  ASSERT_TRUE(result.consumed);
  EXPECT_FALSE(result.needs_feedback);
  ASSERT_EQ(result.updates.size(), 1U);
  EXPECT_EQ(result.updates[0].motor, 1U);
  EXPECT_TRUE(result.updates[0].set_control);
  EXPECT_TRUE(result.updates[0].control.run);
  EXPECT_TRUE(result.updates[0].control.set_zero);
  EXPECT_EQ(result.updates[0].control.mode, protocol::WristMode::Velocity);
}

TEST(DecodeMotor2530Frame, CommandFrameParsesCommandAndNeedsReply) {
  const auto config = make_config_with_bus({{"wrist_1", "motor_2530"}});
  drive::CanFrame frame;
  frame.id = protocol::wrist_command_id(1U);
  frame.size = 12U;
  const protocol::JointCommand command{0.2, 0.0, 0.008, 1.0, 0.0};
  const auto payload = protocol::pack_wrist_command(command);
  std::copy(payload.begin(), payload.end(), frame.data.begin());

  const auto result = decode_motor_2530_frame(config, all_motors(config), frame);
  ASSERT_TRUE(result.consumed);
  EXPECT_TRUE(result.needs_feedback);
  ASSERT_EQ(result.updates.size(), 1U);
  EXPECT_EQ(result.updates[0].motor, 0U);
  EXPECT_TRUE(result.updates[0].set_command);
  EXPECT_NEAR(result.updates[0].command.position, 0.2, 1e-3);
}

TEST(DecodeMotor2530Frame, FrameForNodeNotOnBusIsNotConsumed) {
  const auto config = make_config_with_bus({{"wrist_2", "motor_2530"}});
  drive::CanFrame control_frame;
  control_frame.id = protocol::wrist_control_id(5U);
  control_frame.size = 8U;
  const auto payload = protocol::pack_wrist_control(true);
  std::copy(payload.begin(), payload.end(), control_frame.data.begin());

  const auto result =
      decode_motor_2530_frame(config, all_motors(config), control_frame);
  EXPECT_FALSE(result.consumed);
  EXPECT_TRUE(result.updates.empty());
}

TEST(DecodeMotor2530Frame, UnknownFrameIsNotConsumed) {
  const auto config = make_config_with_bus({{"wrist_2", "motor_2530"}});
  drive::CanFrame frame;
  frame.id = 0x700U;
  frame.size = 8U;

  const auto result = decode_motor_2530_frame(config, all_motors(config), frame);
  EXPECT_FALSE(result.consumed);
  EXPECT_TRUE(result.updates.empty());
}

} // namespace
} // namespace motor_canfd_sim
