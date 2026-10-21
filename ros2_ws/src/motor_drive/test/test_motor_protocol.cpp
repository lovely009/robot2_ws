#include "motor_drive/motor_protocol.hpp"

#include <array>
#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace drive = motor_drive;
namespace protocol = motor_drive::protocol;

namespace {

drive::MotorConfig make_motor(const std::string &name, const std::string &joint,
                              const std::string &type, const unsigned id,
                              const drive::MotorParameterMap &parameters = {}) {
  return {name, joint, "test_bus", type, id, parameters};
}

std::unique_ptr<drive::MotorProtocol> builtin_protocol(const std::string &type) {
  return drive::MotorProtocolRegistry::with_builtin_protocols().create(type);
}

// 断言帧的 ID、长度、FD 标志与载荷前缀逐字节相等。
void expect_frame(const drive::CanFrame &frame, const uint32_t id,
                  const std::vector<uint8_t> &payload, const bool is_fd) {
  EXPECT_EQ(frame.id, id);
  EXPECT_EQ(frame.size, payload.size());
  EXPECT_EQ(frame.is_fd, is_fd);
  EXPECT_EQ(frame.bitrate_switch, is_fd);
  for (std::size_t index = 0; index < payload.size(); ++index) {
    EXPECT_EQ(frame.data[index], payload[index]) << "byte " << index;
  }
}

}  // namespace

TEST(VaxorProtocolStrategy, EnableFramesArePerNodeWithControlPayload) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U,
                 {{"torque_constant_nm_per_amp", "0.35"}}),
      make_motor("w2", "j2", drive::kVaxorType, 2U,
                 {{"torque_constant_nm_per_amp", "0.35"}})};
  const auto frames = builtin_protocol(drive::kVaxorType)
                          ->make_enable_frames(motors, {0U, 1U}, true);
  ASSERT_EQ(frames.size(), 2U);
  expect_frame(frames[0], protocol::wrist_control_id(1U),
               {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}, false);
  expect_frame(frames[1], protocol::wrist_control_id(2U),
               {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}, false);
}

TEST(VaxorProtocolStrategy, CommandFramesMatchManualExample) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U,
                 {{"torque_constant_nm_per_amp", "0.35"}})};
  protocol::JointCommand command;
  command.position =
      65537.0 / protocol::kWristTicksPerRevolution * 2.0 * protocol::kPi;
  command.kp = 0.008;
  command.kd = 1.0;

  const auto frames = builtin_protocol(drive::kVaxorType)
                          ->make_command_frames(motors, {0U}, {command}, true);
  ASSERT_EQ(frames.size(), 1U);
  // 与 test_protocol.cpp 的 CommandMatchesManualExample 同一手册例值。
  const std::vector<uint8_t> expected{0x01, 0x00, 0x01, 0x80, 0x00, 0x80,
                                      0x20, 0x03, 0x64, 0x00, 0x00, 0x80};
  expect_frame(frames[0], protocol::wrist_command_id(1U), expected, true);
}

TEST(VaxorProtocolStrategy, DecodesReplyAndStatusWithTorqueConstantScaling) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U,
                 {{"torque_constant_nm_per_amp", "0.35"}})};
  const auto protocol = builtin_protocol(drive::kVaxorType);

  protocol::JointState current_a;
  current_a.position = 0.5;
  current_a.velocity = -0.25;
  current_a.effort = 1.5;  // 线上反馈 effort 单位为 A，解码后应为 Nm。

  drive::CanFrame frame;
  std::size_t motor_index = 99U;
  protocol::JointState state;
  const auto reply = protocol::pack_wrist_reply(1U, current_a);
  frame.id = protocol::wrist_reply_id(1U);
  frame.size = reply.size();
  std::copy(reply.begin(), reply.end(), frame.data.begin());
  EXPECT_TRUE(protocol->decode_feedback(motors, {0U}, frame, motor_index, state));
  EXPECT_EQ(motor_index, 0U);
  EXPECT_NEAR(state.position, 0.5,
              2.0 * protocol::kPi / protocol::kWristTicksPerRevolution);
  EXPECT_NEAR(state.velocity, -0.25, 2.0 * protocol::kPi / 600.0);
  EXPECT_NEAR(state.effort, 1.5 * 0.35, protocol::kWristCurrentScaleA * 0.35);

  const auto status = protocol::pack_wrist_status(1U, current_a);
  frame.id = protocol::wrist_status_id(1U);
  frame.size = status.size();
  std::copy(status.begin(), status.end(), frame.data.begin());
  EXPECT_TRUE(protocol->decode_feedback(motors, {0U}, frame, motor_index, state));
  EXPECT_NEAR(state.effort, 1.5 * 0.35, protocol::kWristCurrentScaleA * 0.35);

  frame.is_extended = true;
  EXPECT_FALSE(protocol->decode_feedback(motors, {0U}, frame, motor_index, state));
  frame.is_extended = false;
  frame.id = 0x600U;
  EXPECT_FALSE(protocol->decode_feedback(motors, {0U}, frame, motor_index, state));
}

TEST(VaxorProtocolStrategy, ValidateRequiresTorqueConstantAndUniqueNodes) {
  const auto protocol = builtin_protocol(drive::kVaxorType);
  std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U)};
  EXPECT_THROW(protocol->validate(motors, {0U}), std::invalid_argument);

  motors[0].parameters["torque_constant_nm_per_amp"] = "0.35";
  motors.push_back(make_motor("w2", "j2", drive::kVaxorType, 1U,
                              {{"torque_constant_nm_per_amp", "0.35"}}));
  EXPECT_THROW(protocol->validate(motors, {0U, 1U}), std::invalid_argument);

  motors[1].id = 61U;
  EXPECT_THROW(protocol->validate(motors, {0U, 1U}), std::out_of_range);
}

TEST(VaxorProtocolStrategy, GearRatioAndEfficiencyScaleFeedback) {
  // gear_ratio=50、gear_efficiency=0.65：命令位置/速度 ×50（关节→电机），
  // 反馈位置/速度 ÷50、effort = 电流 × Kt × 减速比 × 效率（关节侧 Nm）。
  const std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U,
                 {{"torque_constant_nm_per_amp", "0.35"},
                  {"gear_ratio", "50"},
                  {"gear_efficiency", "0.65"}})};
  const auto protocol = builtin_protocol(drive::kVaxorType);

  // 命令：关节位置 0.5 rad、速度 -0.25 rad/s → 电机侧码值放大 50 倍。
  protocol::JointCommand command;
  command.position = 0.5;
  command.velocity = -0.25;
  const auto frames =
      protocol->make_command_frames(motors, {0U}, {command}, true);
  ASSERT_EQ(frames.size(), 1U);
  const auto motor_cmd =
      protocol::unpack_wrist_command(frames[0].data.data(), frames[0].size);
  EXPECT_NEAR(motor_cmd.position, 0.5 * 50.0,
              2.0 * protocol::kPi / protocol::kWristTicksPerRevolution);
  EXPECT_NEAR(motor_cmd.velocity, -0.25 * 50.0,
              2.0 * protocol::kPi / 600.0);

  // 反馈：电机侧位置 0.5、速度 -0.25、电流 1.5 A → 关节侧位置/速度 ÷50、
  // effort = 电流 × Kt × 减速比 × 效率（关节侧 Nm）。
  protocol::JointState motor_side;
  motor_side.position = 0.5;
  motor_side.velocity = -0.25;
  motor_side.effort = 1.5;
  const auto reply = protocol::pack_wrist_reply(1U, motor_side);
  drive::CanFrame frame;
  frame.id = protocol::wrist_reply_id(1U);
  frame.size = reply.size();
  std::copy(reply.begin(), reply.end(), frame.data.begin());
  std::size_t motor_index = 99U;
  protocol::JointState state;
  EXPECT_TRUE(protocol->decode_feedback(motors, {0U}, frame, motor_index, state));
  EXPECT_EQ(motor_index, 0U);
  EXPECT_NEAR(state.position, 0.5 / 50.0,
              2.0 * protocol::kPi / protocol::kWristTicksPerRevolution);
  EXPECT_NEAR(state.velocity, -0.25 / 50.0, 2.0 * protocol::kPi / 600.0);
  EXPECT_NEAR(state.effort, 1.5 * 0.35 * 50.0 * 0.65,
              protocol::kWristCurrentScaleA * 0.35 * 50.0 * 0.65);
}

TEST(VaxorProtocolStrategy, ValidateRejectsBadMotorTypeCode) {
  const auto protocol = builtin_protocol(drive::kVaxorType);
  std::vector<drive::MotorConfig> motors{
      make_motor("w1", "j1", drive::kVaxorType, 1U,
                 {{"torque_constant_nm_per_amp", "0.35"},
                  {"motor_type_code", "0x05"}})};
  EXPECT_THROW(protocol->validate(motors, {0U}), std::invalid_argument);
}

TEST(CerbProtocolStrategy, CommandFrameUsesCanFdMitPdo) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("r58", "j1", drive::kCerbType, 1U,
                 {{"torque_constant_nm_per_amp", "0.086"},
                  {"gear_ratio", "101"},
                  {"mit_position_limit_rad", "12.5"},
                  {"mit_velocity_limit_rad_s", "30"},
                  {"mit_torque_limit_nm", "10"}})};
  protocol::JointCommand command;
  command.position = 12.5;
  command.velocity = -30.0;
  command.kp = 500.0;
  command.kd = 5.0;
  command.feedforward = -10.0;

  const auto protocol = builtin_protocol(drive::kCerbType);
  const auto disabled_frames =
      protocol->make_command_frames(motors, {0U}, {command}, false);
  ASSERT_EQ(disabled_frames.size(), 1U);
  EXPECT_EQ(disabled_frames[0].data[0], 0x0cU);

  const auto frames =
      protocol->make_command_frames(motors, {0U}, {command}, true);
  ASSERT_EQ(frames.size(), 1U);
  expect_frame(frames[0], 0x111U,
               {0xcc, 0xff, 0xff, 0x00, 0x0f,
                0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00}, true);
}

TEST(CerbProtocolStrategy, EnableAndDisableUseSafeNeutralMitFrames) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("r58", "j1", drive::kCerbType, 2U,
                 {{"torque_constant_nm_per_amp", "0.086"},
                  {"gear_ratio", "101"},
                  {"mit_position_limit_rad", "12.5"},
                  {"mit_velocity_limit_rad_s", "30"},
                  {"mit_torque_limit_nm", "10"}})};
  const auto protocol = builtin_protocol(drive::kCerbType);

  const auto enabled = protocol->make_enable_frames(motors, {0U}, true);
  ASSERT_EQ(enabled.size(), 1U);
  expect_frame(enabled[0], 0x112U,
               {0xcc, 0x7f, 0xff, 0x7f, 0xf0,
                0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00}, true);

  const auto disabled = protocol->make_enable_frames(motors, {0U}, false);
  ASSERT_EQ(disabled.size(), 1U);
  expect_frame(disabled[0], 0x112U,
               {0x0c, 0x7f, 0xff, 0x7f, 0xf0,
                0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00}, true);
}

TEST(CerbProtocolStrategy, FeedbackConvertsMotorSpeedAndIqToJointState) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("r58", "j1", drive::kCerbType, 1U,
                 {{"torque_constant_nm_per_amp", "0.086"},
                  {"gear_ratio", "101"},
                  {"mit_position_limit_rad", "12.5"},
                  {"mit_velocity_limit_rad_s", "30"},
                  {"mit_torque_limit_nm", "10"}})};
  drive::CanFrame frame;
  frame.id = 0x301U;
  frame.size = 12U;
  frame.is_fd = true;
  frame.bitrate_switch = true;
  frame.data = {0x07, 0x1c, 0xff, 0xc4, 0x05, 0xdc,
                0x00, 0x00, 0x00, 0xf0, 0x06, 0xc0};

  std::size_t motor_index = 99U;
  protocol::JointState state;
  EXPECT_TRUE(builtin_protocol(drive::kCerbType)
                  ->decode_feedback(motors, {0U}, frame, motor_index, state));
  EXPECT_EQ(motor_index, 0U);
  EXPECT_NEAR(state.position, 10.0 * protocol::kPi / 180.0, 1e-12);
  EXPECT_NEAR(state.velocity, -2.0 * protocol::kPi / 101.0, 1e-12);
  // Iq 码 1500 × 20 mA/码 = 30 A
  EXPECT_NEAR(state.effort, 1500 * 0.020 * 0.086 * 101.0, 1e-12);

  frame.bitrate_switch = false;
  EXPECT_FALSE(builtin_protocol(drive::kCerbType)
                   ->decode_feedback(motors, {0U}, frame, motor_index, state));
  frame.bitrate_switch = true;
  frame.data[11] = 0xe0;
  EXPECT_FALSE(builtin_protocol(drive::kCerbType)
                   ->decode_feedback(motors, {0U}, frame, motor_index, state));
}

TEST(CerbProtocolStrategy, FeedbackAppliesGearEfficiencyToEffort) {
  // 反馈 Iq 是电机侧电流：Kt 得电机轴 N·m，再乘速比与减速器效率（厂家 2026-10-21 修正值）。
  const std::vector<drive::MotorConfig> motors{
      make_motor("r58", "j1", drive::kCerbType, 1U,
                 {{"torque_constant_nm_per_amp", "0.067"},
                  {"gear_ratio", "101"},
                  {"gear_efficiency", "0.5"},
                  {"mit_position_limit_rad", "12.5"},
                  {"mit_velocity_limit_rad_s", "30"},
                  {"mit_torque_limit_nm", "300"}})};
  drive::CanFrame frame;
  frame.id = 0x301U;
  frame.size = 12U;
  frame.is_fd = true;
  frame.bitrate_switch = true;
  frame.data = {0x07, 0x1c, 0xff, 0xc4, 0x05, 0xdc,
                0x00, 0x00, 0x00, 0xf0, 0x06, 0xc0};
  std::size_t motor_index = 99U;
  protocol::JointState state;
  EXPECT_TRUE(builtin_protocol(drive::kCerbType)
                  ->decode_feedback(motors, {0U}, frame, motor_index, state));
  EXPECT_NEAR(state.effort, 1500 * 0.020 * 0.067 * 101.0 * 0.5, 1e-12);

  // 效率必须是正数：配置成 0 要在 validate 阶段拒掉，而不是静默把力矩算成 0。
  auto zero_efficiency = motors;
  zero_efficiency[0].parameters["gear_efficiency"] = "0";
  EXPECT_THROW(
      builtin_protocol(drive::kCerbType)->validate(zero_efficiency, {0U}),
      std::invalid_argument);
}

TEST(CerbProtocolStrategy, ValidatesNodeIdsAndRequiredParameters) {
  const auto protocol = builtin_protocol(drive::kCerbType);
  std::vector<drive::MotorConfig> motors{
      make_motor("r58", "j1", drive::kCerbType, 0U,
                 {{"torque_constant_nm_per_amp", "0.086"},
                  {"gear_ratio", "101"}})};
  EXPECT_THROW(protocol->validate(motors, {0U}), std::out_of_range);

  motors[0].id = 1U;
  motors[0].parameters.erase("gear_ratio");
  EXPECT_THROW(protocol->validate(motors, {0U}), std::invalid_argument);

  motors[0].parameters["gear_ratio"] = "101";
  EXPECT_THROW(protocol->validate(motors, {0U}), std::invalid_argument);
}

TEST(CerbProtocolStrategy, UsesPerMotorMitRangeOverrides) {
  const std::vector<drive::MotorConfig> motors{
      make_motor("r48", "j1", drive::kCerbType, 1U,
                 {{"torque_constant_nm_per_amp", "0.077"},
                  {"gear_ratio", "101"},
                  {"mit_position_limit_rad", "25"},
                  {"mit_velocity_limit_rad_s", "60"},
                  {"mit_torque_limit_nm", "20"}})};
  protocol::JointCommand command;
  command.position = 12.5;
  command.velocity = -30.0;
  command.feedforward = -10.0;
  const auto frames = builtin_protocol(drive::kCerbType)
                          ->make_command_frames(motors, {0U}, {command}, false);
  ASSERT_EQ(frames.size(), 1U);
  expect_frame(frames[0], 0x111U,
               {0x0c, 0xbf, 0xff, 0x3f, 0xf0,
                0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00}, true);
}
