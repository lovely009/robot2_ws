#include "motor_sim/protocol/cerb_protocol.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace protocol = motor_sim::protocol;

namespace {

protocol::CanFrame frame_from_bytes(const uint32_t id,
                                    const std::vector<uint8_t> &bytes,
                                    const bool fd) {
  protocol::CanFrame frame;
  frame.id = id;
  frame.fd = fd;
  frame.brs = fd;
  frame.size = bytes.size();
  std::copy(bytes.begin(), bytes.end(), frame.data.begin());
  return frame;
}

}  // namespace

TEST(CerbProtocol, ValidatesActuatorIdRange) {
  const protocol::CerbProtocol cerb;
  EXPECT_NO_THROW(cerb.validate_motor_id(1U));
  EXPECT_NO_THROW(cerb.validate_motor_id(127U));
  EXPECT_THROW(cerb.validate_motor_id(0U), std::out_of_range);
  EXPECT_THROW(cerb.validate_motor_id(128U), std::out_of_range);
}

TEST(CerbProtocol, DecodesMitCommandFrame) {
  const protocol::CerbProtocol cerb;
  // golden vector：使能+释放抱闸+MIT 模式，pos=1.0 rad、vel=0、
  // kp=10、kd=1、前馈=0，DLC=12（9 字节有效载荷 + 3 填充）。
  const auto command = cerb.decode_command(
      frame_from_bytes(0x111U,
                       {0xCC, 0x8A, 0x3C, 0x7F, 0xF0, 0x51, 0x33, 0x37, 0xFF,
                        0x00, 0x00, 0x00},
                       true),
      1U);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->motor_id, 1U);
  EXPECT_EQ(command->mode, protocol::CommandMode::Mit);
  EXPECT_TRUE(command->enabled);
  EXPECT_NEAR(command->position, 1.0, 25.0 / 65535.0);
  EXPECT_NEAR(command->velocity, 0.0, 60.0 / 4095.0);
  EXPECT_NEAR(command->kp, 10.0, 500.0 / 4095.0);
  EXPECT_NEAR(command->kd, 1.0, 5.0 / 4095.0);
  EXPECT_NEAR(command->feedforward, 0.0, 20.0 / 4095.0);

  // 下使能帧：bit7=0。
  const auto disabled = cerb.decode_command(
      frame_from_bytes(0x111U,
                       {0x4C, 0x8A, 0x3C, 0x7F, 0xF0, 0x51, 0x33, 0x37, 0xFF,
                        0x00, 0x00, 0x00},
                       true),
      1U);
  ASSERT_TRUE(disabled.has_value());
  EXPECT_FALSE(disabled->enabled);
}

TEST(CerbProtocol, EncodesFeedbackReportBytes) {
  const protocol::CerbProtocol cerb;
  protocol::MotorFeedback input;
  input.motor_id = 5U;
  input.position = 0.5;    // 负载端 rad -> round(0.5*32768/pi)=5215=0x145F
  input.velocity = -4.0 * protocol::kPi;  // 电机端 rad/s -> -120 RPM = 0xFF88
  input.effort = 0.8;      // 电机端 A -> 800 mA = 0x0320
  input.enabled = true;
  input.mode = protocol::CommandMode::Mit;

  const auto frame =
      cerb.encode_feedback(input, protocol::FeedbackKind::Reply);
  EXPECT_EQ(frame.id, 0x305U);
  EXPECT_EQ(frame.size, 12U);
  EXPECT_TRUE(frame.fd);
  EXPECT_TRUE(frame.brs);
  // 手册第 6 节反馈报文布局：位置/速度/电流均为大端 int16。
  EXPECT_EQ(frame.data[0], 0x14U);
  EXPECT_EQ(frame.data[1], 0x5FU);
  EXPECT_EQ(frame.data[2], 0xFFU);
  EXPECT_EQ(frame.data[3], 0x88U);
  EXPECT_EQ(frame.data[4], 0x03U);
  EXPECT_EQ(frame.data[5], 0x20U);
  EXPECT_EQ(frame.data[6], 0x00U);  // 故障码高字节
  EXPECT_EQ(frame.data[7], 0x00U);  // 故障码低字节
  EXPECT_EQ(frame.data[8], 0x00U);  // 温度 25.0°C -> 250 = 0x00FA
  EXPECT_EQ(frame.data[9], 0xFAU);
  EXPECT_EQ(frame.data[10], 0x06U);  // MIT 模式反馈
  EXPECT_EQ(frame.data[11], 0xC0U);  // 上使能 | 抱闸释放，无报错

  // 下使能时状态位清零。
  input.enabled = false;
  const auto idle = cerb.encode_feedback(input, protocol::FeedbackKind::Reply);
  EXPECT_EQ(idle.data[11], 0x00U);
}

TEST(CerbProtocol, RejectsUnsupportedOrForeignFrames) {
  const protocol::CerbProtocol cerb;
  const std::vector<uint8_t> mit{0xCC, 0x8A, 0x3C, 0x7F, 0xF0, 0x51,
                                 0x33, 0x37, 0xFF, 0x00, 0x00, 0x00};

  // 经典 CAN 同 ID：CERB MIT 至少需要 9 字节，必须是 FD 帧。
  EXPECT_FALSE(cerb.decode_command(frame_from_bytes(0x111U, {0xCC}, false),
                                   1U).has_value());
  // 扩展帧拒绝。
  auto extended = frame_from_bytes(0x111U, mit, true);
  extended.extended = true;
  EXPECT_FALSE(cerb.decode_command(extended, 1U).has_value());
  // 长度不足 9 字节。
  EXPECT_FALSE(
      cerb.decode_command(frame_from_bytes(0x111U, {0xCC, 0x8A, 0x3C, 0x7F,
                                                   0xF0, 0x51, 0x33, 0x37},
                                           true),
                          1U)
          .has_value());
  // 非 MIT 模式（CSP=0x03 -> Data0 bit4:1=0x06）暂不支持。
  auto csp = mit;
  csp[0] = 0x06U;
  EXPECT_FALSE(cerb.decode_command(frame_from_bytes(0x111U, csp, true), 1U)
                   .has_value());
  // 其他执行器的命令 ID。
  EXPECT_FALSE(cerb.decode_command(frame_from_bytes(0x112U, mit, true), 1U)
                   .has_value());
}

TEST(CerbProtocol, FactoryCreatesCerbProtocol) {
  EXPECT_NO_THROW(protocol::make_motor_protocol("cerb"));
}
