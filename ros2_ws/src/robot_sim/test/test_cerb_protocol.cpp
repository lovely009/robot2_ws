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
  // golden vector：使能+释放抱闸+MIT 模式，pos≈1.0 rad，增益 raw=16/8，
  // 前馈≈0，12 字节 / DLC=9（9 字节有效载荷 + 3 填充）。
  const auto command = cerb.decode_command(
      frame_from_bytes(0x111U,
                       {0xCC, 0xA8, 0xC3, 0x80, 0x00, 0x10, 0x00, 0x88, 0x00,
                        0x00, 0x00, 0x00},
                       true),
      1U);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->motor_id, 1U);
  EXPECT_EQ(command->mode, protocol::CommandMode::Mit);
  EXPECT_TRUE(command->enabled);
  EXPECT_NEAR(command->position, 1.0, 6.28 / 65535.0);
  EXPECT_NEAR(command->velocity, 0.0, 6.28 / 4095.0);
  EXPECT_NEAR(command->kp, 16.0 * 2500.0 / 4095.0, 1e-12);
  EXPECT_NEAR(command->kd, 8.0 * 500.0 / 4095.0, 1e-12);
  // 零点的精确码是 2047.5，golden 向量取 2048 ⇒ 恒有半个 LSB 的偏置，
  // 容差为默认 300 N·m 量程的 1 个 LSB。
  EXPECT_NEAR(command->feedforward, 0.0,
              2.0 * protocol::kCerbDefaultMitTorqueLimitNm / 4095.0);

  // 下使能帧：bit7=0。
  const auto disabled = cerb.decode_command(
      frame_from_bytes(0x111U,
                       {0x4C, 0xA8, 0xC3, 0x80, 0x00, 0x10, 0x00, 0x88, 0x00,
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
  input.effort = 0.8;      // 电机端 0.8 A ÷ 20 mA/码 = 40 码 = 0x0028
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
  EXPECT_EQ(frame.data[4], 0x00U);
  EXPECT_EQ(frame.data[5], 0x28U);
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

TEST(CerbProtocol, GuideScalesAndMultiAxisHighNode) {
  const protocol::CerbProtocol cerb;
  const std::vector<uint8_t> payload{
      0xcc, 0xa8, 0xc3, 0xab, 0xc0, 0x31, 0x00, 0x88, 0x00, 0, 0, 0};
  auto single = frame_from_bytes(0x111U, payload, true);
  const auto command = cerb.decode_command(single, 1U);
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(command->position, 1.0, 6.28 / 65535.0);
  EXPECT_NEAR(command->velocity, 0xabc * 6.28 / 4095.0 - 3.14, 1e-12);
  EXPECT_NEAR(command->kp, 49.0 * 2500.0 / 4095.0, 1e-12);
  EXPECT_NEAR(command->kd, 8 * 500.0 / 4095.0, 1e-12);
  protocol::CanFrame multi;
  multi.id = 0x210U;
  multi.fd = multi.brs = true;
  multi.size = 64U;
  std::copy_n(payload.begin(), 9U, multi.data.begin() + 45U);
  multi.data[61] = 16U;
  const auto high = cerb.decode_command(multi, 16U);
  ASSERT_TRUE(high.has_value());
  EXPECT_DOUBLE_EQ(high->position, command->position);
  EXPECT_FALSE(cerb.decode_command(multi, 1U).has_value());
  single.data.fill(0U);
  single.data[0] = 0xcc;
  EXPECT_FALSE(cerb.decode_command(single, 1U).has_value());
}

TEST(CerbProtocol, FactoryCreatesCerbProtocol) {
  EXPECT_NO_THROW(protocol::make_motor_protocol("cerb"));
}

// τ_ff 量化满量程须与型号配置一致（R58/R48 均为 300 N·m）；
// 用两档量程验证编解码对称，防止固定量程导致整体缩放。
TEST(CerbProtocol, DecodesFeedforwardWithPerModelQuantizationLimit) {
  for (const double limit_nm : {300.0, 45.0}) {
    const auto torque = protocol::float_to_uint(5.0, -limit_nm, limit_nm, 12U);
    // 复用 golden 帧，只替换 τ_ff 的 12 位（Data7 低半字节 + Data8）。
    std::vector<uint8_t> bytes{0xCC, 0x8A, 0x3C, 0x7F, 0xF0, 0x51,
                               0x33, 0x37, 0xFF, 0x00, 0x00, 0x00};
    bytes[7] = static_cast<uint8_t>((bytes[7] & 0xf0U) |
                                    ((torque >> 8U) & 0x0fU));
    bytes[8] = static_cast<uint8_t>(torque & 0xffU);
    const protocol::CerbProtocol cerb(limit_nm);
    const auto command =
        cerb.decode_command(frame_from_bytes(0x111U, bytes, true), 1U);
    ASSERT_TRUE(command.has_value()) << "limit=" << limit_nm;
    // 容差 = 两倍量化步长（2·limit/4095）。
    EXPECT_NEAR(command->feedforward, 5.0, 2.0 * limit_nm / 4095.0)
        << "limit=" << limit_nm;
  }
}

TEST(CerbProtocol, FactoryPassesAllFiveCustomScales) {
  const auto cerb = protocol::make_motor_protocol("cerb", 150.0, 6.28, 8.0, 1200.0, 200.0);
  const auto command = cerb->decode_command(frame_from_bytes(
      0x111U, {0xcc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0, 0}, true), 1U);
  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->position, 6.28);
  EXPECT_DOUBLE_EQ(command->velocity, 8.0);
  EXPECT_DOUBLE_EQ(command->feedforward, 150.0);
  EXPECT_DOUBLE_EQ(command->kp, 1200.0);
  EXPECT_DOUBLE_EQ(command->kd, 200.0);
}

TEST(CerbProtocol, RejectsNonPositiveQuantizationLimit) {
  EXPECT_THROW(protocol::CerbProtocol(0.0), std::invalid_argument);
  EXPECT_THROW(protocol::CerbProtocol(-11.0), std::invalid_argument);
  EXPECT_NO_THROW(
      protocol::CerbProtocol(protocol::kCerbDefaultMitTorqueLimitNm));
}
