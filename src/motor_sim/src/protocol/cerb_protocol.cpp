#include "motor_sim/protocol/cerb_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motor_sim::protocol {
namespace {

// 手册标称量程（与达妙 MIT 惯例一致，可经上位机调整，仿真取默认值）。
constexpr double kCerbPositionMaxRad = 12.5;
constexpr double kCerbVelocityMaxRadS = 30.0;
constexpr double kCerbKpMax = 500.0;
constexpr double kCerbKdMax = 5.0;
constexpr double kCerbTorqueMaxNm = 10.0;

// 反馈帧负载端位置：±32768 对应 ±180°。
constexpr double kCerbFeedbackCountsPerRad = 32768.0 / kPi;
// 反馈帧线圈温度仿名义值 25.0°C，单位 0.1°C。
constexpr uint16_t kCerbNominalTemperatureDeciDegC = 250U;

// Data[0] 控制位与模式半字节（bit4:1）。
constexpr uint8_t kCerbEnableBit = 0x80U;
constexpr uint8_t kCerbBrakeReleaseBit = 0x40U;
constexpr uint8_t kCerbClearErrorBit = 0x20U;
constexpr uint8_t kCerbModePp = 0x01U;
constexpr uint8_t kCerbModePv = 0x02U;
constexpr uint8_t kCerbModeCsp = 0x03U;
constexpr uint8_t kCerbModeCsv = 0x04U;
constexpr uint8_t kCerbModeCurrent = 0x05U;
constexpr uint8_t kCerbModeMit = 0x06U;

uint16_t pack_u16(const int32_t value) {
  return static_cast<uint16_t>(static_cast<uint32_t>(value) & 0xffffU);
}

}  // namespace

void CerbProtocol::validate_motor_id(const unsigned motor_id) const {
  if (motor_id < 1U || motor_id > 127U) {
    throw std::out_of_range("CERB actuator ID must be in [1, 127]");
  }
}

std::optional<MotorCommand>
CerbProtocol::decode_command(const CanFrame &frame,
                             const unsigned motor_id) const {
  validate_motor_id(motor_id);
  if (frame.extended || !frame.fd || frame.size < 9U) {
    return std::nullopt;
  }
  if (frame.id != 0x110U + motor_id) {
    return std::nullopt;
  }
  const auto mode = static_cast<uint8_t>((frame.data[0] >> 1U) & 0x0fU);
  // 首期仅支持手册第 5 节的 MIT 模式；PP/PV/CSP/CSV/电流模式忽略。
  if (mode != kCerbModeMit) {
    return std::nullopt;
  }
  const uint32_t position =
      (static_cast<uint32_t>(frame.data[1]) << 8U) | frame.data[2];
  const uint32_t velocity =
      (static_cast<uint32_t>(frame.data[3]) << 4U) |
      static_cast<uint32_t>(frame.data[4] >> 4U);
  const uint32_t kp =
      (static_cast<uint32_t>(frame.data[4] & 0x0fU) << 8U) | frame.data[5];
  const uint32_t kd =
      (static_cast<uint32_t>(frame.data[6]) << 4U) |
      static_cast<uint32_t>(frame.data[7] >> 4U);
  const uint32_t torque =
      (static_cast<uint32_t>(frame.data[7] & 0x0fU) << 8U) | frame.data[8];

  MotorCommand command;
  command.motor_id = motor_id;
  command.mode = CommandMode::Mit;
  command.enabled = (frame.data[0] & kCerbEnableBit) != 0U;
  command.set_zero = false;  // CERB 标零走 SDO 字典，不在 MIT 帧内。
  command.position = uint_to_float(position, -kCerbPositionMaxRad,
                                   kCerbPositionMaxRad, 16);
  command.velocity = uint_to_float(velocity, -kCerbVelocityMaxRadS,
                                   kCerbVelocityMaxRadS, 12);
  command.kp = uint_to_float(kp, 0.0, kCerbKpMax, 12);
  command.kd = uint_to_float(kd, 0.0, kCerbKdMax, 12);
  command.feedforward =
      uint_to_float(torque, -kCerbTorqueMaxNm, kCerbTorqueMaxNm, 12);
  return command;
}

CanFrame CerbProtocol::encode_feedback(const MotorFeedback &feedback,
                                       const FeedbackKind kind) const {
  validate_motor_id(feedback.motor_id);
  // 手册未定义周期状态帧，Reply 与 Status 统一编码为命令触发的反馈报文。
  static_cast<void>(kind);

  const auto position_counts = static_cast<int32_t>(std::llround(
      std::clamp(feedback.position, -kPi, kPi) * kCerbFeedbackCountsPerRad));
  const auto velocity_rpm = static_cast<int32_t>(
      std::llround(feedback.velocity * 60.0 / (2.0 * kPi)));
  const auto current_ma =
      static_cast<int32_t>(std::llround(feedback.effort * 1000.0));

  std::array<uint8_t, 12> payload{};
  const uint16_t position_raw =
      pack_u16(std::clamp<int32_t>(position_counts, -32768, 32767));
  payload[0] = static_cast<uint8_t>(position_raw >> 8U);
  payload[1] = static_cast<uint8_t>(position_raw);
  const uint16_t velocity_raw =
      pack_u16(std::clamp<int32_t>(velocity_rpm, -32768, 32767));
  payload[2] = static_cast<uint8_t>(velocity_raw >> 8U);
  payload[3] = static_cast<uint8_t>(velocity_raw);
  const uint16_t current_raw =
      pack_u16(std::clamp<int32_t>(current_ma, -32768, 32767));
  payload[4] = static_cast<uint8_t>(current_raw >> 8U);
  payload[5] = static_cast<uint8_t>(current_raw);
  // 故障码保持 0（无故障），温度为仿名义值。
  payload[8] = static_cast<uint8_t>(kCerbNominalTemperatureDeciDegC >> 8U);
  payload[9] = static_cast<uint8_t>(kCerbNominalTemperatureDeciDegC);
  switch (feedback.mode) {
    case CommandMode::Position:
      payload[10] = kCerbModeCsp;
      break;
    case CommandMode::Velocity:
      payload[10] = kCerbModeCsv;
      break;
    default:
      payload[10] = kCerbModeMit;
      break;
  }
  payload[11] = static_cast<uint8_t>(
      (feedback.enabled ? kCerbEnableBit : 0U) |
      (feedback.enabled ? kCerbBrakeReleaseBit : 0U));

  CanFrame frame;
  frame.id = 0x300U + feedback.motor_id;
  frame.fd = true;
  frame.brs = true;
  frame.size = payload.size();
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

}  // namespace motor_sim::protocol
