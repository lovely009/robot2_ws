#include "motor_sim/protocol/cerb_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motor_sim::protocol {
namespace {

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

CerbProtocol::CerbProtocol(const double mit_torque_limit_nm,
                           const double mit_position_limit_rad,
                           const double mit_velocity_limit_rad_s,
                           const double mit_kp_limit, const double mit_kd_limit)
    : mit_torque_limit_nm_(mit_torque_limit_nm),
      mit_position_limit_rad_(mit_position_limit_rad),
      mit_velocity_limit_rad_s_(mit_velocity_limit_rad_s),
      mit_kp_limit_(mit_kp_limit), mit_kd_limit_(mit_kd_limit) {
  for (const double limit : {mit_torque_limit_nm_, mit_position_limit_rad_,
                            mit_velocity_limit_rad_s_, mit_kp_limit_, mit_kd_limit_}) {
    if (!std::isfinite(limit) || limit <= 0.0) {
      throw std::invalid_argument("CERB MIT limits must be positive and finite");
    }
  }
}

void CerbProtocol::validate_motor_id(const unsigned motor_id) const {
  if (motor_id < 1U || motor_id > 127U) {
    throw std::out_of_range("CERB actuator ID must be in [1, 127]");
  }
}

std::optional<MotorCommand>
CerbProtocol::decode_command(const CanFrame &frame,
                             const unsigned motor_id) const {
  validate_motor_id(motor_id);
  if (frame.extended || !frame.fd || !frame.brs) {
    return std::nullopt;
  }
  std::size_t offset = 0U;
  if (frame.id == 0x210U && frame.size == 64U) {
    bool found = false;
    std::array<bool, 128> seen{};
    if (frame.data[54] || frame.data[55] || frame.data[62] || frame.data[63]) {
      return std::nullopt;
    }
    for (std::size_t slot = 0U; slot < 6U; ++slot) {
      const auto node = frame.data[56U + slot];
      if (node == 0U) { continue; }
      if (node > 127U || seen[node]) { return std::nullopt; }
      seen[node] = true;
      if (node == motor_id) {
        found = true;
        offset = slot * 9U;
      }
    }
    if (!found) { return std::nullopt; }
  } else if (motor_id > 15U || frame.id != (0x110U | motor_id) ||
             frame.size != 12U) {
    return std::nullopt;
  }
  const auto *data = frame.data.data() + offset;
  const auto mode = static_cast<uint8_t>((data[0] >> 1U) & 0x0fU);
  // 首期仅支持手册第 5 节的 MIT 模式；PP/PV/CSP/CSV/电流模式忽略。
  if (mode != kCerbModeMit) {
    return std::nullopt;
  }
  const uint32_t position =
      (static_cast<uint32_t>(data[1]) << 8U) | data[2];
  const uint32_t velocity =
      (static_cast<uint32_t>(data[3]) << 4U) |
      static_cast<uint32_t>(data[4] >> 4U);
  const uint32_t kp =
      (static_cast<uint32_t>(data[4] & 0x0fU) << 8U) | data[5];
  const uint32_t kd =
      (static_cast<uint32_t>(data[6]) << 4U) |
      static_cast<uint32_t>(data[7] >> 4U);
  const uint32_t torque =
      (static_cast<uint32_t>(data[7] & 0x0fU) << 8U) | data[8];

  if ((position | velocity | kp | kd | torque) == 0U) {
    return std::nullopt;
  }
  MotorCommand command;
  command.motor_id = motor_id;
  command.mode = CommandMode::Mit;
  command.enabled = (data[0] & kCerbEnableBit) != 0U;
  command.set_zero = false;  // CERB 标零走 SDO 字典，不在 MIT 帧内。
  command.position = uint_to_float(position, -mit_position_limit_rad_,
                                   mit_position_limit_rad_, 16);
  command.velocity = uint_to_float(velocity, -mit_velocity_limit_rad_s_,
                                   mit_velocity_limit_rad_s_, 12);
  command.kp = uint_to_float(kp, 0.0, mit_kp_limit_, 12);
  command.kd = uint_to_float(kd, 0.0, mit_kd_limit_, 12);
  command.feedforward = uint_to_float(torque, -mit_torque_limit_nm_,
                                      mit_torque_limit_nm_, 12);
  return command;
}

CanFrame CerbProtocol::encode_feedback(const MotorFeedback &feedback,
                                       const FeedbackKind kind) const {
  validate_motor_id(feedback.motor_id);
  // 手册未定义周期状态帧，Reply 与 Status 统一编码为命令触发的反馈报文。
  static_cast<void>(kind);

  // 反馈位置刻度独立于 MIT 命令量程，按 int16 端点饱和。
  const auto position_counts = static_cast<int32_t>(std::llround(
      std::clamp(feedback.position, -kPi, kPi) *
      kCerbFeedbackCountsPerRad));
  const auto velocity_rpm = static_cast<int32_t>(
      std::llround(feedback.velocity * 60.0 / (2.0 * kPi)));
  // Iq 字段每码 20 mA（真机实测），编码端与真机同刻度。
  const auto current_code = static_cast<int32_t>(
      std::llround(feedback.effort / kCerbCurrentCodeA));

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
      pack_u16(std::clamp<int32_t>(current_code, -32768, 32767));
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
