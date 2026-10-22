#include "motor_sim/protocol/motor_protocol.hpp"

#include "motor_sim/protocol/cerb_protocol.hpp"
#include "motor_sim/protocol/vaxor_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motor_sim::protocol {
namespace {

template <typename T>
void write_le(uint8_t *destination, T value, const std::size_t width) {
  const auto raw = static_cast<uint64_t>(value);
  for (std::size_t index = 0U; index < width; ++index) {
    destination[index] = static_cast<uint8_t>(raw >> (8U * index));
  }
}

uint32_t read_le(const uint8_t *data, const std::size_t width) {
  uint32_t value = 0U;
  for (std::size_t index = 0U; index < width; ++index) {
    value |= static_cast<uint32_t>(data[index]) << (8U * index);
  }
  return value;
}

uint16_t vaxor_velocity_to_code(const double velocity_rad_s) {
  const auto rpm10 = static_cast<int32_t>(
      std::llround(velocity_rad_s * 60.0 / (2.0 * kPi) * 10.0));
  return static_cast<uint16_t>(
      std::clamp(rpm10, -kVaxorVelocityCodeLimit, kVaxorVelocityCodeLimit) +
      0x8000);
}

double vaxor_code_to_velocity(const uint16_t code) {
  const auto raw = static_cast<int32_t>(code) - 0x8000;
  return raw / 10.0 * 2.0 * kPi / 60.0;
}

uint32_t vaxor_position_to_code(const double position_rad) {
  const auto ticks = static_cast<int64_t>(std::llround(
      position_rad / (2.0 * kPi) * kVaxorTicksPerRevolution));
  return static_cast<uint32_t>(
      std::clamp<int64_t>(ticks, -kVaxorPositionTickLimit,
                          kVaxorPositionTickLimit) +
      0x80000000LL);
}

double vaxor_code_to_position(const uint32_t code) {
  return (static_cast<int64_t>(code) - 0x80000000LL) /
         static_cast<double>(kVaxorTicksPerRevolution) * 2.0 * kPi;
}

uint16_t vaxor_current_to_code(const double current_a) {
  const auto raw =
      static_cast<int32_t>(std::llround(current_a / kVaxorCurrentScaleA));
  return static_cast<uint16_t>(std::clamp(raw, -32768, 32767) + 0x8000);
}

double vaxor_code_to_current(const uint16_t code) {
  return (static_cast<int32_t>(code) - 0x8000) * kVaxorCurrentScaleA;
}

CommandMode wire_mode(const uint8_t value) {
  if (value > static_cast<uint8_t>(CommandMode::Position)) {
    throw std::invalid_argument("unknown Vaxor control mode");
  }
  return static_cast<CommandMode>(value);
}

// 手册 3.5：母线电压 = 原始码值 / 10 V，该字段无 0x8000 偏移；
// 120 对应 12.0 V 仿名义供电。
constexpr uint16_t kVaxorNominalBusVoltageCode = 120U;
// 手册 3.5：电机类型 0x01=30mm、0x02=25mm、0x03=20mm、0x04=16mm；
// 仿真统一上报 30mm 仿名义类型。
constexpr uint8_t kVaxorSimulatedMotorType = 0x01U;

}  // namespace

std::unique_ptr<MotorProtocol> make_motor_protocol(
    const std::string_view name, const double mit_torque_limit_nm,
    const double mit_position_limit_rad, const double mit_velocity_limit_rad_s,
    const double mit_kp_limit, const double mit_kd_limit) {
  if (name == "vaxor") {
    return std::make_unique<VaxorProtocol>();
  }
  if (name == "cerb") {
    return std::make_unique<CerbProtocol>(mit_torque_limit_nm, mit_position_limit_rad,
                                          mit_velocity_limit_rad_s, mit_kp_limit,
                                          mit_kd_limit);
  }
  throw std::invalid_argument("unsupported motor protocol");
}

uint32_t float_to_uint(double value, const double low, const double high,
                       const unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    throw std::invalid_argument("invalid fixed-point range or bit width");
  }
  value = std::clamp(value, low, high);
  const auto maximum = (uint32_t{1} << bits) - 1U;
  return static_cast<uint32_t>((value - low) * maximum / (high - low));
}

double uint_to_float(const uint32_t value, const double low,
                     const double high, const unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    throw std::invalid_argument("invalid fixed-point range or bit width");
  }
  const auto maximum = (uint32_t{1} << bits) - 1U;
  return value * (high - low) / maximum + low;
}

void validate_vaxor_node(const unsigned node) {
  if (node < 1U || node > 60U) {
    throw std::out_of_range("Vaxor node must be in [1, 60]");
  }
}

uint32_t vaxor_command_id(const unsigned node) {
  validate_vaxor_node(node);
  return 0x501U + 4U * node;
}

uint32_t vaxor_control_id(const unsigned node) {
  validate_vaxor_node(node);
  return 0x502U + 4U * node;
}

uint32_t vaxor_reply_id(const unsigned node) {
  validate_vaxor_node(node);
  return 0x701U + 4U * node;
}

uint32_t vaxor_status_id(const unsigned node) {
  validate_vaxor_node(node);
  return 0x702U + 4U * node;
}

std::array<uint8_t, 12> pack_vaxor_command(const MotorCommand &command) {
  std::array<uint8_t, 12> output{};
  write_le(output.data(), vaxor_position_to_code(command.position), 4U);
  write_le(output.data() + 4U, vaxor_velocity_to_code(command.velocity), 2U);
  const auto kp = static_cast<uint16_t>(std::clamp<int64_t>(
      std::llround(command.kp / kVaxorKpScale), 0, 0xffff));
  const auto kd = static_cast<uint16_t>(std::clamp<int64_t>(
      std::llround(command.kd / kVaxorKdScale), 0, 0xffff));
  write_le(output.data() + 6U, kp, 2U);
  write_le(output.data() + 8U, kd, 2U);
  write_le(output.data() + 10U,
           vaxor_current_to_code(command.feedforward), 2U);
  return output;
}

MotorCommand unpack_vaxor_command(const uint8_t *data,
                                  const std::size_t size) {
  if (data == nullptr || size != 12U) {
    throw std::invalid_argument("Vaxor MIT command must be 12 bytes");
  }
  MotorCommand command;
  command.mode = CommandMode::Mit;
  command.enabled = true;
  command.position = vaxor_code_to_position(read_le(data, 4U));
  command.velocity = vaxor_code_to_velocity(
      static_cast<uint16_t>(read_le(data + 4U, 2U)));
  command.kp = static_cast<uint16_t>(read_le(data + 6U, 2U)) *
               kVaxorKpScale;
  command.kd = static_cast<uint16_t>(read_le(data + 8U, 2U)) *
               kVaxorKdScale;
  command.feedforward = vaxor_code_to_current(
      static_cast<uint16_t>(read_le(data + 10U, 2U)));
  return command;
}

std::array<uint8_t, 8> pack_vaxor_control(const bool run,
                                          const bool set_zero,
                                          const CommandMode mode) {
  if (mode != CommandMode::Mit && mode != CommandMode::Velocity &&
      mode != CommandMode::Position) {
    throw std::invalid_argument("invalid Vaxor wire mode");
  }
  return {static_cast<uint8_t>(run), static_cast<uint8_t>(set_zero),
          static_cast<uint8_t>(mode), 0U, 0U, 0U, 0U, 0U};
}

MotorCommand unpack_vaxor_control(const uint8_t *data,
                                  const std::size_t size) {
  if (data == nullptr || size != 8U) {
    throw std::invalid_argument("Vaxor control must be 8 bytes");
  }
  MotorCommand command;
  command.enabled = data[0] != 0U;
  command.set_zero = data[1] != 0U;
  command.mode = wire_mode(data[2]);
  return command;
}

std::array<uint8_t, 12> pack_vaxor_reply(
    const MotorFeedback &feedback) {
  std::array<uint8_t, 12> output{};
  write_le(output.data(), vaxor_command_id(feedback.motor_id), 2U);
  write_le(output.data() + 2U, vaxor_position_to_code(feedback.position), 4U);
  write_le(output.data() + 6U, vaxor_velocity_to_code(feedback.velocity), 2U);
  write_le(output.data() + 8U, vaxor_current_to_code(feedback.effort), 2U);
  return output;
}

std::array<uint8_t, 20> pack_vaxor_status(
    const MotorFeedback &feedback) {
  std::array<uint8_t, 20> output{};
  const auto reply = pack_vaxor_reply(feedback);
  std::copy(reply.begin(), reply.begin() + 10U, output.begin());
  write_le(output.data() + 10U, kVaxorNominalBusVoltageCode, 2U);
  // Data12~15 故障位保持 0（无故障），Data16=4 表示运行，Data17=0 空闲。
  output[16] = 4U;
  output[18] = static_cast<uint8_t>(feedback.mode);
  output[19] = kVaxorSimulatedMotorType;
  return output;
}

MotorFeedback unpack_vaxor_feedback(const uint8_t *data,
                                    const std::size_t size) {
  if (data == nullptr || (size != 12U && size != 20U)) {
    throw std::invalid_argument("Vaxor feedback must be 12 or 20 bytes");
  }
  MotorFeedback feedback;
  feedback.position = vaxor_code_to_position(read_le(data + 2U, 4U));
  feedback.velocity = vaxor_code_to_velocity(
      static_cast<uint16_t>(read_le(data + 6U, 2U)));
  feedback.effort = vaxor_code_to_current(
      static_cast<uint16_t>(read_le(data + 8U, 2U)));
  return feedback;
}

}  // namespace motor_sim::protocol
