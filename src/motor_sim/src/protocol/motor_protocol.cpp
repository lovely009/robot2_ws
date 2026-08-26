#include "motor_sim/protocol/motor_protocol.hpp"

#include "motor_sim/protocol/rp_protocol.hpp"
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

}  // namespace

std::unique_ptr<MotorProtocol> make_motor_protocol(const std::string_view name) {
  if (name == "rp") {
    return std::make_unique<RpProtocol>();
  }
  if (name == "motor_2530") {
    return std::make_unique<VaxorProtocol>();
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

std::array<uint8_t, 8> pack_rp_slot(const MotorCommand &command) {
  const auto position = float_to_uint(command.position, -10.0, 10.0, 16);
  const auto velocity = float_to_uint(command.velocity, -60.0, 60.0, 12);
  const auto kp = float_to_uint(command.kp, 0.0, 500.0, 12);
  const auto kd = float_to_uint(command.kd, 0.0, 10.0, 12);
  const auto torque =
      float_to_uint(command.feedforward, -60.0, 60.0, 12);
  return {static_cast<uint8_t>(position >> 8U),
          static_cast<uint8_t>(position),
          static_cast<uint8_t>(velocity >> 4U),
          static_cast<uint8_t>((velocity << 4U) | (kp >> 8U)),
          static_cast<uint8_t>(kp),
          static_cast<uint8_t>(kd >> 4U),
          static_cast<uint8_t>((kd << 4U) | (torque >> 8U)),
          static_cast<uint8_t>(torque)};
}

MotorCommand unpack_rp_slot(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || size != 8U) {
    throw std::invalid_argument("RP MIT slot must be 8 bytes");
  }
  const uint32_t position =
      (static_cast<uint32_t>(data[0]) << 8U) | data[1];
  const uint32_t velocity =
      (static_cast<uint32_t>(data[2]) << 4U) | (data[3] >> 4U);
  const uint32_t kp =
      (static_cast<uint32_t>(data[3] & 0x0fU) << 8U) | data[4];
  const uint32_t kd =
      (static_cast<uint32_t>(data[5]) << 4U) | (data[6] >> 4U);
  const uint32_t torque =
      (static_cast<uint32_t>(data[6] & 0x0fU) << 8U) | data[7];
  MotorCommand command;
  command.mode = CommandMode::Mit;
  command.enabled = true;
  command.position = uint_to_float(position, -10.0, 10.0, 16);
  command.velocity = uint_to_float(velocity, -60.0, 60.0, 12);
  command.kp = uint_to_float(kp, 0.0, 500.0, 12);
  command.kd = uint_to_float(kd, 0.0, 10.0, 12);
  command.feedforward = uint_to_float(torque, -60.0, 60.0, 12);
  return command;
}

std::array<uint8_t, 12> pack_rp_state(const MotorFeedback &feedback) {
  const auto position = float_to_uint(feedback.position, -10.0, 10.0, 16);
  const auto velocity = float_to_uint(feedback.velocity, -60.0, 60.0, 12);
  const auto torque = float_to_uint(feedback.effort, -60.0, 60.0, 12);
  return {static_cast<uint8_t>(position >> 8U),
          static_cast<uint8_t>(position),
          static_cast<uint8_t>(velocity >> 4U),
          static_cast<uint8_t>((velocity << 4U) | (torque >> 8U)),
          static_cast<uint8_t>(torque),
          75U,
          1U,
          0U,
          24U,
          73U,
          0U,
          0U};
}

MotorFeedback unpack_rp_state(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || size != 12U) {
    throw std::invalid_argument("RP state must be 12 bytes");
  }
  const uint32_t position =
      (static_cast<uint32_t>(data[0]) << 8U) | data[1];
  const uint32_t velocity =
      (static_cast<uint32_t>(data[2]) << 4U) | (data[3] >> 4U);
  const uint32_t torque =
      (static_cast<uint32_t>(data[3] & 0x0fU) << 8U) | data[4];
  MotorFeedback feedback;
  feedback.position = uint_to_float(position, -10.0, 10.0, 16);
  feedback.velocity = uint_to_float(velocity, -60.0, 60.0, 12);
  feedback.effort = uint_to_float(torque, -60.0, 60.0, 12);
  return feedback;
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
  write_le(output.data() + 10U, static_cast<uint16_t>(0x8000 + 120), 2U);
  output[16] = 4U;
  output[18] = static_cast<uint8_t>(feedback.mode);
  output[19] = static_cast<uint8_t>(feedback.enabled);
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
