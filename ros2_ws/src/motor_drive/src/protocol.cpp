#include "motor_drive/protocol.hpp"

#include <limits>

namespace motor_drive::protocol {
namespace {

template <typename T>
void write_le(uint8_t *destination, T value, const std::size_t width) {
  // 25/30 mm 协议使用小端序：低有效字节先放到低地址。
  const auto raw = static_cast<uint64_t>(value);
  for (std::size_t index = 0; index < width; ++index) {
    destination[index] = static_cast<uint8_t>(raw >> (8U * index));
  }
}

uint32_t read_le(const uint8_t *data, const std::size_t width) {
  uint32_t value = 0U;
  for (std::size_t index = 0; index < width; ++index) {
    value |= static_cast<uint32_t>(data[index]) << (8U * index);
  }
  return value;
}

uint16_t wrist_velocity_to_code(const double velocity_rad_s) {
  // rad/s -> rpm -> 0.1 rpm 协议码，并以 0x8000 表示有符号零点偏置。
  const auto rpm10 = static_cast<int32_t>(
      std::llround(velocity_rad_s * 60.0 / (2.0 * kPi) * 10.0));
  return static_cast<uint16_t>(
      std::clamp(rpm10, -kWristVelocityCodeLimit, kWristVelocityCodeLimit) +
      0x8000);
}

double wrist_code_to_velocity(const uint16_t code) {
  const auto raw = static_cast<int32_t>(code) - 0x8000;
  return raw / 10.0 * 2.0 * kPi / 60.0;
}

uint32_t wrist_position_to_code(const double position_rad) {
  // 一圈 2,097,152 tick。协议用 0x80000000 作为多圈有符号位置的零点偏置。
  const auto ticks = static_cast<int64_t>(std::llround(
      position_rad / (2.0 * kPi) * kWristTicksPerRevolution));
  return static_cast<uint32_t>(
      std::clamp<int64_t>(ticks, -kWristPositionTickLimit,
                          kWristPositionTickLimit) +
      0x80000000LL);
}

double wrist_code_to_position(const uint32_t code) {
  return (static_cast<int64_t>(code) - 0x80000000LL) /
         static_cast<double>(kWristTicksPerRevolution) * 2.0 * kPi;
}

uint16_t wrist_current_to_code(const double current_a) {
  const auto raw = static_cast<int32_t>(std::llround(current_a / kWristCurrentScaleA));
  return static_cast<uint16_t>(std::clamp(raw, -32768, 32767) + 0x8000);
}

double wrist_code_to_current(const uint16_t code) {
  return (static_cast<int32_t>(code) - 0x8000) * kWristCurrentScaleA;
}

} // namespace

uint32_t float_to_uint(double value, const double low, const double high,
                       const unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    throw std::invalid_argument("invalid float_to_uint range or bit width");
  }
  // RP MIT 字段是定点量：先按协议范围限幅，再线性映射到指定 bit 宽度。
  value = std::clamp(value, low, high);
  const auto maximum = (uint32_t{1} << bits) - 1U;
  return static_cast<uint32_t>((value - low) * maximum / (high - low));
}

double uint_to_float(const uint32_t value, const double low, const double high,
                     const unsigned bits) {
  if (!(high > low) || bits == 0U || bits > 31U) {
    throw std::invalid_argument("invalid uint_to_float range or bit width");
  }
  const auto maximum = (uint32_t{1} << bits) - 1U;
  return value * (high - low) / maximum + low;
}

std::array<uint8_t, 8> pack_rp_slot(const JointCommand &command) {
  // RP MIT 8 Byte 槽布局：P(16) + V(12) + Kp(12) + Kd(12) + Tau(12)。
  // 12 bit 字段跨字节拼接，因此这里不要改成普通 memcpy。
  const auto position = float_to_uint(command.position, -10.0, 10.0, 16);
  const auto velocity = float_to_uint(command.velocity, -60.0, 60.0, 12);
  const auto kp = float_to_uint(command.kp, 0.0, 500.0, 12);
  const auto kd = float_to_uint(command.kd, 0.0, 10.0, 12);
  const auto torque = float_to_uint(command.feedforward, -60.0, 60.0, 12);
  return {static_cast<uint8_t>(position >> 8U),
          static_cast<uint8_t>(position),
          static_cast<uint8_t>(velocity >> 4U),
          static_cast<uint8_t>((velocity << 4U) | (kp >> 8U)),
          static_cast<uint8_t>(kp),
          static_cast<uint8_t>(kd >> 4U),
          static_cast<uint8_t>((kd << 4U) | (torque >> 8U)),
          static_cast<uint8_t>(torque)};
}

JointCommand unpack_rp_slot(const uint8_t *data, const std::size_t size) {
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
  return {uint_to_float(position, -10.0, 10.0, 16),
          uint_to_float(velocity, -60.0, 60.0, 12),
          uint_to_float(kp, 0.0, 500.0, 12),
          uint_to_float(kd, 0.0, 10.0, 12),
          uint_to_float(torque, -60.0, 60.0, 12)};
}

std::array<uint8_t, 12>
pack_rp_state(const JointState &state, const double mos_temperature_c,
              const uint16_t status, const double voltage_v,
              const double coil_temperature_c) {
  const auto position = float_to_uint(state.position, -10.0, 10.0, 16);
  const auto velocity = float_to_uint(state.velocity, -60.0, 60.0, 12);
  const auto torque = float_to_uint(state.effort, -60.0, 60.0, 12);
  const auto mos = static_cast<uint8_t>(
      std::clamp(mos_temperature_c + 40.0, 0.0, 255.0));
  const auto voltage = static_cast<uint8_t>(std::clamp(voltage_v, 0.0, 255.0));
  const auto coil = static_cast<uint8_t>(
      std::clamp(coil_temperature_c + 40.0, 0.0, 255.0));
  return {static_cast<uint8_t>(position >> 8U),
          static_cast<uint8_t>(position),
          static_cast<uint8_t>(velocity >> 4U),
          static_cast<uint8_t>((velocity << 4U) | (torque >> 8U)),
          static_cast<uint8_t>(torque),
          mos,
          static_cast<uint8_t>(status),
          static_cast<uint8_t>(status >> 8U),
          voltage,
          coil,
          0U,
          0U};
}

JointState unpack_rp_state(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || size != 12U) {
    throw std::invalid_argument("RP state must be 12 bytes");
  }
  const uint32_t position =
      (static_cast<uint32_t>(data[0]) << 8U) | data[1];
  const uint32_t velocity =
      (static_cast<uint32_t>(data[2]) << 4U) | (data[3] >> 4U);
  const uint32_t torque =
      (static_cast<uint32_t>(data[3] & 0x0fU) << 8U) | data[4];
  return {uint_to_float(position, -10.0, 10.0, 16),
          uint_to_float(velocity, -60.0, 60.0, 12),
          uint_to_float(torque, -60.0, 60.0, 12)};
}

std::array<uint8_t, 12> pack_wrist_command(const JointCommand &command) {
  // 25/30 mm MIT 下行按手册依次写入：位置4B、速度2B、Kp2B、Kd2B、前馈电流2B。
  std::array<uint8_t, 12> output{};
  const auto position = wrist_position_to_code(command.position);
  const auto velocity = wrist_velocity_to_code(command.velocity);
  const auto kp = static_cast<uint16_t>(std::clamp<int64_t>(
      std::llround(command.kp / kWristKpScale), 0, 0xffff));
  const auto kd = static_cast<uint16_t>(std::clamp<int64_t>(
      std::llround(command.kd / kWristKdScale), 0, 0xffff));
  const auto current = wrist_current_to_code(command.feedforward);
  write_le(output.data(), position, 4U);
  write_le(output.data() + 4U, velocity, 2U);
  write_le(output.data() + 6U, kp, 2U);
  write_le(output.data() + 8U, kd, 2U);
  write_le(output.data() + 10U, current, 2U);
  return output;
}

JointCommand unpack_wrist_command(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || size != 12U) {
    throw std::invalid_argument("25/30 mm MIT command must be 12 bytes");
  }
  const auto position = read_le(data, 4U);
  const auto velocity = static_cast<uint16_t>(read_le(data + 4U, 2U));
  const auto kp = static_cast<uint16_t>(read_le(data + 6U, 2U));
  const auto kd = static_cast<uint16_t>(read_le(data + 8U, 2U));
  const auto current = static_cast<uint16_t>(read_le(data + 10U, 2U));
  return {wrist_code_to_position(position), wrist_code_to_velocity(velocity),
          kp * kWristKpScale, kd * kWristKdScale,
          wrist_code_to_current(current)};
}

std::array<uint8_t, 8> pack_wrist_control(const bool run, const bool set_zero,
                                           const WristMode mode) {
  return {static_cast<uint8_t>(run), static_cast<uint8_t>(set_zero),
          static_cast<uint8_t>(mode), 0U, 0U, 0U, 0U, 0U};
}

WristControl unpack_wrist_control(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || size != 8U) {
    throw std::invalid_argument("25/30 mm motor control must be 8 bytes");
  }
  if (data[2] > static_cast<uint8_t>(WristMode::Position)) {
    throw std::invalid_argument("unknown 25/30 mm control mode");
  }
  return {data[0] != 0U, data[1] != 0U, static_cast<WristMode>(data[2])};
}

std::array<uint8_t, 12>
pack_wrist_reply(const unsigned node, const JointState &state_current_a) {
  std::array<uint8_t, 12> output{};
  write_le(output.data(), wrist_command_id(node), 2U);
  write_le(output.data() + 2U, wrist_position_to_code(state_current_a.position),
           4U);
  write_le(output.data() + 6U, wrist_velocity_to_code(state_current_a.velocity),
           2U);
  write_le(output.data() + 8U, wrist_current_to_code(state_current_a.effort),
           2U);
  return output;
}

std::array<uint8_t, 20>
pack_wrist_status(const unsigned node, const JointState &state_current_a,
                  const double voltage_v, const uint32_t fault,
                  const uint8_t motor_state, const uint8_t calibration_state,
                  const WristMode mode, const uint8_t motor_type) {
  std::array<uint8_t, 20> output{};
  write_le(output.data(), wrist_command_id(node), 2U);
  write_le(output.data() + 2U, wrist_position_to_code(state_current_a.position),
           4U);
  write_le(output.data() + 6U, wrist_velocity_to_code(state_current_a.velocity),
           2U);
  write_le(output.data() + 8U, wrist_current_to_code(state_current_a.effort),
           2U);
  const auto voltage_code = static_cast<uint16_t>(
      std::clamp<int32_t>(static_cast<int32_t>(std::llround(voltage_v * 10.0)),
                          -32768, 32767) +
      0x8000);
  write_le(output.data() + 10U, voltage_code, 2U);
  write_le(output.data() + 12U, fault, 4U);
  output[16] = motor_state;
  output[17] = calibration_state;
  output[18] = static_cast<uint8_t>(mode);
  output[19] = motor_type;
  return output;
}

JointState unpack_wrist_feedback(const uint8_t *data, const std::size_t size) {
  if (data == nullptr || (size != 12U && size != 20U)) {
    throw std::invalid_argument("wrist feedback must be 12 or 20 bytes");
  }
  const auto position = read_le(data + 2U, 4U);
  const auto velocity = static_cast<uint16_t>(read_le(data + 6U, 2U));
  const auto current = static_cast<uint16_t>(read_le(data + 8U, 2U));
  return {wrist_code_to_position(position), wrist_code_to_velocity(velocity),
          wrist_code_to_current(current)};
}

double wrist_kp_to_current_gain(const double protocol_kp) {
  return protocol_kp / 0.001115;
}

double wrist_kd_to_current_gain(const double protocol_kd) {
  return protocol_kd / 85.636;
}

} // namespace motor_drive::protocol
