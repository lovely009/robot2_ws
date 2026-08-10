#include "upper_body_canfd_hardware/rp_mit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace upper_body_canfd_hardware::rp {

namespace {

double ClampRange(double value, double low, double high)
{
  return std::min(std::max(value, low), high);
}

}  // namespace

double Clamp(double value, double low, double high)
{
  return ClampRange(value, low, high);
}

uint32_t FloatToUint(double value, double low, double high, int bits)
{
  value = ClampRange(value, low, high);
  return static_cast<uint32_t>(
    (value - low) * ((1ULL << bits) - 1ULL) / (high - low));
}

double UintToFloat(uint32_t value, double low, double high, int bits)
{
  return value * (high - low) / ((1ULL << bits) - 1ULL) + low;
}

std::vector<uint8_t> PackMitSlot(const MitCommand & command)
{
  const uint32_t p = FloatToUint(command.position, kPositionRangeMin, kPositionRangeMax, 16);
  const uint32_t v = FloatToUint(command.velocity, kVelocityRangeMin, kVelocityRangeMax, 12);
  const uint32_t kp = FloatToUint(command.kp, kKpRangeMin, kKpRangeMax, 12);
  const uint32_t kd = FloatToUint(command.kd, kKdRangeMin, kKdRangeMax, 12);
  const uint32_t tq = FloatToUint(command.torque_ff, kTorqueRangeMin, kTorqueRangeMax, 12);
  return {
    static_cast<uint8_t>((p >> 8) & 0xFF),
    static_cast<uint8_t>(p & 0xFF),
    static_cast<uint8_t>((v >> 4) & 0xFF),
    static_cast<uint8_t>(((v & 0x0F) << 4) | ((kp >> 8) & 0x0F)),
    static_cast<uint8_t>(kp & 0xFF),
    static_cast<uint8_t>((kd >> 4) & 0xFF),
    static_cast<uint8_t>(((kd & 0x0F) << 4) | ((tq >> 8) & 0x0F)),
    static_cast<uint8_t>(tq & 0xFF),
  };
}

MitCommand UnpackMitSlot(const std::vector<uint8_t> & data)
{
  if (data.size() != 8) {
    throw std::invalid_argument("RP MIT slot must be exactly 8 bytes");
  }
  const uint32_t p = (static_cast<uint32_t>(data[0]) << 8) | data[1];
  const uint32_t v = (static_cast<uint32_t>(data[2]) << 4) | (data[3] >> 4);
  const uint32_t kp = ((static_cast<uint32_t>(data[3]) & 0x0F) << 8) | data[4];
  const uint32_t kd = (static_cast<uint32_t>(data[5]) << 4) | (data[6] >> 4);
  const uint32_t tq = ((static_cast<uint32_t>(data[6]) & 0x0F) << 8) | data[7];
  return MitCommand{
    UintToFloat(p, kPositionRangeMin, kPositionRangeMax, 16),
    UintToFloat(v, kVelocityRangeMin, kVelocityRangeMax, 12),
    UintToFloat(kp, kKpRangeMin, kKpRangeMax, 12),
    UintToFloat(kd, kKdRangeMin, kKdRangeMax, 12),
    UintToFloat(tq, kTorqueRangeMin, kTorqueRangeMax, 12),
  };
}

std::vector<uint8_t> PackBroadcast(const std::vector<MitCommand> & commands)
{
  if (commands.size() > kMotorsPerBroadcast) {
    throw std::invalid_argument("RP broadcast supports at most 8 joints");
  }
  // Never pad with zero bytes: all-zero bytes decode to -60 Nm feed-forward.
  const MitCommand neutral;
  std::vector<MitCommand> slots = commands;
  slots.resize(kMotorsPerBroadcast, neutral);
  std::vector<uint8_t> out;
  out.reserve(kMotorsPerBroadcast * 8);
  for (const auto & slot : slots) {
    const auto packed = PackMitSlot(slot);
    out.insert(out.end(), packed.begin(), packed.end());
  }
  return out;
}

std::vector<MitCommand> UnpackBroadcast(const std::vector<uint8_t> & data)
{
  if (data.size() != 64) {
    throw std::invalid_argument("RP MIT broadcast must be exactly 64 bytes");
  }
  std::vector<MitCommand> out;
  out.reserve(kMotorsPerBroadcast);
  for (int i = 0; i < kMotorsPerBroadcast; ++i) {
    const std::vector<uint8_t> slot(data.begin() + i * 8, data.begin() + (i + 1) * 8);
    out.push_back(UnpackMitSlot(slot));
  }
  return out;
}

std::vector<uint8_t> PackEnable(bool enabled)
{
  std::vector<uint8_t> out(8, 0);
  out[0] = enabled ? 0x01 : 0x00;
  return out;
}

std::vector<uint8_t> PackState(
  double position, double velocity, double torque,
  double mos_temperature_c, int status, double voltage_v,
  double coil_temperature_c)
{
  const uint32_t p = FloatToUint(position, kPositionRangeMin, kPositionRangeMax, 16);
  const uint32_t v = FloatToUint(velocity, kVelocityRangeMin, kVelocityRangeMax, 12);
  const uint32_t tq = FloatToUint(torque, kTorqueRangeMin, kTorqueRangeMax, 12);
  const uint32_t mos = static_cast<uint32_t>(
    ClampRange(mos_temperature_c + 40.0, 0.0, 255.0));
  const uint32_t coil = static_cast<uint32_t>(
    ClampRange(coil_temperature_c + 40.0, 0.0, 255.0));
  const uint32_t voltage = static_cast<uint32_t>(ClampRange(voltage_v, 0.0, 255.0));
  return {
    static_cast<uint8_t>((p >> 8) & 0xFF),
    static_cast<uint8_t>(p & 0xFF),
    static_cast<uint8_t>((v >> 4) & 0xFF),
    static_cast<uint8_t>(((v & 0x0F) << 4) | ((tq >> 8) & 0x0F)),
    static_cast<uint8_t>(tq & 0xFF),
    static_cast<uint8_t>(mos & 0xFF),
    static_cast<uint8_t>(status & 0xFF),
    static_cast<uint8_t>((status >> 8) & 0xFF),
    static_cast<uint8_t>(voltage & 0xFF),
    static_cast<uint8_t>(coil & 0xFF),
    0x00,
    0x00,
  };
}

State UnpackState(const std::vector<uint8_t> & data)
{
  if (data.size() != 12) {
    throw std::invalid_argument("RP state frame must be exactly 12 bytes");
  }
  const uint32_t p = (static_cast<uint32_t>(data[0]) << 8) | data[1];
  const uint32_t v = (static_cast<uint32_t>(data[2]) << 4) | (data[3] >> 4);
  const uint32_t tq = ((static_cast<uint32_t>(data[3]) & 0x0F) << 8) | data[4];
  return State{
    UintToFloat(p, kPositionRangeMin, kPositionRangeMax, 16),
    UintToFloat(v, kVelocityRangeMin, kVelocityRangeMax, 12),
    UintToFloat(tq, kTorqueRangeMin, kTorqueRangeMax, 12),
    static_cast<double>(data[5]) - 40.0,
    static_cast<int>(data[6]) | (static_cast<int>(data[7]) << 8),
    static_cast<double>(data[8]),
    static_cast<double>(data[9]) - 40.0,
  };
}

}  // namespace upper_body_canfd_hardware::rp
