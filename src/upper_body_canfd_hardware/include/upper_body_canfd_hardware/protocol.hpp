#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace upper_body_canfd_hardware::protocol {
constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kRpEnableId = 0x00;
constexpr uint32_t kRpCommandId = 0x80;
constexpr int32_t kWristTicksPerRevolution = 2097152;

inline uint32_t wrist_command_id(const unsigned node) {
  return 0x501U + 4U * node;
}
inline uint32_t wrist_control_id(const unsigned node) {
  return 0x502U + 4U * node;
}
inline uint32_t wrist_reply_id(const unsigned node) {
  return 0x701U + 4U * node;
}
inline uint32_t wrist_status_id(const unsigned node) {
  return 0x702U + 4U * node;
}

inline uint32_t float_to_uint(double value, const double low, const double high,
                              const unsigned bits) {
  value = std::clamp(value, low, high);
  return static_cast<uint32_t>((value - low) * ((1U << bits) - 1U) /
                               (high - low));
}
inline double uint_to_float(const uint32_t value, const double low,
                            const double high, const unsigned bits) {
  return value * (high - low) / ((1U << bits) - 1U) + low;
}

struct JointCommand {
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double feedforward{0.0};
};
struct JointState {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

inline std::array<uint8_t, 8> pack_rp_slot(const JointCommand &c) {
  const auto p = float_to_uint(c.position, -10.0, 10.0, 16);
  const auto v = float_to_uint(c.velocity, -60.0, 60.0, 12);
  const auto kp = float_to_uint(c.kp, 0.0, 500.0, 12);
  const auto kd = float_to_uint(c.kd, 0.0, 10.0, 12);
  const auto tq = float_to_uint(c.feedforward, -60.0, 60.0, 12);
  return {static_cast<uint8_t>(p >> 8),
          static_cast<uint8_t>(p),
          static_cast<uint8_t>(v >> 4),
          static_cast<uint8_t>((v << 4) | (kp >> 8)),
          static_cast<uint8_t>(kp),
          static_cast<uint8_t>(kd >> 4),
          static_cast<uint8_t>((kd << 4) | (tq >> 8)),
          static_cast<uint8_t>(tq)};
}

inline JointState unpack_rp_state(const uint8_t *d, const size_t size) {
  if (size != 12)
    throw std::invalid_argument("RP state must be 12 bytes");
  const uint32_t p = (d[0] << 8) | d[1];
  const uint32_t v = (d[2] << 4) | (d[3] >> 4);
  const uint32_t tq = ((d[3] & 0x0f) << 8) | d[4];
  return {uint_to_float(p, -10, 10, 16), uint_to_float(v, -60, 60, 12),
          uint_to_float(tq, -60, 60, 12)};
}

inline std::array<uint8_t, 12> pack_wrist_command(const JointCommand &c) {
  std::array<uint8_t, 12> out{};
  const auto ticks = static_cast<int32_t>(
      std::llround(c.position / (2.0 * kPi) * kWristTicksPerRevolution));
  const uint32_t p = static_cast<uint32_t>(
      static_cast<int64_t>(std::clamp(ticks, -0x7cffffff, 0x7cffffff)) +
      0x80000000LL);
  const auto rpm10 = static_cast<int32_t>(
      std::llround(c.velocity * 60.0 / (2.0 * kPi) * 10.0));
  const uint16_t v =
      static_cast<uint16_t>(std::clamp(rpm10, -30000, 30000) + 0x8000);
  const uint16_t kp = static_cast<uint16_t>(
      std::clamp(std::llround(c.kp / 0.00001), 0LL, 65535LL));
  const uint16_t kd = static_cast<uint16_t>(
      std::clamp(std::llround(c.kd / 0.01), 0LL, 65535LL));
  const auto current_raw =
      static_cast<int32_t>(std::llround(c.feedforward / (11.0 / 4096.0)));
  const uint16_t current =
      static_cast<uint16_t>(std::clamp(current_raw, -32768, 32767) + 0x8000);
  const uint32_t values[] = {p, v, kp, kd, current};
  const size_t widths[] = {4, 2, 2, 2, 2};
  size_t offset = 0;
  for (size_t i = 0; i < 5; ++i) {
    for (size_t b = 0; b < widths[i]; ++b)
      out[offset++] = values[i] >> (8 * b);
  }
  return out;
}

inline JointState unpack_wrist_feedback(const uint8_t *d, const size_t size) {
  if (size != 12 && size != 20)
    throw std::invalid_argument("wrist feedback must be 12 or 20 bytes");
  const uint32_t p =
      d[2] | (d[3] << 8) | (d[4] << 16) | (static_cast<uint32_t>(d[5]) << 24);
  const uint16_t v = d[6] | (d[7] << 8);
  const uint16_t i = d[8] | (d[9] << 8);
  return {(static_cast<int64_t>(p) - 0x80000000LL) /
              static_cast<double>(kWristTicksPerRevolution) * 2.0 * kPi,
          (static_cast<int32_t>(v) - 0x8000) / 10.0 * 2.0 * kPi / 60.0,
          (static_cast<int32_t>(i) - 0x8000) * (11.0 / 4096.0)};
}
} // namespace upper_body_canfd_hardware::protocol
