// Codec for the YiYou RP-series CAN-FD MIT protocol.
//
// Reference: "意优RP系列行星关节CAN-FD通信协议.pdf" V1.0. The MIT broadcast
// uses CAN ID 0x80 with one 8-byte slot per joint, up to eight joints / 64
// bytes. This translation is byte-for-byte compatible with the reference
// Python implementation in upper_body_canfd_gazebo/protocol_rp.py and is kept
// free of ROS dependencies so it can be unit-tested directly.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace upper_body_canfd_hardware::rp {

inline constexpr double kPositionRangeMin = -10.0;
inline constexpr double kPositionRangeMax = 10.0;
inline constexpr double kVelocityRangeMin = -60.0;
inline constexpr double kVelocityRangeMax = 60.0;
inline constexpr double kKpRangeMin = 0.0;
inline constexpr double kKpRangeMax = 500.0;
inline constexpr double kKdRangeMin = 0.0;
inline constexpr double kKdRangeMax = 10.0;
inline constexpr double kTorqueRangeMin = -60.0;
inline constexpr double kTorqueRangeMax = 60.0;

inline constexpr uint32_t kMitBroadcastId = 0x80;
inline constexpr uint32_t kEnableId = 0x00;
inline constexpr int kMotorsPerBroadcast = 8;

struct MitCommand {
  double position = 0.0;
  double velocity = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double torque_ff = 0.0;
};

struct State {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double mos_temperature_c = 0.0;
  int status = 0;
  double voltage_v = 0.0;
  double coil_temperature_c = 0.0;
};

double Clamp(double value, double low, double high);
uint32_t FloatToUint(double value, double low, double high, int bits);
double UintToFloat(uint32_t value, double low, double high, int bits);

std::vector<uint8_t> PackMitSlot(const MitCommand & command);
MitCommand UnpackMitSlot(const std::vector<uint8_t> & data);

std::vector<uint8_t> PackBroadcast(const std::vector<MitCommand> & commands);
std::vector<MitCommand> UnpackBroadcast(const std::vector<uint8_t> & data);

std::vector<uint8_t> PackEnable(bool enabled);

std::vector<uint8_t> PackState(
  double position, double velocity, double torque,
  double mos_temperature_c = 35.0, int status = 0x0001, double voltage_v = 24.0,
  double coil_temperature_c = 33.0);
State UnpackState(const std::vector<uint8_t> & data);

}  // namespace upper_body_canfd_hardware::rp
