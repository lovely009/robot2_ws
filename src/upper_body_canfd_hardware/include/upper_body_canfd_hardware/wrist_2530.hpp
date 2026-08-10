// Codec for the 25/30 mm integrated drive CAN-FD protocol.
//
// Reference: "25-30mm驱控一体使用说明手册_20260204.pdf". Note the hardware
// configuration for this robot uses a 5 Mbps data phase (the manual says
// 4 Mbps); packet encoding is unchanged. Translation is byte-for-byte
// compatible with upper_body_canfd_gazebo/protocol_2530.py and is kept free
// of ROS dependencies.

#pragma once

#include <cstdint>
#include <vector>

namespace upper_body_canfd_hardware::wrist {

inline constexpr int64_t kTicksPerRevolution = 2'097'152;
inline constexpr int64_t kPositionTickLimit = 0x7CFFFFFF;
inline constexpr int32_t kVelocityCodeLimit = 30'000;
inline constexpr double kCurrentScaleA = 11.0 / 4096.0;
inline constexpr double kKpScale = 0.00001;
inline constexpr double kKdScale = 0.01;

inline constexpr uint8_t kModeMit = 0x00;
inline constexpr uint8_t kModeVelocity = 0x01;
inline constexpr uint8_t kModePosition = 0x02;

struct MitCommand {
  double position = 0.0;  // rad, multi-turn
  double velocity = 0.0;  // rad/s
  double kp = 0.0;        // protocol coefficient
  double kd = 0.0;        // protocol coefficient
  double current_ff = 0.0;  // A
};

struct Reply {
  uint32_t command_can_id = 0;
  double position = 0.0;
  double velocity = 0.0;
  double current = 0.0;
};

struct Status {
  double position = 0.0;
  double velocity = 0.0;
  double current = 0.0;
  double voltage_v = 0.0;
  uint32_t fault = 0;
  uint8_t motor_state = 0;
  uint8_t calibration_state = 0;
  uint8_t mode = 0;
  uint8_t motor_type = 0;
};

void ValidateNode(int node);

uint32_t MitCommandId(int node);
uint32_t MotorControlId(int node);
uint32_t MitReplyId(int node);
uint32_t StatusId(int node);

std::vector<uint8_t> PackMitCommand(const MitCommand & command);
MitCommand UnpackMitCommand(const std::vector<uint8_t> & data);

std::vector<uint8_t> PackMotorControl(
  bool run, bool set_zero = false, uint8_t mode = kModeMit);

std::vector<uint8_t> PackMitReply(
  int node, double position, double velocity, double current);
Reply UnpackMitReply(const std::vector<uint8_t> & data);

std::vector<uint8_t> PackStatus(
  int node, double position, double velocity, double current,
  double voltage_v = 12.0, uint32_t fault = 0, uint8_t motor_state = 4,
  uint8_t calibration_state = 0, uint8_t mode = kModeMit,
  uint8_t motor_type = 0x01);
Status UnpackStatus(const std::vector<uint8_t> & data);

double KpToCurrentGain(double kp_protocol);
double KdToCurrentGain(double kd_protocol);

}  // namespace upper_body_canfd_hardware::wrist
