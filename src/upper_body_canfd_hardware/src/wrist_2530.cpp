#include "upper_body_canfd_hardware/wrist_2530.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace upper_body_canfd_hardware::wrist {

namespace {

int64_t SignedToBiased(int64_t value, int bits)
{
  const int64_t limit = int64_t{1} << (bits - 1);
  value = std::min(std::max(value, -limit), limit - 1);
  return value + limit;
}

int64_t BiasedToSigned(int64_t value, int bits)
{
  return value - (int64_t{1} << (bits - 1));
}

uint32_t PositionToCode(double position_rad)
{
  int64_t ticks = static_cast<int64_t>(std::llround(
    position_rad / (2.0 * M_PI) * static_cast<double>(kTicksPerRevolution)));
  ticks = std::min(std::max(ticks, -kPositionTickLimit), kPositionTickLimit);
  return static_cast<uint32_t>(ticks + 0x80000000);
}

double CodeToPosition(uint32_t code)
{
  const int64_t ticks = static_cast<int64_t>(code) - 0x80000000;
  return static_cast<double>(ticks) / static_cast<double>(kTicksPerRevolution) *
    (2.0 * M_PI);
}

uint16_t VelocityToCode(double velocity_rad_s)
{
  const double rpm = velocity_rad_s * 60.0 / (2.0 * M_PI);
  int32_t raw = static_cast<int32_t>(std::llround(rpm * 10.0));
  raw = std::min(std::max(raw, -kVelocityCodeLimit), kVelocityCodeLimit);
  return static_cast<uint16_t>(raw + 0x8000);
}

double CodeToVelocity(uint16_t code)
{
  const double rpm = static_cast<double>(BiasedToSigned(code, 16)) / 10.0;
  return rpm * (2.0 * M_PI) / 60.0;
}

uint16_t CurrentToCode(double current_a)
{
  const int64_t raw = static_cast<int64_t>(
    std::llround(current_a / kCurrentScaleA));
  return static_cast<uint16_t>(SignedToBiased(raw, 16));
}

double CodeToCurrent(uint16_t code)
{
  return static_cast<double>(BiasedToSigned(code, 16)) * kCurrentScaleA;
}

uint16_t GainToCode(double gain, double scale)
{
  int64_t raw = static_cast<int64_t>(std::llround(gain / scale));
  raw = std::min(std::max(raw, int64_t{0}), int64_t{0xFFFF});
  return static_cast<uint16_t>(raw);
}

}  // namespace

void ValidateNode(int node)
{
  if (node < 1 || node > 60) {
    throw std::invalid_argument("25/30 mm controller N must be in [1, 60]");
  }
}

uint32_t MitCommandId(int node)
{
  ValidateNode(node);
  return 0x501 + 4 * node;
}

uint32_t MotorControlId(int node)
{
  ValidateNode(node);
  return 0x502 + 4 * node;
}

uint32_t MitReplyId(int node)
{
  ValidateNode(node);
  return 0x701 + 4 * node;
}

uint32_t StatusId(int node)
{
  ValidateNode(node);
  return 0x702 + 4 * node;
}

std::vector<uint8_t> PackMitCommand(const MitCommand & command)
{
  const uint32_t p = PositionToCode(command.position);
  const uint16_t v = VelocityToCode(command.velocity);
  const uint16_t kp = GainToCode(command.kp, kKpScale);
  const uint16_t kd = GainToCode(command.kd, kKdScale);
  const uint16_t current = CurrentToCode(command.current_ff);
  std::vector<uint8_t> out;
  out.reserve(12);
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<uint8_t>((p >> (8 * i)) & 0xFF));
  }
  for (int i = 0; i < 2; ++i) {
    out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
  for (int i = 0; i < 2; ++i) {
    out.push_back(static_cast<uint8_t>((kp >> (8 * i)) & 0xFF));
  }
  for (int i = 0; i < 2; ++i) {
    out.push_back(static_cast<uint8_t>((kd >> (8 * i)) & 0xFF));
  }
  for (int i = 0; i < 2; ++i) {
    out.push_back(static_cast<uint8_t>((current >> (8 * i)) & 0xFF));
  }
  return out;
}

MitCommand UnpackMitCommand(const std::vector<uint8_t> & data)
{
  if (data.size() != 12) {
    throw std::invalid_argument("25/30 mm MIT command must be exactly 12 bytes");
  }
  uint32_t p = 0;
  uint16_t v = 0;
  uint16_t kp = 0;
  uint16_t kd = 0;
  uint16_t current = 0;
  for (int i = 0; i < 4; ++i) {
    p |= static_cast<uint32_t>(data[i]) << (8 * i);
  }
  for (int i = 0; i < 2; ++i) {
    v |= static_cast<uint16_t>(data[4 + i]) << (8 * i);
    kp |= static_cast<uint16_t>(data[6 + i]) << (8 * i);
    kd |= static_cast<uint16_t>(data[8 + i]) << (8 * i);
    current |= static_cast<uint16_t>(data[10 + i]) << (8 * i);
  }
  return MitCommand{
    CodeToPosition(p),
    CodeToVelocity(v),
    static_cast<double>(kp) * kKpScale,
    static_cast<double>(kd) * kKdScale,
    CodeToCurrent(current),
  };
}

std::vector<uint8_t> PackMotorControl(bool run, bool set_zero, uint8_t mode)
{
  if (mode != kModeMit && mode != kModeVelocity && mode != kModePosition) {
    throw std::invalid_argument("mode must be MIT(0), velocity(1), or position(2)");
  }
  return {static_cast<uint8_t>(run ? 1 : 0), static_cast<uint8_t>(set_zero ? 1 : 0),
          mode, 0, 0, 0, 0, 0};
}

std::vector<uint8_t> PackMitReply(
  int node, double position, double velocity, double current)
{
  const uint16_t command_id = static_cast<uint16_t>(MitCommandId(node));
  const uint32_t p = PositionToCode(position);
  const uint16_t v = VelocityToCode(velocity);
  const uint16_t i = CurrentToCode(current);
  std::vector<uint8_t> out;
  out.reserve(12);
  out.push_back(static_cast<uint8_t>(command_id & 0xFF));
  out.push_back(static_cast<uint8_t>((command_id >> 8) & 0xFF));
  for (int byte = 0; byte < 4; ++byte) {
    out.push_back(static_cast<uint8_t>((p >> (8 * byte)) & 0xFF));
  }
  for (int byte = 0; byte < 2; ++byte) {
    out.push_back(static_cast<uint8_t>((v >> (8 * byte)) & 0xFF));
  }
  for (int byte = 0; byte < 2; ++byte) {
    out.push_back(static_cast<uint8_t>((i >> (8 * byte)) & 0xFF));
  }
  out.push_back(0x00);
  out.push_back(0x00);
  return out;
}

Reply UnpackMitReply(const std::vector<uint8_t> & data)
{
  if (data.size() != 12) {
    throw std::invalid_argument("25/30 mm MIT reply must be exactly 12 bytes");
  }
  uint16_t command_id = 0;
  uint32_t p = 0;
  uint16_t v = 0;
  uint16_t i = 0;
  command_id |= static_cast<uint16_t>(data[0]);
  command_id |= static_cast<uint16_t>(data[1]) << 8;
  for (int byte = 0; byte < 4; ++byte) {
    p |= static_cast<uint32_t>(data[2 + byte]) << (8 * byte);
  }
  for (int byte = 0; byte < 2; ++byte) {
    v |= static_cast<uint16_t>(data[6 + byte]) << (8 * byte);
  }
  for (int byte = 0; byte < 2; ++byte) {
    i |= static_cast<uint16_t>(data[8 + byte]) << (8 * byte);
  }
  return Reply{command_id, CodeToPosition(p), CodeToVelocity(v), CodeToCurrent(i)};
}

std::vector<uint8_t> PackStatus(
  int node, double position, double velocity, double current,
  double voltage_v, uint32_t fault, uint8_t motor_state,
  uint8_t calibration_state, uint8_t mode, uint8_t motor_type)
{
  const uint16_t command_id = static_cast<uint16_t>(MitCommandId(node));
  const uint16_t voltage_code = static_cast<uint16_t>(
    SignedToBiased(static_cast<int64_t>(std::llround(voltage_v * 10.0)), 16));
  std::vector<uint8_t> out;
  out.reserve(20);
  out.push_back(static_cast<uint8_t>(command_id & 0xFF));
  out.push_back(static_cast<uint8_t>((command_id >> 8) & 0xFF));
  const uint32_t p = PositionToCode(position);
  for (int byte = 0; byte < 4; ++byte) {
    out.push_back(static_cast<uint8_t>((p >> (8 * byte)) & 0xFF));
  }
  const uint16_t v = VelocityToCode(velocity);
  const uint16_t i = CurrentToCode(current);
  for (int byte = 0; byte < 2; ++byte) {
    out.push_back(static_cast<uint8_t>((v >> (8 * byte)) & 0xFF));
  }
  for (int byte = 0; byte < 2; ++byte) {
    out.push_back(static_cast<uint8_t>((i >> (8 * byte)) & 0xFF));
  }
  for (int byte = 0; byte < 2; ++byte) {
    out.push_back(static_cast<uint8_t>((voltage_code >> (8 * byte)) & 0xFF));
  }
  for (int byte = 0; byte < 4; ++byte) {
    out.push_back(static_cast<uint8_t>((fault >> (8 * byte)) & 0xFF));
  }
  out.push_back(motor_state);
  out.push_back(calibration_state);
  out.push_back(mode);
  out.push_back(motor_type);
  return out;
}

Status UnpackStatus(const std::vector<uint8_t> & data)
{
  if (data.size() != 20) {
    throw std::invalid_argument("25/30 mm status frame must be exactly 20 bytes");
  }
  uint32_t p = 0;
  uint16_t v = 0;
  uint16_t i = 0;
  uint16_t voltage = 0;
  uint32_t fault = 0;
  for (int byte = 0; byte < 4; ++byte) {
    p |= static_cast<uint32_t>(data[2 + byte]) << (8 * byte);
  }
  for (int byte = 0; byte < 2; ++byte) {
    v |= static_cast<uint16_t>(data[6 + byte]) << (8 * byte);
    i |= static_cast<uint16_t>(data[8 + byte]) << (8 * byte);
    voltage |= static_cast<uint16_t>(data[10 + byte]) << (8 * byte);
  }
  for (int byte = 0; byte < 4; ++byte) {
    fault |= static_cast<uint32_t>(data[12 + byte]) << (8 * byte);
  }
  return Status{
    CodeToPosition(p),
    CodeToVelocity(v),
    CodeToCurrent(i),
    static_cast<double>(BiasedToSigned(voltage, 16)) / 10.0,
    fault,
    data[16],
    data[17],
    data[18],
    data[19],
  };
}

double KpToCurrentGain(double kp_protocol)
{
  return kp_protocol / 0.001115;
}

double KdToCurrentGain(double kd_protocol)
{
  return kd_protocol / 85.636;
}

}  // namespace upper_body_canfd_hardware::wrist
