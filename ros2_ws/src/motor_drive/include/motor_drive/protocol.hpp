#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace motor_drive::protocol {

constexpr double kPi = 3.14159265358979323846;

// 25/30 mm 协议码值比例，依据《25-30mm 驱控一体使用说明手册》V20260204
// §3.1（MIT 发送）/ §3.2（回复）/ §3.3（状态）的码值定义。
// 所有单位换算集中在 protocol.cpp，上层统一使用 rad、rad/s、A 或 Nm，
// 避免协议单位渗透到 ROS 控制逻辑。
constexpr int32_t kWristTicksPerRevolution = 2'097'152;
constexpr int32_t kWristPositionTickLimit = 0x7cffffff;
constexpr int32_t kWristVelocityCodeLimit = 30'000;
constexpr double kWristCurrentScaleA = 11.0 / 4096.0;
constexpr double kWristKpScale = 0.00001;
constexpr double kWristKdScale = 0.01;

enum class WristMode : uint8_t { Mit = 0x00, Velocity = 0x01, Position = 0x02 };

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

// CREB MIT 量化范围必须由配置明确给出，并与电机参数设置页保持一致。
struct CrebMitLimits {
  double position_rad;
  double velocity_rad_s;
  double torque_nm;
};

struct WristControl {
  bool run{false};
  bool set_zero{false};
  WristMode mode{WristMode::Mit};
};

inline void validate_wrist_node(const unsigned node) {
  if (node < 1U || node > 60U) {
    throw std::out_of_range("25/30 mm controller node must be in [1, 60]");
  }
}

inline uint32_t wrist_command_id(const unsigned node) {
  validate_wrist_node(node);
  return 0x501U + 4U * node;
}

inline uint32_t wrist_control_id(const unsigned node) {
  validate_wrist_node(node);
  return 0x502U + 4U * node;
}

inline uint32_t wrist_reply_id(const unsigned node) {
  validate_wrist_node(node);
  return 0x701U + 4U * node;
}

inline uint32_t wrist_status_id(const unsigned node) {
  validate_wrist_node(node);
  return 0x702U + 4U * node;
}

uint32_t float_to_uint(double value, double low, double high, unsigned bits);
double uint_to_float(uint32_t value, double low, double high, unsigned bits);

// 25/30 mm：命令、控制帧和反馈帧的字节编解码（帧格式见规格书 §3.1~§3.4）。
// 注意：腕部反馈中的 effort 字段在这一层暂存”电流 A”，转换 Nm 由对应协议策略完成。
std::array<uint8_t, 12> pack_wrist_command(const JointCommand &command);
JointCommand unpack_wrist_command(const uint8_t *data, std::size_t size);

std::array<uint8_t, 8> pack_wrist_control(bool run, bool set_zero = false,
                                           WristMode mode = WristMode::Mit);
WristControl unpack_wrist_control(const uint8_t *data, std::size_t size);

std::array<uint8_t, 12> pack_wrist_reply(unsigned node,
                                          const JointState &state_current_a);
std::array<uint8_t, 20> pack_wrist_status(
    unsigned node, const JointState &state_current_a, double voltage_v = 12.0,
    uint32_t fault = 0, uint8_t motor_state = 4,
    uint8_t calibration_state = 0, WristMode mode = WristMode::Mit,
    uint8_t motor_type = 0x01);
JointState unpack_wrist_feedback(const uint8_t *data, std::size_t size);

// CREB 自定义 PDO：MIT 数据区为 8 Byte，外层策略再添加 1 Byte 控制字。
std::array<uint8_t, 8>
pack_creb_mit_command(const JointCommand &command,
                      const CrebMitLimits &limits);

// CREB 反馈在线协议中的 effort 是 Iq（A），由协议策略换算成输出端 Nm。
JointState unpack_creb_feedback(const uint8_t *data, std::size_t size);

double wrist_kp_to_current_gain(double protocol_kp);
double wrist_kd_to_current_gain(double protocol_kd);

} // namespace motor_drive::protocol
