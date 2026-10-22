#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

#include "motor_sim/protocol/can_frame.hpp"

namespace motor_sim::protocol {

constexpr double kPi = 3.14159265358979323846;
constexpr int32_t kVaxorTicksPerRevolution = 2'097'152;
constexpr int32_t kVaxorPositionTickLimit = 0x7cffffff;
constexpr int32_t kVaxorVelocityCodeLimit = 30'000;
constexpr double kVaxorCurrentScaleA = 11.0 / 4096.0;
constexpr double kVaxorKpScale = 0.00001;
constexpr double kVaxorKdScale = 0.01;

// CERB MIT 帧 τ_ff 的 12 位量化满量程（N·m）：厂家 2026-10-21 修正为 300.0，
// R58/R48 同值（与真机参数页一致），**不是**型号库的机械峰值 peak_torque_nm。
// 此默认值只服务未传逐台量程的旧调用与无参构造。
constexpr double kCerbDefaultMitTorqueLimitNm = 300.0;

// CERB 反馈帧 [4..5]（Iq）每码对应的电流（A）。真机 2026-10-21 扫描反推 20.0 mA/码；
// 模拟器的编码端必须用同一刻度，否则假电机的电流与真机差 20 倍。
constexpr double kCerbCurrentCodeA = 0.020;

enum class CommandMode : uint8_t {
  Mit = 0x00,
  Velocity = 0x01,
  Position = 0x02,
  Enable = 0x10,
  Disable = 0x11,
  SetZero = 0x12,
};

enum class FeedbackKind { Reply, Status };

struct MotorCommand {
  unsigned motor_id{0U};
  CommandMode mode{CommandMode::Mit};
  bool enabled{false};
  bool set_zero{false};
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  // CERB 中单位为 Nm，Vaxor 中单位为 A；传动/plant 层按电机类型解释。
  double feedforward{0.0};
  // 仿真内部标志：协议未携带增益时使用 motors.yaml 的名义 plant 增益。
  bool use_nominal_gains{false};
};

struct MotorFeedback {
  unsigned motor_id{0U};
  double position{0.0};
  double velocity{0.0};
  // 电机侧电流（A）——两种协议同义，各自编码成线上字段。
  double effort{0.0};
  bool enabled{false};
  CommandMode mode{CommandMode::Mit};
};

class MotorProtocol {
public:
  virtual ~MotorProtocol() = default;
  virtual void validate_motor_id(unsigned motor_id) const = 0;
  virtual std::optional<MotorCommand>
  decode_command(const CanFrame &frame, unsigned motor_id) const = 0;
  virtual CanFrame encode_feedback(const MotorFeedback &feedback,
                                   FeedbackKind kind) const = 0;
};

// mit_torque_limit_nm 是 CERB MIT τ_ff 的 12 位量化满量程（逐台/逐型号，单位 N·m），
// 必须与真机编码端一致；Vaxor 忽略该参数。缺省值仅为兼容既有调用。
std::unique_ptr<MotorProtocol>
make_motor_protocol(std::string_view name,
                    double mit_torque_limit_nm = kCerbDefaultMitTorqueLimitNm,
                    double mit_position_limit_rad = 3.14,
                    double mit_velocity_limit_rad_s = 3.14,
                    double mit_kp_limit = 2500.0, double mit_kd_limit = 500.0);

uint32_t float_to_uint(double value, double low, double high, unsigned bits);
double uint_to_float(uint32_t value, double low, double high, unsigned bits);

void validate_vaxor_node(unsigned node);
uint32_t vaxor_command_id(unsigned node);
uint32_t vaxor_control_id(unsigned node);
uint32_t vaxor_reply_id(unsigned node);
uint32_t vaxor_status_id(unsigned node);

std::array<uint8_t, 12> pack_vaxor_command(const MotorCommand &command);
MotorCommand unpack_vaxor_command(const uint8_t *data, std::size_t size);
std::array<uint8_t, 8> pack_vaxor_control(bool run, bool set_zero,
                                          CommandMode mode);
MotorCommand unpack_vaxor_control(const uint8_t *data, std::size_t size);
std::array<uint8_t, 12> pack_vaxor_reply(const MotorFeedback &feedback);
std::array<uint8_t, 20> pack_vaxor_status(const MotorFeedback &feedback);
MotorFeedback unpack_vaxor_feedback(const uint8_t *data, std::size_t size);

}  // namespace motor_sim::protocol
