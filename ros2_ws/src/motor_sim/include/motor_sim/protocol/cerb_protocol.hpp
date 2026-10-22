#pragma once

#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::protocol {

// CERB 关节模组（巨蟹智能）MIT 模式子集：
// - 单轴命令帧 0x110|ID（1..15），CAN-FD 12 字节 / DLC=9，
//   使能/抱闸/清错位与模式半字节随帧携带，位置、速度、Kp、Kd 与前馈力矩
//   均为减速器输出轴侧物理量。
// - 反馈帧 0x300+ID，CAN-FD 12 字节，由命令帧触发（手册未定义周期上报）：
//   负载端位置 int16BE（±32768 对应 ±180°）、电机端转速 int16BE RPM、
//   Iq 电流 int16BE（20 mA/码）、故障码、线圈温度与状态位。
// 多轴命令 0x210 为 64 字节，含六个 9 字节槽位与 NodeID 表（1..127）。
class CerbProtocol final : public MotorProtocol {
public:
  // mit_torque_limit_nm：MIT 帧 τ_ff 的 12 位量化满量程（N·m）。取自配置的
  // profile（厂家 2026-10-21：R58/R48 同为 300.0）；无参构造退回
  // kCerbDefaultMitTorqueLimitNm。
  explicit CerbProtocol(
      double mit_torque_limit_nm = kCerbDefaultMitTorqueLimitNm,
      double mit_position_limit_rad = 3.14,
      double mit_velocity_limit_rad_s = 3.14,
      double mit_kp_limit = 2500.0, double mit_kd_limit = 500.0);

  void validate_motor_id(unsigned motor_id) const override;
  std::optional<MotorCommand>
  decode_command(const CanFrame &frame, unsigned motor_id) const override;
  CanFrame encode_feedback(const MotorFeedback &feedback,
                           FeedbackKind kind) const override;

private:
  double mit_torque_limit_nm_;
  double mit_position_limit_rad_;
  double mit_velocity_limit_rad_s_;
  double mit_kp_limit_;
  double mit_kd_limit_;
};

}  // namespace motor_sim::protocol
