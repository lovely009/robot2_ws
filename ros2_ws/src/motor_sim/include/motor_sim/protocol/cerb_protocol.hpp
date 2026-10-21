#pragma once

#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::protocol {

// CERB 关节模组（巨蟹智能）MIT 模式子集：
// - 命令帧 0x110+ID，CAN-FD，9 字节有效载荷（线上按 DLC=12 填充），
//   使能/抱闸/清错位与模式半字节随帧携带，位置、速度、Kp、Kd 与前馈力矩
//   均为减速器输出轴侧物理量。
// - 反馈帧 0x300+ID，CAN-FD 12 字节，由命令帧触发（手册未定义周期上报）：
//   负载端位置 int16BE（±32768 对应 ±180°）、电机端转速 int16BE RPM、
//   Iq 电流 int16BE mA、故障码、线圈温度与状态位。
// SDO（0x600/0x580）、多控广播（0x200）与同步帧触发不在仿真范围内。
class CerbProtocol final : public MotorProtocol {
public:
  // mit_torque_limit_nm：MIT 帧 τ_ff 的 12 位量化满量程（N·m）。取自配置的
  // profile（厂家 2026-10-21：R58/R48 同为 300.0）；无参构造退回
  // kCerbDefaultMitTorqueLimitNm。
  explicit CerbProtocol(
      double mit_torque_limit_nm = kCerbDefaultMitTorqueLimitNm);

  void validate_motor_id(unsigned motor_id) const override;
  std::optional<MotorCommand>
  decode_command(const CanFrame &frame, unsigned motor_id) const override;
  CanFrame encode_feedback(const MotorFeedback &feedback,
                           FeedbackKind kind) const override;

private:
  double mit_torque_limit_nm_;
};

}  // namespace motor_sim::protocol
