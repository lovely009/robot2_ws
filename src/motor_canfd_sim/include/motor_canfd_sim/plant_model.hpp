#pragma once

#include "motor_canfd_drive/motor_types.hpp"
#include "motor_canfd_drive/motor_protocol.hpp"
#include "motor_canfd_drive/protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace motor_canfd_sim {

inline double clamp_symmetric(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

inline double motor_parameter_as_double(const motor_canfd_drive::MotorConfig &motor,
                                        const std::string &key) {
  const auto it = motor.parameters.find(key);
  if (it == motor.parameters.end()) {
    throw std::runtime_error("motor '" + motor.name + "' is missing parameter '" +
                             key + "'");
  }
  std::size_t parsed = 0U;
  const double value = std::stod(it->second, &parsed);
  if (parsed != it->second.size() || !std::isfinite(value)) {
    throw std::runtime_error("motor '" + motor.name + "' parameter '" + key +
                             "' is invalid");
  }
  return value;
}

// 仿真采样：位置/速度来自 Gazebo joint state，effort 是 plant 计算出的扭矩输出。
struct JointSample {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

// 单台虚拟电机的运行状态，由 plant 与 CAN 帧处理共同维护。
struct SimMotorState {
  bool enabled{false};
  motor_canfd_drive::protocol::JointCommand command{};
  std::chrono::steady_clock::time_point last_command{};
  motor_canfd_drive::protocol::WristMode mode{
      motor_canfd_drive::protocol::WristMode::Mit};
  double zero_offset{0.0};
  double current_a{0.0};
};

// 电机侧植物模型（纯数学，不依赖 ROS）：根据协议命令、使能状态和关节采样
// 计算输出扭矩，并维护 25/30 mm 电机的模式/置零状态。
class MotorPlantModel {
public:
  MotorPlantModel(const double command_timeout_s, const double rp_torque_limit_nm,
                  const double motor_2530_current_limit_a,
                  const double motor_2530_torque_limit_nm,
                  const double motor_2530_position_gain,
                  const double motor_2530_position_damping,
                  const double motor_2530_velocity_gain)
      : command_timeout_s_(command_timeout_s), rp_torque_limit_nm_(rp_torque_limit_nm),
        motor_2530_current_limit_a_(motor_2530_current_limit_a),
        motor_2530_torque_limit_nm_(motor_2530_torque_limit_nm),
        motor_2530_position_gain_(motor_2530_position_gain),
        motor_2530_position_damping_(motor_2530_position_damping),
        motor_2530_velocity_gain_(motor_2530_velocity_gain) {}

  // 处理 25/30 mm 控制帧：失能时更新模式/置零，使能状态与电流清零。
  void handle_motor_2530_control(SimMotorState &state,
                                 const motor_canfd_drive::protocol::WristControl &control,
                                 const JointSample &sample) {
    if (!control.run) {
      state.mode = control.mode;
      if (control.set_zero) {
        // “置零”只改变内部坐标偏置，不瞬移 Gazebo 关节。
        state.zero_offset = sample.position;
      }
    }
    state.enabled = control.run;
    if (!control.run) {
      state.current_a = 0.0;
    }
  }

  // 计算单台电机当前周期的输出扭矩（Nm）并更新 state.current_a。
  // 命令超时或失能时 fresh=false，扭矩自动回到 0。
  double compute_effort(const motor_canfd_drive::MotorConfig &config,
                        SimMotorState &state,
                        const JointSample &sample,
                        const std::chrono::steady_clock::time_point now) {
    const bool fresh =
        state.enabled && std::chrono::duration<double>(now - state.last_command)
                                 .count() <= command_timeout_s_;
    if (config.type == motor_canfd_drive::kRpMotorType) {
      // RP 按 MIT 公式：tau = Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff。
      double torque = 0.0;
      if (fresh) {
        torque = state.command.kp * (state.command.position - sample.position) +
                 state.command.kd * (state.command.velocity - sample.velocity) +
                 state.command.feedforward;
      }
      return clamp_symmetric(torque, rp_torque_limit_nm_);
    }

    // 25/30 mm 协议以电流为核心：先按模式得到 A，再通过 Kt 转成 Nm。
    double current = 0.0;
    if (fresh) {
      const auto relative_position = sample.position - state.zero_offset;
      if (state.mode == motor_canfd_drive::protocol::WristMode::Mit) {
        current =
            motor_canfd_drive::protocol::wrist_kp_to_current_gain(state.command.kp) *
                      (state.command.position - relative_position) +
                  motor_canfd_drive::protocol::wrist_kd_to_current_gain(
                      state.command.kd) *
                      (state.command.velocity - sample.velocity) +
                  state.command.feedforward;
      } else if (state.mode == motor_canfd_drive::protocol::WristMode::Position) {
        current = motor_2530_position_gain_ *
                      (state.command.position - relative_position) -
                  motor_2530_position_damping_ * sample.velocity;
      } else if (state.mode == motor_canfd_drive::protocol::WristMode::Velocity) {
        current =
            motor_2530_velocity_gain_ * (state.command.velocity - sample.velocity);
      }
    }
    state.current_a = clamp_symmetric(current, motor_2530_current_limit_a_);
    const auto torque_constant =
        motor_parameter_as_double(config, "torque_constant_nm_per_amp");
    return clamp_symmetric(state.current_a * torque_constant,
                           motor_2530_torque_limit_nm_);
  }

private:
  double command_timeout_s_;
  double rp_torque_limit_nm_;
  double motor_2530_current_limit_a_;
  double motor_2530_torque_limit_nm_;
  double motor_2530_position_gain_;
  double motor_2530_position_damping_;
  double motor_2530_velocity_gain_;
};

} // namespace motor_canfd_sim
