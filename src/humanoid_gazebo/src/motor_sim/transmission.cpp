#include "humanoid_gazebo/motor_sim/transmission.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace humanoid_gazebo::motor_sim {

Transmission::Transmission(TransmissionConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.gear_ratio) || config_.gear_ratio <= 0.0 ||
      !std::isfinite(config_.efficiency) || config_.efficiency <= 0.0 ||
      config_.efficiency > 1.0 ||
      (config_.direction != -1.0 && config_.direction != 1.0) ||
      !std::isfinite(config_.zero_offset_rad)) {
    throw std::invalid_argument("invalid transmission parameters");
  }
}

double Transmission::joint_to_motor_position(
    const double joint_position_rad) const {
  return config_.direction * config_.gear_ratio *
         (joint_position_rad - config_.zero_offset_rad);
}

double Transmission::motor_to_joint_position(
    const double motor_position_rad) const {
  return config_.zero_offset_rad +
         config_.direction * motor_position_rad / config_.gear_ratio;
}

double Transmission::joint_to_motor_velocity(
    const double joint_velocity_rad_s) const {
  return config_.direction * config_.gear_ratio * joint_velocity_rad_s;
}

double Transmission::motor_to_joint_velocity(
    const double motor_velocity_rad_s) const {
  return config_.direction * motor_velocity_rad_s / config_.gear_ratio;
}

double Transmission::motor_torque_to_joint_effort(
    const double motor_torque_nm) const {
  return config_.direction * motor_torque_nm * config_.gear_ratio *
         config_.efficiency;
}

double Transmission::joint_effort_to_motor_torque(
    const double joint_effort_nm) const {
  return config_.direction * joint_effort_nm /
         (config_.gear_ratio * config_.efficiency);
}

void Transmission::set_zero_at_joint(const double joint_position_rad) {
  if (!std::isfinite(joint_position_rad)) {
    throw std::invalid_argument("joint zero must be finite");
  }
  config_.zero_offset_rad = joint_position_rad;
}

}  // namespace humanoid_gazebo::motor_sim
