#pragma once

#include "motor_sim/core/config.hpp"

namespace motor_sim::core {

class Transmission {
public:
  explicit Transmission(TransmissionConfig config);

  double joint_to_motor_position(double joint_position_rad) const;
  double motor_to_joint_position(double motor_position_rad) const;
  double joint_to_motor_velocity(double joint_velocity_rad_s) const;
  double motor_to_joint_velocity(double motor_velocity_rad_s) const;
  double motor_torque_to_joint_effort(double motor_torque_nm) const;
  double joint_effort_to_motor_torque(double joint_effort_nm) const;
  void set_zero_at_joint(double joint_position_rad);

private:
  TransmissionConfig config_;
};

}  // namespace motor_sim::core
