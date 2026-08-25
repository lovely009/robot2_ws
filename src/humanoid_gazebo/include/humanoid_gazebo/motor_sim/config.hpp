#pragma once

#include <string>
#include <vector>

namespace humanoid_gazebo::motor_sim {

struct TimingConfig {
  unsigned control_hz{100U};
  unsigned rp_feedback_hz{100U};
  unsigned vaxor_feedback_hz{50U};
  unsigned command_timeout_ms{150U};
};

struct BusConfig {
  std::string name;
};

struct TransmissionConfig {
  double gear_ratio{1.0};
  double efficiency{1.0};
  double direction{1.0};
  double zero_offset_rad{0.0};
};

struct JointLimits {
  double hard_lower_rad{0.0};
  double hard_upper_rad{0.0};
  double command_lower_rad{0.0};
  double command_upper_rad{0.0};
  double velocity_rad_s{0.0};
  double effort_nm{0.0};
};

struct PlantConfig {
  double kp{0.0};
  double kd{0.0};
  double effort_limit_nm{0.0};
  double torque_constant_nm_per_a{0.0};
  double current_limit_a{0.0};
};

struct MotorConfig {
  std::string joint;
  std::string bus;
  std::string protocol;
  unsigned id{0U};
  TransmissionConfig transmission;
  JointLimits limits;
  PlantConfig plant;
};

struct SystemConfig {
  TimingConfig timing;
  std::vector<BusConfig> buses;
  std::vector<MotorConfig> motors;
};

SystemConfig load_system_config(const std::string &path);

}  // namespace humanoid_gazebo::motor_sim
