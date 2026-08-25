#include "humanoid_gazebo/motor_sim/plant_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace humanoid_gazebo::motor_sim {

MotorPlantModel::MotorPlantModel(PlantConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.kp) || config_.kp < 0.0 ||
      !std::isfinite(config_.kd) || config_.kd < 0.0 ||
      !std::isfinite(config_.effort_limit_nm) ||
      config_.effort_limit_nm <= 0.0 ||
      !std::isfinite(config_.torque_constant_nm_per_a) ||
      config_.torque_constant_nm_per_a < 0.0 ||
      !std::isfinite(config_.current_limit_a) ||
      config_.current_limit_a < 0.0) {
    throw std::invalid_argument("invalid motor plant parameters");
  }
}

PlantOutput MotorPlantModel::step(const protocol::MotorCommand &command,
                                  const JointSample &state,
                                  const double dt_s) const {
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument("plant time step must be positive and finite");
  }
  if (!command.enabled) {
    return {};
  }
  double kp = command.use_nominal_gains ? config_.kp : command.kp;
  double kd = command.use_nominal_gains ? config_.kd : command.kd;
  if (command.mode == protocol::CommandMode::Velocity) {
    kp = 0.0;
  }
  const double effort =
      kp * (command.position - state.position_rad) +
      kd * (command.velocity - state.velocity_rad_s) + command.feedforward;
  return {std::clamp(effort, -config_.effort_limit_nm,
                     config_.effort_limit_nm)};
}

}  // namespace humanoid_gazebo::motor_sim
