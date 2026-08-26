#pragma once

#include "humanoid_gazebo/motor_sim/config.hpp"
#include "motor_sim/protocol/motor_protocol.hpp"

namespace humanoid_gazebo::motor_sim {

namespace protocol = ::motor_sim::protocol;

struct JointSample {
  double position_rad{0.0};
  double velocity_rad_s{0.0};
};

struct PlantOutput {
  double effort_nm{0.0};
};

class MotorPlantModel {
public:
  explicit MotorPlantModel(PlantConfig config);
  PlantOutput step(const protocol::MotorCommand &command,
                   const JointSample &state, double dt_s) const;

private:
  PlantConfig config_;
};

}  // namespace humanoid_gazebo::motor_sim
