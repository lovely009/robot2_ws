#include "robot_arm_gazebo/topology.hpp"

namespace robot_arm_gazebo {

const motor_sim::core::TopologyContract &robot_arm_topology_contract() {
  static const motor_sim::core::TopologyContract contract{
      {"arm"},
      {{"shoulder_pan_joint", "arm", "rp", 1U},
       {"shoulder_lift_joint", "arm", "rp", 2U},
       {"elbow_joint", "arm", "rp", 3U},
       {"wrist_1_joint", "arm", "motor_2530", 4U},
       {"wrist_2_joint", "arm", "motor_2530", 5U},
       {"wrist_3_joint", "arm", "motor_2530", 6U}}};
  return contract;
}

}  // namespace robot_arm_gazebo
