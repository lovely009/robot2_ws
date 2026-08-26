#include "humanoid_gazebo/topology.hpp"

namespace humanoid_gazebo {

const ::motor_sim::core::TopologyContract &humanoid_topology_contract() {
  static const ::motor_sim::core::TopologyContract contract{
      {"left_arm", "right_arm", "legs"},
      {{"left_scapula_protraction_joint", "left_arm", "rp", 1U},
       {"left_scapula_elevation_joint", "left_arm", "rp", 2U},
       {"left_shoulder_pitch_joint", "left_arm", "rp", 3U},
       {"left_shoulder_roll_joint", "left_arm", "rp", 4U},
       {"left_shoulder_yaw_joint", "left_arm", "rp", 5U},
       {"left_elbow_pitch_joint", "left_arm", "rp", 6U},
       {"left_forearm_roll_joint", "left_arm", "motor_2530", 1U},
       {"left_wrist_flexion_joint", "left_arm", "motor_2530", 2U},
       {"left_wrist_deviation_joint", "left_arm", "motor_2530", 3U},
       {"right_scapula_protraction_joint", "right_arm", "rp", 1U},
       {"right_scapula_elevation_joint", "right_arm", "rp", 2U},
       {"right_shoulder_pitch_joint", "right_arm", "rp", 3U},
       {"right_shoulder_roll_joint", "right_arm", "rp", 4U},
       {"right_shoulder_yaw_joint", "right_arm", "rp", 5U},
       {"right_elbow_pitch_joint", "right_arm", "rp", 6U},
       {"right_forearm_roll_joint", "right_arm", "motor_2530", 1U},
       {"right_wrist_flexion_joint", "right_arm", "motor_2530", 2U},
       {"right_wrist_deviation_joint", "right_arm", "motor_2530", 3U},
       {"left_hip_yaw_joint", "legs", "rp", 1U},
       {"left_hip_roll_joint", "legs", "rp", 2U},
       {"left_hip_pitch_joint", "legs", "rp", 3U},
       {"left_knee_pitch_joint", "legs", "rp", 4U},
       {"left_ankle_pitch_joint", "legs", "motor_2530", 1U},
       {"left_ankle_roll_joint", "legs", "motor_2530", 2U},
       {"right_hip_yaw_joint", "legs", "rp", 5U},
       {"right_hip_roll_joint", "legs", "rp", 6U},
       {"right_hip_pitch_joint", "legs", "rp", 7U},
       {"right_knee_pitch_joint", "legs", "rp", 8U},
       {"right_ankle_pitch_joint", "legs", "motor_2530", 3U},
       {"right_ankle_roll_joint", "legs", "motor_2530", 4U}}};
  return contract;
}

}  // namespace humanoid_gazebo
