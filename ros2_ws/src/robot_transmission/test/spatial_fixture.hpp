#pragma once

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>

inline std::filesystem::path write_spatial_driver(
  const std::filesystem::path &directory, const std::array<double, 2> &signs)
{
  std::ofstream(directory / "driver.yaml") <<
    "motor_hardware:\n  transmissions_file: transmissions.yaml\n  motors:\n"
    "    arm_joint1: {id: 1, joint: Joint_Link1}\n"
    "    arm_joint2: {id: 2, joint: Joint_Link2}\n";
  std::ofstream out(directory / "transmissions.yaml");
  out << std::setprecision(17) <<
    "transmissions:\n  shoulder_linkage:\n"
    "    plugin: robot_transmission/SpatialLinkageTransmission\n"
    "    actuators: [arm_joint1, arm_joint2]\n    joints: [Joint_Link1, Joint_Link2]\n"
    "    actuator_reduction: [" << signs[0] << ", " << signs[1] << "]\n"
    "    parameters:\n"
    "      motor_axis_origin: '0.033699999952 0.04475 0.400'\n"
    "      motor_axis_direction: '1 0 0'\n"
    "      motor_ball_zero: '0.033699999952 0.061556509578 0.389158356406'\n"
    "      joint1_axis_origin: '-0.0025 0.06725 0.4825'\n"
    "      joint1_axis_direction: '0 0 1'\n"
    "      joint2_axis_origin: '0.0025 0.06775 0.5025'\n"
    "      joint2_axis_direction: '0 -1 0'\n"
    "      joint_ball_zero: '0.035840440737 0.06775 0.494319840327'\n"
    "      rod_length: '0.10536545216923923'\n";
  return directory / "driver.yaml";
}
