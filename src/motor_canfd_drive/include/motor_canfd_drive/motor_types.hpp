#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace motor_canfd_drive {

// These structures describe the physical CAN topology only.  They deliberately
// contain no ROS 2 concepts (node, topic, controller, arm, etc.).
using MotorParameterMap = std::unordered_map<std::string, std::string>;

struct BusConfig {
  std::string name;
  std::string interface_name;
};

struct MotorConfig {
  std::string name;
  std::string joint_name;
  std::string bus;
  std::string type;
  unsigned id{0U};
  MotorParameterMap parameters;
};

struct DriverConfig {
  std::vector<BusConfig> buses;
  std::vector<MotorConfig> motors;
};

} // namespace motor_canfd_drive
