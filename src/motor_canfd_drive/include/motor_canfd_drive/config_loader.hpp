#pragma once

#include <string>

#include "motor_canfd_drive/motor_types.hpp"

namespace motor_canfd_drive {

// Load only the transport/motor mapping from the shared YAML. ROS-specific
// timing and controller gains remain the responsibility of the ROS adapter.
DriverConfig load_driver_config_yaml(const std::string &path,
                                     const std::string &root_key = "motor_hardware");

} // namespace motor_canfd_drive
