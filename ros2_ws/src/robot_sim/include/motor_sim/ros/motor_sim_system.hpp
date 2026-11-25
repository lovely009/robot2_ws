#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "motor_sim/core/plant_model.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_transmission/model.hpp"

namespace motor_sim::ros {

/// Isomorphic ros2_control SystemInterface backed by MotorPlantModel + robot_transmission.
class MotorSimSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotorSimSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

private:
  struct ActuatorSim {
    std::string name;
    std::unique_ptr<core::MotorPlantModel> plant;
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double kp{30.0};
    double kd{1.0};
    double effort_limit{36.0};
  };

  std::unique_ptr<robot_transmission::Model> model_;
  std::vector<ActuatorSim> actuators_;
  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> cmd_position_;
  std::vector<double> cmd_velocity_;
  std::vector<double> cmd_effort_;
  bool active_{false};
};

}  // namespace motor_sim::ros
