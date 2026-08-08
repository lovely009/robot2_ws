#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor_canfd_drive/motor_canfd_driver.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motor_hardware_interface {

class CanFdSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanFdSystem)

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
  struct MotorCommandConfig {
    double kp{0.0};
    double kd{0.0};
  };

  struct RuntimeConfig {
    motor_canfd_drive::DriverConfig driver;
    std::vector<MotorCommandConfig> motor_commands;
    double feedback_timeout_s{0.1};
    double initial_feedback_wait_s{0.1};
  };

  bool load_config(const std::string &path, std::string &error);
  bool apply_bus_overrides(std::string &error);
  bool build_joint_mapping(std::string &error);
  bool send_position_commands(bool zero_gains = false);
  void receive_feedback(std::vector<bool> *observed = nullptr);
  void close_driver() noexcept;

  RuntimeConfig config_;
  std::unique_ptr<motor_canfd_drive::MotorCanfdDriver> driver_;
  std::vector<std::size_t> motor_to_joint_;
  std::vector<motor_canfd_drive::protocol::JointState> motor_states_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;
  std::vector<std::chrono::steady_clock::time_point> last_feedback_;
  bool active_{false};
};

} // namespace motor_hardware_interface
