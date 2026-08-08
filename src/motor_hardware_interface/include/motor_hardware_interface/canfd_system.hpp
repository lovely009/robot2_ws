#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor_canfd_drive/arm_driver.hpp"
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
  struct RuntimeConfig {
    std::array<motor_canfd_drive::ArmDriverConfig, 2> arms;
    std::array<std::array<std::string, 7>, 2> joint_names;
    double rp_kp{30.0};
    double rp_kd{1.0};
    double wrist_kp{0.008};
    double wrist_kd{1.0};
    double feedback_timeout_s{0.1};
    double initial_feedback_wait_s{0.1};
  };

  bool load_config(const std::string &path, std::string &error);
  bool validate_hardware_info(std::string &error) const;
  bool send_position_commands(bool zero_gains = false);
  void receive_feedback(std::array<bool, 14> *observed = nullptr);
  void close_buses() noexcept;

  RuntimeConfig config_;
  std::array<std::unique_ptr<motor_canfd_drive::ArmDriver>, 2> drivers_;
  std::array<motor_canfd_drive::ArmDriver::StateArray, 2> arm_states_{};
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;
  std::array<std::chrono::steady_clock::time_point, 14> last_feedback_{};
  bool active_{false};
};

} // namespace motor_hardware_interface
