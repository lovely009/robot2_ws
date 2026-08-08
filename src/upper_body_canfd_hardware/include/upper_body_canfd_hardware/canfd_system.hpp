#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace upper_body_canfd_hardware {
class SocketCan;

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
  on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  bool send_enable(bool enabled);
  bool send_commands();
  void receive_feedback();
  std::array<std::unique_ptr<SocketCan>, 2> buses_;
  std::array<std::string, 2> interfaces_{{"can0", "can1"}};
  std::vector<double> position_, velocity_, effort_, command_;
  double rp_kp_{30.0}, rp_kd_{1.0}, wrist_kp_{0.008}, wrist_kd_{1.0};
  double wrist_torque_constant_{0.35};
  double feedback_timeout_s_{0.1};
  bool active_{false};
  std::array<std::array<std::chrono::steady_clock::time_point, 7>, 2>
      last_feedback_{};
};
} // namespace upper_body_canfd_hardware
