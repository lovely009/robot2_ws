#pragma once
#include <functional>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "motor_hardware_interface/position_profile.hpp"

namespace motor_hardware_interface {
class JointFollowController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  std::vector<std::string> joints_;
  std::vector<PositionProfile> profiles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> positions_, velocities_, efforts_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> measured_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> target_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  int64_t activation_stamp_{0}, target_stamp_{0};
  bool active_{false}, faulted_{false};
};
}  // namespace motor_hardware_interface
