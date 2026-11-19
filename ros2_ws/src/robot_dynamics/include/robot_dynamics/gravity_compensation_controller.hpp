#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_dynamics/arm_model.hpp"

namespace robot_dynamics {

class GravityCompensationController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Parameters {
    bool enabled{false};
    double effort_gain{1.0};
  };

  void zero_effort();
  controller_interface::InterfaceConfiguration interfaces(const std::string &type) const;

  std::vector<std::string> joints_;
  std::unique_ptr<ArmModel> model_;
  std::string model_error_;
  std::atomic<bool> configured_{false};
  std::mutex configuration_mutex_;
  bool active_{false};
  std::vector<std::size_t> torque_indices_;
  std::vector<double> positions_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> states_;
  realtime_tools::RealtimeBuffer<Parameters> parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace robot_dynamics
