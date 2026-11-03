#include "motor_hardware_interface/joint_follow_controller.hpp"
#include <set>
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace motor_hardware_interface {
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

Callback JointFollowController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("max_velocity", {});
    parameter_callback_ = get_node()->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (parameter.get_name() == "joints" || parameter.get_name() == "max_velocity") {
            result.successful = false;
            result.reason = "follow joints and velocity limits are immutable";
          }
        }
        return result;
      });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback JointFollowController::on_configure(const rclcpp_lifecycle::State &) {
  try {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    const auto limits = get_node()->get_parameter("max_velocity").as_double_array();
    const std::set<std::string> names(joints_.begin(), joints_.end());
    if (joints_.empty() || names.count("") || names.size() != joints_.size() || limits.size() != joints_.size()) {
      throw std::invalid_argument("follow joints and velocity limits must match and be unique");
    }
    profiles_.clear();
    for (const auto velocity : limits) profiles_.emplace_back(velocity, 2 * velocity);
    subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("~/target", 1,
      [this](sensor_msgs::msg::JointState::SharedPtr message) {
        if (message->name != joints_ || message->position.size() != joints_.size() ||
            !message->velocity.empty() || !message->effort.empty() ||
            message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000) return;
        for (const auto value : message->position) if (!std::isfinite(value)) return;
        target_.writeFromNonRT(message);
      });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

controller_interface::InterfaceConfiguration JointFollowController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto &name : joints_) for (const auto *channel : {"position", "velocity"}) {
    result.names.push_back(name + "/" + channel);
  }
  return result;
}

controller_interface::InterfaceConfiguration JointFollowController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto &name : joints_) result.names.push_back(name + "/position");
  return result;
}

Callback JointFollowController::on_activate(const rclcpp_lifecycle::State &) {
  active_ = false;
  positions_.clear(); velocities_.clear(); measured_.clear();
  if (command_interfaces_.size() != 2 * joints_.size() || state_interfaces_.size() != joints_.size() ||
      !controller_interface::get_ordered_interfaces(command_interfaces_, joints_, "position", positions_) ||
      !controller_interface::get_ordered_interfaces(command_interfaces_, joints_, "velocity", velocities_) ||
      !controller_interface::get_ordered_interfaces(state_interfaces_, joints_, "position", measured_)) return Callback::ERROR;
  for (const auto &state : measured_) if (!std::isfinite(state.get().get_value())) return Callback::ERROR;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    profiles_[i].reset(measured_[i].get().get_value());
    positions_[i].get().set_value(profiles_[i].position());
    velocities_[i].get().set_value(0);
  }
  // DDS may still deliver an inactive-period sample after activation.
  activation_stamp_ = get_node()->now().nanoseconds();
  target_stamp_ = 0;
  active_ = true; faulted_ = false;
  return Callback::SUCCESS;
}

Callback JointFollowController::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  for (auto &command : velocities_) command.get().set_value(0);
  return Callback::SUCCESS;
}

Return JointFollowController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
  if (faulted_) return Return::ERROR;
  if (!active_) return Return::OK;
  const double dt = period.seconds();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double measured = measured_[i].get().get_value();
    if (!std::isfinite(measured) || std::abs(measured - profiles_[i].position()) > .1 ||
        !std::isfinite(dt) || dt <= 0 || dt > .1) {
      faulted_ = true;
      RCLCPP_ERROR(get_node()->get_logger(), "follow feedback/path tolerance or update period invalid: %s", joints_[i].c_str());
      for (auto &command : velocities_) command.get().set_value(0);
      return Return::ERROR;
    }
  }
  const auto message = *target_.readFromRT();
  if (message) {
    const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    const auto age = time.nanoseconds() - stamp;
    if (stamp >= activation_stamp_ && stamp > target_stamp_ && age >= -50000000 && age <= 500000000) {
      for (std::size_t i = 0; i < joints_.size(); ++i) profiles_[i].target(message->position[i]);
      target_stamp_ = stamp;
    }
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    profiles_[i].advance(dt);
    positions_[i].get().set_value(profiles_[i].position());
    velocities_[i].get().set_value(profiles_[i].velocity());
  }
  return Return::OK;
}
}  // namespace motor_hardware_interface

PLUGINLIB_EXPORT_CLASS(motor_hardware_interface::JointFollowController, controller_interface::ControllerInterface)
