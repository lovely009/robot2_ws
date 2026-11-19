#include "robot_dynamics/gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <set>
#include <stdexcept>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_dynamics {
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

CallbackReturn GravityCompensationController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("urdf_file", "");
    auto_declare<bool>("enabled", false);
    auto_declare<double>("effort_gain", 1.0);
    parameter_callback_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        const std::lock_guard<std::mutex> lock(configuration_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        try {
          Parameters next{get_node()->get_parameter("enabled").as_bool(),
                          get_node()->get_parameter("effort_gain").as_double()};
          for (const auto &parameter : parameters) {
            const auto &name = parameter.get_name();
            if (name == "enabled") {
              next.enabled = parameter.as_bool();
            } else if (name == "effort_gain") {
              next.effort_gain = parameter.as_double();
            } else if (configured_.load() && (name == "joints" || name == "urdf_file")) {
              throw std::invalid_argument("joints and urdf_file are immutable after configuration");
            }
          }
          if (!std::isfinite(next.effort_gain) || next.effort_gain < 0.0 || next.effort_gain > 1.0) {
            throw std::invalid_argument("effort_gain must be finite and within [0,1]");
          }
          if (next.enabled && configured_.load() && !model_error_.empty()) {
            throw std::invalid_argument(model_error_);
          }
          parameters_.writeFromNonRT(next);
          result.successful = true;
        } catch (const std::exception &error) {
          result.reason = error.what();
        }
        return result;
      });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::interfaces(const std::string &type) const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joints_) {
    result.names.push_back(joint + "/" + type);
  }
  return result;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const {
  return interfaces(hardware_interface::HW_IF_EFFORT);
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const {
  return interfaces(hardware_interface::HW_IF_POSITION);
}

CallbackReturn GravityCompensationController::on_configure(const rclcpp_lifecycle::State &) {
  active_ = false;
  try {
    const auto parameters_snapshot = get_node()->get_parameters(
      {"joints", "urdf_file", "enabled", "effort_gain"});
    const std::lock_guard<std::mutex> lock(configuration_mutex_);
    configured_.store(false);
    joints_ = parameters_snapshot[0].as_string_array();
    const std::set<std::string> names(joints_.begin(), joints_.end());
    if (joints_.empty() || names.size() != joints_.size() || names.count("")) {
      throw std::invalid_argument("joints must be nonempty and unique");
    }
    const Parameters parameters{parameters_snapshot[2].as_bool(),
                                parameters_snapshot[3].as_double()};
    if (!std::isfinite(parameters.effort_gain) || parameters.effort_gain < 0.0 ||
        parameters.effort_gain > 1.0) {
      throw std::invalid_argument("effort_gain must be finite and within [0,1]");
    }
    model_error_.clear();
    torque_indices_.clear();
    model_ = std::make_unique<ArmModel>(parameters_snapshot[1].as_string());
    for (const auto &name : model_->non_scalar_joint_names()) {
      model_error_ += "unsupported model joint: " + name + "; ";
    }
    const auto &model_joints = model_->joint_names();
    for (const auto &name : model_joints) {
      if (!names.count(name)) {
        model_error_ += "missing measured model joint: " + name + "; ";
      }
    }
    for (const auto &name : joints_) {
      const auto found = std::find(model_joints.begin(), model_joints.end(), name);
      if (found == model_joints.end()) {
        model_error_ += "controlled joint is not scalar in model: " + name + "; ";
      } else {
        torque_indices_.push_back(static_cast<std::size_t>(found - model_joints.begin()));
      }
    }
    if (parameters.enabled && !model_error_.empty()) {
      throw std::invalid_argument(model_error_);
    }
    positions_.assign(joints_.size(), 0.0);
    parameters_.writeFromNonRT(parameters);
    configured_.store(true);
    if (!model_error_.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "gravity disabled; enabling will be rejected: %s",
                  model_error_.c_str());
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void GravityCompensationController::zero_effort() {
  for (auto &command : command_interfaces_) {
    command.set_value(0.0);
  }
}

CallbackReturn GravityCompensationController::on_activate(const rclcpp_lifecycle::State &) {
  active_ = false;
  zero_effort();
  commands_.clear();
  states_.clear();
  if (!configured_.load() || command_interfaces_.size() != joints_.size() ||
      state_interfaces_.size() != joints_.size() ||
      !controller_interface::get_ordered_interfaces(
        command_interfaces_, joints_, hardware_interface::HW_IF_EFFORT, commands_) ||
      !controller_interface::get_ordered_interfaces(
        state_interfaces_, joints_, hardware_interface::HW_IF_POSITION, states_)) {
    return CallbackReturn::ERROR;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  zero_effort();
  commands_.clear();
  states_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_error(const rclcpp_lifecycle::State &state) {
  return on_deactivate(state);
}

Return GravityCompensationController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  zero_effort();
  if (!active_) {
    return Return::OK;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    positions_[i] = states_[i].get().get_value();
    if (!std::isfinite(positions_[i])) {
      active_ = false;
      return Return::ERROR;
    }
  }
  const auto parameters = *parameters_.readFromRT();
  if (!parameters.enabled) {
    return Return::OK;
  }
  try {
    if (!model_error_.empty()) {
      throw std::invalid_argument(model_error_);
    }
    model_->update_configuration(joints_, positions_);
    const auto torque = model_->gravity_torque();
    for (const auto index : torque_indices_) {
      if (!std::isfinite(torque.at(index))) {
        throw std::invalid_argument("nonfinite gravity torque");
      }
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      commands_[i].get().set_value(parameters.effort_gain * torque[torque_indices_[i]]);
    }
  } catch (const std::exception &) {
    active_ = false;
    zero_effort();
    return Return::ERROR;
  }
  return Return::OK;
}

}  // namespace robot_dynamics

PLUGINLIB_EXPORT_CLASS(robot_dynamics::GravityCompensationController,
                       controller_interface::ControllerInterface)
