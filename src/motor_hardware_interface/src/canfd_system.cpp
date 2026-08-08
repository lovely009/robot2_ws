#include "motor_hardware_interface/canfd_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motor_hardware_interface {
namespace {

template <std::size_t N>
std::array<unsigned, N> parse_unsigned_array(const YAML::Node &node,
                                             const char *name) {
  if (!node || !node.IsSequence() || node.size() != N) {
    throw std::runtime_error(std::string(name) + " must contain exactly " +
                             std::to_string(N) + " entries");
  }
  std::array<unsigned, N> values{};
  for (std::size_t index = 0; index < N; ++index) {
    values[index] = node[index].as<unsigned>();
  }
  return values;
}

template <std::size_t N>
std::array<std::string, N> parse_string_array(const YAML::Node &node,
                                              const char *name) {
  if (!node || !node.IsSequence() || node.size() != N) {
    throw std::runtime_error(std::string(name) + " must contain exactly " +
                             std::to_string(N) + " entries");
  }
  std::array<std::string, N> values{};
  for (std::size_t index = 0; index < N; ++index) {
    values[index] = node[index].as<std::string>();
  }
  return values;
}

bool has_state_interface(const hardware_interface::ComponentInfo &joint,
                         const std::string &name) {
  return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                     [&](const auto &interface) { return interface.name == name; });
}

} // namespace

bool CanFdSystem::load_config(const std::string &path, std::string &error) {
  try {
    if (path.empty()) {
      throw std::runtime_error("hardware parameter 'config_file' is required");
    }
    const auto root = YAML::LoadFile(path);
    const auto hardware = root["motor_hardware"];
    if (!hardware || !hardware.IsMap()) {
      throw std::runtime_error("YAML root 'motor_hardware' is required");
    }

    const auto timing = hardware["timing"];
    config_.feedback_timeout_s = timing["feedback_timeout_s"].as<double>();
    config_.initial_feedback_wait_s = timing["initial_feedback_wait_s"].as<double>();

    const auto gains = hardware["gains"];
    config_.rp_kp = gains["rp"]["kp"].as<double>();
    config_.rp_kd = gains["rp"]["kd"].as<double>();
    config_.wrist_kp = gains["wrist"]["kp"].as<double>();
    config_.wrist_kd = gains["wrist"]["kd"].as<double>();
    const auto wrist_torque_constant =
        gains["wrist"]["torque_constant_nm_per_amp"].as<double>();

    const std::array<std::string, 2> sides{{"left", "right"}};
    for (std::size_t arm = 0; arm < sides.size(); ++arm) {
      const auto arm_node = hardware["arms"][sides[arm]];
      config_.arms[arm].interface_name = arm_node["interface"].as<std::string>();
      config_.arms[arm].rp_ids =
          parse_unsigned_array<4>(arm_node["rp_ids"], "rp_ids");
      config_.arms[arm].wrist_nodes =
          parse_unsigned_array<3>(arm_node["wrist_nodes"], "wrist_nodes");
      config_.arms[arm].wrist_torque_constant_nm_per_amp = wrist_torque_constant;
      config_.joint_names[arm] =
          parse_string_array<7>(arm_node["joint_names"], "joint_names");
    }

    if (config_.rp_kp < 0.0 || config_.rp_kd < 0.0 ||
        config_.wrist_kp < 0.0 || config_.wrist_kd < 0.0 ||
        config_.feedback_timeout_s <= 0.0 ||
        config_.initial_feedback_wait_s <= 0.0 ||
        wrist_torque_constant <= 0.0) {
      throw std::runtime_error("gains, timeouts and torque constant must be valid positive values");
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

bool CanFdSystem::validate_hardware_info(std::string &error) const {
  if (info_.joints.size() != 14U) {
    error = "exactly 14 joints are required by the configured dual-arm hardware";
    return false;
  }
  for (std::size_t arm = 0; arm < 2U; ++arm) {
    for (std::size_t local = 0; local < 7U; ++local) {
      const auto index = arm * 7U + local;
      const auto &joint = info_.joints[index];
      if (joint.name != config_.joint_names[arm][local]) {
        error = "joint index " + std::to_string(index) + " must be '" +
                config_.joint_names[arm][local] + "', got '" + joint.name + "'";
        return false;
      }
      if (joint.command_interfaces.size() != 1U ||
          joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION) {
        error = "joint '" + joint.name + "' must expose one position command interface";
        return false;
      }
      if (joint.state_interfaces.size() != 3U ||
          !has_state_interface(joint, hardware_interface::HW_IF_POSITION) ||
          !has_state_interface(joint, hardware_interface::HW_IF_VELOCITY) ||
          !has_state_interface(joint, hardware_interface::HW_IF_EFFORT)) {
        error = "joint '" + joint.name +
                "' must expose position, velocity and effort state interfaces";
        return false;
      }
    }
  }
  return true;
}

hardware_interface::CallbackReturn
CanFdSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto config_it = info_.hardware_parameters.find("config_file");
  const std::string config_path = config_it == info_.hardware_parameters.end()
                                      ? std::string{}
                                      : config_it->second;
  std::string error;
  if (!load_config(config_path, error)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "invalid motor config '%s': %s", config_path.c_str(), error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto override_interface = [&](const char *parameter, const std::size_t arm) {
    const auto it = info_.hardware_parameters.find(parameter);
    if (it != info_.hardware_parameters.end() && !it->second.empty()) {
      config_.arms[arm].interface_name = it->second;
    }
  };
  override_interface("left_interface", 0U);
  override_interface("right_interface", 1U);

  if (!validate_hardware_info(error)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    drivers_[0] = std::make_unique<motor_canfd_drive::ArmDriver>(config_.arms[0]);
    drivers_[1] = std::make_unique<motor_canfd_drive::ArmDriver>(config_.arms[1]);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "invalid CAN-FD driver configuration: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_.assign(14U, std::numeric_limits<double>::quiet_NaN());
  velocity_.assign(14U, 0.0);
  effort_.assign(14U, 0.0);
  command_.assign(14U, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CanFdSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(42U);
  for (std::size_t index = 0; index < 14U; ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_POSITION, &position_[index]);
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_VELOCITY, &velocity_[index]);
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_EFFORT, &effort_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
CanFdSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(14U);
  for (std::size_t index = 0; index < 14U; ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_POSITION, &command_[index]);
  }
  return interfaces;
}

void CanFdSystem::close_buses() noexcept {
  for (auto &driver : drivers_) {
    if (driver) {
      driver->close();
    }
  }
}

hardware_interface::CallbackReturn
CanFdSystem::on_activate(const rclcpp_lifecycle::State &) {
  try {
    for (auto &driver : drivers_) {
      driver->open();
      if (!driver->set_enabled(false)) {
        throw std::runtime_error("failed to send initial disable command");
      }
    }

    // Query feedback with zero gains while all motors are disabled. This preserves
    // the existing safety rule: never invent q=0 and then enable against an unknown pose.
    std::array<bool, 14> observed{};
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.initial_feedback_wait_s);
    while (std::chrono::steady_clock::now() < deadline) {
      // Repeat the harmless zero-gain query during the startup window. Besides
      // tolerating normal bus/node startup skew, this keeps the activation path
      // deterministic with the vcan simulator without increasing real-hardware gains.
      if (!send_position_commands(true)) {
        throw std::runtime_error("failed to send zero-gain feedback query");
      }
      receive_feedback(&observed);
      if (std::all_of(observed.begin(), observed.end(), [](const bool seen) { return seen; })) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (std::size_t index = 0; index < observed.size(); ++index) {
      if (!observed[index] || !std::isfinite(position_[index])) {
        throw std::runtime_error("no initial feedback from joint '" +
                                 info_.joints[index].name + "'");
      }
      command_[index] = position_[index];
    }

    for (auto &driver : drivers_) {
      if (!driver->set_enabled(true)) {
        throw std::runtime_error("failed to enable one or more motors");
      }
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "activation failed: %s", exception.what());
    for (auto &driver : drivers_) {
      if (driver && driver->is_open()) {
        static_cast<void>(driver->set_enabled(false));
      }
    }
    close_buses();
    active_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn
CanFdSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  for (auto &driver : drivers_) {
    if (driver && driver->is_open()) {
      static_cast<void>(driver->set_enabled(false));
    }
  }
  close_buses();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CanFdSystem::send_position_commands(const bool zero_gains) {
  bool ok = true;
  for (std::size_t arm = 0; arm < 2U; ++arm) {
    motor_canfd_drive::ArmDriver::CommandArray commands{};
    for (std::size_t joint = 0; joint < 7U; ++joint) {
      commands[joint].position = command_[arm * 7U + joint];
      if (!zero_gains) {
        if (joint < 4U) {
          commands[joint].kp = config_.rp_kp;
          commands[joint].kd = config_.rp_kd;
        } else {
          commands[joint].kp = config_.wrist_kp;
          commands[joint].kd = config_.wrist_kd;
        }
      }
    }
    ok &= drivers_[arm]->write_commands(commands);
  }
  return ok;
}

void CanFdSystem::receive_feedback(std::array<bool, 14> *observed) {
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t arm = 0; arm < 2U; ++arm) {
    motor_canfd_drive::ArmDriver::UpdatedArray updated{};
    drivers_[arm]->read_feedback(arm_states_[arm], updated);
    for (std::size_t joint = 0; joint < 7U; ++joint) {
      if (!updated[joint]) {
        continue;
      }
      const auto index = arm * 7U + joint;
      position_[index] = arm_states_[arm][joint].position;
      velocity_[index] = arm_states_[arm][joint].velocity;
      effort_[index] = arm_states_[arm][joint].effort;
      last_feedback_[index] = now;
      if (observed != nullptr) {
        (*observed)[index] = true;
      }
    }
  }
}

hardware_interface::return_type CanFdSystem::read(const rclcpp::Time &,
                                                   const rclcpp::Duration &) {
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    receive_feedback();
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "CAN feedback read failed: %s", exception.what());
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t index = 0; index < last_feedback_.size(); ++index) {
    if (std::chrono::duration<double>(now - last_feedback_[index]).count() >
        config_.feedback_timeout_s) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                   "feedback timeout: %s", info_.joints[index].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanFdSystem::write(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  if (!active_ || !send_position_commands(false)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

} // namespace motor_hardware_interface

PLUGINLIB_EXPORT_CLASS(motor_hardware_interface::CanFdSystem,
                       hardware_interface::SystemInterface)
