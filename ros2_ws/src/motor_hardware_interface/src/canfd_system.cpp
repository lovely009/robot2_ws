#include "motor_hardware_interface/canfd_system.hpp"
#include "motor_hardware_interface/canfd_logic.hpp"
#include "motor_hardware_interface/motor_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motor_hardware_interface {
namespace {

bool has_state_interface(const hardware_interface::ComponentInfo &joint,
                         const std::string &name) {
  return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                     [&](const auto &interface) { return interface.name == name; });
}

std::string join_names(const std::vector<std::string> &names) {
  std::string joined;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0U) {
      joined += "、";
    }
    joined += names[index];
  }
  return joined;
}

} // namespace

bool CanFdSystem::load_config(const std::string &path, std::string &error) {
  try {
    if (path.empty()) {
      throw std::runtime_error("hardware parameter 'config_file' is required");
    }
    config_ = load_system_config(path);
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

bool CanFdSystem::load_transmission_model(const std::string &path, std::string &error) {
  try {
    model_ = std::make_unique<robot_transmission::Model>(
        robot_transmission::Model::load(path));
    return true;
  } catch (const std::exception &exception) {
    error = "transmission model: " + std::string(exception.what());
    return false;
  }
}

bool CanFdSystem::apply_bus_overrides(std::string &error) {
  // bus_override 仅改设备路径；传输类型仍由拓扑 YAML 决定。
  constexpr const char *prefix = "bus_override.";
  const std::string prefix_string(prefix);
  for (const auto &parameter : info_.hardware_parameters) {
    if (parameter.first.rfind(prefix_string, 0U) != 0U) {
      continue;
    }
    const auto bus_name = parameter.first.substr(prefix_string.size());
    const auto bus_it = std::find_if(
        config_.driver.buses.begin(), config_.driver.buses.end(),
        [&](const auto &bus) { return bus.name == bus_name; });
    if (bus_it == config_.driver.buses.end()) {
      error = "hardware parameter '" + parameter.first +
              "' refers to unknown configured bus '" + bus_name + "'";
      return false;
    }
    if (parameter.second.empty()) {
      error = "hardware parameter '" + parameter.first + "' cannot be empty";
      return false;
    }
    if (bus_it->transport == "serial_slcan") {
      bus_it->tty = parameter.second;
    } else {
      bus_it->interface_name = parameter.second;
    }
  }
  return true;
}

bool CanFdSystem::build_joint_mapping(std::string &error) {
  if (!model_) {
    error = "transmission model is not loaded";
    return false;
  }

  joint_indices_.clear();
  joint_indices_.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (!joint_indices_.emplace(joint.name, index).second) {
      error = "duplicate ros2_control joint '" + joint.name + "'";
      return false;
    }
    const auto command_it = std::find_if(
        joint.command_interfaces.begin(), joint.command_interfaces.end(),
        [](const auto &interface) {
          return interface.name == hardware_interface::HW_IF_POSITION;
        });
    if (command_it == joint.command_interfaces.end()) {
      error = "joint '" + joint.name + "' must expose a position command interface";
      return false;
    }
    if (!has_state_interface(joint, hardware_interface::HW_IF_POSITION) ||
        !has_state_interface(joint, hardware_interface::HW_IF_VELOCITY) ||
        !has_state_interface(joint, hardware_interface::HW_IF_EFFORT)) {
      error = "joint '" + joint.name +
              "' must expose position, velocity and effort state interfaces";
      return false;
    }
  }

  const auto &joint_names = model_->joint_names();
  std::set<std::string> unique_joints(joint_names.begin(), joint_names.end());
  if (unique_joints.size() != joint_names.size()) {
    error = "transmission model contains duplicate joint names";
    return false;
  }
  std::vector<std::string> missing;
  for (const auto &name : joint_names) {
    if (joint_indices_.count(name) == 0U) {
      missing.push_back(name);
    }
  }
  if (!missing.empty()) {
    error = "ros2_control is missing transmission joints: " + join_names(missing);
    return false;
  }
  std::vector<std::string> unmapped_joints;
  for (const auto &[name, index] : joint_indices_) {
    static_cast<void>(index);
    if (unique_joints.count(name) == 0U) {
      unmapped_joints.push_back(name);
    }
  }
  if (!unmapped_joints.empty()) {
    error = "ros2_control joints absent from transmission model: " +
            join_names(unmapped_joints);
    return false;
  }
  std::unordered_map<std::string, std::size_t> driver_indices;
  driver_indices.reserve(config_.driver.motors.size());
  for (std::size_t index = 0; index < config_.driver.motors.size(); ++index) {
    if (!driver_indices.emplace(config_.driver.motors[index].name, index).second) {
      error = "duplicate driver motor '" + config_.driver.motors[index].name + "'";
      return false;
    }
  }
  std::vector<std::string> unmapped;
  motor_to_driver_.clear();
  motor_to_driver_.reserve(model_->actuator_names().size());
  for (const auto &name : model_->actuator_names()) {
    const auto motor_it = driver_indices.find(name);
    if (motor_it == driver_indices.end()) {
      unmapped.push_back(name);
    } else {
      motor_to_driver_.push_back(motor_it->second);
    }
  }
  if (!unmapped.empty()) {
    error = "transmission actuators absent from driver: " + join_names(unmapped);
    return false;
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
  if (!load_config(config_path, error) || !load_transmission_model(config_path, error) ||
      !apply_bus_overrides(error) || !build_joint_mapping(error)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "电机硬件配置无效：%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    driver_ = std::make_unique<motor_drive::MotorDriver>(config_.driver);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "CAN-FD 驱动配置无效：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  position_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  velocity_.assign(joint_count, 0.0);
  effort_.assign(joint_count, 0.0);
  command_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  last_feedback_.assign(joint_count, std::chrono::steady_clock::time_point{});
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CanFdSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3U);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
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
  interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name,
                            hardware_interface::HW_IF_POSITION, &command_[index]);
  }
  return interfaces;
}

void CanFdSystem::close_driver() noexcept {
  if (driver_) {
    driver_->close();
  }
}

hardware_interface::CallbackReturn
CanFdSystem::on_activate(const rclcpp_lifecycle::State &) {
  try {
    driver_->open();
    latest_actuator_feedback_.clear();
    motor_last_feedback_.clear();
    std::fill(last_feedback_.begin(), last_feedback_.end(), TimePoint{});
    if (!driver_->set_enabled(false)) {
      throw std::runtime_error("发送初始失能指令失败");
    }

    constexpr std::size_t kMaxActivationAttempts = 3U;
    const auto attempt_window =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config_.initial_feedback_wait_s));
    std::vector<std::string> joint_names;
    joint_names.reserve(info_.joints.size());
    for (const auto &joint : info_.joints) {
      joint_names.push_back(joint.name);
    }
    std::vector<bool> observed;
    bool activated = false;
    for (std::size_t attempt = 1U; attempt <= kMaxActivationAttempts; ++attempt) {
      observed.assign(info_.joints.size(), false);
      const auto deadline = std::chrono::steady_clock::now() + attempt_window;
      while (should_continue_waiting(observed, std::chrono::steady_clock::now(), deadline)) {
        // 旧驱动依赖每轮零目标触发反馈/SYNC。
        if (!send_position_commands(true)) {
          throw std::runtime_error("发送零增益反馈查询失败");
        }
        std::string feedback_error;
        if (!receive_feedback(&observed, feedback_error)) {
          throw std::runtime_error("读取激活反馈失败：" + feedback_error);
        }
        if (!should_continue_waiting(observed, std::chrono::steady_clock::now(), deadline)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (all_feedback_received(observed)) {
        activated = true;
        break;
      }
      RCLCPP_WARN(rclcpp::get_logger("motor_hardware_interface"),
                  "第 %zu/%zu 次激活尝试未收齐全部关节反馈，缺失：%s",
                  attempt, kMaxActivationAttempts,
                  join_names(missing_joint_names(joint_names, observed)).c_str());
    }
    if (!activated) {
      if (driver_->is_open()) {
        static_cast<void>(driver_->set_enabled(false));
      }
      throw std::runtime_error("连续 " + std::to_string(kMaxActivationAttempts) +
                               " 次激活尝试失败，未收到反馈的关节：" +
                               join_names(missing_joint_names(joint_names, observed)));
    }

    std::string activation_error;
    if (!prepare_initial_commands(activation_error)) {
      throw std::runtime_error(activation_error);
    }
    if (!driver_->set_enabled(true)) {
      throw std::runtime_error("使能电机失败");
    }
    // CERB 手册要求上伺服后至少等待 100 ms，才能向新的目标位置运动。
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "激活失败：%s", exception.what());
    if (driver_ && driver_->is_open()) {
      static_cast<void>(driver_->set_enabled(false));
    }
    close_driver();
    active_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn
CanFdSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  if (driver_ && driver_->is_open()) {
    static_cast<void>(driver_->set_enabled(false));
  }
  close_driver();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CanFdSystem::send_position_commands(const bool zero_gains) {
  std::vector<motor_drive::protocol::JointCommand> commands(
      config_.driver.motors.size());
  if (!zero_gains) {
    if (!command_groups_complete()) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                   "拒绝写入不完整的关节目标");
      return false;
    }
    robot_transmission::AxisMap joint_targets;
    for (std::size_t joint = 0; joint < info_.joints.size(); ++joint) {
      if (std::isfinite(command_[joint])) {
        joint_targets.emplace(info_.joints[joint].name,
                              robot_transmission::Axis(command_[joint]));
      }
    }
    try {
      const auto actuator_targets = model_->joint_to_actuator(joint_targets);
      for (std::size_t motor = 0; motor < model_->actuator_names().size(); ++motor) {
        const auto &name = model_->actuator_names()[motor];
        const auto target_it = actuator_targets.find(name);
        if (target_it == actuator_targets.end()) {
          throw std::runtime_error("no transmission target for actuator '" + name + "'");
        }
        const auto &target = target_it->second;
        if (!std::isfinite(target.position)) {
          throw std::runtime_error("nonfinite transmission target for actuator '" + name + "'");
        }
        const auto &motor_config = config_.driver.motors[motor_to_driver_[motor]];
        const auto limit_it = motor_config.parameters.find("mit_position_limit_rad");
        if (limit_it != motor_config.parameters.end()) {
          std::size_t parsed = 0U;
          const double position_limit = std::stod(limit_it->second, &parsed);
          if (!limit_it->second.empty() && parsed == limit_it->second.size() &&
              std::isfinite(position_limit) && position_limit > 0.0) {
          } else {
            throw std::runtime_error("invalid mit_position_limit_rad for actuator '" + name + "'");
          }
          if (std::abs(target.position) > position_limit) {
            throw std::runtime_error("transmission target for actuator '" + name +
                                     "' exceeds mit_position_limit_rad");
          }
        }
        auto &command = commands[motor_to_driver_[motor]];
        command.position = target.position;
        command.kp = config_.motor_commands[motor_to_driver_[motor]].kp;
        command.kd = config_.motor_commands[motor_to_driver_[motor]].kd;
      }
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                   "关节命令无法转换为执行器命令：%s", exception.what());
      return false;
    }
  }
  return driver_->write_commands(commands);
}

bool CanFdSystem::receive_feedback(std::vector<bool> *observed,
                                   std::string &error) {
  // 每轮更新缓存但不提交；交错反馈由后续轮次从缓存组成新鲜窗口。
  std::vector<motor_drive::protocol::JointState> motor_states(
      config_.driver.motors.size());
  std::vector<bool> motor_fresh;
  driver_->read_feedback(motor_states, motor_fresh);
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t motor = 0; motor < motor_fresh.size(); ++motor) {
    if (!motor_fresh[motor]) {
      continue;
    }
    const auto &name = config_.driver.motors[motor].name;
    const auto &state = motor_states[motor];
    latest_actuator_feedback_[name] =
        robot_transmission::Axis(state.position, state.velocity, state.effort);
    motor_last_feedback_[name] = now;
  }

  const auto min_skew =
      std::min(0.05, config_.feedback_timeout_s);
  for (const auto &group : model_->groups()) {
    robot_transmission::AxisMap candidates;
    auto newest = TimePoint::min();
    auto oldest = TimePoint::max();
    for (const auto &name : group.actuators) {
      const auto state_it = latest_actuator_feedback_.find(name);
      const auto time_it = motor_last_feedback_.find(name);
      if (state_it == latest_actuator_feedback_.end() ||
          time_it == motor_last_feedback_.end() ||
          std::chrono::duration<double>(now - time_it->second).count() >
              config_.feedback_timeout_s) {
        continue;
      }
      candidates.emplace(name, state_it->second);
      newest = std::max(newest, time_it->second);
      oldest = std::min(oldest, time_it->second);
    }
    if (candidates.size() != group.actuators.size() ||
        std::chrono::duration<double>(newest - oldest).count() > min_skew) {
      continue;
    }

    robot_transmission::AxisMap joint_states;
    try {
      joint_states = model_->actuator_to_joint(candidates);
    } catch (const std::exception &exception) {
      RCLCPP_WARN(rclcpp::get_logger("motor_hardware_interface"),
                  "忽略无效的执行器反馈：%s", exception.what());
      error = exception.what();
      return false;
    }
    bool complete = true;
    for (const auto &name : group.joints) {
      complete &= joint_states.count(name) != 0U &&
                  joint_indices_.count(name) != 0U;
    }
    if (!complete) {
      error = "transmission group '" + group.name +
              "' has a joint absent from ros2_control";
      return false;
    }
    for (const auto &name : group.joints) {
      const auto index = joint_indices_.at(name);
      const auto &state = joint_states.at(name);
      position_[index] = state.position;
      velocity_[index] = state.velocity;
      effort_[index] = state.effort;
      last_feedback_[index] = oldest;
      if (observed != nullptr) {
        observed->at(index) = true;
      }
    }
  }
  return true;
}

bool CanFdSystem::command_groups_complete() const {
  const auto &groups = model_->groups();
  if (groups.empty()) {
    return false;
  }
  std::vector<bool> covered(info_.joints.size(), false);
  for (const auto &group : groups) {
    if (group.joints.empty()) {
      return false;
    }
    std::size_t present = 0U;
    for (const auto &name : group.joints) {
      const auto index_it = joint_indices_.find(name);
      if (index_it != joint_indices_.end() &&
          std::isfinite(command_[index_it->second])) {
        ++present;
      }
    }
    if (present != group.joints.size()) {
      return false;
    }
  }
  for (const auto &group : groups) {
    for (const auto &name : group.joints) {
      covered[joint_indices_.at(name)] = true;
    }
  }
  return std::all_of(covered.begin(), covered.end(),
                     [](const bool value) { return value; });
}

bool CanFdSystem::prepare_initial_commands(std::string &error) {
  std::vector<bool> observed(info_.joints.size(), false);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    observed[index] = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - last_feedback_[index])
                              .count() <= config_.feedback_timeout_s;
  }
  for (const auto &group : model_->groups()) {
    bool complete = true;
    for (const auto &name : group.joints) {
      const auto index_it = joint_indices_.find(name);
      complete &= index_it != joint_indices_.end() &&
                  observed[index_it->second];
    }
    if (!complete) {
      error = "no complete fresh feedback for transmission group '" + group.name + "'";
      return false;
    }
  }
  if (!take_observed_positions_as_initial_targets(
          model_->joint_names(), observed, position_, command_, error)) {
    return false;
  }
  if (!command_groups_complete()) {
    error = "incomplete initial transmission command";
    return false;
  }
  return true;
}

hardware_interface::return_type CanFdSystem::read(const rclcpp::Time &,
                                                   const rclcpp::Duration &) {
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  std::string feedback_error;
  if (!receive_feedback(nullptr, feedback_error)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "CAN 反馈读取失败：%s", feedback_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto stale =
      first_stale_feedback(last_feedback_, now, config_.feedback_timeout_s);
  if (stale != last_feedback_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "反馈超时：%s", info_.joints[stale].name.c_str());
    return hardware_interface::return_type::ERROR;
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
