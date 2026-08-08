#include "motor_hardware_interface/canfd_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

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

motor_canfd_drive::MotorParameterMap
parse_driver_parameters(const YAML::Node &node, const std::string &context) {
  // 底层 DriverConfig 不依赖 yaml-cpp，所以在 ROS 适配层把协议私有参数统一转换成
  // string->string。具体协议策略再按需要解释数值和单位。
  motor_canfd_drive::MotorParameterMap parameters;
  if (!node) {
    return parameters;
  }
  if (!node.IsMap()) {
    throw std::runtime_error(context + " parameters must be a YAML map");
  }
  for (const auto &entry : node) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error(context +
                               " parameters must contain scalar key/value pairs");
    }
    parameters[entry.first.Scalar()] = entry.second.Scalar();
  }
  return parameters;
}

void merge_parameters(motor_canfd_drive::MotorParameterMap &target,
                      const motor_canfd_drive::MotorParameterMap &overrides) {
  // 后写入的单电机参数覆盖 motor_types 默认值，使同型号电机可以只配置差异项。
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

motor_canfd_drive::DriverConfig parse_driver_config(const YAML::Node &hardware) {
  // YAML 只在本 ROS 侧解析；函数输出纯 C++ DriverConfig 后，底层驱动不再接触 YAML。
  motor_canfd_drive::DriverConfig config;
  const auto buses = hardware["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("motor_hardware.buses must be a non-empty map");
  }
  for (const auto &entry : buses) {
    config.buses.push_back({entry.first.as<std::string>(),
                            entry.second["interface"].as<std::string>()});
  }

  std::unordered_map<std::string, motor_canfd_drive::MotorParameterMap>
      type_parameters;
  const auto motor_types = hardware["motor_types"];
  if (motor_types) {
    if (!motor_types.IsMap()) {
      throw std::runtime_error("motor_hardware.motor_types must be a map");
    }
    for (const auto &entry : motor_types) {
      const auto type = entry.first.as<std::string>();
      type_parameters[type] = parse_driver_parameters(
          entry.second["parameters"], "motor type '" + type + "'");
    }
  }

  const auto motors = hardware["motors"];
  if (!motors || !motors.IsMap() || motors.size() == 0U) {
    throw std::runtime_error("motor_hardware.motors must be a non-empty map");
  }
  for (const auto &entry : motors) {
    motor_canfd_drive::MotorConfig motor;
    motor.name = entry.first.as<std::string>();
    const auto node = entry.second;
    motor.joint_name = node["joint"].as<std::string>();
    motor.bus = node["bus"].as<std::string>();
    motor.type = node["type"].as<std::string>();
    motor.id = node["id"].as<unsigned>();
    const auto defaults = type_parameters.find(motor.type);
    if (defaults != type_parameters.end()) {
      motor.parameters = defaults->second;
    }
    merge_parameters(motor.parameters, parse_driver_parameters(
                                           node["parameters"],
                                           "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

std::pair<double, double> parse_command_config(const YAML::Node &type_node,
                                               const YAML::Node &motor_node,
                                               const std::string &motor_name) {
  const auto type_command = type_node ? type_node["command"] : YAML::Node{};
  const auto motor_command = motor_node["command"];
  const auto resolve = [&](const char *key) -> double {
    // 优先级：单电机 command > motor_types 公共默认值。
    if (motor_command && motor_command[key]) {
      return motor_command[key].as<double>();
    }
    if (type_command && type_command[key]) {
      return type_command[key].as<double>();
    }
    throw std::runtime_error("motor '" + motor_name +
                             "' requires command." + key +
                             " either per motor or in motor_types defaults");
  };
  const double kp = resolve("kp");
  const double kd = resolve("kd");
  if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0) {
    throw std::runtime_error("motor '" + motor_name +
                             "' command kp/kd must be finite and non-negative");
  }
  return {kp, kd};
}

} // namespace

bool CanFdSystem::load_config(const std::string &path, std::string &error) {
  try {
    if (path.empty()) {
      throw std::runtime_error("hardware parameter 'config_file' is required");
    }
    const auto document = YAML::LoadFile(path);
    const auto hardware = document["motor_hardware"];
    if (!hardware || !hardware.IsMap()) {
      throw std::runtime_error("YAML root 'motor_hardware' is required");
    }
    config_.driver = parse_driver_config(hardware);
    const auto timing = hardware["timing"];
    if (!timing || !timing.IsMap()) {
      throw std::runtime_error("motor_hardware.timing is required");
    }
    config_.feedback_timeout_s = timing["feedback_timeout_s"].as<double>();
    config_.initial_feedback_wait_s = timing["initial_feedback_wait_s"].as<double>();
    if (!std::isfinite(config_.feedback_timeout_s) ||
        !std::isfinite(config_.initial_feedback_wait_s) ||
        config_.feedback_timeout_s <= 0.0 || config_.initial_feedback_wait_s <= 0.0) {
      throw std::runtime_error("feedback timeouts must be finite positive values");
    }

    const auto types = hardware["motor_types"];
    const auto motors = hardware["motors"];
    config_.motor_commands.clear();
    config_.motor_commands.reserve(config_.driver.motors.size());
    for (const auto &motor : config_.driver.motors) {
      const auto motor_node = motors[motor.name];
      const auto type_node = types ? types[motor.type] : YAML::Node{};
      const auto gains = parse_command_config(type_node, motor_node, motor.name);
      config_.motor_commands.push_back({gains.first, gains.second});
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

bool CanFdSystem::apply_bus_overrides(std::string &error) {
  // launch 可把 YAML 中的逻辑总线临时改映射到其他 SocketCAN 接口。
  // 例如真机配置 left_arm:can0，回环仿真时覆盖成 left_arm:vcan0。
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
    bus_it->interface_name = parameter.second;
  }
  return true;
}

bool CanFdSystem::build_joint_mapping(std::string &error) {
  const auto &motors = config_.driver.motors;
  if (info_.joints.size() != motors.size()) {
    error = "ros2_control exposes " + std::to_string(info_.joints.size()) +
            " joints but YAML maps " + std::to_string(motors.size()) + " motors";
    return false;
  }

  // 先检查 ros2_control 暴露的接口形状，再按 joint.name 建表。
  // 之后所有 motor->joint 关联都靠名字查找，不依赖 YAML 与 URDF 的排列顺序。
  std::unordered_map<std::string, std::size_t> joint_index;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (!joint_index.emplace(joint.name, index).second) {
      error = "duplicate ros2_control joint '" + joint.name + "'";
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

  // motor_to_joint_ 是运行期唯一需要的快速映射；每个 joint 必须且只能被映射一次。
  motor_to_joint_.assign(motors.size(), 0U);
  std::vector<bool> mapped(info_.joints.size(), false);
  for (std::size_t motor = 0; motor < motors.size(); ++motor) {
    const auto it = joint_index.find(motors[motor].joint_name);
    if (it == joint_index.end()) {
      error = "motor '" + motors[motor].name + "' maps unknown ros2_control joint '" +
              motors[motor].joint_name + "'";
      return false;
    }
    motor_to_joint_[motor] = it->second;
    mapped[it->second] = true;
  }
  if (!std::all_of(mapped.begin(), mapped.end(), [](const bool value) { return value; })) {
    error = "every ros2_control joint must have exactly one motor mapping";
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
  if (!load_config(config_path, error) || !apply_bus_overrides(error) ||
      !build_joint_mapping(error)) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "invalid motor hardware configuration: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    // 构造 MotorCanfdDriver 时会继续做协议级验证，例如 RP ID 范围、
    // 25/30 mm 节点范围以及必需的 torque_constant 参数。
    driver_ = std::make_unique<motor_canfd_drive::MotorCanfdDriver>(config_.driver);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "invalid CAN-FD driver configuration: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  position_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  velocity_.assign(joint_count, 0.0);
  effort_.assign(joint_count, 0.0);
  command_.assign(joint_count, 0.0);
  last_feedback_.assign(joint_count, std::chrono::steady_clock::time_point{});
  motor_states_.assign(config_.driver.motors.size(), {});
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
    if (!driver_->set_enabled(false)) {
      throw std::runtime_error("failed to send initial disable command");
    }

    // 激活安全原则：绝不假设上电时机器人位于“零位”。
    // 先保持失能并发送零增益命令，只利用反馈读取真实当前位置；全部关节可见后，
    // 再把当前姿态复制为初始 position target，避免一使能就突然追赶错误目标。
    std::vector<bool> observed(info_.joints.size(), false);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.initial_feedback_wait_s);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!send_position_commands(true)) {
        throw std::runtime_error("failed to send zero-gain feedback query");
      }
      receive_feedback(&observed);
      if (std::all_of(observed.begin(), observed.end(),
                      [](const bool seen) { return seen; })) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (std::size_t joint = 0; joint < observed.size(); ++joint) {
      if (!observed[joint] || !std::isfinite(position_[joint])) {
        throw std::runtime_error("no initial feedback from joint '" +
                                 info_.joints[joint].name + "'");
      }
      command_[joint] = position_[joint];
    }

    if (!driver_->set_enabled(true)) {
      throw std::runtime_error("failed to enable one or more motors");
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                 "activation failed: %s", exception.what());
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
  std::vector<motor_canfd_drive::protocol::JointCommand> commands(
      config_.driver.motors.size());
  // ros2_control 以 joint 顺序保存 command_，底层 driver 以 motor 顺序收命令；
  // 这里通过 motor_to_joint_ 做显式转换，因此两边顺序可以不同。
  for (std::size_t motor = 0; motor < commands.size(); ++motor) {
    const auto joint = motor_to_joint_[motor];
    commands[motor].position = command_[joint];
    if (!zero_gains) {
      commands[motor].kp = config_.motor_commands[motor].kp;
      commands[motor].kd = config_.motor_commands[motor].kd;
    }
  }
  return driver_->write_commands(commands);
}

void CanFdSystem::receive_feedback(std::vector<bool> *observed) {
  std::vector<bool> updated;
  driver_->read_feedback(motor_states_, updated);
  // updated 只标记本轮真正收到新 CAN 反馈的电机；只有这些 joint 才刷新时间戳，
  // 这样 read() 的反馈超时能识别“某一台电机掉线”，而不是被其他电机的帧掩盖。
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t motor = 0; motor < updated.size(); ++motor) {
    if (!updated[motor]) {
      continue;
    }
    const auto joint = motor_to_joint_[motor];
    position_[joint] = motor_states_[motor].position;
    velocity_[joint] = motor_states_[motor].velocity;
    effort_[joint] = motor_states_[motor].effort;
    last_feedback_[joint] = now;
    if (observed != nullptr) {
      observed->at(joint) = true;
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
  // 使用 steady_clock 做超时判断，避免系统时间校准/跳变造成误报。
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t joint = 0; joint < last_feedback_.size(); ++joint) {
    if (std::chrono::duration<double>(now - last_feedback_[joint]).count() >
        config_.feedback_timeout_s) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_hardware_interface"),
                   "feedback timeout: %s", info_.joints[joint].name.c_str());
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
