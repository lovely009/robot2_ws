#include "motor_hardware_interface/motor_config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace motor_hardware_interface {
namespace {

motor_drive::MotorParameterMap
parse_driver_parameters(const YAML::Node &node, const std::string &context) {
  // 底层 DriverConfig 不依赖 yaml-cpp，所以在这里把协议私有参数统一转换成
  // string->string。具体协议策略再按需要解释数值和单位。
  motor_drive::MotorParameterMap parameters;
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

void merge_parameters(motor_drive::MotorParameterMap &target,
                      const motor_drive::MotorParameterMap &overrides) {
  // 后写入的单电机参数覆盖 motor_types 默认值，使同型号电机可以只配置差异项。
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

motor_drive::DriverConfig parse_driver_config(const YAML::Node &hardware) {
  // 输出纯 C++ DriverConfig 后，底层驱动不再接触 YAML。
  motor_drive::DriverConfig config;
  const auto buses = hardware["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("motor_hardware.buses must be a non-empty map");
  }
  for (const auto &entry : buses) {
    motor_drive::BusConfig bus;
    bus.name = entry.first.as<std::string>();
    const auto node = entry.second;
    if (node["interface"]) {
      bus.interface_name = node["interface"].as<std::string>();
    }
    if (bus.interface_name.empty()) {
      throw std::runtime_error("bus '" + bus.name + "' requires 'interface'");
    }
    config.buses.push_back(std::move(bus));
  }

  std::unordered_map<std::string, motor_drive::MotorParameterMap>
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
    motor_drive::MotorConfig motor;
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
    merge_parameters(motor.parameters,
                     parse_driver_parameters(node["parameters"],
                                             "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

std::pair<double, double> parse_command_gains(const YAML::Node &type_node,
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
    throw std::runtime_error("motor '" + motor_name + "' requires command." + key +
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

struct TimingConfig {
  double feedback_timeout_s{0.1};
  double initial_feedback_wait_s{0.1};
};

TimingConfig parse_timing_config(const YAML::Node &hardware) {
  const auto timing = hardware["timing"];
  if (!timing || !timing.IsMap()) {
    throw std::runtime_error("motor_hardware.timing is required");
  }
  TimingConfig config;
  config.feedback_timeout_s = timing["feedback_timeout_s"].as<double>();
  config.initial_feedback_wait_s = timing["initial_feedback_wait_s"].as<double>();
  if (!std::isfinite(config.feedback_timeout_s) ||
      !std::isfinite(config.initial_feedback_wait_s) ||
      config.feedback_timeout_s <= 0.0 || config.initial_feedback_wait_s <= 0.0) {
    throw std::runtime_error("feedback timeouts must be finite positive values");
  }
  return config;
}

} // namespace

MotorSystemConfig load_system_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor_config_file is required");
  }
  const auto document = YAML::LoadFile(path); // 文件不可读时抛 YAML::BadFile
  const auto hardware = document["motor_hardware"];
  if (!hardware || !hardware.IsMap()) {
    throw std::runtime_error("YAML root 'motor_hardware' is required");
  }

  MotorSystemConfig config;
  config.driver = parse_driver_config(hardware);
  const auto timing = parse_timing_config(hardware);
  config.feedback_timeout_s = timing.feedback_timeout_s;
  config.initial_feedback_wait_s = timing.initial_feedback_wait_s;

  // 每台电机的命令增益：motor_types 给默认值，单电机可覆盖。
  const auto types = hardware["motor_types"];
  const auto motors = hardware["motors"];
  config.motor_commands.reserve(config.driver.motors.size());
  for (const auto &motor : config.driver.motors) {
    const auto motor_node = motors[motor.name];
    const auto type_node = types ? types[motor.type] : YAML::Node{};
    const auto gains = parse_command_gains(type_node, motor_node, motor.name);
    config.motor_commands.push_back({gains.first, gains.second});
  }
  return config;
}

motor_drive::DriverConfig load_driver_config(const std::string &path) {
  return load_system_config(path).driver;
}

} // namespace motor_hardware_interface
