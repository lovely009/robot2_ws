#include "motor_hardware_interface/motor_config.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace motor_hardware_interface {
namespace {

motor_canfd_drive::MotorParameterMap
parse_driver_parameters(const YAML::Node &node, const std::string &context) {
  // 底层 DriverConfig 不依赖 yaml-cpp，所以在这里把协议私有参数统一转换成
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
  // 输出纯 C++ DriverConfig 后，底层驱动不再接触 YAML。
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
    merge_parameters(motor.parameters,
                     parse_driver_parameters(node["parameters"],
                                             "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

} // namespace

motor_canfd_drive::DriverConfig load_driver_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor_config_file is required");
  }
  const auto document = YAML::LoadFile(path); // 文件不可读时抛 YAML::BadFile
  const auto hardware = document["motor_hardware"];
  if (!hardware || !hardware.IsMap()) {
    throw std::runtime_error("YAML root 'motor_hardware' is required");
  }
  return parse_driver_config(hardware);
}

} // namespace motor_hardware_interface
