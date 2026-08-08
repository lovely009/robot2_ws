#include "motor_canfd_drive/config_loader.hpp"

#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace motor_canfd_drive {
namespace {

MotorParameterMap parse_parameters(const YAML::Node &node,
                                   const std::string &context) {
  MotorParameterMap parameters;
  if (!node) {
    return parameters;
  }
  if (!node.IsMap()) {
    throw std::runtime_error(context + " parameters must be a YAML map");
  }
  for (const auto &entry : node) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error(context + " parameters must contain scalar key/value pairs");
    }
    parameters[entry.first.Scalar()] = entry.second.Scalar();
  }
  return parameters;
}

void merge_parameters(MotorParameterMap &target,
                      const MotorParameterMap &overrides) {
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

} // namespace

DriverConfig load_driver_config_yaml(const std::string &path,
                                     const std::string &root_key) {
  if (path.empty()) {
    throw std::invalid_argument("motor config YAML path cannot be empty");
  }
  const auto document = YAML::LoadFile(path);
  const auto root = document[root_key];
  if (!root || !root.IsMap()) {
    throw std::runtime_error("YAML root '" + root_key + "' is required");
  }

  DriverConfig config;
  const auto buses = root["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("'" + root_key + ".buses' must be a non-empty map");
  }
  for (const auto &entry : buses) {
    BusConfig bus;
    bus.name = entry.first.as<std::string>();
    bus.interface_name = entry.second["interface"].as<std::string>();
    config.buses.push_back(std::move(bus));
  }

  std::unordered_map<std::string, MotorParameterMap> type_parameters;
  const auto motor_types = root["motor_types"];
  if (motor_types) {
    if (!motor_types.IsMap()) {
      throw std::runtime_error("'" + root_key + ".motor_types' must be a map");
    }
    for (const auto &entry : motor_types) {
      const auto type = entry.first.as<std::string>();
      type_parameters[type] =
          parse_parameters(entry.second["parameters"], "motor type '" + type + "'");
    }
  }

  const auto motors = root["motors"];
  if (!motors || !motors.IsMap() || motors.size() == 0U) {
    throw std::runtime_error("'" + root_key + ".motors' must be a non-empty map");
  }
  for (const auto &entry : motors) {
    MotorConfig motor;
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
                     parse_parameters(node["parameters"], "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

} // namespace motor_canfd_drive
