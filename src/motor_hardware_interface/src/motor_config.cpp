#include "motor_hardware_interface/motor_config.hpp"

#include <cmath>
#include <filesystem>
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
  // 后写入的单电机参数覆盖型号库默认值，使同型号电机可以只配置差异项。
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

// 型号库 motor_types.<type>.parameters -> MotorParameterMap。
std::unordered_map<std::string, motor_drive::MotorParameterMap>
parse_type_parameters(const YAML::Node &motor_types) {
  std::unordered_map<std::string, motor_drive::MotorParameterMap> result;
  for (const auto &entry : motor_types) {
    const auto type = entry.first.as<std::string>();
    result[type] = parse_driver_parameters(entry.second["parameters"],
                                           "motor type '" + type + "'");
  }
  return result;
}

motor_drive::DriverConfig parse_driver_config(const YAML::Node &hardware,
                                              const YAML::Node &motor_types) {
  // 输出纯 C++ DriverConfig 后，底层驱动不再接触 YAML。
  // buses 来自拓扑文件；参数默认值来自型号库（motor.yaml），单电机可覆盖。
  motor_drive::DriverConfig config;
  const auto buses = hardware["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("motor_hardware.buses must be a non-empty map");
  }
  for (const auto &entry : buses) {
    motor_drive::BusConfig bus;
    bus.name = entry.first.as<std::string>();
    const auto node = entry.second;
    if (node["transport"]) {
      bus.transport = node["transport"].as<std::string>();
    }
    if (node["interface"]) {
      bus.interface_name = node["interface"].as<std::string>();
    }
    if (node["tty"]) {
      bus.tty = node["tty"].as<std::string>();
    }
    // 传输类型缺省 socketcan（struct 默认值）；显式空串非法。
    if (bus.transport.empty()) {
      throw std::runtime_error("bus '" + bus.name +
                               "' has an empty 'transport'");
    }
    if (bus.transport == "socketcan" && bus.interface_name.empty()) {
      throw std::runtime_error("bus '" + bus.name +
                               "' with transport socketcan requires 'interface'");
    }
    if (bus.transport == "serial_slcan" && bus.tty.empty()) {
      throw std::runtime_error("bus '" + bus.name +
                               "' with transport serial_slcan requires 'tty'");
    }
    config.buses.push_back(std::move(bus));
  }

  const auto type_parameters = parse_type_parameters(motor_types);

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
    const auto model_type = node["type"].as<std::string>();  // 型号名，如 rp-50h
    motor.id = node["id"].as<unsigned>();

    // 型号 -> 协议类型：motor_types.<model>.protocol 声明所属协议；缺省时型号名
    // 即协议类型。映射出的协议类型由 MotorDriver 校验是否注册。
    const auto type_node = motor_types[model_type];
    if (!type_node) {
      throw std::runtime_error("motor '" + motor.name +
                               "' references unknown type '" + model_type + "'");
    }
    motor.type = type_node["protocol"] ? type_node["protocol"].as<std::string>()
                                       : model_type;

    // 参数默认值来自型号库（按型号名查找，而非映射后的协议类型）。
    const auto defaults = type_parameters.find(model_type);
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

std::pair<double, double> parse_command_gains(const YAML::Node &motor_node,
                                              const std::string &motor_name) {
  // kp/kd 是动态参数，直接配置在拓扑文件的 motors.<名>.command。
  const auto motor_command = motor_node["command"];
  const auto resolve = [&](const char *key) -> double {
    if (motor_command && motor_command[key]) {
      return motor_command[key].as<double>();
    }
    throw std::runtime_error("motor '" + motor_name + "' requires command." + key);
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

// 从拓扑文件读取型号库路径，并相对拓扑文件目录解析。
std::string resolve_specs_file(const YAML::Node &hardware,
                               const std::string &topology_path) {
  const auto node = hardware["motor_specs_file"];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("motor_hardware.motor_specs_file is required");
  }
  const std::string name = node.as<std::string>();
  if (name.empty()) {
    throw std::runtime_error("motor_hardware.motor_specs_file cannot be empty");
  }
  return (std::filesystem::path(topology_path).parent_path() / name).string();
}

} // namespace

MotorSystemConfig load_system_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor_config_file is required");
  }
  // 拓扑文件是唯一入口：motor_hardware.motor_specs_file 指向型号库 motor.yaml。
  const auto document = YAML::LoadFile(path); // 文件不可读时抛 YAML::BadFile
  const auto hardware = document["motor_hardware"];
  if (!hardware || !hardware.IsMap()) {
    throw std::runtime_error("YAML root 'motor_hardware' is required");
  }

  const auto specs_path = resolve_specs_file(hardware, path);
  const auto specs_document = YAML::LoadFile(specs_path); // 不可读时抛 YAML::BadFile
  const auto motor_types = specs_document["motor_types"];
  if (!motor_types || !motor_types.IsMap()) {
    throw std::runtime_error("motor_types is required in motor specs file '" +
                             specs_path + "'");
  }

  MotorSystemConfig config;
  config.driver = parse_driver_config(hardware, motor_types);
  const auto timing = parse_timing_config(hardware);
  config.feedback_timeout_s = timing.feedback_timeout_s;
  config.initial_feedback_wait_s = timing.initial_feedback_wait_s;

  // 每台电机的命令增益：型号库给默认值，单电机可覆盖。
  const auto motors = hardware["motors"];
  config.motor_commands.reserve(config.driver.motors.size());
  for (const auto &motor : config.driver.motors) {
    const auto motor_node = motors[motor.name];
    const auto gains = parse_command_gains(motor_node, motor.name);
    config.motor_commands.push_back({gains.first, gains.second});
  }
  return config;
}

motor_drive::DriverConfig load_driver_config(const std::string &path) {
  return load_system_config(path).driver;
}

} // namespace motor_hardware_interface
