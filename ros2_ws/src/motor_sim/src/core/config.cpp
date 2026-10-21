#include "motor_sim/core/config.hpp"

#include "motor_sim/protocol/motor_protocol.hpp"

#include <yaml-cpp/yaml.h>

#include <climits>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace motor_sim::core {

namespace protocol = ::motor_sim::protocol;
namespace {

struct MotorProfile {
  TransmissionConfig transmission;
  JointLimits limits;
  PlantConfig plant;
  // SDO 从站的对端契约；非 CERB 型号为空/0，由 motors 循环按协议决定是否必须。
  std::string device_identity;
  int device_max_speed_rpm{0};
  int device_max_current_ma{0};
  // MIT τ_ff 的 12 位量化满量程（N·m）：型号属性，见 profile 的 mit_torque_limit_nm。
  double mit_torque_limit_nm{protocol::kCerbDefaultMitTorqueLimitNm};
};

const YAML::Node required_node(const YAML::Node &parent, const char *key,
                               const std::string &context) {
  const auto node = parent[key];
  if (!node) {
    throw std::runtime_error(context + " requires '" + key + "'");
  }
  return node;
}

double finite_double(const YAML::Node &parent, const char *key,
                     const std::string &context) {
  const double value = required_node(parent, key, context).as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(context + " '" + key + "' must be finite");
  }
  return value;
}

double optional_finite_double(const YAML::Node &parent, const char *key,
                              const double fallback,
                              const std::string &context) {
  if (!parent[key]) {
    return fallback;
  }
  const double value = parent[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(context + " '" + key + "' must be finite");
  }
  return value;
}

// positive_unsigned 要求 > 0，而节拍类字段的 0 有语义（"跟随旧行为"），故另写。
unsigned optional_unsigned(const YAML::Node &parent, const char *key,
                           const unsigned fallback,
                           const std::string &context) {
  if (!parent[key]) {
    return fallback;
  }
  const auto raw = required_node(parent, key, context).as<int64_t>();
  if (raw < 0 || raw > static_cast<int64_t>(UINT32_MAX)) {
    throw std::runtime_error(context + " '" + key + "' cannot be negative");
  }
  return static_cast<unsigned>(raw);
}

double optional_nonnegative_double(const YAML::Node &parent, const char *key,
                                   const double fallback,
                                   const std::string &context) {
  const double value = optional_finite_double(parent, key, fallback, context);
  if (value < 0.0) {
    throw std::runtime_error(context + " '" + key + "' cannot be negative");
  }
  return value;
}

unsigned positive_unsigned(const YAML::Node &parent, const char *key,
                           const std::string &context) {
  const auto raw = required_node(parent, key, context).as<int64_t>();
  if (raw <= 0 || raw > static_cast<int64_t>(UINT32_MAX)) {
    throw std::runtime_error(context + " '" + key + "' must be positive");
  }
  return static_cast<unsigned>(raw);
}

std::string required_string(const YAML::Node &parent, const char *key,
                            const std::string &context) {
  const auto value = required_node(parent, key, context).as<std::string>();
  if (value.empty()) {
    throw std::runtime_error(context + " '" + key + "' cannot be empty");
  }
  return value;
}

std::string optional_string(const YAML::Node &parent, const char *key,
                            const std::string &fallback,
                            const std::string &context) {
  if (!parent[key]) {
    return fallback;
  }
  return required_string(parent, key, context);
}

double radians(const double degrees) {
  return degrees * protocol::kPi / 180.0;
}

bool optional_bool(const YAML::Node &parent, const char *key, const bool fallback,
                   const std::string &context) {
  if (!parent[key]) {
    return fallback;
  }
  return required_node(parent, key, context).as<bool>();
}

int optional_int(const YAML::Node &parent, const char *key, const int fallback,
                 const std::string &context) {
  if (!parent[key]) {
    return fallback;
  }
  const auto raw = required_node(parent, key, context).as<int64_t>();
  if (raw < INT32_MIN || raw > INT32_MAX) {
    throw std::runtime_error(context + " '" + key + "' is out of range");
  }
  return static_cast<int>(raw);
}

BusTransport parse_bus_transport(const YAML::Node &node,
                                 const std::string &context) {
  const auto value = required_string(node, "transport", context);
  if (value == "usb_serial") {
    return BusTransport::UsbSerial;
  }
  if (value == "serial_slcan") {
    return BusTransport::SerialSlcan;
  }
  if (value == "socket_can") {
    return BusTransport::SocketCan;
  }
  throw std::runtime_error(context + " has unknown transport '" + value + "'");
}

MotorProfile parse_profile(const std::string &name, const YAML::Node &node,
                           const double command_margin_deg) {
  const std::string context = "profile '" + name + "'";
  MotorProfile profile;
  const double hard_lower_deg = finite_double(node, "hard_lower_deg", context);
  const double hard_upper_deg = finite_double(node, "hard_upper_deg", context);
  const double command_lower_deg = optional_finite_double(
      node, "command_lower_deg", hard_lower_deg + command_margin_deg, context);
  const double command_upper_deg = optional_finite_double(
      node, "command_upper_deg", hard_upper_deg - command_margin_deg, context);
  profile.limits.hard_lower_rad = radians(hard_lower_deg);
  profile.limits.hard_upper_rad = radians(hard_upper_deg);
  profile.limits.command_lower_rad = radians(command_lower_deg);
  profile.limits.command_upper_rad = radians(command_upper_deg);
  profile.limits.velocity_rad_s = finite_double(node, "velocity_rad_s", context);
  profile.limits.effort_nm = finite_double(node, "effort_nm", context);
  if (!(profile.limits.hard_lower_rad <
            profile.limits.command_lower_rad &&
        profile.limits.command_lower_rad <
            profile.limits.command_upper_rad &&
        profile.limits.command_upper_rad < profile.limits.hard_upper_rad) ||
      profile.limits.velocity_rad_s <= 0.0 ||
      profile.limits.effort_nm <= 0.0) {
    throw std::runtime_error(
        context + " has invalid hard/command, velocity or effort limits");
  }

  profile.transmission.gear_ratio =
      finite_double(node, "gear_ratio", context);
  profile.transmission.efficiency =
      finite_double(node, "efficiency", context);
  // MIT τ_ff 的量化满量程（真机侧 mit_torque_limit_nm，厂家 2026-10-21：R58/R48
  // 同 300.0）；缺省退回 kCerbDefaultMitTorqueLimitNm。
  profile.mit_torque_limit_nm = optional_finite_double(
      node, "mit_torque_limit_nm", protocol::kCerbDefaultMitTorqueLimitNm,
      context);
  if (profile.transmission.gear_ratio <= 0.0 ||
      profile.transmission.efficiency <= 0.0 ||
      profile.transmission.efficiency > 1.0) {
    throw std::runtime_error(context + " has invalid transmission values");
  }

  profile.device_identity =
      optional_string(node, "device_identity", "", context);
  profile.device_max_speed_rpm =
      optional_int(node, "device_max_speed_rpm", 0, context);
  profile.device_max_current_ma =
      optional_int(node, "device_max_current_ma", 0, context);

  const auto plant = required_node(node, "plant", context);
  profile.plant.kp = finite_double(plant, "kp", context + ".plant");
  profile.plant.kd = finite_double(plant, "kd", context + ".plant");
  profile.plant.effort_limit_nm = profile.limits.effort_nm;
  profile.plant.torque_constant_nm_per_a = optional_finite_double(
      plant, "torque_constant_nm_per_a", 0.0, context + ".plant");
  profile.plant.current_limit_a = optional_finite_double(
      plant, "current_limit_a", 0.0, context + ".plant");
  if (profile.plant.kp < 0.0 || profile.plant.kd < 0.0 ||
      profile.plant.torque_constant_nm_per_a < 0.0 ||
      profile.plant.current_limit_a < 0.0) {
    throw std::runtime_error(context + " has invalid plant values");
  }
  return profile;
}

// 解析 YAML 中 motors 段的通用逻辑。
// 当 contract 不为 nullptr 时，逐项对比 buses/motors 顺序与地址；
// 为 nullptr 时，仅做内部一致性校验（bus 已声明、joint 唯一、地址唯一、ID 合法）。
SystemConfig parse_config_common(const YAML::Node &root,
                                 const TopologyContract *contract) {
  const auto simulation = required_node(root, "motor_simulation", "root");
  if (!simulation.IsMap()) {
    throw std::runtime_error("motor_simulation must be a map");
  }
  SystemConfig config;
  const auto timing = required_node(simulation, "timing", "motor_simulation");
  config.timing.control_hz =
      positive_unsigned(timing, "control_hz", "timing");
  config.timing.vaxor_feedback_hz =
      positive_unsigned(timing, "vaxor_feedback_hz", "timing");
  config.timing.command_timeout_ms =
      positive_unsigned(timing, "command_timeout_ms", "timing");
  config.timing.feedback_period_ms =
      optional_unsigned(timing, "feedback_period_ms", 0U, "timing");
  config.timing.stagger_ms =
      optional_nonnegative_double(timing, "stagger_ms", 0.0, "timing");
  config.timing.effort_rate_hz =
      optional_nonnegative_double(timing, "effort_rate_hz", 0.0, "timing");
  config.timing.can_period_ms =
      optional_unsigned(timing, "can_period_ms", 0U, "timing");

  const auto buses = required_node(simulation, "buses", "motor_simulation");
  if (!buses.IsSequence()) {
    throw std::runtime_error("buses must be a sequence");
  }
  // 串口可选位率的白名单：从 ROS 层搬来（原 validate_port_mapping_common），
  // 因为它是配置取值域，而不是运行期环境事实。
  const std::set<int> kSupportedBaudRates{115200, 460800,  921600,
                                          1000000, 2000000, 3000000};
  std::unordered_set<std::string> bus_names;
  std::unordered_set<std::string> bus_endpoints;
  for (std::size_t index = 0U; index < buses.size(); ++index) {
    const std::string context = "bus[" + std::to_string(index) + "]";
    const auto node = buses[index];
    if (!node.IsMap()) {
      throw std::runtime_error(context +
                               " must be a map with name/transport/endpoint");
    }
    BusConfig bus;
    bus.name = required_string(node, "name", context);
    bus.transport = parse_bus_transport(node, context);
    bus.endpoint = required_string(node, "endpoint", context);
    bus.baud_rate = optional_int(node, "baud_rate", 3000000, context);
    bus.data_bitrate_index =
        optional_int(node, "data_bitrate_index", -1, context);
    bus.brs = optional_bool(node, "brs", true, context);
    bus.probe_adapter = optional_bool(node, "probe_adapter", false, context);
    bus.allow_external_canfd =
        optional_bool(node, "allow_external_canfd", false, context);
    // serial_slcan 会把帧打到真实 CANable 所在的物理总线上，必须由配置显式声明
    // 才允许（原 ROS 层 validate_port_mapping_common 的守卫）。
    if (bus.transport == BusTransport::SerialSlcan &&
        !bus.allow_external_canfd) {
      throw std::runtime_error(context +
                               " serial_slcan requires allow_external_canfd: "
                               "true");
    }
    if (bus.transport == BusTransport::SocketCan) {
      // 接口名不是设备路径；带 '/' 会让下游拿路径去 if_nametoindex，
      // 报出的却是"接口不存在"这种与真因无关的错。
      if (bus.endpoint.find('/') != std::string::npos) {
        throw std::runtime_error(
            context + " socket_can endpoint must be an interface name");
      }
    } else if (kSupportedBaudRates.find(bus.baud_rate) ==
               kSupportedBaudRates.end()) {
      throw std::runtime_error(context + " has unsupported baud_rate");
    }
    if (bus.transport == BusTransport::SerialSlcan &&
        !(bus.data_bitrate_index == -1 ||
          (bus.data_bitrate_index >= 0 && bus.data_bitrate_index <= 9))) {
      throw std::runtime_error(context + " data_bitrate_index must be -1..9");
    }
    if (!bus_names.insert(bus.name).second) {
      throw std::runtime_error("bus names must be nonempty and unique");
    }
    if (!bus_endpoints.insert(bus.endpoint).second) {
      throw std::runtime_error("bus endpoints must be unique");
    }
    config.buses.push_back(std::move(bus));
  }
  if (contract != nullptr) {
    std::vector<std::string> parsed_buses;
    parsed_buses.reserve(config.buses.size());
    for (const auto &bus : config.buses) {
      parsed_buses.push_back(bus.name);
    }
    if (parsed_buses != contract->buses) {
      throw std::runtime_error("buses violate the model topology contract");
    }
  }

  const double margin = optional_finite_double(
      simulation, "command_margin_deg", 0.0, "motor_simulation");
  if (margin < 0.0) {
    throw std::runtime_error("command_margin_deg cannot be negative");
  }
  const auto profiles =
      required_node(simulation, "profiles", "motor_simulation");
  if (!profiles.IsMap() || profiles.size() == 0U) {
    throw std::runtime_error("profiles must be a nonempty map");
  }
  std::unordered_map<std::string, MotorProfile> parsed_profiles;
  for (const auto &entry : profiles) {
    const auto name = entry.first.as<std::string>();
    parsed_profiles.emplace(name, parse_profile(name, entry.second, margin));
  }

  const auto motors = required_node(simulation, "motors", "motor_simulation");
  if (!motors.IsSequence()) {
    throw std::runtime_error("motors must be a sequence");
  }
  if (contract != nullptr && motors.size() != contract->motors.size()) {
    throw std::runtime_error("motors violate the model topology contract");
  }
  std::unordered_set<std::string> joints;
  std::unordered_set<std::string> addresses;
  for (std::size_t index = 0U; index < motors.size(); ++index) {
    const auto node = motors[index];
    const std::string context = "motor[" + std::to_string(index) + "]";
    MotorConfig motor;
    motor.joint = required_string(node, "joint", context);
    motor.bus = required_string(node, "bus", context);
    motor.protocol = required_string(node, "protocol", context);
    motor.id = positive_unsigned(node, "id", context);
    const auto profile_name = required_string(node, "profile", context);
    const auto profile = parsed_profiles.find(profile_name);
    if (profile == parsed_profiles.end()) {
      throw std::runtime_error(context + " references unknown profile");
    }
    if (bus_names.find(motor.bus) == bus_names.end()) {
      throw std::runtime_error(context + " references unknown bus");
    }
    protocol::make_motor_protocol(motor.protocol)->validate_motor_id(motor.id);
    motor.transmission = profile->second.transmission;
    motor.transmission.direction =
        optional_finite_double(node, "direction", 1.0, context);
    motor.transmission.zero_offset_rad = radians(
        optional_finite_double(node, "zero_offset_deg", 0.0, context));
    if (motor.transmission.direction != -1.0 &&
        motor.transmission.direction != 1.0) {
      throw std::runtime_error(context + " direction must be -1 or 1");
    }
    motor.limits = profile->second.limits;
    motor.plant = profile->second.plant;
    motor.mit_torque_limit_nm = profile->second.mit_torque_limit_nm;
    motor.device_identity = profile->second.device_identity;
    motor.device_max_speed_rpm = profile->second.device_max_speed_rpm;
    motor.device_max_current_ma = profile->second.device_max_current_ma;
    // CERB 的 SDO 从站靠这三个量成为固件的合格对端。缺了它们节点会起一个固件
    // 永远绑不上型号的从站，所以在加载期就拒绝——身份串没有合理的缺省。
    if (motor.protocol == "cerb" &&
        (motor.device_identity.empty() || motor.device_max_speed_rpm <= 0 ||
         motor.device_max_current_ma <= 0)) {
      throw std::runtime_error(
          context +
          " cerb motor needs device_identity, device_max_speed_rpm and "
          "device_max_current_ma");
    }
    if (const auto overrides = node["sdo_overrides"]) {
      if (!overrides.IsMap()) {
        throw std::runtime_error(context + ".sdo_overrides must be a map");
      }
      motor.sdo_overrides.protection_enabled = optional_unsigned(
          overrides, "protection_enabled", 1U, context + ".sdo_overrides");
      motor.sdo_overrides.direction = optional_unsigned(
          overrides, "direction", 0U, context + ".sdo_overrides");
      if (motor.sdo_overrides.protection_enabled > 1U ||
          motor.sdo_overrides.direction > 1U) {
        throw std::runtime_error(context +
                                 ".sdo_overrides values must be 0 or 1");
      }
    }
    // Vaxor 与 CERB 的反馈电流都由关节力矩除以 Kt（含传动）换算，
    // 必须提供正的转矩常数与电流限幅。
    if ((motor.protocol == "vaxor" || motor.protocol == "cerb") &&
        (motor.plant.torque_constant_nm_per_a <= 0.0 ||
         motor.plant.current_limit_a <= 0.0)) {
      throw std::runtime_error(context +
                                 " Vaxor/CERB plant needs Kt and current limit");
    }
    const auto address = motor.bus + "\n" + motor.protocol + "\n" +
                         std::to_string(motor.id);
    if (!joints.insert(motor.joint).second ||
        !addresses.insert(address).second) {
      throw std::runtime_error(context + " duplicates a joint or bus address");
    }
    if (contract != nullptr) {
      const auto &required = contract->motors[index];
      if (motor.joint != required.joint || motor.bus != required.bus ||
          motor.protocol != required.protocol || motor.id != required.id) {
        throw std::runtime_error(
            context + " violates the model topology contract");
      }
    }
    config.motors.push_back(std::move(motor));
  }
  if (config.motors.empty()) {
    throw std::runtime_error("motors must be nonempty");
  }
  return config;
}

SystemConfig parse_config(const YAML::Node &root,
                          const TopologyContract &contract) {
  return parse_config_common(root, &contract);
}

}  // namespace

SystemConfig load_system_config(const std::string &path,
                                const TopologyContract &contract) {
  if (path.empty()) {
    throw std::runtime_error("motor configuration path cannot be empty");
  }
  try {
    return parse_config(YAML::LoadFile(path), contract);
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("cannot load motor configuration '" + path +
                             "': " + error.what());
  }
}

SystemConfig load_system_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor configuration path cannot be empty");
  }
  try {
    return parse_config_common(YAML::LoadFile(path), nullptr);
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("cannot load motor configuration '" + path +
                             "': " + error.what());
  }
}

}  // namespace motor_sim::core
