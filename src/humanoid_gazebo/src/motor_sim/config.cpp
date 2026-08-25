#include "humanoid_gazebo/motor_sim/config.hpp"

#include "humanoid_gazebo/protocol/motor_protocol.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace humanoid_gazebo::motor_sim {
namespace {

struct RequiredMotorAddress {
  const char *joint;
  const char *bus;
  const char *protocol;
  unsigned id;
};

constexpr std::array<RequiredMotorAddress, 30> kRequiredMotorOrder{{
    {"left_scapula_protraction_joint", "left_arm", "rp", 1U},
    {"left_scapula_elevation_joint", "left_arm", "rp", 2U},
    {"left_shoulder_pitch_joint", "left_arm", "rp", 3U},
    {"left_shoulder_roll_joint", "left_arm", "rp", 4U},
    {"left_shoulder_yaw_joint", "left_arm", "rp", 5U},
    {"left_elbow_pitch_joint", "left_arm", "rp", 6U},
    {"left_forearm_roll_joint", "left_arm", "motor_2530", 1U},
    {"left_wrist_flexion_joint", "left_arm", "motor_2530", 2U},
    {"left_wrist_deviation_joint", "left_arm", "motor_2530", 3U},
    {"right_scapula_protraction_joint", "right_arm", "rp", 1U},
    {"right_scapula_elevation_joint", "right_arm", "rp", 2U},
    {"right_shoulder_pitch_joint", "right_arm", "rp", 3U},
    {"right_shoulder_roll_joint", "right_arm", "rp", 4U},
    {"right_shoulder_yaw_joint", "right_arm", "rp", 5U},
    {"right_elbow_pitch_joint", "right_arm", "rp", 6U},
    {"right_forearm_roll_joint", "right_arm", "motor_2530", 1U},
    {"right_wrist_flexion_joint", "right_arm", "motor_2530", 2U},
    {"right_wrist_deviation_joint", "right_arm", "motor_2530", 3U},
    {"left_hip_yaw_joint", "legs", "rp", 1U},
    {"left_hip_roll_joint", "legs", "rp", 2U},
    {"left_hip_pitch_joint", "legs", "rp", 3U},
    {"left_knee_pitch_joint", "legs", "rp", 4U},
    {"left_ankle_pitch_joint", "legs", "motor_2530", 1U},
    {"left_ankle_roll_joint", "legs", "motor_2530", 2U},
    {"right_hip_yaw_joint", "legs", "rp", 5U},
    {"right_hip_roll_joint", "legs", "rp", 6U},
    {"right_hip_pitch_joint", "legs", "rp", 7U},
    {"right_knee_pitch_joint", "legs", "rp", 8U},
    {"right_ankle_pitch_joint", "legs", "motor_2530", 3U},
    {"right_ankle_roll_joint", "legs", "motor_2530", 4U},
}};

struct MotorProfile {
  TransmissionConfig transmission;
  JointLimits limits;
  PlantConfig plant;
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

double radians(const double degrees) {
  return degrees * protocol::kPi / 180.0;
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
  if (profile.transmission.gear_ratio <= 0.0 ||
      profile.transmission.efficiency <= 0.0 ||
      profile.transmission.efficiency > 1.0) {
    throw std::runtime_error(context + " has invalid transmission values");
  }

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

SystemConfig parse_config(const YAML::Node &root) {
  const auto simulation = required_node(root, "motor_simulation", "root");
  if (!simulation.IsMap()) {
    throw std::runtime_error("motor_simulation must be a map");
  }
  SystemConfig config;
  const auto timing = required_node(simulation, "timing", "motor_simulation");
  config.timing.control_hz =
      positive_unsigned(timing, "control_hz", "timing");
  config.timing.rp_feedback_hz =
      positive_unsigned(timing, "rp_feedback_hz", "timing");
  config.timing.vaxor_feedback_hz =
      positive_unsigned(timing, "vaxor_feedback_hz", "timing");
  config.timing.command_timeout_ms =
      positive_unsigned(timing, "command_timeout_ms", "timing");

  const auto buses = required_node(simulation, "buses", "motor_simulation");
  if (!buses.IsSequence() || buses.size() != 3U) {
    throw std::runtime_error("buses must contain exactly three names");
  }
  std::unordered_set<std::string> bus_names;
  for (const auto &node : buses) {
    const auto name = node.as<std::string>();
    if (name.empty() || !bus_names.insert(name).second) {
      throw std::runtime_error("bus names must be nonempty and unique");
    }
    config.buses.push_back({name});
  }
  if (config.buses[0].name != "left_arm" ||
      config.buses[1].name != "right_arm" ||
      config.buses[2].name != "legs") {
    throw std::runtime_error("bus order must be left_arm, right_arm, legs");
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
  if (!motors.IsSequence() || motors.size() != kRequiredMotorOrder.size()) {
    throw std::runtime_error("motors must match the fixed 30-axis ABI");
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
    if (motor.protocol == "motor_2530" &&
        (motor.plant.torque_constant_nm_per_a <= 0.0 ||
         motor.plant.current_limit_a <= 0.0)) {
      throw std::runtime_error(context + " Vaxor plant needs Kt and current limit");
    }
    const auto address = motor.bus + "\n" + motor.protocol + "\n" +
                         std::to_string(motor.id);
    if (!joints.insert(motor.joint).second ||
        !addresses.insert(address).second) {
      throw std::runtime_error(context + " duplicates a joint or bus address");
    }
    const auto &required = kRequiredMotorOrder[index];
    if (motor.joint != required.joint || motor.bus != required.bus ||
        motor.protocol != required.protocol || motor.id != required.id) {
      throw std::runtime_error(context + " violates the fixed 30-axis ABI");
    }
    config.motors.push_back(std::move(motor));
  }
  return config;
}

}  // namespace

SystemConfig load_system_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor configuration path cannot be empty");
  }
  try {
    return parse_config(YAML::LoadFile(path));
  } catch (const YAML::Exception &error) {
    throw std::runtime_error("cannot load motor configuration '" + path +
                             "': " + error.what());
  }
}

}  // namespace humanoid_gazebo::motor_sim
