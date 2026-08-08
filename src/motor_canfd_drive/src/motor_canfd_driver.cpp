#include "motor_canfd_drive/motor_canfd_driver.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace motor_canfd_drive {

MotorCanfdDriver::MotorCanfdDriver(DriverConfig config)
    : MotorCanfdDriver(std::move(config),
                       MotorProtocolRegistry::with_builtin_protocols()) {}

MotorCanfdDriver::MotorCanfdDriver(DriverConfig config,
                                   MotorProtocolRegistry registry)
    : config_(std::move(config)) {
  validate_and_build(std::move(registry));
}

void MotorCanfdDriver::validate_and_build(MotorProtocolRegistry registry) {
  if (config_.buses.empty()) {
    throw std::invalid_argument("at least one CAN bus must be configured");
  }
  if (config_.motors.empty()) {
    throw std::invalid_argument("at least one motor must be configured");
  }

  std::unordered_map<std::string, std::size_t> bus_index;
  std::unordered_set<std::string> interfaces;
  buses_.reserve(config_.buses.size());
  for (const auto &bus : config_.buses) {
    if (bus.name.empty() || bus.interface_name.empty()) {
      throw std::invalid_argument("CAN bus name and interface cannot be empty");
    }
    if (!bus_index.emplace(bus.name, buses_.size()).second) {
      throw std::invalid_argument("duplicate CAN bus name '" + bus.name + "'");
    }
    if (!interfaces.insert(bus.interface_name).second) {
      throw std::invalid_argument("SocketCAN interface '" + bus.interface_name +
                                  "' is assigned to more than one logical bus");
    }
    BusRuntime runtime;
    runtime.config = bus;
    buses_.push_back(std::move(runtime));
  }

  std::unordered_set<std::string> motor_names;
  std::unordered_set<std::string> joint_names;
  std::vector<std::map<std::string, MotorIndexList>> grouped(buses_.size());
  for (std::size_t index = 0; index < config_.motors.size(); ++index) {
    const auto &motor = config_.motors[index];
    if (motor.name.empty() || motor.joint_name.empty() || motor.bus.empty() ||
        motor.type.empty()) {
      throw std::invalid_argument(
          "motor name, joint, bus and type are required for every motor");
    }
    if (!motor_names.insert(motor.name).second) {
      throw std::invalid_argument("duplicate motor name '" + motor.name + "'");
    }
    if (!joint_names.insert(motor.joint_name).second) {
      throw std::invalid_argument("joint '" + motor.joint_name +
                                  "' is mapped to more than one motor");
    }
    const auto bus_it = bus_index.find(motor.bus);
    if (bus_it == bus_index.end()) {
      throw std::invalid_argument("motor '" + motor.name +
                                  "' references unknown bus '" + motor.bus + "'");
    }
    if (!registry.contains(motor.type)) {
      throw std::invalid_argument("motor '" + motor.name +
                                  "' uses unregistered type '" + motor.type + "'");
    }
    grouped[bus_it->second][motor.type].push_back(index);
  }

  for (std::size_t bus = 0; bus < buses_.size(); ++bus) {
    for (auto &entry : grouped[bus]) {
      ProtocolGroup group;
      group.type = entry.first;
      group.motor_indices = std::move(entry.second);
      group.protocol = registry.create(group.type);
      group.protocol->validate(config_.motors, group.motor_indices);
      buses_[bus].groups.push_back(std::move(group));
    }
  }
}

void MotorCanfdDriver::open() {
  try {
    for (auto &bus : buses_) {
      if (!bus.socket) {
        bus.socket = std::make_unique<SocketCan>(bus.config.interface_name);
      }
    }
  } catch (...) {
    close();
    throw;
  }
}

void MotorCanfdDriver::close() noexcept {
  for (auto &bus : buses_) {
    bus.socket.reset();
  }
}

bool MotorCanfdDriver::is_open() const noexcept {
  return !buses_.empty() &&
         std::all_of(buses_.begin(), buses_.end(),
                     [](const auto &bus) { return static_cast<bool>(bus.socket); });
}

bool MotorCanfdDriver::send_frame(const SocketCan &socket, const CanFrame &frame) {
  return socket.send(frame.id, frame.data.data(), frame.size, frame.is_fd,
                     frame.bitrate_switch, frame.is_extended);
}

bool MotorCanfdDriver::set_enabled(const bool enabled) {
  if (!is_open()) {
    return false;
  }
  bool ok = true;
  for (const auto &bus : buses_) {
    for (const auto &group : bus.groups) {
      const auto frames = group.protocol->make_enable_frames(
          config_.motors, group.motor_indices, enabled);
      for (const auto &frame : frames) {
        ok &= send_frame(*bus.socket, frame);
      }
    }
  }
  return ok;
}

bool MotorCanfdDriver::write_commands(
    const std::vector<protocol::JointCommand> &commands) {
  if (!is_open() || commands.size() != config_.motors.size()) {
    return false;
  }
  bool ok = true;
  for (const auto &bus : buses_) {
    for (const auto &group : bus.groups) {
      const auto frames = group.protocol->make_command_frames(
          config_.motors, group.motor_indices, commands);
      for (const auto &frame : frames) {
        ok &= send_frame(*bus.socket, frame);
      }
    }
  }
  return ok;
}

std::size_t MotorCanfdDriver::read_feedback(
    std::vector<protocol::JointState> &states, std::vector<bool> &updated) {
  if (states.size() != config_.motors.size()) {
    states.resize(config_.motors.size());
  }
  updated.assign(config_.motors.size(), false);
  if (!is_open()) {
    return 0U;
  }

  std::size_t count = 0U;
  for (const auto &bus : buses_) {
    CanFrame frame;
    while (bus.socket->receive(frame)) {
      for (const auto &group : bus.groups) {
        std::size_t motor_index = 0U;
        protocol::JointState state;
        if (group.protocol->decode_feedback(config_.motors, group.motor_indices,
                                            frame, motor_index, state)) {
          states.at(motor_index) = state;
          updated.at(motor_index) = true;
          ++count;
          break;
        }
      }
    }
  }
  return count;
}

} // namespace motor_canfd_drive
