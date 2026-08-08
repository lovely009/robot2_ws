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

  // 第一步：建立“逻辑总线名 -> 运行时下标”。同一个 SocketCAN 接口不能被两个
  // 逻辑总线重复占用，否则接收帧归属会变得不确定。
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

  // 第二步：逐台验证电机，并按 (bus, type) 分组。
  // 这里完全不关心某条机械臂有几个关节，也不假设某种电机位于数组的固定区间。
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

  // 第三步：为每个协议组创建策略对象，并把协议特有的 ID/数量/参数校验
  // 交还给对应 MotorProtocol。通用 driver 因此无需写 type 判断分支。
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
    // 每条逻辑总线只打开一个 SocketCAN socket。任何一条总线打开失败时，
    // catch 会统一 close，避免留下“部分总线已打开”的半初始化状态。
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
  // 使能帧由协议策略生成：RP 可能是一条广播帧，25/30 mm 则可能每台一帧。
  // 通用层只负责把策略返回的帧发送到正确的总线。
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
  // commands 的下标始终对应 DriverConfig::motors 的全局下标；协议策略通过
  // motor_indices 只取本组电机，再自行决定广播还是逐节点组帧。
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
  // SocketCAN 使用非阻塞读取：把当前已经到达的帧一次排空。
  // 每帧按同一总线上的协议组依次尝试解码；首个识别成功的策略负责该帧。
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
