#include "motor_drive/motor_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace motor_drive {
namespace {

CanFrame make_frame(const uint32_t id, const uint8_t *data,
                    const std::size_t size, const bool is_fd) {
  if (data == nullptr || size > CanFrame{}.data.size()) {
    throw std::invalid_argument("invalid CAN frame payload");
  }
  CanFrame frame;
  frame.id = id;
  frame.size = size;
  frame.is_fd = is_fd;
  frame.bitrate_switch = is_fd;
  std::copy(data, data + size, frame.data.begin());
  return frame;
}

double required_positive_parameter(const MotorConfig &motor,
                                   const std::string &key) {
  const auto it = motor.parameters.find(key);
  if (it == motor.parameters.end()) {
    throw std::invalid_argument("motor '" + motor.name + "' (type " + motor.type +
                                ") requires parameter '" + key + "'");
  }
  std::size_t parsed = 0U;
  const double value = std::stod(it->second, &parsed);
  if (parsed != it->second.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument("motor '" + motor.name + "' parameter '" + key +
                                "' must be a positive number");
  }
  return value;
}

// 可选正数参数：缺省返回 fallback；配置时校验为正数（与 required_ 对称）。
double optional_positive_parameter(const MotorConfig &motor,
                                   const std::string &key,
                                   const double fallback) {
  const auto it = motor.parameters.find(key);
  if (it == motor.parameters.end()) {
    return fallback;
  }
  std::size_t parsed = 0U;
  const double value = std::stod(it->second, &parsed);
  if (parsed != it->second.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument("motor '" + motor.name + "' parameter '" + key +
                                "' must be a positive number");
  }
  return value;
}

class RpMotorProtocol final : public MotorProtocol {
public:
  void validate(const std::vector<MotorConfig> &motors,
                const MotorIndexList &indices) const override {
    // RP 的 1..8 限制来自一条 64 Byte 广播帧只有 8 个 MIT 槽位。
    // 这个限制只作用于“同一 CAN 总线上的 RP 协议组”，不是整机电机数限制。
    if (indices.empty() || indices.size() > protocol::kRpBroadcastSlots) {
      throw std::invalid_argument("RP protocol supports 1..8 configured motors per CAN bus");
    }
    std::unordered_set<unsigned> ids;
    for (const auto index : indices) {
      const auto &motor = motors.at(index);
      const auto id = motor.id;
      if (id < 1U || id > protocol::kRpBroadcastSlots || !ids.insert(id).second) {
        throw std::invalid_argument(
            "RP IDs on one CAN bus must be unique values in [1, 8]");
      }
    }
  }

  std::vector<CanFrame>
  make_enable_frames(const std::vector<MotorConfig> &, const MotorIndexList &,
                     const bool enabled) const override {
    const std::array<uint8_t, 8> payload{
        {static_cast<uint8_t>(enabled), 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
    return {make_frame(protocol::kRpEnableId, payload.data(), payload.size(), false)};
  }

  std::vector<CanFrame>
  make_command_frames(const std::vector<MotorConfig> &motors,
                      const MotorIndexList &indices,
                      const std::vector<protocol::JointCommand> &commands) const override {
    std::array<uint8_t, 64> payload{};
    // RP 命令固定发送完整 8 槽广播。未配置的槽位填中性命令，已配置电机则按
    // motor.id - 1 放进自己的协议槽位，因此 YAML 顺序不会改变 RP ID 映射。
    const auto neutral = protocol::pack_rp_slot({});
    for (std::size_t slot = 0; slot < protocol::kRpBroadcastSlots; ++slot) {
      std::copy(neutral.begin(), neutral.end(), payload.begin() + slot * 8U);
    }
    for (const auto index : indices) {
      const auto packed = protocol::pack_rp_slot(commands.at(index));
      const auto slot = motors.at(index).id - 1U;
      std::copy(packed.begin(), packed.end(), payload.begin() + slot * 8U);
    }
    return {make_frame(protocol::kRpCommandId, payload.data(), payload.size(), true)};
  }

  bool decode_feedback(const std::vector<MotorConfig> &motors,
                       const MotorIndexList &indices, const CanFrame &frame,
                       std::size_t &motor_index,
                       protocol::JointState &state) const override {
    if (frame.is_extended || frame.size != 12U) {
      return false;
    }
    for (const auto index : indices) {
      if (frame.id == motors.at(index).id) {
        state = protocol::unpack_rp_state(frame.data.data(), frame.size);
        motor_index = index;
        return true;
      }
    }
    return false;
  }
};

class VaxorProtocol final : public MotorProtocol {
public:
  void validate(const std::vector<MotorConfig> &motors,
                const MotorIndexList &indices) const override {
    if (indices.empty()) {
      throw std::invalid_argument("vaxor protocol group cannot be empty");
    }
    std::unordered_set<unsigned> ids;
    for (const auto index : indices) {
      const auto &motor = motors.at(index);
      protocol::validate_wrist_node(motor.id);
      if (!ids.insert(motor.id).second) {
        throw std::invalid_argument(
            "25/30 mm motor IDs on one CAN bus must be unique values in [1, 60]");
      }
      static_cast<void>(required_positive_parameter(
          motor, "torque_constant_nm_per_amp"));
      // 可选换算参数：gear_ratio / gear_efficiency 配置时校验正值；
      // motor_type_code 配置时校验合法范围。
      static_cast<void>(optional_positive_parameter(motor, "gear_ratio", 1.0));
      static_cast<void>(optional_positive_parameter(motor, "gear_efficiency", 1.0));
      const auto type_code = motor.parameters.find("motor_type_code");
      if (type_code != motor.parameters.end()) {
        const unsigned code = static_cast<unsigned>(
            std::stoul(type_code->second, nullptr, 0));
        if (code < 0x01U || code > 0x04U) {
          throw std::invalid_argument(
              "motor '" + motor.name + "' parameter 'motor_type_code' "
              "must be in [0x01, 0x04] (30/25/20/16 mm)");
        }
      }
    }
  }

  std::vector<CanFrame>
  make_enable_frames(const std::vector<MotorConfig> &motors,
                     const MotorIndexList &indices,
                     const bool enabled) const override {
    std::vector<CanFrame> frames;
    frames.reserve(indices.size());
    const auto payload = protocol::pack_wrist_control(enabled);
    for (const auto index : indices) {
      const auto id = motors.at(index).id;
      frames.push_back(make_frame(protocol::wrist_control_id(id), payload.data(),
                                  payload.size(), false));
    }
    return frames;
  }

  std::vector<CanFrame>
  make_command_frames(const std::vector<MotorConfig> &motors,
                      const MotorIndexList &indices,
                      const std::vector<protocol::JointCommand> &commands) const override {
    std::vector<CanFrame> frames;
    frames.reserve(indices.size());
    for (const auto index : indices) {
      const auto &motor = motors.at(index);
      // 关节侧命令换算为电机侧：position/velocity 乘减速比
      // （kp/kd/前馈电流为电机侧量，不变）。
      const double gear = optional_positive_parameter(motor, "gear_ratio", 1.0);
      auto command = commands.at(index);
      command.position *= gear;
      command.velocity *= gear;
      const auto payload = protocol::pack_wrist_command(command);
      frames.push_back(make_frame(protocol::wrist_command_id(motor.id),
                                  payload.data(), payload.size(), true));
    }
    return frames;
  }

  bool decode_feedback(const std::vector<MotorConfig> &motors,
                       const MotorIndexList &indices, const CanFrame &frame,
                       std::size_t &motor_index,
                       protocol::JointState &state) const override {
    if (frame.is_extended) {
      return false;
    }
    for (const auto index : indices) {
      const auto &motor = motors.at(index);
      if ((frame.id == protocol::wrist_reply_id(motor.id) && frame.size == 12U) ||
          (frame.id == protocol::wrist_status_id(motor.id) && frame.size == 20U)) {
        state = protocol::unpack_wrist_feedback(frame.data.data(), frame.size);
        // 反馈给的是电流 A，统一 JointState::effort 对上层表示关节侧 Nm：
        // 先由 Kt 得电机轴 Nm，再乘减速比与效率折算关节侧；位置/速度除以减速比。
        const double gear = optional_positive_parameter(motor, "gear_ratio", 1.0);
        const double efficiency = optional_positive_parameter(motor, "gear_efficiency", 1.0);
        state.effort *= required_positive_parameter(
            motor, "torque_constant_nm_per_amp") * gear * efficiency;
        state.position /= gear;
        state.velocity /= gear;
        motor_index = index;
        return true;
      }
    }
    return false;
  }
};

} // namespace

void MotorProtocolRegistry::register_factory(std::string type, Factory factory) {
  if (type.empty() || !factory) {
    throw std::invalid_argument("protocol type and factory must be valid");
  }
  if (!factories_.emplace(std::move(type), std::move(factory)).second) {
    throw std::invalid_argument("protocol type is already registered");
  }
}

bool MotorProtocolRegistry::contains(const std::string &type) const noexcept {
  return factories_.find(type) != factories_.end();
}

std::unique_ptr<MotorProtocol>
MotorProtocolRegistry::create(const std::string &type) const {
  const auto it = factories_.find(type);
  if (it == factories_.end()) {
    throw std::invalid_argument("unknown motor protocol type '" + type + "'");
  }
  auto protocol = it->second();
  if (!protocol) {
    throw std::runtime_error("protocol factory returned null for type '" + type + "'");
  }
  return protocol;
}

MotorProtocolRegistry MotorProtocolRegistry::with_builtin_protocols() {
  MotorProtocolRegistry registry;
  // 内置类型只在这里注册。以后增加新电机协议时，不需要修改 MotorDriver。
  registry.register_factory(kRpMotorType,
                            [] { return std::make_unique<RpMotorProtocol>(); });
  registry.register_factory(kVaxorType,
                            [] { return std::make_unique<VaxorProtocol>(); });
  return registry;
}

} // namespace motor_drive
