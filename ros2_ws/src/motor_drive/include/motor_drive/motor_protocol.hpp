#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_drive/motor_types.hpp"
#include "motor_drive/protocol.hpp"
#include "motor_drive/can_transport.hpp"

namespace motor_drive {

using MotorIndexList = std::vector<std::size_t>;

// MotorProtocol 是“协议策略”抽象：每一种电机家族只负责自己的线协议规则。
// 通用 MotorDriver 不需要知道 CAN ID、字节布局或该协议一次能带多少台电机。
class MotorProtocol {
public:
  virtual ~MotorProtocol() = default;

  virtual void validate(const std::vector<MotorConfig> &motors,
                        const MotorIndexList &indices) const = 0;
  virtual std::vector<CanFrame>
  make_enable_frames(const std::vector<MotorConfig> &motors,
                     const MotorIndexList &indices, bool enabled) const = 0;
  virtual std::vector<CanFrame>
  make_command_frames(const std::vector<MotorConfig> &motors,
                      const MotorIndexList &indices,
                      const std::vector<protocol::JointCommand> &commands,
                      bool enabled) const = 0;

  // 只有当该协议识别出 frame，并成功解码出某台已配置电机的反馈时才返回 true。
  // motor_index 返回这台电机在 DriverConfig::motors 中的全局下标，供上层统一写状态。
  virtual bool decode_feedback(const std::vector<MotorConfig> &motors,
                               const MotorIndexList &indices,
                               const CanFrame &frame,
                               std::size_t &motor_index,
                               protocol::JointState &state) const = 0;
};

class MotorProtocolRegistry {
public:
  using Factory = std::function<std::unique_ptr<MotorProtocol>()>;

  void register_factory(std::string type, Factory factory);
  bool contains(const std::string &type) const noexcept;
  std::unique_ptr<MotorProtocol> create(const std::string &type) const;

  // 内置协议在 registry 中注册，而不是硬编码到 MotorDriver 的收发循环里。
  // 未来增加电机家族时，实现协议策略并增加一项工厂注册即可。
  static MotorProtocolRegistry with_builtin_protocols();

private:
  std::unordered_map<std::string, Factory> factories_;
};

inline constexpr const char *kVaxorType = "vaxor";
inline constexpr const char *kCrebType = "creb";

} // namespace motor_drive
