#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_canfd_drive/motor_types.hpp"
#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"

namespace motor_canfd_drive {

using MotorIndexList = std::vector<std::size_t>;

// A protocol strategy owns the wire rules for one motor family on one CAN bus.
// The generic driver knows neither frame IDs nor packet layouts.
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
                      const std::vector<protocol::JointCommand> &commands) const = 0;

  // Return true only when this protocol recognizes the frame and produces a
  // state for one configured motor. motor_index is the global DriverConfig index.
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

  // Built-ins are registered here instead of being hard-coded in the generic
  // driver. A future motor family adds a strategy + one registration entry.
  static MotorProtocolRegistry with_builtin_protocols();

private:
  std::unordered_map<std::string, Factory> factories_;
};

inline constexpr const char *kRpMotorType = "rp";
inline constexpr const char *kMotor2530Type = "motor_2530";

} // namespace motor_canfd_drive
