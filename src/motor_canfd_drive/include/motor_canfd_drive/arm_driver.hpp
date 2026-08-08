#pragma once

#include <array>
#include <memory>
#include <string>

#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"

namespace motor_canfd_drive {

struct ArmDriverConfig {
  std::string interface_name;
  std::array<unsigned, 4> rp_ids{{1U, 2U, 3U, 4U}};
  std::array<unsigned, 3> wrist_nodes{{1U, 2U, 3U}};
  double wrist_torque_constant_nm_per_amp{0.35};
};

class ArmDriver final {
public:
  static constexpr std::size_t kJointCount = 7U;
  using CommandArray = std::array<protocol::JointCommand, kJointCount>;
  using StateArray = std::array<protocol::JointState, kJointCount>;
  using UpdatedArray = std::array<bool, kJointCount>;

  explicit ArmDriver(ArmDriverConfig config);
  ~ArmDriver() = default;

  ArmDriver(const ArmDriver &) = delete;
  ArmDriver &operator=(const ArmDriver &) = delete;

  void open();
  void close() noexcept;
  bool is_open() const noexcept { return static_cast<bool>(bus_); }

  bool set_enabled(bool enabled);
  bool write_commands(const CommandArray &commands);
  std::size_t read_feedback(StateArray &states, UpdatedArray &updated);

  const ArmDriverConfig &config() const noexcept { return config_; }

private:
  void validate_config() const;
  ArmDriverConfig config_;
  std::unique_ptr<SocketCan> bus_;
};

} // namespace motor_canfd_drive
