#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "motor_canfd_drive/motor_protocol.hpp"
#include "motor_canfd_drive/motor_types.hpp"
#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"

namespace motor_canfd_drive {

// Generic, ROS-independent CAN-FD driver.  Motor count, bus count, IDs and
// motor families all come from DriverConfig; no robot topology is compiled in.
class MotorCanfdDriver final {
public:
  explicit MotorCanfdDriver(DriverConfig config);
  MotorCanfdDriver(DriverConfig config, MotorProtocolRegistry registry);
  ~MotorCanfdDriver() = default;

  MotorCanfdDriver(const MotorCanfdDriver &) = delete;
  MotorCanfdDriver &operator=(const MotorCanfdDriver &) = delete;

  void open();
  void close() noexcept;
  bool is_open() const noexcept;

  bool set_enabled(bool enabled);
  bool write_commands(const std::vector<protocol::JointCommand> &commands);
  std::size_t read_feedback(std::vector<protocol::JointState> &states,
                            std::vector<bool> &updated);

  const DriverConfig &config() const noexcept { return config_; }
  std::size_t motor_count() const noexcept { return config_.motors.size(); }

private:
  struct ProtocolGroup {
    std::string type;
    MotorIndexList motor_indices;
    std::unique_ptr<MotorProtocol> protocol;
  };

  struct BusRuntime {
    BusConfig config;
    std::vector<ProtocolGroup> groups;
    std::unique_ptr<SocketCan> socket;
  };

  void validate_and_build(MotorProtocolRegistry registry);
  static bool send_frame(const SocketCan &socket, const CanFrame &frame);

  DriverConfig config_;
  std::vector<BusRuntime> buses_;
};

} // namespace motor_canfd_drive
