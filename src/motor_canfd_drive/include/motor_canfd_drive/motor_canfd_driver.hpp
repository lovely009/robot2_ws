#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "motor_canfd_drive/can_transport.hpp"
#include "motor_canfd_drive/motor_protocol.hpp"
#include "motor_canfd_drive/motor_types.hpp"
#include "motor_canfd_drive/protocol.hpp"

namespace motor_canfd_drive {

// 与 ROS 无关的通用 CAN-FD 驱动。
// 电机数量、总线数量、ID、电机类型全部来自 DriverConfig，编译代码中不包含机器人拓扑。
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
  // 同一条总线、同一种 type 的电机组成一个协议组。
  // 例如 can0 上的 4 台 RP 是一组，can0 上的 3 台 motor_2530 是另一组。
  struct ProtocolGroup {
    std::string type;
    MotorIndexList motor_indices;
    std::unique_ptr<MotorProtocol> protocol;
  };

  // 每个逻辑总线运行时只打开一个 CAN 传输，再挂接若干协议组。
  struct BusRuntime {
    BusConfig config;
    std::vector<ProtocolGroup> groups;
    std::unique_ptr<CanTransport> transport;
  };

  // 在真正打开 CAN 前一次性验证配置并建立运行时分组，尽早暴露重复 ID/未知类型等错误。
  void validate_and_build(MotorProtocolRegistry registry);
  static bool send_frame(const CanTransport &transport, const CanFrame &frame);

  DriverConfig config_;
  std::vector<BusRuntime> buses_;
};

} // namespace motor_canfd_drive
