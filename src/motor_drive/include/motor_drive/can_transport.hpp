#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "motor_drive/motor_types.hpp"

namespace motor_drive {

// 对 Linux classic CAN / CAN-FD 帧做统一封装；协议层无需直接依赖 linux/can.h。
struct CanFrame {
  uint32_t id{0U};
  std::array<uint8_t, 64> data{};
  std::size_t size{0U};
  bool is_fd{false};
  bool bitrate_switch{false};
  bool is_extended{false};
};

// CAN 传输抽象：驱动只依赖 send/receive，不关心底层是 SocketCAN 内核接口
// 还是未来的其他后端。新增传输只需实现本接口并注册到工厂。
class CanTransport {
public:
  virtual ~CanTransport() = default;

  // 发送一帧；失败返回 false（不抛异常，调用方按控制周期容错）。
  virtual bool send(uint32_t id, const uint8_t *data, std::size_t size,
                    bool force_fd = false, bool bitrate_switch = true,
                    bool is_extended = false) const = 0;

  // 非阻塞接收一帧：有帧返回 true 并填充 output，无帧返回 false。
  // 传输层错误抛 std::runtime_error。
  virtual bool receive(CanFrame &output) const = 0;
};

// 传输工厂注册表：按 BusConfig.transport 字符串创建对应后端。
// 与 MotorProtocolRegistry 对称——新增传输只需实现 CanTransport 并注册工厂，
// 不需要修改 MotorDriver 的打开/收发循环。
class CanTransportRegistry {
public:
  using Factory = std::function<std::unique_ptr<CanTransport>(const BusConfig &)>;

  void register_factory(std::string transport, Factory factory);
  bool contains(const std::string &transport) const noexcept;
  std::unique_ptr<CanTransport> create(const BusConfig &bus) const;

  // 内置传输在 registry 中注册：socketcan（Linux 内核接口）与
  // serial_slcan（CANable2 串口方言）。未来增加后端时增加一项工厂注册即可。
  static CanTransportRegistry with_builtin_transports();

private:
  std::unordered_map<std::string, Factory> factories_;
};

inline constexpr const char *kSocketCanTransport = "socketcan";
inline constexpr const char *kSerialSlcanTransport = "serial_slcan";

} // namespace motor_drive
