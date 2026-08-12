#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace motor_canfd_drive {

// 对 Linux classic CAN / CAN-FD 帧做统一封装；协议层无需直接依赖 linux/can.h。
// 同时作为 SocketCan 与 SerialSlcanTransport 的公共帧格式。
struct CanFrame {
  uint32_t id{0U};
  std::array<uint8_t, 64> data{};
  std::size_t size{0U};
  bool is_fd{false};
  bool bitrate_switch{false};
  bool is_extended{false};
};

// CAN 传输抽象：驱动只依赖 send/receive，不关心底层是 SocketCAN 内核接口
// 还是 CANable2 串口 slcan 方言。新增传输只需实现本接口并注册到工厂。
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

} // namespace motor_canfd_drive
