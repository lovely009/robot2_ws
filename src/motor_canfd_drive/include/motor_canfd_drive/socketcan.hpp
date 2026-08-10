#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace motor_canfd_drive {

// 对 Linux classic CAN / CAN-FD 帧做统一封装；协议层无需直接依赖 linux/can.h。
struct CanFrame {
  uint32_t id{0U};
  std::array<uint8_t, 64> data{};
  std::size_t size{0U};
  bool is_fd{false};
  bool bitrate_switch{false};
  bool is_extended{false};
};

// SocketCan 只负责 Linux SocketCAN 收发，不理解“电机”“关节”或任何私有协议。
// 构造函数打开并绑定接口；对象析构时自动关闭文件描述符（RAII）。
class SocketCan final {
public:
  explicit SocketCan(const std::string &interface_name);
  ~SocketCan();

  SocketCan(const SocketCan &) = delete;
  SocketCan &operator=(const SocketCan &) = delete;
  SocketCan(SocketCan &&other) noexcept;
  SocketCan &operator=(SocketCan &&other) noexcept;

  bool send(uint32_t id, const uint8_t *data, std::size_t size,
            bool force_fd = false, bool bitrate_switch = true,
            bool is_extended = false) const;
  bool receive(CanFrame &frame) const;
  int native_handle() const noexcept { return fd_; }

private:
  void close_socket() noexcept;
  int fd_{-1};
};

} // namespace motor_canfd_drive
