#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "motor_drive/can_transport.hpp"

namespace motor_drive {

// SocketCan 只负责 Linux SocketCAN 收发，不理解“电机”“关节”或任何私有协议。
// 构造函数打开并绑定接口；对象析构时自动关闭文件描述符（RAII）。
class SocketCan final : public CanTransport {
public:
  explicit SocketCan(const std::string &interface_name);
  ~SocketCan() override;

  SocketCan(const SocketCan &) = delete;
  SocketCan &operator=(const SocketCan &) = delete;
  SocketCan(SocketCan &&other) noexcept;
  SocketCan &operator=(SocketCan &&other) noexcept;

  bool send(uint32_t id, const uint8_t *data, std::size_t size,
            bool force_fd = false, bool bitrate_switch = true,
            bool is_extended = false) const override;
  bool receive(CanFrame &frame) const override;

private:
  void close_socket() noexcept;
  int fd_{-1};
};

} // namespace motor_drive
