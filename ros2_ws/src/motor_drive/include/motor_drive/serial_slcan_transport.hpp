#pragma once

#include <string>

#include "motor_drive/can_transport.hpp"

namespace motor_drive {

// CANable2（slcan 固件）串口传输：把 SocketCAN 语义映射到 /dev/ttyACM0 的
// slcan-FD 方言。构造时打开串口并按项目位速配置总线：
//   C（关通道）→ S8（仲裁 1M）→ Y5（数据 5M）→ M0 → A1 → O（打开通道）。
// 不需要 SocketCAN 接口，也不需要 NET_ADMIN；只需 tty 写权限（dialout/root）。
// Every complete line has a 5 ms write budget. An I/O failure closes the stream;
// this instance cannot resume or append another command to a partial line.
class SerialSlcanTransport final : public CanTransport {
public:
  // nominal_index/data_index 为 slcan 位速索引（S 命令 0-8，Y 命令 2/5）。
  // data_index == 0 时不下发 Y 命令（仅经典 CAN 场景）。
  explicit SerialSlcanTransport(const std::string &tty_path,
                                unsigned nominal_index = 8U,
                                unsigned data_index = 5U);
  ~SerialSlcanTransport() override;

  SerialSlcanTransport(const SerialSlcanTransport &) = delete;
  SerialSlcanTransport &operator=(const SerialSlcanTransport &) = delete;

  bool send(uint32_t id, const uint8_t *data, std::size_t size,
            bool force_fd = false, bool bitrate_switch = true,
            bool is_extended = false) const override;
  bool receive(CanFrame &output) const override;

private:
  bool write_all(const std::string &data) const;
  void close_serial() const noexcept;
  void configure_bus();

  // A failed/partial ASCII write invalidates the stream: later sends must not
  // append another command to a prefix already delivered to the adapter.
  mutable int fd_{-1};
  unsigned nominal_index_{8U};
  unsigned data_index_{5U};
  mutable std::string rx_buffer_;
};

} // namespace motor_drive
