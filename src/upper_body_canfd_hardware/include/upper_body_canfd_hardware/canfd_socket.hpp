// Minimal dependency-free SocketCAN / CAN-FD wrapper (Linux raw socket).
//
// Mirrors the semantics of upper_body_canfd_gazebo/socketcan.py: non-blocking
// receive, CAN-FD enabled, optional receive-own-messages filtering, and the
// same 11-bit/29-bit ID handling.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace upper_body_canfd_hardware {

struct CanFrame {
  uint32_t arbitration_id = 0;
  std::vector<uint8_t> data;
  bool is_fd = false;
  bool bitrate_switch = false;
  bool is_extended_id = false;
};

class SocketCanBus {
public:
  SocketCanBus() = default;

  // receive_own_messages: when false the socket drops frames it transmitted
  // itself (default for the hardware plugin; true for loopback tests).
  explicit SocketCanBus(const std::string & interface, bool receive_own_messages = false);
  ~SocketCanBus();

  SocketCanBus(const SocketCanBus &) = delete;
  SocketCanBus & operator=(const SocketCanBus &) = delete;
  SocketCanBus(SocketCanBus && other) noexcept;
  SocketCanBus & operator=(SocketCanBus && other) noexcept;

  bool IsOpen() const { return fd_ >= 0; }
  void Close();

  bool Send(
    uint32_t arbitration_id, const std::vector<uint8_t> & data,
    bool force_fd = false, bool bitrate_switch = true,
    bool is_extended_id = false);

  std::vector<CanFrame> RecvAvailable(int max_frames = 128);

  const std::string & interface() const { return interface_; }

private:
  int fd_ = -1;
  std::string interface_;
};

}  // namespace upper_body_canfd_hardware
