#include "upper_body_canfd_hardware/canfd_socket.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace upper_body_canfd_hardware {

namespace {

constexpr uint32_t kCanEffFlag = 0x80000000;
constexpr uint32_t kCanSffMask = 0x000007FF;
constexpr uint32_t kCanEffMask = 0x1FFFFFFF;
constexpr uint32_t kCanFdBrs = 0x01;

}  // namespace

SocketCanBus::SocketCanBus(const std::string & interface, bool receive_own_messages)
: interface_(interface)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error(
      "failed to open raw CAN socket: " + std::string(std::strerror(errno)));
  }

  int enable_fd = 1;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(
      "failed to enable CAN-FD frames: " + std::string(std::strerror(errno)));
  }

  int receive_own = receive_own_messages ? 1 : 0;
  if (::setsockopt(
        fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own, sizeof(receive_own)) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(
      "failed to configure receive-own-messages: " + std::string(std::strerror(errno)));
  }

  const int ifindex = ::if_nametoindex(interface.c_str());
  if (ifindex == 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("unknown CAN interface: " + interface);
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifindex;
  if (::bind(fd_, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
    const std::string error = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("failed to bind CAN socket to " + interface + ": " + error);
  }

  // Non-blocking receive; send uses MSG_DONTWAIT so neither path blocks the
  // ros2_control real-time update loop.
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

SocketCanBus::~SocketCanBus()
{
  Close();
}

SocketCanBus::SocketCanBus(SocketCanBus && other) noexcept
: fd_(other.fd_), interface_(std::move(other.interface_))
{
  other.fd_ = -1;
}

SocketCanBus & SocketCanBus::operator=(SocketCanBus && other) noexcept
{
  if (this != &other) {
    Close();
    fd_ = other.fd_;
    interface_ = std::move(other.interface_);
    other.fd_ = -1;
  }
  return *this;
}

void SocketCanBus::Close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanBus::Send(
  uint32_t arbitration_id, const std::vector<uint8_t> & data,
  bool force_fd, bool bitrate_switch, bool is_extended_id)
{
  if (fd_ < 0) {
    return false;
  }
  if (data.size() > 64) {
    throw std::invalid_argument("CAN-FD payload cannot exceed 64 bytes");
  }

  uint32_t can_id = arbitration_id;
  if (is_extended_id) {
    if (can_id > kCanEffMask) {
      throw std::invalid_argument("29-bit CAN ID out of range");
    }
    can_id |= kCanEffFlag;
  } else if (can_id > kCanSffMask) {
    throw std::invalid_argument("11-bit CAN ID out of range");
  }

  const bool use_fd = (data.size() > 8) ? true : force_fd;
  ssize_t sent = 0;
  if (!use_fd) {
    if (data.size() > 8) {
      throw std::invalid_argument("Classic CAN payload cannot exceed 8 bytes");
    }
    can_frame frame{};
    frame.can_id = can_id;
    frame.len = static_cast<uint8_t>(data.size());
    std::memcpy(frame.data, data.data(), data.size());
    sent = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  } else {
    canfd_frame frame{};
    frame.can_id = can_id;
    frame.len = static_cast<uint8_t>(data.size());
    frame.flags = bitrate_switch ? kCanFdBrs : 0;
    std::memcpy(frame.data, data.data(), data.size());
    sent = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  }
  if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    return false;
  }
  return sent >= 0;
}

std::vector<CanFrame> SocketCanBus::RecvAvailable(int max_frames)
{
  std::vector<CanFrame> frames;
  frames.reserve(static_cast<size_t>(max_frames));
  std::array<uint8_t, sizeof(canfd_frame)> buffer{};
  for (int i = 0; i < max_frames; ++i) {
    const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      break;
    }

    CanFrame frame;
    if (n == static_cast<ssize_t>(sizeof(can_frame))) {
      can_frame raw{};
      std::memcpy(&raw, buffer.data(), sizeof(raw));
      frame.arbitration_id = raw.can_id;
      frame.is_fd = false;
      frame.bitrate_switch = false;
      const size_t length = std::min<size_t>(raw.len, CAN_MAX_DLEN);
      frame.data.assign(raw.data, raw.data + length);
    } else if (n == static_cast<ssize_t>(sizeof(canfd_frame))) {
      canfd_frame raw{};
      std::memcpy(&raw, buffer.data(), sizeof(raw));
      frame.arbitration_id = raw.can_id;
      frame.is_fd = true;
      frame.bitrate_switch = (raw.flags & kCanFdBrs) != 0;
      const size_t length = std::min<size_t>(raw.len, CANFD_MAX_DLEN);
      frame.data.assign(raw.data, raw.data + length);
    } else {
      // A malformed kernel frame should not terminate a long-running control
      // loop; skip it and keep draining.
      continue;
    }

    frame.is_extended_id = (frame.arbitration_id & kCanEffFlag) != 0;
    frame.arbitration_id &=
      frame.is_extended_id ? kCanEffMask : kCanSffMask;
    frames.push_back(std::move(frame));
  }
  return frames;
}

}  // namespace upper_body_canfd_hardware
