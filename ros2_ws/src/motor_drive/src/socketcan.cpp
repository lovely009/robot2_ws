#include "motor_drive/socketcan.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace motor_drive {

SocketCan::SocketCan(const std::string &interface_name) {
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    throw std::invalid_argument("invalid SocketCAN interface name");
  }

  // 非阻塞 socket 让控制循环可以“读完当前已有反馈后立即返回”，不会因为某台电机
  // 暂时没发帧而卡住整个 ros2_control read() 周期。
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error("cannot create SocketCAN socket: " +
                             std::string(std::strerror(errno)));
  }

  // Linux CAN_RAW socket 默认只收 classic CAN；显式开启 CAN-FD 才能处理 12/20/64 Byte 帧。
  const int enable_fd = 1;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
    const auto message = std::string(std::strerror(errno));
    close_socket();
    throw std::runtime_error("cannot enable CAN-FD: " + message);
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
    const auto message = std::string(std::strerror(errno));
    close_socket();
    throw std::runtime_error("unknown CAN interface '" + interface_name +
                             "': " + message);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    const auto message = std::string(std::strerror(errno));
    close_socket();
    throw std::runtime_error("cannot bind CAN interface '" + interface_name +
                             "': " + message);
  }
}

SocketCan::~SocketCan() { close_socket(); }

SocketCan::SocketCan(SocketCan &&other) noexcept : fd_(other.fd_) {
  other.fd_ = -1;
}

SocketCan &SocketCan::operator=(SocketCan &&other) noexcept {
  if (this != &other) {
    close_socket();
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void SocketCan::close_socket() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCan::send(const uint32_t id, const uint8_t *data,
                     const std::size_t size, const bool force_fd,
                     const bool bitrate_switch,
                     const bool is_extended) const {
  if (fd_ < 0 || data == nullptr || size > CANFD_MAX_DLEN) {
    return false;
  }

  canid_t can_id = id;
  if (is_extended) {
    if (id > CAN_EFF_MASK) {
      return false;
    }
    can_id |= CAN_EFF_FLAG;
  } else if (id > CAN_SFF_MASK) {
    return false;
  }

  // 超过 classic CAN 的 8 Byte 必须走 CAN-FD；调用方也可 force_fd 强制 8 Byte FD 帧。
  const bool use_fd = force_fd || size > CAN_MAX_DLEN;
  if (use_fd) {
    canfd_frame frame{};
    frame.can_id = can_id;
    frame.len = static_cast<__u8>(size);
    frame.flags = bitrate_switch ? CANFD_BRS : 0U;
    std::memcpy(frame.data, data, size);
    return ::write(fd_, &frame, CANFD_MTU) == CANFD_MTU;
  }

  can_frame frame{};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(size);
  std::memcpy(frame.data, data, size);
  return ::write(fd_, &frame, CAN_MTU) == CAN_MTU;
}

bool SocketCan::receive(CanFrame &output) const {
  if (fd_ < 0) {
    throw std::runtime_error("receive on closed SocketCAN socket");
  }

  // 用 CANFD_MTU 大缓冲区同时接收 classic CAN 与 CAN-FD，再根据 read() 返回长度判型。
  canfd_frame raw{};
  const auto count = ::read(fd_, &raw, CANFD_MTU);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    throw std::runtime_error("SocketCAN receive failed: " +
                             std::string(std::strerror(errno)));
  }
  if (count != CAN_MTU && count != CANFD_MTU) {
    return false;
  }

  output = {};
  output.is_fd = count == CANFD_MTU;
  output.bitrate_switch = output.is_fd && ((raw.flags & CANFD_BRS) != 0U);
  output.is_extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
  output.id = raw.can_id & (output.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  output.size = output.is_fd ? raw.len : reinterpret_cast<const can_frame *>(&raw)->can_dlc;
  if (output.size > output.data.size()) {
    return false;
  }
  std::memcpy(output.data.data(), raw.data, output.size);
  return true;
}

} // namespace motor_drive
