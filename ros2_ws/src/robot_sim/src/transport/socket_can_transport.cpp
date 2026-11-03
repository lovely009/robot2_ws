#include "motor_sim/transport/socket_can_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace motor_sim::transport {
namespace {

std::runtime_error socket_error(const std::string &operation,
                                const std::string &interface_name) {
  return std::runtime_error(operation + " '" + interface_name + "': " +
                            std::strerror(errno));
}

// timespec（CLOCK_REALTIME）转纳秒时长。
std::chrono::nanoseconds realtime_nanoseconds(const timespec &stamp) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::seconds(stamp.tv_sec) +
      std::chrono::nanoseconds(stamp.tv_nsec));
}

bool write_all(const int fd, const void *data,
               const std::size_t size) noexcept {
  const auto *bytes = static_cast<const uint8_t *>(data);
  std::size_t offset = 0U;
  while (offset < size) {
    const auto count = ::write(fd, bytes + offset, size - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace

SocketCanTransport::SocketCanTransport(std::string interface_name)
    : interface_name_(std::move(interface_name)) {
  if (interface_name_.empty()) {
    throw std::invalid_argument("socketcan interface name cannot be empty");
  }
}

SocketCanTransport::~SocketCanTransport() { close(); }

void SocketCanTransport::open() {
  if (fd_ >= 0) {
    throw std::logic_error("socketcan transport is already open");
  }
  const unsigned int index = ::if_nametoindex(interface_name_.c_str());
  if (index == 0U) {
    throw std::runtime_error("cannot find socketcan interface '" +
                             interface_name_ + "'");
  }
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) {
    throw socket_error("cannot create socketcan socket", interface_name_);
  }
  // 同时接收经典 CAN 与 CAN FD 帧；FD 命令/反馈帧（12/20/64 字节）必须开启。
  // SO_TIMESTAMPNS 让内核在收包时刻打时间戳，批量消费也不压缩帧间隔。
  const int enable = 1;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                   sizeof(enable)) < 0 ||
      ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &enable, sizeof(enable)) <
          0) {
    const auto error =
        socket_error("cannot configure socketcan socket", interface_name_);
    close();
    throw error;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) <
      0) {
    const auto error = socket_error("cannot bind socketcan interface",
                                    interface_name_);
    close();
    throw error;
  }
  timespec realtime{};
  ::clock_gettime(CLOCK_REALTIME, &realtime);
  realtime_to_steady_ = std::chrono::steady_clock::now().time_since_epoch() -
                        realtime_nanoseconds(realtime);
  disconnected_.store(false);
}

void SocketCanTransport::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  disconnected_.store(true);
}

bool SocketCanTransport::send(const protocol::CanFrame &frame) {
  if (fd_ < 0 || disconnected_.load()) {
    return false;
  }
  if (frame.size > frame.data.size()) {
    return false;
  }
  const canid_t id = frame.id | (frame.extended ? CAN_EFF_FLAG : 0U);
  if ((frame.extended ? frame.id > CAN_EFF_MASK : frame.id > CAN_SFF_MASK)) {
    return false;
  }
  if (frame.fd || frame.size > 8U) {
    if (!frame.fd) {
      return false;  // 经典 CAN 载荷不能超过 8 字节。
    }
    canfd_frame raw{};
    raw.can_id = id;
    raw.flags = frame.brs ? CANFD_BRS : 0U;
    raw.len = static_cast<uint8_t>(frame.size);
    std::memcpy(raw.data, frame.data.data(), frame.size);
    if (write_all(fd_, &raw, CANFD_MTU)) {
      return true;
    }
    mark_disconnected();
    return false;
  }
  can_frame raw{};
  raw.can_id = id;
  raw.len = static_cast<uint8_t>(frame.size);
  std::memcpy(raw.data, frame.data.data(), frame.size);
  if (write_all(fd_, &raw, CAN_MTU)) {
    return true;
  }
  mark_disconnected();
  return false;
}

ReceiveResult SocketCanTransport::receive(ReceivedFrame &out) {
  if (fd_ < 0 || disconnected_.load()) {
    return ReceiveResult::Disconnected;
  }
  alignas(8) uint8_t buffer[CANFD_MTU]{};
  uint8_t control[CMSG_SPACE(sizeof(timespec))]{};
  iovec iov{buffer, sizeof(buffer)};
  msghdr header{};
  header.msg_iov = &iov;
  header.msg_iovlen = 1U;
  header.msg_control = control;
  header.msg_controllen = sizeof(control);
  const auto count = ::recvmsg(fd_, &header, MSG_DONTWAIT);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return ReceiveResult::NoData;
    }
    mark_disconnected();
    return ReceiveResult::Disconnected;
  }
  if (count == 0) {
    mark_disconnected();
    return ReceiveResult::Disconnected;
  }

  auto received_at = std::chrono::steady_clock::now();
  for (auto header_message = CMSG_FIRSTHDR(&header); header_message != nullptr;
       header_message = CMSG_NXTHDR(&header, header_message)) {
    if (header_message->cmsg_level == SOL_SOCKET &&
        header_message->cmsg_type == SCM_TIMESTAMPNS &&
        header_message->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
      timespec stamp{};
      std::memcpy(&stamp, CMSG_DATA(header_message), sizeof(stamp));
      received_at = std::chrono::steady_clock::time_point{} +
                    (realtime_to_steady_ + realtime_nanoseconds(stamp));
      break;
    }
  }

  protocol::CanFrame frame;
  if (static_cast<std::size_t>(count) == CANFD_MTU) {
    canfd_frame raw{};
    std::memcpy(&raw, buffer, sizeof(raw));
    frame.fd = true;
    frame.brs = (raw.flags & CANFD_BRS) != 0U;
    frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
    frame.id = raw.can_id & CAN_EFF_MASK;
    frame.size = raw.len;
    if (frame.size <= frame.data.size()) {
      std::memcpy(frame.data.data(), raw.data, frame.size);
    }
  } else if (static_cast<std::size_t>(count) == CAN_MTU) {
    can_frame raw{};
    std::memcpy(&raw, buffer, sizeof(raw));
    frame.fd = false;
    frame.brs = false;
    frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
    frame.id = raw.can_id & CAN_EFF_MASK;
    frame.size = raw.len;
    if (frame.size <= frame.data.size()) {
      std::memcpy(frame.data.data(), raw.data, frame.size);
    }
  } else {
    return ReceiveResult::NoData;  // 未知长度，丢弃等待下一帧。
  }
  if (frame.size > frame.data.size()) {
    return ReceiveResult::NoData;
  }
  out = ReceivedFrame{frame, received_at};
  return ReceiveResult::Frame;
}

void SocketCanTransport::mark_disconnected() noexcept {
  disconnected_.store(true);
}

}  // namespace motor_sim::transport
