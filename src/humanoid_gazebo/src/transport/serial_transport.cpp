#include "humanoid_gazebo/transport/serial_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <utility>

namespace humanoid_gazebo::transport {
namespace {

speed_t baud_constant(const int baud_rate) {
  switch (baud_rate) {
    case 115200:
      return B115200;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
    default:
      throw std::invalid_argument("unsupported serial baud rate");
  }
}

std::runtime_error io_error(const std::string &operation,
                            const std::string &device) {
  return std::runtime_error(operation + " '" + device + "': " +
                            std::strerror(errno));
}

}  // namespace

SerialTransport::SerialTransport(std::string device, const TransportMode mode,
                                 const int baud_rate)
    : device_(std::move(device)), mode_(mode), baud_rate_(baud_rate) {
  if (device_.empty()) {
    throw std::invalid_argument("serial device path cannot be empty");
  }
  static_cast<void>(baud_constant(baud_rate_));
}

SerialTransport::~SerialTransport() { close(); }

void SerialTransport::open() {
  if (fd_ >= 0) {
    throw std::logic_error("serial transport is already open");
  }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw io_error("cannot open serial device", device_);
  }

  termios options{};
  if (::tcgetattr(fd_, &options) < 0) {
    const auto error = io_error("cannot read serial attributes", device_);
    close();
    throw error;
  }
  ::cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  const auto speed = baud_constant(baud_rate_);
  if (::cfsetispeed(&options, speed) < 0 ||
      ::cfsetospeed(&options, speed) < 0 ||
      ::tcsetattr(fd_, TCSANOW, &options) < 0) {
    const auto error = io_error("cannot configure serial device", device_);
    close();
    throw error;
  }
  ::tcflush(fd_, TCIOFLUSH);
  disconnected_.store(false);

  if (mode_ == TransportMode::SerialSlcan &&
      (!write_bytes("C\rS8\rY5\rO\r") || ::tcdrain(fd_) < 0)) {
    const auto error = io_error("cannot initialize CANable", device_);
    close();
    throw error;
  }
  stop_reader_.store(false);
  try {
    reader_thread_ = std::thread([this] { reader_loop(); });
  } catch (...) {
    close();
    throw;
  }
}

void SerialTransport::close() noexcept {
  stop_reader_.store(true);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  {
    const std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
  disconnected_.store(true);
  const std::lock_guard<std::mutex> lock(receive_mutex_);
  received_frames_.clear();
  parser_ = FrameRecordParser{};
}

bool SerialTransport::write_bytes(const std::string &bytes) noexcept {
  const std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0 || disconnected_.load()) {
    return false;
  }
  std::size_t offset = 0U;
  while (offset < bytes.size()) {
    const auto count =
        ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{fd_, POLLOUT, 0};
      const int result = ::poll(&descriptor, 1, 100);
      if (result > 0 && (descriptor.revents & POLLOUT) != 0) {
        continue;
      }
    }
    mark_disconnected();
    return false;
  }
  return true;
}

bool SerialTransport::send(const protocol::CanFrame &frame) {
  if (fd_ < 0 || disconnected_.load()) {
    return false;
  }
  try {
    return write_bytes(encode_frame_record(frame));
  } catch (const std::invalid_argument &) {
    return false;
  }
}

ReceiveResult SerialTransport::receive(ReceivedFrame &frame) {
  {
    const std::lock_guard<std::mutex> lock(receive_mutex_);
    if (!received_frames_.empty()) {
      frame = received_frames_.front();
      received_frames_.pop_front();
      return ReceiveResult::Frame;
    }
  }
  if (fd_ < 0 || disconnected_.load()) {
    return ReceiveResult::Disconnected;
  }
  return ReceiveResult::NoData;
}

void SerialTransport::reader_loop() noexcept {
  while (!stop_reader_.load() && !disconnected_.load()) {
    pollfd descriptor{fd_, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, 10);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      mark_disconnected();
      return;
    }
    if (poll_result == 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      mark_disconnected();
      return;
    }
    if ((descriptor.revents & POLLIN) == 0) {
      continue;
    }

    char buffer[1024]{};
    const auto count = ::read(fd_, buffer, sizeof(buffer));
    if (count == 0) {
      mark_disconnected();
      return;
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      mark_disconnected();
      return;
    }

    const auto decoded = parser_.push(
        std::string_view(buffer, static_cast<std::size_t>(count)));
    if (!decoded.empty()) {
      const auto received_at = std::chrono::steady_clock::now();
      const std::lock_guard<std::mutex> lock(receive_mutex_);
      for (const auto &frame : decoded) {
        received_frames_.push_back({frame, received_at});
      }
    }
  }
}

void SerialTransport::mark_disconnected() noexcept {
  disconnected_.store(true);
  const std::lock_guard<std::mutex> lock(receive_mutex_);
  received_frames_.clear();
}

}  // namespace humanoid_gazebo::transport
