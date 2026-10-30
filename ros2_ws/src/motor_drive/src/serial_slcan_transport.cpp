#include "motor_drive/serial_slcan_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <unistd.h>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>

#include "motor_drive/slcan.hpp"

namespace motor_drive {
namespace {

// 通道命令之间的间隔：CANable2 固件在串口侧有处理时延，过快的连续命令可能丢帧。
constexpr useconds_t kCommandDelayUs = 150000U;
// One complete ASCII frame shares a single budget, including short writes and
// interruptions. Backpressure must not block the 200 Hz control loop forever.
constexpr auto kWriteTimeout = std::chrono::milliseconds(5);

} // namespace

SerialSlcanTransport::SerialSlcanTransport(const std::string &tty_path,
                                           const unsigned nominal_index,
                                           const unsigned data_index)
    : nominal_index_(nominal_index), data_index_(data_index) {
  if (tty_path.empty()) {
    throw std::invalid_argument("serial_slcan transport requires a tty path");
  }
  if (nominal_index_ > 8U) {
    throw std::invalid_argument("slcan nominal bitrate index must be 0..8");
  }
  fd_ = ::open(tty_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open serial device '" + tty_path +
                             "': " + std::strerror(errno));
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) < 0) {
    const auto message = std::string(std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("cannot read serial attributes: " + message);
  }
  // 原始 8N1；USB CDC-ACM 实际按 USB 全速 bulk 传输，波特率只是名义值。
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  ::cfsetispeed(&tio, B115200);
  ::cfsetospeed(&tio, B115200);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  if (::tcsetattr(fd_, TCSANOW, &tio) < 0) {
    const auto message = std::string(std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("cannot configure serial device: " + message);
  }
  ::tcflush(fd_, TCIOFLUSH);

  try {
    configure_bus();
  } catch (...) {
    close_serial();
    throw;
  }
}

SerialSlcanTransport::~SerialSlcanTransport() {
  if (fd_ >= 0) {
    // Best effort only: closing an adapter channel is not a motor disable.
    // write_all has the same bounded budget here as in the control loop.
    static_cast<void>(write_all("C\r"));
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
}

void SerialSlcanTransport::close_serial() const noexcept {
  if (fd_ >= 0) {
    // Discard locally queued bytes after a failed frame. Never append a close
    // command or another target to an uncertain, potentially partial line.
    ::tcflush(fd_, TCOFLUSH);
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialSlcanTransport::configure_bus() {
  const auto configure = [this](const std::string &command) {
    if (!write_all(command)) {
      throw std::runtime_error("serial slcan bus configuration write failed");
    }
    ::usleep(kCommandDelayUs);
  };
  configure("C\r");
  configure("S" + std::to_string(nominal_index_) + "\r");
  if (data_index_ != 0U) {
    configure("Y" + std::to_string(data_index_) + "\r");
  }
  // CANable2 保留上一次会话的 M/A 状态。若此前被抓包工具设成 silent
  // 或关闭自动重发，仅 C/S/Y/O 不会恢复 ACK，表现为控制板 TEC 上升到
  // bus-off；每次打开都显式恢复正常模式和自动重发。
  configure("M0\r");
  configure("A1\r");
  configure("O\r");
  ::tcflush(fd_, TCIOFLUSH);
}

bool SerialSlcanTransport::write_all(const std::string &data) const {
  if (fd_ < 0) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + kWriteTimeout;
  std::size_t written = 0U;
  while (written < data.size()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    const auto count =
        ::write(fd_, data.data() + written, data.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        break;
      }
      // Round up for poll's millisecond API; the absolute deadline is checked
      // again before every write, so waking or EINTR cannot renew the budget.
      const auto wait_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining);
      pollfd descriptor{fd_, POLLOUT, 0};
      const auto ready = ::poll(&descriptor, 1, static_cast<int>(wait_ms.count()));
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready > 0 && !(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) &&
          (descriptor.revents & POLLOUT)) {
        continue;
      }
    }
    break;
  }
  if (written == data.size()) {
    return true;
  }
  close_serial();
  return false;
}

bool SerialSlcanTransport::send(const uint32_t id, const uint8_t *data,
                                const std::size_t size,
                                const bool force_fd,
                                const bool bitrate_switch,
                                const bool is_extended) const {
  if (fd_ < 0 || data == nullptr) {
    return false;
  }
  CanFrame frame{};
  frame.id = id;
  frame.size = size;
  frame.is_fd = force_fd || size > 8U;
  frame.bitrate_switch = bitrate_switch;
  frame.is_extended = is_extended;
  if (size > frame.data.size()) {
    return false;
  }
  std::copy(data, data + size, frame.data.begin());

  const auto line = encode_slcan_frame(frame);
  if (line.empty()) {
    return false;
  }
  return write_all(line + "\r");
}

bool SerialSlcanTransport::receive(CanFrame &output) const {
  if (fd_ < 0) {
    throw std::runtime_error("receive on closed serial transport");
  }

  char buffer[256]{};
  const auto count = ::read(fd_, buffer, sizeof(buffer));
  if (count > 0) {
    rx_buffer_.append(buffer, static_cast<std::size_t>(count));
  } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    throw std::runtime_error("serial receive failed: " +
                             std::string(std::strerror(errno)));
  }

  std::size_t end = 0U;
  while ((end = rx_buffer_.find('\r')) != std::string::npos) {
    const std::string line = rx_buffer_.substr(0U, end);
    rx_buffer_.erase(0U, end + 1U);
    if (decode_slcan_line(line, output)) {
      return true;
    }
  }
  return false;
}

} // namespace motor_drive
