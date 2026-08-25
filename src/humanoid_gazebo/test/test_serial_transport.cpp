#include "humanoid_gazebo/transport/serial_transport.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace protocol = humanoid_gazebo::protocol;
namespace transport = humanoid_gazebo::transport;
using namespace std::chrono_literals;

namespace {

class PseudoTerminal {
public:
  PseudoTerminal() {
    char name[128]{};
    if (::openpty(&master_fd_, &slave_fd_, name, nullptr, nullptr) < 0) {
      throw std::runtime_error("openpty failed");
    }
    slave_path_ = name;
    ::close(slave_fd_);
    slave_fd_ = -1;
    const int flags = ::fcntl(master_fd_, F_GETFL, 0);
    ::fcntl(master_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  ~PseudoTerminal() {
    close_master();
    if (slave_fd_ >= 0) {
      ::close(slave_fd_);
    }
  }

  PseudoTerminal(const PseudoTerminal &) = delete;
  PseudoTerminal &operator=(const PseudoTerminal &) = delete;

  const std::string &slave_path() const noexcept { return slave_path_; }

  std::string read_with_timeout(const std::chrono::milliseconds timeout) {
    pollfd descriptor{master_fd_, POLLIN, 0};
    if (::poll(&descriptor, 1, static_cast<int>(timeout.count())) <= 0) {
      return {};
    }
    char buffer[256]{};
    const auto count = ::read(master_fd_, buffer, sizeof(buffer));
    return count > 0 ? std::string(buffer, static_cast<std::size_t>(count))
                     : std::string{};
  }

  void write_all(const std::string &bytes) {
    std::size_t offset = 0U;
    while (offset < bytes.size()) {
      const auto count = ::write(master_fd_, bytes.data() + offset,
                                 bytes.size() - offset);
      if (count <= 0) {
        throw std::runtime_error("PTY write failed");
      }
      offset += static_cast<std::size_t>(count);
    }
  }

  void close_master() noexcept {
    if (master_fd_ >= 0) {
      ::close(master_fd_);
      master_fd_ = -1;
    }
  }

private:
  int master_fd_{-1};
  int slave_fd_{-1};
  std::string slave_path_;
};

protocol::CanFrame sample_frame() {
  protocol::CanFrame frame;
  frame.id = 0x123U;
  frame.fd = true;
  frame.brs = true;
  frame.size = 3U;
  frame.data[0] = 0xAAU;
  frame.data[1] = 0xBBU;
  frame.data[2] = 0xCCU;
  return frame;
}

transport::ReceiveResult receive_with_timeout(
    transport::SerialTransport &serial, transport::ReceivedFrame &received,
    const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  transport::ReceiveResult result = transport::ReceiveResult::NoData;
  while (std::chrono::steady_clock::now() < deadline) {
    result = serial.receive(received);
    if (result != transport::ReceiveResult::NoData) {
      return result;
    }
    std::this_thread::sleep_for(1ms);
  }
  return result;
}

}  // namespace

TEST(SerialTransport, UsbSerialDoesNotSendAdapterCommands) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::UsbSerial, 3000000);
  serial.open();
  EXPECT_EQ(pty.read_with_timeout(20ms), "");
}

TEST(SerialTransport, SerialSlcanInitializesCanableInOrder) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::SerialSlcan, 3000000);
  serial.open();
  EXPECT_EQ(pty.read_with_timeout(100ms), "C\rS8\rY5\rO\r");
}

TEST(SerialTransport, ReceivesFragmentedRecordAndSendsEncodedFrame) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::UsbSerial, 3000000);
  serial.open();
  transport::ReceivedFrame received;
  pty.write_all("b1233AA");
  EXPECT_EQ(serial.receive(received), transport::ReceiveResult::NoData);
  pty.write_all("BBCC\r");
  EXPECT_EQ(receive_with_timeout(serial, received, 100ms),
            transport::ReceiveResult::Frame);
  EXPECT_EQ(received.frame.id, 0x123U);
  EXPECT_EQ(received.frame.data[2], 0xCCU);

  EXPECT_TRUE(serial.send(sample_frame()));
  EXPECT_EQ(pty.read_with_timeout(100ms), "b1233AABBCC\r");
}

TEST(SerialTransport, ReportsPeerHangupAsDisconnected) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::UsbSerial, 3000000);
  serial.open();
  pty.close_master();
  transport::ReceivedFrame frame;
  EXPECT_EQ(receive_with_timeout(serial, frame, 100ms),
            transport::ReceiveResult::Disconnected);
  EXPECT_FALSE(serial.send(sample_frame()));
}

TEST(SerialTransport, CloseIsIdempotentAndClosedTransportIsDisconnected) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::UsbSerial, 3000000);
  serial.open();
  serial.close();
  serial.close();
  transport::ReceivedFrame frame;
  EXPECT_EQ(serial.receive(frame), transport::ReceiveResult::Disconnected);
  EXPECT_FALSE(serial.send(sample_frame()));
}

TEST(SerialTransport, PreservesArrivalTimeAcrossConsumerBacklog) {
  PseudoTerminal pty;
  transport::SerialTransport serial(
      pty.slave_path(), transport::TransportMode::UsbSerial, 3000000);
  serial.open();
  pty.write_all(transport::encode_frame_record(sample_frame()));
  std::this_thread::sleep_for(10ms);
  auto second = sample_frame();
  second.data[0] = 0x55U;
  pty.write_all(transport::encode_frame_record(second));
  std::this_thread::sleep_for(10ms);

  transport::ReceivedFrame first_received;
  transport::ReceivedFrame second_received;
  ASSERT_EQ(receive_with_timeout(serial, first_received, 100ms),
            transport::ReceiveResult::Frame);
  ASSERT_EQ(receive_with_timeout(serial, second_received, 100ms),
            transport::ReceiveResult::Frame);
  EXPECT_GE(second_received.received_at - first_received.received_at, 5ms);
}
