#include "motor_drive/serial_slcan_transport.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/time.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using motor_drive::SerialSlcanTransport;

class PseudoTerminal {
public:
  PseudoTerminal() {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master_ < 0) {
      throw std::runtime_error("posix_openpt failed");
    }
    if (::grantpt(master_) < 0 || ::unlockpt(master_) < 0) {
      ::close(master_);
      throw std::runtime_error("cannot prepare pseudo-terminal");
    }
    char name[128]{};
    if (::ptsname_r(master_, name, sizeof(name)) != 0) {
      ::close(master_);
      throw std::runtime_error("ptsname_r failed");
    }
    path_ = name;
  }
  ~PseudoTerminal() {
    if (master_ >= 0) {
      ::close(master_);
    }
  }
  int master() const { return master_; }
  const std::string &path() const { return path_; }
  int release_master() {
    const int fd = master_;
    master_ = -1;
    return fd;
  }

private:
  int master_{-1};
  std::string path_;
};

std::string read_available(const int fd) {
  std::string output;
  char buffer[4096];
  for (;;) {
    const auto size = ::read(fd, buffer, sizeof(buffer));
    if (size > 0) {
      output.append(buffer, static_cast<std::size_t>(size));
    } else if (size < 0 && errno == EINTR) {
      continue;
    } else {
      return output;  // EAGAIN, or EIO after the slave has closed.
    }
  }
}

std::string read_line(const int fd, const std::chrono::milliseconds budget) {
  std::string output;
  const auto deadline = Clock::now() + budget;
  while (Clock::now() < deadline) {
    output += read_available(fd);
    if (output.find('\r') != std::string::npos) {
      break;
    }
    std::this_thread::sleep_for(1ms);
  }
  return output;
}

// Fill the real tty output queue without first putting the transport itself in
// its failed state. Repeated EAGAINs allow the tty worker to finish moving bytes
// into the master's queue before the timed send/destructor starts.
bool saturate_output(const std::string &path) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }
  std::array<char, 4096> bytes{};
  std::size_t write_size = bytes.size();
  unsigned full_observations = 0;
  for (unsigned attempt = 0; attempt < 65536; ++attempt) {
    const auto count = ::write(fd, bytes.data(), write_size);
    if (count > 0) {
      full_observations = 0;
    } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // A large write may report EAGAIN while a short ASCII line would still
      // fit. Fill the remaining slack a byte at a time before declaring full.
      if (write_size > 1) {
        write_size = 1;
        continue;
      }
      if (++full_observations == 3) {
        ::close(fd);
        return true;
      }
      std::this_thread::sleep_for(5ms);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      break;
    }
  }
  ::close(fd);
  return false;
}

volatile std::sig_atomic_t signal_count = 0;
void interrupt_wait(int) { signal_count = signal_count + 1; }

enum class BlockedOperation { Send, InterruptedSend, Destructor };

int backpressure_child(const PseudoTerminal &tty, const int ready_fd,
                       const BlockedOperation operation) {
  auto transport = std::make_unique<SerialSlcanTransport>(tty.path());
  read_available(tty.master());
  if (::write(ready_fd, "R", 1) != 1) {
    return 10;
  }
  if (!saturate_output(tty.path())) {
    return 11;
  }
  if (operation == BlockedOperation::InterruptedSend) {
    struct sigaction action{};
    action.sa_handler = interrupt_wait;
    ::sigemptyset(&action.sa_mask);
    // No SA_RESTART: repeated EINTR must not restart the frame's deadline.
    if (::sigaction(SIGALRM, &action, nullptr) != 0) {
      return 12;
    }
    itimerval timer{};
    timer.it_value.tv_usec = 100;
    timer.it_interval.tv_usec = 100;
    if (::setitimer(ITIMER_REAL, &timer, nullptr) != 0) {
      return 13;
    }
  }

  const auto started = Clock::now();
  std::array<uint8_t, 64> payload{};
  bool sent = false;
  if (operation == BlockedOperation::Destructor) {
    transport.reset();
  } else {
    sent = transport->send(0x210, payload.data(), payload.size(), true);
  }
  const auto elapsed = Clock::now() - started;
  if (operation == BlockedOperation::InterruptedSend) {
    const itimerval disabled{};
    ::setitimer(ITIMER_REAL, &disabled, nullptr);
    if (signal_count == 0) {
      return 14;
    }
  }
  if (elapsed >= 500ms) {
    return 15;
  }
  if (operation == BlockedOperation::Destructor) {
    return 0;
  }
  if (sent) {
    return 16;
  }

  // Restoring the reader must not revive a transport after an uncertain frame.
  read_available(tty.master());
  if (transport->send(0x123, payload.data(), 2, false)) {
    return 17;
  }
  if (!read_available(tty.master()).empty()) {
    return 18;
  }
  motor_drive::CanFrame frame;
  try {
    transport->receive(frame);
    return 19;
  } catch (const std::runtime_error &) {
  }
  transport.reset();
  // A failed transport must not append a C command to a possibly partial line.
  return read_available(tty.master()).empty() ? 0 : 20;
}

void expect_bounded_backpressure(const BlockedOperation operation) {
  PseudoTerminal tty;
  int ready[2];
  ASSERT_EQ(::pipe(ready), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(ready[0]);
    int result = 30;
    try {
      result = backpressure_child(tty, ready[1], operation);
    } catch (...) {
    }
    ::_exit(result);
  }
  ::close(ready[1]);
  // Construction deliberately takes about 900 ms. Start the one-second hang
  // watchdog only after configuration completes, so a slow CI host has room.
  pollfd descriptor{ready[0], POLLIN, 0};
  const int configured = ::poll(&descriptor, 1, 3000);
  char marker = 0;
  const bool started = configured > 0 && ::read(ready[0], &marker, 1) == 1;
  ::close(ready[0]);
  int status = 0;
  bool exited = false;
  if (started) {
    const auto deadline = Clock::now() + 1s;
    while (Clock::now() < deadline) {
      if (::waitpid(child, &status, WNOHANG) == child) {
        exited = true;
        break;
      }
      std::this_thread::sleep_for(1ms);
    }
  }
  if (!exited) {
    ::kill(child, SIGKILL);
    ::waitpid(child, &status, 0);
  }
  ASSERT_TRUE(started) << "child did not finish serial configuration";
  ASSERT_TRUE(exited) << "serial operation hung under tty backpressure";
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0) << "child invariant failure code";
}

std::size_t open_fd_count() {
  DIR *directory = ::opendir("/proc/self/fd");
  if (!directory) {
    throw std::runtime_error("cannot inspect open file descriptors");
  }
  std::size_t count = 0;
  while (const auto *entry = ::readdir(directory)) {
    if (std::strcmp(entry->d_name, ".") != 0 &&
        std::strcmp(entry->d_name, "..") != 0) {
      ++count;
    }
  }
  ::closedir(directory);
  return count;
}
} // namespace

TEST(SerialSlcanTransport, ConfiguresBusAndExchangesAsciiFrames) {
  PseudoTerminal tty;
  {
    SerialSlcanTransport transport(tty.path());
    EXPECT_EQ(read_available(tty.master()), "C\rS8\rY5\rM0\rA1\rO\r");
    const std::array<uint8_t, 2> payload{0xab, 0xcd};
    ASSERT_TRUE(transport.send(0x123, payload.data(), payload.size(), false));
    EXPECT_EQ(read_line(tty.master(), 500ms), "t1232ABCD\r");
    ASSERT_TRUE(transport.send(0x321, payload.data(), payload.size(), true));
    EXPECT_EQ(read_line(tty.master(), 500ms), "b3212ABCD\r");

    const std::string incoming = "\rb1233A1B2C3\r";
    ASSERT_EQ(::write(tty.master(), incoming.data(), incoming.size()),
              static_cast<ssize_t>(incoming.size()));
    motor_drive::CanFrame frame;
    bool received = false;
    const auto deadline = Clock::now() + 500ms;
    while (!received && Clock::now() < deadline) {
      received = transport.receive(frame);
      if (!received) {
        std::this_thread::sleep_for(1ms);
      }
    }
    ASSERT_TRUE(received);
    EXPECT_EQ(frame.id, 0x123U);
    EXPECT_TRUE(frame.is_fd);
    EXPECT_TRUE(frame.bitrate_switch);
    EXPECT_FALSE(frame.is_extended);
    ASSERT_EQ(frame.size, 3U);
    EXPECT_EQ(frame.data[0], 0xa1);
    EXPECT_EQ(frame.data[1], 0xb2);
    EXPECT_EQ(frame.data[2], 0xc3);
    EXPECT_FALSE(transport.receive(frame));
  }
  EXPECT_EQ(read_line(tty.master(), 500ms), "C\r");
}

TEST(SerialSlcanTransport, BackpressureFailsWithinBudgetAndClosesTransport) {
  expect_bounded_backpressure(BlockedOperation::Send);
}

TEST(SerialSlcanTransport, InterruptedWaitsShareTheSameFrameBudget) {
  expect_bounded_backpressure(BlockedOperation::InterruptedSend);
}

TEST(SerialSlcanTransport, DestructorHasABoundedWriteBudget) {
  expect_bounded_backpressure(BlockedOperation::Destructor);
}

TEST(SerialSlcanTransport, ConfigurationWriteFailureThrowsAndReleasesDescriptor) {
  const auto baseline = open_fd_count();
  PseudoTerminal tty;
  const int master = tty.release_master();
  std::thread disconnect([master] {
    read_line(master, 2s);
    ::close(master);  // The next configuration command now fails with EIO.
  });
  EXPECT_THROW(SerialSlcanTransport transport(tty.path()), std::runtime_error);
  disconnect.join();
  EXPECT_EQ(open_fd_count(), baseline);
}
