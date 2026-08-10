#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "upper_body_canfd_hardware/canfd_socket.hpp"

namespace upper_body_canfd_hardware {
namespace {

bool VcanAvailable(const std::string & interface)
{
  try {
    SocketCanBus probe(interface);
    return probe.IsOpen();
  } catch (const std::exception &) {
    return false;
  }
}

TEST(TestCanfdSocket, ClassicAndFdLoopback)
{
  if (!VcanAvailable("vcan0")) {
    GTEST_SKIP() << "vcan0 not available";
  }
  SocketCanBus bus("vcan0", true);
  ASSERT_TRUE(bus.IsOpen());

  const std::vector<uint8_t> classic{1, 2, 3, 4, 5, 6, 7, 8};
  ASSERT_TRUE(bus.Send(0x123, classic, false, true));
  const std::vector<uint8_t> fd_payload(24, 0xAB);
  ASSERT_TRUE(bus.Send(0x124, fd_payload, true, true));

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  auto frames = bus.RecvAvailable(16);
  ASSERT_EQ(frames.size(), 2u);

  bool saw_classic = false;
  bool saw_fd = false;
  for (const auto & frame : frames) {
    if (frame.arbitration_id == 0x123 && !frame.is_fd) {
      EXPECT_EQ(frame.data, classic);
      saw_classic = true;
    } else if (frame.arbitration_id == 0x124 && frame.is_fd) {
      EXPECT_EQ(frame.data, fd_payload);
      EXPECT_TRUE(frame.bitrate_switch);
      saw_fd = true;
    }
  }
  EXPECT_TRUE(saw_classic);
  EXPECT_TRUE(saw_fd);
}

TEST(TestCanfdSocket, OwnMessagesFilteredByDefault)
{
  if (!VcanAvailable("vcan1")) {
    GTEST_SKIP() << "vcan1 not available";
  }
  SocketCanBus sender("vcan1");  // receive_own_messages = false
  ASSERT_TRUE(sender.Send(0x555, {0xAA}, false, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(sender.RecvAvailable(8).empty());
}

}  // namespace
}  // namespace upper_body_canfd_hardware
