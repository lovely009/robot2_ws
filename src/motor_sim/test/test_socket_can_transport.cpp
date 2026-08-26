#include "motor_sim/transport/socket_can_transport.hpp"

#include <gtest/gtest.h>

namespace transport = motor_sim::transport;
namespace protocol = motor_sim::protocol;

TEST(SocketCanTransport, RejectsEmptyInterfaceName) {
  EXPECT_THROW(transport::SocketCanTransport(""), std::invalid_argument);
}

TEST(SocketCanTransport, OpenRejectsUnknownInterface) {
  transport::SocketCanTransport transport("no_such_vcan_interface");
  EXPECT_THROW(transport.open(), std::runtime_error);
  // 打开失败后仍可安全关闭。
  transport.close();
}

TEST(SocketCanTransport, ClosedTransportRefusesSendAndReportsDisconnected) {
  transport::SocketCanTransport transport("lo");
  protocol::CanFrame frame;
  frame.id = 0x505U;
  frame.size = 8U;
  EXPECT_FALSE(transport.send(frame));
  transport::ReceivedFrame received;
  EXPECT_EQ(transport.receive(received),
            transport::ReceiveResult::Disconnected);
  transport.close();
  transport.close();
}
