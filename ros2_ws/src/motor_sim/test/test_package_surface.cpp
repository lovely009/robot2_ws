#include "motor_sim/protocol/motor_protocol.hpp"
#include "motor_sim/transport/serial_transport.hpp"

#include <gtest/gtest.h>

TEST(MotorSimPackageSurface, ExposesProtocolAndTransportNamespaces) {
  motor_sim::protocol::CanFrame frame;
  EXPECT_EQ(frame.size, 0U);
  EXPECT_EQ(motor_sim::transport::ReceiveResult::NoData,
            motor_sim::transport::ReceiveResult::NoData);
}
