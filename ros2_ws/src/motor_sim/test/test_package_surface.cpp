#include "motor_sim/protocol/motor_protocol.hpp"
#include "motor_sim/transport/serial_transport.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string read_file(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(MotorSimPackageSurface, ExposesProtocolAndTransportNamespaces) {
  motor_sim::protocol::CanFrame frame;
  EXPECT_EQ(frame.size, 0U);
  EXPECT_EQ(motor_sim::transport::ReceiveResult::NoData,
            motor_sim::transport::ReceiveResult::NoData);
}

// 总线端点与传输类型已由配置的 buses[] 独占；节点不再声明 transport_mode /
// baud_rate / allow_external_canfd / <bus>_port。而 rclcpp 对**未声明**参数的
// 覆盖是静默忽略——包内 launch 若还转发它们，用户照文档传参会以为生效，节点却
// 改用 YAML 里的端点，可能静默绑到另一条总线上。
TEST(MotorSimLaunchSurface, ForwardsNoRemovedBusParameters) {
  const std::string path =
      std::string(TEST_SOURCE_DIR) + "/launch/motor_sim.launch.py";
  const auto text = read_file(path);
  for (const char *removed :
       {"\"transport_mode\"", "\"baud_rate\"", "\"allow_external_canfd\""}) {
    EXPECT_EQ(text.find(removed), std::string::npos)
        << path << " still references the removed parameter " << removed;
  }
  EXPECT_EQ(text.find("_port"), std::string::npos)
      << path << " still advertises <bus_name>_port";
}
