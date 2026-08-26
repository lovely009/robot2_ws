#include "motor_sim/ros/motor_sim_node.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <fstream>
#include <stdexcept>
#include <string>

namespace core = motor_sim::core;
namespace ros = motor_sim::ros;
namespace transport = motor_sim::transport;

namespace {

class PseudoTerminal {
public:
  PseudoTerminal() {
    char path[128]{};
    int slave = -1;
    if (::openpty(&master_, &slave, path, nullptr, nullptr) < 0) {
      throw std::runtime_error("openpty failed");
    }
    slave_path_ = path;
    ::close(slave);
  }
  ~PseudoTerminal() { ::close(master_); }
  const std::string &slave_path() const noexcept { return slave_path_; }

private:
  int master_{-1};
  std::string slave_path_;
};

core::TopologyContract arm_contract() {
  return {{"arm"}, {{"joint_a", "arm", "rp", 1U}}};
}

core::TopologyContract three_bus_contract() {
  return {{"left_arm", "right_arm", "legs"}, {}};
}

ros::NodeParameters parameters(const std::string &device) {
  ros::NodeParameters result;
  result.transport_mode = transport::TransportMode::UsbSerial;
  result.config_file = "/tmp/motors.yaml";
  result.ports = {{"arm", device}};
  return result;
}

}  // namespace

TEST(MotorSimNodeParameters, AcceptsSingleArmPty) {
  PseudoTerminal arm;
  EXPECT_NO_THROW(ros::validate_node_parameters(parameters(arm.slave_path()),
                                                 arm_contract()));
}

TEST(MotorSimNodeParameters, AcceptsThreeBusPtys) {
  PseudoTerminal left;
  PseudoTerminal right;
  PseudoTerminal legs;
  auto value = parameters(left.slave_path());
  value.ports = {{"left_arm", left.slave_path()},
                 {"right_arm", right.slave_path()},
                 {"legs", legs.slave_path()}};
  EXPECT_NO_THROW(ros::validate_node_parameters(value, three_bus_contract()));
}

TEST(MotorSimNodeParameters, RejectsMissingBusAndDuplicateDevice) {
  PseudoTerminal arm;
  auto missing = parameters(arm.slave_path());
  missing.ports.clear();
  EXPECT_THROW(ros::validate_node_parameters(missing, arm_contract()),
               std::invalid_argument);

  auto duplicate = parameters(arm.slave_path());
  duplicate.ports.push_back({"other", arm.slave_path()});
  const core::TopologyContract two_buses{{"arm", "other"}, {}};
  EXPECT_THROW(ros::validate_node_parameters(duplicate, two_buses),
               std::invalid_argument);
}

TEST(MotorSimNodeParameters, SerialSlcanRequiresExplicitOptIn) {
  PseudoTerminal arm;
  auto value = parameters(arm.slave_path());
  value.transport_mode = transport::TransportMode::SerialSlcan;
  EXPECT_THROW(ros::validate_node_parameters(value, arm_contract()),
               std::invalid_argument);
  value.allow_external_canfd = true;
  EXPECT_NO_THROW(ros::validate_node_parameters(value, arm_contract()));
}

TEST(MotorSimNodeParameters, SocketCanAcceptsExistingInterfaceName) {
  auto value = parameters("/dev/null");
  value.transport_mode = transport::TransportMode::SocketCan;
  // socket_can 不使用波特率，非法值也应被忽略。
  value.baud_rate = 12345;
  value.ports = {{"arm", "lo"}};
  EXPECT_NO_THROW(ros::validate_node_parameters(value, arm_contract()));
}

TEST(MotorSimNodeParameters, SocketCanRejectsMissingOrPathLikeInterfaces) {
  auto missing = parameters("");
  missing.transport_mode = transport::TransportMode::SocketCan;
  missing.ports = {{"arm", "no_such_vcan_interface"}};
  EXPECT_THROW(ros::validate_node_parameters(missing, arm_contract()),
               std::invalid_argument);

  auto path_like = parameters("");
  path_like.transport_mode = transport::TransportMode::SocketCan;
  path_like.ports = {{"arm", "/dev/serial/by-id/usb-arm"}};
  EXPECT_THROW(ros::validate_node_parameters(path_like, arm_contract()),
               std::invalid_argument);

  auto empty = parameters("");
  empty.transport_mode = transport::TransportMode::SocketCan;
  empty.ports = {{"arm", ""}};
  EXPECT_THROW(ros::validate_node_parameters(empty, arm_contract()),
               std::invalid_argument);
}

TEST(MotorSimNodeParameters, RejectsRegularFileAndUnsupportedBaud) {
  const std::string path = "/tmp/motor_sim_regular_file";
  std::ofstream(path) << "not a tty";
  auto regular = parameters(path);
  EXPECT_THROW(ros::validate_node_parameters(regular, arm_contract()),
               std::invalid_argument);
  static_cast<void>(::unlink(path.c_str()));

  PseudoTerminal arm;
  auto unsupported = parameters(arm.slave_path());
  unsupported.baud_rate = 12345;
  EXPECT_THROW(ros::validate_node_parameters(unsupported, arm_contract()),
               std::invalid_argument);
}
