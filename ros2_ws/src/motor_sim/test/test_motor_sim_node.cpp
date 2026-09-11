#include "motor_sim/ros/motor_sim_node.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace core = motor_sim::core;
namespace ros = motor_sim::ros;
namespace transport = motor_sim::transport;

namespace {

using namespace std::chrono_literals;

class IdleTransport final : public transport::FrameTransport {
public:
  void open() override { opened_ = true; }
  void close() noexcept override { opened_ = false; }
  bool send(const motor_sim::protocol::CanFrame &) override { return opened_; }
  transport::ReceiveResult receive(transport::ReceivedFrame &) override {
    return transport::ReceiveResult::NoData;
  }

private:
  bool opened_{false};
};

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
  return {{"arm"}, {{"joint_a", "arm", "vaxor", 1U}}};
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

core::SystemConfig one_axis_config() {
  core::SystemConfig config;
  config.timing.control_hz = 100U;
  config.timing.vaxor_feedback_hz = 50U;
  config.timing.command_timeout_ms = 150U;
  config.buses.push_back({"arm"});
  core::MotorConfig motor;
  motor.joint = "joint_a";
  motor.bus = "arm";
  motor.protocol = "vaxor";
  motor.id = 1U;
  motor.limits.command_lower_rad = -1.0;
  motor.limits.command_upper_rad = 1.0;
  motor.limits.effort_nm = 10.0;
  motor.plant.kp = 10.0;
  motor.plant.kd = 1.0;
  motor.plant.effort_limit_nm = 10.0;
  config.motors.push_back(motor);
  return config;
}

void ensure_rclcpp_initialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
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

TEST(MotorSimNodeInterfaces, PublishesBrakeStateAndRejectsResetWithoutFault) {
  ensure_rclcpp_initialized();
  ros::TransportMap transports;
  transports.emplace("arm", std::make_unique<IdleTransport>());
  ros::NodeDefaults defaults;
  defaults.drive_state_topic = "test_drive_state";
  defaults.control_gate_topic = "test_control_gate";
  defaults.reset_fault_service = "test_reset_fault";
  defaults.effort_topic = "test_efforts";

  auto motor_node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, one_axis_config(), std::move(transports), defaults);
  auto observer = std::make_shared<rclcpp::Node>("motor_sim_interface_test");
  std::vector<std::string> states;
  auto state_subscription = observer->create_subscription<std_msgs::msg::String>(
      "test_drive_state", 10,
      [&states](const std_msgs::msg::String &message) {
        states.push_back(message.data);
      });
  auto gate_publisher = observer->create_publisher<std_msgs::msg::Bool>(
      "test_control_gate", 10);
  auto reset_client =
      observer->create_client<std_srvs::srv::Trigger>("test_reset_fault");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(motor_node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (states.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
  ASSERT_FALSE(states.empty());
  EXPECT_EQ(states.back(), "brake_engaged");
  EXPECT_EQ(motor_node->last_efforts(), std::vector<double>(1U, 0.0));

  std_msgs::msg::Bool gate;
  gate.data = true;
  gate_publisher->publish(gate);
  executor.spin_some();
  EXPECT_EQ(motor_node->last_efforts(), std::vector<double>(1U, 0.0));

  ASSERT_TRUE(reset_client->wait_for_service(1s));
  auto future =
      reset_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, 1s),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "当前没有锁存故障");

  static_cast<void>(state_subscription);
  executor.remove_node(observer);
  executor.remove_node(motor_node);
  rclcpp::shutdown();
}
