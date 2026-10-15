#include "motor_sim/ros/motor_sim_node.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
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

// 环境校验的输入现在是一份已解析的 SystemConfig（总线端点来自 YAML 的 buses[]），
// 不再是 ROS 节点参数——所以测试直接构造 config，逐条总线给端点。
// 传输类型的结构约束（如 serial_slcan 必须显式 opt-in）归 core，这里只测环境。
void add_serial_bus(core::SystemConfig &config, const std::string &name,
                    const std::string &endpoint) {
  core::BusConfig bus;
  bus.name = name;
  bus.transport = core::BusTransport::SerialSlcan;
  bus.endpoint = endpoint;
  bus.baud_rate = 3000000;
  bus.data_bitrate_index = -1;
  bus.allow_external_canfd = true;
  config.buses.push_back(bus);
}

void add_socket_can_bus(core::SystemConfig &config, const std::string &name,
                        const std::string &interface) {
  core::BusConfig bus;
  bus.name = name;
  bus.transport = core::BusTransport::SocketCan;
  bus.endpoint = interface;
  config.buses.push_back(bus);
}

void add_one_axis_motor(core::SystemConfig &config, const std::string &bus) {
  core::MotorConfig motor;
  motor.joint = "joint_a";
  motor.bus = bus;
  motor.protocol = "vaxor";
  motor.id = 1U;
  motor.limits.command_lower_rad = -1.0;
  motor.limits.command_upper_rad = 1.0;
  motor.limits.effort_nm = 10.0;
  motor.plant.kp = 10.0;
  motor.plant.kd = 1.0;
  motor.plant.effort_limit_nm = 10.0;
  config.motors.push_back(motor);
}

core::SystemConfig bus_config(core::BusTransport transport,
                              const std::string &endpoint) {
  core::SystemConfig config;
  config.timing.control_hz = 100U;
  config.timing.vaxor_feedback_hz = 50U;
  config.timing.command_timeout_ms = 150U;
  if (transport == core::BusTransport::SocketCan) {
    add_socket_can_bus(config, "arm", endpoint);
  } else {
    add_serial_bus(config, "arm", endpoint);
  }
  add_one_axis_motor(config, "arm");
  return config;
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
  EXPECT_NO_THROW(ros::validate_bus_endpoints(
      bus_config(core::BusTransport::SerialSlcan, arm.slave_path())));
}

TEST(MotorSimNodeParameters, AcceptsThreeBusPtys) {
  PseudoTerminal left;
  PseudoTerminal right;
  PseudoTerminal legs;
  auto config = bus_config(core::BusTransport::SerialSlcan, left.slave_path());
  config.buses[0].name = "left_arm";
  add_serial_bus(config, "right_arm", right.slave_path());
  add_serial_bus(config, "legs", legs.slave_path());
  EXPECT_NO_THROW(ros::validate_bus_endpoints(config));
}

// 两条总线指向同一个设备：旧行为在 validate_port_mapping_common 里拒绝，
// 改成配置驱动后不能丢——否则两条总线会去开同一个 tty 互相抢字节。
TEST(MotorSimNodeParameters, RejectsDuplicateSerialDevice) {
  PseudoTerminal arm;
  auto config = bus_config(core::BusTransport::SerialSlcan, arm.slave_path());
  add_serial_bus(config, "legs", arm.slave_path());
  EXPECT_THROW(ros::validate_bus_endpoints(config), std::invalid_argument);
}

// 不存在的设备路径：期望加载期就拒绝，而不是等到 open() 才失败。
TEST(MotorSimNodeParameters, RejectsMissingSerialDevice) {
  EXPECT_THROW(ros::validate_bus_endpoints(bus_config(
                   core::BusTransport::SerialSlcan, "/dev/definitely-not-here")),
               std::invalid_argument);
}

TEST(MotorSimNodeParameters, RejectsMissingSocketCanInterface) {
  EXPECT_THROW(ros::validate_bus_endpoints(bus_config(
                   core::BusTransport::SocketCan, "no_such_iface")),
               std::invalid_argument);
}

TEST(MotorSimNodeParameters, SocketCanAcceptsExistingInterfaceName) {
  // socket_can 不使用波特率，取值应被忽略。
  auto config = bus_config(core::BusTransport::SocketCan, "lo");
  config.buses[0].baud_rate = 12345;
  EXPECT_NO_THROW(ros::validate_bus_endpoints(config));
}

TEST(MotorSimNodeParameters, RejectsRegularFileAsSerialPort) {
  const std::string path = "/tmp/motor_sim_regular_file";
  std::ofstream(path) << "not a tty";
  EXPECT_THROW(ros::validate_bus_endpoints(
                   bus_config(core::BusTransport::SerialSlcan, path)),
               std::invalid_argument);
  static_cast<void>(::unlink(path.c_str()));
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

namespace {

// 可注入一帧、记录发出去所有帧的替身；不碰任何真实设备。
class ScriptedTransport final : public transport::FrameTransport {
public:
  void open() override { opened_ = true; }
  void close() noexcept override { opened_ = false; }
  bool send(const motor_sim::protocol::CanFrame &frame) override {
    sent_.push_back(frame);
    return opened_;
  }
  transport::ReceiveResult receive(transport::ReceivedFrame &out) override {
    if (cursor_ >= inbound_.size()) {
      return transport::ReceiveResult::NoData;
    }
    out.frame = inbound_[cursor_];
    out.received_at = std::chrono::steady_clock::now();
    ++cursor_;
    return transport::ReceiveResult::Frame;
  }
  void push(const motor_sim::protocol::CanFrame &frame) {
    inbound_.push_back(frame);
  }
  const std::vector<motor_sim::protocol::CanFrame> &sent() const {
    return sent_;
  }

private:
  bool opened_{false};
  std::vector<motor_sim::protocol::CanFrame> inbound_;
  std::size_t cursor_{0U};
  std::vector<motor_sim::protocol::CanFrame> sent_;
};

motor_sim::protocol::CanFrame sdo_request(
    const std::uint32_t id, const std::array<std::uint8_t, 8> &body) {
  motor_sim::protocol::CanFrame frame;
  frame.id = id;
  frame.fd = true;
  frame.brs = true;
  frame.size = 8U;
  std::copy(body.begin(), body.end(), frame.data.begin());
  return frame;
}

// 一台带 SDO 从站的 CERB 配置，总线指向注入的替身。
core::SystemConfig cerb_bus_config() {
  auto config = bus_config(core::BusTransport::SocketCan, "lo");
  config.motors[0].protocol = "cerb";
  config.motors[0].device_identity = "R48LITE_V2";
  config.motors[0].device_max_speed_rpm = 3325;
  config.motors[0].device_max_current_ma = 5000;
  return config;
}

}  // namespace

// 固件靠 0x600+node 发 SDO 请求、听 0x580+node 的应答。请求帧必须被 SDO 从站
// 接走，**不能**流进 MIT 路由（否则会被当成一条畸形命令）。
TEST(MotorSimNodeSdo, AnswersIdentityRequestOnTheResponseId) {
  ensure_rclcpp_initialized();
  auto transport = std::make_unique<ScriptedTransport>();
  auto *scripted = transport.get();
  ros::TransportMap transports;
  transports.emplace("arm", std::move(transport));

  scripted->push(sdo_request(0x601U, {0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0}));
  auto node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, cerb_bus_config(), std::move(transports));

  for (int attempt = 0; attempt < 100 && scripted->sent().empty(); ++attempt) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_FALSE(scripted->sent().empty());
  const auto &reply = scripted->sent().back();
  EXPECT_EQ(reply.id, 0x581U);
  EXPECT_EQ(reply.data[0], 0x41);
  EXPECT_EQ(reply.data[4], 10);  // 身份串长度
}

// 非本从站的节点号：完全静默。
TEST(MotorSimNodeSdo, StaysSilentForNodesItDoesNotServe) {
  ensure_rclcpp_initialized();
  auto transport = std::make_unique<ScriptedTransport>();
  auto *scripted = transport.get();
  ros::TransportMap transports;
  transports.emplace("arm", std::move(transport));

  scripted->push(sdo_request(0x609U, {0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0}));
  auto node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, cerb_bus_config(), std::move(transports));

  for (int attempt = 0; attempt < 100; ++attempt) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(scripted->sent().empty());
}
