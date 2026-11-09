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
#include <filesystem>
#include "robot_transmission/model.hpp"

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
  core::BusConfig bus;
  bus.name = "arm";
  config.buses.push_back(bus);
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

// 总线设备路径须唯一，防止重复打开同一 tty 抢读字节。
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

TEST(MotorSimNodeTransmission, MapsFeedbackAndEffortByNameAndBlocksIncompleteGroup) {
  ensure_rclcpp_initialized();
  const auto root = std::string("/tmp/motor_sim_transmission_") + std::to_string(getpid());
  const auto driver = root + ".yaml";
  const auto mapping = root + "_groups.yaml";
  {
    std::ofstream out(driver);
    out << "motor_hardware:\n  transmissions_file: " << mapping
        << "\n  motors:\n    a: {id: 1, joint: yaw}\n    b: {id: 2, joint: pitch}\n";
    std::ofstream groups(mapping);
    groups << "transmissions:\n  wrist:\n    plugin: transmission_interface/DifferentialTransmission\n"
           << "    actuators: [a,b]\n    joints: [yaw,pitch]\n";
  }
  auto config = one_axis_config();
  config.timing.command_timeout_ms = 3000U;
  config.motors[0].joint = "yaw";
  config.motors[0].plant.torque_constant_nm_per_a = 1.0;
  config.motors[0].plant.current_limit_a = 10.0;
  auto second = config.motors[0];
  second.id = 2U;
  second.joint = "pitch";
  config.motors.insert(config.motors.begin(), second);
  auto transport = std::make_unique<ScriptedTransport>();
  auto *wire = transport.get();
  ros::TransportMap transports;
  transports.emplace("arm", std::move(transport));
  ros::NodeDefaults defaults;
  defaults.driver_file = driver;
  defaults.joint_names = {"pitch", "yaw"};
  defaults.joint_states_topic = "transmission_test/states";
  defaults.effort_topic = "transmission_test/efforts";
  defaults.control_gate_topic = "transmission_test/gate";
  defaults.drive_state_topic = "transmission_test/drive";
  auto node = std::make_shared<ros::MotorSimNode>(
      rclcpp::NodeOptions{}, config, std::move(transports), defaults);
  auto peer = std::make_shared<rclcpp::Node>("motor_sim_transmission_test");
  auto states = peer->create_publisher<sensor_msgs::msg::JointState>(defaults.joint_states_topic, 10);
  auto gate = peer->create_publisher<std_msgs::msg::Bool>(defaults.control_gate_topic, 10);
  std::string drive_state;
  auto drive = peer->create_subscription<std_msgs::msg::String>(
      defaults.drive_state_topic, 10, [&drive_state](const std_msgs::msg::String &msg) {
        drive_state = msg.data;
      });
  std::vector<double> output;
  auto sub = peer->create_subscription<std_msgs::msg::Float64MultiArray>(
      defaults.effort_topic, 10, [&output](const std_msgs::msg::Float64MultiArray &msg) { output = msg.data; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(peer);
  auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) executor.spin_some();
  };
  const auto discovery_end = std::chrono::steady_clock::now() + 2s;
  while ((!states->get_subscription_count() || !gate->get_subscription_count()) &&
         std::chrono::steady_clock::now() < discovery_end) spin_for(10ms);
  ASSERT_GT(states->get_subscription_count(), 0U);
  ASSERT_GT(gate->get_subscription_count(), 0U);
  std_msgs::msg::Bool open;
  open.data = true;
  for (unsigned id : {1U, 2U}) {
    motor_sim::protocol::CanFrame control;
    control.id = motor_sim::protocol::vaxor_control_id(id);
    control.size = 8U;
    const auto bytes = motor_sim::protocol::pack_vaxor_control(true, false,
        motor_sim::protocol::CommandMode::Mit);
    std::copy(bytes.begin(), bytes.end(), control.data.begin());
    wire->push(control);
    motor_sim::protocol::MotorCommand command;
    command.position = 0.0;
    command.kp = 0.5;
    command.kd = 0.0;
    motor_sim::protocol::CanFrame frame;
    frame.id = motor_sim::protocol::vaxor_command_id(id);
    frame.fd = true;
    frame.size = 12U;
    const auto data = motor_sim::protocol::pack_vaxor_command(command);
    std::copy(data.begin(), data.end(), frame.data.begin());
    wire->push(frame);
  }
  sensor_msgs::msg::JointState feedback;
  feedback.name = {"yaw"};
  feedback.position = {0.2};
  feedback.velocity = {0.0};
  states->publish(feedback);
  spin_for(50ms);
  EXPECT_EQ(node->last_efforts(), (std::vector<double>{0.0, 0.0}));
  const auto ready_end = std::chrono::steady_clock::now() + 2s;
  while (drive_state != "ready_for_control" &&
         std::chrono::steady_clock::now() < ready_end) spin_for(10ms);
  ASSERT_EQ(drive_state, "ready_for_control");
  gate->publish(open);
  feedback.name = {"pitch", "yaw"};
  feedback.position = {0.1, 0.2};
  feedback.velocity = {0.0, 0.0};
  states->publish(feedback);
  spin_for(50ms);
  ASSERT_EQ(output.size(), 2U);
  EXPECT_NEAR(node->last_efforts().at(0), -0.05, 0.002);
  EXPECT_NEAR(node->last_efforts().at(1), -0.15, 0.002);
  EXPECT_NEAR(output[0], -0.1, 0.004);
  EXPECT_NEAR(output[1], -0.2, 0.004);
  spin_for(550ms);
  EXPECT_EQ(node->last_efforts(), (std::vector<double>{0.0, 0.0}));
  executor.remove_node(peer);
  executor.remove_node(node);
  node.reset();
  std::remove(driver.c_str());
  std::remove(mapping.c_str());
  rclcpp::shutdown();
}

class SpatialMotorSimTest : public ::testing::Test {
protected:
  std::filesystem::path root;
  std::shared_ptr<ros::MotorSimNode> node;
  std::shared_ptr<rclcpp::Node> peer;
  ScriptedTransport *wire{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::vector<double> output;
  std::string drive_state;
  void spin_for(std::chrono::milliseconds duration) {
    const auto end=std::chrono::steady_clock::now()+duration;
    while(std::chrono::steady_clock::now()<end) executor->spin_some();
  }
  void command(double target=0) {
    for(unsigned id:{1U,2U,3U}) {
      motor_sim::protocol::CanFrame control;
      control.id=motor_sim::protocol::vaxor_control_id(id); control.size=8U;
      const auto bytes=motor_sim::protocol::pack_vaxor_control(true,false,motor_sim::protocol::CommandMode::Mit);
      std::copy(bytes.begin(),bytes.end(),control.data.begin()); wire->push(control);
      motor_sim::protocol::MotorCommand cmd;
      cmd.position=target; cmd.kp=0; cmd.kd=0; cmd.feedforward=2;
      motor_sim::protocol::CanFrame frame;
      frame.id=motor_sim::protocol::vaxor_command_id(id); frame.fd=true; frame.size=12U;
      const auto data=motor_sim::protocol::pack_vaxor_command(cmd);
      std::copy(data.begin(),data.end(),frame.data.begin()); wire->push(frame);
    }
  }
  void feedback(std::vector<std::string> names, std::vector<double> positions) {
    sensor_msgs::msg::JointState msg;
    msg.name=std::move(names); msg.position=std::move(positions); msg.velocity.assign(msg.name.size(),0.);
    states->publish(msg); spin_for(25ms);
  }
  void SetUp() override {
    ensure_rclcpp_initialized();
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    char directory[]="/tmp/spatial_motor_sim_XXXXXX"; root=mkdtemp(directory);
    std::filesystem::copy_file(std::filesystem::path(__FILE__).parent_path()/"fixtures/spatial_transmissions.yaml",root/"transmissions.yaml");
    std::ofstream(root/"driver.yaml") << "motor_hardware:\n  transmissions_file: transmissions.yaml\n  motors:\n"
      "    a: {id: 1, joint: yaw}\n    b: {id: 2, joint: pitch}\n    c: {id: 3, joint: elbow}\n";
    auto config=one_axis_config(); config.timing.command_timeout_ms=3000;
    config.motors[0].joint="yaw";
    config.motors[0].plant.torque_constant_nm_per_a=1;
    config.motors[0].plant.current_limit_a=10;
    auto second=config.motors[0]; second.id=2; second.joint="pitch";
    auto third=config.motors[0]; third.id=3; third.joint="elbow";
    config.motors.insert(config.motors.begin(),second); config.motors.push_back(third);
    auto scripted=std::make_unique<ScriptedTransport>(); wire=scripted.get();
    ros::TransportMap transports; transports.emplace("arm",std::move(scripted));
    ros::NodeDefaults defaults;
    defaults.driver_file=(root/"driver.yaml").string(); defaults.joint_names={"pitch","elbow","yaw"};
    defaults.joint_states_topic="spatial/states"; defaults.effort_topic="spatial/efforts";
    defaults.control_gate_topic="spatial/gate"; defaults.drive_state_topic="spatial/drive";
    node=std::make_shared<ros::MotorSimNode>(rclcpp::NodeOptions{},config,std::move(transports),defaults);
    peer=std::make_shared<rclcpp::Node>("spatial_motor_sim_peer");
    states=peer->create_publisher<sensor_msgs::msg::JointState>(defaults.joint_states_topic,10);
    gate=peer->create_publisher<std_msgs::msg::Bool>(defaults.control_gate_topic,10);
    sub=peer->create_subscription<std_msgs::msg::Float64MultiArray>(defaults.effort_topic,10,
      [this](const std_msgs::msg::Float64MultiArray &msg){output=msg.data;});
    drive=peer->create_subscription<std_msgs::msg::String>(defaults.drive_state_topic,10,
      [this](const std_msgs::msg::String &msg){drive_state=msg.data;});
    executor->add_node(node); executor->add_node(peer);
    const auto end=std::chrono::steady_clock::now()+2s;
    while((!states->get_subscription_count() || !gate->get_subscription_count()) && std::chrono::steady_clock::now()<end) spin_for(10ms);
    ASSERT_GT(states->get_subscription_count(),0U);
    command();
    while(drive_state!="ready_for_control" && std::chrono::steady_clock::now()<end) spin_for(10ms);
    ASSERT_EQ(drive_state,"ready_for_control");
    std_msgs::msg::Bool open; open.data=true; gate->publish(open); spin_for(20ms);
  }
  void TearDown() override {
    if(peer) executor->remove_node(peer);
    if(node) executor->remove_node(node);
    sub.reset(); drive.reset(); states.reset(); gate.reset(); peer.reset(); node.reset();
    executor.reset();
    std::filesystem::remove_all(root);
    rclcpp::shutdown();
  }
  void expect_pose_torque(double yaw, double pitch) {
    auto model=robot_transmission::Model::load((root/"driver.yaml").string());
    auto a=model.joint_to_actuator({{"yaw",robot_transmission::Axis(yaw)},{"pitch",robot_transmission::Axis(pitch)},
                                  {"elbow",robot_transmission::Axis(.1)}});
    const auto efforts=node->last_efforts();
    ASSERT_EQ(efforts.size(),3U); ASSERT_GT(efforts[0],1.5);
    auto expected=model.actuator_effort_to_joint({{"a",efforts[1]},{"b",efforts[0]},{"c",efforts[2]}},
      {{"a",a.at("a").position},{"b",a.at("b").position},{"c",a.at("c").position}});
    ASSERT_EQ(output.size(),3U);
    EXPECT_NEAR(output[0],expected.at("pitch"),1e-9);
    EXPECT_NEAR(output[1],expected.at("elbow"),1e-9);
    EXPECT_NEAR(output[2],expected.at("yaw"),1e-9);
  }
};
TEST_F(SpatialMotorSimTest, NonlinearEffortFollowsMeasuredPoseNotCommand) {
  feedback({"elbow","pitch","yaw"},{.1,.08,.17});
  expect_pose_torque(.17,.08); const auto first=output;
  command(.5); feedback({"yaw","elbow","pitch"},{.17,.1,.08});
  expect_pose_torque(.17,.08); EXPECT_NEAR(output[0],first[0],1e-9);
  feedback({"pitch","yaw","elbow"},{-.05,.1,.1});
  expect_pose_torque(.1,-.05); EXPECT_GT(std::abs(output[0]-first[0]),.01);
}
TEST_F(SpatialMotorSimTest, InvalidNonlinearGroupDoesNotReuseCachedPose) {
  feedback({"yaw","elbow"},{.17,.1});
  ASSERT_EQ(output.size(),3U); EXPECT_EQ(output[0],0); EXPECT_EQ(output[2],0); EXPECT_GT(output[1],1.5);
  feedback({"pitch","yaw","elbow"},{.08,.17,.1}); expect_pose_torque(.17,.08);
  spin_for(70ms); feedback({"pitch","elbow"},{.08,.1});
  EXPECT_EQ(output[0],0); EXPECT_EQ(output[2],0); EXPECT_GT(output[1],1.5);
  feedback({"pitch","yaw","elbow"},{.08,.17,.1}); expect_pose_torque(.17,.08);
  spin_for(510ms); feedback({"elbow"},{.1});
  EXPECT_EQ(output[0],0); EXPECT_EQ(output[2],0); EXPECT_GT(output[1],1.5);
  feedback({"pitch","yaw","elbow"},{1.57,0,.1});
  EXPECT_EQ(output[0],0); EXPECT_EQ(output[2],0); EXPECT_GT(output[1],1.5);
}
