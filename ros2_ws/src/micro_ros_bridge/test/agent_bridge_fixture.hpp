#pragma once
#include <gtest/gtest.h>
#include <chrono>
#include <functional>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include "micro_ros_bridge/bridge_node.hpp"
#include <can_bridge_msgs/msg/agent_wire_contract.hpp>
using namespace std::chrono_literals;
using namespace can_bridge_msgs::msg;
namespace {
uint64_t stamp() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}
class BridgeTest : public ::testing::Test {
public:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options; options.set_domain_id(176); rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
protected:
  rclcpp::executors::SingleThreadedExecutor exec;
  std::shared_ptr<micro_ros_bridge::BridgeNode> bridge;
  rclcpp::Node::SharedPtr peer;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  rclcpp::Publisher<AgentStatus>::SharedPtr status_pub;
  rclcpp::Publisher<AgentInfo>::SharedPtr info_pub;
  rclcpp::Publisher<AgentMotion>::SharedPtr motion_pub;
  rclcpp::Publisher<MotorCommandArray>::SharedPtr command_pub;
  rclcpp::Publisher<MotorPolicyArray>::SharedPtr policy_pub;
  rclcpp::Publisher<MotorTrajectory>::SharedPtr trajectory_pub;
  rclcpp::Publisher<SystemCommand>::SharedPtr system_command_pub;
  rclcpp::Publisher<ConfigRequest>::SharedPtr maintenance_pub;
  rclcpp::TimerBase::SharedPtr status_timer;
  AgentStatus status;
  std::vector<AgentCommand> wire_grants;
  bool auto_authority = true, initializing_policy = false;
  uint64_t lease_id = 100000001;
  void enable() {
    MotorPolicyArray m; m.count = 1; m.sync_time_ns = stamp(); m.policy_generation = 9;
    MotorPolicyTarget p; p.motor_id = 64; p.enabled = true; p.kp = 20; p.kd = 2;
    m.targets.push_back(p); policy_pub->publish(m);
  }
  std::vector<AgentConfigRequest> configs;
  std::vector<AgentTrajectoryRequest> trajectories;
  std::vector<AgentCommand> commands;
  std::vector<MotorMotionStateArray> motions;
  std::vector<MotorControlStateArray> controls;
  std::vector<MotorTrajectoryState> trajectory_states;
  std::vector<ConfigResult> results;
  std::vector<MotorTopology> topologies;
  std::vector<DeviceParamReport> params;
  bool auto_info = true;
  bool spin(const std::function<bool()> & done, std::chrono::milliseconds limit = 1500ms) {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      exec.spin_some(); if (done()) {return true;} std::this_thread::sleep_for(2ms);
    }
    return done();
  }
  void run(std::chrono::milliseconds time = 35ms) {spin([] {return false;}, time);}
  template<class T> void listen(const std::string & name, std::vector<T> & received,
    rclcpp::QoS qos = rclcpp::QoS(10).reliable()) {
    subscriptions.push_back(peer->create_subscription<T>(name, qos,
      [&received](typename T::SharedPtr m) {received.push_back(*m);}));
  }
  AgentInfo page(const AgentConfigRequest & q) {
    AgentInfo out; out.session_id = status.session_id; out.snapshot_stamp = stamp();
    out.clock_valid = true; out.query_id = q.query_id; out.revision = status.info_revision;
    out.cursor = q.cursor;
    const auto end = std::min<size_t>(status.motors.size(), q.cursor + 16);
    for (size_t i = q.cursor; i < end; ++i) {
      AgentDeviceInfo device; device.motor_id = status.motors[i].motor_id;
      device.parameters.motor_id = device.motor_id; device.protocol = 1;
      device.model_name = "cerb"; device.online = true; out.devices.push_back(device);
    }
    out.next_cursor = static_cast<uint16_t>(end); out.complete = end == status.motors.size();
    return out;
  }
  void publish_status() {
    status.snapshot_stamp = status.clock_valid ? stamp() : 0;
    status.snapshot_tick_ms += 40; ++status.sequence;
    status_pub->publish(status);
  }
  void start(size_t count = 1) {
    status.session_id = 100; status.clock_valid = true; status.info_revision = 1;
    status.command_sequence = 41;
    for (size_t i = 0; i < count; ++i) {
      AgentMotorStatus motor; motor.motor_id = static_cast<uint8_t>(65 - count + i);
      motor.lifecycle = MotorControlState::LIFECYCLE_INACTIVE;
      motor.execution_state = MotorControlState::EXECUTION_IDLE;
      motor.online = true; motor.status_flags = 1;
      status.motors.push_back(motor);
    }
    publish_status();
    status_timer = peer->create_wall_timer(40ms, [this] {publish_status();});
    ASSERT_TRUE(spin([&] {return !topologies.empty();}));
    if (auto_authority) {
      run(100ms); initializing_policy = true; enable();
      ASSERT_TRUE(spin([&] {return !wire_grants.empty() && status.lease_active;}));
      ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(),configs.end(),[](const auto & c) {
        return c.operation == AgentConfigRequest::ACK_RESULTS;});}));
      initializing_policy = false; status.results.clear(); results.clear();
      configs.erase(std::remove_if(configs.begin(),configs.end(),[](const auto & c) {
        return c.operation == AgentConfigRequest::SET_POLICY || c.operation == AgentConfigRequest::ACK_RESULTS;
      }),configs.end());
      run(25ms);
    }
  }
  MotorCommandArray command() {
    MotorCommandArray m; m.sync_time_ns = stamp(); m.count = 1;
    m.session_id = status.session_id; m.lease_id = lease_id;
    MotorCommandTarget t; t.motor_id = 64; t.position = 0.5f; m.targets.push_back(t); return m;
  }
  virtual bool host_execution() const {return false;}
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter(
      "trajectory_execution", host_execution() ? "host" : "firmware")});
    bridge = std::make_shared<micro_ros_bridge::BridgeNode>(options);
    peer = std::make_shared<rclcpp::Node>("fake_agent");
    exec.add_node(bridge); exec.add_node(peer);
    status_pub = peer->create_publisher<AgentStatus>(AgentWireContract::TOPIC_STATUS, 10);
    info_pub = peer->create_publisher<AgentInfo>(AgentWireContract::TOPIC_INFO, 10);
    motion_pub = peer->create_publisher<AgentMotion>(AgentWireContract::TOPIC_MOTION,
      rclcpp::QoS(2).best_effort());
    command_pub = peer->create_publisher<MotorCommandArray>(
      "/motor/host_command_array", 1);
    policy_pub = peer->create_publisher<MotorPolicyArray>("/motor/policy_array", 10);
    trajectory_pub = peer->create_publisher<MotorTrajectory>("/motor/trajectory", 10);
    system_command_pub = peer->create_publisher<SystemCommand>("/motor/system_command", 10);
    maintenance_pub = peer->create_publisher<ConfigRequest>("/motor/config_request", 10);
    subscriptions.push_back(peer->create_subscription<AgentCommand>(AgentWireContract::TOPIC_COMMAND,
      rclcpp::QoS(10).best_effort(), [this](AgentCommand::SharedPtr m) {
        if (m->mode == AgentCommand::AUTHORITY) {
          wire_grants.push_back(*m);
        } else commands.push_back(*m);
        if (m->mode == AgentCommand::AUTHORITY || m->lease_sequence != 0U) {
          if (auto_authority) {
            status.lease_id = m->lease_id; lease_id = m->lease_id;
            status.lease_sequence = m->lease_sequence; status.lease_active = m->lease_active;
            status.lease_valid_until_tick_ms = m->lease_valid_until_tick_ms;
          }
        }
      }));
    listen(AgentWireContract::TOPIC_TRAJECTORY, trajectories);
    listen("/motor/motion_states", motions, rclcpp::QoS(10).best_effort());
    listen("/motor/config_result", results); listen("/motor/control_states", controls);
    listen("/motor/topology", topologies); listen("/motor/device_param_report", params);
    listen("/motor/trajectory_state", trajectory_states);
    subscriptions.push_back(peer->create_subscription<AgentConfigRequest>(
      AgentWireContract::TOPIC_CONFIG, 10, [this](AgentConfigRequest::SharedPtr m) {
        configs.push_back(*m);
        if (initializing_policy && m->operation == AgentConfigRequest::SET_POLICY) {
          AgentConfigResult result; result.request_id=m->request_id; result.operation=m->operation;
          result.state=AgentConfigResult::SUCCEEDED; status.results={result};
        }
        if (auto_info && m->operation == AgentConfigRequest::QUERY_INFO) {info_pub->publish(page(*m));}
      }));
    ASSERT_TRUE(spin([&] {return command_pub->get_subscription_count() &&
      status_pub->get_subscription_count() && info_pub->get_subscription_count() &&
      trajectory_pub->get_subscription_count();}));
    run(50ms);
  }
  void TearDown() override {
    status_timer.reset(); exec.remove_node(bridge); exec.remove_node(peer);
    subscriptions.clear(); bridge.reset(); peer.reset();
  }
};
class HostBridgeTest : public BridgeTest {
  bool host_execution() const override {return true;}
};
}
