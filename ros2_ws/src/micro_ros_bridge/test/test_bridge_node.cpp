// 节点图接线测试：话题名与 QoS 是**契约**，不是实现细节。
// 话题名取自 WireContract 常量，QoS 按消息性质选（高频流 vs 低频状态声明）。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/codec.hpp"

namespace {

using namespace std::chrono_literals;

struct RclcppSession
{
  RclcppSession() {rclcpp::init(0, nullptr);}
  ~RclcppSession() {rclcpp::shutdown();}
};

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & exec, const std::function<bool()> & done,
  std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return done();
}

void spin_for(rclcpp::executors::SingleThreadedExecutor & exec, std::chrono::milliseconds duration)
{
  spin_until(exec, [] {return false;}, duration);
}

}  // namespace

// 线上话题名是契约：少一个、拼错一个，症状都是"页面某块永远没数"。
TEST(BridgeNode, WireTopicsAreDeclared) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  namespace t = micro_ros_bridge::contract::topic;
  for (const auto * name : {&t::kLowCmd, &t::kPolicy, &t::kTrajectory, &t::kConfig,
      &t::kLowState, &t::kMotorStatus, &t::kTopology, &t::kEvents, &t::kDeviceParam,
      &t::kImuData, &t::kImuTemp})
  {
    EXPECT_EQ(topics.count(*name), 1u) << "线上话题缺失：" << *name;
  }
}

// 主机侧话题名是页面与动作库的接口。
TEST(BridgeNode, HostTopicsAreDeclared) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  for (const char * name : {"/motor/command_array", "/motor/motion_states",
      "/motor/status_array", "/motor/trajectory_state", "/motor/system_status",
      "/motor/diagnostics", "/motor/param_set", "/motor/system_command",
      "/motor/policy_array", "/motor/trajectory", "/motor/topology", "/motor/events",
      "/motor/device_param_report", "/imu/data", "/imu/temp"})
  {
    EXPECT_EQ(topics.count(name), 1u) << "主机侧话题缺失：" << name;
  }
}

// 端到端（同进程）：主机侧命令进 → 线上 low_cmd 出，且已被定点化。
TEST(BridgeNode, CommandArrayBecomesLowCommand) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  auto pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  can_bridge_msgs::msg::LowCommand got;
  std::atomic<bool> received{false};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](can_bridge_msgs::msg::LowCommand::SharedPtr m) { got = *m; received = true; });

  can_bridge_msgs::msg::MotorCommandArray in;
  in.count = 1;
  can_bridge_msgs::msg::MotorCommandTarget t;
  t.motor_id = 0;
  t.position = 0.5;
  in.targets.push_back(t);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  pub->publish(in);
  ASSERT_TRUE(spin_until(exec, [&] {return received.load();})) << "2 s 内没有收到 low_cmd";
  EXPECT_EQ(got.count, 1);
  EXPECT_EQ(got.motor_ids[0], 0);
  ASSERT_EQ(got.position_q.size(), 1u);
  EXPECT_EQ(got.position_q[0], 1311);  // round(0.5 * 32767 / 12.5)
}

TEST(BridgeNode, PolicyFlowsFromHostToWire) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  using Message = can_bridge_msgs::msg::MotorPolicyArray;
  auto pub = node->create_publisher<Message>("/motor/policy_array", rclcpp::QoS(10).reliable());
  Message got;
  bool received = false;
  auto sub = node->create_subscription<Message>(
    micro_ros_bridge::contract::topic::kPolicy, rclcpp::QoS(10).reliable(),
    [&](Message::SharedPtr m) {got = *m; received = true;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  Message in;
  in.request_id = 123;
  in.policy_generation = 8;
  pub->publish(in);
  ASSERT_TRUE(spin_until(exec, [&] {return received;}));
  EXPECT_EQ(got.request_id, 123u);
  EXPECT_EQ(got.policy_generation, 8u);
}

TEST(BridgeNode, TrajectoryFlowsFromHostToWire) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  using Message = can_bridge_msgs::msg::MotorTrajectory;
  auto pub = node->create_publisher<Message>("/motor/trajectory", rclcpp::QoS(10).reliable());
  Message got;
  bool received = false;
  auto sub = node->create_subscription<Message>(
    micro_ros_bridge::contract::topic::kTrajectory, rclcpp::QoS(10).reliable(),
    [&](Message::SharedPtr m) {got = *m; received = true;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  Message in;
  in.sequence = 17;
  in.trajectory_id = 456;
  in.segment_count = 1;
  pub->publish(in);
  ASSERT_TRUE(spin_until(exec, [&] {return received;}));
  EXPECT_EQ(got.sequence, 17u);
  EXPECT_EQ(got.trajectory_id, 456u);
  EXPECT_EQ(got.segment_count, 1u);
}

TEST(BridgeNode, StreamAndReliableEndpointsUseExpectedQoS) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  namespace t = micro_ros_bridge::contract::topic;
  const auto check = [&](const std::string & topic, bool publisher,
      rclcpp::ReliabilityPolicy reliability) {
      const auto endpoints = publisher ? node->get_publishers_info_by_topic(topic) :
        node->get_subscriptions_info_by_topic(topic);
      ASSERT_EQ(endpoints.size(), 1u) << topic;
      EXPECT_EQ(endpoints[0].qos_profile().reliability(), reliability) << topic;
    };
  check(t::kLowCmd, true, rclcpp::ReliabilityPolicy::BestEffort);
  check(t::kLowState, false, rclcpp::ReliabilityPolicy::BestEffort);
  check(t::kPolicy, true, rclcpp::ReliabilityPolicy::Reliable);
  check("/motor/policy_array", false, rclcpp::ReliabilityPolicy::Reliable);
  check(t::kTrajectory, true, rclcpp::ReliabilityPolicy::Reliable);
  check("/motor/trajectory", false, rclcpp::ReliabilityPolicy::Reliable);
  check(t::kMotorStatus, false, rclcpp::ReliabilityPolicy::Reliable);
  check("/motor/status_array", true, rclcpp::ReliabilityPolicy::Reliable);
}

TEST(BridgeNode, OnlyValidMatchingStatusOpensSchemaGate) {
  RclcppSession session;
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  auto status_pub = node->create_publisher<can_bridge_msgs::msg::MotorStatus>(
    micro_ros_bridge::contract::topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto cmd_pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  std::atomic<int> low_cmd_count{0};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&low_cmd_count](can_bridge_msgs::msg::LowCommand::SharedPtr) {++low_cmd_count;});
  int status_count = 0;
  auto status_sub = node->create_subscription<can_bridge_msgs::msg::MotorStatusArray>(
    "/motor/status_array", rclcpp::QoS(10).reliable(),
    [&](can_bridge_msgs::msg::MotorStatusArray::SharedPtr) {++status_count;});
  ASSERT_TRUE(spin_until(exec, [&] {
    return status_pub->get_subscription_count() > 0 && cmd_pub->get_subscription_count() > 0 &&
           sub->get_publisher_count() > 0 && status_sub->get_publisher_count() > 0;
  }));

  can_bridge_msgs::msg::MotorCommandArray in;
  in.count = 1;
  can_bridge_msgs::msg::MotorCommandTarget target;
  target.motor_id = 0;
  in.targets.push_back(target);
  cmd_pub->publish(in);
  ASSERT_TRUE(spin_until(exec, [&] {return low_cmd_count.load() == 1;}));

  auto announce_valid_schema = [&](uint16_t v) {
      can_bridge_msgs::msg::MotorStatus st;
      st.schema_version = v;
      st.count = 0;
      status_pub->publish(st);
      return spin_until(exec, [&] {return status_count > 0;});
    };
  auto expect_blocked = [&] {
      const int before = low_cmd_count.load();
      cmd_pub->publish(in);
      spin_for(exec, 150ms);
      EXPECT_EQ(low_cmd_count.load(), before);
    };

  ASSERT_TRUE(announce_valid_schema(99));
  expect_blocked();

  status_count = 0;
  ASSERT_TRUE(announce_valid_schema(0));
  expect_blocked();

  can_bridge_msgs::msg::MotorStatus malformed;
  malformed.schema_version = micro_ros_bridge::contract::kSchemaVersion;
  malformed.count = 1;
  malformed.motor_ids = {0};  // 其余逐台数组缺失
  status_pub->publish(malformed);
  spin_for(exec, 100ms);
  expect_blocked();

  status_count = 0;
  ASSERT_TRUE(announce_valid_schema(micro_ros_bridge::contract::kSchemaVersion));
  const int before_recovery = low_cmd_count.load();
  cmd_pub->publish(in);
  EXPECT_TRUE(spin_until(exec, [&] {return low_cmd_count.load() > before_recovery;}));
}
