#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/codec.hpp"

using namespace std::chrono_literals;
using namespace can_bridge_msgs::msg;
namespace topic = micro_ros_bridge::contract::topic;

namespace
{
class LeaseBridge : public ::testing::Test
{
protected:
  static constexpr uint64_t kSession = 1800000000000ULL;
  static constexpr uint64_t kGrant = kSession * 1000000ULL + 1;
  std::shared_ptr<micro_ros_bridge::BridgeNode> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<MotorStatus>::SharedPtr statuses;
  rclcpp::Publisher<MotorTopology>::SharedPtr topologies;
  rclcpp::Publisher<MotorControlStateArray>::SharedPtr controls;
  rclcpp::Publisher<ControlLease>::SharedPtr grants;
  rclcpp::Publisher<MotorCommandArray>::SharedPtr commands;
  rclcpp::Publisher<MotorPolicyArray>::SharedPtr policies;
  rclcpp::Publisher<MotorTrajectory>::SharedPtr trajectories;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::vector<ControlLease> wire_grants;
  std::vector<LowCommand> wire_commands;
  std::vector<MotorPolicyArray> wire_policies;
  std::vector<MotorTrajectory> wire_trajectories;
  uint32_t status_sequence = 0;
  uint32_t control_sequence = 0;
  std::chrono::steady_clock::time_point started;

  uint64_t stamp() const {return node->now().nanoseconds();}
  uint32_t tick() const
  {
    return 1000u + static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started).count());
  }
  bool until(const std::function<bool()> & done, std::chrono::milliseconds duration = 1s)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some();
      if (done()) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return done();
  }
  void spin(std::chrono::milliseconds duration = 20ms)
  {until([] {return false;}, duration);}

  void SetUp() override
  {
    rclcpp::InitOptions options;
    options.set_domain_id(175);
    rclcpp::init(0, nullptr, options);
    started = std::chrono::steady_clock::now();
    node = std::make_shared<micro_ros_bridge::BridgeNode>();
    exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    const auto qos = rclcpp::QoS(10).reliable();
    statuses = node->create_publisher<MotorStatus>(topic::kMotorStatus, qos);
    topologies = node->create_publisher<MotorTopology>(topic::kTopology, qos);
    controls = node->create_publisher<MotorControlStateArray>(topic::kControlStates, qos);
    grants = node->create_publisher<ControlLease>("/motor/control_lease", qos);
    commands = node->create_publisher<MotorCommandArray>("/motor/command_array", qos);
    policies = node->create_publisher<MotorPolicyArray>("/motor/policy_array", qos);
    trajectories = node->create_publisher<MotorTrajectory>("/motor/trajectory", qos);
    subscriptions.push_back(node->create_subscription<ControlLease>(topic::kControlLease, qos,
      [this](ControlLease::SharedPtr msg) {wire_grants.push_back(*msg);}));
    subscriptions.push_back(node->create_subscription<LowCommand>(topic::kLowCmd,
      rclcpp::QoS(1).best_effort(),
      [this](LowCommand::SharedPtr msg) {wire_commands.push_back(*msg);}));
    subscriptions.push_back(node->create_subscription<MotorPolicyArray>(topic::kPolicy, qos,
      [this](MotorPolicyArray::SharedPtr msg) {wire_policies.push_back(*msg);}));
    subscriptions.push_back(node->create_subscription<MotorTrajectory>(topic::kTrajectory, qos,
      [this](MotorTrajectory::SharedPtr msg) {wire_trajectories.push_back(*msg);}));
    ASSERT_TRUE(until([this] {
      return statuses->get_subscription_count() && topologies->get_subscription_count() &&
             controls->get_subscription_count() && grants->get_subscription_count() &&
             commands->get_subscription_count() && policies->get_subscription_count() &&
             trajectories->get_subscription_count() &&
             node->count_subscribers(topic::kControlLease) &&
             node->count_subscribers(topic::kLowCmd) &&
             node->count_subscribers(topic::kPolicy) &&
             node->count_subscribers(topic::kTrajectory);
    }));
    refresh();
    controls->publish(ack(0));
    spin();
  }
  void TearDown() override
  {
    exec->remove_node(node);
    subscriptions.clear();
    statuses.reset(); topologies.reset(); controls.reset(); grants.reset();
    commands.reset(); policies.reset(); trajectories.reset();
    node.reset(); exec.reset();
    rclcpp::shutdown();
  }
  void refresh()
  {
    MotorStatus status;
    status.schema_version = WireContract::SCHEMA_VERSION;
    status.session_id = kSession;
    status.sequence = ++status_sequence;
    status.tick_ms = tick();
    status.sync_time_ns = stamp();
    statuses->publish(status);
    spin(5ms);
    MotorTopology topology;
    topology.schema_version = WireContract::SCHEMA_VERSION;
    topology.session_id = kSession;
    topology.sync_time_ns = stamp();
    topologies->publish(topology);
    spin(5ms);
  }
  MotorControlStateArray ack(uint64_t grant = kGrant, uint32_t lifetime = 250)
  {
    MotorControlStateArray msg;
    msg.schema_version = MotorControlStateArray::SCHEMA_VERSION;
    msg.session_id = kSession;
    msg.sequence = ++control_sequence;
    msg.tick_ms = tick();
    msg.sync_time_ns = stamp();
    msg.lease_id = grant;
    msg.lease_sequence = grant ? 1 : 0;
    msg.lease_active = grant != 0;
    msg.lease_valid_until_tick_ms = grant ? msg.tick_ms + lifetime : 0;
    return msg;
  }
  ControlLease grant(uint64_t id = kGrant, uint32_t sequence = 1, uint32_t lifetime = 250)
  {
    ControlLease msg;
    msg.session_id = kSession;
    msg.lease_id = id;
    msg.sequence = sequence;
    msg.valid_until_tick_ms = tick() + lifetime;
    msg.active = true;
    return msg;
  }
  MotorCommandArray command(uint64_t id = kGrant)
  {
    MotorCommandArray msg;
    msg.sync_time_ns = stamp();
    msg.session_id = kSession;
    msg.lease_id = id;
    msg.count = 1;
    MotorCommandTarget target;
    target.motor_id = 0;
    target.position = .5;
    msg.targets.push_back(target);
    return msg;
  }
  MotorPolicyArray policy(uint64_t id = kGrant)
  {
    MotorPolicyArray msg;
    msg.sync_time_ns = stamp();
    msg.session_id = kSession;
    msg.lease_id = id;
    msg.count = 1;
    MotorPolicyTarget target;
    target.motor_id = 0;
    target.enabled = true;
    msg.targets.push_back(target);
    return msg;
  }
  MotorTrajectory trajectory(uint64_t id = kGrant)
  {
    MotorTrajectory msg;
    msg.sync_time_ns = stamp();
    msg.session_id = kSession;
    msg.lease_id = id;
    msg.count = 1;
    msg.segment_count = 1;
    msg.target_count = 1;
    msg.target_ids = {0};
    MotorTrajectoryPoint point;
    point.motor_id = 0;
    msg.points.push_back(point);
    return msg;
  }
};
}  // namespace

TEST_F(LeaseBridge, HeartbeatKeepsItsDeadlineAndDoesNotAuthorizeBeforeMcuAck)
{
  auto request = grant();
  grants->publish(request);
  ASSERT_TRUE(until([this] {return wire_grants.size() == 1;}));
  EXPECT_EQ(wire_grants[0], request);
  commands->publish(command());
  policies->publish(policy());
  trajectories->publish(trajectory());
  spin();
  EXPECT_TRUE(wire_commands.empty());
  EXPECT_TRUE(wire_policies.empty());
  EXPECT_TRUE(wire_trajectories.empty());
  controls->publish(ack());
  spin();
  commands->publish(command());
  policies->publish(policy());
  trajectories->publish(trajectory());
  ASSERT_TRUE(until([this] {return wire_commands.size() == 1 &&
    wire_policies.size() == 1 && wire_trajectories.size() == 1;}));
  EXPECT_EQ(wire_commands[0].lease_id, kGrant);
  EXPECT_EQ(wire_policies[0].lease_id, kGrant);
  EXPECT_EQ(wire_trajectories[0].lease_id, kGrant);
}

TEST_F(LeaseBridge, LegacyOrWrongLeaseAndSessionCannotBeUpgraded)
{
  controls->publish(ack());
  spin();
  for (const uint64_t id : std::vector<uint64_t>{0, kGrant + 1}) {
    commands->publish(command(id));
    policies->publish(policy(id));
    trajectories->publish(trajectory(id));
    spin(10ms);
  }
  auto bad_command = command(); bad_command.session_id = kSession - 1;
  auto bad_policy = policy(); bad_policy.session_id = kSession - 1;
  auto bad_trajectory = trajectory(); bad_trajectory.session_id = kSession - 1;
  commands->publish(bad_command);
  policies->publish(bad_policy);
  trajectories->publish(bad_trajectory);
  spin();
  EXPECT_TRUE(wire_commands.empty());
  EXPECT_TRUE(wire_policies.empty());
  EXPECT_TRUE(wire_trajectories.empty());
}

TEST_F(LeaseBridge, HeartbeatReplayWrongPartitionAndExpiredDeadlineAreRejected)
{
  auto valid = grant();
  grants->publish(valid);
  ASSERT_TRUE(until([this] {return wire_grants.size() == 1;}));
  grants->publish(valid);
  spin();
  EXPECT_EQ(wire_grants.size(), 1u);
  auto bad = grant(kGrant, 2); bad.session_id -= 1;
  grants->publish(bad); spin(10ms);
  bad = grant(kGrant + 1000000, 2);
  grants->publish(bad); spin(10ms);
  bad = grant(kGrant, 2, 1000);
  grants->publish(bad); spin(10ms);
  bad = grant(kGrant, 2); bad.valid_until_tick_ms = tick() - 1;
  grants->publish(bad); spin(10ms);
  EXPECT_EQ(wire_grants.size(), 1u);
  auto renewal = grant(kGrant, 2);
  grants->publish(renewal);
  ASSERT_TRUE(until([this] {return wire_grants.size() == 2;}));
  EXPECT_EQ(wire_grants.back().valid_until_tick_ms, renewal.valid_until_tick_ms);
}

TEST_F(LeaseBridge, ExpiryAndControlReplayDoNotRefreshMotionPermission)
{
  auto short_ack = ack(kGrant, 80);
  controls->publish(short_ack);
  spin(10ms);
  commands->publish(command());
  ASSERT_TRUE(until([this] {return wire_commands.size() == 1;}));
  spin(90ms);
  // Replaying the same control sequence with a fabricated new source time and
  // deadline must not reopen a lease that elapsed locally.
  short_ack.sync_time_ns = stamp();
  short_ack.tick_ms = tick();
  short_ack.lease_valid_until_tick_ms = tick() + 250;
  controls->publish(short_ack);
  spin(10ms);
  commands->publish(command());
  spin();
  EXPECT_EQ(wire_commands.size(), 1u);
}

TEST_F(LeaseBridge, RevokeClosesGateAndOnlyNewLeaseCanResume)
{
  controls->publish(ack());
  spin(10ms);
  auto revoke = grant(); revoke.active = false;
  grants->publish(revoke);
  ASSERT_TRUE(until([this] {return wire_grants.size() == 1;}));
  controls->publish(ack());
  spin(10ms);
  commands->publish(command());
  spin();
  EXPECT_TRUE(wire_commands.empty());
  grants->publish(grant(kGrant, 2));
  spin();
  EXPECT_EQ(wire_grants.size(), 1u);
  grants->publish(grant(kGrant + 1));
  ASSERT_TRUE(until([this] {return wire_grants.size() == 2;}));
  controls->publish(ack(kGrant + 1));
  spin(10ms);
  commands->publish(command());
  spin();
  EXPECT_TRUE(wire_commands.empty());
  commands->publish(command(kGrant + 1));
  ASSERT_TRUE(until([this] {return wire_commands.size() == 1;}));
}
