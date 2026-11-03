#include "agent_bridge_fixture.hpp"
namespace {
class BenchControl : public HostBridgeTest {
protected:
  void SetUp() override {auto_authority = false; HostBridgeTest::SetUp(); start(); run(100ms);}
  void ack() {
    const auto & grant = wire_grants.back();
    status.lease_id = grant.lease_id; status.lease_sequence = grant.lease_sequence;
    status.lease_active = grant.lease_active;
    status.lease_valid_until_tick_ms = grant.lease_valid_until_tick_ms;
    publish_status();
  }
};
}
TEST_F(BenchControl, StartupAndCommandsCannotEnableTransport) {
  command_pub->publish(command()); run(80ms);
  EXPECT_TRUE(wire_grants.empty()); EXPECT_TRUE(commands.empty());
}
TEST_F(BenchControl, ExplicitEnableWaitsForMcuAckAndPreservesSourceTimestamp) {
  enable(); ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));
  EXPECT_TRUE(wire_grants.back().lease_active);
  EXPECT_EQ(wire_grants.back().sequence, 0u);
  command_pub->publish(command()); run(10ms); EXPECT_TRUE(commands.empty());
  EXPECT_TRUE(std::none_of(configs.begin(),configs.end(),[](const auto & c){
    return c.operation == AgentConfigRequest::SET_POLICY;}));
  ack();
  ASSERT_TRUE(spin([&]{return std::any_of(configs.begin(),configs.end(),[](const auto & c){
    return c.operation == AgentConfigRequest::SET_POLICY;});}));
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==1;}));
  EXPECT_EQ(commands.back().lease_id, wire_grants.back().lease_id);
}
TEST_F(BenchControl, UnconfirmedEnableExpiresWithoutReplay) {
  enable(); ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));
  const auto grant=wire_grants.back(); run(140ms);
  EXPECT_FALSE(wire_grants.back().lease_active);
  status.lease_id=grant.lease_id; status.lease_sequence=grant.lease_sequence;
  status.lease_active=true; status.lease_valid_until_tick_ms=status.snapshot_tick_ms+200;
  publish_status(); command_pub->publish(command()); run();
  EXPECT_TRUE(commands.empty());
  EXPECT_TRUE(std::none_of(configs.begin(),configs.end(),[](const auto & c){
    return c.operation == AgentConfigRequest::SET_POLICY;}));
}
TEST_F(BenchControl, StopCancelsPendingEnableAndRequiresFreshExplicitEnable) {
  auto stop=peer->create_publisher<std_msgs::msg::Empty>("/control/stop",10); run(50ms);
  enable(); ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));
  const auto previous=wire_grants.back().lease_id;
  stop->publish(std_msgs::msg::Empty{}); run(); ack(); run();
  EXPECT_FALSE(wire_grants.back().lease_active);
  command_pub->publish(command()); run(); EXPECT_TRUE(commands.empty());
  enable(); ASSERT_TRUE(spin([&]{return wire_grants.back().lease_active;}));
  EXPECT_GT(wire_grants.back().lease_id, previous);
}
TEST_F(BenchControl, DisabledPolicyCancelsUnconfirmedEnable) {
  enable(); ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));
  MotorPolicyArray off;off.count=1;off.targets.push_back(MotorPolicyTarget{});
  off.targets[0].motor_id=64; policy_pub->publish(off);run(10ms);ack();run(20ms);
  EXPECT_TRUE(std::none_of(configs.begin(),configs.end(),[](const auto & c){
    return c.operation==AgentConfigRequest::SET_POLICY && !c.policies.empty() && c.policies[0].enabled;}));
}
TEST_F(BenchControl, LostStatusRevokesAndDoesNotRecoverFromStatusAlone) {
  enable();ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));ack();run(25ms);
  status_timer.reset(); run(320ms);
  EXPECT_FALSE(wire_grants.back().lease_active);
  publish_status(); command_pub->publish(command());run();EXPECT_TRUE(commands.empty());
}

TEST_F(BenchControl, QueuedEnableFromBeforeStopCannotRestart) {
  auto stop=peer->create_publisher<std_msgs::msg::Empty>("/control/stop",10);run(50ms);
  MotorPolicyArray old;old.sync_time_ns=stamp();old.count=1;old.policy_generation=9;
  MotorPolicyTarget p;p.motor_id=64;p.enabled=true;old.targets.push_back(p);
  stop->publish(std_msgs::msg::Empty{});run(5ms);
  policy_pub->publish(old);run(35ms);EXPECT_TRUE(wire_grants.empty());
  enable();ASSERT_TRUE(spin([&]{return !wire_grants.empty();}));
}
TEST_F(BenchControl, BrowserCorrelationIdDoesNotBecomeWireRequestId) {
  auto_authority=true;
  MotorPolicyArray m;m.sync_time_ns=stamp();m.request_id=0x70000000u;m.count=1;m.policy_generation=19;
  MotorPolicyTarget p;p.motor_id=64;p.enabled=true;m.targets.push_back(p);policy_pub->publish(m);
  ASSERT_TRUE(spin([&]{return std::any_of(configs.begin(),configs.end(),[](const auto & c){
    return c.operation==AgentConfigRequest::SET_POLICY;});}));
  const auto found=std::find_if(configs.begin(),configs.end(),[](const auto & c){
    return c.operation==AgentConfigRequest::SET_POLICY;});
  EXPECT_LT(found->request_id,AgentWireContract::REQUEST_ID_MAX);
}

TEST_F(BenchControl, ActiveSnapshotsRenewAuthorityWithoutExtraHeartbeats) {
  auto_authority = true;
  enable(); ASSERT_TRUE(spin([&]{return status.lease_active;}));
  run(40ms);
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return !commands.empty();}));
  wire_grants.clear(); commands.clear();
  for (int i = 0; i < 10; ++i) {
    command_pub->publish(command()); run(20ms);
  }
  ASSERT_EQ(commands.size(), 10u);
  for (const auto & snapshot : commands) {
    EXPECT_TRUE(snapshot.lease_active);
    EXPECT_GT(snapshot.lease_sequence, 0u);
    EXPECT_GT(snapshot.lease_valid_until_tick_ms, snapshot.valid_until_tick_ms);
  }
  EXPECT_TRUE(wire_grants.empty());
  const auto last = commands.back();
  // Agent and authority may stay alive after the command source exits.
  run(150ms);
  EXPECT_EQ(commands.size(), 10u);
  EXPECT_EQ(commands.back().sequence, last.sequence);
  EXPECT_EQ(commands.back().valid_until_tick_ms, last.valid_until_tick_ms);
}

TEST_F(HostBridgeTest, PreviousSchemaCannotCompleteTopologyHandshake) {
  auto_info = false;
  status.session_id = 100; status.clock_valid = true; status.info_revision = 1;
  status.schema_version = 3;
  publish_status(); run(40ms);
  EXPECT_TRUE(topologies.empty());
  EXPECT_TRUE(configs.empty());
  status.schema_version = AgentWireContract::SCHEMA_VERSION;
  status_timer = peer->create_wall_timer(40ms, [this] {publish_status();});
  publish_status();
  ASSERT_TRUE(spin([&]{return std::any_of(configs.begin(), configs.end(), [](const auto & request) {
    return request.operation == AgentConfigRequest::QUERY_INFO;
  });}));
  const auto query = std::find_if(configs.begin(), configs.end(), [](const auto & request) {
    return request.operation == AgentConfigRequest::QUERY_INFO;
  });
  ASSERT_NE(query, configs.end());
  auto old = page(*query); old.schema_version = 3;
  info_pub->publish(old); run(40ms);
  EXPECT_TRUE(topologies.empty());
  old.schema_version = AgentWireContract::SCHEMA_VERSION;
  info_pub->publish(old);
  ASSERT_TRUE(spin([&]{return !topologies.empty();}));
}

TEST_F(BenchControl, ExpiredSourceAndClockRecoveryCannotResumeMotion) {
  auto_authority = true;
  enable(); ASSERT_TRUE(spin([&]{return status.lease_active;})); run(40ms);
  const auto old = command();
  command_pub->publish(old); ASSERT_TRUE(spin([&]{return !commands.empty();}));
  const auto deadline = commands.back().valid_until_tick_ms;
  run(40ms); command_pub->publish(old); run(10ms);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.back().valid_until_tick_ms, deadline);
  status.clock_valid = false; publish_status(); run(40ms);
  EXPECT_FALSE(wire_grants.back().lease_active);
  status.clock_valid = true; publish_status(); run(40ms);
  command_pub->publish(command()); run(20ms);
  EXPECT_EQ(commands.size(), 1u);
}
