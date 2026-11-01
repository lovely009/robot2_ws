#include "agent_bridge_fixture.hpp"
namespace {
class LeaseBridge : public HostBridgeTest {
protected:
  void SetUp() override {auto_authority = false; HostBridgeTest::SetUp(); start(); run(100ms);}
  void ack(uint64_t id, uint32_t sequence=1, bool active=true, unsigned duration=250) {
    status.lease_id = id; status.lease_sequence = sequence; status.lease_active = active;
    status.lease_valid_until_tick_ms = status.snapshot_tick_ms + duration;
    publish_status(); run(10ms);
  }
};
}
TEST_F(LeaseBridge, HeartbeatKeepsItsDeadlineAndDoesNotAuthorizeBeforeMcuAck) {
  const auto deadline = status.snapshot_tick_ms + 250;
  grant(lease_id, 1);
  ASSERT_TRUE(spin([&]{return wire_grants.size()==1;}));
  EXPECT_EQ(wire_grants[0].lease_valid_until_tick_ms, deadline);
  EXPECT_EQ(wire_grants[0].sequence, 0u); // Authority must not become motion evidence.
  command_pub->publish(command()); run(); EXPECT_TRUE(commands.empty());
  ack(lease_id);
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==1;}));
  EXPECT_EQ(commands.back().lease_id, lease_id);
}
TEST_F(LeaseBridge, LegacyOrWrongLeaseAndSessionCannotBeUpgraded) {
  grant(lease_id, 1); ASSERT_TRUE(spin([&]{return wire_grants.size()==1;})); ack(lease_id);
  auto m=command(); m.lease_id=0; command_pub->publish(m); run();
  m=command(); ++m.session_id; command_pub->publish(m); run();
  m=command(); ++m.lease_id; command_pub->publish(m); run();
  EXPECT_TRUE(commands.empty());
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==1;}));
}
TEST_F(LeaseBridge, HeartbeatReplayWrongPartitionAndExpiredDeadlineAreRejected) {
  grant(lease_id,1); ASSERT_TRUE(spin([&]{return wire_grants.size()==1;}));
  grant(lease_id,1); run(); EXPECT_EQ(wire_grants.size(),1u);
  grant(lease_id+1000000,2); run(); EXPECT_EQ(wire_grants.size(),1u);
  ControlLease old; old.session_id=100; old.lease_id=lease_id; old.sequence=2;
  old.active=true; old.valid_until_tick_ms=status.snapshot_tick_ms-1;
  authority_pub->publish(old); run(); EXPECT_EQ(wire_grants.size(),1u);
  grant(lease_id,2); ASSERT_TRUE(spin([&]{return wire_grants.size()==2;}));
}
TEST_F(LeaseBridge, ExpiryAndControlReplayDoNotRefreshMotionPermission) {
  grant(lease_id,1); ASSERT_TRUE(spin([&]{return wire_grants.size()==1;})); ack(lease_id,1,true,60);
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==1;}));
  run(100ms);
  command_pub->publish(command()); run(); EXPECT_EQ(commands.size(),1u);
  // Re-publishing an authority ACK with its old absolute deadline cannot extend it.
  publish_status(); command_pub->publish(command()); run(); EXPECT_EQ(commands.size(),1u);
}
TEST_F(LeaseBridge, RevokeClosesGateAndOnlyNewLeaseCanResume) {
  grant(lease_id,1); ASSERT_TRUE(spin([&]{return wire_grants.size()==1;})); ack(lease_id);
  command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==1;}));
  grant(lease_id,2,false); ASSERT_TRUE(spin([&]{return wire_grants.size()==2;}));
  command_pub->publish(command()); run(); EXPECT_EQ(commands.size(),1u);
  grant(lease_id,3); run(); EXPECT_EQ(wire_grants.size(),2u);
  ++lease_id; grant(lease_id,1); ASSERT_TRUE(spin([&]{return wire_grants.size()==3;}));
  ack(lease_id); command_pub->publish(command()); ASSERT_TRUE(spin([&]{return commands.size()==2;}));
}
