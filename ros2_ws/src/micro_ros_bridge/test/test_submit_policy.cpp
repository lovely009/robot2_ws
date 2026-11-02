#include "agent_bridge_fixture.hpp"
#include <can_bridge_msgs/srv/submit_policy.hpp>
#include <limits>

namespace {
using SubmitPolicy = can_bridge_msgs::srv::SubmitPolicy;
class PolicyServiceTest : public BridgeTest {
protected:
  rclcpp::Client<SubmitPolicy>::SharedPtr first, second;
  void SetUp() override {
    BridgeTest::SetUp();
    first = peer->create_client<SubmitPolicy>("/motor/submit_policy");
    second = peer->create_client<SubmitPolicy>("/motor/submit_policy");
  }
  bool service_ready() {return spin([&] {return first->service_is_ready();}, 300ms);}
  SubmitPolicy::Request::SharedPtr request(bool enabled = true) {
    auto out = std::make_shared<SubmitPolicy::Request>();
    out->session_id = status.session_id; out->source_time_ns = status.snapshot_stamp;
    MotorPolicyTarget target; target.motor_id = 64; target.enabled = enabled;
    target.kp = 20; target.kd = 2; out->targets = {target}; return out;
  }
  SubmitPolicy::Response::SharedPtr submit(const SubmitPolicy::Request::SharedPtr & req) {
    auto future = first->async_send_request(req);
    const bool done = spin([&] {return future.wait_for(0s) == std::future_status::ready;});
    EXPECT_TRUE(done); return done ? future.get() : nullptr;
  }
  void stop() {
    auto publisher = peer->create_publisher<std_msgs::msg::Empty>("/control/stop", 10);
    ASSERT_TRUE(spin([&] {return publisher->get_subscription_count() != 0;}));
    configs.clear(); publisher->publish(std_msgs::msg::Empty{});
    ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
      return c.operation == AgentConfigRequest::STOP_ALL;});}));
  }
  bool terminal(const SubmitPolicy::Response & token, uint8_t state) {
    return std::any_of(results.begin(), results.end(), [&](const auto & result) {
      return result.request_id == token.request_id && result.intent_id == token.intent_id &&
             result.status == state;
    });
  }
  AgentConfigRequest block_config(uint16_t motor = 64) {
    SystemCommand command; command.command = SystemCommand::COMMAND_REFRESH;
    command.motor_id = motor; command.request_id = 100; command.sync_time_ns = stamp();
    system_command_pub->publish(command);
    const bool sent = spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
      return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});});
    EXPECT_TRUE(sent);
    return sent ? *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
      return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;}) : AgentConfigRequest{};
  }
  void acknowledge(uint32_t id, uint8_t operation = AgentConfigRequest::SET_POLICY) {
    AgentConfigResult ack; ack.request_id = id; ack.operation = operation;
    ack.state = AgentConfigResult::SUCCEEDED; ack.completed_motor_mask = UINT64_C(1) << 63;
    status.results = {ack}; publish_status();
  }
};

TEST_F(PolicyServiceTest, IndependentClientsWithSameSourceGetUniqueHostIdentities)
{
  start(); ASSERT_TRUE(service_ready());
  auto req = request();
  auto a = first->async_send_request(req), b = second->async_send_request(req);
  ASSERT_TRUE(spin([&] {return a.wait_for(0s) == std::future_status::ready &&
    b.wait_for(0s) == std::future_status::ready;}));
  const auto initial_a = a.get(), initial_b = b.get();
  ASSERT_NE(initial_a->accepted, initial_b->accepted);
  const auto one = initial_a->accepted ? initial_a : initial_b;
  const auto busy = initial_a->accepted ? initial_b : initial_a;
  EXPECT_FALSE(busy->accepted); EXPECT_NE(busy->reason.find("忙"), std::string::npos);
  acknowledge(one->request_id);
  ASSERT_TRUE(spin([&] {return terminal(*one, ConfigResult::STATUS_SUCCEEDED);}));
  auto next = second->async_send_request(req);
  ASSERT_TRUE(spin([&] {return next.wait_for(0s) == std::future_status::ready;}));
  const auto two = next.get(); ASSERT_TRUE(two->accepted) << two->reason;
  EXPECT_EQ(one->session_id, status.session_id); EXPECT_FALSE(one->intent_id.empty());
  EXPECT_NE(one->intent_id, two->intent_id); EXPECT_NE(one->request_id, two->request_id);
  EXPECT_GT(std::min(one->policy_generation, two->policy_generation), 9u);
  EXPECT_EQ(std::max(one->policy_generation, two->policy_generation),
    std::min(one->policy_generation, two->policy_generation) + 1u);
  EXPECT_EQ(std::count_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}), 2);
  const auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;});
  EXPECT_EQ(wire.issued_stamp, req->source_time_ns); EXPECT_EQ(wire.ttl_ms, 100);
}

TEST_F(PolicyServiceTest, AdvancesPastMotorAndStopBarrierAndMapsNativeAckToIntent)
{
  start(); ASSERT_TRUE(service_ready());
  status.required_policy_generation = 50; status.motors[0].policy_generation = 70;
  publish_status(); run();
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted) << token->reason;
  EXPECT_EQ(token->policy_generation, 71u);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == token->request_id;
  });}));
  const auto wire = *std::find_if(configs.begin(), configs.end(), [&](const auto & c) {
    return c.request_id == token->request_id && c.operation == AgentConfigRequest::SET_POLICY;});
  EXPECT_EQ(wire.policy_generation, token->policy_generation);
  AgentConfigResult ack; ack.request_id = token->request_id;
  ack.operation = AgentConfigRequest::SET_POLICY; ack.state = AgentConfigResult::SUCCEEDED;
  ack.completed_motor_mask = UINT64_C(1) << 63; status.results = {ack};
  ASSERT_TRUE(spin([&] {return terminal(*token, ConfigResult::STATUS_SUCCEEDED);}));
  const auto result = *std::find_if(results.begin(), results.end(), [&](const auto & r) {
    return r.intent_id == token->intent_id;});
  EXPECT_EQ(result.session_id, token->session_id); EXPECT_EQ(result.kind, 2);
  EXPECT_EQ(result.command, AgentConfigRequest::SET_POLICY); EXPECT_EQ(result.completed_targets, 1);
}

TEST_F(PolicyServiceTest, RejectsOldFutureSessionAndExpiredSourceWithoutPublishing)
{
  start(); ASSERT_TRUE(service_ready());
  for (const auto session : {status.session_id - 1, status.session_id + 1}) {
    auto req = request(); req->session_id = session;
    const auto response = submit(req); ASSERT_TRUE(response); EXPECT_FALSE(response->accepted);
  }
  auto req = request(); req->source_time_ns = stamp() - 101000000ULL;
  const auto response = submit(req); ASSERT_TRUE(response); EXPECT_FALSE(response->accepted);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}));
}

TEST_F(PolicyServiceTest, StopRejectsAQueuedOldSourceAndDoesNotAutomaticallyReenable)
{
  start(); ASSERT_TRUE(service_ready()); auto old = request(); stop();
  const auto response = submit(old); ASSERT_TRUE(response); EXPECT_FALSE(response->accepted);
  run(150ms);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}));
}

TEST_F(PolicyServiceTest, QueuedIntentExpiresWithOriginalSourceAndTerminalIdentity)
{
  start(); ASSERT_TRUE(service_ready());
  block_config();
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted);
  ASSERT_TRUE(spin([&] {return terminal(*token, ConfigResult::STATUS_FAILED);}, 250ms));
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == token->request_id;}));
  run(100ms);
  EXPECT_EQ(std::count_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}), 0);
}

TEST_F(PolicyServiceTest, WaitingForLeaseRejectsSecondIntentAndStopsWithTerminalReceipt)
{
  start(); ASSERT_TRUE(service_ready()); stop(); run(60ms); auto_authority = false;
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted) << token->reason;
  const auto other = submit(request()); ASSERT_TRUE(other); EXPECT_FALSE(other->accepted);
  ASSERT_TRUE(spin([&] {return terminal(*token, ConfigResult::STATUS_FAILED);}, 250ms));
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}));
}

TEST_F(PolicyServiceTest, StopAndSessionEndRetireAcceptedIntents)
{
  start(); ASSERT_TRUE(service_ready());
  const auto sent = submit(request()); ASSERT_TRUE(sent); ASSERT_TRUE(sent->accepted);
  stop();
  ASSERT_TRUE(spin([&] {return terminal(*sent, ConfigResult::STATUS_FAILED);}));
  run(60ms);
  const auto next = submit(request()); ASSERT_TRUE(next); ASSERT_TRUE(next->accepted) << next->reason;
  ++status.session_id; status.results.clear(); publish_status();
  ASSERT_TRUE(spin([&] {return terminal(*next, ConfigResult::STATUS_FAILED);}));
}

TEST_F(PolicyServiceTest, ExhaustedGenerationOrWireIdsStillAllowDisable)
{
  start(); ASSERT_TRUE(service_ready());
  status.required_policy_generation = UINT32_MAX; publish_status(); run();
  const auto denied = submit(request()); ASSERT_TRUE(denied); EXPECT_FALSE(denied->accepted);
  status.config_request_high_water = AgentWireContract::REQUEST_ID_MAX; publish_status(); run();
  auto off = request(false); off->session_id = 0; off->source_time_ns = 0;
  off->targets[0].confirm_unprotected = true;
  const auto response = submit(off); ASSERT_TRUE(response); ASSERT_TRUE(response->accepted);
  EXPECT_EQ(response->policy_generation, 0u); EXPECT_EQ(response->request_id, 0u);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && !c.policies[0].enabled;});}));
  const auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && !c.policies[0].enabled;});
  EXPECT_EQ(wire.policies[0].kp, 0); EXPECT_EQ(wire.policies[0].kd, 0);
  EXPECT_FALSE(wire.policies[0].confirm_unprotected);
}

TEST_F(PolicyServiceTest, RejectsMalformedTargetsAndNativeRequestIdExhaustion)
{
  start(); ASSERT_TRUE(service_ready());
  auto empty = request(); empty->targets.clear();
  ASSERT_FALSE(submit(empty)->accepted);
  auto invalid = request(); invalid->targets[0].kp = std::numeric_limits<float>::quiet_NaN();
  ASSERT_FALSE(submit(invalid)->accepted);
  status.config_request_high_water = AgentWireContract::REQUEST_ID_MAX; publish_status(); run();
  const auto response = submit(request()); ASSERT_TRUE(response); EXPECT_FALSE(response->accepted);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY;}));
}

TEST_F(PolicyServiceTest, PendingNativeIdBlocksLaterOrdinaryWritesButAllowsQueries)
{
  start(); ASSERT_TRUE(service_ready()); stop(); run(60ms); auto_authority = false;
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted);
  SystemCommand command; command.command = SystemCommand::COMMAND_REFRESH;
  command.motor_id = 64; command.request_id = 101; command.sync_time_ns = stamp();
  system_command_pub->publish(command);
  ASSERT_TRUE(spin([&] {return std::any_of(results.begin(), results.end(), [](const auto & r) {
    return r.request_id == 101 && r.status == ConfigResult::STATUS_BUSY;});}));
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;}));
  const auto before = topologies.size();
  command.command = SystemCommand::COMMAND_REPORT; command.motor_id = SystemCommand::MOTOR_ID_NONE;
  command.request_id = 102; command.sync_time_ns = stamp(); system_command_pub->publish(command);
  ASSERT_TRUE(spin([&] {return topologies.size() > before;}));
}

TEST_F(PolicyServiceTest, DisableCancelsAnOverlappingQueuedPolicyBeforeItCanSend)
{
  start(); ASSERT_TRUE(service_ready()); const auto blocker = block_config();
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted);
  const auto off = submit(request(false)); ASSERT_TRUE(off); ASSERT_TRUE(off->accepted);
  ASSERT_TRUE(spin([&] {return terminal(*token, ConfigResult::STATUS_FAILED);}));
  acknowledge(blocker.request_id, blocker.operation); run(60ms);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == token->request_id;}));
}

TEST_F(PolicyServiceTest, SparseDisablePreservesADifferentMotorQueuedIntent)
{
  start(2); ASSERT_TRUE(service_ready()); const auto blocker = block_config(63);
  auto enabling = request(); enabling->targets[0].motor_id = 63;
  const auto token = submit(enabling); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted);
  const auto off = submit(request(false)); ASSERT_TRUE(off); ASSERT_TRUE(off->accepted);
  EXPECT_FALSE(terminal(*token, ConfigResult::STATUS_FAILED));
  acknowledge(blocker.request_id, blocker.operation);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == token->request_id;
  });}));
}

TEST_F(PolicyServiceTest, BridgeRestartFencesAnUnsentNativeRequestIdReuse)
{
  start(); ASSERT_TRUE(service_ready());
  status.config_request_high_water = 1; publish_status(); run();
  stop(); run(60ms); auto_authority = false;
  const auto old = submit(request()); ASSERT_TRUE(old); ASSERT_TRUE(old->accepted);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == old->request_id;}));
  exec.remove_node(bridge); bridge.reset(); auto_authority = true;
  const auto previous = topologies.size();
  bridge = std::make_shared<micro_ros_bridge::BridgeNode>(); exec.add_node(bridge);
  ASSERT_TRUE(spin([&] {return topologies.size() > previous;})); run(60ms);
  const auto current = submit(request()); ASSERT_TRUE(current); ASSERT_TRUE(current->accepted) << current->reason;
  EXPECT_EQ(old->request_id, current->request_id); EXPECT_NE(old->intent_id, current->intent_id);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY && c.request_id == current->request_id;
  });}));
  acknowledge(current->request_id);
  ASSERT_TRUE(spin([&] {return terminal(*current, ConfigResult::STATUS_SUCCEEDED);}));
  EXPECT_FALSE(terminal(*old, ConfigResult::STATUS_SUCCEEDED));
}

TEST_F(PolicyServiceTest, LegacyCallerIdDoesNotAliasACompletedNativeIntent)
{
  start(); ASSERT_TRUE(service_ready());
  const auto token = submit(request()); ASSERT_TRUE(token); ASSERT_TRUE(token->accepted);
  acknowledge(token->request_id);
  ASSERT_TRUE(spin([&] {return terminal(*token, ConfigResult::STATUS_SUCCEEDED);}));
  MotorPolicyArray legacy; legacy.request_id = token->request_id;
  legacy.policy_generation = token->policy_generation + 1; legacy.sync_time_ns = stamp();
  legacy.count = 1; legacy.targets.push_back(request()->targets.front()); policy_pub->publish(legacy);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::SET_POLICY &&
      c.policy_generation == legacy.policy_generation && c.request_id != token->request_id;
  });}));
}
}  // namespace
