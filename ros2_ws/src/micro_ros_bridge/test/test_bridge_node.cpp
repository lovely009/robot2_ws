#include <gtest/gtest.h>
#include <chrono>
#include <functional>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/codec.hpp"

using namespace std::chrono_literals;
using namespace can_bridge_msgs::msg;
using micro_ros_bridge::BridgeNode;
namespace topic = micro_ros_bridge::contract::topic;

namespace {
struct Session {
  Session() {
    rclcpp::InitOptions options;
    options.set_domain_id(174);
    rclcpp::init(0, nullptr, options);
  }
  ~Session() {rclcpp::shutdown();}
};
bool spin_until(rclcpp::executors::SingleThreadedExecutor & exec,
  const std::function<bool()> & done, std::chrono::milliseconds limit = 2s)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    exec.spin_some();
    if (done()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return done();
}
void spin_for(rclcpp::executors::SingleThreadedExecutor & exec, std::chrono::milliseconds ms)
{spin_until(exec, [] {return false;}, ms);}
MotorStatus status(uint64_t session, uint32_t seq = 1)
{
  MotorStatus m;
  m.sync_time_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  m.schema_version = WireContract::SCHEMA_VERSION;
  m.session_id = session;
  m.sequence = seq;
  return m;
}
MotorTopology topology(uint64_t session)
{
  MotorTopology m;
  m.sync_time_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  m.schema_version = WireContract::SCHEMA_VERSION;
  m.session_id = session;
  return m;
}
MotorCommandArray command()
{
  MotorCommandArray m;
  m.sync_time_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
  m.count = 1;
  MotorCommandTarget t;
  t.motor_id = 0;
  t.position = 0.5;
  m.targets.push_back(t);
  return m;
}
}

TEST(BridgeNode, DeclaresContractTopics)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  const auto names = node->get_topic_names_and_types();
  for (const auto * name : {&topic::kLowCmd, &topic::kLowState, &topic::kMotorStatus,
      &topic::kTopology, &topic::kControlStates, &topic::kEvents, &topic::kDeviceParam,
      &topic::kConfigResult}) {EXPECT_EQ(names.count(*name), 1u) << *name;}
  for (const char * name : {"/motor/command_array", "/motor/motion_states", "/motor/status_array",
      "/motor/topology", "/motor/control_states", "/motor/config_result"})
    {EXPECT_EQ(names.count(name), 1u) << name;}
}

TEST(BridgeNode, StreamMessagesStayWithinCdrBudget)
{
  LowCommand command;
  command.count = 64;
  command.motor_ids.resize(64);
  command.position_q.resize(64);
  command.velocity_q.resize(64);
  command.effort_q.resize(64);
  rclcpp::Serialization<LowCommand> command_serializer;
  rclcpp::SerializedMessage command_bytes;
  command_serializer.serialize_message(&command, &command_bytes);
  EXPECT_EQ(command_bytes.size(), 488u);
  EXPECT_LE(command_bytes.size(), 490u);

  LowState state;
  state.count = 32;
  state.total_count = 32;
  state.chunk_count = 1;
  state.motor_ids.resize(32);
  state.position_q.resize(32);
  state.velocity_q.resize(32);
  state.effort_q.resize(32);
  state.state_age_ms.resize(32);
  state.target_source.resize(32);
  rclcpp::Serialization<LowState> state_serializer;
  rclcpp::SerializedMessage state_bytes;
  state_serializer.serialize_message(&state, &state_bytes);
  EXPECT_LE(state_bytes.size(), 490u);
}

TEST(BridgeNode, MotionRequiresCurrentStatusAndTopologyAndExpires)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto cmd_pub = node->create_publisher<MotorCommandArray>("/motor/command_array", rclcpp::QoS(1).reliable());
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  int received = 0;
  LowCommand got;
  auto sub = node->create_subscription<LowCommand>(topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](LowCommand::SharedPtr m) {got = *m; ++received;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return cmd_pub->get_subscription_count() > 0 &&
    status_pub->get_subscription_count() > 0 && topo_pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  cmd_pub->publish(command());
  spin_for(exec, 80ms);
  EXPECT_EQ(received, 0);
  auto first_status = status(100);
  first_status.command_sequence = 41;
  status_pub->publish(first_status);
  spin_for(exec, 50ms);
  cmd_pub->publish(command());
  spin_for(exec, 80ms);
  EXPECT_EQ(received, 0);
  topo_pub->publish(topology(100));
  spin_for(exec, 30ms);
  auto old_intent = command();
  old_intent.sync_time_ns -= 1000000000ULL;
  cmd_pub->publish(old_intent);
  spin_for(exec, 30ms);
  EXPECT_EQ(received, 0);
  auto refreshed = status(100, 2);
  refreshed.command_sequence = 41;
  status_pub->publish(refreshed);
  topo_pub->publish(topology(100));
  spin_for(exec, 15ms);
  cmd_pub->publish(command());
  ASSERT_TRUE(spin_until(exec, [&] {return received == 1;}));
  EXPECT_EQ(got.session_id, 100u);
  EXPECT_EQ(got.sequence, 42u);
  EXPECT_NE(got.valid_until_tick_ms, 0u);
  EXPECT_EQ(got.position_q[0], 1311);
  auto lagging = status(100, 3);
  lagging.command_sequence = 1;
  status_pub->publish(lagging);
  spin_for(exec, 20ms);
  cmd_pub->publish(command());
  ASSERT_TRUE(spin_until(exec, [&] {return received == 2;}));
  EXPECT_EQ(got.sequence, 43u);
  std::this_thread::sleep_for(310ms);
  cmd_pub->publish(command());
  spin_for(exec, 80ms);
  EXPECT_EQ(received, 2);
}

TEST(BridgeNode, ConfigIdsSeedFromFirmwareAndRejectConflictingReplay)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  auto config_pub = node->create_publisher<SystemCommand>("/motor/system_command",
    rclcpp::QoS(10).reliable());
  std::vector<ConfigRequest> received;
  auto sub = node->create_subscription<ConfigRequest>(topic::kConfig, rclcpp::QoS(10).reliable(),
    [&](ConfigRequest::SharedPtr m) {received.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return status_pub->get_subscription_count() > 0 &&
    topo_pub->get_subscription_count() > 0 && config_pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  auto st = status(300);
  st.config_request_id = 70;
  status_pub->publish(st);
  topo_pub->publish(topology(300));
  spin_for(exec, 40ms);
  SystemCommand request;
  request.command = SystemCommand::COMMAND_REPORT;
  request.motor_id = SystemCommand::MOTOR_ID_NONE;
  config_pub->publish(request);
  ASSERT_TRUE(spin_until(exec, [&] {return received.size() == 1;}));
  EXPECT_EQ(received[0].request_id, 71u);
  EXPECT_EQ(received[0].session_id, 300u);
  request.request_id = 71;
  config_pub->publish(request);
  ASSERT_TRUE(spin_until(exec, [&] {return received.size() == 2;}));
  request.command = SystemCommand::COMMAND_SAVE;
  config_pub->publish(request);
  spin_for(exec, 60ms);
  EXPECT_EQ(received.size(), 2u);
  request.request_id = 70;
  request.command = SystemCommand::COMMAND_REPORT;
  config_pub->publish(request);
  spin_for(exec, 60ms);
  EXPECT_EQ(received.size(), 2u);
}

TEST(BridgeNode, StopPathsPassWhenConnectionUnknown)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto cmd_pub = node->create_publisher<MotorCommandArray>("/motor/command_array", rclcpp::QoS(1).reliable());
  auto policy_pub = node->create_publisher<MotorPolicyArray>("/motor/policy_array", rclcpp::QoS(10).reliable());
  auto traj_pub = node->create_publisher<MotorTrajectory>("/motor/trajectory", rclcpp::QoS(10).reliable());
  int commands = 0, policies = 0, trajectories = 0;
  auto cmd_sub = node->create_subscription<LowCommand>(topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](LowCommand::SharedPtr) {++commands;});
  auto policy_sub = node->create_subscription<MotorPolicyArray>(topic::kPolicy, rclcpp::QoS(10).reliable(),
    [&](MotorPolicyArray::SharedPtr) {++policies;});
  auto traj_sub = node->create_subscription<MotorTrajectory>(topic::kTrajectory, rclcpp::QoS(10).reliable(),
    [&](MotorTrajectory::SharedPtr) {++trajectories;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return cmd_pub->get_subscription_count() > 0 &&
    policy_pub->get_subscription_count() > 0 && traj_pub->get_subscription_count() > 0 &&
    cmd_sub->get_publisher_count() > 0 && policy_sub->get_publisher_count() > 0 &&
    traj_sub->get_publisher_count() > 0;}));
  MotorCommandArray stop;
  MotorPolicyArray disabled;
  disabled.count = 1;
  MotorPolicyTarget target;
  target.motor_id = 0;
  target.enabled = false;
  disabled.targets.push_back(target);
  MotorTrajectory cancel;
  cmd_pub->publish(stop);
  policy_pub->publish(disabled);
  traj_pub->publish(cancel);
  ASSERT_TRUE(spin_until(exec, [&] {return commands == 1 && policies == 1 && trajectories == 1;}));
  disabled.targets[0].enabled = true;
  policy_pub->publish(disabled);
  auto motion = command();
  cmd_pub->publish(motion);
  spin_for(exec, 100ms);
  EXPECT_EQ(commands, 1);
  EXPECT_EQ(policies, 1);
}

TEST(BridgeNode, ConfigResultAndControlSnapshotRequireCurrentSession)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto result_pub = node->create_publisher<ConfigResult>(topic::kConfigResult, rclcpp::QoS(10).reliable());
  auto control_pub = node->create_publisher<MotorControlStateArray>(topic::kControlStates,
    rclcpp::QoS(10).reliable());
  int results = 0, controls = 0;
  auto result_sub = node->create_subscription<ConfigResult>("/motor/config_result", rclcpp::QoS(10).reliable(),
    [&](ConfigResult::SharedPtr) {++results;});
  auto control_sub = node->create_subscription<MotorControlStateArray>("/motor/control_states",
    rclcpp::QoS(10).reliable(), [&](MotorControlStateArray::SharedPtr) {++controls;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return status_pub->get_subscription_count() > 0 &&
    result_pub->get_subscription_count() > 0 && control_pub->get_subscription_count() > 0 &&
    result_sub->get_publisher_count() > 0 && control_sub->get_publisher_count() > 0;}));
  status_pub->publish(status(50));
  spin_for(exec, 30ms);
  ConfigResult result;
  result.session_id = 49;
  result_pub->publish(result);
  MotorControlStateArray control;
  control.schema_version = MotorControlStateArray::SCHEMA_VERSION;
  control.session_id = 49;
  control_pub->publish(control);
  spin_for(exec, 80ms);
  EXPECT_EQ(results, 0);
  EXPECT_EQ(controls, 0);
  result.session_id = 50;
  control.session_id = 50;
  result_pub->publish(result);
  control_pub->publish(control);
  ASSERT_TRUE(spin_until(exec, [&] {return results == 1 && controls == 1;}));
}

TEST(BridgeNode, OldSourceTimestampCannotOpenMotionGate)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  auto cmd_pub = node->create_publisher<MotorCommandArray>("/motor/command_array", rclcpp::QoS(1).reliable());
  int received = 0;
  auto sub = node->create_subscription<LowCommand>(topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](LowCommand::SharedPtr) {++received;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return status_pub->get_subscription_count() > 0 &&
    topo_pub->get_subscription_count() > 0 && cmd_pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  auto stale = status(20);
  stale.sync_time_ns -= 1000000000ULL;
  status_pub->publish(stale);
  topo_pub->publish(topology(20));
  spin_for(exec, 50ms);
  cmd_pub->publish(command());
  spin_for(exec, 80ms);
  EXPECT_EQ(received, 0);
}

TEST(BridgeNode, OldLowStateCannotPublishAsFreshMotion)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  auto low_pub = node->create_publisher<LowState>(topic::kLowState, rclcpp::QoS(1).best_effort());
  int received = 0;
  auto sub = node->create_subscription<MotorMotionStateArray>("/motor/motion_states",
    rclcpp::QoS(1).best_effort(), [&](MotorMotionStateArray::SharedPtr) {++received;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return status_pub->get_subscription_count() > 0 &&
    topo_pub->get_subscription_count() > 0 && low_pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  status_pub->publish(status(30));
  topo_pub->publish(topology(30));
  spin_for(exec, 40ms);
  LowState low;
  low.schema_version = WireContract::SCHEMA_VERSION;
  low.session_id = 30;
  low.seq = 1;
  low.chunk_count = 1;
  low.total_count = 1;
  low.count = 1;
  low.motor_ids = {1};
  low.position_q = {100};
  low.velocity_q = {0};
  low.effort_q = {0};
  low.state_age_ms = {0};
  low.target_source = {0};
  low.sync_time_ns = status(30).sync_time_ns - 1000000000ULL;
  low_pub->publish(low);
  spin_for(exec, 60ms);
  EXPECT_EQ(received, 0);
  low.sync_time_ns = status(30).sync_time_ns;
  low_pub->publish(low);
  ASSERT_TRUE(spin_until(exec, [&] {return received == 1;}));
}

TEST(BridgeNode, BurstTwoLowStateChunksSurviveReaderHistory)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  auto low_pub = node->create_publisher<LowState>(topic::kLowState, rclcpp::QoS(10).best_effort());
  int received = 0;
  size_t count = 0;
  auto sub = node->create_subscription<MotorMotionStateArray>("/motor/motion_states",
    rclcpp::QoS(10).best_effort(), [&](MotorMotionStateArray::SharedPtr m) {
      ++received;
      count = m->states.size();
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return status_pub->get_subscription_count() > 0 &&
    topo_pub->get_subscription_count() > 0 && low_pub->get_subscription_count() > 0 &&
    sub->get_publisher_count() > 0;}));
  status_pub->publish(status(600));
  topo_pub->publish(topology(600));
  spin_for(exec, 30ms);
  const auto stamp = status(600).sync_time_ns;
  LowState first;
  first.schema_version = WireContract::SCHEMA_VERSION;
  first.session_id = 600;
  first.seq = 1;
  first.tick_ms = 1;
  first.sync_time_ns = stamp;
  first.chunk_count = 2;
  first.total_count = 33;
  first.count = 32;
  for (uint8_t id = 0; id < 32; ++id) {
    first.motor_ids.push_back(id);
    first.position_q.push_back(0);
    first.velocity_q.push_back(0);
    first.effort_q.push_back(0);
    first.state_age_ms.push_back(0);
    first.target_source.push_back(0);
  }
  LowState second = first;
  second.chunk_index = 1;
  second.count = 1;
  second.motor_ids = {32};
  second.position_q = {0};
  second.velocity_q = {0};
  second.effort_q = {0};
  second.state_age_ms = {0};
  second.target_source = {0};
  low_pub->publish(first);
  low_pub->publish(second);
  std::this_thread::sleep_for(20ms);  // 两片均先交付 DDS，executor 此时尚未取消息。
  spin_for(exec, 30ms);
  EXPECT_EQ(received, 1);
  EXPECT_EQ(count, 33u);
}

TEST(BridgeNode, ExecuteConfigWaitsForTerminalResultAndTimesOut)
{
  Session session;
  auto node = std::make_shared<BridgeNode>();
  using Service = can_bridge_msgs::srv::ExecuteConfig;
  auto client = node->create_client<Service>("/motor/execute_config");
  auto status_pub = node->create_publisher<MotorStatus>(topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto topo_pub = node->create_publisher<MotorTopology>(topic::kTopology, rclcpp::QoS(10).reliable());
  auto result_pub = node->create_publisher<ConfigResult>(topic::kConfigResult, rclcpp::QoS(10).reliable());
  std::vector<ConfigRequest> sent;
  auto wire_sub = node->create_subscription<ConfigRequest>(topic::kConfig, rclcpp::QoS(10).reliable(),
    [&](ConfigRequest::SharedPtr m) {sent.push_back(*m);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(spin_until(exec, [&] {return client->service_is_ready() &&
    status_pub->get_subscription_count() > 0 && topo_pub->get_subscription_count() > 0 &&
    result_pub->get_subscription_count() > 0 && wire_sub->get_publisher_count() > 0;}));
  auto st = status(500);
  st.config_request_id = 9;
  status_pub->publish(st);
  topo_pub->publish(topology(500));
  spin_for(exec, 30ms);
  auto mixed = std::make_shared<Service::Request>();
  mixed->request.kind = ConfigRequest::CONFIG_KIND_PARAM_SET;
  mixed->request.command = SystemCommand::COMMAND_SAVE;
  mixed->request.motor_id = 0;  // PARAM_SET 不得携带 SYSTEM_COMMAND 目标
  auto rejected = client->async_send_request(mixed);
  ASSERT_TRUE(spin_until(exec, [&] {return rejected.wait_for(0ms) == std::future_status::ready;}));
  EXPECT_EQ(rejected.get()->result.status, ConfigResult::STATUS_FAILED);
  EXPECT_TRUE(sent.empty());
  auto request = std::make_shared<Service::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REPORT;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  request->timeout_ms = 200;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(exec, [&] {return sent.size() == 1;}));
  EXPECT_EQ(sent[0].request_id, 10u);
  ConfigResult pending;
  pending.session_id = 500;
  pending.request_id = sent[0].request_id;
  pending.kind = sent[0].kind;
  pending.command = sent[0].command;
  pending.status = ConfigResult::STATUS_PENDING;
  result_pub->publish(pending);
  spin_for(exec, 30ms);
  EXPECT_NE(future.wait_for(0ms), std::future_status::ready);
  pending.status = ConfigResult::STATUS_FAILED;
  pending.error_code = 7;
  result_pub->publish(pending);
  ASSERT_TRUE(spin_until(exec, [&] {return future.wait_for(0ms) == std::future_status::ready;}));
  const auto failed_response = future.get();
  EXPECT_EQ(failed_response->result.status, ConfigResult::STATUS_FAILED);
  EXPECT_EQ(failed_response->result.error_code, 7u);
  EXPECT_EQ(failed_response->result.request_id, 10u);

  request = std::make_shared<Service::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REPORT;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  request->timeout_ms = 100;
  auto timed = client->async_send_request(request);
  ASSERT_TRUE(spin_until(exec, [&] {return sent.size() == 2;}));
  ASSERT_TRUE(spin_until(exec, [&] {return timed.wait_for(0ms) == std::future_status::ready;}));
  const auto timeout_response = timed.get();
  EXPECT_EQ(timeout_response->result.status, ConfigResult::STATUS_FAILED);
  EXPECT_EQ(timeout_response->result.error_code, 21u);
  status_pub->publish(status(500, 2));
  topo_pub->publish(topology(500));
  spin_for(exec, 20ms);
  request->timeout_ms = 200;
  auto succeeded = client->async_send_request(request);
  ASSERT_TRUE(spin_until(exec, [&] {return sent.size() == 3;}));
  pending.request_id = sent[2].request_id;
  pending.status = ConfigResult::STATUS_SUCCEEDED;
  pending.error_code = 0;
  result_pub->publish(pending);
  ASSERT_TRUE(spin_until(exec, [&] {return succeeded.wait_for(0ms) == std::future_status::ready;}));
  const auto success_response = succeeded.get();
  EXPECT_EQ(success_response->result.status, ConfigResult::STATUS_SUCCEEDED);
  EXPECT_EQ(success_response->result.request_id, sent[2].request_id);
}
