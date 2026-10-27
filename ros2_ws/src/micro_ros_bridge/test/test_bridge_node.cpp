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
  rclcpp::TimerBase::SharedPtr status_timer;
  AgentStatus status;
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
    status.snapshot_tick_ms += 40; ++status.sequence; status_pub->publish(status);
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
  }
  MotorCommandArray command() {
    MotorCommandArray m; m.sync_time_ns = stamp(); m.count = 1;
    MotorCommandTarget t; t.motor_id = 64; t.position = 0.5f; m.targets.push_back(t); return m;
  }
  void SetUp() override {
    bridge = std::make_shared<micro_ros_bridge::BridgeNode>();
    peer = std::make_shared<rclcpp::Node>("fake_agent");
    exec.add_node(bridge); exec.add_node(peer);
    status_pub = peer->create_publisher<AgentStatus>(AgentWireContract::TOPIC_STATUS, 10);
    info_pub = peer->create_publisher<AgentInfo>(AgentWireContract::TOPIC_INFO, 10);
    motion_pub = peer->create_publisher<AgentMotion>(AgentWireContract::TOPIC_MOTION,
      rclcpp::QoS(2).best_effort());
    command_pub = peer->create_publisher<MotorCommandArray>("/motor/command_array", 1);
    policy_pub = peer->create_publisher<MotorPolicyArray>("/motor/policy_array", 10);
    trajectory_pub = peer->create_publisher<MotorTrajectory>("/motor/trajectory", 10);
    system_command_pub = peer->create_publisher<SystemCommand>("/motor/system_command", 10);
    listen(AgentWireContract::TOPIC_COMMAND, commands, rclcpp::QoS(10).best_effort());
    listen(AgentWireContract::TOPIC_TRAJECTORY, trajectories);
    listen("/motor/motion_states", motions, rclcpp::QoS(10).best_effort());
    listen("/motor/config_result", results); listen("/motor/control_states", controls);
    listen("/motor/topology", topologies); listen("/motor/device_param_report", params);
    listen("/motor/trajectory_state", trajectory_states);
    subscriptions.push_back(peer->create_subscription<AgentConfigRequest>(
      AgentWireContract::TOPIC_CONFIG, 10, [this](AgentConfigRequest::SharedPtr m) {
        configs.push_back(*m);
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
}
TEST_F(BridgeTest, RegistersOnlyEightV3WireTopics)
{
  const auto names = bridge->get_topic_names_and_types(); size_t wire = 0;
  for (const auto & p : names) {
    if (p.first.rfind("/micro_ros_", 0) == 0) {++wire; EXPECT_EQ(p.second.size(), 1u);}
  }
  EXPECT_EQ(wire, 8u);
  EXPECT_EQ(names.count("/micro_ros_do/low_cmd"), 0u);
  EXPECT_EQ(names.at(AgentWireContract::TOPIC_CONFIG)[0], "can_bridge_msgs/msg/AgentConfigRequest");
}
TEST_F(BridgeTest, MotionGateNeedsStatusInfoSingleWriterAndFreshSource)
{
  command_pub->publish(command()); run(); EXPECT_TRUE(commands.empty());
  start();
  auto old = command(); old.sync_time_ns -= 1000000000ULL;
  command_pub->publish(old); run(); EXPECT_TRUE(commands.empty());
  command_pub->publish(command()); ASSERT_TRUE(spin([&] {return commands.size() == 1;}));
  EXPECT_EQ(commands.back().sequence, 42u); EXPECT_EQ(commands.back().session_id, 100u);
  EXPECT_EQ(commands.back().motor_ids[0], 64); EXPECT_EQ(commands.back().position_q[0], 1311);
  EXPECT_EQ(commands.back().mode, AgentCommand::SNAPSHOT);
  auto second = peer->create_publisher<MotorCommandArray>("/motor/command_array", 1);
  run(60ms); command_pub->publish(command()); run(); EXPECT_EQ(commands.size(), 1u);
  second.reset(); run();
  status_timer.reset(); run(320ms);
  command_pub->publish(command()); run(); EXPECT_EQ(commands.size(), 1u);
}
TEST_F(BridgeTest, EmptySnapshotRenewsLeaseWhileExplicitCancelStopsWithoutSession)
{
  MotorTrajectory cancel; trajectory_pub->publish(cancel);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::STOP_ALL;});}));
  start(); MotorCommandArray heartbeat; heartbeat.sync_time_ns = stamp();
  command_pub->publish(heartbeat); ASSERT_TRUE(spin([&] {return !commands.empty();}));
  EXPECT_EQ(commands.back().mode, AgentCommand::SNAPSHOT); EXPECT_EQ(commands.back().count, 0);
  EXPECT_NE(commands.back().valid_until_tick_ms, 0u);
}
TEST_F(BridgeTest, DisableBypassesClockButEnableDoesNot)
{
  MotorPolicyArray policy; policy.count = 1; policy.policy_generation = 9;
  MotorPolicyTarget p; p.motor_id = 64; p.kp = 20; p.kd = 2; p.confirm_unprotected = true;
  policy.targets.push_back(p); policy_pub->publish(policy);
  ASSERT_TRUE(spin([&] {return !configs.empty();}));
  EXPECT_EQ(configs.back().operation, AgentConfigRequest::SET_POLICY);
  EXPECT_FLOAT_EQ(configs.back().policies[0].kp, 0);
  EXPECT_FALSE(configs.back().policies[0].confirm_unprotected);
  configs.clear(); policy.targets[0].enabled = true; policy.sync_time_ns = stamp();
  policy_pub->publish(policy); run(); EXPECT_TRUE(configs.empty());
}
TEST_F(BridgeTest, InfoPagesPublishOneCompleteRevision)
{
  start(48); ASSERT_EQ(topologies.size(), 1u); ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(topologies.back().entries.size(), 48u); EXPECT_EQ(params.back().count, 48u);
  std::vector<uint16_t> cursors;
  for (const auto & c : configs) if (c.operation == AgentConfigRequest::QUERY_INFO) {
    cursors.push_back(c.cursor);
  }
  EXPECT_EQ(cursors, (std::vector<uint16_t>{0,16,32}));
  run(300ms); EXPECT_EQ(topologies.size(), 1u); // Info is on demand, not a 1.5-second lease.
  status.info_revision = 2; ASSERT_TRUE(spin([&] {return topologies.size() == 2;}));
}
TEST_F(BridgeTest, InvalidClockStatusIsVisibleButCannotAuthorizeMotion)
{
  start(); const auto before = controls.size();
  status.clock_valid = false;
  ASSERT_TRUE(spin([&] {return controls.size() > before;}));
  command_pub->publish(command()); run(); EXPECT_TRUE(commands.empty());
}
TEST_F(BridgeTest, ConfigWaitsForReceiptThenAcknowledgesTerminalAndRejectsConflict)
{
  start(); status.config_request_high_water = 50; publish_status(); run();
  SystemCommand cmd; cmd.sync_time_ns = stamp(); cmd.command = SystemCommand::COMMAND_SET_ZERO;
  cmd.motor_id = 64; cmd.request_id = 7; system_command_pub->publish(cmd);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::SET_ZERO;});}));
  auto wire = *std::find_if(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::SET_ZERO;});
  EXPECT_EQ(wire.request_id, 51u);
  auto next = cmd; next.request_id = 8; next.sync_time_ns = stamp(); system_command_pub->publish(next);
  run(); EXPECT_EQ(std::count_if(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::SET_ZERO;}), 1);
  AgentConfigResult result; result.request_id = 51; result.operation = AgentConfigRequest::SET_ZERO;
  result.state = AgentConfigResult::PENDING; status.results = {result};
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(),configs.end(),[](const auto & c) {
    return c.request_id == 52;});}));
  status.results[0].state = AgentConfigResult::SUCCEEDED;
  status.results[0].completed_motor_mask = UINT64_C(1) << 63;
  ASSERT_TRUE(spin([&] {return !results.empty();}));
  EXPECT_EQ(results.back().request_id, 7u); EXPECT_EQ(results.back().status, ConfigResult::STATUS_SUCCEEDED);
  EXPECT_EQ(results.back().completed_targets, 1);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::ACK_RESULTS && c.ack_request_ids[0] == 51;});}));
  cmd.motor_id = 63; cmd.sync_time_ns = stamp(); system_command_pub->publish(cmd);
  ASSERT_TRUE(spin([&] {return results.back().status == ConfigResult::STATUS_CONFLICT;}));
}
TEST_F(BridgeTest, ExecuteConfigReturnsOnlyAfterTerminalAndExpires)
{
  start(); auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/motor/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_CLEAR_FAULT;
  request->request.motor_id = 64; request->request.sync_time_ns = stamp(); request->timeout_ms = 100;
  auto future = client->async_send_request(request);
  run(40ms); EXPECT_NE(future.wait_for(0ms), std::future_status::ready);
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  auto timed_out = future.get()->result;
  EXPECT_EQ(timed_out.error_code, 21);
  EXPECT_EQ(timed_out.status, ConfigResult::STATUS_PENDING);
}
TEST_F(BridgeTest, BurstMotionGroupsAssembleAndRejectWrongSessionAndStaleTime)
{
  start(48); AgentMotion motion; motion.session_id = 100; motion.snapshot_stamp = stamp();
  motion.snapshot_tick_ms = 10; motion.sequence = 1; motion.group_count = 2;
  for (size_t group = 0; group < 2; ++group) {
    motion.group_index = static_cast<uint8_t>(group); motion.count = group == 0 ? 32 : 16;
    motion.motor_ids.clear(); motion.position_q.clear(); motion.velocity_q.clear();
    motion.effort_q.clear(); motion.state_age_ms.clear(); motion.target_source.clear();
    for (size_t i = 0; i < motion.count; ++i) {
      motion.motor_ids.push_back(static_cast<uint8_t>(17 + group*32+i));
      motion.position_q.push_back(0); motion.velocity_q.push_back(0); motion.effort_q.push_back(0);
      motion.state_age_ms.push_back(5); motion.target_source.push_back(0);
    }
    motion_pub->publish(motion);
  }
  ASSERT_TRUE(spin([&] {return !motions.empty();}));
  EXPECT_EQ(motions.back().states.size(), 48u); EXPECT_EQ(motions.back().states.back().motor_id, 64);
  motion.sequence = 2; motion.session_id = 99; motion_pub->publish(motion); run();
  EXPECT_EQ(motions.size(), 1u);
  motion.session_id = 100; motion.snapshot_stamp -= 1000000000ULL; motion_pub->publish(motion); run();
  EXPECT_EQ(motions.size(), 1u);
}
TEST_F(BridgeTest, TrajectorySplitsLoadsThenCommitsAfterReceipts)
{
  start(); MotorTrajectory trajectory; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 77;
  trajectory.segment_count = 1; trajectory.target_count = 1; trajectory.target_ids = {64};
  for (size_t i = 0; i < 33; ++i) {
    MotorTrajectoryPoint p; p.motor_id = 64; p.time_from_start_ms = static_cast<uint32_t>(i * 10);
    p.position = static_cast<float>(i)*0.01f; trajectory.points.push_back(p);
  }
  trajectory.count = 33; trajectory_pub->publish(trajectory);
  ASSERT_TRUE(spin([&] {return !trajectories.empty();}));
  auto first = trajectories.back(); EXPECT_EQ(first.operation, AgentTrajectoryRequest::LOAD);
  EXPECT_EQ(first.points.size(), 32u); EXPECT_EQ(first.segment_count, 2);
  status.trajectory.trajectory_id = first.trajectory_id;
  status.trajectory.state = AgentTrajectoryState::LOADING;
  status.trajectory.target_ids = {64};
  status.trajectory_request_receipt.request_id = first.request_id;
  status.trajectory_request_receipt.trajectory_id = first.trajectory_id;
  status.trajectory_request_receipt.operation = AgentTrajectoryRequest::LOAD;
  status.trajectory_request_receipt.accepted = true; status.trajectory_request_receipt.received_segments = 1;
  ASSERT_TRUE(spin([&] {return std::any_of(trajectories.begin(),trajectories.end(),[](const auto & r) {
    return r.operation == AgentTrajectoryRequest::LOAD && r.segment_index == 1;});}));
  auto second = *std::find_if(trajectories.begin(),trajectories.end(),[](const auto & r) {
    return r.operation == AgentTrajectoryRequest::LOAD && r.segment_index == 1;});
  EXPECT_EQ(second.points.size(), 1u);
  status.trajectory.state = AgentTrajectoryState::READY;
  status.trajectory_request_receipt.request_id = second.request_id;
  status.trajectory_request_receipt.received_segments = 2;
  ASSERT_TRUE(spin([&] {return std::any_of(trajectories.begin(),trajectories.end(),[](const auto & r) {
    return r.operation == AgentTrajectoryRequest::COMMIT;});}));
  auto commit = *std::find_if(trajectories.begin(),trajectories.end(),[](const auto & r) {
    return r.operation == AgentTrajectoryRequest::COMMIT;});
  status.trajectory_request_receipt.request_id = commit.request_id;
  status.trajectory_request_receipt.operation = AgentTrajectoryRequest::COMMIT;
  status.trajectory.state = AgentTrajectoryState::RUNNING;
  ASSERT_TRUE(spin([&] {return std::any_of(trajectories.begin(),trajectories.end(),[&](const auto & r) {
    return r.operation == AgentTrajectoryRequest::ACK && r.ack_request_id == commit.request_id;});}));
  EXPECT_EQ(trajectory_states.back().trajectory_id, 77u);
}
TEST_F(BridgeTest, SessionChangeFailsPendingServiceAndRestartsInfo)
{
  start(); auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/motor/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_CLEAR_FAULT; request->request.motor_id = 64;
  request->request.sync_time_ns = stamp(); auto future = client->async_send_request(request);
  run(); status.session_id = 101;
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  EXPECT_EQ(future.get()->result.error_code, 21);
  ASSERT_TRUE(spin([&] {return topologies.back().session_id == 101;}));
}
TEST_F(BridgeTest, RevisionRetryWithZeroCursorRestartsWithoutPublishingMixedPages)
{
  start(48); auto_info = false; configs.clear();
  status.info_revision = 2;
  ASSERT_TRUE(spin([&] {return !configs.empty();}));
  auto query = configs.back();
  auto first = page(query); info_pub->publish(first);
  ASSERT_TRUE(spin([&] {return configs.back().cursor == 16;}));
  query = configs.back();
  AgentInfo retry; retry.session_id = 100; retry.query_id = query.query_id;
  retry.result = AgentInfo::RETRY; retry.cursor = 0; retry.revision = 3;
  info_pub->publish(retry);
  ASSERT_TRUE(spin([&] {return configs.back().query_id != query.query_id;}));
  EXPECT_EQ(configs.back().cursor, 0); EXPECT_EQ(topologies.size(), 1u);
  command_pub->publish(command()); ASSERT_TRUE(spin([&] {return !commands.empty();}));
}
TEST_F(BridgeTest, LoadingDoesNotLookCanceledAndOldSetpointCannotReplayAfterTrajectory)
{
  start(); command_pub->publish(command()); ASSERT_TRUE(spin([&] {return !commands.empty();}));
  MotorTrajectory trajectory; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 77;
  trajectory.segment_count = 1; trajectory.target_count = 1; trajectory.target_ids = {64};
  trajectory.count = 1; MotorTrajectoryPoint p; p.motor_id = 64; p.position = 1;
  trajectory.points.push_back(p); trajectory_pub->publish(trajectory);
  ASSERT_TRUE(spin([&] {return !trajectories.empty();}));
  command_pub->publish(command()); ASSERT_TRUE(spin([&] {return commands.size() >= 2;}));
  EXPECT_EQ(commands.back().count, 0);
  auto changed = command(); changed.targets[0].position = 0.75;
  command_pub->publish(changed); ASSERT_TRUE(spin([&] {return commands.size() >= 3;}));
  EXPECT_EQ(commands.back().count, 1);
  const auto before = trajectory_states.size();
  status.trajectory.trajectory_id = trajectories.front().trajectory_id;
  status.trajectory.state = AgentTrajectoryState::LOADING;
  status.trajectory.target_ids = {64};
  run(100ms); EXPECT_EQ(trajectory_states.size(), before);
}
TEST_F(BridgeTest, StaleRejectionReceiptDoesNotOverrideAcceptedTransaction)
{
  start(); SystemCommand cmd; cmd.command = SystemCommand::COMMAND_SET_ZERO;
  cmd.motor_id = 64; cmd.sync_time_ns = stamp(); system_command_pub->publish(cmd);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_ZERO;});}));
  const auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SET_ZERO;});
  AgentConfigResult pending; pending.request_id = wire.request_id;
  pending.operation = wire.operation; pending.state = AgentConfigResult::PENDING;
  status.results = {pending}; status.rejection_receipt = pending;
  status.rejection_receipt.state = AgentConfigResult::BUSY;
  run(90ms); EXPECT_TRUE(results.empty());
  status.results[0].state = AgentConfigResult::SUCCEEDED;
  ASSERT_TRUE(spin([&] {return !results.empty();}));
  EXPECT_EQ(results.back().status, ConfigResult::STATUS_SUCCEEDED);
}
TEST_F(BridgeTest, TrajectoryIdSeedsFromRequestHighWaterAfterBridgeRestart)
{
  start(); status.trajectory_request_high_water = 80; publish_status(); run();
  MotorTrajectory trajectory; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 9;
  trajectory.segment_count = 1; trajectory.target_count = 1; trajectory.target_ids = {64};
  trajectory.count = 1; MotorTrajectoryPoint p; p.motor_id = 64; trajectory.points.push_back(p);
  trajectory_pub->publish(trajectory);
  ASSERT_TRUE(spin([&] {return !trajectories.empty();}));
  EXPECT_GT(trajectories.back().trajectory_id, 80u);
  EXPECT_EQ(trajectories.back().request_id, 81u);
}
TEST_F(BridgeTest, ReadOnlyReportServiceCompletesWithUnhealthyClock)
{
  start(); status.clock_valid = false; publish_status(); run();
  auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/motor/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REPORT;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  EXPECT_EQ(future.get()->result.status, ConfigResult::STATUS_SUCCEEDED);
}
TEST_F(BridgeTest, ServiceTimeoutRetainsCorrelationAndPublishesLateTerminalBeforeAck)
{
  start(); auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/motor/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_SAVE;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  request->request.request_id = 7; request->request.sync_time_ns = stamp(); request->timeout_ms = 100;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SAVE;});}));
  auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::SAVE;});
  AgentConfigResult result; result.request_id = wire.request_id; result.operation = wire.operation;
  result.state = AgentConfigResult::PENDING; status.results = {result};
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  auto response = future.get()->result;
  EXPECT_EQ(response.status, ConfigResult::STATUS_PENDING); EXPECT_EQ(response.error_code, 21);
  EXPECT_TRUE(std::none_of(results.begin(), results.end(), [](const auto & r) {
    return r.status == ConfigResult::STATUS_FAILED;
  }));
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::ACK_RESULTS;
  }));
  status.results[0].state = AgentConfigResult::SUCCEEDED;
  ASSERT_TRUE(spin([&] {return std::any_of(results.begin(), results.end(), [](const auto & r) {
    return r.request_id == 7 && r.status == ConfigResult::STATUS_SUCCEEDED;
  });}));
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::ACK_RESULTS && c.ack_request_ids[0] == wire.request_id;
  });}));
}
