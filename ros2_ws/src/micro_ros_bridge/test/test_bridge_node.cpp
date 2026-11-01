#include "agent_bridge_fixture.hpp"
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
TEST_F(BridgeTest, ExplicitLegacyExecutionCannotChangeWhileRunning)
{
  ASSERT_TRUE(bridge->has_parameter("trajectory_execution"));
  EXPECT_EQ(bridge->get_parameter("trajectory_execution").as_string(), "firmware");
  EXPECT_FALSE(bridge->set_parameter(rclcpp::Parameter("trajectory_execution", "host")).successful);
}
TEST_F(HostBridgeTest, NewBridgeDefaultsToHostControllerExecution)
{
  auto defaults = std::make_shared<micro_ros_bridge::BridgeNode>();
  EXPECT_EQ(defaults->get_parameter("trajectory_execution").as_string(), "host");
}
TEST_F(HostBridgeTest, OnlyHostControllerOutputCanReachMcu)
{
  start(); run(100ms);
  auto legacy = peer->create_publisher<MotorCommandArray>("/motor/command_array", 1);
  run(50ms); legacy->publish(command()); run();
  EXPECT_TRUE(commands.empty());
  command_pub->publish(command());
  ASSERT_TRUE(spin([&] {return commands.size() == 1;}));
  EXPECT_EQ(commands.back().motor_ids[0], 64);
  run(80ms); EXPECT_EQ(commands.size(), 1u);
}
TEST_F(HostBridgeTest, RejectsFirmwareTrajectoryWhileCancelKeepsIndependentStop)
{
  start(); run(100ms); configs.clear();
  std::vector<MotorEvent> events; listen("/motor/events", events); run(50ms);
  MotorTrajectory trajectory; trajectory.trajectory_id = 7; trajectory.count = 1;
  trajectory_pub->publish(trajectory);
  ASSERT_TRUE(spin([&] {return !events.empty();}));
  EXPECT_EQ(events.back().event_type, MotorEvent::EVENT_TRAJECTORY_REJECTED);
  EXPECT_EQ(events.back().request_id, 7u); EXPECT_TRUE(trajectories.empty());
  configs.clear();
  trajectory_pub->publish(MotorTrajectory{}); run(80ms);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::STOP_ALL;
  }));
}
TEST_F(HostBridgeTest, TakeoverRequiresOldTrajectoryAndAuthorizationToBeCleared)
{
  status.trajectory.state = AgentTrajectoryState::RUNNING;
  status.trajectory.trajectory_id = 1;
  start(); status.motors[0].policy_enabled = true;
  run(100ms); command_pub->publish(command()); run();
  EXPECT_TRUE(commands.empty());
  EXPECT_TRUE(std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::STOP_ALL;}));
  status.trajectory.state = AgentTrajectoryState::ABORTED;
  status.motors[0].policy_enabled = false;
  run(100ms); command_pub->publish(command());
  ASSERT_TRUE(spin([&] {return !commands.empty();}));
}
TEST_F(BridgeTest, MultipleWireCommandWritersStopInsteadOfCompeting)
{
  start(); configs.clear();
  auto duplicate = peer->create_publisher<AgentCommand>(AgentWireContract::TOPIC_COMMAND,
    rclcpp::QoS(1).best_effort());
  ASSERT_TRUE(spin([&] {return bridge->count_publishers(AgentWireContract::TOPIC_COMMAND) == 2;}));
  command_pub->publish(command());
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::STOP_ALL;});}));
  EXPECT_TRUE(commands.empty());
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
  auto second = peer->create_publisher<MotorCommandArray>("/robot/managed/motor/command_array", 1);
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
  MotorPolicyArray policy; policy.session_id = status.session_id; policy.lease_id = lease_id; policy.count = 1; policy.policy_generation = 9;
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
  SystemCommand cmd; cmd.sync_time_ns = stamp(); cmd.command = SystemCommand::COMMAND_REFRESH;
  cmd.motor_id = 64; cmd.request_id = 7; system_command_pub->publish(cmd);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});}));
  auto wire = *std::find_if(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});
  EXPECT_EQ(wire.request_id, 51u);
  auto next = cmd; next.request_id = 8; next.sync_time_ns = stamp(); system_command_pub->publish(next);
  run(); EXPECT_EQ(std::count_if(configs.begin(),configs.end(),[](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;}), 1);
  AgentConfigResult result; result.request_id = 51; result.operation = AgentConfigRequest::REFRESH_PARAMETERS;
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
TEST_F(BridgeTest, ReadOnlyMaintenanceTerminalKeepsStatusFlowAndIsAcknowledged)
{
  start();
  std::vector<SystemStatus> systems;
  listen("/motor/system_status", systems);
  auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/robot/managed/maintenance/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  request->request.request_id = 7;
  request->request.sync_time_ns = stamp();
  request->request.count = 0;
  request->request.command = SystemCommand::COMMAND_REFRESH;
  request->request.motor_id = 1;
  maintenance_pub->publish(request->request);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;
  });}));
  const auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;
  });
  AgentConfigResult receipt;
  receipt.request_id = wire.request_id;
  receipt.operation = wire.operation;
  receipt.state = AgentConfigResult::SUCCEEDED;
  receipt.completed_motor_mask = 1;
  status.results = {receipt};
  status.firmware_values_valid = true;
  status.firmware_values = {100, 100, 10, 100, 2, 200};
  status.firmware_unsaved = true;
  ASSERT_TRUE(spin([&] {return std::any_of(results.begin(), results.end(), [](const auto &r) {
    return r.request_id == 7 && r.status == ConfigResult::STATUS_SUCCEEDED;});}));
  ASSERT_TRUE(spin([&] {return !systems.empty() && systems.back().firmware_values_valid;}));
  EXPECT_EQ(systems.back().firmware_values[1], 100);
  EXPECT_TRUE(systems.back().firmware_unsaved);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [&](const auto & c) {
    return c.operation == AgentConfigRequest::ACK_RESULTS && c.ack_request_ids[0] == wire.request_id;
  });}));
}

TEST_F(BridgeTest, LowLevelServiceRejectsMaintenanceWritesBeforeAnyWireRequest)
{
  start(); auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/robot/managed/maintenance/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_CLEAR_FAULT;
  request->request.motor_id = 64; request->request.sync_time_ns = stamp(); request->timeout_ms = 100;
  auto future = client->async_send_request(request);
  run(40ms);
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  auto timed_out = future.get()->result;
  EXPECT_EQ(timed_out.error_code, 10);
  EXPECT_EQ(timed_out.status, ConfigResult::STATUS_FAILED);
  EXPECT_TRUE(std::none_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::CLEAR_FAULT; }));
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
  start(); MotorTrajectory trajectory; trajectory.session_id=status.session_id; trajectory.lease_id=lease_id; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 77;
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
  start(); auto_info = false; auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/robot/managed/maintenance/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REPORT; request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  request->request.sync_time_ns = stamp(); auto future = client->async_send_request(request);
  run(); auto_info = true; status.session_id = 101;
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
  MotorTrajectory trajectory; trajectory.session_id=status.session_id; trajectory.lease_id=lease_id; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 77;
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
  start(); SystemCommand cmd; cmd.command = SystemCommand::COMMAND_REFRESH;
  cmd.motor_id = 64; cmd.sync_time_ns = stamp(); system_command_pub->publish(cmd);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});}));
  const auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});
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
  MotorTrajectory trajectory; trajectory.session_id=status.session_id; trajectory.lease_id=lease_id; trajectory.sync_time_ns = stamp(); trajectory.trajectory_id = 9;
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
  auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/robot/managed/maintenance/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REPORT;
  request->request.motor_id = SystemCommand::MOTOR_ID_NONE;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin([&] {return future.wait_for(0ms) == std::future_status::ready;}));
  EXPECT_EQ(future.get()->result.status, ConfigResult::STATUS_SUCCEEDED);
}
TEST_F(BridgeTest, TopicTimeoutRetainsCorrelationAndPublishesLateTerminalBeforeAck)
{
  start(); auto client = peer->create_client<can_bridge_msgs::srv::ExecuteConfig>("/robot/managed/maintenance/execute_config");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<can_bridge_msgs::srv::ExecuteConfig::Request>();
  request->request.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  request->request.command = SystemCommand::COMMAND_REFRESH;
  request->request.motor_id = 1;
  request->request.request_id = 7; request->request.sync_time_ns = stamp(); request->timeout_ms = 100;
  maintenance_pub->publish(request->request);
  ASSERT_TRUE(spin([&] {return std::any_of(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});}));
  auto wire = *std::find_if(configs.begin(), configs.end(), [](const auto & c) {
    return c.operation == AgentConfigRequest::REFRESH_PARAMETERS;});
  AgentConfigResult result; result.request_id = wire.request_id; result.operation = wire.operation;
  result.state = AgentConfigResult::PENDING; status.results = {result};
  run(5100ms);
  EXPECT_TRUE(std::any_of(results.begin(), results.end(), [](const auto & r) {
    return r.request_id == 7 && r.status == ConfigResult::STATUS_PENDING && r.error_code == 21; }));
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
