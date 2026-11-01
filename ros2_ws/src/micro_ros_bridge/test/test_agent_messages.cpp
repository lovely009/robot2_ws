#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <can_bridge_msgs/msg/agent_command.hpp>
#include <can_bridge_msgs/msg/agent_config_request.hpp>
#include <can_bridge_msgs/msg/agent_device_info.hpp>
#include <can_bridge_msgs/msg/agent_info.hpp>
#include <can_bridge_msgs/msg/agent_motion.hpp>
#include <can_bridge_msgs/msg/agent_motor_status.hpp>
#include <can_bridge_msgs/msg/agent_status.hpp>
#include <can_bridge_msgs/msg/agent_trajectory_state.hpp>
#include <can_bridge_msgs/msg/agent_wire_contract.hpp>
#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/device_param_entry.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_command_target.hpp>
#include <can_bridge_msgs/msg/motor_control_state.hpp>
#include <can_bridge_msgs/msg/motor_control_state_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_target.hpp>
#include <can_bridge_msgs/msg/motor_policy_array.hpp>
#include <can_bridge_msgs/msg/motor_policy_target.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_status_target.hpp>
#include <can_bridge_msgs/msg/motor_topology.hpp>
#include <can_bridge_msgs/msg/motor_topology_entry.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/param_set_target.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>

#include "micro_ros_bridge/agent_messages.hpp"

namespace
{
using can_bridge_msgs::msg::AgentCommand;
using can_bridge_msgs::msg::AgentConfigRequest;
using can_bridge_msgs::msg::AgentDeviceInfo;
using can_bridge_msgs::msg::AgentInfo;
using can_bridge_msgs::msg::AgentMotion;
using can_bridge_msgs::msg::AgentMotorStatus;
using can_bridge_msgs::msg::AgentStatus;
using can_bridge_msgs::msg::AgentTrajectoryState;
using can_bridge_msgs::msg::ConfigRequest;
using can_bridge_msgs::msg::DeviceParamEntry;
using can_bridge_msgs::msg::DeviceParamReport;
using can_bridge_msgs::msg::LowState;
using can_bridge_msgs::msg::MotorCommandArray;
using can_bridge_msgs::msg::MotorCommandTarget;
using can_bridge_msgs::msg::MotorControlState;
using can_bridge_msgs::msg::MotorControlStateArray;
using can_bridge_msgs::msg::MotorDiagnostics;
using can_bridge_msgs::msg::MotorMotionStateTarget;
using can_bridge_msgs::msg::MotorPolicyArray;
using can_bridge_msgs::msg::MotorPolicyTarget;
using can_bridge_msgs::msg::MotorStatusArray;
using can_bridge_msgs::msg::MotorStatusTarget;
using can_bridge_msgs::msg::MotorTopology;
using can_bridge_msgs::msg::MotorTrajectoryState;
using can_bridge_msgs::msg::ParamSetTarget;
using can_bridge_msgs::msg::SystemCommand;
using can_bridge_msgs::msg::SystemStatus;

MotorCommandTarget command_target(uint16_t id, float p = 0.1f, float v = 0.2f, float e = 0.3f)
{
  MotorCommandTarget target;
  target.motor_id = id;
  target.position = p;
  target.velocity = v;
  target.effort = e;
  return target;
}

MotorPolicyTarget policy_target(
  uint16_t id, bool enabled, float kp, float kd, bool confirm = false)
{
  MotorPolicyTarget target;
  target.motor_id = id;
  target.enabled = enabled;
  target.kp = kp;
  target.kd = kd;
  target.confirm_unprotected = confirm;
  return target;
}

AgentMotorStatus motor_status(uint8_t id)
{
  AgentMotorStatus motor;
  motor.motor_id = id;
  motor.lifecycle = MotorControlState::LIFECYCLE_ACTIVE;
  motor.execution_state = MotorControlState::EXECUTION_COMMAND;
  motor.online = true;
  motor.policy_enabled = true;
  motor.kernel_enabled = true;
  motor.policy_generation = 9;
  motor.temperature_dc = 253;
  motor.current_ma = 100000;
  motor.voltage_mv = 100000;
  motor.status_flags = 0x01;
  return motor;
}

AgentDeviceInfo device_info(uint16_t id)
{
  AgentDeviceInfo device;
  device.motor_id = id;
  device.bus_index = 0;
  device.node_id = id;
  device.protocol = can_bridge_msgs::msg::MotorTopologyEntry::PROTOCOL_CERB;
  device.model_name = "model";
  device.parameters.motor_id = id;
  device.parameters.rec_pos_min = -1.0f;
  device.parameters.rec_pos_max = 1.0f;
  return device;
}

AgentStatus valid_status()
{
  AgentStatus status;
  status.schema_version = can_bridge_msgs::msg::AgentWireContract::SCHEMA_VERSION;
  status.session_id = 7;
  status.snapshot_stamp = 123456;
  status.sequence = 4;
  status.snapshot_tick_ms = 999;
  status.clock_valid = true;
  return status;
}
}  // namespace

TEST(AgentCommand, AcceptsId64LimitAndKeepsEmptyAsSnapshot)
{
  MotorCommandArray in;
  in.sequence = 12;
  in.count = 2;
  in.targets.push_back(command_target(63));
  in.targets.push_back(command_target(64));
  AgentCommand out;
  const auto result = micro_ros_bridge::encode_agent_command(in, out);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(out.schema_version, 3u);
  EXPECT_EQ(out.mode, AgentCommand::SNAPSHOT);
  EXPECT_EQ(out.sequence, 12u);
  ASSERT_EQ(out.motor_ids.size(), 2u);
  EXPECT_EQ(out.motor_ids[0], 63);
  EXPECT_EQ(out.motor_ids[1], 64);

  in.targets.clear();
  in.count = 0;
  ASSERT_TRUE(micro_ros_bridge::encode_agent_command(in, out).ok);
  EXPECT_EQ(out.mode, AgentCommand::SNAPSHOT);
  EXPECT_EQ(out.count, 0u);
  EXPECT_TRUE(out.motor_ids.empty());
}

TEST(AgentCommand, EnforcesCountRangeIdsUniquenessAndFiniteValues)
{
  AgentCommand untouched;
  untouched.sequence = 77;
  untouched.count = 9;
  const auto unchanged = untouched;

  MotorCommandArray in;
  in.count = 1;
  in.targets.push_back(command_target(0));
  EXPECT_FALSE(micro_ros_bridge::encode_agent_command(in, untouched).ok);
  in.targets[0] = command_target(65);
  EXPECT_FALSE(micro_ros_bridge::encode_agent_command(in, untouched).ok);
  in.targets[0] = command_target(7);
  in.targets.push_back(command_target(7));
  in.count = 2;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_command(in, untouched).ok);
  in.targets.pop_back();
  in.targets[0].position = std::numeric_limits<float>::quiet_NaN();
  in.count = 1;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_command(in, untouched).ok);
  EXPECT_EQ(untouched.sequence, unchanged.sequence);
  EXPECT_EQ(untouched.count, unchanged.count);

  in.targets.clear();
  for (uint16_t id = 0; id < 48; ++id) {
    in.targets.push_back(command_target(id + 1));
  }
  in.count = 48;
  const auto boundary_result = micro_ros_bridge::encode_agent_command(in, untouched);
  EXPECT_TRUE(boundary_result.ok) << boundary_result.error;
  in.targets.push_back(command_target(49));
  in.count = 49;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_command(in, untouched).ok);
}

TEST(AgentPolicy, NormalizesAllDisabledAndPreservesMixedPolicySemantics)
{
  MotorPolicyArray in;
  in.session_id = 8;
  in.sync_time_ns = 99;
  in.request_id = 12;
  in.policy_generation = 21;
  in.count = 2;
  in.targets = {
    policy_target(2, false, 3.0f, 4.0f, true),
    policy_target(64, false, 5.0f, 6.0f, true)};
  AgentConfigRequest out;
  const auto result = micro_ros_bridge::encode_agent_policy(in, out);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(out.operation, AgentConfigRequest::SET_POLICY);
  EXPECT_EQ(out.request_id, 12u);
  EXPECT_EQ(out.policy_generation, 21u);
  EXPECT_EQ(out.issued_stamp, 99u);
  EXPECT_FALSE(out.allow_no_protect);
  ASSERT_EQ(out.policies.size(), 2u);
  EXPECT_FLOAT_EQ(out.policies[0].kp, 0.0f);
  EXPECT_FLOAT_EQ(out.policies[1].kd, 0.0f);
  EXPECT_FALSE(out.policies[0].confirm_unprotected);

  in.targets[0].enabled = true;
  EXPECT_TRUE(micro_ros_bridge::encode_agent_policy(in, out).ok);
  EXPECT_FLOAT_EQ(out.policies[1].kp, 5.0f);
  EXPECT_TRUE(in.targets[0].confirm_unprotected);
}

TEST(AgentPolicy, RejectsEmptyDuplicateBadRangeAndNonFiniteGains)
{
  MotorPolicyArray in;
  in.count = 0;
  AgentConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);

  in.count = 2;
  in.targets = {policy_target(1, true, 1.0f, 0.1f), policy_target(1, false, 1.0f, 0.1f)};
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);
  in.targets[1] = policy_target(0, false, 0.0f, 0.0f);
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);
  in.targets[1] = policy_target(2, true, -1.0f, 0.1f);
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);
  in.targets[1] = policy_target(2, true, std::numeric_limits<float>::infinity(), 0.1f);
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);

  in.targets.clear();
  for (uint16_t id = 1; id <= 48; ++id) {
    in.targets.push_back(policy_target(id, false, 0.0f, 0.0f));
  }
  in.count = 48;
  EXPECT_TRUE(micro_ros_bridge::encode_agent_policy(in, out).ok);
  in.targets.push_back(policy_target(49, false, 0.0f, 0.0f));
  in.count = 49;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_policy(in, out).ok);
}

TEST(AgentConfig, RejectsRetiredWritesAndMapsReadOperationsWithStrictIds)
{
  ConfigRequest in;
  in.sync_time_ns = 88;
  in.session_id = 7;
  in.request_id = 6;
  in.kind = ConfigRequest::CONFIG_KIND_PARAM_SET;
  ParamSetTarget param;
  param.motor_id = 64;
  param.field = ParamSetTarget::FIELD_VEL_LIMIT;
  param.value = 8.0f;
  in.targets = {param};
  in.count = 1;
  AgentConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
  EXPECT_TRUE(out.parameters.empty());

  in.targets[0].motor_id = 0;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);

  in.kind = ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  in.targets.clear();
  in.count = 0;
  in.command = SystemCommand::COMMAND_REPORT;
  in.motor_id = SystemCommand::MOTOR_ID_NONE;
  EXPECT_TRUE(micro_ros_bridge::encode_agent_config(in, out).ok);
  EXPECT_EQ(out.operation, AgentConfigRequest::QUERY_INFO);
  EXPECT_EQ(out.motor_id, SystemCommand::MOTOR_ID_NONE);

  in.command = SystemCommand::COMMAND_REFRESH;
  in.motor_id = 64;
  EXPECT_TRUE(micro_ros_bridge::encode_agent_config(in, out).ok);
  EXPECT_EQ(out.operation, AgentConfigRequest::REFRESH_PARAMETERS);
  EXPECT_EQ(out.motor_id, 64u);
  in.motor_id = 0;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
}

TEST(AgentMotion, ValidatesGroupCardinalityAndMapsId64)
{
  AgentMotion empty;
  empty.schema_version = 3;
  empty.session_id = 5;
  empty.group_count = 1;
  LowState out;
  ASSERT_TRUE(micro_ros_bridge::agent_motion_frame(empty, 0, out).ok);
  EXPECT_EQ(out.chunk_count, 1u);
  EXPECT_EQ(out.schema_version, can_bridge_msgs::msg::WireContract::SCHEMA_VERSION);

  AgentMotion in;
  in.schema_version = 3;
  in.session_id = 5;
  in.snapshot_stamp = 10;
  in.sequence = 2;
  in.snapshot_tick_ms = 3;
  in.group_index = 0;
  in.group_count = 1;
  in.count = 1;
  in.motor_ids.push_back(64);
  in.position_q.push_back(1);
  in.velocity_q.push_back(2);
  in.effort_q.push_back(3);
  in.state_age_ms.push_back(4);
  in.target_source.push_back(MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY);
  ASSERT_TRUE(micro_ros_bridge::agent_motion_frame(in, 1, out).ok);
  EXPECT_EQ(out.motor_ids[0], 64);
  EXPECT_EQ(out.total_count, 1u);

  in.motor_ids[0] = 0;
  EXPECT_FALSE(micro_ros_bridge::agent_motion_frame(in, 1, out).ok);
  in.motor_ids[0] = 64;
  EXPECT_FALSE(micro_ros_bridge::agent_motion_frame(in, 2, out).ok);
  in.group_count = 2;
  EXPECT_FALSE(micro_ros_bridge::agent_motion_frame(in, 1, out).ok);
}

TEST(AgentMotion, RejectsWrongChunkLengthForSecondPage)
{
  AgentMotion in;
  in.schema_version = 3;
  in.session_id = 5;
  in.group_index = 1;
  in.group_count = 2;
  in.count = 1;
  in.motor_ids.push_back(33);
  in.position_q.push_back(0);
  in.velocity_q.push_back(0);
  in.effort_q.push_back(0);
  in.state_age_ms.push_back(0);
  in.target_source.push_back(0);
  in.position_q.push_back(0);
  LowState out;
  EXPECT_FALSE(micro_ros_bridge::agent_motion_frame(in, 33, out).ok);
}

TEST(AgentStatus, MapsControlStateLifecycleTrajectoryAndElectricalSentinels)
{
  AgentStatus in = valid_status();
  auto motor = motor_status(64);
  motor.execution_state = MotorControlState::EXECUTION_SOFT_STOP;
  motor.status_flags = 0x1F;
  motor.protection_flags = 0x10;
  motor.limit_flags = 0x8;
  in.motors = {motor};
  in.trajectory.state = AgentTrajectoryState::RUNNING;
  in.trajectory.trajectory_id = 22;
  in.trajectory.target_ids = {64};
  in.trajectory.total_segments = 2;
  in.trajectory.received_segments = 1;
  in.diagnostics.ring_drop = {1, 2, 3};

  MotorStatusArray statuses;
  MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  SystemStatus system;
  system.refresh_seq = 31;
  system.save_seq = 32;
  MotorDiagnostics diagnostics;
  const auto result = micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(statuses.statuses.size(), 1u);
  EXPECT_EQ(statuses.statuses[0].lifecycle, MotorStatusTarget::LIFECYCLE_SAFE_STOP);
  EXPECT_NEAR(statuses.statuses[0].temperature, 25.3, 1e-5);
  EXPECT_NEAR(statuses.statuses[0].current, 100.0, 1e-5);
  EXPECT_NEAR(statuses.statuses[0].voltage, 100.0, 1e-5);
  ASSERT_EQ(controls.states.size(), 1u);
  EXPECT_EQ(controls.states[0].control_lifecycle, MotorControlState::LIFECYCLE_ACTIVE);
  EXPECT_EQ(controls.states[0].state_age_ms, std::numeric_limits<uint32_t>::max());
  EXPECT_TRUE(trajectory.active);
  EXPECT_FALSE(trajectory.completed);
  EXPECT_EQ(system.refresh_seq, 31u);
  EXPECT_EQ(system.save_seq, 32u);
  EXPECT_EQ(diagnostics.ring_drop.size(), 3u);

  in.motors[0].temperature_dc = std::numeric_limits<int16_t>::min();
  in.motors[0].current_ma = std::numeric_limits<int32_t>::min();
  in.motors[0].voltage_mv = std::numeric_limits<uint32_t>::max();
  in.trajectory.state = AgentTrajectoryState::SUCCEEDED;
  ASSERT_TRUE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
  EXPECT_TRUE(std::isnan(statuses.statuses[0].temperature));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].current));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].voltage));
  EXPECT_FALSE(trajectory.active);
  EXPECT_TRUE(trajectory.completed);

  in.trajectory.state = AgentTrajectoryState::CANCELED;
  ASSERT_TRUE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
  EXPECT_FALSE(trajectory.active);
  EXPECT_FALSE(trajectory.completed);
}

TEST(AgentStatus, RejectsDuplicateTargetsAndLeavesOutputsUntouched)
{
  AgentStatus in = valid_status();
  in.motors = {motor_status(1), motor_status(1)};
  in.trajectory.state = AgentTrajectoryState::RUNNING;
  in.trajectory.trajectory_id = 3;
  in.trajectory.target_ids = {1, 1};
  MotorStatusArray statuses;
  statuses.sync_time_ns = 314;
  MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  trajectory.trajectory_id = 271;
  SystemStatus system;
  system.last_error = 9;
  MotorDiagnostics diagnostics;
  EXPECT_FALSE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
  EXPECT_EQ(statuses.sync_time_ns, 314u);
  EXPECT_EQ(trajectory.trajectory_id, 271u);
  EXPECT_EQ(system.last_error, 9u);
}

TEST(AgentInfo, AppendsId64DeviceWithMatchingParameterId)
{
  AgentInfo info;
  info.schema_version = 3;
  info.session_id = 7;
  info.snapshot_stamp = 22;
  info.kind = AgentInfo::DEVICE_PAGE;
  info.result = AgentInfo::OK;
  info.cursor = 0;
  info.next_cursor = 1;
  info.complete = true;
  info.motor_capacity = 48;
  info.devices = {device_info(64)};
  MotorTopology topology;
  DeviceParamReport report;
  const auto result = micro_ros_bridge::append_agent_info(info, topology, report);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(topology.entries.size(), 1u);
  ASSERT_EQ(report.entries.size(), 1u);
  EXPECT_EQ(topology.entries[0].motor_id, 64u);
  EXPECT_EQ(report.entries[0].motor_id, 64u);
  EXPECT_EQ(topology.schema_version, can_bridge_msgs::msg::WireContract::SCHEMA_VERSION);
}

TEST(AgentInfo, DuplicateAcrossPagesIsAtomicAndParameterIdMustMatch)
{
  AgentInfo info;
  info.schema_version = 3;
  info.session_id = 7;
  info.kind = AgentInfo::DEVICE_PAGE;
  info.result = AgentInfo::OK;
  info.cursor = 0;
  info.next_cursor = 1;
  info.complete = false;
  info.motor_capacity = 48;
  info.devices = {device_info(5)};
  MotorTopology topology;
  can_bridge_msgs::msg::MotorTopologyEntry prior;
  prior.motor_id = 5;
  topology.entries = {prior};
  DeviceParamReport report;
  DeviceParamEntry prior_param;
  prior_param.motor_id = 5;
  report.entries = {prior_param};
  report.count = 1;
  const auto before_topology = topology;
  const auto before_report = report;
  EXPECT_FALSE(micro_ros_bridge::append_agent_info(info, topology, report).ok);
  EXPECT_EQ(topology.entries.size(), before_topology.entries.size());
  EXPECT_EQ(report.entries.size(), before_report.entries.size());

  topology = MotorTopology{};
  report = DeviceParamReport{};
  info.devices[0].parameters.motor_id = 6;
  EXPECT_FALSE(micro_ros_bridge::append_agent_info(info, topology, report).ok);
  EXPECT_TRUE(topology.entries.empty());
  EXPECT_TRUE(report.entries.empty());
}

TEST(AgentInfo, CompletePageMayHaveFewerThanCapacityOrBeEmpty)
{
  AgentInfo one;
  one.schema_version = 3;
  one.session_id = 7;
  one.kind = AgentInfo::DEVICE_PAGE;
  one.result = AgentInfo::OK;
  one.cursor = 0;
  one.next_cursor = 1;
  one.complete = true;
  one.motor_capacity = 48;
  one.devices = {device_info(1)};
  MotorTopology topology;
  DeviceParamReport report;
  EXPECT_TRUE(micro_ros_bridge::append_agent_info(one, topology, report).ok);

  AgentInfo empty;
  empty.schema_version = 3;
  empty.session_id = 7;
  empty.kind = AgentInfo::DEVICE_PAGE;
  empty.result = AgentInfo::OK;
  empty.cursor = 0;
  empty.next_cursor = 0;
  empty.complete = true;
  empty.motor_capacity = 48;
  MotorTopology empty_topology;
  DeviceParamReport empty_report;
  EXPECT_TRUE(micro_ros_bridge::append_agent_info(empty, empty_topology, empty_report).ok);
}

TEST(AgentStatus, AcceptsFirmwareParameterResultsAndRejectsUnknownOperations)
{
  AgentStatus in = valid_status();
  in.firmware_values_valid = true;
  in.firmware_values = {100, 100, 10, 100, 2, 200};
  can_bridge_msgs::msg::AgentConfigResult receipt;
  receipt.request_id = 5;
  receipt.operation = AgentConfigRequest::SET_FIRMWARE_PARAMETERS;
  MotorStatusArray statuses;
  MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  SystemStatus system;
  MotorDiagnostics diagnostics;
  for (uint8_t state = receipt.PENDING; state <= receipt.RETIRED; ++state) {
    receipt.state = state;
    in.results = {receipt};
    ASSERT_TRUE(micro_ros_bridge::decode_agent_status(
      in, statuses, controls, trajectory, system, diagnostics).ok);
    EXPECT_TRUE(system.firmware_values_valid);
    EXPECT_EQ(system.firmware_values[1], 100);
  }
  in.results[0].operation = AgentConfigRequest::SET_FIRMWARE_PARAMETERS + 1;
  EXPECT_FALSE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
}

TEST(AgentStatus, AllowsPartialMasksToOverlap)
{
  AgentStatus in = valid_status();
  can_bridge_msgs::msg::AgentConfigResult result;
  result.request_id = 5;
  result.operation = AgentConfigRequest::SET_PARAMETERS;
  result.state = can_bridge_msgs::msg::AgentConfigResult::PARTIAL;
  result.completed_motor_mask = 1u;
  result.failed_motor_mask = 1u;
  in.results = {result};
  MotorStatusArray statuses;
  MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  SystemStatus system;
  MotorDiagnostics diagnostics;
  EXPECT_TRUE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
}

TEST(AgentStatus, OfflineMotorMayRetainPolicyAndKernelIntent)
{
  AgentStatus in = valid_status();
  auto motor = motor_status(2);
  motor.online = false;
  motor.policy_enabled = true;
  motor.kernel_enabled = true;
  motor.status_flags = 0;
  in.motors = {motor};
  MotorStatusArray statuses;
  MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  SystemStatus system;
  MotorDiagnostics diagnostics;
  ASSERT_TRUE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
  EXPECT_EQ(statuses.statuses[0].lifecycle, MotorStatusTarget::LIFECYCLE_OFFLINE);
  EXPECT_TRUE(controls.states[0].policy_enabled);
  EXPECT_TRUE(controls.states[0].motion_enabled);
}

TEST(AgentStatus, MapsEveryAllowedControlLifecycle)
{
  struct Case { uint8_t wire; uint8_t legacy; };
  const std::array<Case, 9> cases = {{
    {0, MotorStatusTarget::LIFECYCLE_ABSENT},
    {1, MotorStatusTarget::LIFECYCLE_DISCOVERED},
    {2, MotorStatusTarget::LIFECYCLE_READY_DISABLED},
    {3, MotorStatusTarget::LIFECYCLE_ACTIVE},
    {4, MotorStatusTarget::LIFECYCLE_ABSENT},
    {10, MotorStatusTarget::LIFECYCLE_IDENTIFYING},
    {13, MotorStatusTarget::LIFECYCLE_READY_DISABLED},
    {14, MotorStatusTarget::LIFECYCLE_SAFE_STOP},
    {15, MotorStatusTarget::LIFECYCLE_FAULT}}};
  for (const auto & item : cases) {
    AgentStatus in = valid_status();
    auto motor = motor_status(1);
    motor.lifecycle = item.wire;
    if (item.wire != 3) {motor.execution_state = MotorControlState::EXECUTION_IDLE;}
    in.motors = {motor};
    MotorStatusArray statuses;
    MotorControlStateArray controls;
    MotorTrajectoryState trajectory;
    SystemStatus system;
    MotorDiagnostics diagnostics;
    ASSERT_TRUE(micro_ros_bridge::decode_agent_status(
      in, statuses, controls, trajectory, system, diagnostics).ok);
    EXPECT_EQ(statuses.statuses[0].lifecycle, item.legacy) << +item.wire;
  }
}

TEST(AgentInfo, AppendsFullPageAndRejectsCapacityAboveFortyEight)
{
  AgentInfo info;
  info.schema_version = 3;
  info.session_id = 7;
  info.kind = AgentInfo::DEVICE_PAGE;
  info.result = AgentInfo::OK;
  info.cursor = 0;
  info.next_cursor = 16;
  info.complete = true;
  info.motor_capacity = 48;
  for (uint16_t id = 1; id <= 16; ++id) {info.devices.push_back(device_info(id));}
  MotorTopology topology;
  DeviceParamReport report;
  ASSERT_TRUE(micro_ros_bridge::append_agent_info(info, topology, report).ok);
  EXPECT_EQ(topology.entries.size(), 16u);
  EXPECT_EQ(report.entries.size(), 16u);

  info.motor_capacity = 49;
  MotorTopology invalid_topology;
  DeviceParamReport invalid_report;
  EXPECT_FALSE(micro_ros_bridge::append_agent_info(info, invalid_topology, invalid_report).ok);
}

TEST(AgentStatus, RejectedMalformedRequestEchoCannotPoisonHealthyStatus)
{
  AgentStatus in = valid_status();
  in.rejection_receipt.request_id = 0;
  in.rejection_receipt.operation = 255;
  in.rejection_receipt.state = can_bridge_msgs::msg::AgentConfigResult::FAILED;
  in.rejection_receipt.error = 10;
  in.trajectory_request_receipt.accepted = false;
  in.trajectory_request_receipt.request_id = 0;
  in.trajectory_request_receipt.trajectory_id = 0;
  in.trajectory_request_receipt.operation = 255;
  in.trajectory_request_receipt.error = 2;
  MotorStatusArray statuses; MotorControlStateArray controls;
  MotorTrajectoryState trajectory; SystemStatus system; MotorDiagnostics diagnostics;
  EXPECT_TRUE(micro_ros_bridge::decode_agent_status(
    in, statuses, controls, trajectory, system, diagnostics).ok);
}

TEST(AgentMessages, RetiredFirmwareTopicRequestIsRejected) {
  ConfigRequest in;
  in.kind = ConfigRequest::CONFIG_KIND_FIRMWARE_PARAMETERS;
  in.motor_id = UINT16_MAX; in.command = 0; in.count = 1;
  in.request_id = 1; in.session_id = 9;
  ParamSetTarget target; target.motor_id = UINT16_MAX; target.field = 0; target.value = 20;
  in.targets.push_back(target);
  AgentConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
  EXPECT_TRUE(out.parameters.empty());
  in.targets[0].field = 6;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
  in.targets[0].field = 0; in.targets[0].value = 10.5;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
  in.targets[0].value = 20; in.targets[0].motor_id = 1;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
  in.targets[0].motor_id = UINT16_MAX; in.request_id = 0x40000000U;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
}

TEST(AgentMessages, NodeIdCannotBeMixedWithOtherDeviceParameters) {
  ConfigRequest in;
  in.kind = ConfigRequest::CONFIG_KIND_PARAM_SET; in.motor_id = UINT16_MAX;
  in.count = 2; in.request_id = 1;
  ParamSetTarget id; id.motor_id = 1; id.field = ParamSetTarget::FIELD_NODE_ID; id.value = 7;
  ParamSetTarget scale = id; scale.field = ParamSetTarget::FIELD_T_MAX; scale.value = 150;
  in.targets.push_back(id); in.targets.push_back(scale);
  AgentConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_agent_config(in, out).ok);
}
