#include "micro_ros_bridge/agent_messages.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <can_bridge_msgs/msg/agent_config_result.hpp>
#include <can_bridge_msgs/msg/agent_device_info.hpp>
#include <can_bridge_msgs/msg/agent_trajectory_request.hpp>
#include <can_bridge_msgs/msg/agent_trajectory_state.hpp>
#include <can_bridge_msgs/msg/agent_wire_contract.hpp>
#include <can_bridge_msgs/msg/device_param_entry.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_target.hpp>
#include <can_bridge_msgs/msg/motor_policy_target.hpp>
#include <can_bridge_msgs/msg/motor_status_target.hpp>
#include <can_bridge_msgs/msg/motor_topology_entry.hpp>
#include <can_bridge_msgs/msg/param_set_array.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>

namespace micro_ros_bridge
{
namespace
{
using Config = can_bridge_msgs::msg::AgentConfigRequest;
using ConfigResult = can_bridge_msgs::msg::AgentConfigResult;
using TrajectoryState = can_bridge_msgs::msg::AgentTrajectoryState;
using OldLifecycle = can_bridge_msgs::msg::MotorStatusTarget;

constexpr uint16_t kAgentSchema = can_bridge_msgs::msg::AgentWireContract::SCHEMA_VERSION;
constexpr uint16_t kLegacySchema = can_bridge_msgs::msg::WireContract::SCHEMA_VERSION;
constexpr size_t kMotorIdMax = can_bridge_msgs::msg::AgentWireContract::MOTOR_ID_MAX;
constexpr size_t kAgentMotorMax = can_bridge_msgs::msg::AgentWireContract::MOTOR_COUNT_MAX;
constexpr size_t kMotionChunkMax = can_bridge_msgs::msg::AgentWireContract::MOTION_CHUNK_MAX;
constexpr size_t kInfoPageMax = can_bridge_msgs::msg::AgentWireContract::INFO_PAGE_MAX;
constexpr size_t kConfigResultMax = can_bridge_msgs::msg::AgentWireContract::CONFIG_RESULTS_MAX;
constexpr size_t kParamTargetMax = can_bridge_msgs::msg::AgentWireContract::PARAM_TARGETS_MAX;
constexpr uint32_t kRequestIdMax = can_bridge_msgs::msg::AgentWireContract::REQUEST_ID_MAX;

Result reject(Result result, const char * error)
{
  result.ok = false;
  result.error = error;
  return result;
}

bool valid_motor_id(size_t id)
{
  return id > 0 && id <= kMotorIdMax;
}

bool valid_agent_lifecycle(uint8_t lifecycle)
{
  return lifecycle == 0 || lifecycle == 1 || lifecycle == 2 || lifecycle == 3 ||
         lifecycle == 4 || lifecycle == 10 || lifecycle == 13 || lifecycle == 14 ||
         lifecycle == 15;
}

bool valid_config_operation(uint8_t operation)
{
  return operation <= Config::SET_FIRMWARE_PARAMETERS;
}

bool valid_config_result(const ConfigResult & result)
{
  if (!result.request_id || result.request_id > kRequestIdMax ||
    !valid_config_operation(result.operation) || result.state > ConfigResult::RETIRED) {
    return false;
  }
  return true;
}

uint8_t legacy_lifecycle(const can_bridge_msgs::msg::AgentMotorStatus & motor)
{
  if (!motor.online) {
    return OldLifecycle::LIFECYCLE_OFFLINE;
  }
  if (motor.unified_fault_code != 0 || motor.lifecycle == 15) {
    return OldLifecycle::LIFECYCLE_FAULT;
  }
  switch (motor.lifecycle) {
    case 0:
    case 4:
      return OldLifecycle::LIFECYCLE_ABSENT;
    case 1:
      return OldLifecycle::LIFECYCLE_DISCOVERED;
    case 2:
    case 13:
      return OldLifecycle::LIFECYCLE_READY_DISABLED;
    case 3:
      return motor.execution_state ==
             can_bridge_msgs::msg::MotorControlState::EXECUTION_SOFT_STOP ?
             OldLifecycle::LIFECYCLE_SAFE_STOP : OldLifecycle::LIFECYCLE_ACTIVE;
    case 10:
      return OldLifecycle::LIFECYCLE_IDENTIFYING;
    case 14:
      return OldLifecycle::LIFECYCLE_SAFE_STOP;
    default:
      return OldLifecycle::LIFECYCLE_FAULT;
  }
}

}  // namespace

Result encode_agent_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::AgentCommand & out)
{
  Result result;
  if (in.targets.size() != in.count || in.count > kAgentMotorMax) {
    return reject(result, "command count 与 targets 长度不一致或超过 48");
  }
  bool seen[kMotorIdMax + 1] = {};
  for (const auto & target : in.targets) {
    if (!valid_motor_id(target.motor_id) || seen[target.motor_id]) {
      return reject(result, "command motor_id 必须在 1..64 且唯一");
    }
    if (!std::isfinite(target.position) || !std::isfinite(target.velocity) ||
      !std::isfinite(target.effort)) {
      return reject(result, "command 含 NaN/Inf");
    }
    seen[target.motor_id] = true;
  }

  can_bridge_msgs::msg::LowCommand legacy;
  result = encode_low_command(in, legacy);
  if (!result.ok) {
    return result;
  }
  can_bridge_msgs::msg::AgentCommand encoded;
  encoded.schema_version = kAgentSchema;
  encoded.mode = can_bridge_msgs::msg::AgentCommand::SNAPSHOT;
  encoded.sequence = legacy.sequence;
  encoded.count = legacy.count;
  encoded.motor_ids.assign(legacy.motor_ids.begin(), legacy.motor_ids.end());
  encoded.position_q.assign(legacy.position_q.begin(), legacy.position_q.end());
  encoded.velocity_q.assign(legacy.velocity_q.begin(), legacy.velocity_q.end());
  encoded.effort_q.assign(legacy.effort_q.begin(), legacy.effort_q.end());
  out = std::move(encoded);
  return result;
}

Result encode_agent_config(
  const can_bridge_msgs::msg::ConfigRequest & in,
  can_bridge_msgs::msg::AgentConfigRequest & out)
{
  using OldConfig = can_bridge_msgs::msg::ConfigRequest;
  using OldSystem = can_bridge_msgs::msg::SystemCommand;
  Result result;
  if (in.kind != OldConfig::CONFIG_KIND_SYSTEM_COMMAND || in.count || !in.targets.empty()) {
    return reject(result, "未知 ConfigRequest kind");
  }
  OldSystem legacy;
  legacy.sync_time_ns = in.sync_time_ns;
  legacy.request_id = in.request_id;
  legacy.command = in.command;
  legacy.motor_id = in.motor_id;
  OldConfig checked;
  result = encode_config(legacy, checked);
  if (!result.ok) {
    return result;
  }
  if (legacy.command != OldSystem::COMMAND_REPORT && !valid_motor_id(legacy.motor_id)) {
    return reject(result, "system command motor_id 必须在 1..64");
  }

  Config encoded;
  encoded.schema_version = kAgentSchema;
  encoded.session_id = in.session_id;
  encoded.issued_stamp = in.sync_time_ns;
  encoded.request_id = in.request_id;
  switch (legacy.command) {
    case OldSystem::COMMAND_REFRESH:
      encoded.operation = Config::REFRESH_PARAMETERS;
      encoded.motor_id = legacy.motor_id;
      break;
    case OldSystem::COMMAND_CLEAR_FAULT:
      encoded.operation = Config::CLEAR_FAULT;
      encoded.motor_id = legacy.motor_id;
      break;
    case OldSystem::COMMAND_SET_ZERO:
      encoded.operation = Config::SET_ZERO;
      encoded.motor_id = legacy.motor_id;
      break;
    case OldSystem::COMMAND_REPORT:
      encoded.operation = Config::QUERY_INFO;
      encoded.motor_id = OldSystem::MOTOR_ID_NONE;
      break;
    default:
      return reject(result, "未知系统命令码");
  }
  out = std::move(encoded);
  return result;
}

Result encode_agent_policy(
  const can_bridge_msgs::msg::MotorPolicyArray & in,
  can_bridge_msgs::msg::AgentConfigRequest & out)
{
  Result result;
  if (!in.count || in.count > kAgentMotorMax || in.targets.size() != in.count) {
    return reject(result, "policy 必须非空、<=48 且 count 与 targets 一致");
  }
  bool seen[kMotorIdMax + 1] = {};
  bool all_disabled = true;
  bool any_unprotected = false;
  for (const auto & target : in.targets) {
    if (!valid_motor_id(target.motor_id) || seen[target.motor_id]) {
      return reject(result, "policy motor_id 必须在 1..64 且唯一");
    }
    if (!std::isfinite(target.kp) || !std::isfinite(target.kd) ||
      target.kp < 0.0f || target.kd < 0.0f) {
      return reject(result, "policy 增益必须有限且 >= 0");
    }
    seen[target.motor_id] = true;
    all_disabled = all_disabled && !target.enabled;
    any_unprotected = any_unprotected || (target.enabled && target.confirm_unprotected);
  }

  Config encoded;
  encoded.schema_version = kAgentSchema;
  encoded.session_id = in.session_id;
  encoded.issued_stamp = in.sync_time_ns;
  encoded.request_id = in.request_id;
  encoded.policy_generation = in.policy_generation;
  encoded.operation = Config::SET_POLICY;
  encoded.allow_no_protect = !all_disabled && any_unprotected;
  encoded.policies.assign(in.targets.begin(), in.targets.end());
  if (all_disabled) {
    for (auto & target : encoded.policies) {
      target.kp = 0.0f;
      target.kd = 0.0f;
      target.confirm_unprotected = false;
    }
    encoded.allow_no_protect = false;
  }
  out = std::move(encoded);
  return result;
}

Result agent_motion_frame(
  const can_bridge_msgs::msg::AgentMotion & in,
  size_t total_count,
  can_bridge_msgs::msg::LowState & out)
{
  using Source = can_bridge_msgs::msg::MotorMotionStateTarget;
  Result result;
  if (in.schema_version != kAgentSchema || !in.session_id || total_count > kAgentMotorMax) {
    return reject(result, "motion schema/session/total_count 非法");
  }
  const size_t expected_groups = total_count == 0 ? 1 :
    (total_count + kMotionChunkMax - 1) / kMotionChunkMax;
  if (in.group_count != expected_groups || in.group_count == 0 ||
    in.group_count > 2 || in.group_index >= in.group_count) {
    return reject(result, "motion group_count/index 与 total_count 不一致");
  }
  const size_t begin = static_cast<size_t>(in.group_index) * kMotionChunkMax;
  const size_t expected_count = total_count == 0 ? 0 :
    std::min(kMotionChunkMax, total_count - begin);
  const size_t n = in.motor_ids.size();
  if (in.count != expected_count || in.position_q.size() != n || in.velocity_q.size() != n ||
    in.effort_q.size() != n || in.state_age_ms.size() != n || in.target_source.size() != n ||
    n != expected_count || n > kMotionChunkMax) {
    return reject(result, "motion count/组长度不一致");
  }
  bool seen[kMotorIdMax + 1] = {};
  for (size_t i = 0; i < n; ++i) {
    if (!valid_motor_id(in.motor_ids[i]) || seen[in.motor_ids[i]] ||
      in.target_source[i] > Source::TARGET_SOURCE_TRAJECTORY) {
      return reject(result, "motion motor_id/target_source 非法或重复");
    }
    seen[in.motor_ids[i]] = true;
  }

  can_bridge_msgs::msg::LowState frame;
  frame.sync_time_ns = in.snapshot_stamp;
  frame.session_id = in.session_id;
  frame.seq = in.sequence;
  frame.tick_ms = in.snapshot_tick_ms;
  frame.schema_version = kLegacySchema;
  frame.chunk_index = in.group_index;
  frame.chunk_count = in.group_count;
  frame.total_count = static_cast<uint8_t>(total_count);
  frame.count = in.count;
  frame.motor_ids = in.motor_ids;
  frame.position_q = in.position_q;
  frame.velocity_q = in.velocity_q;
  frame.effort_q = in.effort_q;
  frame.state_age_ms = in.state_age_ms;
  frame.target_source = in.target_source;
  out = std::move(frame);
  return result;
}

Result decode_agent_status(
  const can_bridge_msgs::msg::AgentStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorControlStateArray & controls,
  can_bridge_msgs::msg::MotorTrajectoryState & trajectory,
  can_bridge_msgs::msg::SystemStatus & system,
  can_bridge_msgs::msg::MotorDiagnostics & diagnostics)
{
  using Control = can_bridge_msgs::msg::MotorControlState;
  using OldStatus = can_bridge_msgs::msg::MotorStatusTarget;
  Result result;
  if (in.schema_version != kAgentSchema || !in.session_id ||
    in.motors.size() > kAgentMotorMax || in.results.size() > kConfigResultMax) {
    return reject(result, "status schema/session/数组上界非法");
  }

  bool seen_ids[kMotorIdMax + 1] = {};
  for (const auto & motor : in.motors) {
    if (!valid_motor_id(motor.motor_id) || seen_ids[motor.motor_id] ||
      !valid_agent_lifecycle(motor.lifecycle) ||
      motor.execution_state > Control::EXECUTION_DISABLING || motor.bus_index > 2 ||
      (motor.status_flags & ~0x1fu) != 0 ||
      static_cast<bool>(motor.status_flags & 0x01u) != motor.online ||
      ((motor.status_flags & 0x04u) && !(motor.status_flags & 0x02u)) ||
      ((motor.status_flags & 0x10u) && !(motor.status_flags & 0x08u))) {
      return reject(result, "status motor_id/lifecycle/execution/flags 非法");
    }
    seen_ids[motor.motor_id] = true;
  }

  for (const auto & receipt : in.results) {
    if (receipt.request_id && !valid_config_result(receipt)) {
      return reject(result, "config result request/operation/state 非法");
    }
  }
  for (size_t i = 0; i < in.results.size(); ++i) {
    if (in.results[i].request_id) {
      for (size_t j = i + 1; j < in.results.size(); ++j) {
        if (in.results[i].request_id == in.results[j].request_id) {
          return reject(result, "config result request_id 重复");
        }
      }
    }
  }
  if (in.rejection_receipt.state > ConfigResult::RETIRED) {
    return reject(result, "rejection_receipt 不完整");
  }

  const auto & wire_receipt = in.trajectory_request_receipt;
  const bool receipt_zero = wire_receipt.request_id == 0 && wire_receipt.trajectory_id == 0 &&
    wire_receipt.operation == 0 && !wire_receipt.accepted && wire_receipt.error == 0 &&
    wire_receipt.received_segments == 0;
  if (!receipt_zero && wire_receipt.accepted && (!wire_receipt.request_id || wire_receipt.request_id > kRequestIdMax ||
    !wire_receipt.trajectory_id ||
    wire_receipt.operation > can_bridge_msgs::msg::AgentTrajectoryRequest::ACK ||
    wire_receipt.received_segments > can_bridge_msgs::msg::AgentWireContract::TRAJECTORY_POINTS_MAX)) {
    return reject(result, "trajectory request receipt 非法");
  }

  const auto & wire_trajectory = in.trajectory;
  if (wire_trajectory.state > TrajectoryState::ABORTED ||
    wire_trajectory.target_ids.size() > kAgentMotorMax ||
    wire_trajectory.received_segments > wire_trajectory.total_segments ||
    wire_trajectory.total_segments > can_bridge_msgs::msg::AgentWireContract::TRAJECTORY_POINTS_MAX ||
    (wire_trajectory.state != TrajectoryState::IDLE && !wire_trajectory.trajectory_id)) {
    return reject(result, "trajectory state/segments 非法");
  }
  bool seen_targets[kMotorIdMax + 1] = {};
  for (const auto id : wire_trajectory.target_ids) {
    if (!valid_motor_id(id) || seen_targets[id]) {
      return reject(result, "trajectory target_id 非法或重复");
    }
    seen_targets[id] = true;
  }
  if (in.diagnostics.ring_drop.size() > 3 || in.diagnostics.rx_hw_lost.size() > 3 ||
    in.diagnostics.tx_frame_count.size() > 3) {
    return reject(result, "diagnostics 总线数组超过 3");
  }

  can_bridge_msgs::msg::MotorStatusArray decoded_statuses;
  decoded_statuses.sync_time_ns = in.snapshot_stamp;
  can_bridge_msgs::msg::MotorControlStateArray decoded_controls;
  decoded_controls.schema_version = can_bridge_msgs::msg::MotorControlStateArray::SCHEMA_VERSION;
  decoded_controls.lease_id = in.lease_id;
  decoded_controls.lease_sequence = in.lease_sequence;
  decoded_controls.lease_valid_until_tick_ms = in.lease_valid_until_tick_ms;
  decoded_controls.lease_active = in.lease_active;
  decoded_controls.sync_time_ns = in.snapshot_stamp;
  decoded_controls.session_id = in.session_id;
  decoded_controls.sequence = in.sequence;
  decoded_controls.tick_ms = in.snapshot_tick_ms;

  for (const auto & motor : in.motors) {
    OldStatus status;
    status.motor_id = motor.motor_id;
    status.temperature = motor.temperature_dc == std::numeric_limits<int16_t>::min() ?
      std::numeric_limits<float>::quiet_NaN() : static_cast<float>(motor.temperature_dc) / 10.0f;
    status.current = motor.current_ma == std::numeric_limits<int32_t>::min() ?
      std::numeric_limits<float>::quiet_NaN() : static_cast<float>(motor.current_ma) / 1000.0f;
    status.voltage = motor.voltage_mv == std::numeric_limits<uint32_t>::max() ?
      std::numeric_limits<float>::quiet_NaN() : static_cast<float>(motor.voltage_mv) / 1000.0f;
    status.drive_mode = motor.drive_mode;
    status.unified_fault_code = motor.unified_fault_code;
    status.vendor_fault_code = motor.vendor_fault_code;
    status.protection_flags = motor.protection_flags;
    status.limit_flags = motor.limit_flags;
    status.device_protect_known = (motor.status_flags & 0x02u) != 0;
    status.device_protect_enabled = (motor.status_flags & 0x04u) != 0;
    status.device_direction_known = (motor.status_flags & 0x08u) != 0;
    status.device_direction = (motor.status_flags & 0x10u) ? 1u : 0u;
    status.online = motor.online;
    status.lifecycle = legacy_lifecycle(motor);
    status.bus_index = motor.bus_index;
    decoded_statuses.statuses.push_back(status);

    Control control;
    control.motor_id = motor.motor_id;
    control.online = motor.online;
    control.control_lifecycle = motor.lifecycle;
    control.execution_state = motor.execution_state;
    control.policy_enabled = motor.policy_enabled;
    control.motion_enabled = motor.kernel_enabled;
    control.state_age_ms = std::numeric_limits<uint32_t>::max();
    control.unified_fault_code = motor.unified_fault_code;
    control.vendor_fault_code = motor.vendor_fault_code;
    control.last_failure_reason = motor.unified_fault_code;
    control.policy_generation = motor.policy_generation;
    decoded_controls.states.push_back(control);
  }

  can_bridge_msgs::msg::MotorTrajectoryState decoded_trajectory;
  decoded_trajectory.sync_time_ns = in.snapshot_stamp;
  decoded_trajectory.trajectory_id = wire_trajectory.trajectory_id;
  decoded_trajectory.active = wire_trajectory.state == TrajectoryState::RUNNING;
  decoded_trajectory.completed = wire_trajectory.state == TrajectoryState::SUCCEEDED;
  decoded_trajectory.elapsed_ms = wire_trajectory.elapsed_ms;
  decoded_trajectory.total_ms = wire_trajectory.total_ms;
  decoded_trajectory.target_count = static_cast<uint16_t>(wire_trajectory.target_ids.size());
  decoded_trajectory.target_ids.assign(
    wire_trajectory.target_ids.begin(), wire_trajectory.target_ids.end());

  auto decoded_system = system;
  decoded_system.sync_time_ns = in.snapshot_stamp;
  decoded_system.last_error = in.last_error;
  decoded_system.last_error_motor = in.last_error_motor;
  decoded_system.save_in_progress = in.save_in_progress;
  decoded_system.firmware_values_valid = in.firmware_values_valid;
  decoded_system.firmware_values = in.firmware_values;
  decoded_system.firmware_unsaved = in.firmware_unsaved;

  statuses = std::move(decoded_statuses);
  controls = std::move(decoded_controls);
  trajectory = std::move(decoded_trajectory);
  system = std::move(decoded_system);
  diagnostics = in.diagnostics;
  return result;
}

Result append_agent_info(
  const can_bridge_msgs::msg::AgentInfo & in,
  can_bridge_msgs::msg::MotorTopology & topology,
  can_bridge_msgs::msg::DeviceParamReport & report)
{
  using TopologyEntry = can_bridge_msgs::msg::MotorTopologyEntry;
  Result result;
  if (in.schema_version != kAgentSchema || !in.session_id ||
    in.kind != can_bridge_msgs::msg::AgentInfo::DEVICE_PAGE ||
    in.result != can_bridge_msgs::msg::AgentInfo::OK || in.devices.size() > kInfoPageMax ||
    !in.motor_capacity || in.motor_capacity > kAgentMotorMax ||
    in.cursor > in.motor_capacity || in.next_cursor > in.motor_capacity ||
    in.next_cursor != in.cursor + in.devices.size() ||
    (!in.complete && in.devices.empty()) ||
    report.count != report.entries.size() || topology.entries.size() > kMotorIdMax ||
    report.entries.size() > kMotorIdMax) {
    return reject(result, "info kind/result/cursor/capacity 非法");
  }

  bool page_seen[kMotorIdMax + 1] = {};
  bool existing[kMotorIdMax + 1] = {};
  for (const auto & entry : topology.entries) {
    if (!valid_motor_id(entry.motor_id) || existing[entry.motor_id]) {
      return reject(result, "已有 topology motor_id 非法或重复");
    }
    existing[entry.motor_id] = true;
  }
  bool report_seen[kMotorIdMax + 1] = {};
  for (const auto & entry : report.entries) {
    if (!valid_motor_id(entry.motor_id) || report_seen[entry.motor_id]) {
      return reject(result, "已有 report motor_id 非法或重复");
    }
    report_seen[entry.motor_id] = true;
    existing[entry.motor_id] = true;
  }
  for (const auto & device : in.devices) {
    if (!valid_motor_id(device.motor_id) || page_seen[device.motor_id] ||
      existing[device.motor_id] || device.parameters.motor_id != device.motor_id ||
      device.bus_index > 2 || device.protocol > TopologyEntry::PROTOCOL_CERB ||
      device.model_name.size() > 31) {
      return reject(result, "info 页内/跨页 motor_id 重复或字段非法");
    }
    page_seen[device.motor_id] = true;
  }

  auto topology_out = topology;
  auto report_out = report;
  topology_out.schema_version = kLegacySchema;
  topology_out.sync_time_ns = in.snapshot_stamp;
  topology_out.session_id = in.session_id;
  report_out.sync_time_ns = in.snapshot_stamp;
  report_out.session_id = in.session_id;
  for (const auto & device : in.devices) {
    TopologyEntry topology_entry;
    topology_entry.motor_id = device.motor_id;
    topology_entry.bus_index = device.bus_index;
    topology_entry.protocol = device.protocol;
    topology_entry.model_name = device.model_name;
    topology_entry.capability_flags = device.capability_flags;
    topology_entry.online = device.online;
    topology_entry.conflict = device.conflict;
    topology_out.entries.push_back(topology_entry);
    report_out.entries.push_back(device.parameters);
  }
  report_out.count = static_cast<uint16_t>(report_out.entries.size());
  topology = std::move(topology_out);
  report = std::move(report_out);
  return result;
}

}  // namespace micro_ros_bridge
