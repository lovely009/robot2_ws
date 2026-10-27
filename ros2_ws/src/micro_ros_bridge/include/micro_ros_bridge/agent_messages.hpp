// Agent V3 wire messages <-> retained host facade messages.
//
// These functions perform validation and field mapping only. Session allocation,
// request/query IDs, deadlines/TTLs and multi-page transaction state belong to the
// bridge node. Every function is atomic: an error leaves all output arguments
// untouched.
#ifndef MICRO_ROS_BRIDGE__AGENT_MESSAGES_HPP_
#define MICRO_ROS_BRIDGE__AGENT_MESSAGES_HPP_

#include <cstddef>

#include <can_bridge_msgs/msg/agent_command.hpp>
#include <can_bridge_msgs/msg/agent_config_request.hpp>
#include <can_bridge_msgs/msg/agent_info.hpp>
#include <can_bridge_msgs/msg/agent_motion.hpp>
#include <can_bridge_msgs/msg/agent_status.hpp>
#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_control_state_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_policy_array.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_topology.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>

#include "micro_ros_bridge/messages.hpp"

namespace micro_ros_bridge
{

Result encode_agent_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::AgentCommand & out);

Result encode_agent_config(
  const can_bridge_msgs::msg::ConfigRequest & in,
  can_bridge_msgs::msg::AgentConfigRequest & out);

Result encode_agent_policy(
  const can_bridge_msgs::msg::MotorPolicyArray & in,
  can_bridge_msgs::msg::AgentConfigRequest & out);

Result agent_motion_frame(
  const can_bridge_msgs::msg::AgentMotion & in,
  size_t total_count,
  can_bridge_msgs::msg::LowState & out);

Result decode_agent_status(
  const can_bridge_msgs::msg::AgentStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorControlStateArray & controls,
  can_bridge_msgs::msg::MotorTrajectoryState & trajectory,
  can_bridge_msgs::msg::SystemStatus & system,
  can_bridge_msgs::msg::MotorDiagnostics & diagnostics);

Result append_agent_info(
  const can_bridge_msgs::msg::AgentInfo & in,
  can_bridge_msgs::msg::MotorTopology & topology,
  can_bridge_msgs::msg::DeviceParamReport & report);

}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__AGENT_MESSAGES_HPP_
