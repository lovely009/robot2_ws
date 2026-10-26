// 消息级编解码：主机 facade 消息 ↔ 线上消息。
//
// 校验口径与固件旧契约逐字一致：**整条受理或整条拒绝**（任一目标不合法则一个
// 字节都不下发），命令路径的 NaN/Inf 同样整条拒收。
//
// 枚举取值沿用保留消息的常量（target_source / lifecycle / command），不在此重定义。
#ifndef MICRO_ROS_BRIDGE__MESSAGES_HPP_
#define MICRO_ROS_BRIDGE__MESSAGES_HPP_

#include <cstdint>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/low_command.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_array.hpp>
#include <can_bridge_msgs/msg/motor_status.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/param_set_array.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>

#include "micro_ros_bridge/codec.hpp"

namespace micro_ros_bridge
{
struct Result
{
  bool ok = true;
  const char * error = "";
  uint32_t saturated = 0;
};

struct MotorFreshness
{
  uint32_t state_age_ms = 0xFFFFFFFFu;
  uint8_t target_source = 0;
};

class LowStateAssembler
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  bool accept(const can_bridge_msgs::msg::LowState & in, uint64_t session,
    TimePoint now, can_bridge_msgs::msg::MotorMotionStateArray & out,
    uint32_t source_delay_ms = 0);
  void reset();

private:
  uint64_t session_ = 0;
  uint32_t last_seq_ = 0;
  bool have_last_ = false;
  uint32_t pending_seq_ = 0;
  uint32_t pending_tick_ = 0;
  uint64_t pending_sync_ = 0;
  uint8_t pending_total_ = 0;
  TimePoint started_{};
  std::vector<std::optional<can_bridge_msgs::msg::LowState>> chunks_;
  std::vector<TimePoint> received_;
  std::vector<uint32_t> source_delays_ms_;
};

Result encode_low_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::LowCommand & out);

Result decode_low_state(
  const can_bridge_msgs::msg::LowState & in,
  can_bridge_msgs::msg::MotorMotionStateArray & out);

Result encode_config(
  const can_bridge_msgs::msg::ParamSetArray & in,
  can_bridge_msgs::msg::ConfigRequest & out);

Result encode_config(
  const can_bridge_msgs::msg::SystemCommand & in,
  can_bridge_msgs::msg::ConfigRequest & out);

Result decode_motor_status(
  const can_bridge_msgs::msg::MotorStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorTrajectoryState & traj,
  can_bridge_msgs::msg::SystemStatus & sys,
  can_bridge_msgs::msg::MotorDiagnostics & diag,
  std::vector<MotorFreshness> & freshness);
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__MESSAGES_HPP_
