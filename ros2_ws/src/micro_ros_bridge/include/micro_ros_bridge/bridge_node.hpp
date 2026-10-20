// 主机 facade 与线上契约的边界节点。
#ifndef MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
#define MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/low_command.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_event.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_array.hpp>
#include <can_bridge_msgs/msg/motor_policy_array.hpp>
#include <can_bridge_msgs/msg/motor_status.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_topology.hpp>
#include <can_bridge_msgs/msg/motor_trajectory.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/param_set_array.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include "micro_ros_bridge/messages.hpp"  // MotorFreshness

namespace micro_ros_bridge
{
class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode();

private:
  // 保留消息字段，仅更换话题名。
  template<typename T>
  void make_pass_through(
    const std::string & wire_topic, const std::string & host_topic, const rclcpp::QoS & qos);
  template<typename T>
  void make_host_to_wire(
    const std::string & host_topic, const std::string & wire_topic, const rclcpp::QoS & qos);

  // 主机侧 → 线上（需要编码的两条）
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr sub_command_;
  rclcpp::Subscription<can_bridge_msgs::msg::ParamSetArray>::SharedPtr sub_param_set_;
  rclcpp::Subscription<can_bridge_msgs::msg::SystemCommand>::SharedPtr sub_system_command_;
  rclcpp::Publisher<can_bridge_msgs::msg::LowCommand>::SharedPtr pub_low_cmd_;
  rclcpp::Publisher<can_bridge_msgs::msg::ConfigRequest>::SharedPtr pub_config_;

  // 线上 → 主机侧（需要解码的两条）
  rclcpp::Subscription<can_bridge_msgs::msg::LowState>::SharedPtr sub_low_state_;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorStatus>::SharedPtr sub_motor_status_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorMotionStateArray>::SharedPtr pub_motion_states_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorStatusArray>::SharedPtr pub_status_array_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTrajectoryState>::SharedPtr pub_traj_state_;
  rclcpp::Publisher<can_bridge_msgs::msg::SystemStatus>::SharedPtr pub_system_status_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorDiagnostics>::SharedPtr pub_diagnostics_;

  // 持有透传端点的生命周期。
  std::vector<rclcpp::PublisherBase::SharedPtr> pass_pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> pass_subs_;

  // 仅有效状态帧更新对端版本；未知状态放行命令。
  std::atomic<bool> peer_schema_known_{false};
  std::atomic<uint16_t> peer_schema_{0};

  // 按 motor_id 缓存低频新鲜度，供循环面输出使用。
  std::vector<MotorFreshness> freshness_ =
    std::vector<MotorFreshness>(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX);

  bool peer_schema_mismatch() const;

  void on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg);
  void on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg);
  void on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg);
};
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
