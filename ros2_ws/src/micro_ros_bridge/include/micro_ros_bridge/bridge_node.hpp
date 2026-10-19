// 线上契约与主机侧之间的**唯一边界节点**。
//
// 订阅（主机侧 → 线上）：/motor/command_array、/motor/param_set、
//   /motor/system_command、/motor/policy_array、/motor/trajectory
// 发布（线上 → 主机侧）：/motor/motion_states、/motor/status_array、
//   /motor/trajectory_state、/motor/system_status、/motor/diagnostics、
//   /motor/topology、/motor/events、/motor/device_param_report，IMU 两条透传。
//
// QoS 按消息性质选，不看业务域：高频流 best-effort depth 1，低频状态声明 reliable。
// 线上话题名一律取自 WireContract 常量——两端不可能拼错。
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
  // 纯改名透传：消息类型一个字段都不动，只把主机侧话题名与线上话题名对接。
  // 用类型擦除的容器持有，避免为每条透传写一对成员。
  template<typename T>
  void make_pass_through(
    const std::string & wire_topic, const std::string & host_topic, const rclcpp::QoS & qos);

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

  // 透传的生命周期持有者。订阅必须被持有，否则构造完就析构、话题静默消失。
  std::vector<rclcpp::PublisherBase::SharedPtr> pass_pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> pass_subs_;

  // 对端（固件）的 schema_version。0 = 尚未收到，**不据以拦截**（否则起不来）。
  // 非 0 且不等于本机版本时，拒绝一切下行命令（ADR 0015 决定 8）。
  std::atomic<uint16_t> peer_schema_{0};

  // 每台最近一次低频面给出的新鲜度，按 motor_id 索引。low_state 不带这两个量
  // （循环面不分片），但页面读的是 motion_states 元素上的它们（readout.js:34、
  // arm.js:270），所以在这里缓存、在 50 Hz 填充时盖章。构造期定长，之后只写不分配。
  std::vector<MotorFreshness> freshness_ =
    std::vector<MotorFreshness>(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX);

  bool peer_schema_mismatch() const;

  void on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg);
  void on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg);
  void on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg);
};
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
