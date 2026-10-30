// 主机 facade 与线上契约的边界节点。
#ifndef MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
#define MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_

#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/control_lease.hpp>
#include <can_bridge_msgs/msg/config_result.hpp>
#include <can_bridge_msgs/srv/execute_config.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/low_command.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_control_state_array.hpp>
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

  // 主机侧 → 线上（需要编码的两条）
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr sub_command_;
  rclcpp::Subscription<can_bridge_msgs::msg::ControlLease>::SharedPtr sub_control_lease_;
  rclcpp::Subscription<can_bridge_msgs::msg::ParamSetArray>::SharedPtr sub_param_set_;
  rclcpp::Subscription<can_bridge_msgs::msg::SystemCommand>::SharedPtr sub_system_command_;
  rclcpp::Publisher<can_bridge_msgs::msg::LowCommand>::SharedPtr pub_low_cmd_;
  rclcpp::Publisher<can_bridge_msgs::msg::ControlLease>::SharedPtr pub_control_lease_;
  rclcpp::Publisher<can_bridge_msgs::msg::ConfigRequest>::SharedPtr pub_config_;
  rclcpp::Service<can_bridge_msgs::srv::ExecuteConfig>::SharedPtr service_config_;
  rclcpp::TimerBase::SharedPtr config_timer_;

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

  using TimePoint = std::chrono::steady_clock::time_point;
  uint64_t session_id_ = 0;
  uint32_t status_sequence_ = 0;
  uint32_t status_tick_ms_ = 0;
  uint32_t command_sequence_ = 0;
  uint32_t next_request_id_ = 1;
  uint32_t last_request_id_ = 0;
  std::deque<can_bridge_msgs::msg::ConfigRequest> recent_requests_;
  struct PendingConfig
  {
    std::shared_ptr<rmw_request_id_t> header;
    uint64_t session_id;
    uint32_t wire_id;
    uint32_t caller_id;
    uint8_t kind;
    uint8_t command;
    TimePoint deadline;
  };
  std::vector<PendingConfig> pending_configs_;
  bool have_status_sequence_ = false;
  bool have_status_ = false;
  bool have_topology_ = false;
  TimePoint status_at_{};
  TimePoint topology_at_{};
  TimePoint clock_sample_at_{};
  int64_t clock_sample_ns_ = 0;
  bool have_clock_sample_ = false;
  bool have_control_ = false;
  bool have_control_sequence_ = false;
  uint32_t control_sequence_ = 0;
  uint32_t control_tick_ms_ = 0;
  TimePoint control_at_{};
  uint64_t lease_id_ = 0;
  uint32_t lease_sequence_ = 0;
  uint32_t lease_deadline_tick_ms_ = 0;
  bool lease_active_ = false;
  uint64_t sent_lease_id_ = 0;
  uint32_t sent_lease_sequence_ = 0;
  uint64_t revoked_lease_id_ = 0;
  LowStateAssembler assembler_;
  std::vector<MotorFreshness> freshness_ =
    std::vector<MotorFreshness>(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX);

  bool motion_ready();
  bool lease_ready(uint64_t session, uint64_t lease);
  std::optional<uint32_t> control_tick_now();
  void on_control_lease(const can_bridge_msgs::msg::ControlLease::SharedPtr msg);
  void on_control_state(const can_bridge_msgs::msg::MotorControlStateArray::SharedPtr msg,
    const rclcpp::Publisher<can_bridge_msgs::msg::MotorControlStateArray>::SharedPtr & pub);
  bool clock_stable();
  bool source_time_valid(uint64_t source_ns, std::chrono::milliseconds max_age);
  void clear_cache();
  void on_topology(const can_bridge_msgs::msg::MotorTopology::SharedPtr msg,
    const rclcpp::Publisher<can_bridge_msgs::msg::MotorTopology>::SharedPtr & pub);
  bool assign_request_id(can_bridge_msgs::msg::ConfigRequest & request);
  void on_config_result(const can_bridge_msgs::msg::ConfigResult::SharedPtr & result,
    const rclcpp::Publisher<can_bridge_msgs::msg::ConfigResult>::SharedPtr & pub);
  void finish_config(size_t index, const can_bridge_msgs::msg::ConfigResult & result);
  void expire_configs();

  void on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg);
  void on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg);
  void on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg);
};
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
