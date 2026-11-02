#ifndef MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
#define MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_

#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <can_bridge_msgs/msg/agent_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <can_bridge_msgs/msg/agent_config_request.hpp>
#include <can_bridge_msgs/msg/agent_info.hpp>
#include <can_bridge_msgs/msg/agent_motion.hpp>
#include <can_bridge_msgs/msg/agent_status.hpp>
#include <can_bridge_msgs/msg/agent_trajectory_request.hpp>
#include <can_bridge_msgs/msg/config_result.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/motor_control_state_array.hpp>
#include <can_bridge_msgs/msg/motor_event.hpp>
#include <can_bridge_msgs/msg/motor_policy_array.hpp>
#include <can_bridge_msgs/msg/motor_topology.hpp>
#include <can_bridge_msgs/msg/motor_trajectory.hpp>
#include <can_bridge_msgs/srv/execute_config.hpp>
#include <can_bridge_msgs/srv/submit_policy.hpp>
#include "micro_ros_bridge/messages.hpp"

namespace micro_ros_bridge
{
class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  bool fill_authority(can_bridge_msgs::msg::AgentCommand & command, bool active);
  void send_authority(bool active);
  void stop_control();
  bool start_control();
  bool authority_ready(uint64_t session, uint64_t lease);
  uint64_t authority_high_water_ = 0, enable_after_ns_ = 0;
  uint32_t authority_sequence_ = 0;
  bool authority_revoked_ = true;
  bool authority_confirmed_ = false;
  std::chrono::steady_clock::time_point authority_started_{};
  std::chrono::steady_clock::time_point authority_sent_at_{};
  std::optional<can_bridge_msgs::msg::MotorPolicyArray> pending_policy_;
  using TimePoint = std::chrono::steady_clock::time_point;
  using Config = can_bridge_msgs::msg::AgentConfigRequest;
  using Status = can_bridge_msgs::msg::AgentStatus;
  using Trajectory = can_bridge_msgs::msg::AgentTrajectoryRequest;
  using Reply = can_bridge_msgs::msg::ConfigResult;
  using Header = std::shared_ptr<rmw_request_id_t>;
  struct ConfigJob {
    Config wire;
    can_bridge_msgs::msg::ConfigRequest original;
    Header header;
    uint32_t caller_id = 0;
    uint8_t kind = 0, command = 0;
    TimePoint deadline{}, sent_at{};
    bool sent = false, accepted = false, wait_expired = false;
    std::string intent_id;
  };
  std::optional<ConfigJob> pending_intent_;
  std::string policy_instance_;
  uint64_t last_intent_stamp_ = 0;
  uint32_t policy_generation_ = 0;
  struct FinishedConfig {
    can_bridge_msgs::msg::ConfigRequest original;
    Reply reply;
    Config wire;
  };
  struct Upload {
    uint32_t caller_id = 0, wire_id = 0;
    uint64_t issued = 0, lease_id = 0;
    uint16_t host_segments = 0, next_host_segment = 0;
    std::vector<uint16_t> ids;
    std::vector<can_bridge_msgs::msg::MotorTrajectoryPoint> points;
    size_t next_point = 0;
    uint16_t segment_count = 0;
    bool complete = false;
    std::optional<Trajectory> pending;
    TimePoint last_sent{}, began = std::chrono::steady_clock::now();
  };
  rclcpp::Publisher<can_bridge_msgs::msg::AgentCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<Config>::SharedPtr config_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorMotionStateArray>::SharedPtr motion_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorStatusArray>::SharedPtr status_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorControlStateArray>::SharedPtr control_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTopology>::SharedPtr topology_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::DeviceParamReport>::SharedPtr params_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTrajectoryState>::SharedPtr traj_state_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::SystemStatus>::SharedPtr system_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<Reply>::SharedPtr result_pub_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr command_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Service<can_bridge_msgs::srv::ExecuteConfig>::SharedPtr config_service_;
  rclcpp::Service<can_bridge_msgs::srv::SubmitPolicy>::SharedPtr policy_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t session_id_ = 0;
  uint32_t command_sequence_ = 0, next_config_id_ = 1, next_trajectory_request_ = 1;
  uint32_t next_trajectory_id_ = 1, query_id_ = 0, info_revision_ = 0;
  uint32_t event_sequence_ = 0;
  Status status_;
  bool have_status_ = false, have_info_ = false, querying_ = false;
  TimePoint status_at_{}, info_sent_at_{}, clock_at_{};
  int64_t clock_ns_ = 0;
  bool have_clock_ = false;
  Config query_;
  can_bridge_msgs::msg::MotorTopology topology_, info_topology_;
  can_bridge_msgs::msg::DeviceParamReport info_params_;
  can_bridge_msgs::msg::SystemStatus system_;
  uint32_t last_caller_ids_[3] = {};
  std::deque<ConfigJob> configs_, reports_;
  std::deque<FinishedConfig> finished_configs_;
  std::optional<Upload> upload_;
  uint32_t active_wire_trajectory_ = 0, active_caller_trajectory_ = 0;
  LowStateAssembler assembler_;
  std::vector<MotorFreshness> freshness_ = std::vector<MotorFreshness>(65);
  TimePoint motion_at_{};
  std::optional<can_bridge_msgs::msg::MotorCommandTarget> last_targets_[65];
  bool trajectory_targets_[65] = {};
  bool host_execution_ = false, host_takeover_pending_ = false, takeover_stop_sent_ = false;

  bool clock_stable();
  bool source_time_valid(uint64_t stamp, std::chrono::milliseconds age);
  bool connected();
  bool motion_ready();
  void clear_session(uint64_t session);
  void tick();
  void on_status(const Status & msg);
  void on_info(const can_bridge_msgs::msg::AgentInfo & msg);
  void query_info(bool restart = false);
  void on_motion(const can_bridge_msgs::msg::AgentMotion & msg);
  void on_command(const can_bridge_msgs::msg::MotorCommandArray & msg);
  void on_policy(const can_bridge_msgs::msg::MotorPolicyArray & msg);
  void submit_policy(const can_bridge_msgs::srv::SubmitPolicy::Request & request,
    can_bridge_msgs::srv::SubmitPolicy::Response & response);
  void cancel_policy_intents(uint64_t motor_mask = UINT64_MAX);
  void submit_config(const can_bridge_msgs::msg::ConfigRequest & request, Header header = {},
    uint32_t timeout_ms = 5000);
  void enqueue_config(ConfigJob job);
  void publish_reply(ConfigJob & job, uint8_t state, uint16_t error,
    uint64_t completed = 0, uint64_t failed = 0);
  void config_results(const Status & msg);
  void dispatch_config();
  void trajectory_event(uint32_t caller_id, uint16_t error);
  void on_trajectory(const can_bridge_msgs::msg::MotorTrajectory & msg);
  void dispatch_trajectory();
  void trajectory_receipt(const Status & msg);
};
}  // namespace micro_ros_bridge
#endif
