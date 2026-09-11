#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "motor_sim/core/frame_router.hpp"
#include "motor_sim/transport/frame_transport.hpp"
#include "motor_sim/transport/serial_transport.hpp"

namespace motor_sim::ros {

namespace transport = ::motor_sim::transport;
namespace protocol = ::motor_sim::protocol;
using ::motor_sim::core::FrameRouter;
using ::motor_sim::core::JointSample;
using ::motor_sim::core::SystemConfig;

struct BusPort {
  std::string bus;
  std::string device;
};

using PortMap = std::vector<BusPort>;
using TransportMap =
    std::map<std::string, std::unique_ptr<transport::FrameTransport>>;

struct NodeParameters {
  transport::TransportMode transport_mode{transport::TransportMode::UsbSerial};
  PortMap ports;
  int baud_rate{3000000};
  std::string config_file;
  bool allow_external_canfd{false};
};

struct NodeDefaults {
  std::string config_file;
  std::string joint_states_topic{"joint_states"};
  std::string effort_topic{"effort_controller/commands"};
  std::string diagnostics_topic{"diagnostics"};
  std::string drive_state_topic{"drive_state"};
  std::string control_gate_topic{"control_gate"};
  std::string reset_fault_service{"reset_fault"};
  // 仅调试用：false 时故障只记录不锁存，输出不清零；默认 true。
  bool fault_latching_enabled{true};
};

// 按外部拓扑契约校验参数：buses 顺序、数量与每台电机地址必须与契约一致。
void validate_node_parameters(
    const NodeParameters &parameters,
    const ::motor_sim::core::TopologyContract &contract);

// 仅按一组预期 bus 名校验参数：用于独立入口，总线集合来自配置文件而非外部契约。
void validate_node_parameters(
    const NodeParameters &parameters,
    const std::vector<std::string> &expected_buses);

class MotorSimNode final : public rclcpp::Node {
public:
  // 机器人集成层入口：按外部拓扑契约校验 YAML 与端口映射，保证与 URDF/关节清单一致。
  MotorSimNode(const rclcpp::NodeOptions &options,
               ::motor_sim::core::TopologyContract contract,
               NodeDefaults defaults);

  // 通用独立入口：不依赖外部拓扑契约，buses/motors 完全由 config_file 决定。
  // bus→endpoint 映射通过 `<bus_name>_port` 参数显式传入；不提供任何机器人默认值。
  explicit MotorSimNode(const rclcpp::NodeOptions &options,
                        NodeDefaults defaults = {});

  // 测试/集成入口：直接注入已构造的 config 与 transports。
  MotorSimNode(const rclcpp::NodeOptions &options, SystemConfig config,
               TransportMap transports, NodeDefaults defaults = {});

  bool shutdown_requested() const noexcept { return shutdown_requested_; }
  const std::vector<double> &last_efforts() const noexcept {
    return last_efforts_;
  }

private:
  void initialize(SystemConfig config, TransportMap transports,
                  const NodeDefaults &defaults);
  void joint_state_callback(const sensor_msgs::msg::JointState &message);
  void transport_tick();
  void control_tick();
  void send_feedback(std::size_t tick);
  void publish_efforts(const std::vector<double> &efforts);
  void publish_drive_state();
  void publish_diagnostic(int level, const std::string &message);
  void control_gate_callback(const std_msgs::msg::Bool &message);
  void reset_fault_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_disconnect(const std::string &bus);

  std::unique_ptr<FrameRouter> router_;
  TransportMap transports_;
  std::vector<JointSample> joint_states_;
  std::vector<double> last_efforts_;
  std::map<std::string, std::size_t> joint_indices_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drive_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      control_gate_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr transport_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  std::size_t control_tick_count_{0U};
  bool shutdown_requested_{false};
  bool control_gate_open_{false};
  std::string last_transition_reason_{"startup"};
  std::string reset_rejection_reason_;
};

}  // namespace motor_sim::ros
