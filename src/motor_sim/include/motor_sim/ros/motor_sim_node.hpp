#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
};

void validate_node_parameters(
    const NodeParameters &parameters,
    const ::motor_sim::core::TopologyContract &contract);

class MotorSimNode final : public rclcpp::Node {
public:
  MotorSimNode(const rclcpp::NodeOptions &options,
               ::motor_sim::core::TopologyContract contract,
               NodeDefaults defaults);
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
  void publish_diagnostic(int level, const std::string &message);
  void handle_disconnect(const std::string &bus);

  std::unique_ptr<FrameRouter> router_;
  TransportMap transports_;
  std::vector<JointSample> joint_states_;
  std::vector<double> last_efforts_;
  std::map<std::string, std::size_t> joint_indices_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr transport_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  std::size_t control_tick_count_{0U};
  bool shutdown_requested_{false};
};

}  // namespace motor_sim::ros
