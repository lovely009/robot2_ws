#include "humanoid_gazebo/motor_sim/motor_sim_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <chrono>
#include <set>
#include <stdexcept>
#include <utility>

namespace humanoid_gazebo::motor_sim {
namespace {

constexpr const char *kLeftArm = "left_arm";
constexpr const char *kRightArm = "right_arm";
constexpr const char *kLegs = "legs";

transport::TransportMode parse_transport_mode(const std::string &value) {
  if (value == "usb_serial") {
    return transport::TransportMode::UsbSerial;
  }
  if (value == "serial_slcan") {
    return transport::TransportMode::SerialSlcan;
  }
  throw std::invalid_argument(
      "transport_mode must be usb_serial or serial_slcan");
}

std::string fault_message(const Fault fault) {
  switch (fault) {
    case Fault::None:
      return "ready";
    case Fault::CommandLimit:
      return "command limit fault";
    case Fault::CommandTimeout:
      return "command timeout";
    case Fault::TransportDisconnected:
      return "serial transport disconnected";
    case Fault::DuplicateController:
      return "duplicate controller detected";
    case Fault::WrongBus:
      return "frame received on wrong bus";
  }
  return "unknown fault";
}

}  // namespace

void validate_node_parameters(const NodeParameters &parameters) {
  if (parameters.config_file.empty()) {
    throw std::invalid_argument("config_file cannot be empty");
  }
  if (parameters.baud_rate <= 0) {
    throw std::invalid_argument("baud_rate must be positive");
  }
  if (parameters.transport_mode == transport::TransportMode::SerialSlcan &&
      !parameters.allow_external_canfd) {
    throw std::invalid_argument(
        "serial_slcan requires allow_external_canfd:=true");
  }
  const std::set<std::string> expected{kLeftArm, kRightArm, kLegs};
  std::set<std::string> buses;
  std::set<std::string> devices;
  for (const auto &[bus, device] : parameters.ports) {
    if (expected.find(bus) == expected.end() || !buses.insert(bus).second) {
      throw std::invalid_argument("ports must define each logical bus once");
    }
    constexpr const char *kStablePrefix = "/dev/serial/by-id/";
    if (device.rfind(kStablePrefix, 0U) != 0U ||
        device.size() <= std::char_traits<char>::length(kStablePrefix)) {
      throw std::invalid_argument(
          "serial ports must use nonempty /dev/serial/by-id paths");
    }
    if (!devices.insert(device).second) {
      throw std::invalid_argument("serial ports must be unique");
    }
  }
  if (buses != expected) {
    throw std::invalid_argument(
        "ports must contain left_arm, right_arm and legs");
  }
}

MotorSimNode::MotorSimNode() : MotorSimNode(rclcpp::NodeOptions{}) {}

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("humanoid_motor_sim", options) {
  const auto share =
      ament_index_cpp::get_package_share_directory("humanoid_gazebo");
  const auto mode_name =
      declare_parameter<std::string>("transport_mode", "usb_serial");
  NodeParameters parameters;
  parameters.transport_mode = parse_transport_mode(mode_name);
  parameters.ports = {
      {kLeftArm, declare_parameter<std::string>("left_arm_port", "")},
      {kRightArm, declare_parameter<std::string>("right_arm_port", "")},
      {kLegs, declare_parameter<std::string>("legs_port", "")},
  };
  parameters.baud_rate = declare_parameter<int>("baud_rate", 3000000);
  parameters.config_file = declare_parameter<std::string>(
      "config_file", share + "/config/motors.yaml");
  parameters.allow_external_canfd =
      declare_parameter<bool>("allow_external_canfd", false);
  validate_node_parameters(parameters);

  TransportMap transports;
  for (const auto &[bus, device] : parameters.ports) {
    transports.emplace(bus, std::make_unique<transport::SerialTransport>(
                                device, parameters.transport_mode,
                                parameters.baud_rate));
  }
  initialize(load_system_config(parameters.config_file), std::move(transports));
}

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options,
                           SystemConfig config, TransportMap transports)
    : rclcpp::Node("humanoid_motor_sim", options) {
  initialize(std::move(config), std::move(transports));
}

void MotorSimNode::initialize(SystemConfig config, TransportMap transports) {
  router_ = std::make_unique<FrameRouter>(std::move(config));
  transports_ = std::move(transports);
  if (transports_.size() != router_->config().buses.size()) {
    throw std::invalid_argument("transport count does not match configured buses");
  }
  for (const auto &bus : router_->config().buses) {
    const auto found = transports_.find(bus.name);
    if (found == transports_.end() || !found->second) {
      throw std::invalid_argument("missing transport for bus '" + bus.name + "'");
    }
    found->second->open();
  }

  joint_states_.resize(router_->config().motors.size());
  last_efforts_.assign(router_->config().motors.size(), 0.0);
  for (std::size_t index = 0U; index < router_->config().motors.size(); ++index) {
    const auto &motor = router_->config().motors[index];
    joint_indices_.emplace(motor.joint, index);
  }

  effort_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands", 10);
  diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "/humanoid_gazebo/diagnostics", 10);
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState &message) {
        joint_state_callback(message);
      });
  const auto period = std::chrono::duration<double>(
      1.0 / static_cast<double>(router_->config().timing.control_hz));
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this] { control_tick(); });
  transport_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                       [this] { transport_tick(); });
  publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     "three serial buses opened; motors await explicit enable");
}

void MotorSimNode::joint_state_callback(
    const sensor_msgs::msg::JointState &message) {
  for (std::size_t source = 0U; source < message.name.size(); ++source) {
    const auto target = joint_indices_.find(message.name[source]);
    if (target == joint_indices_.end()) {
      continue;
    }
    if (source < message.position.size()) {
      joint_states_[target->second].position_rad = message.position[source];
    }
    if (source < message.velocity.size()) {
      joint_states_[target->second].velocity_rad_s = message.velocity[source];
    }
  }
}

void MotorSimNode::transport_tick() {
  for (auto &[bus, endpoint] : transports_) {
    for (std::size_t received = 0U; received < 128U; ++received) {
      transport::ReceivedFrame received_frame;
      const auto result = endpoint->receive(received_frame);
      if (result == transport::ReceiveResult::Disconnected) {
        handle_disconnect(bus);
        return;
      }
      if (result == transport::ReceiveResult::NoData) {
        break;
      }
      // 使用独立 I/O 线程记录的到达时间，ROS executor 延迟后成批消费
      // 也不会压缩帧间隔，避免 2 ms duplicate-controller 误报。
      static_cast<void>(router_->handle(bus, received_frame.frame,
                                        received_frame.received_at));
    }
  }
}

void MotorSimNode::control_tick() {
  const auto now = FrameRouter::Clock::now();
  last_efforts_ = router_->compute_efforts(
      joint_states_,
      1.0 / static_cast<double>(router_->config().timing.control_hz), now);
  publish_efforts(last_efforts_);
  const auto safety = router_->safety_state();
  if (safety.latched) {
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                       fault_message(safety.fault));
  }
  send_feedback(control_tick_count_++);
}

void MotorSimNode::send_feedback(const std::size_t tick) {
  const auto &config = router_->config();
  const auto rp_divisor =
      std::max(1U, config.timing.control_hz / config.timing.rp_feedback_hz);
  const auto vaxor_divisor = std::max(
      1U, config.timing.control_hz / config.timing.vaxor_feedback_hz);
  for (std::size_t index = 0U; index < config.motors.size(); ++index) {
    const auto &motor = config.motors[index];
    const bool rp = motor.protocol == "rp";
    if ((rp && tick % rp_divisor != 0U) ||
        (!rp && tick % vaxor_divisor != 0U)) {
      continue;
    }
    const auto feedback =
        router_->motor_feedback(index, joint_states_[index], last_efforts_[index]);
    const auto kind = rp ? protocol::FeedbackKind::Reply
                         : protocol::FeedbackKind::Status;
    const auto frame =
        protocol::make_motor_protocol(motor.protocol)->encode_feedback(feedback,
                                                                       kind);
    const auto endpoint = transports_.find(motor.bus);
    if (endpoint == transports_.end() || !endpoint->second->send(frame)) {
      handle_disconnect(motor.bus);
      return;
    }
  }
}

void MotorSimNode::publish_efforts(const std::vector<double> &efforts) {
  std_msgs::msg::Float64MultiArray message;
  message.data = efforts;
  effort_publisher_->publish(message);
}

void MotorSimNode::publish_diagnostic(const int level,
                                      const std::string &message) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<uint8_t>(level);
  status.name = "humanoid_gazebo/motor_sim";
  status.hardware_id = "gazebo-serial-simulator";
  status.message = message;
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  diagnostic_publisher_->publish(array);
}

void MotorSimNode::handle_disconnect(const std::string &bus) {
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  router_->observe_disconnect(bus);
  last_efforts_.assign(router_->config().motors.size(), 0.0);
  publish_efforts(last_efforts_);
  publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                     "serial bus '" + bus + "' disconnected; shutting down");
  if (control_timer_) {
    control_timer_->cancel();
  }
  if (transport_timer_) {
    transport_timer_->cancel();
  }
  // Reliable publisher 的 ACK 证明零 effort 已进入订阅端中间件；再留一个短的
  // executor 窗口，让 controller 回调实际消费零值后才终止整个 launch。
  static_cast<void>(effort_publisher_->wait_for_all_acked(
      std::chrono::milliseconds(50)));
  shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
    shutdown_timer_->cancel();
    rclcpp::shutdown();
  });
}

}  // namespace humanoid_gazebo::motor_sim
