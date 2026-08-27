#include "motor_sim/ros/motor_sim_node.hpp"
#include "motor_sim/transport/socket_can_transport.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>

namespace motor_sim::ros {
using ::motor_sim::core::Fault;
namespace {

transport::TransportMode parse_transport_mode(const std::string &value) {
  if (value == "usb_serial") {
    return transport::TransportMode::UsbSerial;
  }
  if (value == "serial_slcan") {
    return transport::TransportMode::SerialSlcan;
  }
  if (value == "socket_can") {
    return transport::TransportMode::SocketCan;
  }
  throw std::invalid_argument(
      "transport_mode must be usb_serial, serial_slcan or socket_can");
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

diagnostic_msgs::msg::KeyValue diagnostic_value(const std::string &key,
                                                const std::string &value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

void validate_node_parameters(
    const NodeParameters &parameters,
    const ::motor_sim::core::TopologyContract &contract) {
  if (parameters.config_file.empty()) {
    throw std::invalid_argument("config_file cannot be empty");
  }
  if (parameters.transport_mode == transport::TransportMode::SerialSlcan &&
      !parameters.allow_external_canfd) {
    throw std::invalid_argument(
        "serial_slcan requires allow_external_canfd:=true");
  }
  if (parameters.transport_mode != transport::TransportMode::SocketCan) {
    const std::set<int> supported_baud_rates{
        115200, 460800, 921600, 1000000, 2000000, 3000000};
    if (supported_baud_rates.find(parameters.baud_rate) ==
        supported_baud_rates.end()) {
      throw std::invalid_argument("unsupported baud_rate");
    }
  }
  const std::set<std::string> expected(contract.buses.begin(),
                                       contract.buses.end());
  std::set<std::string> buses;
  std::set<std::string> endpoints;
  for (const auto &[bus, endpoint] : parameters.ports) {
    if (expected.find(bus) == expected.end() || !buses.insert(bus).second) {
      throw std::invalid_argument("ports must define each logical bus once");
    }
    if (parameters.transport_mode == transport::TransportMode::SocketCan) {
      // SocketCAN 端口是接口名（如 vcan0），不是设备路径；必须已存在
      // （ip link 可见）且各逻辑总线互不重复。
      if (endpoint.empty() || endpoint.find('/') != std::string::npos) {
        throw std::invalid_argument(
            "socketcan port must be an interface name without '/'");
      }
      if (::if_nametoindex(endpoint.c_str()) == 0U) {
        throw std::invalid_argument("socketcan interface must exist");
      }
      if (!endpoints.insert(endpoint).second) {
        throw std::invalid_argument("socketcan interfaces must be unique");
      }
      continue;
    }
    if (endpoint.empty() || !std::filesystem::exists(endpoint)) {
      throw std::invalid_argument("serial port must exist");
    }
    const int descriptor =
        ::open(endpoint.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0 || ::isatty(descriptor) != 1) {
      if (descriptor >= 0) {
        ::close(descriptor);
      }
      throw std::invalid_argument("serial port must open as a tty");
    }
    ::close(descriptor);
    const auto resolved = std::filesystem::canonical(endpoint).string();
    if (!endpoints.insert(resolved).second) {
      throw std::invalid_argument("serial ports must be unique");
    }
  }
  if (buses != expected) {
    throw std::invalid_argument("ports must match the topology contract");
  }
}

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options,
                           ::motor_sim::core::TopologyContract contract,
                           NodeDefaults defaults)
    : rclcpp::Node("motor_sim", options) {
  const auto mode_name =
      declare_parameter<std::string>("transport_mode", "usb_serial");
  NodeParameters parameters;
  parameters.transport_mode = parse_transport_mode(mode_name);
  for (const auto &bus : contract.buses) {
    parameters.ports.push_back(
        {bus, declare_parameter<std::string>(bus + "_port", "")});
  }
  parameters.baud_rate = declare_parameter<int>("baud_rate", 3000000);
  parameters.config_file = declare_parameter<std::string>(
      "config_file", defaults.config_file);
  parameters.allow_external_canfd =
      declare_parameter<bool>("allow_external_canfd", false);
  // 调试开关：false 时故障只记录不锁存；默认 true 保持安全语义。
  defaults.fault_latching_enabled =
      declare_parameter<bool>("fault_latching_enabled",
                              defaults.fault_latching_enabled);
  defaults.drive_state_topic = declare_parameter<std::string>(
      "drive_state_topic", defaults.drive_state_topic);
  defaults.control_gate_topic = declare_parameter<std::string>(
      "control_gate_topic", defaults.control_gate_topic);
  defaults.reset_fault_service = declare_parameter<std::string>(
      "reset_fault_service", defaults.reset_fault_service);
  validate_node_parameters(parameters, contract);

  TransportMap transports;
  for (const auto &[bus, device] : parameters.ports) {
    if (parameters.transport_mode == transport::TransportMode::SocketCan) {
      transports.emplace(
          bus, std::make_unique<transport::SocketCanTransport>(device));
    } else {
      transports.emplace(bus, std::make_unique<transport::SerialTransport>(
                                  device, parameters.transport_mode,
                                  parameters.baud_rate));
    }
  }
  initialize(::motor_sim::core::load_system_config(parameters.config_file,
                                                    contract),
             std::move(transports), defaults);
  for (const auto &[bus, device] : parameters.ports) {
    if (parameters.transport_mode == transport::TransportMode::SocketCan) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                         "socketcan bus '" + bus + "' bound to '" + device +
                             "'");
    } else {
      publish_diagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "serial bus '" + bus + "' requested '" + device + "', resolved '" +
              std::filesystem::canonical(device).string() + "'");
    }
  }
}

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options, SystemConfig config,
                           TransportMap transports, NodeDefaults defaults)
    : rclcpp::Node("motor_sim", options) {
  initialize(std::move(config), std::move(transports), defaults);
}

void MotorSimNode::initialize(SystemConfig config, TransportMap transports,
                              const NodeDefaults &defaults) {
  router_ = std::make_unique<FrameRouter>(std::move(config),
                                          defaults.fault_latching_enabled);
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
      defaults.effort_topic, 10);
  drive_state_publisher_ = create_publisher<std_msgs::msg::String>(
      defaults.drive_state_topic, 10);
  diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          defaults.diagnostics_topic, 10);
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      defaults.joint_states_topic, 10,
      [this](const sensor_msgs::msg::JointState &message) {
        joint_state_callback(message);
      });
  control_gate_subscription_ = create_subscription<std_msgs::msg::Bool>(
      defaults.control_gate_topic, 10,
      [this](const std_msgs::msg::Bool &message) {
        control_gate_callback(message);
      });
  reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      defaults.reset_fault_service,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reset_fault_callback(request, std::move(response));
      });
  const auto period = std::chrono::duration<double>(
      1.0 / static_cast<double>(router_->config().timing.control_hz));
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this] { control_tick(); });
  transport_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                       [this] { transport_tick(); });
  publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     defaults.fault_latching_enabled
                         ? "serial buses opened; motors await explicit enable"
                         : "serial buses opened; motors await explicit enable; "
                           "DEBUG: fault latching disabled");
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
      const auto router_result = router_->handle(bus, received_frame.frame,
                                                 received_frame.received_at);
      if (router_result.status == core::RouterStatus::Applied) {
        // 手册 3.4 一问一答：Vaxor 每收到一帧 MIT 命令立即回一帧 Reply。
        for (const auto index : router_result.reply_motor_indices) {
          const auto feedback = router_->motor_feedback(
              index, joint_states_[index], last_efforts_[index]);
          const auto frame = protocol::make_motor_protocol(
                                 router_->config().motors[index].protocol)
                                 ->encode_feedback(feedback,
                                                   protocol::FeedbackKind::
                                                       Reply);
          if (!endpoint->send(frame)) {
            handle_disconnect(bus);
            return;
          }
        }
      }
    }
  }
}

void MotorSimNode::control_tick() {
  const auto now = FrameRouter::Clock::now();
  last_efforts_ = router_->compute_efforts(
      joint_states_,
      1.0 / static_cast<double>(router_->config().timing.control_hz), now);
  publish_efforts(last_efforts_);
  publish_drive_state();
  const auto safety = router_->safety_state();
  if (safety.latched) {
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                       fault_message(safety.fault));
  } else if (safety.rearm_required) {
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
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
    // CERB 没有周期上报（手册未定义），反馈完全由命令帧触发，
    // 在 transport_tick 的一问一答路径中回发。
    if (motor.protocol == "cerb") {
      continue;
    }
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

void MotorSimNode::publish_drive_state() {
  std_msgs::msg::String message;
  message.data = std::string(core::drive_state_name(router_->drive_state()));
  drive_state_publisher_->publish(message);
}

void MotorSimNode::control_gate_callback(const std_msgs::msg::Bool &message) {
  control_gate_open_ = message.data;
  router_->set_control_gate(message.data);
  last_transition_reason_ = message.data ? "controller gate opened"
                                         : "controller gate closed";
}

void MotorSimNode::reset_fault_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const auto result = router_->reset_fault();
  response->success = result == core::ResetFaultResult::Cleared;
  switch (result) {
    case core::ResetFaultResult::Cleared:
      response->message = "故障已清除；请重新使能并发送全轴新鲜命令";
      reset_rejection_reason_.clear();
      last_transition_reason_ = "fault reset";
      break;
    case core::ResetFaultResult::NotLatched:
      response->message = "当前没有锁存故障";
      break;
    case core::ResetFaultResult::AxesStillEnabled:
      response->message = "拒绝复位：必须先显式 disable 全部轴";
      break;
    case core::ResetFaultResult::TransportDisconnected:
      response->message = "拒绝复位：传输已断开";
      break;
    case core::ResetFaultResult::ConditionActive:
      response->message = "拒绝复位：故障条件仍然存在";
      break;
  }
  if (!response->success) {
    reset_rejection_reason_ = response->message;
  }
  publish_drive_state();
}

void MotorSimNode::publish_diagnostic(const int level,
                                      const std::string &message) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<uint8_t>(level);
  status.name = "motor_sim";
  status.hardware_id = "gazebo-serial-simulator";
  status.message = message;
  const auto safety = router_->safety_state();
  status.values.resize(7U);
  status.values[0] = diagnostic_value(
      "drive_state", std::string(core::drive_state_name(router_->drive_state())));
  status.values[1] = diagnostic_value(
      "brake_engaged",
      router_->drive_state() == core::DriveState::MotorControlActive ? "false"
                                                                     : "true");
  status.values[2] = diagnostic_value(
      "control_gate", control_gate_open_ ? "true" : "false");
  status.values[3] = diagnostic_value("fault", fault_message(safety.fault));
  status.values[4] = diagnostic_value(
      "fault_latched", safety.latched ? "true" : "false");
  status.values[5] =
      diagnostic_value("last_transition_reason", last_transition_reason_);
  status.values[6] =
      diagnostic_value("reset_rejection_reason", reset_rejection_reason_);
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
  publish_drive_state();
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
      std::chrono::milliseconds(100)));
  shutdown_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
    shutdown_timer_->cancel();
    rclcpp::shutdown();
  });
}

}  // namespace motor_sim::ros
