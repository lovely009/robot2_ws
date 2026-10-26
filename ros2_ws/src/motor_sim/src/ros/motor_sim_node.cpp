#include "motor_sim/ros/motor_sim_node.hpp"
#include "motor_sim/transport/socket_can_transport.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>

namespace motor_sim::ros {
using ::motor_sim::core::Fault;
namespace {

// core 层不依赖 motor_transport，故传输类型在 core::BusTransport 里定义，
// 到传输实现的映射落在这一层。
transport::TransportMode bus_transport_mode(
    const ::motor_sim::core::BusTransport value) {
  switch (value) {
    case ::motor_sim::core::BusTransport::UsbSerial:
      return transport::TransportMode::UsbSerial;
    case ::motor_sim::core::BusTransport::SerialSlcan:
      return transport::TransportMode::SerialSlcan;
    case ::motor_sim::core::BusTransport::SocketCan:
      return transport::TransportMode::SocketCan;
  }
  throw std::invalid_argument("unhandled bus transport");
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

// 定义在 motor_sim::ros 作用域（头文件对外声明，集成层入口的守卫）：
// 结构合法性归 core，这里只看运行期环境事实。
void validate_bus_endpoints(const ::motor_sim::core::SystemConfig &config) {
  std::set<std::string> resolved;
  for (const auto &bus : config.buses) {
    const std::string context = "bus '" + bus.name + "'";
    if (bus.transport == ::motor_sim::core::BusTransport::SocketCan) {
      if (::if_nametoindex(bus.endpoint.c_str()) == 0U) {
        throw std::invalid_argument(context +
                                    " socketcan interface must exist");
      }
      continue;
    }
    if (!std::filesystem::exists(bus.endpoint)) {
      throw std::invalid_argument(context + " serial port must exist");
    }
    const int descriptor =
        ::open(bus.endpoint.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0 || ::isatty(descriptor) != 1) {
      if (descriptor >= 0) {
        ::close(descriptor);
      }
      throw std::invalid_argument(context + " serial port must open as a tty");
    }
    ::close(descriptor);
    if (!resolved.insert(std::filesystem::canonical(bus.endpoint).string())
             .second) {
      throw std::invalid_argument("serial ports must be unique");
    }
  }
}

namespace {

// 从 config 构造 transports，并返回按 bus 排序的端口映射（用于诊断发布）。
// 每条总线各自持有传输类型：多总线可以混合 socket_can 与串口。
std::pair<TransportMap, PortMap> build_transports(
    const SystemConfig &config) {
  TransportMap transports;
  PortMap ordered;
  for (const auto &bus_cfg : config.buses) {
    ordered.push_back({bus_cfg.name, bus_cfg.endpoint});
    const auto mode = bus_transport_mode(bus_cfg.transport);
    if (mode == transport::TransportMode::SocketCan) {
      transports.emplace(bus_cfg.name,
                         std::make_unique<transport::SocketCanTransport>(
                             bus_cfg.endpoint));
    } else {
      transports.emplace(
          bus_cfg.name,
          std::make_unique<transport::SerialTransport>(bus_cfg.endpoint, mode,
                                                       bus_cfg.baud_rate));
    }
  }
  return {std::move(transports), ordered};
}

}  // namespace

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options,
                           NodeDefaults defaults)
    : rclcpp::Node("motor_sim", options) {
  NodeParameters parameters;
  // 先读取 config_file，再从配置推导出总线集合；
  // 这样独立入口不需要调用方预先声明 bus 列表或拓扑契约。
  parameters.config_file = declare_parameter<std::string>(
      "config_file", defaults.config_file);
  if (parameters.config_file.empty()) {
    throw std::invalid_argument("config_file cannot be empty");
  }
  auto config =
      ::motor_sim::core::load_system_config(parameters.config_file);
  validate_bus_endpoints(config);
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
  defaults.joint_states_topic = declare_parameter<std::string>(
      "joint_states_topic", defaults.joint_states_topic);
  defaults.effort_topic =
      declare_parameter<std::string>("effort_topic", defaults.effort_topic);
  defaults.diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", defaults.diagnostics_topic);

  std::set<std::string> socket_can_buses;
  for (const auto &bus : config.buses) {
    if (bus.transport == ::motor_sim::core::BusTransport::SocketCan) {
      socket_can_buses.insert(bus.name);
    }
  }
  auto [transports, ordered] = build_transports(config);
  initialize(std::move(config), std::move(transports), defaults);
  for (const auto &[bus, device] : ordered) {
    if (socket_can_buses.count(bus) != 0U) {
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

MotorSimNode::MotorSimNode(const rclcpp::NodeOptions &options,
                           ::motor_sim::core::TopologyContract contract,
                           NodeDefaults defaults)
    : rclcpp::Node("motor_sim", options) {
  NodeParameters parameters;
  parameters.config_file = declare_parameter<std::string>(
      "config_file", defaults.config_file);
  if (parameters.config_file.empty()) {
    throw std::invalid_argument("config_file cannot be empty");
  }
  auto config = ::motor_sim::core::load_system_config(parameters.config_file,
                                                      contract);
  validate_bus_endpoints(config);
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
  defaults.joint_states_topic = declare_parameter<std::string>(
      "joint_states_topic", defaults.joint_states_topic);
  defaults.effort_topic =
      declare_parameter<std::string>("effort_topic", defaults.effort_topic);
  defaults.diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", defaults.diagnostics_topic);
  std::set<std::string> socket_can_buses;
  for (const auto &bus : config.buses) {
    if (bus.transport == ::motor_sim::core::BusTransport::SocketCan) {
      socket_can_buses.insert(bus.name);
    }
  }
  auto [transports, ordered] = build_transports(config);
  initialize(std::move(config), std::move(transports), defaults);
  for (const auto &[bus, device] : ordered) {
    if (socket_can_buses.count(bus) != 0U) {
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
  const auto driver_file = declare_parameter<std::string>("driver_file", defaults.driver_file);
  output_joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", defaults.joint_names);
  if (!driver_file.empty()) {
    transmission_ = std::make_unique<robot_transmission::Model>(
        robot_transmission::Model::load(driver_file));
    std::map<int, std::string> names_by_id;
    for (const auto &[name, id] : transmission_->actuator_ids()) {
      if (!names_by_id.emplace(id, name).second) {
        throw std::invalid_argument("motor_sim transmission requires unique motor IDs");
      }
    }
    if (names_by_id.size() != config.motors.size()) {
      throw std::invalid_argument("driver_file and simulator motor counts differ");
    }
    std::set<int> seen;
    for (const auto &motor : config.motors) {
      const auto found = names_by_id.find(static_cast<int>(motor.id));
      if (found == names_by_id.end() || !seen.insert(static_cast<int>(motor.id)).second) {
        throw std::invalid_argument("driver_file and simulator motor IDs differ");
      }
      actuator_names_.push_back(found->second);
    }
    if (output_joint_names_.empty()) {
      for (const auto &motor : config.motors) output_joint_names_.push_back(motor.joint);
    }
    const std::set<std::string> expected(transmission_->joint_names().begin(),
                                         transmission_->joint_names().end());
    const std::set<std::string> actual(output_joint_names_.begin(), output_joint_names_.end());
    if (actual != expected || actual.size() != output_joint_names_.size()) {
      throw std::invalid_argument("joint_names must cover transmission joints exactly once");
    }
    actuator_state_valid_.assign(config.motors.size(), false);
  }
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

  for (const auto &motor : router_->config().motors) {
    if (motor.protocol != "cerb") {
      continue;  // 只有 CERB 有 SDO 从站；vaxor 不回 SDO。
    }
    sdo_server_.add_node(motor.id, motor.device_identity,
                         core::cerb_sdo_initial_objects(motor));
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
  if (transmission_) {
    const auto now = FrameRouter::Clock::now();
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      const auto &name = message.name[i];
      if (std::find(output_joint_names_.begin(), output_joint_names_.end(), name)
          == output_joint_names_.end()) continue;
      if (i >= message.position.size() || i >= message.velocity.size() ||
          !std::isfinite(message.position[i]) || !std::isfinite(message.velocity[i])) {
        measured_joints_.erase(name);
        measured_at_.erase(name);
        continue;
      }
      measured_joints_[name] = {message.position[i], message.velocity[i]};
      measured_at_[name] = now;
    }
    update_actuator_states();
    return;
  }
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

void MotorSimNode::update_actuator_states() {
  if (!transmission_) return;
  const auto now = FrameRouter::Clock::now();
  robot_transmission::AxisMap complete;
  for (const auto &group : transmission_->groups()) {
    bool valid = true;
    auto oldest = now;
    auto newest = FrameRouter::TimePoint::min();
    for (const auto &name : group.joints) {
      const auto found = measured_at_.find(name);
      if (found == measured_at_.end() || now - found->second > std::chrono::milliseconds(500)) {
        valid = false;
        break;
      }
      oldest = std::min(oldest, found->second);
      newest = std::max(newest, found->second);
    }
    if (valid && newest - oldest <= std::chrono::milliseconds(50)) {
      for (const auto &name : group.joints) complete.emplace(name, measured_joints_.at(name));
    }
  }
  const auto states = transmission_->joint_to_actuator(complete);
  for (std::size_t i = 0; i < actuator_names_.size(); ++i) {
    const auto found = states.find(actuator_names_[i]);
    actuator_state_valid_[i] = found != states.end();
    if (found != states.end()) {
      joint_states_[i].position_rad = found->second.position;
      joint_states_[i].velocity_rad_s = found->second.velocity;
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
      // SDO 是独立的一问一答命名空间（0x600/0x580 + node），不能流进 MIT 路由。
      if (handle_sdo_request(bus, *endpoint, received_frame.frame)) {
        continue;
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
          const auto &motor = router_->config().motors[index];
          const auto frame =
              protocol::make_motor_protocol(motor.protocol,
                                            motor.mit_torque_limit_nm,
                                            motor.mit_position_limit_rad,
                                            motor.mit_velocity_limit_rad_s,
                                            motor.mit_kp_limit, motor.mit_kd_limit)
                  ->encode_feedback(feedback, protocol::FeedbackKind::Reply);
          if (!endpoint->send(frame)) {
            handle_disconnect(bus);
            return;
          }
        }
      }
    }
  }
}

bool MotorSimNode::handle_sdo_request(const std::string &bus,
                                      transport::FrameTransport &endpoint,
                                      const protocol::CanFrame &frame) {
  constexpr uint32_t kSdoRequestBase = 0x600U;
  constexpr uint32_t kSdoResponseBase = 0x580U;
  constexpr uint32_t kSdoIdSpan = 0x80U;  // CERB 的 node id 上限
  if (frame.id < kSdoRequestBase || frame.id >= kSdoRequestBase + kSdoIdSpan) {
    return false;
  }
  const auto node_id = frame.id - kSdoRequestBase;
  core::SdoFrame request{};
  std::copy_n(frame.data.begin(), request.size(), request.begin());
  core::SdoFrame response{};
  if (!sdo_server_.handle(node_id, request, response)) {
    return true;  // 是 SDO 命名空间但不是本从站管的：吞掉，不喂 MIT 路由
  }
  protocol::CanFrame reply;
  reply.id = kSdoResponseBase + node_id;
  reply.fd = true;
  reply.brs = frame.brs;  // 与请求同 BRS，别自作主张改成经典帧
  reply.size = response.size();
  std::copy(response.begin(), response.end(), reply.data.begin());
  if (!endpoint.send(reply)) {
    handle_disconnect(bus);
  }
  return true;
}

void MotorSimNode::control_tick() {
  const auto now = FrameRouter::Clock::now();
  update_actuator_states();
  last_efforts_ = router_->compute_efforts(
      joint_states_,
      1.0 / static_cast<double>(router_->config().timing.control_hz), now);
  if (transmission_) {
    for (std::size_t i = 0; i < last_efforts_.size(); ++i) {
      if (!actuator_state_valid_[i]) last_efforts_[i] = 0.0;
    }
  }
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
  const auto vaxor_divisor = std::max(
      1U, config.timing.control_hz / config.timing.vaxor_feedback_hz);
  for (std::size_t index = 0U; index < config.motors.size(); ++index) {
    const auto &motor = config.motors[index];
    // CERB 没有周期上报（手册未定义），反馈完全由命令帧触发，
    // 在 transport_tick 的一问一答路径中回发。
    if (motor.protocol == "cerb") {
      continue;
    }
    if (tick % vaxor_divisor != 0U) {
      continue;
    }
    const auto feedback =
        router_->motor_feedback(index, joint_states_[index], last_efforts_[index]);
    const auto frame =
        protocol::make_motor_protocol(motor.protocol, motor.mit_torque_limit_nm,
                                      motor.mit_position_limit_rad,
                                      motor.mit_velocity_limit_rad_s,
                                      motor.mit_kp_limit, motor.mit_kd_limit)
            ->encode_feedback(feedback, protocol::FeedbackKind::Status);
    const auto endpoint = transports_.find(motor.bus);
    if (endpoint == transports_.end() || !endpoint->second->send(frame)) {
      handle_disconnect(motor.bus);
      return;
    }
  }
}

void MotorSimNode::publish_efforts(const std::vector<double> &efforts) {
  std_msgs::msg::Float64MultiArray message;
  if (transmission_) {
    std::map<std::string, double> actuator_efforts;
    for (std::size_t i = 0; i < actuator_names_.size(); ++i) {
      actuator_efforts.emplace(actuator_names_[i], efforts.at(i));
    }
    const auto joints = transmission_->actuator_effort_to_joint(actuator_efforts);
    for (const auto &name : output_joint_names_) message.data.push_back(joints.at(name));
  } else {
    message.data = efforts;
  }
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
