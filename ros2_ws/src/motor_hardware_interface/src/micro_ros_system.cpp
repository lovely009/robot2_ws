#include "motor_hardware_interface/micro_ros_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <thread>
#include <utility>

#include "can_bridge_msgs/msg/motor_command_target.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

namespace motor_hardware_interface {
namespace {
constexpr std::size_t kMotorMax = 48;
constexpr uint16_t kIdMax = 64;
constexpr auto kMotionFresh = std::chrono::milliseconds(100);
constexpr auto kControlFresh = std::chrono::milliseconds(300);
constexpr auto kClockFutureTolerance = std::chrono::milliseconds(20);
using Clock = std::chrono::steady_clock;

bool newer(uint32_t sequence, uint32_t previous) {
  return static_cast<int32_t>(sequence - previous) > 0;
}

uint64_t now_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

uint64_t elapsed_ms(Clock::time_point from, Clock::time_point to) {
  return static_cast<uint64_t>(std::max<int64_t>(0,
    std::chrono::duration_cast<std::chrono::milliseconds>(to - from).count()));
}

bool source_stamp_ok(uint64_t sync_time_ns, Clock::time_point received,
                     std::chrono::milliseconds limit, Clock::time_point now) {
  if (sync_time_ns == 0U) {
    return false;
  }
  const int64_t delta = static_cast<int64_t>(now_ns()) - static_cast<int64_t>(sync_time_ns);
  if (delta < -std::chrono::duration_cast<std::chrono::nanoseconds>(kClockFutureTolerance).count() ||
      delta > std::chrono::duration_cast<std::chrono::nanoseconds>(limit).count()) {
    return false;
  }
  return elapsed_ms(received, now) <= static_cast<uint64_t>(limit.count());
}

bool within(const std::optional<double> &bound, double value, bool upper) {
  return !bound || (upper ? value <= *bound : value >= *bound);
}

bool healthy_motion_source(uint8_t source) {
  return source == can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_NONE ||
         source == can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_SNAPSHOT;
}

bool healthy_execution(uint8_t state) {
  return state == can_bridge_msgs::msg::MotorControlState::EXECUTION_IDLE ||
         state == can_bridge_msgs::msg::MotorControlState::EXECUTION_HOLDING ||
         state == can_bridge_msgs::msg::MotorControlState::EXECUTION_COMMAND ||
         state == can_bridge_msgs::msg::MotorControlState::EXECUTION_ENABLING;
}

std::string joined(const std::vector<std::string> &names) {
  std::string result;
  for (const auto &name : names) {
    if (!result.empty()) {
      result += ",";
    }
    result += name;
  }
  return result;
}
} // namespace

bool MicroRosSystem::load_model(const std::string &path, std::string &error) {
  if (path.empty()) {
    error = "hardware parameter 'config_file' is required";
    return false;
  }
  try {
    const auto transmissions = info_.hardware_parameters.find("transmissions_file");
    model_ = std::make_unique<robot_transmission::Model>(robot_transmission::Model::load(
      path, transmissions == info_.hardware_parameters.end() ? "" : transmissions->second));
    return true;
  } catch (const std::exception &exception) {
    error = "transmission model: " + std::string(exception.what());
    return false;
  }
}

bool MicroRosSystem::parse_limit(const std::string &raw, std::optional<double> &bound, std::string &error) const {
  auto text = raw;
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), [](unsigned char c) {
    return !std::isspace(c);
  }));
  text.erase(std::find_if(text.rbegin(), text.rend(), [](unsigned char c) {
    return !std::isspace(c);
  }).base(), text.end());
  if (text.empty()) {
    return true;
  }
  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end != text.c_str() + text.size() || !std::isfinite(value)) {
    error = "command interface bound is not a finite number: " + raw;
    return false;
  }
  bound = value;
  return true;
}

bool MicroRosSystem::build_interface_mapping(std::string &error) {
  const auto &actuators = model_->actuator_names();
  if (actuators.size() > kMotorMax) {
    error = "configured actuators exceed 48";
    return false;
  }
  const auto &ids = model_->actuator_ids();
  for (const auto &name : actuators) {
    const auto id = static_cast<uint16_t>(ids.at(name));
    if (id == 0U || id > kIdMax) {
      error = "configured actuator id outside 1..64: " + name;
      return false;
    }
    if (!id_to_actuator_.emplace(id, name).second || !expected_ids_.insert(id).second) {
      error = "configured actuator motor_id is duplicated: " + std::to_string(id);
      return false;
    }
  }

  const auto &model_joints = model_->joint_names();
  const std::set<std::string> expected_joints(model_joints.begin(), model_joints.end());
  if (expected_joints.size() != model_joints.size()) {
    error = "transmission model contains duplicate joints";
    return false;
  }
  std::set<std::string> ros_joints;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (!ros_joints.insert(joint.name).second) {
      error = "ros2_control contains duplicate joint " + joint.name;
      return false;
    }
    std::set<std::string> commands;
    Limit position_limit;
    Limit velocity_limit;
    Limit effort_limit;
    for (const auto &interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        if (!parse_limit(interface.min, position_limit.min, error) ||
            !parse_limit(interface.max, position_limit.max, error)) {
          return false;
        }
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (!parse_limit(interface.min, velocity_limit.min, error) ||
            !parse_limit(interface.max, velocity_limit.max, error)) {
          return false;
        }
      } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
        if (!parse_limit(interface.min, effort_limit.min, error) ||
            !parse_limit(interface.max, effort_limit.max, error)) {
          return false;
        }
      } else {
        error = "joint " + joint.name + " has unsupported command interface " + interface.name;
        return false;
      }
      if (!commands.insert(interface.name).second) {
        error = "joint " + joint.name + " has duplicate command interfaces";
        return false;
      }
    }
    if (commands.count(hardware_interface::HW_IF_POSITION) == 0U ||
        commands.count(hardware_interface::HW_IF_VELOCITY) == 0U) {
      error = "joint " + joint.name + " requires position and velocity command interfaces";
      return false;
    }
    std::set<std::string> states;
    for (const auto &interface : joint.state_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION &&
          interface.name != hardware_interface::HW_IF_VELOCITY &&
          interface.name != hardware_interface::HW_IF_EFFORT) {
        error = "joint " + joint.name + " has unsupported state interface " + interface.name;
        return false;
      }
      if (!states.insert(interface.name).second) {
        error = "joint " + joint.name + " has duplicate state interfaces";
        return false;
      }
    }
    if (states.size() != 3U) {
      error = "joint " + joint.name + " requires position, velocity and effort states";
      return false;
    }
    if ((position_limit.min && position_limit.max &&
         *position_limit.min > *position_limit.max) ||
        (velocity_limit.min && velocity_limit.max &&
         *velocity_limit.min > *velocity_limit.max) ||
        (effort_limit.min && effort_limit.max && *effort_limit.min > *effort_limit.max)) {
      error = "joint " + joint.name + " has reversed command limits";
      return false;
    }
    joint_indices_.emplace(joint.name, index);
    for (const auto &interface : commands) {
      if (interface == hardware_interface::HW_IF_EFFORT) {
        effort_command_interfaces_.insert(joint.name + "/" + interface);
      } else {
        required_command_interfaces_.insert(joint.name + "/" + interface);
      }
    }
    position_limits_.push_back(position_limit);
    velocity_limits_.push_back(velocity_limit);
    effort_limits_.push_back(effort_limit);
  }
  if (ros_joints != expected_joints) {
    std::vector<std::string> missing;
    std::vector<std::string> extra;
    for (const auto &name : expected_joints) {
      if (ros_joints.count(name) == 0U) {
        missing.push_back(name);
      }
    }
    for (const auto &name : ros_joints) {
      if (expected_joints.count(name) == 0U) {
        extra.push_back(name);
      }
    }
    error = "ROS/model joints differ; missing=" + joined(missing) + " extra=" + joined(extra);
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn
MicroRosSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string error;
  const auto config = info_.hardware_parameters.find("config_file");
  if (!load_model(config == info_.hardware_parameters.end() ? "" : config->second, error) ||
      !build_interface_mapping(error)) {
    RCLCPP_ERROR(rclcpp::get_logger("micro_ros_system"), "invalid configuration: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("micro_ros_system"), "ROS context is not initialized");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("micro_ros_system_" + info_.name, options);
    command_pub_ = node_->create_publisher<Command>("/motor/host_command_array",
      rclcpp::QoS(1).reliable());
    motion_sub_ = node_->create_subscription<Motion>("/motor/motion_states",
      rclcpp::QoS(1).best_effort(), [this](Motion::SharedPtr message) { on_motion(message); });
    control_sub_ = node_->create_subscription<Control>("/motor/control_states",
      rclcpp::QoS(1).reliable(), [this](Control::SharedPtr message) { on_control(message); });
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>("/host_control/stop",
      rclcpp::QoS(1).reliable(), [this](std_msgs::msg::Empty::SharedPtr) { close_writer(); });
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("micro_ros_system"), "ROS node setup failed: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto unlimited = std::numeric_limits<double>::quiet_NaN();
  position_.assign(info_.joints.size(), unlimited);
  velocity_.assign(info_.joints.size(), 0.0);
  effort_.assign(info_.joints.size(), 0.0);
  command_position_.assign(info_.joints.size(), unlimited);
  command_velocity_.assign(info_.joints.size(), 0.0);
  command_effort_.assign(info_.joints.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroRosSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_POSITION, &position_[index]);
    result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_[index]);
    result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_EFFORT, &effort_[index]);
  }
  return result;
}

hardware_interface::return_type MicroRosSystem::prepare_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces) {
  std::set<std::string> next;
  std::set<std::string> started;
  return transitioned_claims(start_interfaces, stop_interfaces, next, started) ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

std::vector<hardware_interface::CommandInterface> MicroRosSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_POSITION,
                        &command_position_[index]);
    result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
                        &command_velocity_[index]);
    if (effort_command_interfaces_.count(info_.joints[index].name + "/effort")) {
      result.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_EFFORT,
                          &command_effort_[index]);
    }
  }
  return result;
}

bool MicroRosSystem::transitioned_claims(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces,
  std::set<std::string> &next,
  std::set<std::string> &started) const {
  next = claimed_command_interfaces_;
  started.clear();
  for (const auto &name : stop_interfaces) {
    next.erase(name);
  }
  for (const auto &name : start_interfaces) {
    if (required_command_interfaces_.count(name) || effort_command_interfaces_.count(name)) {
      if (next.count(name)) {
        return false;
      }
      next.insert(name);
      started.insert(name);
    }
  }
  const auto complete_group = [](const auto &claims, const auto &group) {
    const auto count = std::count_if(group.begin(), group.end(),
      [&](const auto &name) { return claims.count(name); });
    return count == 0 || static_cast<std::size_t>(count) == group.size();
  };
  return complete_group(started, required_command_interfaces_) &&
         complete_group(started, effort_command_interfaces_) &&
         complete_group(next, required_command_interfaces_) &&
         complete_group(next, effort_command_interfaces_);
}

void MicroRosSystem::close_writer() {
  writer_enabled_ = false;
  armed_ = false;
  command_position_.assign(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());
  command_velocity_.assign(info_.joints.size(), 0.0);
  std::fill(command_effort_.begin(), command_effort_.end(), 0.0);
}

bool MicroRosSystem::fail(std::string error) {
  faulted_ = true;
  armed_ = false;
  std::fill(command_effort_.begin(), command_effort_.end(), 0.0);
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
  }
  return false;
}

void MicroRosSystem::on_motion(const Motion::SharedPtr message) {
  motion_valid_ = false;
  latest_motion_.reset();
  if (!message || message->seq == 0U || message->states.size() > kIdMax ||
      message->sync_time_ns < session_time_floor_ns_) {
    fail("invalid motion snapshot");
    return;
  }
  std::set<uint16_t> seen;
  for (const auto &state : message->states) {
    if (state.motor_id == 0U || state.motor_id > kIdMax ||
        !seen.insert(state.motor_id).second) {
      fail("invalid or duplicate motion motor id");
      return;
    }
    if (!expected_ids_.count(state.motor_id)) {
      continue;
    }
    if (!healthy_motion_source(state.target_source) ||
        !std::isfinite(state.position) || !std::isfinite(state.velocity) ||
        !std::isfinite(state.effort) || state.state_age_ms == std::numeric_limits<uint32_t>::max()) {
      fail("invalid motion snapshot id/source/value/age");
      return;
    }
  }
  if (!std::includes(seen.begin(), seen.end(), expected_ids_.begin(), expected_ids_.end())) {
    fail("motion snapshot misses configured actuator");
    return;
  }
  const auto now = Clock::now();
  if (!source_stamp_ok(message->sync_time_ns, now, kMotionFresh, now)) {
    fail("motion snapshot is stale");
    return;
  }
  const auto received_ns = now_ns();
  const auto delay_ns = received_ns > message->sync_time_ns ?
    received_ns - message->sync_time_ns : 0U;
  // 桥已累计部分源延迟；保守再次计入以覆盖桥到本回调的排队时间。
  const uint64_t source_delay_ms = (delay_ns + 999999U) / 1000000U;
  for (const auto &state : message->states) {
    if (expected_ids_.count(state.motor_id) &&
        static_cast<uint64_t>(state.state_age_ms) + source_delay_ms > kMotionFresh.count()) {
      fail("motion snapshot is stale");
      return;
    }
  }
  if (motion_seq_ != 0U && !newer(message->seq, motion_seq_)) {
    fail("motion sequence is replay or out of order");
    return;
  }
  motion_seq_ = message->seq;
  latest_motion_ = *message;
  auto &states = latest_motion_->states;
  states.erase(std::remove_if(states.begin(), states.end(), [this](const auto &state) {
    return !expected_ids_.count(state.motor_id);
  }), states.end());
  motion_received_ = now;
  motion_source_delay_ms_ = source_delay_ms;
  motion_valid_ = true;
}

void MicroRosSystem::on_control(const Control::SharedPtr message) {
  control_valid_ = false;
  latest_control_.reset();
  if (!message || message->schema_version != Control::SCHEMA_VERSION ||
      message->session_id == 0U || message->sequence == 0U || message->sync_time_ns == 0U ||
      message->states.size() > kIdMax) {
    fail("invalid control snapshot header/coverage");
    return;
  }
  std::set<uint16_t> seen;
  for (const auto &state : message->states) {
    if (state.motor_id == 0U || state.motor_id > kIdMax ||
        !seen.insert(state.motor_id).second) {
      fail("invalid or duplicate control motor id");
      return;
    }
    if (expected_ids_.count(state.motor_id) &&
        (state.unified_fault_code != 0U || state.vendor_fault_code != 0U ||
         state.control_lifecycle ==
           can_bridge_msgs::msg::MotorControlState::LIFECYCLE_ERROR_PROCESSING)) {
      fail("configured motor reports a current fault");
      return;
    }
  }
  if (!std::includes(seen.begin(), seen.end(), expected_ids_.begin(), expected_ids_.end())) {
    fail("control snapshot misses configured actuator");
    return;
  }
  const auto now = Clock::now();
  if (!source_stamp_ok(message->sync_time_ns, now, kControlFresh, now)) {
    fail("control stream snapshot is stale");
    return;
  }
  if (control_session_ != message->session_id) {
    if (active_ && control_session_ != 0U) {
      fail("control session changed while active");
      return;
    }
    control_session_ = message->session_id;
    session_time_floor_ns_ = message->sync_time_ns;
    control_seq_ = 0U;
    motion_seq_ = 0U;
    latest_motion_.reset();
    motion_valid_ = false;
  } else if (control_seq_ != 0U && !newer(message->sequence, control_seq_)) {
    fail("control sequence is replay or out of order");
    return;
  }
  control_seq_ = message->sequence;
  latest_control_ = *message;
  auto &states = latest_control_->states;
  states.erase(std::remove_if(states.begin(), states.end(), [this](const auto &state) {
    return !expected_ids_.count(state.motor_id);
  }), states.end());
  control_received_ = now;
  control_valid_ = true;
}

bool MicroRosSystem::motion_current(const TimePoint &now, bool latch, std::string &error) {
  if (!motion_valid_ || !latest_motion_) {
    error = "fresh complete motion feedback is unavailable";
    return latch ? fail(error) : false;
  }
  if (latest_motion_->sync_time_ns < session_time_floor_ns_ ||
      !source_stamp_ok(latest_motion_->sync_time_ns, motion_received_, kMotionFresh, now)) {
    error = "motion feedback is stale or predates control session";
    return latch ? fail(error) : false;
  }
  for (const auto &state : latest_motion_->states) {
    if (static_cast<uint64_t>(state.state_age_ms) + motion_source_delay_ms_ +
        elapsed_ms(motion_received_, now) >
        static_cast<uint64_t>(kMotionFresh.count())) {
      error = "measured motion feedback exceeds 100 ms";
      return latch ? fail(error) : false;
    }
  }
  return true;
}

bool MicroRosSystem::control_current(const TimePoint &now, bool latch, std::string &error) {
  if (!control_valid_ || !latest_control_) {
    error = "control stream snapshot is unavailable";
    return latch ? fail(error) : false;
  }
  if (!source_stamp_ok(latest_control_->sync_time_ns, control_received_, kControlFresh, now)) {
    error = "control stream snapshot exceeds 300 ms";
    return latch ? fail(error) : false;
  }
  return true;
}

bool MicroRosSystem::controls_armed(const TimePoint &now, std::string &error) const {
  static_cast<void>(now);
  if (!latest_control_) {
    return false;
  }
  for (const auto &state : latest_control_->states) {
    if (!state.online ||
        state.control_lifecycle != can_bridge_msgs::msg::MotorControlState::LIFECYCLE_ACTIVE ||
        !state.policy_enabled || !healthy_execution(state.execution_state) ||
        state.unified_fault_code != 0U || state.vendor_fault_code != 0U) {
      error = "not all configured motors have ACTIVE, healthy policy";
      return false;
    }
  }
  return true;
}

bool MicroRosSystem::update_states(const TimePoint &now, bool latch, std::string &error) {
  if (!motion_current(now, latch, error)) {
    return false;
  }
  robot_transmission::AxisMap actuators;
  for (const auto &[id, name] : id_to_actuator_) {
    const auto found = std::find_if(latest_motion_->states.begin(), latest_motion_->states.end(),
      [id](const auto &state) { return state.motor_id == id; });
    if (found == latest_motion_->states.end()) {
      error = "motion snapshot misses configured actuator";
      return latch ? fail(error) : false;
    }
    actuators.emplace(name, robot_transmission::Axis(found->position, found->velocity, found->effort));
  }
  robot_transmission::AxisMap joints;
  try {
    joints = model_->actuator_to_joint(actuators);
  } catch (const std::exception &exception) {
    error = "transmission feedback mapping failed: " + std::string(exception.what());
    return latch ? fail(error) : false;
  }
  if (joints.size() != model_->joint_names().size()) {
    error = "transmission feedback group is incomplete";
    return latch ? fail(error) : false;
  }
  for (const auto &[name, axis] : joints) {
    if (joint_indices_.count(name) == 0U || !std::isfinite(axis.position) ||
        !std::isfinite(axis.velocity) || !std::isfinite(axis.effort)) {
      error = "transmission feedback contains invalid joint";
      return latch ? fail(error) : false;
    }
    const auto index = joint_indices_.at(name);
    position_[index] = axis.position;
    velocity_[index] = axis.velocity;
    effort_[index] = axis.effort;
  }
  return true;
}

hardware_interface::return_type MicroRosSystem::perform_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces) {
  std::set<std::string> next;
  std::set<std::string> started;
  if (!transitioned_claims(start_interfaces, stop_interfaces, next, started)) {
    close_writer();
    return hardware_interface::return_type::ERROR;
  }
  const bool start_motion = std::any_of(started.begin(), started.end(),
    [this](const auto &name) { return required_command_interfaces_.count(name); });
  const bool motion_claimed = std::includes(next.begin(), next.end(),
    required_command_interfaces_.begin(), required_command_interfaces_.end());
  const bool effort_changed = std::any_of(start_interfaces.begin(), start_interfaces.end(),
    [this](const auto &name) { return effort_command_interfaces_.count(name); }) ||
    std::any_of(stop_interfaces.begin(), stop_interfaces.end(),
    [this](const auto &name) { return effort_command_interfaces_.count(name); });
  if (start_motion || effort_changed || !motion_claimed) {
    std::fill(command_effort_.begin(), command_effort_.end(), 0.0);
  }
  if (start_motion) {
    const auto now = Clock::now();
    std::string error;
    if (!active_ || faulted_ || !control_current(now, false, error) ||
        !update_states(now, false, error)) {
      close_writer();
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      command_position_[index] = position_[index];
      command_velocity_[index] = 0.0;
    }
    writer_enabled_ = true;
    armed_ = controls_armed(now, error);
  } else if (!motion_claimed) {
    close_writer();
  }
  claimed_command_interfaces_ = std::move(next);
  return hardware_interface::return_type::OK;
}

bool MicroRosSystem::command_valid(std::string &error) const {
  if (command_position_.size() != info_.joints.size() ||
      command_velocity_.size() != info_.joints.size() ||
      command_effort_.size() != info_.joints.size()) {
    error = "command interface storage is invalid";
    return false;
  }
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const double position = command_position_[index];
    const double velocity = command_velocity_[index];
    if (!std::isfinite(position) || !std::isfinite(velocity)) {
      error = "joint command is NaN/Inf: " + info_.joints[index].name;
      return false;
    }
    if (!within(position_limits_[index].min, position, false) ||
        !within(position_limits_[index].max, position, true)) {
      error = "joint position command exceeds URDF limit: " + info_.joints[index].name;
      return false;
    }
    if (!within(velocity_limits_[index].min, velocity, false) ||
        !within(velocity_limits_[index].max, velocity, true)) {
      error = "joint velocity command exceeds URDF limit: " + info_.joints[index].name;
      return false;
    }
    if (claimed_command_interfaces_.count(info_.joints[index].name + "/effort")) {
      const double effort = command_effort_[index];
      if (!std::isfinite(effort) || !within(effort_limits_[index].min, effort, false) ||
          !within(effort_limits_[index].max, effort, true)) {
        error = "joint effort command is nonfinite or exceeds URDF limit: " + info_.joints[index].name;
        return false;
      }
    }
  }
  return true;
}

bool MicroRosSystem::publish(bool commanded, std::string &error) {
  Command message;
  message.sync_time_ns = now_ns();
  if (++command_seq_ == 0U) {
    ++command_seq_;
  }
  message.sequence = command_seq_;
  if (commanded) {
    robot_transmission::AxisMap joints;
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      joints.emplace(info_.joints[index].name,
        robot_transmission::Axis(command_position_[index], command_velocity_[index],
          claimed_command_interfaces_.count(info_.joints[index].name + "/effort") ?
          command_effort_[index] : 0.0));
    }
    robot_transmission::AxisMap actuators;
    try {
      actuators = model_->joint_to_actuator(joints);
    } catch (const std::exception &exception) {
      error = "transmission command mapping failed: " + std::string(exception.what());
      return false;
    }
    std::set<uint16_t> seen;
    for (const auto &name : model_->actuator_names()) {
      const auto axis = actuators.find(name);
      if (axis == actuators.end() || !std::isfinite(axis->second.position) ||
          !std::isfinite(axis->second.velocity) || !std::isfinite(axis->second.effort)) {
        error = "transmission command group is incomplete or nonfinite";
        return false;
      }
      can_bridge_msgs::msg::MotorCommandTarget target;
      target.motor_id = static_cast<uint16_t>(model_->actuator_ids().at(name));
      target.position = static_cast<float>(axis->second.position);
      target.velocity = static_cast<float>(axis->second.velocity);
      target.effort = static_cast<float>(axis->second.effort);
      if (!std::isfinite(target.position) || !std::isfinite(target.velocity) ||
          !std::isfinite(target.effort)) {
        error = "actuator command exceeds wire float range";
        return false;
      }
      if (!seen.insert(target.motor_id).second) {
        error = "duplicate actuator id in command snapshot";
        return false;
      }
      message.targets.push_back(target);
    }
  }
  message.count = static_cast<uint8_t>(message.targets.size());
  command_pub_->publish(message);
  return true;
}

hardware_interface::CallbackReturn
MicroRosSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
  static_cast<void>(previous_state);
  if (!node_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_ = false;
  armed_ = false;
  close_writer();
  claimed_command_interfaces_.clear();
  faulted_ = false;
  motion_valid_ = false;
  control_valid_ = false;
  latest_motion_.reset();
  latest_control_.reset();
  control_session_ = 0U;
  session_time_floor_ns_ = 0U;
  motion_seq_ = 0U;
  control_seq_ = 0U;
  const auto deadline = Clock::now() + std::chrono::seconds(1);
  while (Clock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (faulted_) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto now = Clock::now();
    std::string error;
    if (control_current(now, false, error) && update_states(now, false, error)) {
      for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        command_position_[index] = position_[index];
        command_velocity_[index] = 0.0;
      }
      active_ = true;
      armed_ = controls_armed(now, error);
      if (!publish(false, error)) {
        active_ = false;
        return hardware_interface::CallbackReturn::ERROR;
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    }
  }
  RCLCPP_ERROR(node_->get_logger(), "activation timed out waiting for fresh complete feedback");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn
MicroRosSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  static_cast<void>(previous_state);
  std::string error;
  if (node_ && !publish(false, error)) {
    RCLCPP_ERROR(node_->get_logger(), "failed to publish stop snapshot: %s", error.c_str());
  }
  active_ = false;
  armed_ = false;
  close_writer();
  claimed_command_interfaces_.clear();
  faulted_ = false;
  motion_valid_ = false;
  control_valid_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MicroRosSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
  static_cast<void>(time);
  static_cast<void>(period);
  if (!active_ || faulted_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    rclcpp::spin_some(node_);
  } catch (const std::exception &exception) {
    fail("ROS callback failed: " + std::string(exception.what()));
    return hardware_interface::return_type::ERROR;
  }
  if (faulted_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = Clock::now();
  std::string error;
  if (!control_current(now, true, error) || !update_states(now, true, error)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!writer_enabled_) {
    armed_ = false;
  } else if (controls_armed(now, error)) {
    armed_ = true;
  } else if (armed_) {
    fail(error);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MicroRosSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
  static_cast<void>(time);
  static_cast<void>(period);
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  std::string error;
  if (faulted_) {
    publish(false, error);
    return hardware_interface::return_type::ERROR;
  }
  if (!writer_enabled_) {
    return publish(false, error) ? hardware_interface::return_type::OK :
      hardware_interface::return_type::ERROR;
  }
  const auto now = Clock::now();
  if (!control_current(now, true, error) || !update_states(now, true, error)) {
    const bool sent = publish(false, error);
    static_cast<void>(sent);
    return hardware_interface::return_type::ERROR;
  }
  if (!controls_armed(now, error)) {
    if (armed_) {
      fail(error);
      publish(false, error);
      return hardware_interface::return_type::ERROR;
    }
    if (!publish(false, error)) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  armed_ = true;
  if (!command_valid(error)) {
    fail(error);
    publish(false, error);
    return hardware_interface::return_type::ERROR;
  }
  if (!publish(true, error)) {
    fail(error);
    publish(false, error);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

} // namespace motor_hardware_interface

PLUGINLIB_EXPORT_CLASS(motor_hardware_interface::MicroRosSystem,
                       hardware_interface::SystemInterface)
