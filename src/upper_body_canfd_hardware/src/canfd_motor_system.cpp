#include "upper_body_canfd_hardware/canfd_motor_system.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "upper_body_canfd_hardware/rp_mit.hpp"
#include "upper_body_canfd_hardware/wrist_2530.hpp"

namespace upper_body_canfd_hardware {

namespace {

constexpr double kDefaultRpKp = 36.0;
constexpr double kDefaultRpKd = 2.0;
constexpr double kDefaultWristKp = 0.008;
constexpr double kDefaultWristKd = 1.0;

bool StartsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

hardware_interface::CallbackReturn CanFdMotorSystem::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  if (!ParseParameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CanFdMotorSystem::ParseParameters()
{
  const auto & params = info_.hardware_parameters;
  interface_ = "vcan0";
  if (params.count("interface") != 0) {
    interface_ = params.at("interface");
  }
  allow_hardware_ = ParseDouble(params, "allow_hardware_interfaces", 0.0) != 0.0;
  const auto allow_it = params.find("allow_hardware_interfaces");
  if (allow_it != params.end()) {
    const std::string value = allow_it->second;
    allow_hardware_ = (value == "true" || value == "1");
  }
  feedback_timeout_cycles_ = ParseInt(params, "feedback_timeout_cycles", 50);
  command_timeout_ms_ = ParseDouble(params, "command_timeout_ms", 150.0);

  // Safety gate: refuse physical CAN interfaces unless explicitly enabled.
  if (StartsWith(interface_, "can") && !allow_hardware_) {
    Log(
      "refusing physical interface '" + interface_ +
      "'; set allow_hardware_interfaces:=true only for planned hardware-in-the-loop runs");
    return false;
  }

  if (joints_.size() != info_.joints.size()) {
    joints_.resize(info_.joints.size());
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & component = info_.joints[i];
    auto & joint = joints_[i];
    joint.name = component.name;
    joint.command_effort = 0.0;
    joint.state_position = 0.0;
    joint.state_velocity = 0.0;
    joint.state_effort = 0.0;
    joint.has_feedback = false;
    joint.feedback_miss_cycles = 0;

    const auto & p = component.parameters;
    std::string motor_type = "wrist";
    if (p.count("motor_type") != 0) {
      motor_type = p.at("motor_type");
    }
    if (motor_type == "rp") {
      joint.type = MotorType::kRp;
    } else if (motor_type == "wrist") {
      joint.type = MotorType::kWrist;
    } else {
      Log("joint '" + joint.name + "': motor_type must be 'rp' or 'wrist'");
      return false;
    }

    joint.can_node_id = ParseInt(p, "can_node_id", 1);
    if (joint.type == MotorType::kRp && (joint.can_node_id < 1 || joint.can_node_id > 8)) {
      Log("joint '" + joint.name + "': RP can_node_id must be in [1, 8]");
      return false;
    }
    if (joint.type == MotorType::kWrist &&
        (joint.can_node_id < 1 || joint.can_node_id > 60)) {
      Log("joint '" + joint.name + "': wrist can_node_id must be in [1, 60]");
      return false;
    }

    joint.direction = ParseDouble(p, "direction", 1.0);
    joint.zero_offset = ParseDouble(p, "zero_offset", 0.0);
    joint.effort_scale = ParseDouble(p, "effort_scale", 1.0);
    if (joint.type == MotorType::kRp) {
      joint.kp = ParseDouble(p, "kp", kDefaultRpKp);
      joint.kd = ParseDouble(p, "kd", kDefaultRpKd);
      joint.effort_min = -60.0;
      joint.effort_max = 60.0;
    } else {
      joint.kp = ParseDouble(p, "kp", kDefaultWristKp);
      joint.kd = ParseDouble(p, "kd", kDefaultWristKd);
      joint.effort_min = -5.0;
      joint.effort_max = 5.0;
    }

    // Effort limits come from the declared effort command interface when
    // present (the sim URDF already carries per-joint effort limits).
    bool has_effort_command = false;
    for (const auto & iface : component.command_interfaces) {
      if (iface.name == hardware_interface::HW_IF_EFFORT) {
        has_effort_command = true;
        if (!iface.min.empty()) {
          joint.effort_min = std::strtod(iface.min.c_str(), nullptr);
        }
        if (!iface.max.empty()) {
          joint.effort_max = std::strtod(iface.max.c_str(), nullptr);
        }
      }
    }
    if (!has_effort_command) {
      Log("joint '" + joint.name + "': missing effort command interface");
      return false;
    }
  }
  return true;
}

double CanFdMotorSystem::ParseDouble(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  return std::strtod(it->second.c_str(), nullptr);
}

int CanFdMotorSystem::ParseInt(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, int fallback)
{
  return static_cast<int>(ParseDouble(params, key, static_cast<double>(fallback)));
}

std::vector<hardware_interface::StateInterface> CanFdMotorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & component = info_.joints[i];
    auto & joint = joints_[i];
    for (const auto & iface : component.state_interfaces) {
      double * value = nullptr;
      if (iface.name == hardware_interface::HW_IF_POSITION) {
        value = &joint.state_position;
      } else if (iface.name == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.state_velocity;
      } else if (iface.name == hardware_interface::HW_IF_EFFORT) {
        value = &joint.state_effort;
      }
      if (value != nullptr) {
        interfaces.emplace_back(component.name, iface.name, value);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> CanFdMotorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & component = info_.joints[i];
    auto & joint = joints_[i];
    for (const auto & iface : component.command_interfaces) {
      if (iface.name == hardware_interface::HW_IF_EFFORT) {
        interfaces.emplace_back(component.name, iface.name, &joint.command_effort);
      }
    }
  }
  return interfaces;
}

bool CanFdMotorSystem::OpenBus()
{
  try {
    bus_ = SocketCanBus(interface_);
    return bus_.IsOpen();
  } catch (const std::exception & error) {
    Log(std::string("failed to open CAN interface: ") + error.what());
    return false;
  }
}

void CanFdMotorSystem::CloseBus()
{
  bus_.Close();
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!OpenBus()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }
  // Configure every 25/30 mm drive into MIT mode without running it.
  for (const auto & joint : joints_) {
    if (joint.type == MotorType::kWrist) {
      try {
        bus_.Send(
          wrist::MotorControlId(joint.can_node_id),
          wrist::PackMotorControl(false, false, wrist::kModeMit), false, true);
      } catch (const std::exception & error) {
        Log(std::string("failed to configure wrist motor: ") + error.what());
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }
  }
  SendEnable(false);
  ResetFeedbackCounters();
  commands_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_.IsOpen()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  ResetFeedbackCounters();
  commands_active_ = false;
  feedback_error_logged_ = false;
  if (!SendEnable(true)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void CanFdMotorSystem::ResetFeedbackCounters()
{
  for (auto & joint : joints_) {
    joint.feedback_miss_cycles = 0;
  }
}

bool CanFdMotorSystem::SendEnable(bool enabled)
{
  bool ok = true;
  for (const auto & joint : joints_) {
    try {
      if (joint.type == MotorType::kRp) {
        ok = bus_.Send(
          rp::kEnableId, rp::PackEnable(enabled), false, true) && ok;
      } else {
        ok = bus_.Send(
          wrist::MotorControlId(joint.can_node_id),
          wrist::PackMotorControl(enabled, false, wrist::kModeMit), false, true) && ok;
      }
    } catch (const std::exception & error) {
      Log(std::string("enable/disable send failed: ") + error.what());
      return false;
    }
  }
  return ok;
}

void CanFdMotorSystem::SendZeroCommands()
{
  if (!bus_.IsOpen()) {
    return;
  }
  bool has_rp = false;
  for (const auto & joint : joints_) {
    if (joint.type == MotorType::kRp) {
      has_rp = true;
      continue;
    }
    try {
      bus_.Send(
        wrist::MitCommandId(joint.can_node_id),
        wrist::PackMitCommand(wrist::MitCommand{}), true, true);
    } catch (const std::exception &) {
      // Ignore transient send errors during safe stop; keep going.
    }
  }
  if (has_rp) {
    try {
      const std::vector<rp::MitCommand> neutral(4, rp::MitCommand{});
      bus_.Send(rp::kMitBroadcastId, rp::PackBroadcast(neutral), true, true);
    } catch (const std::exception &) {
    }
  }
}

void CanFdMotorSystem::SendCommands()
{
  if (!bus_.IsOpen()) {
    return;
  }

  std::unordered_map<int, rp::MitCommand> rp_slots;
  bool has_rp = false;
  for (const auto & joint : joints_) {
    const double effort =
      std::min(std::max(joint.command_effort, joint.effort_min), joint.effort_max);
    const double motor_effort = joint.direction * effort * joint.effort_scale;
    if (joint.type == MotorType::kRp) {
      has_rp = true;
      if (joint.has_feedback) {
        rp_slots[joint.can_node_id] = rp::MitCommand{
          joint.last_motor_position, 0.0, joint.kp, joint.kd, motor_effort};
      } else {
        // No position feedback yet: send a zero-gain / zero-ff coast slot.
        rp_slots[joint.can_node_id] = rp::MitCommand{};
      }
      continue;
    }

    wrist::MitCommand command{};
    if (joint.has_feedback) {
      command = wrist::MitCommand{
        joint.last_motor_position, 0.0, joint.kp, joint.kd, motor_effort};
    }
    bus_.Send(
      wrist::MitCommandId(joint.can_node_id),
      wrist::PackMitCommand(command), true, true);
  }

  if (has_rp) {
    std::vector<rp::MitCommand> slots(8, rp::MitCommand{});
    for (const auto & [node, command] : rp_slots) {
      slots[node - 1] = command;
    }
    bus_.Send(rp::kMitBroadcastId, rp::PackBroadcast(slots), true, true);
  }
}

hardware_interface::return_type CanFdMotorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto frames = bus_.RecvAvailable();

  for (auto & joint : joints_) {
    joint.feedback_this_cycle = false;
  }

  for (const auto & frame : frames) {
    if (frame.is_extended_id) {
      continue;
    }
    const uint32_t id = frame.arbitration_id;
    for (auto & joint : joints_) {
      if (joint.type == MotorType::kRp) {
        if (id == static_cast<uint32_t>(joint.can_node_id) && frame.data.size() == 12) {
          try {
            const auto state = rp::UnpackState(frame.data);
            joint.last_motor_position = state.position;
            joint.last_motor_velocity = state.velocity;
            joint.last_motor_effort = state.torque;
            joint.has_feedback = true;
            joint.feedback_this_cycle = true;
            joint.state_position = (state.position - joint.zero_offset) * joint.direction;
            joint.state_velocity = state.velocity * joint.direction;
            joint.state_effort = state.torque * joint.direction / joint.effort_scale;
          } catch (const std::exception &) {
          }
        }
      } else {
        if (id == wrist::MitReplyId(joint.can_node_id) && frame.data.size() == 12) {
          const auto reply = wrist::UnpackMitReply(frame.data);
          joint.last_motor_position = reply.position;
          joint.last_motor_velocity = reply.velocity;
          joint.last_motor_effort = reply.current;
          joint.has_feedback = true;
          joint.feedback_this_cycle = true;
          joint.state_position = (reply.position - joint.zero_offset) * joint.direction;
          joint.state_velocity = reply.velocity * joint.direction;
          joint.state_effort = reply.current * joint.direction / joint.effort_scale;
        } else if (id == wrist::StatusId(joint.can_node_id) && frame.data.size() == 20) {
          const auto status = wrist::UnpackStatus(frame.data);
          joint.last_motor_position = status.position;
          joint.last_motor_velocity = status.velocity;
          joint.last_motor_effort = status.current;
          joint.has_feedback = true;
          joint.feedback_this_cycle = true;
          joint.state_position = (status.position - joint.zero_offset) * joint.direction;
          joint.state_velocity = status.velocity * joint.direction;
          joint.state_effort = status.current * joint.direction / joint.effort_scale;
          if (status.fault != 0 && !feedback_error_logged_) {
            char fault_hex[16];
            std::snprintf(fault_hex, sizeof(fault_hex), "0x%04X", status.fault);
            Log("motor fault bits " + std::string(fault_hex) +
                " reported by node " + std::to_string(joint.can_node_id));
          }
        }
      }
    }
  }

  // Per-joint feedback watchdog.
  for (auto & joint : joints_) {
    if (joint.feedback_this_cycle) {
      joint.feedback_miss_cycles = 0;
    } else {
      ++joint.feedback_miss_cycles;
    }
    if (joint.feedback_miss_cycles > feedback_timeout_cycles_) {
      if (!feedback_error_logged_) {
        Log(
          "no feedback from joint '" + joint.name + "' for " +
          std::to_string(joint.feedback_miss_cycles) + " cycles; triggering error");
        feedback_error_logged_ = true;
      }
      SafeStop();
      return hardware_interface::return_type::ERROR;
    }
  }

  // Command watchdog: if the controllers stop writing, drop to zero commands.
  if (commands_active_) {
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - last_write_time_)
                           .count();
    if (elapsed > command_timeout_ms_) {
      SendZeroCommands();
      commands_active_ = false;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanFdMotorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!bus_.IsOpen()) {
    return hardware_interface::return_type::ERROR;
  }
  SendCommands();
  last_write_time_ = std::chrono::steady_clock::now();
  commands_active_ = true;
  return hardware_interface::return_type::OK;
}

void CanFdMotorSystem::SafeStop()
{
  SendZeroCommands();
  SendEnable(false);
  commands_active_ = false;
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  SafeStop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  SafeStop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  SafeStop();
  CloseBus();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanFdMotorSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  CloseBus();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void CanFdMotorSystem::Log(const std::string & message) const
{
  std::fprintf(stderr, "[upper_body_canfd_hardware:%s] %s\n", info_.name.c_str(), message.c_str());
}

}  // namespace upper_body_canfd_hardware

PLUGINLIB_EXPORT_CLASS(
  upper_body_canfd_hardware::CanFdMotorSystem, hardware_interface::SystemInterface)
