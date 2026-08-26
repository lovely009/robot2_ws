#include "motor_sim/core/frame_router.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace motor_sim::core {
namespace {

std::vector<uint8_t> payload_bytes(const protocol::CanFrame &frame) {
  return {frame.data.begin(), frame.data.begin() + frame.size};
}

bool is_control_frame(const protocol::CanFrame &frame,
                      const MotorConfig &motor) {
  if (motor.protocol == "rp") {
    return frame.id == protocol::kRpEnableId && !frame.fd && frame.size == 8U;
  }
  return frame.id == protocol::vaxor_control_id(motor.id) && !frame.fd &&
         frame.size == 8U;
}

}  // namespace

FrameRouter::FrameRouter(SystemConfig config)
    : config_(std::move(config)),
      safety_(config_.motors.size(),
              std::chrono::milliseconds(config_.timing.command_timeout_ms),
              std::chrono::milliseconds(2)),
      commands_(config_.motors.size()),
      enabled_(config_.motors.size(), false),
      modes_(config_.motors.size(), protocol::CommandMode::Mit),
      last_states_(config_.motors.size()) {
  if (config_.motors.empty()) {
    throw std::invalid_argument("frame router requires configured motors");
  }
  protocols_.reserve(config_.motors.size());
  transmissions_.reserve(config_.motors.size());
  plants_.reserve(config_.motors.size());
  for (const auto &motor : config_.motors) {
    protocols_.push_back(protocol::make_motor_protocol(motor.protocol));
    transmissions_.emplace_back(motor.transmission);
    plants_.emplace_back(motor.plant);
  }
}

RouterResult FrameRouter::handle(const std::string &bus,
                                 const protocol::CanFrame &frame,
                                 const TimePoint now) {
  if (safety_.latched()) {
    return {RouterStatus::SafetyFault, {}};
  }
  bool bus_exists = false;
  for (const auto &configured_bus : config_.buses) {
    bus_exists = bus_exists || configured_bus.name == bus;
  }
  if (!bus_exists) {
    safety_.latch(Fault::WrongBus);
    return {RouterStatus::WrongBus, {}};
  }

  struct PendingCommand {
    std::size_t index;
    protocol::MotorCommand command;
    bool control;
  };
  std::vector<PendingCommand> pending;
  for (std::size_t index = 0U; index < config_.motors.size(); ++index) {
    const auto &motor = config_.motors[index];
    if (motor.bus != bus) {
      continue;
    }
    const auto decoded = protocols_[index]->decode_command(frame, motor.id);
    if (!decoded.has_value()) {
      continue;
    }
    const bool control = is_control_frame(frame, motor);
    auto joint_command = control ? *decoded : to_joint_command(index, *decoded);
    if (!control && joint_command.mode != protocol::CommandMode::Velocity &&
        (!std::isfinite(joint_command.position) ||
         joint_command.position < motor.limits.command_lower_rad ||
         joint_command.position > motor.limits.command_upper_rad)) {
      safety_.latch(Fault::CommandLimit);
      return {RouterStatus::SafetyFault, {}};
    }
    pending.push_back({index, joint_command, control});
  }

  if (pending.empty()) {
    if (frame_belongs_to_other_bus(bus, frame)) {
      safety_.latch(Fault::WrongBus);
      return {RouterStatus::WrongBus, {}};
    }
    return {RouterStatus::Ignored, {}};
  }

  safety_.observe_valid_command(bus, frame.id, payload_bytes(frame), now);
  if (safety_.latched()) {
    return {RouterStatus::SafetyFault, {}};
  }

  RouterResult result{RouterStatus::Applied, {}};
  result.updated_motor_indices.reserve(pending.size());
  for (const auto &item : pending) {
    if (item.control) {
      enabled_[item.index] = item.command.enabled;
      modes_[item.index] = item.command.mode;
      commands_[item.index].enabled = item.command.enabled;
      commands_[item.index].mode = item.command.mode;
      if (item.command.set_zero &&
          config_.motors[item.index].protocol == "motor_2530") {
        transmissions_[item.index].set_zero_at_joint(
            last_states_[item.index].position_rad);
        commands_[item.index].position = last_states_[item.index].position_rad;
      }
    } else {
      commands_[item.index] = item.command;
      commands_[item.index].enabled = enabled_[item.index];
    }
    result.updated_motor_indices.push_back(item.index);
  }
  return result;
}

std::vector<double>
FrameRouter::compute_efforts(const std::vector<JointSample> &states,
                             const double dt_s, const TimePoint now) {
  if (states.size() != config_.motors.size()) {
    throw std::invalid_argument("joint sample count does not match motor count");
  }
  last_states_ = states;
  if (safety_.tick(now).latched) {
    return std::vector<double>(config_.motors.size(), 0.0);
  }
  std::vector<double> efforts(config_.motors.size(), 0.0);
  for (std::size_t index = 0U; index < config_.motors.size(); ++index) {
    efforts[index] = plants_[index].step(commands_[index], states[index], dt_s)
                         .effort_nm;
  }
  return efforts;
}

void FrameRouter::observe_disconnect(const std::string &bus) {
  safety_.observe_disconnect(bus);
}

protocol::MotorCommand FrameRouter::to_joint_command(
    const std::size_t index,
    const protocol::MotorCommand &wire_command) const {
  auto command = wire_command;
  if (config_.motors[index].protocol == "rp") {
    command.use_nominal_gains = true;
    return command;
  }
  command.mode = modes_[index];
  const auto &plant = config_.motors[index].plant;
  command.position =
      transmissions_[index].motor_to_joint_position(wire_command.position);
  command.velocity =
      transmissions_[index].motor_to_joint_velocity(wire_command.velocity);
  if (command.mode == protocol::CommandMode::Mit) {
    const auto &transmission = config_.motors[index].transmission;
    const double gain_scale = plant.torque_constant_nm_per_a *
                              transmission.gear_ratio *
                              transmission.gear_ratio *
                              transmission.efficiency;
    command.kp = wire_command.kp * gain_scale;
    command.kd = wire_command.kd * gain_scale;
  } else {
    command.kp = 0.0;
    command.kd = 0.0;
    command.use_nominal_gains = true;
  }
  const double motor_current =
      std::clamp(wire_command.feedforward, -plant.current_limit_a,
                 plant.current_limit_a);
  const double motor_torque =
      motor_current * plant.torque_constant_nm_per_a;
  command.feedforward =
      transmissions_[index].motor_torque_to_joint_effort(motor_torque);
  return command;
}

protocol::MotorFeedback FrameRouter::motor_feedback(
    const std::size_t index, const JointSample &state,
    const double joint_effort_nm) const {
  if (index >= config_.motors.size()) {
    throw std::out_of_range("motor feedback index is out of range");
  }
  const auto &motor = config_.motors[index];
  protocol::MotorFeedback feedback;
  feedback.motor_id = motor.id;
  feedback.enabled = enabled_[index];
  feedback.mode = modes_[index];
  if (motor.protocol == "rp") {
    feedback.position = state.position_rad;
    feedback.velocity = state.velocity_rad_s;
    feedback.effort = joint_effort_nm;
    return feedback;
  }
  feedback.position =
      transmissions_[index].joint_to_motor_position(state.position_rad);
  feedback.velocity =
      transmissions_[index].joint_to_motor_velocity(state.velocity_rad_s);
  const double motor_torque =
      transmissions_[index].joint_effort_to_motor_torque(joint_effort_nm);
  feedback.effort = std::clamp(
      motor_torque / motor.plant.torque_constant_nm_per_a,
      -motor.plant.current_limit_a, motor.plant.current_limit_a);
  return feedback;
}

bool FrameRouter::frame_belongs_to_other_bus(
    const std::string &bus, const protocol::CanFrame &frame) const {
  for (std::size_t index = 0U; index < config_.motors.size(); ++index) {
    const auto &motor = config_.motors[index];
    if (motor.bus == bus) {
      continue;
    }
    if (protocols_[index]->decode_command(frame, motor.id).has_value()) {
      return true;
    }
  }
  return false;
}

}  // namespace motor_sim::core
