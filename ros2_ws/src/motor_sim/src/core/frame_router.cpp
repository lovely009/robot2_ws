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
  // RP 已弃用；Vaxor 的独立控制帧是当前唯一支持的控制帧。
  if (motor.protocol == "cerb") {
    return false;
  }
  return frame.id == protocol::vaxor_control_id(motor.id) && !frame.fd &&
         frame.size == 8U;
}

}  // namespace

FrameRouter::FrameRouter(SystemConfig config,
                         const bool fault_latching_enabled)
    : config_(std::move(config)),
      safety_(config_.motors.size(),
              std::chrono::milliseconds(config_.timing.command_timeout_ms),
              std::chrono::milliseconds(2), fault_latching_enabled),
      lifecycle_(config_.motors.size()),
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
  bool bus_exists = false;
  for (const auto &configured_bus : config_.buses) {
    bus_exists = bus_exists || configured_bus.name == bus;
  }
  if (!bus_exists) {
    safety_.latch(Fault::WrongBus);
    lifecycle_.enter_fault(safety_.latched());
    return {RouterStatus::WrongBus, {}, {}};
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
      lifecycle_.enter_fault(safety_.latched());
      return {RouterStatus::SafetyFault, {}, {}};
    }
    pending.push_back({index, joint_command, control});
  }

  if (pending.empty()) {
    if (safety_.latched()) {
      return {RouterStatus::SafetyFault, {}, {}};
    }
    if (frame_belongs_to_other_bus(bus, frame)) {
      safety_.latch(Fault::WrongBus);
      lifecycle_.enter_fault(safety_.latched());
      return {RouterStatus::WrongBus, {}, {}};
    }
    return {RouterStatus::Ignored, {}, {}};
  }

  RouterResult result{RouterStatus::Applied, {}, {}};
  result.updated_motor_indices.reserve(pending.size());
  for (const auto &item : pending) {
    // 锁存后只接受显式失能，用于满足 reset_fault 的全轴停用前置条件；
    // 任何带使能位的帧（整臂使能、单轴恢复、运动命令）都不能清除或绕过故障。
    // 现阶段仅 Vaxor 与 CERB 仍受支持：Vaxor 走独立控制帧，CERB 随 MIT 命令帧携带使能位。
    const bool is_disable = !item.command.enabled &&
                            (item.control ||
                             config_.motors[item.index].protocol == "cerb");
    if (safety_.latched()) {
      if (!is_disable) {
        return {RouterStatus::SafetyFault, {}, {}};
      }
      safety_.set_enabled(item.index, false, now);
      lifecycle_.set_enabled(item.index, false);
      enabled_[item.index] = false;
      continue;
    }
    if (item.control) {
      safety_.set_enabled(item.index, item.command.enabled, now);
      lifecycle_.set_enabled(item.index, item.command.enabled);
      enabled_[item.index] = item.command.enabled;
      modes_[item.index] = item.command.mode;
      commands_[item.index].enabled = item.command.enabled;
      commands_[item.index].mode = item.command.mode;
      if (item.command.set_zero &&
          config_.motors[item.index].protocol == "vaxor") {
        transmissions_[item.index].set_zero_at_joint(
            last_states_[item.index].position_rad);
        commands_[item.index].position = last_states_[item.index].position_rad;
      }
    } else {
      // CERB 的使能位随 MIT 命令帧携带；必须在 observe_valid_command 之前同步
      // 使能状态，否则最新命令时间戳不会被记录（monitor 仅在已使能时记录），
      // 后续超时锁存永不触发。Vaxor/RP 的使能只来自独立控制帧，此处不变。
      if (config_.motors[item.index].protocol == "cerb") {
        safety_.set_enabled(item.index, item.command.enabled, now);
        lifecycle_.set_enabled(item.index, item.command.enabled);
        enabled_[item.index] = item.command.enabled;
      }
      safety_.observe_valid_command(item.index, bus, frame.id,
                                    payload_bytes(frame), now);
      if (safety_.latched() || safety_.rearm_required()) {
        lifecycle_.enter_fault(safety_.latched());
        return {RouterStatus::SafetyFault, {}, {}};
      }
      commands_[item.index] = item.command;
      commands_[item.index].enabled = enabled_[item.index];
      lifecycle_.observe_fresh_command(item.index);
      // Vaxor 的 MIT 命令帧要求逐帧应答（手册 3.4）；CERB 反馈报文同样由
      // 命令帧触发；RP 反馈由节点分频发送。
      if (config_.motors[item.index].protocol == "vaxor" ||
          config_.motors[item.index].protocol == "cerb") {
        result.reply_motor_indices.push_back(item.index);
      }
    }
    result.updated_motor_indices.push_back(item.index);
  }
  if (safety_.ready_for_new_enable_cycle()) {
    lifecycle_.reset_to_brake();
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
  const auto safety = safety_.tick(now);
  if (safety.latched || safety.rearm_required) {
    lifecycle_.enter_fault(safety.latched);
  }
  if (lifecycle_.state() != DriveState::MotorControlActive) {
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
  lifecycle_.enter_fault(safety_.latched());
}

ResetFaultResult FrameRouter::reset_fault() noexcept {
  const auto result = safety_.reset_fault();
  if (result == ResetFaultResult::Cleared) {
    lifecycle_.reset_to_brake();
  }
  return result;
}

protocol::MotorCommand FrameRouter::to_joint_command(
    const std::size_t index,
    const protocol::MotorCommand &wire_command) const {
  auto command = wire_command;
  // RP 已弃用，当前仅保留 Vaxor/CERB。
  // CERB MIT 的位置、速度、增益与前馈力矩都是减速器输出轴侧物理量，
  // 与关节侧一致，直接透传，不做传动换算。
  if (config_.motors[index].protocol == "cerb") {
    command.mode = wire_command.mode;
    command.use_nominal_gains = false;
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
  // CERB 反馈帧：位置为负载端（与关节侧一致，直接透传），
  // 速度为电机端（rad/s，编码器内换算为 RPM），电流由关节力矩经传动与 Kt 还原。
  if (motor.protocol == "cerb") {
    feedback.position = state.position_rad;
    feedback.velocity =
        transmissions_[index].joint_to_motor_velocity(state.velocity_rad_s);
    const double motor_torque =
        transmissions_[index].joint_effort_to_motor_torque(joint_effort_nm);
    feedback.effort = std::clamp(
        motor_torque / motor.plant.torque_constant_nm_per_a,
        -motor.plant.current_limit_a, motor.plant.current_limit_a);
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
