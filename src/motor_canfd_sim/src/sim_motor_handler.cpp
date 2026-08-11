#include "motor_canfd_sim/sim_motor_handler.hpp"

#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

namespace motor_canfd_sim {

bool RpSimMotorHandler::handle_frame(const SimHandlerContext &ctx, const std::size_t bus,
                                     const drive::CanFrame &frame,
                                     const Clock::time_point now) {
  if (frame.id == protocol::kRpEnableId && frame.size >= 1U) {
    const bool enabled = frame.data[0] != 0U;
    for (const auto motor : ctx.buses[bus].motor_indices) {
      if (ctx.driver_config.motors[motor].type == drive::kRpMotorType) {
        ctx.motor_states[motor].enabled = enabled;
      }
    }
    return true;
  }
  if (frame.id == protocol::kRpCommandId && frame.size == 64U) {
    bool has_rp = false;
    for (const auto motor : ctx.buses[bus].motor_indices) {
      const auto &config = ctx.driver_config.motors[motor];
      if (config.type != drive::kRpMotorType) {
        continue;
      }
      has_rp = true;
      // RP 的物理协议规定 ID 1..8 对应广播帧的槽 0..7。
      // 用 config.id 计算偏移，不依赖 YAML 中电机出现的先后顺序。
      const auto offset = static_cast<std::size_t>(config.id - 1U) * 8U;
      ctx.motor_states[motor].command =
          protocol::unpack_rp_slot(frame.data.data() + offset, 8U);
      ctx.motor_states[motor].last_command = now;
    }
    if (has_rp) {
      send_feedback(ctx, bus);
    }
    return true;
  }
  return false;
}

void RpSimMotorHandler::status_tick(const SimHandlerContext &) {}

void RpSimMotorHandler::send_feedback(const SimHandlerContext &ctx,
                                      const std::size_t bus) const {
  for (const auto motor : ctx.buses[bus].motor_indices) {
    const auto &config = ctx.driver_config.motors[motor];
    if (config.type != drive::kRpMotorType) {
      continue;
    }
    const auto payload = protocol::pack_rp_state(
        {ctx.samples[motor].position, ctx.samples[motor].velocity,
         ctx.samples[motor].effort});
    if (!ctx.buses[bus].socket->send(config.id, payload.data(), payload.size(), true)) {
      RCLCPP_WARN(ctx.logger, "failed to send RP simulated feedback for %s",
                  config.name.c_str());
    }
  }
}

bool Motor2530SimMotorHandler::handle_frame(const SimHandlerContext &ctx,
                                            const std::size_t bus,
                                            const drive::CanFrame &frame,
                                            const Clock::time_point now) {
  for (const auto motor : ctx.buses[bus].motor_indices) {
    const auto &config = ctx.driver_config.motors[motor];
    if (config.type != drive::kMotor2530Type) {
      continue;
    }
    if (frame.id == protocol::wrist_control_id(config.id) && frame.size == 8U) {
      ctx.plant.handle_motor_2530_control(
          ctx.motor_states[motor],
          protocol::unpack_wrist_control(frame.data.data(), frame.size),
          ctx.samples[motor]);
      return true;
    }
    if (frame.id == protocol::wrist_command_id(config.id) && frame.size == 12U) {
      auto &state = ctx.motor_states[motor];
      state.command = protocol::unpack_wrist_command(frame.data.data(), frame.size);
      state.last_command = now;
      send_reply(ctx, motor);
      return true;
    }
  }
  return false;
}

void Motor2530SimMotorHandler::status_tick(const SimHandlerContext &ctx) {
  try {
    for (std::size_t motor = 0; motor < ctx.driver_config.motors.size(); ++motor) {
      const auto &config = ctx.driver_config.motors[motor];
      if (config.type != drive::kMotor2530Type) {
        continue;
      }
      const auto &state = ctx.motor_states[motor];
      // 腕部反馈协议的第三个量是“电流 A”。这里沿用 JointState 容器传给 pack 函数，
      // 真机 driver 收到后会在 Motor2530Protocol 中再用 Kt 换算为统一的 Nm effort。
      const protocol::JointState wire_state{
          ctx.samples[motor].position - state.zero_offset, ctx.samples[motor].velocity,
          state.current_a};
      const auto payload = protocol::pack_wrist_status(
          config.id, wire_state, 12.0, 0U, state.enabled ? 4U : 3U, 0U,
          state.mode, 0x01U);
      auto &socket = *ctx.buses[ctx.motor_to_bus[motor]].socket;
      if (!socket.send(protocol::wrist_status_id(config.id), payload.data(),
                       payload.size(), true)) {
        RCLCPP_WARN(ctx.logger, "failed to send 25/30 mm simulated status for %s",
                    config.name.c_str());
      }
    }
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(ctx.logger, "status publisher failed: %s", exception.what());
  }
}

void Motor2530SimMotorHandler::send_reply(const SimHandlerContext &ctx,
                                          const std::size_t motor) const {
  const auto &config = ctx.driver_config.motors[motor];
  const auto &state = ctx.motor_states[motor];
  const protocol::JointState wire_state{
      ctx.samples[motor].position - state.zero_offset, ctx.samples[motor].velocity,
      state.current_a};
  const auto payload = protocol::pack_wrist_reply(config.id, wire_state);
  auto &socket = *ctx.buses[ctx.motor_to_bus[motor]].socket;
  if (!socket.send(protocol::wrist_reply_id(config.id), payload.data(), payload.size(),
                   true)) {
    RCLCPP_WARN(ctx.logger, "failed to send 25/30 mm simulated reply for %s",
                config.name.c_str());
  }
}

void SimMotorHandlerRegistry::register_factory(std::string type, Factory factory) {
  factories_[std::move(type)] = std::move(factory);
}

bool SimMotorHandlerRegistry::contains(const std::string &type) const noexcept {
  return factories_.find(type) != factories_.end();
}

std::unique_ptr<SimMotorHandler>
SimMotorHandlerRegistry::create(const std::string &type) const {
  const auto it = factories_.find(type);
  if (it == factories_.end()) {
    throw std::runtime_error("no simulated motor handler registered for type '" + type +
                             "'");
  }
  return it->second();
}

SimMotorHandlerRegistry SimMotorHandlerRegistry::with_builtin_handlers() {
  SimMotorHandlerRegistry registry;
  registry.register_factory(drive::kRpMotorType,
                            [] { return std::make_unique<RpSimMotorHandler>(); });
  registry.register_factory(drive::kMotor2530Type,
                            [] { return std::make_unique<Motor2530SimMotorHandler>(); });
  return registry;
}

} // namespace motor_canfd_sim
