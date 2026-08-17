#include "motor_canfd_sim/sim_frame_decode.hpp"

namespace motor_canfd_sim {

SimFrameResult decode_rp_frame(const drive::DriverConfig &config,
                               const std::vector<std::size_t> &bus_motors,
                               const drive::CanFrame &frame) {
  SimFrameResult result;
  if (frame.id == protocol::kRpEnableId && frame.size >= 1U) {
    const bool enabled = frame.data[0] != 0U;
    for (const auto motor : bus_motors) {
      if (config.motors[motor].type != drive::kRpMotorType) {
        continue;
      }
      SimFrameUpdate update;
      update.motor = motor;
      update.set_enabled = true;
      update.enabled = enabled;
      result.updates.push_back(update);
    }
    result.consumed = true;
    return result;
  }
  if (frame.id == protocol::kRpCommandId && frame.size == 64U) {
    for (const auto motor : bus_motors) {
      const auto &motor_config = config.motors[motor];
      if (motor_config.type != drive::kRpMotorType) {
        continue;
      }
      // RP 的物理协议规定 ID 1..8 对应广播帧的槽 0..7。
      // 用 config.id 计算偏移，不依赖 YAML 中电机出现的先后顺序。
      const auto offset = static_cast<std::size_t>(motor_config.id - 1U) * 8U;
      SimFrameUpdate update;
      update.motor = motor;
      update.set_command = true;
      update.command =
          protocol::unpack_rp_slot(frame.data.data() + offset, 8U);
      result.updates.push_back(update);
      result.needs_feedback = true;
    }
    result.consumed = true;
    return result;
  }
  return result;
}

SimFrameResult decode_motor_2530_frame(const drive::DriverConfig &config,
                                       const std::vector<std::size_t> &bus_motors,
                                       const drive::CanFrame &frame) {
  SimFrameResult result;
  for (const auto motor : bus_motors) {
    const auto &motor_config = config.motors[motor];
    if (motor_config.type != drive::kVaxorType) {
      continue;
    }
    if (frame.id == protocol::wrist_control_id(motor_config.id) &&
        frame.size == 8U) {
      SimFrameUpdate update;
      update.motor = motor;
      update.set_control = true;
      update.control =
          protocol::unpack_wrist_control(frame.data.data(), frame.size);
      result.updates.push_back(update);
      result.consumed = true;
      return result;
    }
    if (frame.id == protocol::wrist_command_id(motor_config.id) &&
        frame.size == 12U) {
      SimFrameUpdate update;
      update.motor = motor;
      update.set_command = true;
      update.command =
          protocol::unpack_wrist_command(frame.data.data(), frame.size);
      result.updates.push_back(update);
      result.needs_feedback = true;
      result.consumed = true;
      return result;
    }
  }
  return result;
}

} // namespace motor_canfd_sim
