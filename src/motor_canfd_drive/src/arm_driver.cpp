#include "motor_canfd_drive/arm_driver.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace motor_canfd_drive {

ArmDriver::ArmDriver(ArmDriverConfig config) : config_(std::move(config)) {
  validate_config();
}

void ArmDriver::validate_config() const {
  if (config_.interface_name.empty()) {
    throw std::invalid_argument("CAN interface name cannot be empty");
  }
  if (!(config_.wrist_torque_constant_nm_per_amp > 0.0)) {
    throw std::invalid_argument("wrist torque constant must be positive");
  }

  auto rp = config_.rp_ids;
  std::sort(rp.begin(), rp.end());
  if (rp.front() < 1U || rp.back() > protocol::kRpBroadcastSlots ||
      std::adjacent_find(rp.begin(), rp.end()) != rp.end()) {
    throw std::invalid_argument("RP IDs must be unique values in [1, 8]");
  }

  auto wrist = config_.wrist_nodes;
  std::sort(wrist.begin(), wrist.end());
  if (wrist.front() < 1U || wrist.back() > 60U ||
      std::adjacent_find(wrist.begin(), wrist.end()) != wrist.end()) {
    throw std::invalid_argument("wrist nodes must be unique values in [1, 60]");
  }
}

void ArmDriver::open() {
  if (!bus_) {
    bus_ = std::make_unique<SocketCan>(config_.interface_name);
  }
}

void ArmDriver::close() noexcept { bus_.reset(); }

bool ArmDriver::set_enabled(const bool enabled) {
  if (!bus_) {
    return false;
  }
  const std::array<uint8_t, 8> rp_enable{
      {static_cast<uint8_t>(enabled), 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
  bool ok = bus_->send(protocol::kRpEnableId, rp_enable.data(), rp_enable.size());
  const auto wrist_control = protocol::pack_wrist_control(enabled);
  for (const auto node : config_.wrist_nodes) {
    ok &= bus_->send(protocol::wrist_control_id(node), wrist_control.data(),
                     wrist_control.size());
  }
  return ok;
}

bool ArmDriver::write_commands(const CommandArray &commands) {
  if (!bus_) {
    return false;
  }

  std::array<uint8_t, 64> broadcast{};
  const auto neutral = protocol::pack_rp_slot({});
  for (std::size_t slot = 0; slot < protocol::kRpBroadcastSlots; ++slot) {
    std::copy(neutral.begin(), neutral.end(), broadcast.begin() + slot * 8U);
  }
  for (std::size_t index = 0; index < config_.rp_ids.size(); ++index) {
    const auto packed = protocol::pack_rp_slot(commands[index]);
    const auto slot = config_.rp_ids[index] - 1U;
    std::copy(packed.begin(), packed.end(), broadcast.begin() + slot * 8U);
  }

  bool ok = bus_->send(protocol::kRpCommandId, broadcast.data(), broadcast.size(), true);
  for (std::size_t index = 0; index < config_.wrist_nodes.size(); ++index) {
    const auto node = config_.wrist_nodes[index];
    const auto packed = protocol::pack_wrist_command(commands[4U + index]);
    ok &= bus_->send(protocol::wrist_command_id(node), packed.data(), packed.size(), true);
  }
  return ok;
}

std::size_t ArmDriver::read_feedback(StateArray &states, UpdatedArray &updated) {
  updated.fill(false);
  if (!bus_) {
    return 0U;
  }

  std::size_t count = 0U;
  CanFrame frame;
  while (bus_->receive(frame)) {
    if (frame.is_extended) {
      continue;
    }

    bool decoded = false;
    if (frame.size == 12U) {
      for (std::size_t index = 0; index < config_.rp_ids.size(); ++index) {
        if (frame.id == config_.rp_ids[index]) {
          states[index] = protocol::unpack_rp_state(frame.data.data(), frame.size);
          updated[index] = true;
          decoded = true;
          break;
        }
      }
    }

    if (!decoded) {
      for (std::size_t index = 0; index < config_.wrist_nodes.size(); ++index) {
        const auto node = config_.wrist_nodes[index];
        if ((frame.id == protocol::wrist_reply_id(node) && frame.size == 12U) ||
            (frame.id == protocol::wrist_status_id(node) && frame.size == 20U)) {
          auto state = protocol::unpack_wrist_feedback(frame.data.data(), frame.size);
          state.effort *= config_.wrist_torque_constant_nm_per_amp;
          states[4U + index] = state;
          updated[4U + index] = true;
          decoded = true;
          break;
        }
      }
    }
    if (decoded) {
      ++count;
    }
  }
  return count;
}

} // namespace motor_canfd_drive
