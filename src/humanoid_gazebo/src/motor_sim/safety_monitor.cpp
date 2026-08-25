#include "humanoid_gazebo/motor_sim/safety_monitor.hpp"

#include <stdexcept>
#include <utility>

namespace humanoid_gazebo::motor_sim {

SafetyMonitor::SafetyMonitor(
    const std::size_t axis_count,
    const std::chrono::milliseconds command_timeout,
    const std::chrono::milliseconds duplicate_window)
    : axis_count_(axis_count),
      command_timeout_(command_timeout),
      duplicate_window_(duplicate_window) {
  if (axis_count_ == 0U || command_timeout_.count() <= 0 ||
      duplicate_window_.count() <= 0) {
    throw std::invalid_argument("invalid safety monitor timing or axis count");
  }
}

void SafetyMonitor::observe_valid_command(
    const std::string &bus, const uint32_t can_id,
    const std::vector<uint8_t> &payload, const TimePoint now) {
  if (latched()) {
    return;
  }
  const std::string key = bus + "\n" + std::to_string(can_id);
  const auto existing = recent_frames_.find(key);
  if (existing != recent_frames_.end() &&
      now >= existing->second.received_at &&
      now - existing->second.received_at <= duplicate_window_ &&
      existing->second.payload != payload) {
    latch(Fault::DuplicateController);
    return;
  }
  recent_frames_[key] = {payload, now};
  last_valid_command_ = now;
}

void SafetyMonitor::observe_disconnect(const std::string &) {
  latch(Fault::TransportDisconnected);
}

void SafetyMonitor::observe_reconnect(const std::string &) noexcept {
  // 故障锁存不能由串口重新出现自动清除。
}

void SafetyMonitor::latch(const Fault fault) noexcept {
  if (fault_ == Fault::None && fault != Fault::None) {
    fault_ = fault;
  }
}

SafetySnapshot SafetyMonitor::tick(const TimePoint now) {
  if (!latched() && last_valid_command_.has_value() &&
      now >= *last_valid_command_ &&
      now - *last_valid_command_ > command_timeout_) {
    latch(Fault::CommandTimeout);
  }
  return snapshot();
}

SafetySnapshot SafetyMonitor::snapshot() const {
  SafetySnapshot state;
  state.latched = latched();
  state.fault = fault_;
  if (state.latched) {
    state.efforts.assign(axis_count_, 0.0);
  }
  return state;
}

}  // namespace humanoid_gazebo::motor_sim
