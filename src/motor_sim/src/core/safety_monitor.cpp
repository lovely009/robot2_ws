#include "motor_sim/core/safety_monitor.hpp"

#include <stdexcept>
#include <utility>

namespace motor_sim::core {

SafetyMonitor::SafetyMonitor(
    const std::size_t axis_count,
    const std::chrono::milliseconds command_timeout,
    const std::chrono::milliseconds duplicate_window,
    const bool latching_enabled)
    : axis_count_(axis_count),
      command_timeout_(command_timeout),
      duplicate_window_(duplicate_window),
      latching_enabled_(latching_enabled),
      enabled_(axis_count, false),
      last_valid_commands_(axis_count),
      last_motor_status_(axis_count) {
  if (axis_count_ == 0U || command_timeout_.count() <= 0 ||
      duplicate_window_.count() <= 0) {
    throw std::invalid_argument("invalid safety monitor timing or axis count");
  }
}

void SafetyMonitor::set_enabled(const std::size_t motor_index,
                                const bool enabled, const TimePoint now) {
  if (motor_index >= axis_count_) {
    throw std::out_of_range("motor safety index is out of range");
  }
  enabled_[motor_index] = enabled;
  last_valid_commands_[motor_index] = enabled ? std::optional<TimePoint>(now)
                                               : std::nullopt;
}

void SafetyMonitor::observe_valid_command(
    const std::size_t motor_index, const std::string &bus,
    const uint32_t can_id,
    const std::vector<uint8_t> &payload, const TimePoint now) {
  if (motor_index >= axis_count_) {
    throw std::out_of_range("motor safety index is out of range");
  }
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
  if (enabled_[motor_index]) {
    last_valid_commands_[motor_index] = now;
  }
}

void SafetyMonitor::observe_disconnect(const std::string &) {
  latch(Fault::TransportDisconnected);
}

void SafetyMonitor::observe_reconnect(const std::string &) noexcept {
  // 故障锁存不能由串口重新出现自动清除。
}

void SafetyMonitor::latch(const Fault fault) noexcept {
  // 调试模式下所有故障路径（超时/越限/错总线/重复控制器/断线）都汇聚到
  // 这里；禁用锁存时只保留检测逻辑，不置位故障、不清零输出。
  if (!latching_enabled_) {
    return;
  }
  if (fault_ == Fault::None && fault != Fault::None) {
    fault_ = fault;
  }
}

SafetySnapshot SafetyMonitor::tick(const TimePoint now) {
  for (std::size_t index = 0U; index < axis_count_; ++index) {
    auto &status = last_motor_status_[index];
    status.enabled = enabled_[index];
    status.timed_out = false;
    status.command_age.reset();
    if (enabled_[index] && last_valid_commands_[index].has_value() &&
        now >= *last_valid_commands_[index]) {
      status.command_age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - *last_valid_commands_[index]);
      status.timed_out = *status.command_age > command_timeout_;
      if (!latched() && status.timed_out) {
        latch(Fault::CommandTimeout);
      }
    }
  }
  return snapshot();
}

SafetySnapshot SafetyMonitor::snapshot() const {
  SafetySnapshot state;
  state.latched = latched();
  state.fault = fault_;
  state.motor_status = last_motor_status_;
  if (state.latched) {
    state.efforts.assign(axis_count_, 0.0);
  }
  return state;
}

}  // namespace motor_sim::core
