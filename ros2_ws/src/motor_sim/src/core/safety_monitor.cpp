#include "motor_sim/core/safety_monitor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace motor_sim::core {

namespace {

// 窗口内连续 payload 冲突达到该次数才判定"双控制器"。单个冲突不足以定罪：
// 驱动器激活时先以 2 ms 节拍发零增益轮询帧、收齐反馈后 0.1 ms 内紧接着发
// 使能帧（见 canfd_system.cpp on_activate），同一 CAN ID 的 payload 会在
// duplicate_window 内变化——那是同一个控制器的合法状态切换，不是冲突。
constexpr std::size_t kConflictsToLatch = 2U;

}  // namespace

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
      disabled_since_fault_(axis_count, false),
      last_valid_commands_(axis_count),
      last_motor_status_(axis_count) {
  if (axis_count_ == 0U || command_timeout_.count() <= 0 ||
      duplicate_window_.count() <= 0) {
    throw std::invalid_argument("invalid safety monitor timing or axis count");
  }
}

void SafetyMonitor::set_enabled(const std::size_t motor_index,
                                const bool enabled, const TimePoint) {
  if (motor_index >= axis_count_) {
    throw std::out_of_range("motor safety index is out of range");
  }
  enabled_[motor_index] = enabled;
  last_valid_commands_[motor_index] = std::nullopt;
  if (enabled) {
    new_enable_cycle_ready_ = false;
  }
  if (rearm_required_ && !enabled) {
    disabled_since_fault_[motor_index] = true;
    const bool all_explicitly_disabled =
        std::all_of(enabled_.begin(), enabled_.end(),
                    [](const bool axis_enabled) { return !axis_enabled; }) &&
        std::all_of(disabled_since_fault_.begin(),
                    disabled_since_fault_.end(),
                    [](const bool disabled) { return disabled; });
    if (all_explicitly_disabled) {
      new_enable_cycle_ready_ = true;
    }
    if (!latching_enabled_ && all_explicitly_disabled) {
      detected_fault_ = Fault::None;
      rearm_required_ = false;
      recent_frames_.clear();
      std::fill(disabled_since_fault_.begin(), disabled_since_fault_.end(),
                false);
    }
  }
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
  // 多轴广播可连续承载不同节点组，冲突判定须区分实际控制轴。
  const std::string key = bus + "\n" + std::to_string(can_id) + "\n" +
                          std::to_string(motor_index);
  const auto existing = recent_frames_.find(key);
  if (existing != recent_frames_.end() &&
      now >= existing->second.received_at &&
      now - existing->second.received_at <= duplicate_window_ &&
      existing->second.payload != payload) {
    const std::size_t conflicts = existing->second.conflict_streak + 1U;
    if (conflicts >= kConflictsToLatch) {
      latch(Fault::DuplicateController);
      return;
    }
    recent_frames_[key] = {payload, now, conflicts};
  } else {
    recent_frames_[key] = {payload, now, 0U};
  }
  if (enabled_[motor_index]) {
    last_valid_commands_[motor_index] = now;
  }
}

void SafetyMonitor::observe_disconnect(const std::string &) {
  transport_connected_ = false;
  latch(Fault::TransportDisconnected);
}

void SafetyMonitor::observe_reconnect(const std::string &) noexcept {
  // 故障锁存不能由串口重新出现自动清除。
}

void SafetyMonitor::latch(const Fault fault) noexcept {
  if (fault == Fault::None || detected_fault_ != Fault::None) {
    return;
  }
  detected_fault_ = fault;
  rearm_required_ = true;
  new_enable_cycle_ready_ = false;
  std::fill(disabled_since_fault_.begin(), disabled_since_fault_.end(), false);
  if (latching_enabled_ && fault_ == Fault::None) {
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

ResetFaultResult SafetyMonitor::reset_fault() noexcept {
  if (!latched()) {
    return ResetFaultResult::NotLatched;
  }
  if (!transport_connected_ || fault_ == Fault::TransportDisconnected) {
    return ResetFaultResult::TransportDisconnected;
  }
  if (!ready_for_new_enable_cycle()) {
    return ResetFaultResult::AxesStillEnabled;
  }
  if (detected_fault_ == Fault::None) {
    return ResetFaultResult::ConditionActive;
  }
  fault_ = Fault::None;
  detected_fault_ = Fault::None;
  rearm_required_ = false;
  new_enable_cycle_ready_ = true;
  std::fill(last_valid_commands_.begin(), last_valid_commands_.end(),
            std::nullopt);
  std::fill(disabled_since_fault_.begin(), disabled_since_fault_.end(), false);
  recent_frames_.clear();
  return ResetFaultResult::Cleared;
}

bool SafetyMonitor::ready_for_new_enable_cycle() const noexcept {
  return new_enable_cycle_ready_;
}

SafetySnapshot SafetyMonitor::snapshot() const {
  SafetySnapshot state;
  state.latched = latched();
  state.rearm_required = rearm_required_;
  state.fault = fault();
  state.motor_status = last_motor_status_;
  if (state.latched || state.rearm_required) {
    state.efforts.assign(axis_count_, 0.0);
  }
  return state;
}

}  // namespace motor_sim::core
