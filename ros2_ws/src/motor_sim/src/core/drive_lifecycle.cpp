#include "motor_sim/core/drive_lifecycle.hpp"

#include <algorithm>
#include <stdexcept>

namespace motor_sim::core {

std::string_view drive_state_name(const DriveState state) noexcept {
  switch (state) {
    case DriveState::BrakeEngaged:
      return "brake_engaged";
    case DriveState::AwaitingFirstCommand:
      return "awaiting_first_command";
    case DriveState::ReadyForControl:
      return "ready_for_control";
    case DriveState::MotorControlActive:
      return "motor_control_active";
    case DriveState::FaultLatched:
      return "fault_latched";
  }
  return "brake_engaged";
}

DriveLifecycle::DriveLifecycle(const std::size_t axis_count)
    : enabled_(axis_count, false), fresh_command_(axis_count, false) {
  if (axis_count == 0U) {
    throw std::invalid_argument("drive lifecycle requires at least one axis");
  }
}

void DriveLifecycle::set_enabled(const std::size_t axis_index,
                                 const bool enabled) {
  if (axis_index >= enabled_.size()) {
    throw std::out_of_range("drive lifecycle axis index is out of range");
  }
  const bool changed = enabled_[axis_index] != enabled;
  enabled_[axis_index] = enabled;
  if (!changed) {
    return;
  }
  fresh_command_[axis_index] = false;
  if (!enabled) {
    std::fill(fresh_command_.begin(), fresh_command_.end(), false);
    control_gate_ = false;
    disable_brake_ = true;
  } else if (all_enabled() && !fault_brake_ && !fault_latched_) {
    disable_brake_ = false;
  }
}

void DriveLifecycle::observe_fresh_command(const std::size_t axis_index) {
  if (axis_index >= enabled_.size()) {
    throw std::out_of_range("drive lifecycle axis index is out of range");
  }
  if (enabled_[axis_index] && !fault_brake_ && !fault_latched_) {
    fresh_command_[axis_index] = true;
  }
}

void DriveLifecycle::set_control_gate(const bool open) noexcept {
  control_gate_ = open && state() == DriveState::ReadyForControl;
}

void DriveLifecycle::enter_fault(const bool latched) noexcept {
  control_gate_ = false;
  fault_brake_ = true;
  fault_latched_ = latched;
  std::fill(fresh_command_.begin(), fresh_command_.end(), false);
}

void DriveLifecycle::reset_to_brake() noexcept {
  control_gate_ = false;
  fault_brake_ = false;
  fault_latched_ = false;
  std::fill(fresh_command_.begin(), fresh_command_.end(), false);
}

DriveState DriveLifecycle::state() const noexcept {
  if (fault_latched_) {
    return DriveState::FaultLatched;
  }
  if (fault_brake_ || disable_brake_ || all_disabled()) {
    return DriveState::BrakeEngaged;
  }
  if (!all_enabled() || !all_fresh()) {
    return DriveState::AwaitingFirstCommand;
  }
  return control_gate_ ? DriveState::MotorControlActive
                       : DriveState::ReadyForControl;
}

bool DriveLifecycle::all_disabled() const noexcept {
  return std::none_of(enabled_.begin(), enabled_.end(),
                      [](const bool enabled) { return enabled; });
}

bool DriveLifecycle::all_enabled() const noexcept {
  return std::all_of(enabled_.begin(), enabled_.end(),
                     [](const bool enabled) { return enabled; });
}

bool DriveLifecycle::all_fresh() const noexcept {
  return std::all_of(fresh_command_.begin(), fresh_command_.end(),
                     [](const bool fresh) { return fresh; });
}

}  // namespace motor_sim::core
