#pragma once

#include <cstddef>
#include <string_view>
#include <vector>

namespace motor_sim::core {

enum class DriveState {
  BrakeEngaged,
  AwaitingFirstCommand,
  ReadyForControl,
  MotorControlActive,
  FaultLatched,
};

std::string_view drive_state_name(DriveState state) noexcept;

// 与机器人拓扑无关的整组驱动生命周期。只有全部配置轴在当前使能周期内
// 收到新鲜命令，并且下游 controller 明确打开 gate，才允许电机控制生效。
class DriveLifecycle {
public:
  explicit DriveLifecycle(std::size_t axis_count);

  void set_enabled(std::size_t axis_index, bool enabled);
  void observe_fresh_command(std::size_t axis_index);
  void set_control_gate(bool open) noexcept;
  void enter_fault(bool latched) noexcept;
  void reset_to_brake() noexcept;

  DriveState state() const noexcept;
  bool all_disabled() const noexcept;

private:
  bool all_enabled() const noexcept;
  bool all_fresh() const noexcept;

  std::vector<bool> enabled_;
  std::vector<bool> fresh_command_;
  bool control_gate_{false};
  bool disable_brake_{false};
  bool fault_brake_{false};
  bool fault_latched_{false};
};

}  // namespace motor_sim::core
