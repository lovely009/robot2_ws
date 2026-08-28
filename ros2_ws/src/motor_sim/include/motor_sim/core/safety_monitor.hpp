#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace motor_sim::core {

enum class Fault {
  None,
  CommandLimit,
  CommandTimeout,
  TransportDisconnected,
  DuplicateController,
  WrongBus,
};

enum class ResetFaultResult {
  Cleared,
  NotLatched,
  AxesStillEnabled,
  TransportDisconnected,
  ConditionActive,
};

struct MotorSafetyStatus {
  bool enabled{false};
  bool timed_out{false};
  std::optional<std::chrono::milliseconds> command_age;
};

struct SafetySnapshot {
  bool latched{false};
  bool rearm_required{false};
  Fault fault{Fault::None};
  std::vector<double> efforts;
  std::vector<MotorSafetyStatus> motor_status;
};

class SafetyMonitor {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  // latching_enabled=false 仅用于本机调试：所有故障只记录不锁存，
  // 输出不清零；默认 true 保持完整安全语义。
  SafetyMonitor(std::size_t axis_count,
                std::chrono::milliseconds command_timeout,
                std::chrono::milliseconds duplicate_window,
                bool latching_enabled = true);

  void set_enabled(std::size_t motor_index, bool enabled, TimePoint now);
  void observe_valid_command(std::size_t motor_index, const std::string &bus,
                             uint32_t can_id,
                             const std::vector<uint8_t> &payload,
                             TimePoint now);
  void observe_disconnect(const std::string &bus);
  void observe_reconnect(const std::string &bus) noexcept;
  void latch(Fault fault) noexcept;
  SafetySnapshot tick(TimePoint now);
  ResetFaultResult reset_fault() noexcept;

  bool latched() const noexcept { return fault_ != Fault::None; }
  bool rearm_required() const noexcept { return rearm_required_; }
  bool ready_for_new_enable_cycle() const noexcept;
  Fault fault() const noexcept {
    return latched() ? fault_ : detected_fault_;
  }
  SafetySnapshot snapshot() const;

private:
  struct RecentFrame {
    std::vector<uint8_t> payload;
    TimePoint received_at;
  };

  std::size_t axis_count_;
  std::chrono::milliseconds command_timeout_;
  std::chrono::milliseconds duplicate_window_;
  bool latching_enabled_;
  Fault fault_{Fault::None};
  Fault detected_fault_{Fault::None};
  bool rearm_required_{false};
  bool transport_connected_{true};
  bool new_enable_cycle_ready_{false};
  std::vector<bool> enabled_;
  std::vector<bool> disabled_since_fault_;
  std::vector<std::optional<TimePoint>> last_valid_commands_;
  std::vector<MotorSafetyStatus> last_motor_status_;
  std::unordered_map<std::string, RecentFrame> recent_frames_;
};

}  // namespace motor_sim::core
