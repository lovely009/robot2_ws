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

struct MotorSafetyStatus {
  bool enabled{false};
  bool timed_out{false};
  std::optional<std::chrono::milliseconds> command_age;
};

struct SafetySnapshot {
  bool latched{false};
  Fault fault{Fault::None};
  std::vector<double> efforts;
  std::vector<MotorSafetyStatus> motor_status;
};

class SafetyMonitor {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  SafetyMonitor(std::size_t axis_count,
                std::chrono::milliseconds command_timeout,
                std::chrono::milliseconds duplicate_window);

  void set_enabled(std::size_t motor_index, bool enabled, TimePoint now);
  void observe_valid_command(std::size_t motor_index, const std::string &bus,
                             uint32_t can_id,
                             const std::vector<uint8_t> &payload,
                             TimePoint now);
  void observe_disconnect(const std::string &bus);
  void observe_reconnect(const std::string &bus) noexcept;
  void latch(Fault fault) noexcept;
  SafetySnapshot tick(TimePoint now);

  bool latched() const noexcept { return fault_ != Fault::None; }
  Fault fault() const noexcept { return fault_; }
  SafetySnapshot snapshot() const;

private:
  struct RecentFrame {
    std::vector<uint8_t> payload;
    TimePoint received_at;
  };

  std::size_t axis_count_;
  std::chrono::milliseconds command_timeout_;
  std::chrono::milliseconds duplicate_window_;
  Fault fault_{Fault::None};
  std::vector<bool> enabled_;
  std::vector<std::optional<TimePoint>> last_valid_commands_;
  std::vector<MotorSafetyStatus> last_motor_status_;
  std::unordered_map<std::string, RecentFrame> recent_frames_;
};

}  // namespace motor_sim::core
