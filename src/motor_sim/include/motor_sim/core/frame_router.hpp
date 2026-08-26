#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "motor_sim/core/config.hpp"
#include "motor_sim/core/plant_model.hpp"
#include "motor_sim/core/safety_monitor.hpp"
#include "motor_sim/core/transmission.hpp"
#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::core {

namespace protocol = ::motor_sim::protocol;

enum class RouterStatus { Applied, Ignored, WrongBus, SafetyFault };

struct RouterResult {
  RouterStatus status{RouterStatus::Ignored};
  std::vector<std::size_t> updated_motor_indices;
  // 手册一问一答语义：本帧命中了哪些 Vaxor 电机的 MIT 命令，
  // 节点层需立即回发 FeedbackKind::Reply 帧。
  std::vector<std::size_t> reply_motor_indices;
};

class FrameRouter {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  // fault_latching_enabled=false 仅用于本机调试，默认保持锁存安全语义。
  explicit FrameRouter(SystemConfig config,
                       bool fault_latching_enabled = true);
  RouterResult handle(const std::string &bus,
                      const protocol::CanFrame &frame, TimePoint now);
  std::vector<double> compute_efforts(const std::vector<JointSample> &states,
                                      double dt_s, TimePoint now);
  void observe_disconnect(const std::string &bus);
  protocol::MotorFeedback motor_feedback(std::size_t index,
                                         const JointSample &state,
                                         double joint_effort_nm) const;

  SafetySnapshot safety_state() const { return safety_.snapshot(); }
  const SystemConfig &config() const noexcept { return config_; }

private:
  protocol::MotorCommand to_joint_command(
      std::size_t index, const protocol::MotorCommand &wire_command) const;
  bool frame_belongs_to_other_bus(const std::string &bus,
                                  const protocol::CanFrame &frame) const;

  SystemConfig config_;
  SafetyMonitor safety_;
  std::vector<std::unique_ptr<protocol::MotorProtocol>> protocols_;
  std::vector<Transmission> transmissions_;
  std::vector<MotorPlantModel> plants_;
  std::vector<protocol::MotorCommand> commands_;
  std::vector<bool> enabled_;
  std::vector<protocol::CommandMode> modes_;
  std::vector<JointSample> last_states_;
};

}  // namespace motor_sim::core
