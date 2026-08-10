#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "upper_body_canfd_hardware/canfd_socket.hpp"

namespace upper_body_canfd_hardware {

enum class MotorType : uint8_t { kRp, kWrist };

struct JointConfig {
  std::string name;
  MotorType type = MotorType::kWrist;
  int can_node_id = 1;
  double direction = 1.0;
  double zero_offset = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double effort_scale = 1.0;
  double effort_min = -60.0;
  double effort_max = 60.0;

  // Runtime state.
  double command_effort = 0.0;
  double state_position = 0.0;
  double state_velocity = 0.0;
  double state_effort = 0.0;

  // Latest decoded motor-frame feedback (position rad / velocity rad/s /
  // effort: Nm for RP, A for wrist drives).
  double last_motor_position = 0.0;
  double last_motor_velocity = 0.0;
  double last_motor_effort = 0.0;
  bool has_feedback = false;
  int feedback_miss_cycles = 0;
  bool feedback_this_cycle = false;
};

/// ros2_control system hardware for one CAN-FD upper-body motor bus.
///
/// Command interface: `effort` per joint (joint frame). State interfaces:
/// `position`, `velocity`, `effort`. `write()` converts effort into MIT frames
/// (RP broadcast ID 0x80 + per-node 25/30 mm frames); `read()` decodes the
/// motor replies into the state interfaces. Gains (kp/kd), direction, zero
/// offset and effort scale are per-joint URDF parameters.
class CanFdMotorSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  bool ParseParameters();
  bool OpenBus();
  void CloseBus();
  bool SendEnable(bool enabled);
  void SendZeroCommands();
  void SendCommands();
  void SafeStop();
  void ResetFeedbackCounters();
  void Log(const std::string & message) const;

  static double ParseDouble(
    const std::unordered_map<std::string, std::string> & params,
    const std::string & key, double fallback);
  static int ParseInt(
    const std::unordered_map<std::string, std::string> & params,
    const std::string & key, int fallback);

  SocketCanBus bus_;
  std::vector<JointConfig> joints_;

  std::string interface_;
  bool allow_hardware_ = false;
  int feedback_timeout_cycles_ = 50;
  double command_timeout_ms_ = 150.0;
  bool commands_active_ = false;
  bool feedback_error_logged_ = false;
  std::chrono::steady_clock::time_point last_write_time_{};
};

}  // namespace upper_body_canfd_hardware
