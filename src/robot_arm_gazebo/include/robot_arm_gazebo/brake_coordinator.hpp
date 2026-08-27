#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <memory>
#include <string>
#include <string_view>

namespace robot_arm_gazebo {

enum class CoordinatorAction {
  None,
  PublishGateFalse,
  PublishGateTrue,
  ActivateController,
  DeactivateController,
  FailFast,
};

enum class SwitchOperation { Activate, Deactivate };

// 纯状态机负责保证切换顺序；ROS 节点只执行它返回的发布和 service 动作。
class BrakeCoordinatorStateMachine {
public:
  CoordinatorAction on_drive_state(std::string_view state) noexcept;
  CoordinatorAction on_switch_result(SwitchOperation operation,
                                     bool success) noexcept;
  CoordinatorAction on_switch_timeout() noexcept;

  bool gate_open() const noexcept { return gate_open_; }
  bool controller_active() const noexcept { return controller_active_; }
  bool brake_engaged() const noexcept { return brake_engaged_; }

private:
  bool is_known_state(std::string_view state) const noexcept;
  bool is_braking_state(std::string_view state) const noexcept;

  std::optional<SwitchOperation> pending_switch_;
  std::string drive_state_{"brake_engaged"};
  bool received_drive_state_{false};
  bool awaiting_zero_confirmation_{false};
  bool gate_open_{false};
  bool controller_active_{false};
  bool brake_engaged_{true};
};

class BrakeCoordinator final : public rclcpp::Node {
public:
  explicit BrakeCoordinator(const rclcpp::NodeOptions &options);

  bool shutdown_requested() const noexcept { return shutdown_requested_; }

private:
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  void drive_state_callback(const std_msgs::msg::String &message);
  void handle_action(CoordinatorAction action, const std::string &reason);
  void publish_gate(bool open);
  void request_switch(SwitchOperation operation);
  void fail_fast(const std::string &reason);
  void publish_diagnostic(int level, const std::string &message);

  BrakeCoordinatorStateMachine state_machine_;
  std::string controller_name_;
  std::string controller_manager_name_;
  double switch_timeout_s_{2.0};
  std::string current_drive_state_{"brake_engaged"};
  std::string last_transition_reason_{"startup"};
  std::string switch_failure_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_subscription_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr switch_timeout_timer_;
  bool shutdown_requested_{false};
};

}  // namespace robot_arm_gazebo
