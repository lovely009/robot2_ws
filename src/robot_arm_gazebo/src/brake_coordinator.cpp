#include "robot_arm_gazebo/brake_coordinator.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <utility>

namespace robot_arm_gazebo {

namespace {

diagnostic_msgs::msg::KeyValue diagnostic_value(const std::string &key,
                                                const std::string &value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

CoordinatorAction BrakeCoordinatorStateMachine::on_drive_state(
    const std::string_view state) noexcept {
  if (!is_known_state(state)) {
    return CoordinatorAction::FailFast;
  }
  drive_state_ = state;
  received_drive_state_ = true;

  if (state == "ready_for_control") {
    awaiting_zero_confirmation_ = false;
    brake_engaged_ = false;
    if (pending_switch_.has_value()) {
      return CoordinatorAction::None;
    }
    if (controller_active_) {
      if (!gate_open_) {
        gate_open_ = true;
        return CoordinatorAction::PublishGateTrue;
      }
      return CoordinatorAction::None;
    }
    pending_switch_ = SwitchOperation::Activate;
    return CoordinatorAction::ActivateController;
  }

  if (state == "motor_control_active") {
    return gate_open_ && controller_active_ ? CoordinatorAction::None
                                            : CoordinatorAction::FailFast;
  }

  if (is_braking_state(state)) {
    if (pending_switch_ == SwitchOperation::Deactivate) {
      return CoordinatorAction::None;
    }
    if (gate_open_ || !awaiting_zero_confirmation_) {
      gate_open_ = false;
      brake_engaged_ = false;
      awaiting_zero_confirmation_ = true;
      return CoordinatorAction::PublishGateFalse;
    }
    if (pending_switch_.has_value()) {
      return CoordinatorAction::None;
    }
    awaiting_zero_confirmation_ = false;
    if (controller_active_) {
      pending_switch_ = SwitchOperation::Deactivate;
      return CoordinatorAction::DeactivateController;
    }
    brake_engaged_ = true;
  }
  return CoordinatorAction::None;
}

CoordinatorAction BrakeCoordinatorStateMachine::on_switch_result(
    const SwitchOperation operation, const bool success) noexcept {
  if (!pending_switch_.has_value() || *pending_switch_ != operation) {
    return CoordinatorAction::None;
  }
  pending_switch_.reset();
  if (!success) {
    gate_open_ = false;
    return CoordinatorAction::FailFast;
  }
  if (operation == SwitchOperation::Activate) {
    controller_active_ = true;
    if (drive_state_ == "ready_for_control") {
      gate_open_ = true;
      brake_engaged_ = false;
      return CoordinatorAction::PublishGateTrue;
    }
    return CoordinatorAction::None;
  }
  controller_active_ = false;
  gate_open_ = false;
  brake_engaged_ = true;
  return CoordinatorAction::None;
}

CoordinatorAction BrakeCoordinatorStateMachine::on_switch_timeout() noexcept {
  if (!pending_switch_.has_value()) {
    return CoordinatorAction::None;
  }
  pending_switch_.reset();
  gate_open_ = false;
  return CoordinatorAction::FailFast;
}

bool BrakeCoordinatorStateMachine::is_known_state(
    const std::string_view state) const noexcept {
  return state == "brake_engaged" || state == "awaiting_first_command" ||
         state == "ready_for_control" || state == "motor_control_active" ||
         state == "fault_latched";
}

bool BrakeCoordinatorStateMachine::is_braking_state(
    const std::string_view state) const noexcept {
  return state == "brake_engaged" || state == "awaiting_first_command" ||
         state == "fault_latched";
}

BrakeCoordinator::BrakeCoordinator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("brake_coordinator", options) {
  const auto drive_state_topic =
      declare_parameter<std::string>("drive_state_topic", "drive_state");
  const auto control_gate_topic =
      declare_parameter<std::string>("control_gate_topic", "control_gate");
  const auto diagnostics_topic =
      declare_parameter<std::string>("diagnostics_topic", "diagnostics");
  controller_name_ =
      declare_parameter<std::string>("controller_name", "effort_controller");
  brake_controller_name_ = declare_parameter<std::string>(
      "brake_controller_name", "brake_controller");
  controller_manager_name_ = declare_parameter<std::string>(
      "controller_manager_name", "controller_manager");
  switch_timeout_s_ =
      declare_parameter<double>("controller_switch_timeout", 2.0);
  if (controller_name_.empty() || brake_controller_name_.empty() ||
      controller_name_ == brake_controller_name_ ||
      controller_manager_name_.empty() ||
      switch_timeout_s_ <= 0.0) {
    throw std::invalid_argument("invalid brake coordinator parameters");
  }

  gate_publisher_ =
      create_publisher<std_msgs::msg::Bool>(control_gate_topic, 10);
  diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic,
                                                               10);
  drive_subscription_ = create_subscription<std_msgs::msg::String>(
      drive_state_topic, 10,
      [this](const std_msgs::msg::String &message) {
        drive_state_callback(message);
      });
  std::string service_name = controller_manager_name_;
  if (!service_name.empty() && service_name.back() == '/') {
    service_name.pop_back();
  }
  service_name += "/switch_controller";
  switch_client_ = create_client<SwitchController>(service_name);
  publish_gate(false);
  publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "仿真抱闸协调器等待驱动状态");
}

void BrakeCoordinator::drive_state_callback(
    const std_msgs::msg::String &message) {
  current_drive_state_ = message.data;
  handle_action(state_machine_.on_drive_state(message.data),
                "drive state: " + message.data);
}

void BrakeCoordinator::handle_action(const CoordinatorAction action,
                                     const std::string &reason) {
  last_transition_reason_ = reason;
  switch (action) {
    case CoordinatorAction::None:
      publish_diagnostic(
          state_machine_.gate_open()
              ? diagnostic_msgs::msg::DiagnosticStatus::OK
              : diagnostic_msgs::msg::DiagnosticStatus::WARN,
          reason);
      return;
    case CoordinatorAction::PublishGateFalse:
      publish_gate(false);
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         "gate 已关闭，等待零 effort 确认");
      return;
    case CoordinatorAction::PublishGateTrue:
      publish_gate(true);
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                         "effort controller 已激活，gate 已打开");
      return;
    case CoordinatorAction::ActivateController:
      request_switch(SwitchOperation::Activate);
      return;
    case CoordinatorAction::DeactivateController:
      request_switch(SwitchOperation::Deactivate);
      return;
    case CoordinatorAction::FailFast:
      fail_fast(reason);
      return;
  }
}

void BrakeCoordinator::publish_gate(const bool open) {
  std_msgs::msg::Bool message;
  message.data = open;
  gate_publisher_->publish(message);
}

void BrakeCoordinator::request_switch(const SwitchOperation operation) {
  if (!switch_client_->service_is_ready()) {
    fail_fast("controller manager switch_controller 服务不可用");
    return;
  }
  auto request = std::make_shared<SwitchController::Request>();
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(switch_timeout_s_);
  if (operation == SwitchOperation::Activate) {
    request->activate_controllers.push_back(controller_name_);
    request->deactivate_controllers.push_back(brake_controller_name_);
  } else {
    request->deactivate_controllers.push_back(controller_name_);
    request->activate_controllers.push_back(brake_controller_name_);
  }

  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(switch_timeout_s_));
  switch_timeout_timer_ = create_wall_timer(timeout, [this] {
    if (switch_timeout_timer_) {
      switch_timeout_timer_->cancel();
    }
    handle_action(state_machine_.on_switch_timeout(),
                  "controller 切换确认超时");
  });
  switch_client_->async_send_request(
      request, [this, operation](rclcpp::Client<SwitchController>::SharedFuture
                                    response_future) {
        if (switch_timeout_timer_) {
          switch_timeout_timer_->cancel();
        }
        const bool success = response_future.get()->ok;
        handle_action(state_machine_.on_switch_result(operation, success),
                      success ? "controller STRICT 切换成功"
                              : "controller STRICT 切换被拒绝");
      });
}

void BrakeCoordinator::fail_fast(const std::string &reason) {
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  switch_failure_ = reason;
  publish_gate(false);
  publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason);
  RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  rclcpp::shutdown();
}

void BrakeCoordinator::publish_diagnostic(const int level,
                                          const std::string &message) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<uint8_t>(level);
  status.name = "brake_coordinator";
  status.hardware_id = "gazebo-simulated-brake";
  status.message = message;
  status.values = {
      diagnostic_value("drive_state", current_drive_state_),
      diagnostic_value("brake_engaged",
                       state_machine_.brake_engaged() ? "true" : "false"),
      diagnostic_value("effort_controller_state",
                       state_machine_.controller_active() ? "active"
                                                          : "inactive"),
      diagnostic_value("control_gate",
                       state_machine_.gate_open() ? "true" : "false"),
      diagnostic_value("last_transition_reason", last_transition_reason_),
      diagnostic_value("switch_failure", switch_failure_),
  };
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  diagnostic_publisher_->publish(array);
}

}  // namespace robot_arm_gazebo
