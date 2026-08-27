#include "robot_arm_gazebo/brake_coordinator.hpp"

#include <gtest/gtest.h>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace arm = robot_arm_gazebo;

namespace {

using namespace std::chrono_literals;

void ensure_rclcpp_initialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

template <typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor &executor,
                Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
  return predicate();
}

void spin_for(rclcpp::executors::SingleThreadedExecutor &executor,
              const std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
}

}  // namespace

TEST(BrakeCoordinatorStateMachine, ActivatesOnceAndOpensGateOnlyAfterSuccess) {
  arm::BrakeCoordinatorStateMachine state;
  EXPECT_EQ(state.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::ActivateController);
  EXPECT_EQ(state.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::None);
  EXPECT_FALSE(state.gate_open());

  EXPECT_EQ(state.on_switch_result(arm::SwitchOperation::Activate, true),
            arm::CoordinatorAction::PublishGateTrue);
  EXPECT_TRUE(state.gate_open());
  EXPECT_TRUE(state.controller_active());
  EXPECT_EQ(state.on_drive_state("motor_control_active"),
            arm::CoordinatorAction::None);
}

TEST(BrakeCoordinatorStateMachine, ClosesGateBeforeDeactivatingController) {
  arm::BrakeCoordinatorStateMachine state;
  ASSERT_EQ(state.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::ActivateController);
  ASSERT_EQ(state.on_switch_result(arm::SwitchOperation::Activate, true),
            arm::CoordinatorAction::PublishGateTrue);

  EXPECT_EQ(state.on_drive_state("brake_engaged"),
            arm::CoordinatorAction::PublishGateFalse);
  EXPECT_FALSE(state.gate_open());
  EXPECT_TRUE(state.controller_active());
  EXPECT_EQ(state.on_drive_state("brake_engaged"),
            arm::CoordinatorAction::DeactivateController);
  EXPECT_EQ(state.on_drive_state("brake_engaged"),
            arm::CoordinatorAction::None);

  EXPECT_EQ(state.on_switch_result(arm::SwitchOperation::Deactivate, true),
            arm::CoordinatorAction::None);
  EXPECT_TRUE(state.brake_engaged());
  EXPECT_FALSE(state.controller_active());
}

TEST(BrakeCoordinatorStateMachine, StaleActivationCannotReopenGateAfterFault) {
  arm::BrakeCoordinatorStateMachine state;
  ASSERT_EQ(state.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::ActivateController);
  EXPECT_EQ(state.on_drive_state("fault_latched"),
            arm::CoordinatorAction::PublishGateFalse);

  EXPECT_EQ(state.on_switch_result(arm::SwitchOperation::Activate, true),
            arm::CoordinatorAction::None);
  EXPECT_FALSE(state.gate_open());
  EXPECT_EQ(state.on_drive_state("fault_latched"),
            arm::CoordinatorAction::DeactivateController);
}

TEST(BrakeCoordinatorStateMachine, SwitchFailureAndTimeoutFailFast) {
  arm::BrakeCoordinatorStateMachine failed;
  ASSERT_EQ(failed.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::ActivateController);
  EXPECT_EQ(failed.on_switch_result(arm::SwitchOperation::Activate, false),
            arm::CoordinatorAction::FailFast);

  arm::BrakeCoordinatorStateMachine timed_out;
  ASSERT_EQ(timed_out.on_drive_state("ready_for_control"),
            arm::CoordinatorAction::ActivateController);
  EXPECT_EQ(timed_out.on_switch_timeout(), arm::CoordinatorAction::FailFast);
}

TEST(BrakeCoordinatorStateMachine, RejectsUnknownDriveState) {
  arm::BrakeCoordinatorStateMachine state;
  EXPECT_EQ(state.on_drive_state("unexpected"),
            arm::CoordinatorAction::FailFast);
}

TEST(BrakeCoordinatorNode, UsesStrictSwitchAndPublishesOrderedGate) {
  ensure_rclcpp_initialized();
  auto harness = std::make_shared<rclcpp::Node>("brake_coordinator_test_harness");
  std::vector<std::string> operations;
  std::vector<int> strictness;
  auto switch_service =
      harness->create_service<controller_manager_msgs::srv::SwitchController>(
          "/test_controller_manager/switch_controller",
          [&operations, &strictness](
              const std::shared_ptr<controller_manager_msgs::srv::
                                        SwitchController::Request> request,
              std::shared_ptr<controller_manager_msgs::srv::SwitchController::
                                  Response> response) {
            strictness.push_back(request->strictness);
            std::string operation = "activate:";
            for (const auto &name : request->activate_controllers) {
              operation += name + ",";
            }
            operation += " deactivate:";
            for (const auto &name : request->deactivate_controllers) {
              operation += name + ",";
            }
            operations.push_back(operation);
            response->ok = true;
          });
  auto drive_publisher =
      harness->create_publisher<std_msgs::msg::String>("test_drive_state", 10);
  std::vector<bool> gates;
  auto gate_subscription = harness->create_subscription<std_msgs::msg::Bool>(
      "test_control_gate", 10,
      [&gates](const std_msgs::msg::Bool &message) {
        gates.push_back(message.data);
      });

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {{"drive_state_topic", "test_drive_state"},
       {"control_gate_topic", "test_control_gate"},
       {"diagnostics_topic", "test_brake_diagnostics"},
       {"controller_manager_name", "/test_controller_manager"},
       {"controller_name", "effort_controller"},
       {"brake_controller_name", "brake_controller"},
       {"controller_switch_timeout", 0.5}});
  auto coordinator = std::make_shared<arm::BrakeCoordinator>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(coordinator);
  EXPECT_STREQ(switch_service->get_service_name(),
               "/test_controller_manager/switch_controller");
  spin_for(executor, 200ms);

  std_msgs::msg::String state;
  state.data = "ready_for_control";
  drive_publisher->publish(state);
  ASSERT_TRUE(spin_until(executor, [&] {
    return !operations.empty() && !gates.empty() && gates.back();
  }));
  EXPECT_EQ(operations.front(),
            "activate:effort_controller, deactivate:brake_controller,");
  ASSERT_FALSE(strictness.empty());
  EXPECT_EQ(strictness.front(),
            controller_manager_msgs::srv::SwitchController::Request::STRICT);

  state.data = "brake_engaged";
  drive_publisher->publish(state);
  ASSERT_TRUE(spin_until(executor, [&] {
    return gates.size() >= 2U && !gates.back();
  }));
  drive_publisher->publish(state);
  ASSERT_TRUE(spin_until(executor, [&] { return operations.size() >= 2U; }));
  EXPECT_EQ(operations.back(),
            "activate:brake_controller, deactivate:effort_controller,");
  EXPECT_FALSE(coordinator->shutdown_requested());

  static_cast<void>(gate_subscription);
  executor.remove_node(coordinator);
  executor.remove_node(harness);
  rclcpp::shutdown();
}
