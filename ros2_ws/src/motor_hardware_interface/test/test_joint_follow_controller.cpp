#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <thread>
#include "motor_hardware_interface/joint_follow_controller.hpp"

using motor_hardware_interface::JointFollowController;
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

TEST(JointFollowController, ContinuousRetargetDelayedFeedbackAndStopBarrier) {
  JointFollowController controller;
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
    rclcpp::Parameter("max_velocity", std::vector<double>{1.0})});
  ASSERT_EQ(controller.init("follow_test", "", options), Return::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_EQ(controller.command_interface_configuration().names,
    (std::vector<std::string>{"joint/position", "joint/velocity"}));
  EXPECT_FALSE(controller.get_node()->set_parameters_atomically({
    rclcpp::Parameter("joints", std::vector<std::string>{"other"})}).successful);
  EXPECT_FALSE(controller.get_node()->set_parameters_atomically({
    rclcpp::Parameter("max_velocity", std::vector<double>{3.0})}).successful);
  double measured = 0, p = 0, v = 0, e = 4;
  hardware_interface::CommandInterface cp("joint", "position", &p),
    cv("joint", "velocity", &v), ce("joint", "effort", &e);
  hardware_interface::StateInterface sp("joint", "position", &measured);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(cv); commands.emplace_back(cp);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(sp);
  controller.assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  auto node = std::make_shared<rclcpp::Node>("follow_test_peer");
  std::vector<diagnostic_msgs::msg::DiagnosticArray> diagnostics;
  auto diagnostic_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/follow_test/diagnostics", 1, [&](const diagnostic_msgs::msg::DiagnosticArray &message) {
      diagnostics.push_back(message);
    });
  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("/follow_test/target", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  executor.add_node(node);
  auto send = [&](sensor_msgs::msg::JointState message) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
    do {
      publisher->publish(message); executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < end);
  };
  auto target = [&](double value) {
    sensor_msgs::msg::JointState message;
    message.header.stamp = node->now(); message.name = {"joint"}; message.position = {value};
    send(message);
    return message;
  };
  auto tick = [&]() { return controller.update(node->now(), rclcpp::Duration::from_seconds(.02)); };
  auto old = target(.5);
  for (int i = 0; i < 15; ++i) {
    const double before_p = p, before_v = v;
    if (i == 7) target(-.3);
    ASSERT_EQ(tick(), Return::OK);
    EXPECT_LE(std::abs(v - before_v), .04 + 1e-12);
    EXPECT_LE(std::abs(p - before_p), .02 + 1e-12);
    EXPECT_DOUBLE_EQ(e, 4);  // The independent gravity controller owns effort.
    measured = p;
  }
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_DOUBLE_EQ(e, 4);
  measured = .1;
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  send(old);
  ASSERT_EQ(tick(), Return::OK);
  EXPECT_DOUBLE_EQ(p, .1); EXPECT_DOUBLE_EQ(v, 0);
  target(.5);
  // A delayed plant can be more than 0.1 rad behind the smooth command.
  // Its response must not permanently freeze the controller's target stream.
  for (int i = 0; i < 60; ++i) {
    ASSERT_EQ(tick(), Return::OK);
  }
  EXPECT_DOUBLE_EQ(p, .5);
  measured = p;
  ASSERT_EQ(tick(), Return::OK);
  measured = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tick(), Return::ERROR);
  measured = p;
  EXPECT_EQ(tick(), Return::ERROR);  // A fresh sample alone must not restart a faulted controller.
  EXPECT_DOUBLE_EQ(e, 4);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (diagnostics.size() < 2 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_GE(diagnostics.size(), 2U);
  ASSERT_EQ(diagnostics.front().status.size(), 1U);
  EXPECT_EQ(diagnostics.front().status.front().level,
            diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostics.front().status.front().name, "/follow_test");
  EXPECT_NE(diagnostics.front().status.front().message.find("joint"), std::string::npos);
  EXPECT_GT(rclcpp::Time(diagnostics.front().header.stamp).nanoseconds(), 0);
  EXPECT_EQ(diagnostics.front().header.stamp, diagnostics.back().header.stamp);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  const auto count = diagnostics.size();
  target(.2);
  ASSERT_EQ(tick(), Return::OK);
  EXPECT_EQ(diagnostics.size(), count);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
