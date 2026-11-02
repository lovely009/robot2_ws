#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "motor_hardware_interface/joint_follow_controller.hpp"

using motor_hardware_interface::JointFollowController;
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

TEST(JointFollowController, ContinuousRetargetStopBarrierAndFeedbackProtection) {
  JointFollowController controller;
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
    rclcpp::Parameter("max_velocity", std::vector<double>{1.0})});
  ASSERT_EQ(controller.init("follow_test", "", options), Return::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), Callback::SUCCESS);
  EXPECT_FALSE(controller.get_node()->set_parameters_atomically({
    rclcpp::Parameter("joints", std::vector<std::string>{"other"})}).successful);
  EXPECT_FALSE(controller.get_node()->set_parameters_atomically({
    rclcpp::Parameter("max_velocity", std::vector<double>{3.0})}).successful);
  double measured = 0, p = 0, v = 0, e = 4;
  hardware_interface::CommandInterface cp("joint", "position", &p),
    cv("joint", "velocity", &v), ce("joint", "effort", &e);
  hardware_interface::StateInterface sp("joint", "position", &measured);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(ce); commands.emplace_back(cp); commands.emplace_back(cv);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(sp);
  controller.assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  auto node = std::make_shared<rclcpp::Node>("follow_test_peer");
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
    EXPECT_DOUBLE_EQ(e, 0);
    measured = p;
  }
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  measured = .1;
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), Callback::SUCCESS);
  send(old);
  ASSERT_EQ(tick(), Return::OK);
  EXPECT_DOUBLE_EQ(p, .1); EXPECT_DOUBLE_EQ(v, 0);
  target(.5);
  ASSERT_EQ(tick(), Return::OK);
  measured = p - .11;
  EXPECT_EQ(tick(), Return::ERROR);
  measured = p;
  EXPECT_EQ(tick(), Return::ERROR);  // A fresh sample alone must not restart a faulted controller.
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
