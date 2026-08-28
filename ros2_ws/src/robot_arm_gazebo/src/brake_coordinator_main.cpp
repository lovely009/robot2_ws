#include "robot_arm_gazebo/brake_coordinator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_arm_gazebo::BrakeCoordinator>(
      rclcpp::NodeOptions{});
  rclcpp::spin(node);
  const bool failed = node->shutdown_requested();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return failed ? 1 : 0;
}
