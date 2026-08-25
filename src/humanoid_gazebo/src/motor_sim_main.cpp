#include "humanoid_gazebo/motor_sim/motor_sim_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_gazebo::motor_sim::MotorSimNode>();
  rclcpp::spin(node);
  const bool failed = node->shutdown_requested();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return failed ? 1 : 0;
}
