#include "humanoid_gazebo/topology.hpp"
#include "motor_sim/ros/motor_sim_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto share =
      ament_index_cpp::get_package_share_directory("humanoid_gazebo");
  motor_sim::ros::NodeDefaults defaults;
  defaults.config_file = share + "/config/motors.yaml";
  defaults.joint_states_topic = "/joint_states";
  defaults.effort_topic = "/effort_controller/commands";
  defaults.diagnostics_topic = "/humanoid_gazebo/diagnostics";
  auto node = std::make_shared<motor_sim::ros::MotorSimNode>(
      rclcpp::NodeOptions{}, humanoid_gazebo::humanoid_topology_contract(),
      std::move(defaults));
  rclcpp::spin(node);
  const bool failed = node->shutdown_requested();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return failed ? 1 : 0;
}
