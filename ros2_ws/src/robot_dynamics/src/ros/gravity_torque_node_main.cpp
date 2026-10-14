// gravity_torque_node 入口。
//
// 参数：
//   urdf_path             URDF 路径（必填，fail-fast）
//   joint_states_topic    输入关节状态话题（默认 joint_states）
//   gravity_torque_topic  输出重力矩话题（默认 robot_dynamics/gravity_torque）
//
// 订阅 /joint_states，发布 sensor_msgs/JointState，只填 name 与 effort；
// 不感知仿真、ros2_control 与 CAN，无门控与超时逻辑。

#include <robot_dynamics/ros/gravity_torque_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_dynamics::GravityTorqueNode>(
      rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
