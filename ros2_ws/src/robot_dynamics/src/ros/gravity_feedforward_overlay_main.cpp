// gravity_feedforward_overlay 入口。
//
// 参数：见 README「重力前馈叠加节点」一节。urdf_path 必填（空或解析失败即起不来）。
// 命令源启用前馈时把话题指到 /robot_dynamics/command_array 与
// /robot_dynamics/trajectory，本节点加上 τ_g 后转发到固件订阅的 /micro_ros_do/*。

#include <robot_dynamics/ros/gravity_feedforward_overlay_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<robot_dynamics::GravityFeedforwardOverlayNode>(
          rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
