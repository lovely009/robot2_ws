// motor_sim 通用独立入口。
//
// 启动参数（全部显式，无机器人默认值）：
//   config_file            YAML 配置路径（必填）
//
// 拓扑与总线端点完全由 config_file 决定，不依赖任何外部 TopologyContract 或
// 机器人集成层：buses[] 给出每条总线的传输类型与端点，motors[] 的顺序、协议
// 与 ID 也来自 YAML。节点不再接受任何总线类参数。
//
// 状态来源：外部 /joint_states（默认 topic）。节点本身不积分动力学，
// 只根据命令与外部关节状态计算 effort，并将反馈帧发回 CAN 总线。
//
// 控制门：默认关闭；只有收到 control_gate := true 且生命周期进入
// ReadyForControl 后才会产生非零 effort。

#include <motor_sim/ros/motor_sim_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_sim::ros::MotorSimNode>(
      rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
