// motor_sim 通用独立入口。
//
// 启动参数（全部显式，无机器人默认值）：
//   config_file            YAML 配置路径（必填）
//   transport_mode         usb_serial | serial_slcan | socket_can（默认 usb_serial）
//   <bus_name>_port        每个逻辑总线一个，值为 tty 路径或 socketcan 接口名
//   baud_rate              串口波特率（默认 3000000）
//   allow_external_canfd   serial_slcan 时必须显式设为 true
//
// 拓扑完全由 config_file 决定，不依赖任何外部 TopologyContract 或
// 机器人集成层。buses/motors 的顺序、协议与 ID 都来自 YAML。
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
