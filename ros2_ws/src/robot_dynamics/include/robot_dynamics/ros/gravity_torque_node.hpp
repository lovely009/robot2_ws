#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

#include "robot_dynamics/arm_model.hpp"

namespace robot_dynamics {

struct GravityTorqueNodeParameters {
  std::string urdf_path;  // 必填；空或解析失败即构造失败
  std::string joint_states_topic{"joint_states"};
  std::string gravity_torque_topic{"robot_dynamics/gravity_torque"};
};

// 纯计算节点：/joint_states → τ_g。不感知 Gazebo、ros2_control 与 CAN，
// 不含门控与超时逻辑（叠加侧负责），只在收到 joint_states 时发布一帧。
class GravityTorqueNode final : public rclcpp::Node {
public:
  explicit GravityTorqueNode(const rclcpp::NodeOptions & options);

  // 测试入口：跳过参数声明，直接给定配置。
  GravityTorqueNode(const rclcpp::NodeOptions & options,
                    GravityTorqueNodeParameters parameters);

private:
  void initialize(GravityTorqueNodeParameters parameters);
  void joint_state_callback(const sensor_msgs::msg::JointState & message);

  std::unique_ptr<ArmModel> model_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      gravity_torque_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
};

}  // namespace robot_dynamics
