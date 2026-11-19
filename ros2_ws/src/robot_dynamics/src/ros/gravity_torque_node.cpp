#include "robot_dynamics/ros/gravity_torque_node.hpp"

#include <utility>
#include <vector>

namespace robot_dynamics {
namespace {

constexpr int kLogThrottleMs = 5000;

std::string join_names(const std::vector<std::string> & names) {
  std::string joined;
  for (const std::string & name : names) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += name;
  }
  return joined;
}

}  // namespace

GravityTorqueNode::GravityTorqueNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("gravity_torque_node", options) {
  GravityTorqueNodeParameters parameters;
  parameters.urdf_path = declare_parameter<std::string>("urdf_path", "");
  parameters.joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "joint_states");
  parameters.gravity_torque_topic = declare_parameter<std::string>(
      "gravity_torque_topic", "robot_dynamics/gravity_torque");
  initialize(std::move(parameters));
}

GravityTorqueNode::GravityTorqueNode(const rclcpp::NodeOptions & options,
                                     GravityTorqueNodeParameters parameters)
    : rclcpp::Node("gravity_torque_node", options) {
  initialize(std::move(parameters));
}

void GravityTorqueNode::initialize(GravityTorqueNodeParameters parameters) {
  model_ = std::make_unique<ArmModel>(parameters.urdf_path);
  if (model_->joint_names().empty()) {
    RCLCPP_WARN(get_logger(),
                "URDF 里没有 nq==1 && nv==1 的可动关节，只会发布空 JointState");
  }
  gravity_torque_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>(
          parameters.gravity_torque_topic, 10);
  // 话题名可能为空（参数被显式清空），订阅空话题无意义，直接暴露为构造期错误。
  joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
          parameters.joint_states_topic, 10,
          [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
            joint_state_callback(*message);
          });
}

void GravityTorqueNode::joint_state_callback(
    const sensor_msgs::msg::JointState & message) {
  // 丢弃畸形帧而不是抛异常（异常会穿出 executor）；不门控、不超时。
  if (message.name.size() != message.position.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 的 name/position 长度不符（%zu/%zu），"
                         "本帧不发布",
                         message.name.size(), message.position.size());
    return;
  }
  const std::vector<std::string> ignored =
      model_->update_configuration(message.name, message.position);
  if (!ignored.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 含模型外或非单自由度关节，已忽略：%s",
                         join_names(ignored).c_str());
  }
  sensor_msgs::msg::JointState torque;
  torque.header.stamp = now();
  torque.name = model_->joint_names();
  torque.effort = model_->gravity_torque();
  gravity_torque_publisher_->publish(torque);
}

}  // namespace robot_dynamics
