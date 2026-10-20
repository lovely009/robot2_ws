#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "robot_dynamics/arm_model.hpp"
#include "robot_dynamics/trajectory_feedforward.hpp"

namespace robot_dynamics {

// 兜底 joint_map：默认模型 half_body_robot 的电机→关节表（与 web/js/core/store.js
// 的内置兜底表同形同值）。**不是真相源**——换模型必须显式传参，否则映射对不上时
// 不报错、只是那些关节恒为 0（store.js 的注释记过这次事故）。
inline constexpr const char * kDefaultJointMap =
    "1:joint_link1,2:joint_link2,3:joint_link3,4:joint_link4,5:joint_link5,"
    "6:joint_link6,7:joint_link7,8:joint_link8,9:joint_link9,"
    "10:joint_link1.1,11:joint_link2.2,12:joint_link3.3,13:joint_link4.4,"
    "14:joint_link5.5,15:joint_link6.6,16:joint_link7.7,17:joint_link8.8,"
    "18:joint_link9.9";

struct GravityFeedforwardOverlayParameters {
  std::string urdf_path;  // 必填；空或解析失败即构造失败
  std::string joint_map{kDefaultJointMap};  // 空串 = 用 kDefaultJointMap
  std::string command_input_topic{"robot_dynamics/command_array"};
  std::string trajectory_input_topic{"robot_dynamics/trajectory"};
  std::string command_output_topic{"/motor/command_array"};
  std::string trajectory_output_topic{"/motor/trajectory"};
  std::string joint_states_topic{"joint_states"};
  double joint_states_timeout_s{0.5};
  double effort_gain{1.0};  // 运行期可改（ros2 param set）
};

// 命令快照与轨迹两个通道的 τ_g 叠加层。
//
// 安全不变式：**任何情况下都不丢帧**。τ_g 不可用（无 /joint_states、超时、电机/
// 关节没映射上、帧不符合契约）时原样转发该帧——快照是完整快照语义，缺席 = 固件立即
// 软停，丢一帧比少加一点前馈危险得多。只有 urdf_path 缺失或解析失败才 fail-fast。
//
// 沿用 ArmModel 的约束：非线程安全，必须单线程执行器（`rclcpp::spin` 即可）。
class GravityFeedforwardOverlayNode final : public rclcpp::Node {
public:
  explicit GravityFeedforwardOverlayNode(const rclcpp::NodeOptions & options);

  // 测试入口：跳过参数声明，直接给定配置。
  GravityFeedforwardOverlayNode(const rclcpp::NodeOptions & options,
                                GravityFeedforwardOverlayParameters parameters);

private:
  void initialize(GravityFeedforwardOverlayParameters parameters);
  void joint_state_callback(const sensor_msgs::msg::JointState & message);
  void command_callback(const can_bridge_msgs::msg::MotorCommandArray & message);
  void trajectory_callback(const can_bridge_msgs::msg::MotorTrajectory & message);

  bool joint_states_fresh() const;
  // 未命中时按节流告警并返回 nullptr（该电机没有前馈，帧照发）。
  const std::size_t * torque_index_for(std::uint16_t motor_id,
                                       const char * channel);

  std::unique_ptr<ArmModel> model_;
  MotorJointMap joint_map_;
  std::unordered_map<std::uint16_t, std::size_t> torque_index_;  // 落回模型的
  std::unordered_set<std::uint16_t> mentioned_motor_ids_;  // joint_map 里出现过的
  bool have_joint_states_{false};
  int64_t last_joint_states_ns_{0};
  double joint_states_timeout_s_{0.5};
  double effort_gain_{1.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameters_callback_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr
      command_publisher_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTrajectory>::SharedPtr
      trajectory_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr
      command_subscription_;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorTrajectory>::SharedPtr
      trajectory_subscription_;
};

}  // namespace robot_dynamics
