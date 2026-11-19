#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace robot_dynamics {

// 无副作用求值的结果。
struct GravityTorqueResult {
  std::vector<double> torque_nm;             // 与 ArmModel::joint_names() 逐位对齐 [Nm]
  std::vector<std::string> ignored_names;    // 本次输入里未参与构型的名字
};

// URDF → Pinocchio 模型的重力矩计算，只按关节名寻址，不依赖关节顺序。
// 同一组 (names, positions) 既可写入内部构型（ROS 节点路径），也可无副作用求值
// （轨迹前馈按路点求值路径）。
// 非线程安全：内部共享一份 Pinocchio scratch，只在单线程执行器里用。
class ArmModel {
public:
  // urdf_path 为空、文件缺失或解析失败时抛 std::invalid_argument。
  explicit ArmModel(const std::string & urdf_path);

  // nq==1 && nv==1 的可动关节，顺序为 Pinocchio 模型内部顺序，也是 τ_g 的位序。
  const std::vector<std::string> & joint_names() const noexcept {
    return joint_names_;
  }

  // 可动但非单自由度的关节（URDF continuous 在 Pinocchio 里 nq==2），不接收位置。
  const std::vector<std::string> & non_scalar_joint_names() const noexcept {
    return non_scalar_joint_names_;
  }

  // URDF 的 fixed joint：Pinocchio 已把其惯量并入父连杆，不出现在 model.names 里。
  const std::vector<std::string> & fixed_joint_names() const noexcept {
    return fixed_joint_names_;
  }

  bool has_joint(const std::string & name) const noexcept;

  // 按名字更新内部构型；未出现的关节保留上一次的值（初始为 neutral）。
  // 返回未能生效的输入名字（模型里不存在，或存在但非可控），个别多余名字不会
  // 导致整帧拒绝。/joint_states 常带夹爪等无关关节，由调用方决定如何诊断。
  // names 与 positions 长度不一致抛 std::invalid_argument。
  std::vector<std::string> update_configuration(
      const std::vector<std::string> & names,
      const std::vector<double> & positions);

  // 当前内部构型下的 τ_g [Nm]，与 joint_names() 对齐（含不可控关节的自由度不受影响）。
  std::vector<double> gravity_torque() const;

  // 以内部构型为底、按 (names, positions) 覆盖后求 τ_g，不修改内部构型。
  GravityTorqueResult gravity_torque_at(
      const std::vector<std::string> & names,
      const std::vector<double> & positions) const;

private:
  std::vector<std::string> apply_input(const std::vector<std::string> & names,
                                       const std::vector<double> & positions,
                                       Eigen::VectorXd & q) const;
  std::vector<double> torque_from_configuration(const Eigen::VectorXd & q) const;

  pinocchio::Model model_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> non_scalar_joint_names_;
  std::vector<std::string> fixed_joint_names_;
  std::vector<int> joint_q_positions_;  // 与 joint_names_ 对齐：关节在 q 里的下标
  std::vector<int> joint_v_positions_;  // 与 joint_names_ 对齐：关节在 τ_g 里的下标
  Eigen::VectorXd q_;
  mutable pinocchio::Data data_;
};

}  // namespace robot_dynamics
