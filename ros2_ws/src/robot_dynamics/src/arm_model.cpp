#include "robot_dynamics/arm_model.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <stdexcept>

namespace robot_dynamics {
namespace {

// fixed root（不注入 free-flyer）：臂基座固定在世界里，τ_g 才是关节侧真实要补偿的量。
pinocchio::Model load_model(const std::string & urdf_path) {
  if (urdf_path.empty()) {
    throw std::invalid_argument("robot_dynamics: URDF 路径为空");
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  return model;
}

}  // namespace

ArmModel::ArmModel(const std::string & urdf_path)
    : model_(load_model(urdf_path)),
      q_(pinocchio::neutral(model_)),
      data_(model_) {
  // 关节分类只在构造期做一次；joint_0 是 universe，不属于 URDF。
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(model_.joints.size()); ++id) {
    if (model_.nqs[id] == 1 && model_.nvs[id] == 1) {
      joint_names_.push_back(model_.names[id]);
      joint_q_positions_.push_back(model_.idx_qs[id]);
      joint_v_positions_.push_back(model_.idx_vs[id]);
    } else {
      non_scalar_joint_names_.push_back(model_.names[id]);
    }
  }
  // frames[0] 是 universe 的 fixed frame，不是 URDF 里的固定关节。
  for (pinocchio::FrameIndex id = 1; id < static_cast<pinocchio::FrameIndex>(model_.frames.size()); ++id) {
    if (model_.frames[id].type == pinocchio::FrameType::FIXED_JOINT) {
      fixed_joint_names_.push_back(model_.frames[id].name);
    }
  }
}

bool ArmModel::has_joint(const std::string & name) const noexcept {
  return std::find(joint_names_.begin(), joint_names_.end(), name) !=
         joint_names_.end();
}

std::vector<std::string> ArmModel::apply_input(
    const std::vector<std::string> & names,
    const std::vector<double> & positions,
    Eigen::VectorXd & q) const {
  if (names.size() != positions.size()) {
    throw std::invalid_argument(
        "robot_dynamics: names 与 positions 长度不一致");
  }
  std::vector<std::string> ignored;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto slot =
        std::find(joint_names_.begin(), joint_names_.end(), names[i]);
    if (slot == joint_names_.end()) {
      ignored.push_back(names[i]);
      continue;
    }
    q[joint_q_positions_[static_cast<std::size_t>(slot - joint_names_.begin())]] =
        positions[i];
  }
  return ignored;
}

std::vector<double> ArmModel::torque_from_configuration(
    const Eigen::VectorXd & q) const {
  const auto & tau = pinocchio::computeGeneralizedGravity(model_, data_, q);
  std::vector<double> torque;
  torque.reserve(joint_v_positions_.size());
  for (const int v : joint_v_positions_) {
    torque.push_back(tau[v]);
  }
  return torque;
}

std::vector<std::string> ArmModel::update_configuration(
    const std::vector<std::string> & names,
    const std::vector<double> & positions) {
  return apply_input(names, positions, q_);
}

std::vector<double> ArmModel::gravity_torque() const {
  return torque_from_configuration(q_);
}

GravityTorqueResult ArmModel::gravity_torque_at(
    const std::vector<std::string> & names,
    const std::vector<double> & positions) const {
  Eigen::VectorXd q = q_;  // 以内部构型为底，不写回
  GravityTorqueResult result;
  result.ignored_names = apply_input(names, positions, q);
  result.torque_nm = torque_from_configuration(q);
  return result;
}

}  // namespace robot_dynamics
