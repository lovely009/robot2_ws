#include "robot_dynamics/arm_model.hpp"

#include <gtest/gtest.h>

#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// test/fixtures/planar_arm.urdf 的几何/质量参数，闭式解见该文件顶部注释。
constexpr double kGravity = 9.81;
constexpr double kM1 = 1.0;
constexpr double kC1 = 0.2;
constexpr double kD1 = 0.4;
constexpr double kM2 = 0.5;
constexpr double kC2 = 0.15;
constexpr double kD2 = 0.3;
constexpr double kM3 = 0.25;
constexpr double kC3 = 0.1;

std::string fixture_path() {
  return std::string(TEST_SOURCE_DIR) + "/test/fixtures/planar_arm.urdf";
}

// τ_g 闭式解 [Nm]，三个关节都绕 y 轴、q=0 时沿 +x 水平。
std::vector<double> closed_form(double q1, double q2, double q3) {
  const double c1 = std::cos(q1);
  const double c12 = std::cos(q1 + q2);
  const double c123 = std::cos(q1 + q2 + q3);
  const double tau1 =
      -kGravity * (kM1 * kC1 * c1 + kM2 * (kD1 * c1 + kC2 * c12) +
                   kM3 * (kD1 * c1 + kD2 * c12 + kC3 * c123));
  const double tau2 =
      -kGravity * (kM2 * kC2 * c12 + kM3 * (kD2 * c12 + kC3 * c123));
  const double tau3 = -kGravity * kM3 * kC3 * c123;
  return {tau1, tau2, tau3};
}

void expect_torque_near(const std::vector<double> &actual,
                        const std::vector<double> &expected, double tolerance) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance) << "关节下标 " << i;
  }
}

}  // namespace

TEST(ArmModelConstruction, ClassifiesScalarNonScalarAndFixedJoints) {
  const robot_dynamics::ArmModel model(fixture_path());

  EXPECT_EQ(model.joint_names(),
            (std::vector<std::string>{"joint_1", "joint_2", "joint_3"}));
  EXPECT_EQ(model.non_scalar_joint_names(),
            (std::vector<std::string>{"joint_aux"}));
  EXPECT_EQ(model.fixed_joint_names(),
            (std::vector<std::string>{"joint_tool"}));

  EXPECT_TRUE(model.has_joint("joint_2"));
  EXPECT_FALSE(model.has_joint("joint_aux"));    // continuous：nq==2
  EXPECT_FALSE(model.has_joint("joint_tool"));   // fixed：已并入父连杆
  EXPECT_FALSE(model.has_joint("joint_9"));
}

TEST(ArmModelConstruction, RejectsEmptyPathAndMissingFile) {
  EXPECT_THROW(robot_dynamics::ArmModel(""), std::invalid_argument);
  EXPECT_THROW(robot_dynamics::ArmModel(fixture_path() + ".missing"),
               std::invalid_argument);
}

TEST(ArmModelGravity, NeutralConfigurationMatchesClosedForm) {
  const robot_dynamics::ArmModel model(fixture_path());
  expect_torque_near(model.gravity_torque(), closed_form(0.0, 0.0, 0.0),
                     1e-12);
}

TEST(ArmModelGravity, MatchesClosedFormAtArbitraryConfigurations) {
  robot_dynamics::ArmModel model(fixture_path());
  const std::vector<std::vector<double>> configurations = {
      {0.5, 0.0, 0.0}, {0.3, -0.4, 0.7}, {1.0, 0.5, -0.2}, {-1.2, 0.8, 0.4}};
  for (const std::vector<double> &q : configurations) {
    model.update_configuration(model.joint_names(), q);
    expect_torque_near(model.gravity_torque(), closed_form(q[0], q[1], q[2]),
                       1e-12);
  }
}

// ADR 0009 的硬约束：computeGeneralizedGravity 必须与势能梯度的有限差分一致。
TEST(ArmModelGravity, MatchesPotentialEnergyGradient) {
  const std::vector<std::string> names{"joint_1", "joint_2", "joint_3"};
  const std::vector<double> positions{0.3, -0.4, 0.7};

  robot_dynamics::ArmModel model(fixture_path());
  model.update_configuration(names, positions);
  const std::vector<double> tau = model.gravity_torque();

  // 独立再解析一份模型，只用于差分势能，不经 ArmModel 的内部状态。
  pinocchio::Model reference;
  pinocchio::urdf::buildModel(fixture_path(), reference);
  pinocchio::Data data(reference);
  Eigen::VectorXd q = pinocchio::neutral(reference);
  for (std::size_t i = 0; i < names.size(); ++i) {
    q[reference.idx_qs[reference.getJointId(names[i])]] = positions[i];
  }

  constexpr double kStep = 1e-6;  // rad；中心差分，误差 O(step²)
  for (std::size_t i = 0; i < names.size(); ++i) {
    const int q_index = reference.idx_qs[reference.getJointId(names[i])];
    Eigen::VectorXd plus = q;
    Eigen::VectorXd minus = q;
    plus[q_index] += kStep;
    minus[q_index] -= kStep;
    const double gradient =
        (pinocchio::computePotentialEnergy(reference, data, plus) -
         pinocchio::computePotentialEnergy(reference, data, minus)) /
        (2.0 * kStep);
    EXPECT_NEAR(tau[i], gradient, 1e-6) << "关节 " << names[i];
  }
}

// /joint_states 常见形态：子集、乱序、含模型外名字，且不能因此整帧拒绝。
TEST(ArmModelInput, AppliesSubsetOutOfOrderAndReportsIgnoredNames) {
  robot_dynamics::ArmModel model(fixture_path());
  const std::vector<std::string> ignored = model.update_configuration(
      {"gripper_finger", "joint_3", "joint_aux", "joint_1"},
      {9.0, 0.4, 7.0, -0.2});

  EXPECT_EQ(ignored,
            (std::vector<std::string>{"gripper_finger", "joint_aux"}));
  // joint_2 未出现 → 沿用初始 neutral 0
  expect_torque_near(model.gravity_torque(), closed_form(-0.2, 0.0, 0.4),
                     1e-12);
}

TEST(ArmModelInput, MissingJointsKeepPreviousValues) {
  robot_dynamics::ArmModel model(fixture_path());
  EXPECT_TRUE(
      model.update_configuration({"joint_1", "joint_2", "joint_3"}, {0.7, -0.3, 0.9})
          .empty());
  EXPECT_TRUE(model.update_configuration({"joint_2"}, {-0.5}).empty());

  // joint_1 与 joint_3 保持上一次的值
  expect_torque_near(model.gravity_torque(), closed_form(0.7, -0.5, 0.9), 1e-12);
}

TEST(ArmModelInput, GravityTorqueAtIsSideEffectFree) {
  robot_dynamics::ArmModel model(fixture_path());
  model.update_configuration({"joint_1", "joint_2", "joint_3"}, {0.4, 0.2, -0.1});
  const std::vector<double> before = model.gravity_torque();

  const robot_dynamics::GravityTorqueResult result =
      model.gravity_torque_at({"joint_3", "typo_joint"}, {1.1, 0.0});
  EXPECT_EQ(result.ignored_names, (std::vector<std::string>{"typo_joint"}));
  expect_torque_near(result.torque_nm, closed_form(0.4, 0.2, 1.1), 1e-12);

  // 内部构型与求值前逐位一致
  EXPECT_EQ(model.gravity_torque(), before);
}

TEST(ArmModelInput, RejectsMismatchedNameAndPositionLengths) {
  robot_dynamics::ArmModel model(fixture_path());
  EXPECT_THROW(model.update_configuration({"joint_1"}, {}),
               std::invalid_argument);
  EXPECT_THROW(model.gravity_torque_at({"joint_1", "joint_2"}, {0.0}),
               std::invalid_argument);
}
