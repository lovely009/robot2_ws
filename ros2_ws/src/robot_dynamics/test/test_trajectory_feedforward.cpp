#include "robot_dynamics/trajectory_feedforward.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

// 与 test_arm_model.cpp 同一份 fixture 闭式解（planar_arm.urdf 顶部有推导）。
constexpr double kGravity = 9.81;

std::string fixture_path() {
  return std::string(TEST_SOURCE_DIR) + "/test/fixtures/planar_arm.urdf";
}

std::vector<double> closed_form(double q1, double q2, double q3) {
  const double c1 = std::cos(q1);
  const double c12 = std::cos(q1 + q2);
  const double c123 = std::cos(q1 + q2 + q3);
  return {-kGravity *
              (1.0 * 0.2 * c1 + 0.5 * (0.4 * c1 + 0.15 * c12) +
               0.25 * (0.4 * c1 + 0.3 * c12 + 0.1 * c123)),
          -kGravity * (0.5 * 0.15 * c12 + 0.25 * (0.3 * c12 + 0.1 * c123)),
          -kGravity * 0.25 * 0.1 * c123};
}

robot_dynamics::MotorJointMap fixture_map() {
  return {{1, "joint_1"}, {2, "joint_2"}, {3, "joint_3"}};
}

robot_dynamics::TrajectoryPoint point(std::uint32_t time_ms,
                                      std::uint16_t motor_id,
                                      double position_rad) {
  return {time_ms, motor_id, position_rad};
}

}  // namespace

// 常见情形：所有电机同拍。结果应逐点等于 g(q_i)。
TEST(TrajectoryFeedforward, SameTickEqualsPointwiseGravity) {
  const robot_dynamics::ArmModel model(fixture_path());
  const std::vector<robot_dynamics::TrajectoryPoint> points{
      point(0, 1, 0.0),   point(0, 2, 0.0),    point(0, 3, 0.0),
      point(100, 1, 0.5), point(100, 2, 0.0),  point(100, 3, 0.0),
      point(200, 1, 0.3), point(200, 2, -0.4), point(200, 3, 0.7)};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(),
                                                     {1, 2, 3}, points);

  ASSERT_EQ(result.effort_delta_nm.size(), points.size());
  const std::vector<std::vector<double>> expected = {
      closed_form(0.0, 0.0, 0.0), closed_form(0.5, 0.0, 0.0),
      closed_form(0.3, -0.4, 0.7)};
  for (std::size_t tick = 0; tick < expected.size(); ++tick) {
    for (std::size_t motor = 0; motor < expected[tick].size(); ++motor) {
      EXPECT_NEAR(result.effort_delta_nm[tick * 3 + motor],
                  expected[tick][motor], 1e-12)
          << "时刻下标 " << tick << " 电机 " << motor + 1;
    }
  }
  EXPECT_TRUE(result.unmapped_motor_ids.empty());
  EXPECT_TRUE(result.ignored_joint_names.empty());
}

// 某台电机缺一个时刻：用它在自己相邻路点之间的线性插值构型参与 g(q)。
TEST(TrajectoryFeedforward, MissingTickIsLinearlyInterpolated) {
  const robot_dynamics::ArmModel model(fixture_path());
  const std::vector<robot_dynamics::TrajectoryPoint> points{
      point(0, 1, 0.2),  point(0, 2, -0.4),
      point(100, 1, 0.5),  // motor 2 在 t=100 缺拍
      point(200, 1, 0.8), point(200, 2, 0.6)};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(),
                                                     {1, 2}, points);

  ASSERT_EQ(result.effort_delta_nm.size(), points.size());
  // motor 2 在 t=100 的插值位置 = -0.4 + (0.6-(-0.4)) * (100-0)/(200-0) = 0.1
  const std::vector<double> at_0 = closed_form(0.2, -0.4, 0.0);
  const std::vector<double> at_100 = closed_form(0.5, 0.1, 0.0);
  const std::vector<double> at_200 = closed_form(0.8, 0.6, 0.0);
  EXPECT_NEAR(result.effort_delta_nm[0], at_0[0], 1e-12);    // (0, motor 1)
  EXPECT_NEAR(result.effort_delta_nm[1], at_0[1], 1e-12);    // (0, motor 2)
  EXPECT_NEAR(result.effort_delta_nm[2], at_100[0], 1e-12);  // (100, motor 1)
  EXPECT_NEAR(result.effort_delta_nm[3], at_200[0], 1e-12);  // (200, motor 1)
  EXPECT_NEAR(result.effort_delta_nm[4], at_200[1], 1e-12);  // (200, motor 2)
  EXPECT_TRUE(result.unmapped_motor_ids.empty());
}

// 只有单侧路点（t 早于该电机首点）：保持该侧值，不外推。
TEST(TrajectoryFeedforward, SingleSidedSamplesHoldTheirValue) {
  const robot_dynamics::ArmModel model(fixture_path());
  const std::vector<robot_dynamics::TrajectoryPoint> points{
      point(0, 1, 0.2),   point(100, 1, 0.5), point(200, 1, 0.8),
      point(200, 2, 0.6) /* motor 2 只有 t=200 一个路点 */};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(),
                                                     {1, 2}, points);

  ASSERT_EQ(result.effort_delta_nm.size(), points.size());
  // t=0 与 t=100 的 motor 2 都用它唯一的 0.6（没有另一侧可插值）。
  EXPECT_NEAR(result.effort_delta_nm[0], closed_form(0.2, 0.6, 0.0)[0], 1e-12);
  EXPECT_NEAR(result.effort_delta_nm[1], closed_form(0.5, 0.6, 0.0)[0], 1e-12);
  EXPECT_NEAR(result.effort_delta_nm[2], closed_form(0.8, 0.6, 0.0)[0], 1e-12);
  EXPECT_NEAR(result.effort_delta_nm[3], closed_form(0.8, 0.6, 0.0)[1], 1e-12);
}

// target_ids 含未知电机：诊断里记下，它没有前馈，其余电机照算（帧不失败）。
TEST(TrajectoryFeedforward, UnknownMotorIdIsReportedAndOthersStillComputed) {
  const robot_dynamics::ArmModel model(fixture_path());
  const std::vector<robot_dynamics::TrajectoryPoint> points{
      point(0, 1, 0.4), point(0, 9, 1.1)};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(),
                                                     {1, 9}, points);

  EXPECT_EQ(result.unmapped_motor_ids, (std::vector<std::uint16_t>{9}));
  EXPECT_NEAR(result.effort_delta_nm[0], closed_form(0.4, 0.0, 0.0)[0], 1e-12);
  EXPECT_EQ(result.effort_delta_nm[1], 0.0);  // 未知电机：增量恒 0
}

// joint_map 里有名字但 URDF 不可控：同样只进诊断，别的电机照算。
TEST(TrajectoryFeedforward, JointNameOutsideUrdfIsReported) {
  const robot_dynamics::ArmModel model(fixture_path());
  const robot_dynamics::MotorJointMap joint_map{
      {1, "joint_1"}, {2, "not_a_joint"}, {3, "joint_tool"} /* fixed */};
  const std::vector<robot_dynamics::TrajectoryPoint> points{
      point(0, 1, 0.4), point(0, 2, 1.1), point(0, 3, -0.9)};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, joint_map, {1, 2, 3},
                                                     points);

  EXPECT_EQ(result.ignored_joint_names,
            (std::vector<std::string>{"not_a_joint", "joint_tool"}));
  EXPECT_NEAR(result.effort_delta_nm[0], closed_form(0.4, 0.0, 0.0)[0], 1e-12);
  EXPECT_EQ(result.effort_delta_nm[1], 0.0);
  EXPECT_EQ(result.effort_delta_nm[2], 0.0);
}

// 单点轨迹：只有一个时刻、一台电机。
TEST(TrajectoryFeedforward, SinglePoint) {
  const robot_dynamics::ArmModel model(fixture_path());
  const std::vector<robot_dynamics::TrajectoryPoint> points{point(0, 1, 0.9)};
  const robot_dynamics::TrajectoryFeedforward result =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(), {1},
                                                     points);

  ASSERT_EQ(result.effort_delta_nm.size(), 1U);
  EXPECT_NEAR(result.effort_delta_nm[0], closed_form(0.9, 0.0, 0.0)[0], 1e-12);
}

// 空轨迹（取消语义）：没有路点就没有增量，也不该报诊断。
TEST(TrajectoryFeedforward, EmptyTrajectoryIsNoOp) {
  const robot_dynamics::ArmModel model(fixture_path());

  const robot_dynamics::TrajectoryFeedforward cancel =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(), {},
                                                     {});
  EXPECT_TRUE(cancel.effort_delta_nm.empty());
  EXPECT_TRUE(cancel.unmapped_motor_ids.empty());
  EXPECT_TRUE(cancel.ignored_joint_names.empty());

  // target_ids 非空但一个路点都没有：同样没有增量（该电机保留内部构型）。
  const robot_dynamics::TrajectoryFeedforward no_points =
      robot_dynamics::trajectory_gravity_feedforward(model, fixture_map(),
                                                     {1, 2, 3}, {});
  EXPECT_TRUE(no_points.effort_delta_nm.empty());
  EXPECT_TRUE(no_points.unmapped_motor_ids.empty());
}
