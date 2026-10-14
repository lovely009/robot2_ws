#include "motor_sim/core/config.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace core = motor_sim::core;

namespace {

std::string fixture_path(const std::string &name) {
  return std::string(TEST_SOURCE_DIR) + "/test/fixtures/" + name;
}

}  // namespace

TEST(SystemConfig, AcceptsContractOwnedByCaller) {
  const core::TopologyContract contract{
      {"arm"},
      {{"joint_a", "arm", "vaxor", 1U},
       {"joint_b", "arm", "vaxor", 4U}}};
  const auto config =
      core::load_system_config(fixture_path("two_axis.yaml"), contract);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].joint, "joint_b");
}

TEST(SystemConfig, RejectsOrderDifferentFromContract) {
  const core::TopologyContract contract{
      {"arm"}, {{"joint_b", "arm", "vaxor", 4U}}};
  EXPECT_THROW(core::load_system_config(fixture_path("two_axis.yaml"), contract),
               std::runtime_error);
}

// MIT τ_ff 的 12 位量化满量程是型号属性（写在 profile 里），必须落到逐台 MotorConfig，
// 否则协议对象只能拿到兼容默认值（R48 的 11.0）。
TEST(SystemConfig, ReadsPerModelMitTorqueLimitIntoMotorConfig) {
  const auto r58 = core::load_system_config(fixture_path("cerb_r58_axis.yaml"));
  ASSERT_EQ(r58.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(r58.motors[0].mit_torque_limit_nm, 45.0);
  // 与 plant 的力矩饱和（本 fixture 照拓扑取 10.0）是两回事，不能互相顶替。
  EXPECT_DOUBLE_EQ(r58.motors[0].plant.effort_limit_nm, 10.0);
}

TEST(SystemConfig, DefaultsMitTorqueLimitForConfigsWithoutTheField) {
  const auto legacy = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(legacy.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(legacy.motors[0].mit_torque_limit_nm, 11.0);
}
