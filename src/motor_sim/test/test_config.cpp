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
      {{"joint_a", "arm", "rp", 1U},
       {"joint_b", "arm", "motor_2530", 4U}}};
  const auto config =
      core::load_system_config(fixture_path("two_axis.yaml"), contract);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].joint, "joint_b");
}

TEST(SystemConfig, RejectsOrderDifferentFromContract) {
  const core::TopologyContract contract{
      {"arm"}, {{"joint_b", "arm", "motor_2530", 4U}}};
  EXPECT_THROW(core::load_system_config(fixture_path("two_axis.yaml"), contract),
               std::runtime_error);
}
