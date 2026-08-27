#include "motor_sim/core/config.hpp"
#include "robot_arm_gazebo/topology.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace core = motor_sim::core;

namespace {

const std::string kMotorsYaml =
    std::string(TEST_SOURCE_DIR) + "/config/motors.yaml";

}  // namespace

TEST(RobotArmMotorConfig, LoadsExactSixAxisContract) {
  const auto config = core::load_system_config(
      kMotorsYaml, robot_arm_gazebo::robot_arm_topology_contract());
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses.front().name, "arm");
  ASSERT_EQ(config.motors.size(), 6U);
  EXPECT_EQ(config.motors[2].id, 3U);
  EXPECT_EQ(config.motors[3].protocol, "vaxor");
  EXPECT_DOUBLE_EQ(config.motors[3].transmission.gear_ratio, 80.0);
}

TEST(RobotArmMotorConfig, RejectsSwappedWristIdsAndSeventhMotorContracts) {
  auto swapped = robot_arm_gazebo::robot_arm_topology_contract();
  std::swap(swapped.motors[3].id, swapped.motors[4].id);
  EXPECT_THROW(core::load_system_config(kMotorsYaml, swapped),
               std::runtime_error);

  auto seventh = robot_arm_gazebo::robot_arm_topology_contract();
  seventh.motors.push_back({"extra_joint", "arm", "cerb", 7U});
  EXPECT_THROW(core::load_system_config(kMotorsYaml, seventh),
               std::runtime_error);
}
