#include "humanoid_gazebo/motor_sim/config.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace motor_sim = humanoid_gazebo::motor_sim;

namespace {

const std::string kMotorsYaml =
    std::string(TEST_SOURCE_DIR) + "/config/motors.yaml";
const std::string kBadLimitsYaml =
    std::string(TEST_SOURCE_DIR) + "/test/fixtures/bad_limits.yaml";

std::size_t count_protocol(const motor_sim::SystemConfig &config,
                           const std::string &protocol) {
  return static_cast<std::size_t>(std::count_if(
      config.motors.begin(), config.motors.end(),
      [&protocol](const auto &motor) { return motor.protocol == protocol; }));
}

}  // namespace

TEST(MotorConfig, LoadsExactThirtyAxisContract) {
  const auto config = motor_sim::load_system_config(kMotorsYaml);
  ASSERT_EQ(config.buses.size(), 3U);
  EXPECT_EQ(config.buses[0].name, "left_arm");
  EXPECT_EQ(config.buses[1].name, "right_arm");
  EXPECT_EQ(config.buses[2].name, "legs");
  ASSERT_EQ(config.motors.size(), 30U);
  EXPECT_EQ(count_protocol(config, "rp"), 20U);
  EXPECT_EQ(count_protocol(config, "motor_2530"), 10U);
  EXPECT_EQ(config.motors.front().joint,
            "left_scapula_protraction_joint");
  EXPECT_EQ(config.motors[8].joint, "left_wrist_deviation_joint");
  EXPECT_EQ(config.motors[9].joint,
            "right_scapula_protraction_joint");
  EXPECT_EQ(config.motors[18].joint, "left_hip_yaw_joint");
  EXPECT_EQ(config.motors.back().joint, "right_ankle_roll_joint");
}

TEST(MotorConfig, LoadsTimingIdsLimitsAndNominalTransmission) {
  const auto config = motor_sim::load_system_config(kMotorsYaml);
  EXPECT_EQ(config.timing.control_hz, 100U);
  EXPECT_EQ(config.timing.rp_feedback_hz, 100U);
  EXPECT_EQ(config.timing.vaxor_feedback_hz, 50U);
  EXPECT_EQ(config.timing.command_timeout_ms, 150U);

  const auto &forearm = config.motors.at(6);
  EXPECT_EQ(forearm.bus, "left_arm");
  EXPECT_EQ(forearm.id, 1U);
  EXPECT_DOUBLE_EQ(forearm.transmission.gear_ratio, 36.0);
  EXPECT_DOUBLE_EQ(forearm.transmission.efficiency, 0.80);
  EXPECT_DOUBLE_EQ(forearm.plant.torque_constant_nm_per_a, 0.012);

  const auto &right_ankle_roll = config.motors.back();
  EXPECT_EQ(right_ankle_roll.bus, "legs");
  EXPECT_EQ(right_ankle_roll.id, 4U);
  EXPECT_DOUBLE_EQ(right_ankle_roll.transmission.gear_ratio, 80.0);
  EXPECT_LT(right_ankle_roll.limits.hard_lower_rad,
            right_ankle_roll.limits.command_lower_rad);
  EXPECT_LT(right_ankle_roll.limits.command_upper_rad,
            right_ankle_roll.limits.hard_upper_rad);
}

TEST(MotorConfig, RejectsCommandLimitOutsideHardLimit) {
  EXPECT_THROW(motor_sim::load_system_config(kBadLimitsYaml),
               std::runtime_error);
}

TEST(MotorConfig, RejectsMissingFile) {
  EXPECT_THROW(motor_sim::load_system_config("/does/not/exist.yaml"),
               std::runtime_error);
}

TEST(MotorConfig, RejectsRuntimeConfigWithWrongThirtyAxisOrder) {
  std::ifstream source(kMotorsYaml);
  ASSERT_TRUE(source.good());
  std::string yaml((std::istreambuf_iterator<char>(source)),
                   std::istreambuf_iterator<char>());
  const std::string first = "joint: left_scapula_protraction_joint";
  const std::string second = "joint: left_scapula_elevation_joint";
  const auto first_at = yaml.find(first);
  const auto second_at = yaml.find(second);
  ASSERT_NE(first_at, std::string::npos);
  ASSERT_NE(second_at, std::string::npos);
  const std::string placeholder = "joint: __temporary_joint_name__";
  yaml.replace(first_at, first.size(), placeholder);
  const auto shifted_second_at = yaml.find(second);
  ASSERT_NE(shifted_second_at, std::string::npos);
  yaml.replace(shifted_second_at, second.size(), first);
  const auto placeholder_at = yaml.find(placeholder);
  ASSERT_NE(placeholder_at, std::string::npos);
  yaml.replace(placeholder_at, placeholder.size(), second);

  const std::string path = "/tmp/humanoid_gazebo_wrong_order.yaml";
  {
    std::ofstream output(path);
    ASSERT_TRUE(output.good());
    output << yaml;
  }
  EXPECT_THROW(motor_sim::load_system_config(path), std::runtime_error);
  static_cast<void>(std::remove(path.c_str()));
}
