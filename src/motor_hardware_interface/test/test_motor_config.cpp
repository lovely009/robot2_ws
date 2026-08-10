#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "motor_hardware_interface/motor_config.hpp"

namespace {

const std::string kSampleConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/sample_motor_config.yaml";
const std::string kBadConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/bad_motor_config.yaml";

TEST(LoadDriverConfig, ParsesTopologyFromYaml) {
  const auto config = motor_hardware_interface::load_driver_config(kSampleConfig);
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses[0].name, "test_bus");
  EXPECT_EQ(config.buses[0].interface_name, "vcan0");

  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[0].name, "shoulder_yaw");
  EXPECT_EQ(config.motors[0].joint_name, "shoulder_yaw_joint");
  EXPECT_EQ(config.motors[0].bus, "test_bus");
  EXPECT_EQ(config.motors[0].type, "rp");
  EXPECT_EQ(config.motors[0].id, 1U);
  EXPECT_EQ(config.motors[0].parameters.at("torque_constant_nm_per_amp"), "0.35");
}

TEST(LoadDriverConfig, MotorOverridesTypeDefaults) {
  const auto config = motor_hardware_interface::load_driver_config(kSampleConfig);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].parameters.at("torque_constant_nm_per_amp"), "0.55");
  EXPECT_EQ(config.motors[1].parameters.at("extra"), "per_motor");
}

TEST(LoadDriverConfig, EmptyPathThrows) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(""),
               std::runtime_error);
}

TEST(LoadDriverConfig, MissingRootThrows) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(kBadConfig),
               std::runtime_error);
}

} // namespace
