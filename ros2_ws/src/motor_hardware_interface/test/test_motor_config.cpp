#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "motor_hardware_interface/motor_config.hpp"

namespace {

const std::string kSampleConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/sample_motor_config.yaml";
const std::string kBadConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/bad_motor_config.yaml";
const std::string kMissingTimingConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/missing_timing_motor_config.yaml";
const std::string kMixedTransportConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/mixed_transport_motor_config.yaml";
const std::string kMissingTtyConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/missing_tty_motor_config.yaml";
const std::string kEmptyTransportConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/empty_transport_motor_config.yaml";
const std::string kNoSpecsConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/no_specs_motor_config.yaml";
const std::string kMissingSpecsConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/missing_specs_motor_config.yaml";
TEST(LoadDriverConfig, ParsesTopologyFromYaml) {
  const auto config = motor_hardware_interface::load_driver_config(kSampleConfig);
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses[0].name, "test_bus");
  EXPECT_EQ(config.buses[0].interface_name, "vcan0");
  // transport 缺省为 socketcan，向后兼容不带 transport 字段的配置。
  EXPECT_EQ(config.buses[0].transport, "socketcan");
  EXPECT_TRUE(config.buses[0].tty.empty());

  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[0].name, "shoulder_yaw");
  EXPECT_EQ(config.motors[0].joint_name, "shoulder_yaw_joint");
  EXPECT_EQ(config.motors[0].bus, "test_bus");
  // 拓扑里 type 是型号名 vaxor_2530，解析后经 motor_types.<型号>.protocol 映射为协议类型 vaxor。
  EXPECT_EQ(config.motors[0].type, "vaxor");
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

TEST(LoadSystemConfig, ParsesTimingAndCommandGains) {
  const auto config = motor_hardware_interface::load_system_config(kSampleConfig);
  EXPECT_EQ(config.feedback_timeout_s, 0.10);
  EXPECT_EQ(config.initial_feedback_wait_s, 0.10);

  ASSERT_EQ(config.motor_commands.size(), 2U);
  // 未配置单电机 command 时回落到 motor_types 默认值。
  EXPECT_EQ(config.motor_commands[0].kp, 30.0);
  EXPECT_EQ(config.motor_commands[0].kd, 1.0);
  // 单电机 command 覆盖 motor_types 默认值。
  EXPECT_EQ(config.motor_commands[1].kp, 0.9);
  EXPECT_EQ(config.motor_commands[1].kd, 0.5);
}

TEST(LoadSystemConfig, DriverPartMatchesLoadDriverConfig) {
  const auto system = motor_hardware_interface::load_system_config(kSampleConfig);
  const auto driver = motor_hardware_interface::load_driver_config(kSampleConfig);
  EXPECT_EQ(system.driver.buses.size(), driver.buses.size());
  EXPECT_EQ(system.driver.motors.size(), driver.motors.size());
  EXPECT_EQ(system.driver.motors[0].name, driver.motors[0].name);
  EXPECT_EQ(system.driver.motors[1].parameters, driver.motors[1].parameters);
}

TEST(LoadSystemConfig, MissingTimingThrows) {
  EXPECT_THROW(motor_hardware_interface::load_system_config(kMissingTimingConfig),
               std::runtime_error);
}

TEST(LoadSystemConfig, EmptyPathThrows) {
  EXPECT_THROW(motor_hardware_interface::load_system_config(""),
               std::runtime_error);
}

TEST(LoadSystemConfig, MissingRootThrows) {
  EXPECT_THROW(motor_hardware_interface::load_system_config(kBadConfig),
               std::runtime_error);
}

TEST(LoadSystemConfig, MissingSpecsFileReferenceThrows) {
  EXPECT_THROW(motor_hardware_interface::load_system_config(kNoSpecsConfig),
               std::runtime_error);
}

TEST(LoadSystemConfig, UnreadableSpecsFileThrows) {
  EXPECT_THROW(motor_hardware_interface::load_system_config(kMissingSpecsConfig),
               std::runtime_error);
}

TEST(LoadDriverConfig, ParsesMixedTransportBuses) {
  const auto config =
      motor_hardware_interface::load_driver_config(kMixedTransportConfig);
  ASSERT_EQ(config.buses.size(), 2U);
  EXPECT_EQ(config.buses[0].name, "socket_bus");
  EXPECT_EQ(config.buses[0].transport, "socketcan");
  EXPECT_EQ(config.buses[0].interface_name, "vcan0");
  EXPECT_EQ(config.buses[1].name, "serial_bus");
  EXPECT_EQ(config.buses[1].transport, "serial_slcan");
  EXPECT_EQ(config.buses[1].tty, "/dev/ttyACM0");
  EXPECT_TRUE(config.buses[1].interface_name.empty());
}

TEST(LoadDriverConfig, SerialSlcanBusRequiresTty) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(kMissingTtyConfig),
               std::runtime_error);
}

TEST(LoadDriverConfig, EmptyTransportThrows) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(kEmptyTransportConfig),
               std::runtime_error);
}

} // namespace
