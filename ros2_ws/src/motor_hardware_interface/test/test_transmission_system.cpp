#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include "motor_hardware_interface/canfd_system.hpp"

using hardware_interface::CallbackReturn;
class TransmissionSystemTest : public ::testing::Test {
protected:
  std::filesystem::path root;
  void SetUp() override {
    root=std::filesystem::temp_directory_path()/("transmission_system_"+std::to_string(getpid()));
    std::filesystem::create_directories(root);
    const auto specs=(std::filesystem::path(TEST_SOURCE_DIR).parent_path()/"config"/"motor.yaml").string();
    std::ofstream(root/"driver.yaml") << "motor_hardware:\n  motor_specs_file: " << specs
      << "\n  transmissions_file: transmissions.yaml\n  timing: {feedback_timeout_s: 0.1, initial_feedback_wait_s: 1.0}\n  buses:\n    bench: {transport: socketcan, interface: can_never_open}\n  motors:\n    a: {id: 1, joint: legacy_a, type: cerb_r48_101, bus: bench, command: {kp: 0.0, kd: 0.0}}\n    b: {id: 2, joint: legacy_b, type: cerb_r48_101, bus: bench, command: {kp: 0.0, kd: 0.0}}\n";
    std::ofstream(root/"transmissions.yaml") << "transmissions:\n  wrist:\n    plugin: transmission_interface/DifferentialTransmission\n    actuators: [a,b]\n    joints: [yaw,pitch]\n";
  }
  void TearDown() override {std::filesystem::remove_all(root);}
  hardware_interface::HardwareInfo info(const std::vector<std::string>& names) {
    hardware_interface::HardwareInfo result;
    result.name="transmission_test"; result.type="system";
    result.hardware_parameters["config_file"]=(root/"driver.yaml").string();
    for (const auto &name:names) {
      hardware_interface::ComponentInfo joint; joint.name=name;
      hardware_interface::InterfaceInfo command; command.name="position";
      joint.command_interfaces.push_back(command);
      for (const auto &state:{"position","velocity","effort"}) {
        hardware_interface::InterfaceInfo interface; interface.name=state;
        joint.state_interfaces.push_back(interface);
      }
      result.joints.push_back(joint);
    }
    return result;
  }
};
TEST_F(TransmissionSystemTest, UsesGroupJointNamesAndSupportsReorderedRosInterfacesWithoutOpeningCan) {
  motor_hardware_interface::CanFdSystem system;
  ASSERT_EQ(system.on_init(info({"pitch","yaw"})),CallbackReturn::SUCCESS);
  auto commands=system.export_command_interfaces();
  ASSERT_EQ(commands.size(),2U);
  EXPECT_EQ(commands[0].get_name(),"pitch/position");
  EXPECT_EQ(commands[1].get_name(),"yaw/position");
}
TEST_F(TransmissionSystemTest, MissingOrExtraRosJointIsRejectedBeforeActivation) {
  motor_hardware_interface::CanFdSystem missing;
  EXPECT_EQ(missing.on_init(info({"yaw"})),CallbackReturn::ERROR);
  motor_hardware_interface::CanFdSystem extra;
  EXPECT_EQ(extra.on_init(info({"yaw","pitch","unmapped"})),CallbackReturn::ERROR);
}
