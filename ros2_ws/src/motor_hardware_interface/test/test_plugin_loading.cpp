#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

hardware_interface::HardwareInfo spatial_info() {
  const auto fixtures = std::filesystem::path(__FILE__).parent_path() / "fixtures";
  hardware_interface::HardwareInfo info;
  info.name = "nested_plugin_loading_test";
  info.type = "system";
  info.hardware_parameters["config_file"] = (fixtures / "spatial_motor_driver.yaml").string();
  for (const auto &name : {"yaw", "pitch"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    for (const auto &name : {hardware_interface::HW_IF_POSITION,
                             hardware_interface::HW_IF_VELOCITY,
                             hardware_interface::HW_IF_EFFORT}) {
      hardware_interface::InterfaceInfo command;
      command.name = name;
      command.min = "-100";
      command.max = "100";
      joint.command_interfaces.push_back(command);
      hardware_interface::InterfaceInfo state;
      state.name = name;
      joint.state_interfaces.push_back(state);
    }
    info.joints.push_back(joint);
  }
  return info;
}

TEST(PluginLoading, MicroRosSystemInitializesNestedSpatialTransmission) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  std::shared_ptr<hardware_interface::SystemInterface> system;
  ASSERT_NO_THROW(system = loader.createSharedInstance("motor_hardware_interface/MicroRosSystem"));
  ASSERT_TRUE(system);
  EXPECT_EQ(system->on_init(spatial_info()), hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
