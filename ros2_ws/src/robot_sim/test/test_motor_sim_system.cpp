#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_python.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_transmission/model.hpp"

namespace {

std::string full_body_config() {
  return std::string(ROBOT_DESCRIPTION_MODELS_DIR) +
         "/full_body_robot/config/motor_driver.hil.yaml";
}

}  // namespace

TEST(MotorSimSystemTest, PluginLoadsAndTracks37Joints) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto hw = loader.createSharedInstance("robot_sim/MotorSimSystem");
  ASSERT_NE(hw, nullptr);

  const std::string cfg = full_body_config();
  const auto model = robot_transmission::Model::load(cfg);
  ASSERT_EQ(model.joint_names().size(), 37U);

  hardware_interface::HardwareInfo info;
  info.name = "motor_sim_system";
  info.type = "system";
  info.hardware_parameters["config_file"] = cfg;
  for (const auto &jname : model.joint_names()) {
    hardware_interface::ComponentInfo joint;
    joint.name = jname;
    for (const char *iface : {hardware_interface::HW_IF_POSITION,
                              hardware_interface::HW_IF_VELOCITY,
                              hardware_interface::HW_IF_EFFORT}) {
      hardware_interface::InterfaceInfo ci;
      ci.name = iface;
      joint.command_interfaces.push_back(ci);
      joint.state_interfaces.push_back(ci);
    }
    info.joints.push_back(joint);
  }

  ASSERT_EQ(hw->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  auto states = hw->export_state_interfaces();
  auto cmds = hw->export_command_interfaces();
  EXPECT_EQ(states.size(), 37U * 3U);
  EXPECT_EQ(cmds.size(), 37U * 3U);

  rclcpp_lifecycle::State active_state;
  ASSERT_EQ(hw->on_activate(active_state), hardware_interface::CallbackReturn::SUCCESS);

  // Command joint 0 (waist_pitch) or waist_yaw to 0.1 rad and step 200 cycles at 10ms.
  for (std::size_t i = 0; i < cmds.size(); i += 3U) {
    cmds[i].set_value(0.1);
  }
  const rclcpp::Time t(0);
  const rclcpp::Duration dt(std::chrono::milliseconds(10));
  for (int step = 0; step < 200; ++step) {
    ASSERT_EQ(hw->write(t, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(hw->read(t, dt), hardware_interface::return_type::OK);
  }
  EXPECT_NEAR(states[0].get_value(), 0.1, 0.03);
  ASSERT_EQ(hw->on_deactivate(active_state), hardware_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
