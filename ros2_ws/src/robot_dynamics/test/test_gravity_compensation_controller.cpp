#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_dynamics/gravity_compensation_controller.hpp"

namespace {
using Controller = robot_dynamics::GravityCompensationController;
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

class GravityControllerTest : public ::testing::Test {
protected:
  // 配置顺序和硬件接口顺序均故意不等于模型顺序。
  std::vector<std::string> joints_{"elbow", "shoulder"};
  double shoulder_position_{0.4}, elbow_position_{-0.2};
  double shoulder_effort_{7.0}, elbow_effort_{8.0};
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;

  std::unique_ptr<Controller> configured(
    bool enabled = false, double gain = 1.0, std::vector<std::string> joints = {"elbow", "shoulder"},
    const std::string &fixture = "gravity_controller.urdf",
    CallbackReturn expected = CallbackReturn::SUCCESS) {
    auto controller = std::make_unique<Controller>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("joints", joints),
      rclcpp::Parameter("urdf_file", std::string(TEST_SOURCE_DIR) + "/test/fixtures/" + fixture),
      rclcpp::Parameter("enabled", enabled), rclcpp::Parameter("effort_gain", gain)});
    EXPECT_EQ(controller->init("gravity_controller_test", "", options), Return::OK);
    EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()), expected);
    return controller;
  }

  void assign(Controller &controller, bool complete = true) {
    commands_.clear();
    states_.clear();
    commands_.emplace_back("shoulder", "effort", &shoulder_effort_);
    commands_.emplace_back("elbow", "effort", &elbow_effort_);
    states_.emplace_back("shoulder", "position", &shoulder_position_);
    if (complete) {
      states_.emplace_back("elbow", "position", &elbow_position_);
    }
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (auto &command : commands_) { commands.emplace_back(command); }
    for (auto &state : states_) { states.emplace_back(state); }
    controller.assign_interfaces(std::move(commands), std::move(states));
  }

  Return update(Controller &controller) {
    return controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  }
};

TEST_F(GravityControllerTest, ClaimsOnlyEffortAndMeasuredPositionAndDefaultsToZero) {
  auto controller = configured();
  const auto commands = controller->command_interface_configuration();
  EXPECT_EQ(commands.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(commands.names, (std::vector<std::string>{"elbow/effort", "shoulder/effort"}));
  EXPECT_EQ(controller->state_interface_configuration().names,
            (std::vector<std::string>{"elbow/position", "shoulder/position"}));
  assign(*controller);
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
  EXPECT_EQ(update(*controller), Return::OK);
  EXPECT_DOUBLE_EQ(elbow_effort_, 0.0);
}

TEST_F(GravityControllerTest, ComputesMeasuredGravityByNameAndRuntimeGainAndDisableClearOutput) {
  auto controller = configured(true, 0.5);
  assign(*controller);
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(update(*controller), Return::OK);
  const double elbow = -9.81 * 0.5 * 0.25 * std::cos(0.2);
  const double shoulder = -9.81 * (std::cos(0.4) + 0.125 * std::cos(0.2));
  EXPECT_NEAR(shoulder_effort_, 0.5 * shoulder, 1e-12);
  EXPECT_NEAR(elbow_effort_, 0.5 * elbow, 1e-12);
  ASSERT_TRUE(controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("effort_gain", 0.25)}).successful);
  ASSERT_EQ(update(*controller), Return::OK);
  EXPECT_NEAR(shoulder_effort_, 0.25 * shoulder, 1e-12);
  ASSERT_TRUE(controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("enabled", false)}).successful);
  ASSERT_EQ(update(*controller), Return::OK);
  EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
  EXPECT_DOUBLE_EQ(elbow_effort_, 0.0);
  ASSERT_TRUE(controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("enabled", true)}).successful);
  ASSERT_EQ(update(*controller), Return::OK);
  ASSERT_EQ(controller->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
  EXPECT_DOUBLE_EQ(elbow_effort_, 0.0);
  EXPECT_EQ(update(*controller), Return::OK);
  EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
}

TEST_F(GravityControllerTest, NonfiniteStateClearsAllOutputsAndRequiresReactivation) {
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()}) {
    auto controller = configured(true);
    assign(*controller);
    ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(update(*controller), Return::OK);
    shoulder_position_ = value;
    EXPECT_EQ(update(*controller), Return::ERROR);
    EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
    EXPECT_DOUBLE_EQ(elbow_effort_, 0.0);
    shoulder_position_ = 0.4;
    EXPECT_EQ(update(*controller), Return::OK);
    EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
  }
}

TEST_F(GravityControllerTest, RejectsMissingInterfacesAndZerosCommandsOnFailedActivation) {
  auto controller = configured(true);
  assign(*controller, false);
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  EXPECT_DOUBLE_EQ(shoulder_effort_, 0.0);
  EXPECT_DOUBLE_EQ(elbow_effort_, 0.0);
}

TEST_F(GravityControllerTest, ValidatesGainAndFreezesConfiguredModelParameters) {
  for (double gain : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    configured(false, gain, joints_, "gravity_controller.urdf", CallbackReturn::ERROR);
  }
  auto controller = configured();
  for (double gain : {-0.1, 1.1, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
      {rclcpp::Parameter("enabled", true), rclcpp::Parameter("effort_gain", gain)}).successful);
    EXPECT_FALSE(controller->get_node()->get_parameter("enabled").as_bool());
  }
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("joints", std::vector<std::string>{"elbow"})}).successful);
  EXPECT_FALSE(controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("urdf_file", "other.urdf")}).successful);
  configured(false, 1.0, {"elbow", "elbow"}, "gravity_controller.urdf", CallbackReturn::ERROR);
}

TEST_F(GravityControllerTest, DisabledUnsupportedOrIncompleteModelsDoNotBlockOtherControllers) {
  for (auto joints : {std::vector<std::string>{"elbow"},
                       std::vector<std::string>{"elbow", "shoulder", "unknown"}}) {
    auto controller = configured(false, 1.0, joints);
    const auto result = controller->get_node()->set_parameters_atomically(
      {rclcpp::Parameter("enabled", true)});
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
    configured(true, 1.0, joints, "gravity_controller.urdf", CallbackReturn::ERROR);
  }
  auto controller = configured(false, 1.0, {"joint_1", "joint_2", "joint_3", "joint_aux"},
                               "planar_arm.urdf");
  const auto result = controller->get_node()->set_parameters_atomically(
    {rclcpp::Parameter("enabled", true)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("joint_aux"), std::string::npos);
}

TEST_F(GravityControllerTest, PluginIsDiscoverable) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable("robot_dynamics/GravityCompensationController"));
  EXPECT_NE(loader.createSharedInstance("robot_dynamics/GravityCompensationController"), nullptr);
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
