#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <unistd.h>

#include "can_bridge_msgs/msg/motor_command_array.hpp"
#include "can_bridge_msgs/msg/motor_control_state_array.hpp"
#include "can_bridge_msgs/msg/motor_motion_state_array.hpp"
#include "motor_hardware_interface/micro_ros_system.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_transmission/model.hpp"
#include "std_msgs/msg/empty.hpp"

namespace {
using M = can_bridge_msgs::msg::MotorMotionStateArray;
using C = can_bridge_msgs::msg::MotorControlStateArray;
using Q = can_bridge_msgs::msg::MotorCommandArray;
using CallbackReturn = hardware_interface::CallbackReturn;
using namespace std::chrono_literals;

uint64_t stamp() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

class MicroRosSystemTest : public ::testing::Test {
protected:
  std::filesystem::path root_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Publisher<M>::SharedPtr motion_pub_;
  rclcpp::Publisher<C>::SharedPtr control_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Subscription<Q>::SharedPtr command_sub_;
  std::vector<Q> commands_;
  int motion_seq_{0};
  int control_seq_{0};

  void SetUp() override {
    static int serial = 0;
    root_ = std::filesystem::temp_directory_path() /
      ("micro_ros_system_" + std::to_string(getpid()) + "_" + std::to_string(++serial));
    std::filesystem::create_directories(root_);
    std::ofstream(root_ / "driver.yaml") <<
      "motor_hardware:\n"
      "  transmissions_file: transmissions.yaml\n"
      "  motors:\n"
      "    a: {id: 8, joint: legacy_a}\n"
      "    b: {id: 9, joint: legacy_b}\n";
    std::ofstream(root_ / "transmissions.yaml") <<
      "transmissions:\n"
      "  pair:\n"
      "    plugin: transmission_interface/DifferentialTransmission\n"
      "    actuators: [a,b]\n"
      "    joints: [yaw,pitch]\n";
    peer_ = std::make_shared<rclcpp::Node>("micro_ros_system_test_peer_" + std::to_string(serial));
    motion_pub_ = peer_->create_publisher<M>("/motor/motion_states",
      rclcpp::QoS(1).best_effort());
    control_pub_ = peer_->create_publisher<C>("/motor/control_states",
      rclcpp::QoS(1).reliable());
    stop_pub_ = peer_->create_publisher<std_msgs::msg::Empty>("/host_control/stop",
      rclcpp::QoS(1).reliable());
    command_sub_ = peer_->create_subscription<Q>("/motor/host_command_array",
      rclcpp::QoS(1).reliable(), [this](Q::SharedPtr message) {
        commands_.push_back(*message);
      });
  }

  void TearDown() override {
    command_sub_.reset();
    stop_pub_.reset();
    control_pub_.reset();
    motion_pub_.reset();
    peer_.reset();
    std::filesystem::remove_all(root_);
  }

  hardware_interface::HardwareInfo info(
    const std::vector<std::string> &joints = {"pitch", "yaw"}) {
    hardware_interface::HardwareInfo result;
    result.name = "micro_ros_system_test";
    result.type = "system";
    result.hardware_parameters["config_file"] = (root_ / "driver.yaml").string();
    for (const auto &name : joints) {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      for (const auto &interface_name : {"position", "velocity"}) {
        hardware_interface::InterfaceInfo command;
        command.name = interface_name;
        command.min = "-1.5";
        command.max = "1.5";
        joint.command_interfaces.push_back(command);
      }
      for (const auto &interface_name : {"position", "velocity", "effort"}) {
        hardware_interface::InterfaceInfo state;
        state.name = interface_name;
        joint.state_interfaces.push_back(state);
      }
      result.joints.push_back(joint);
    }
    return result;
  }

  hardware_interface::HardwareInfo effort_info() {
    auto result = info();
    for (auto &joint : result.joints) {
      hardware_interface::InterfaceInfo effort;
      effort.name = "effort";
      effort.min = "-2";
      effort.max = "2";
      joint.command_interfaces.push_back(effort);
    }
    return result;
  }

  std::vector<std::string> effort_interfaces() {
    return {"pitch/effort", "yaw/effort"};
  }

  void wait_for_feedback_connections() {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while ((motion_pub_->get_subscription_count() == 0U ||
            control_pub_->get_subscription_count() == 0U ||
            command_sub_->get_publisher_count() == 0U) &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(peer_);
      std::this_thread::sleep_for(2ms);
    }
    ASSERT_GE(motion_pub_->get_subscription_count(), 1U);
    ASSERT_GE(control_pub_->get_subscription_count(), 1U);
    ASSERT_GE(command_sub_->get_publisher_count(), 1U);
    std::this_thread::sleep_for(10ms);
  }

  std::unique_ptr<motor_hardware_interface::MicroRosSystem>
  initialized(const hardware_interface::HardwareInfo &hardware_info) {
    auto system = std::make_unique<motor_hardware_interface::MicroRosSystem>();
    EXPECT_EQ(system->on_init(hardware_info), CallbackReturn::SUCCESS);
    wait_for_feedback_connections();
    return system;
  }

  M motion_message(uint32_t age = 0) {
    M message;
    message.sync_time_ns = stamp();
    message.seq = ++motion_seq_;
    for (const auto &[id, position, velocity, effort] :
         std::vector<std::tuple<uint16_t, float, float, float>>{
           {8, 0.2f, 0.3f, 0.4f}, {9, -0.2f, -0.3f, -0.4f}}) {
      can_bridge_msgs::msg::MotorMotionStateTarget state;
      state.motor_id = id;
      state.position = position;
      state.velocity = velocity;
      state.effort = effort;
      state.state_age_ms = age;
      state.target_source = can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_SNAPSHOT;
      message.states.push_back(state);
    }
    return message;
  }

  void publish_motion(uint32_t age = 0) {
    motion_pub_->publish(motion_message(age));
  }

  C control_message(
    uint64_t session = 1, bool armed = true, uint32_t age = 0, bool complete = true,
    bool motion_enabled = true,
    uint8_t execution_state = can_bridge_msgs::msg::MotorControlState::EXECUTION_COMMAND,
    uint16_t last_failure_reason = 0) {
    C message;
    message.schema_version = C::SCHEMA_VERSION;
    message.sync_time_ns = stamp();
    message.session_id = session;
    message.sequence = ++control_seq_;
    message.tick_ms = 1;
    for (uint16_t id : {uint16_t{8}, uint16_t{9}}) {
      if (!complete && id == 9) {
        continue;
      }
      can_bridge_msgs::msg::MotorControlState state;
      state.motor_id = id;
      state.online = true;
      state.control_lifecycle = can_bridge_msgs::msg::MotorControlState::LIFECYCLE_ACTIVE;
      state.execution_state = execution_state;
      state.policy_enabled = armed;
      state.motion_enabled = motion_enabled;
      state.state_age_ms = age;
      state.unified_fault_code = 0;
      state.vendor_fault_code = 0;
      state.last_failure_reason = last_failure_reason;
      state.policy_generation = 1;
      message.states.push_back(state);
    }
    return message;
  }

  void publish_control(
    uint64_t session = 1, bool armed = true, uint32_t age = 0, bool complete = true,
    bool motion_enabled = true,
    uint8_t execution_state = can_bridge_msgs::msg::MotorControlState::EXECUTION_COMMAND,
    uint16_t last_failure_reason = 0) {
    control_pub_->publish(control_message(session, armed, age, complete, motion_enabled,
                                          execution_state, last_failure_reason));
  }

  CallbackReturn run_activation(motor_hardware_interface::MicroRosSystem &system,
                                bool armed = false, uint32_t motion_age = 0) {
    std::atomic<bool> stop{false};
    std::thread feedback([&] {
      while (!stop.load()) {
        publish_motion(motion_age);
        publish_control(1, armed);
        std::this_thread::sleep_for(2ms);
      }
    });
    const auto result = system.on_activate(rclcpp_lifecycle::State());
    stop.store(true);
    feedback.join();
    return result;
  }

  void activate(motor_hardware_interface::MicroRosSystem &system,
                bool armed = false) {
    EXPECT_EQ(run_activation(system, armed), CallbackReturn::SUCCESS);
  }

  void spin_until_command(std::size_t count, std::chrono::milliseconds timeout = 300ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (commands_.size() < count && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(peer_);
      std::this_thread::sleep_for(2ms);
    }
  }

  bool spin_until_target_count(
    std::size_t target_count, std::chrono::milliseconds timeout = 300ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(peer_);
      if (!commands_.empty() && commands_.back().targets.size() == target_count) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  std::vector<std::string> all_command_interfaces() {
    return {"pitch/position", "pitch/velocity", "yaw/position", "yaw/velocity"};
  }

  void start_writer(motor_hardware_interface::MicroRosSystem &system) {
    const auto interfaces = all_command_interfaces();
    ASSERT_EQ(system.prepare_command_mode_switch(interfaces, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.perform_command_mode_switch(interfaces, {}),
              hardware_interface::return_type::OK);
  }

  void stop_writer(motor_hardware_interface::MicroRosSystem &system) {
    const auto interfaces = all_command_interfaces();
    ASSERT_EQ(system.prepare_command_mode_switch({}, interfaces),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.perform_command_mode_switch({}, interfaces),
              hardware_interface::return_type::OK);
  }

  void arm(motor_hardware_interface::MicroRosSystem &system) {
    publish_motion();
    publish_control(1, true);
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
    start_writer(system);
  }

  void stop_via_topic(motor_hardware_interface::MicroRosSystem &system) {
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (!stop_pub_->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    ASSERT_GE(stop_pub_->get_subscription_count(), 1U);
    stop_pub_->publish(std_msgs::msg::Empty());
    auto interfaces = system.export_command_interfaces();
    while (!std::isnan(interfaces[0].get_value()) && std::chrono::steady_clock::now() < deadline) {
      ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(10ms)),
                hardware_interface::return_type::OK);
      std::this_thread::sleep_for(1ms);
    }
    ASSERT_TRUE(std::isnan(interfaces[0].get_value()));
  }

  void expect_snapshot(motor_hardware_interface::MicroRosSystem &system, std::size_t target_count) {
    rclcpp::spin_some(peer_);
    const auto count = commands_.size();
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
    spin_until_command(count + 1);
    ASSERT_EQ(commands_.size(), count + 1);
    EXPECT_EQ(commands_.back().targets.size(), target_count);
  }
};

TEST_F(MicroRosSystemTest, ValidatesIdsJointsAndFiniteInterfaceBoundsBeforeActivation) {
  auto valid = initialized(info());
  auto commands = valid->export_command_interfaces();
  auto states = valid->export_state_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  EXPECT_EQ(commands[0].get_name(), "pitch/position");
  EXPECT_EQ(commands[1].get_name(), "pitch/velocity");
  ASSERT_EQ(states.size(), 6U);
  EXPECT_EQ(states[0].get_name(), "pitch/position");

  motor_hardware_interface::MicroRosSystem missing_joint;
  EXPECT_EQ(missing_joint.on_init(info({"pitch"})), CallbackReturn::ERROR);
  motor_hardware_interface::MicroRosSystem extra_joint;
  EXPECT_EQ(extra_joint.on_init(info({"pitch", "yaw", "extra"})), CallbackReturn::ERROR);

  auto bad_bounds = info();
  bad_bounds.joints[0].command_interfaces[0].min = "nan";
  motor_hardware_interface::MicroRosSystem nonfinite;
  EXPECT_EQ(nonfinite.on_init(bad_bounds), CallbackReturn::ERROR);
  bad_bounds.joints[0].command_interfaces[0].min = "2";
  bad_bounds.joints[0].command_interfaces[0].max = "1";
  motor_hardware_interface::MicroRosSystem reversed;
  EXPECT_EQ(reversed.on_init(bad_bounds), CallbackReturn::ERROR);
}

TEST_F(MicroRosSystemTest, RejectsDuplicateOrOutOfRangeMotorIds) {
  std::ofstream(root_ / "driver.yaml") <<
    "motor_hardware:\n  transmissions_file: transmissions.yaml\n"
    "  motors:\n    a: {id: 8, joint: legacy_a}\n    b: {id: 8, joint: legacy_b}\n";
  motor_hardware_interface::MicroRosSystem duplicate;
  EXPECT_EQ(duplicate.on_init(info()), CallbackReturn::ERROR);
  std::ofstream(root_ / "driver.yaml") <<
    "motor_hardware:\n  transmissions_file: transmissions.yaml\n"
    "  motors:\n    a: {id: 8, joint: legacy_a}\n    b: {id: 65, joint: legacy_b}\n";
  motor_hardware_interface::MicroRosSystem out_of_range;
  EXPECT_EQ(out_of_range.on_init(info()), CallbackReturn::ERROR);
}

TEST_F(MicroRosSystemTest, ActivationUsesMeasuredJointPositionsAndLeaseStaysEmpty) {
  auto system = initialized(info());
  activate(*system, false);
  auto states = system->export_state_interfaces();
  const auto model = robot_transmission::Model::load((root_ / "driver.yaml").string());
  const auto expected = model.actuator_to_joint({
    {"a", robot_transmission::Axis(0.2, 0.3, 0.4)},
    {"b", robot_transmission::Axis(-0.2, -0.3, -0.4)}});
  EXPECT_NEAR(states[0].get_value(), expected.at("pitch").position, 1e-6);
  EXPECT_NEAR(states[1].get_value(), expected.at("pitch").velocity, 1e-6);
  EXPECT_NEAR(states[2].get_value(), expected.at("pitch").effort, 1e-6);
  EXPECT_NEAR(states[3].get_value(), expected.at("yaw").position, 1e-6);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(1);
  ASSERT_EQ(commands_.size(), 1U);
  EXPECT_EQ(commands_[0].targets.size(), 0U);
  EXPECT_NE(commands_[0].sequence, 0U);
  EXPECT_NE(commands_[0].sync_time_ns, 0U);
}

TEST_F(MicroRosSystemTest, HistoricalFailureDoesNotOverrideCurrentActivePermission) {
  auto system = initialized(info());
  activate(*system, false);
  arm(*system);
  publish_motion();
  publish_control(1, true, 0, true, true,
    can_bridge_msgs::msg::MotorControlState::EXECUTION_COMMAND, 9);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  EXPECT_TRUE(spin_until_target_count(2));
}

TEST_F(MicroRosSystemTest, UnarmedCurrentFaultCannotResumePendingCommand) {
  auto system = initialized(info());
  activate(*system, false);
  start_writer(*system);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(0.8);
  publish_motion();
  auto fault = control_message(1, false);
  fault.states[0].unified_fault_code = 1;
  control_pub_->publish(fault);
  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  publish_motion();
  publish_control(1, true);
  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  spin_until_command(1);
  for (const auto &command : commands_) {
    EXPECT_TRUE(command.targets.empty());
  }
  EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  activate(*system, true);
  start_writer(*system);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  EXPECT_TRUE(spin_until_target_count(2));
}

TEST_F(MicroRosSystemTest, ActivationRejectsPresentDeviceFault) {
  auto system = initialized(info());
  std::atomic<bool> stop{false};
  std::thread feedback([&] {
    while (!stop.load()) {
      auto control = control_message(1, false);
      control.states[0].vendor_fault_code = 2;
      control_pub_->publish(control);
      publish_motion();
      std::this_thread::sleep_for(2ms);
    }
  });
  const auto result = system->on_activate(rclcpp_lifecycle::State());
  stop.store(true);
  feedback.join();
  EXPECT_EQ(result, CallbackReturn::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, AggregatedFeedbackControlsOnlyConfiguredDevices) {
  auto system = initialized(info());
  std::atomic<bool> stop{false};
  std::thread feedback([&] {
    while (!stop.load()) {
      auto control = control_message();
      can_bridge_msgs::msg::MotorControlState unrelated_control;
      unrelated_control.motor_id = 10;
      unrelated_control.vendor_fault_code = 1;
      control.states.push_back(unrelated_control);
      control_pub_->publish(control);
      auto motion = motion_message();
      can_bridge_msgs::msg::MotorMotionStateTarget unrelated_motion;
      unrelated_motion.motor_id = 10;
      unrelated_motion.position = std::numeric_limits<float>::quiet_NaN();
      unrelated_motion.state_age_ms = std::numeric_limits<uint32_t>::max();
      unrelated_motion.target_source = unrelated_motion.TARGET_SOURCE_TRAJECTORY;
      motion.states.push_back(unrelated_motion);
      motion_pub_->publish(motion);
      std::this_thread::sleep_for(2ms);
    }
  });
  const auto result = system->on_activate(rclcpp_lifecycle::State());
  stop.store(true);
  feedback.join();
  ASSERT_EQ(result, CallbackReturn::SUCCESS);
  start_writer(*system);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(spin_until_target_count(2));
  for (const auto &target : commands_.back().targets) {
    EXPECT_TRUE(target.motor_id == 8 || target.motor_id == 9);
  }
}

TEST_F(MicroRosSystemTest, AggregatesStillRequireUniqueAndCompleteConfiguredIds) {
  for (bool duplicate : {false, true}) {
    auto system = initialized(info());
    activate(*system, false);
    auto motion = motion_message();
    if (duplicate) {
      motion.states.push_back(motion.states.front());
    } else {
      motion.states.pop_back();
    }
    motion_pub_->publish(motion);
    publish_control(1, false);
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
  }
}

TEST_F(MicroRosSystemTest, ActivationCannotSucceedAfterRejectedStartupFrame) {
  auto system = initialized(info());
  std::atomic<bool> stop{false};
  std::thread feedback([&] {
    const auto healthy_at = std::chrono::steady_clock::now() + 40ms;
    while (!stop.load()) {
      publish_control(1, false);
      auto motion = motion_message();
      if (std::chrono::steady_clock::now() < healthy_at) {
        motion.states[0].target_source = motion.states[0].TARGET_SOURCE_TRAJECTORY;
      }
      motion_pub_->publish(motion);
      std::this_thread::sleep_for(2ms);
    }
  });
  const auto result = system->on_activate(rclcpp_lifecycle::State());
  stop.store(true);
  feedback.join();
  EXPECT_EQ(result, CallbackReturn::ERROR);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, MeasuredAgeIncludesQueuedSourceDelay) {
  auto system = initialized(info());
  activate(*system, false);
  std::this_thread::sleep_for(60ms);
  auto motion = motion_message(70);
  motion.sync_time_ns -= 50000000U;
  motion_pub_->publish(motion);
  publish_control(1, false);
  std::this_thread::sleep_for(5ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, QueuedMeasuredAgeContinuesAgingAfterReception) {
  auto system = initialized(info());
  activate(*system, false);
  std::this_thread::sleep_for(60ms);
  auto motion = motion_message(40);
  motion.sync_time_ns -= 40000000U;
  motion_pub_->publish(motion);
  publish_control(1, false);
  std::this_thread::sleep_for(5ms);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, ControllerClaimsGateWriterAndRestartUsesFreshMeasurement) {
  auto system = initialized(info());
  activate(*system, false);

  EXPECT_EQ(system->prepare_command_mode_switch({"other/position"}, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system->perform_command_mode_switch({"other/position"}, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system->prepare_command_mode_switch({"pitch/position"}, {}),
            hardware_interface::return_type::ERROR);
  const auto baseline = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(baseline + 1);
  ASSERT_EQ(commands_.size(), baseline + 1);
  EXPECT_EQ(commands_.back().targets.size(), 0U);

  arm(*system);
  auto command_interfaces = system->export_command_interfaces();
  command_interfaces[0].set_value(0.1);
  command_interfaces[2].set_value(0.3);
  const auto after_disarmed_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(after_disarmed_write + 1);
  ASSERT_EQ(commands_.size(), after_disarmed_write + 1);
  EXPECT_EQ(commands_.back().targets.size(), 2U);

  stop_writer(*system);
  const auto before_stop_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(before_stop_write + 1);
  ASSERT_EQ(commands_.size(), before_stop_write + 1);
  EXPECT_EQ(commands_.back().targets.size(), 0U);

  publish_motion();
  publish_control(1, true);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  start_writer(*system);
  const auto before_restart_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(before_restart_write + 1);
  ASSERT_EQ(commands_.size(), before_restart_write + 1);
  ASSERT_EQ(commands_.back().targets.size(), 2U);
  const auto model = robot_transmission::Model::load((root_ / "driver.yaml").string());
  const auto states = system->export_state_interfaces();
  const auto expected = model.joint_to_actuator({
    {"pitch", robot_transmission::Axis(states[0].get_value(), 0.0, 0.0)},
    {"yaw", robot_transmission::Axis(states[3].get_value(), 0.0, 0.0)}});
  for (const auto &target : commands_.back().targets) {
    const auto name = target.motor_id == 8 ? "a" : "b";
    ASSERT_TRUE(expected.count(name));
    EXPECT_NEAR(target.position, expected.at(name).position, 1e-6);
    EXPECT_NEAR(target.velocity, expected.at(name).velocity, 1e-6);
  }
}

TEST_F(MicroRosSystemTest, PolicyActiveWithoutMotionEnabledCanEmitFirstTarget) {
  auto system = initialized(info());
  activate(*system, false);
  for (int cycle = 0; cycle < 5; ++cycle) {
    publish_motion();
    publish_control(1, true, 0, true, false);
    std::this_thread::sleep_for(10ms);
    ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
  }
  start_writer(*system);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(0.1);
  interfaces[2].set_value(0.2);
  const auto before_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(before_write + 1);
  ASSERT_TRUE(spin_until_target_count(2));
  EXPECT_GE(commands_.size(), before_write + 1);
  EXPECT_EQ(commands_.back().targets.size(), 2U);
}

TEST_F(MicroRosSystemTest, EnablingAfterFirstTargetDoesNotRevokeAuthority) {
  auto system = initialized(info());
  activate(*system, false);
  for (int cycle = 0; cycle < 5; ++cycle) {
    publish_motion();
    publish_control(1, true, 0, true, false,
                    can_bridge_msgs::msg::MotorControlState::EXECUTION_ENABLING);
    std::this_thread::sleep_for(10ms);
    ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
  }
  start_writer(*system);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(0.1);
  interfaces[2].set_value(0.2);
  const auto before_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(before_write + 1);
  ASSERT_TRUE(spin_until_target_count(2));
  EXPECT_GE(commands_.size(), before_write + 1);
  EXPECT_EQ(commands_.back().targets.size(), 2U);

  for (int cycle = 0; cycle < 5; ++cycle) {
    publish_control(1, true, 0, true, false,
                    can_bridge_msgs::msg::MotorControlState::EXECUTION_ENABLING);
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
  }
  const auto after_enabling = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(after_enabling + 1);
  ASSERT_TRUE(spin_until_target_count(2));
  EXPECT_GE(commands_.size(), after_enabling + 1);
  EXPECT_EQ(commands_.back().targets.size(), 2U);
}

TEST_F(MicroRosSystemTest, WritesMappedJointPositionAndVelocityOncePerControlCycle) {
  auto system = initialized(info());
  activate(*system, false);
  arm(*system);
  auto command_interfaces = system->export_command_interfaces();
  command_interfaces[0].set_value(0.1);
  command_interfaces[1].set_value(0.2);
  command_interfaces[2].set_value(0.3);
  command_interfaces[3].set_value(0.4);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(1);
  ASSERT_EQ(commands_.size(), 1U);
  ASSERT_EQ(commands_[0].targets.size(), 2U);
  auto model = robot_transmission::Model::load((root_ / "driver.yaml").string());
  const auto expected = model.joint_to_actuator({
    {"pitch", robot_transmission::Axis(0.1, 0.2, 0.0)},
    {"yaw", robot_transmission::Axis(0.3, 0.4, 0.0)}});
  for (const auto &target : commands_[0].targets) {
    const auto name = target.motor_id == 8 ? "a" : "b";
    ASSERT_TRUE(expected.count(name));
    EXPECT_NEAR(target.position, expected.at(name).position, 1e-6);
    EXPECT_NEAR(target.velocity, expected.at(name).velocity, 1e-6);
    EXPECT_NEAR(target.effort, 0.0, 1e-6);
  }
  std::this_thread::sleep_for(40ms);
  rclcpp::spin_some(peer_);
  EXPECT_EQ(commands_.size(), 1U);
  std::this_thread::sleep_for(1ms);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(2);
  ASSERT_EQ(commands_.size(), 2U);
  EXPECT_NE(commands_[1].sequence, commands_[0].sequence);
  EXPECT_NE(commands_[1].sync_time_ns, commands_[0].sync_time_ns);
}

TEST_F(MicroRosSystemTest, JointCommandOutsideUrdfBoundsStopsWholeSnapshot) {
  auto system = initialized(info());
  activate(*system, false);
  arm(*system);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(2.0);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  spin_until_command(1);
  ASSERT_EQ(commands_.size(), 1U);
  EXPECT_EQ(commands_[0].targets.size(), 0U);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, ArmRevocationLatchesUntilExplicitReactivation) {
  auto system = initialized(info());
  activate(*system, false);
  arm(*system);
  publish_control(1, false);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  publish_motion();
  publish_control(1, true);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  activate(*system, true);
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
}

TEST_F(MicroRosSystemTest, SessionChangeInvalidatesMotionFromPreviousSession) {
  auto system = initialized(info());
  activate(*system, false);
  arm(*system);
  publish_control(2, true);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  publish_motion();
  publish_control(2, true);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, SessionChangeWhileActiveButUnarmedLatchesUntilReactivation) {
  auto system = initialized(info());
  activate(*system, false);
  for (int cycle = 0; cycle < 5; ++cycle) {
    publish_motion();
    publish_control(1, false);
    std::this_thread::sleep_for(10ms);
    ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
  }

  std::atomic<bool> stop{false};
  std::thread feedback([&] {
    while (!stop.load()) {
      publish_motion();
      publish_control(2, false);
      std::this_thread::sleep_for(2ms);
    }
  });
  for (int cycle = 0; cycle < 12; ++cycle) {
    std::this_thread::sleep_for(5ms);
    EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
  }
  stop.store(true);
  feedback.join();
  EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  activate(*system, true);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
}

TEST_F(MicroRosSystemTest, ExpiredMeasuredMotionLatchesAndDoesNotRecoverOnFreshInput) {
  auto system = initialized(info());
  ASSERT_EQ(run_activation(*system, true, 90), CallbackReturn::SUCCESS);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  publish_motion();
  publish_control(1, true);
  EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
}

TEST_F(MicroRosSystemTest, TenHzControlStreamPermitsFreshMotionAndCommands) {
  auto system = initialized(info());
  std::atomic<bool> stop{false};
  std::thread feedback([&] {
    while (!stop.load()) {
      publish_motion();
      publish_control(1, true, std::numeric_limits<uint32_t>::max());
      std::this_thread::sleep_for(2ms);
    }
  });
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  stop.store(true);
  feedback.join();
  start_writer(*system);
  for (int cycle = 0; cycle < 12; ++cycle) {
    std::this_thread::sleep_for(10ms);
    publish_motion();
    EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::OK);
  }
  auto command_interfaces = system->export_command_interfaces();
  command_interfaces[0].set_value(0.1);
  command_interfaces[2].set_value(0.2);
  const auto before_write = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(before_write + 1);
  ASSERT_EQ(commands_.size(), before_write + 1);
  EXPECT_EQ(commands_.back().targets.size(), 2U);
}

TEST_F(MicroRosSystemTest, DeactivatePublishesEmptySnapshotAndStops) {
  auto system = initialized(info());
  activate(*system, false);
  const auto before_deactivate = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  spin_until_command(before_deactivate + 1);
  ASSERT_EQ(commands_.size(), before_deactivate + 1);
  EXPECT_EQ(commands_.back().targets.size(), 0U);
  const auto count = commands_.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::ERROR);
  std::this_thread::sleep_for(20ms);
  rclcpp::spin_some(peer_);
  EXPECT_EQ(commands_.size(), count);
}

TEST_F(MicroRosSystemTest, OptionalEffortInterfacesValidateBoundsAndGroupClaims) {
  auto system = initialized(effort_info());
  EXPECT_EQ(system->export_command_interfaces().size(), 6U);
  EXPECT_EQ(system->prepare_command_mode_switch({"pitch/effort"}, {}),
            hardware_interface::return_type::ERROR);
  auto pve = all_command_interfaces();
  const auto effort = effort_interfaces();
  pve.insert(pve.end(), effort.begin(), effort.end());
  EXPECT_EQ(system->prepare_command_mode_switch(pve, {}), hardware_interface::return_type::OK);
  for (const auto &bound : {"nan", "3"}) {
    auto invalid = effort_info();
    invalid.joints[0].command_interfaces.back().min = bound;
    motor_hardware_interface::MicroRosSystem rejected;
    EXPECT_EQ(rejected.on_init(invalid), CallbackReturn::ERROR);
  }
}

TEST_F(MicroRosSystemTest, MapsEffortAndClearsItAcrossControllerSwitches) {
  auto system = initialized(effort_info());
  activate(*system);
  arm(*system);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(0.1);
  interfaces[1].set_value(0.2);
  interfaces[3].set_value(0.3);
  ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), {}),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(interfaces[0].get_value(), 0.1);
  EXPECT_DOUBLE_EQ(interfaces[1].get_value(), 0.2);
  interfaces[2].set_value(0.8);
  interfaces[5].set_value(-0.6);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(1);
  ASSERT_EQ(commands_.size(), 1U);
  ASSERT_EQ(commands_.back().targets.size(), 2U);
  const auto model = robot_transmission::Model::load((root_ / "driver.yaml").string());
  const auto expected = model.joint_to_actuator({
    {"pitch", robot_transmission::Axis(0.1, 0.2, 0.8)},
    {"yaw", robot_transmission::Axis(0.3, 0.0, -0.6)}});
  for (const auto &target : commands_.back().targets) {
    const auto &axis = expected.at(target.motor_id == 8 ? "a" : "b");
    EXPECT_NEAR(target.position, axis.position, 1e-6);
    EXPECT_NEAR(target.velocity, axis.velocity, 1e-6);
    EXPECT_NEAR(target.effort, axis.effort, 1e-6);
  }
  ASSERT_EQ(system->perform_command_mode_switch({}, effort_interfaces()),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
  EXPECT_DOUBLE_EQ(interfaces[5].get_value(), 0.0);
  interfaces[2].set_value(1.0);  // 已释放的接口即使残留写入也不得参与输出。
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(2);
  ASSERT_EQ(commands_.size(), 2U);
  for (const auto &target : commands_.back().targets) {
    EXPECT_FLOAT_EQ(target.effort, 0.0f);
  }
  ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), {}),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
  stop_writer(*system);
  interfaces[2].set_value(1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  spin_until_command(3);
  ASSERT_EQ(commands_.size(), 3U);
  EXPECT_TRUE(commands_.back().targets.empty());
  start_writer(*system);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
}

TEST_F(MicroRosSystemTest, EffortOnlyNeverDrivesAndManualSwitchDropsOldTorque) {
  auto system = initialized(effort_info());
  activate(*system, true);
  ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), {}),
            hardware_interface::return_type::OK);
  auto interfaces = system->export_command_interfaces();
  interfaces[2].set_value(1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(spin_until_target_count(0));
  start_writer(*system);
  interfaces[2].set_value(1.0);
  auto pve = all_command_interfaces();
  const auto effort = effort_interfaces();
  pve.insert(pve.end(), effort.begin(), effort.end());
  ASSERT_EQ(system->perform_command_mode_switch(pve, pve), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
  ASSERT_EQ(system->perform_command_mode_switch({}, pve), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(spin_until_target_count(0));
}

TEST_F(MicroRosSystemTest, InvalidEffortStopsWholeSnapshotAndLatches) {
  for (const double invalid : {2.1, -2.1, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()}) {
    auto system = initialized(effort_info());
    activate(*system);
    arm(*system);
    ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), {}),
              hardware_interface::return_type::OK);
    auto interfaces = system->export_command_interfaces();
    interfaces[2].set_value(invalid);
    EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
    ASSERT_TRUE(spin_until_target_count(0));
    EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
    interfaces[2].set_value(0.0);
    EXPECT_EQ(system->write(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
    system->on_deactivate(rclcpp_lifecycle::State());
  }
}

TEST_F(MicroRosSystemTest, StopTopicParksClaimedControllersUntilFullRestartAndNewAuthorization) {
  auto system = initialized(effort_info());
  activate(*system);
  arm(*system);
  ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), {}),
            hardware_interface::return_type::OK);
  auto interfaces = system->export_command_interfaces();
  interfaces[0].set_value(0.8);
  interfaces[2].set_value(1.0);
  expect_snapshot(*system, 2);

  stop_via_topic(*system);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
  for (auto &interface : interfaces) { interface.set_value(0.5); }
  publish_motion();
  publish_control(1, true);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  expect_snapshot(*system, 0);

  publish_motion();
  publish_control(1, false);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  expect_snapshot(*system, 0);
  stop_writer(*system);
  ASSERT_EQ(system->perform_command_mode_switch(effort_interfaces(), effort_interfaces()),
            hardware_interface::return_type::OK);
  expect_snapshot(*system, 0);

  publish_motion();
  publish_control(1, true);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  expect_snapshot(*system, 0);
  publish_motion();
  publish_control(1, false);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  start_writer(*system);
  expect_snapshot(*system, 0);

  publish_motion();
  publish_control(1, true);
  ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
            hardware_interface::return_type::OK);
  expect_snapshot(*system, 2);
  const auto states = system->export_state_interfaces();
  EXPECT_DOUBLE_EQ(interfaces[0].get_value(), states[0].get_value());
  EXPECT_DOUBLE_EQ(interfaces[3].get_value(), states[3].get_value());
  EXPECT_DOUBLE_EQ(interfaces[1].get_value(), 0.0);
  EXPECT_DOUBLE_EQ(interfaces[2].get_value(), 0.0);
}

TEST_F(MicroRosSystemTest, StopTopicDoesNotBypassFeedbackOrSessionFaultLatch) {
  for (const bool changed_session : {false, true}) {
    auto system = initialized(info());
    activate(*system);
    arm(*system);
    expect_snapshot(*system, 2);
    stop_via_topic(*system);
    publish_motion(changed_session ? 0 : 1000);
    publish_control(changed_session ? 2 : 1, true);
    ASSERT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
    stop_writer(*system);
    publish_motion();
    publish_control(1, true);
    EXPECT_EQ(system->read(rclcpp::Time(0), rclcpp::Duration(10ms)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(system->perform_command_mode_switch(all_command_interfaces(), {}),
              hardware_interface::return_type::ERROR);
    system->on_deactivate(rclcpp_lifecycle::State());
  }
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
