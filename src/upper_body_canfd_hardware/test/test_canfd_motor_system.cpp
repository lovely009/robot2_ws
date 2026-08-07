#include <gtest/gtest.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"

#include "upper_body_canfd_hardware/canfd_motor_system.hpp"
#include "upper_body_canfd_hardware/rp_mit.hpp"
#include "upper_body_canfd_hardware/wrist_2530.hpp"

namespace upper_body_canfd_hardware {
namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;

constexpr int kFeedbackTimeoutCycles = 10;
const rclcpp::Time kTestTime(0, 0, RCL_SYSTEM_TIME);
const rclcpp::Duration kTestPeriod(0, 0);
const rclcpp_lifecycle::State kTestState(
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");

bool VcanAvailable(const std::string & interface)
{
  try {
    SocketCanBus probe(interface);
    return probe.IsOpen();
  } catch (const std::exception &) {
    return false;
  }
}

ComponentInfo MakeJoint(
  const std::string & name,
  const std::unordered_map<std::string, std::string> & params)
{
  ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces.push_back(InterfaceInfo{"effort"});
  joint.state_interfaces.push_back(InterfaceInfo{"position"});
  joint.state_interfaces.push_back(InterfaceInfo{"velocity"});
  joint.state_interfaces.push_back(InterfaceInfo{"effort"});
  joint.parameters = params;
  return joint;
}

HardwareInfo MakeWristInfo(
  const std::string & interface,
  const std::unordered_map<std::string, std::string> & joint_params)
{
  HardwareInfo info;
  info.name = "TestWristSystem";
  info.type = "system";
  info.hardware_class_type = "upper_body_canfd_hardware/CanFdMotorSystem";
  info.hardware_parameters = {
    {"interface", interface},
    {"allow_hardware_interfaces", "true"},
    {"feedback_timeout_cycles", std::to_string(kFeedbackTimeoutCycles)},
  };
  info.joints.push_back(MakeJoint("left_wrist_roll_joint", joint_params));
  return info;
}

HardwareInfo MakeRpInfo(const std::string & interface)
{
  HardwareInfo info;
  info.name = "TestRpSystem";
  info.type = "system";
  info.hardware_class_type = "upper_body_canfd_hardware/CanFdMotorSystem";
  info.hardware_parameters = {
    {"interface", interface},
    {"allow_hardware_interfaces", "true"},
    {"feedback_timeout_cycles", std::to_string(kFeedbackTimeoutCycles)},
  };
  const std::vector<std::string> names{
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_pitch_joint",
    "left_elbow_pitch_joint",
  };
  for (size_t i = 0; i < names.size(); ++i) {
    info.joints.push_back(MakeJoint(
      names[i],
      {{"motor_type", "rp"}, {"can_node_id", std::to_string(i + 1)}}));
  }
  return info;
}

// Bus-side motor emulator: answers RP MIT broadcasts and 25/30 mm MIT commands
// with the same codecs the plugin uses, and records what it received.
class FakeSlave {
public:
  FakeSlave(const std::string & interface, int rp_count, std::vector<int> wrist_nodes)
  : bus_(interface), rp_count_(rp_count), wrist_nodes_(std::move(wrist_nodes))
  {
    thread_ = std::thread([this]() { Loop(); });
  }

  ~FakeSlave()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  std::atomic<bool> rp_enabled{false};
  std::atomic<bool> wrist_enabled{false};

  rp::MitCommand LastRpSlot(int index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_rp_slots_[index];
  }

  bool GotRpBroadcast()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return got_rp_broadcast_;
  }

  wrist::MitCommand LastWristCommand()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_wrist_command_;
  }

  bool GotWristCommand()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return got_wrist_command_;
  }

private:
  void Loop()
  {
    while (running_) {
      const auto frames = bus_.RecvAvailable(32);
      for (const auto & frame : frames) {
        Handle(frame);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void Handle(const CanFrame & frame)
  {
    if (frame.is_extended_id) {
      return;
    }
    const uint32_t id = frame.arbitration_id;
    if (id == rp::kEnableId && frame.data.size() == 8) {
      rp_enabled = frame.data[0] == 1;
      return;
    }
    if (id == rp::kMitBroadcastId && frame.data.size() == 64) {
      const auto slots = rp::UnpackBroadcast(frame.data);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_rp_slots_ = slots;
        got_rp_broadcast_ = true;
      }
      for (int i = 0; i < rp_count_; ++i) {
        bus_.Send(
          static_cast<uint32_t>(i + 1),
          rp::PackState(0.6 + 0.1 * i, 0.05, slots[i].torque_ff), false, true);
      }
      return;
    }
    for (const int node : wrist_nodes_) {
      if (id == wrist::MitCommandId(node) && frame.data.size() == 12) {
        const auto command = wrist::UnpackMitCommand(frame.data);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_wrist_command_ = command;
          got_wrist_command_ = true;
        }
        bus_.Send(
          wrist::MitReplyId(node),
          wrist::PackMitReply(node, 0.25, 0.1, command.current_ff), true, true);
        return;
      }
      if (id == wrist::MotorControlId(node) && frame.data.size() == 8) {
        wrist_enabled = frame.data[0] == 1;
        return;
      }
    }
  }

  SocketCanBus bus_;
  int rp_count_;
  std::vector<int> wrist_nodes_;
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<rp::MitCommand> last_rp_slots_;
  wrist::MitCommand last_wrist_command_;
  bool got_rp_broadcast_ = false;
  bool got_wrist_command_ = false;
};

hardware_interface::CommandInterface * FindCommand(
  std::vector<hardware_interface::CommandInterface> & interfaces,
  const std::string & joint, const std::string & name)
{
  for (auto & interface : interfaces) {
    if (interface.get_prefix_name() == joint && interface.get_interface_name() == name) {
      return &interface;
    }
  }
  return nullptr;
}

hardware_interface::StateInterface * FindState(
  std::vector<hardware_interface::StateInterface> & interfaces,
  const std::string & joint, const std::string & name)
{
  for (auto & interface : interfaces) {
    if (interface.get_prefix_name() == joint && interface.get_interface_name() == name) {
      return &interface;
    }
  }
  return nullptr;
}

TEST(TestCanFdMotorSystem, WristSingleJointLoop)
{
  if (!VcanAvailable("vcan0")) {
    GTEST_SKIP() << "vcan0 not available";
  }
  auto info = MakeWristInfo("vcan0", {
    {"motor_type", "wrist"},
    {"can_node_id", "3"},
    {"direction", "1"},
    {"zero_offset", "0"},
    {"kp", "0.008"},
    {"kd", "1.0"},
    {"effort_scale", "1.0"},
  });

  FakeSlave slave("vcan0", 0, {3});
  CanFdMotorSystem hardware;
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);

  auto state_interfaces = hardware.export_state_interfaces();
  auto command_interfaces = hardware.export_command_interfaces();
  auto * cmd_effort = FindCommand(command_interfaces, "left_wrist_roll_joint", "effort");
  auto * pos = FindState(state_interfaces, "left_wrist_roll_joint", "position");
  auto * vel = FindState(state_interfaces, "left_wrist_roll_joint", "velocity");
  auto * eff = FindState(state_interfaces, "left_wrist_roll_joint", "effort");
  ASSERT_NE(cmd_effort, nullptr);
  ASSERT_NE(pos, nullptr);
  ASSERT_NE(vel, nullptr);
  ASSERT_NE(eff, nullptr);

  ASSERT_EQ(hardware.on_configure(kTestState), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(kTestState), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(slave.wrist_enabled.load());

  cmd_effort->set_value(1.0);
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  // Second cycle: the plugin holds feedback and sends the commanded MIT frame.
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);

  EXPECT_NEAR(pos->get_value(), 0.25, 1e-4);
  EXPECT_NEAR(vel->get_value(), 0.1, 0.01);
  EXPECT_NEAR(eff->get_value(), 1.0, 0.003);

  EXPECT_TRUE(slave.GotWristCommand());
  const auto command = slave.LastWristCommand();
  EXPECT_NEAR(command.current_ff, 1.0, 0.003);
  EXPECT_NEAR(command.kp, 0.008, 1e-5);
  EXPECT_NEAR(command.kd, 1.0, 1e-4);

  // Effort clamping to the wrist URDF limit (default +/-5 Nm).
  cmd_effort->set_value(99.0);
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  EXPECT_NEAR(slave.LastWristCommand().current_ff, 5.0, 0.003);
  EXPECT_NEAR(eff->get_value(), 5.0, 0.003);

  // Deactivate sends disable and stops the command stream.
  ASSERT_EQ(hardware.on_deactivate(kTestState), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(slave.wrist_enabled.load());
}

TEST(TestCanFdMotorSystem, FeedbackTimeoutDetected)
{
  if (!VcanAvailable("vcan1")) {
    GTEST_SKIP() << "vcan1 not available";
  }
  auto info = MakeWristInfo("vcan1", {
    {"motor_type", "wrist"},
    {"can_node_id", "1"},
  });
  std::unique_ptr<FakeSlave> slave = std::make_unique<FakeSlave>("vcan1", 0, std::vector<int>{1});
  CanFdMotorSystem hardware;
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  auto state_interfaces = hardware.export_state_interfaces();
  auto command_interfaces = hardware.export_command_interfaces();
  ASSERT_EQ(hardware.on_configure(kTestState), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(kTestState), CallbackReturn::SUCCESS);

  auto * cmd_effort = FindCommand(command_interfaces, "left_wrist_roll_joint", "effort");
  ASSERT_NE(cmd_effort, nullptr);
  cmd_effort->set_value(0.5);
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);

  slave.reset();  // stop replying
  hardware_interface::return_type result = hardware_interface::return_type::OK;
  for (int i = 0; i < kFeedbackTimeoutCycles + 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    result = hardware.read(kTestTime, kTestPeriod);
    if (result == hardware_interface::return_type::ERROR) {
      break;
    }
  }
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST(TestCanFdMotorSystem, DirectionOffsetAndScaleMapping)
{
  if (!VcanAvailable("vcan0")) {
    GTEST_SKIP() << "vcan0 not available";
  }
  auto info = MakeWristInfo("vcan0", {
    {"motor_type", "wrist"},
    {"can_node_id", "2"},
    {"direction", "-1"},
    {"zero_offset", "0.3"},
    {"effort_scale", "2.0"},
    {"kp", "0.004"},
    {"kd", "0.5"},
  });
  FakeSlave slave("vcan0", 0, {2});
  CanFdMotorSystem hardware;
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  auto state_interfaces = hardware.export_state_interfaces();
  auto command_interfaces = hardware.export_command_interfaces();
  ASSERT_EQ(hardware.on_configure(kTestState), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(kTestState), CallbackReturn::SUCCESS);

  auto * cmd_effort = FindCommand(command_interfaces, "left_wrist_roll_joint", "effort");
  auto * pos = FindState(state_interfaces, "left_wrist_roll_joint", "position");
  auto * vel = FindState(state_interfaces, "left_wrist_roll_joint", "velocity");
  auto * eff = FindState(state_interfaces, "left_wrist_roll_joint", "effort");
  ASSERT_NE(cmd_effort, nullptr);
  ASSERT_NE(pos, nullptr);
  ASSERT_NE(vel, nullptr);
  ASSERT_NE(eff, nullptr);

  cmd_effort->set_value(1.0);
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);

  // Motor replies position 0.25 / velocity 0.1; direction -1 and offset 0.3:
  // joint = (motor - offset) * direction.
  EXPECT_NEAR(pos->get_value(), (0.25 - 0.3) * -1.0, 1e-4);
  EXPECT_NEAR(vel->get_value(), -0.1, 0.01);
  // current_ff commanded = direction * effort * scale = -2.0; reply echoes it,
  // joint effort = current * direction / scale = 1.0.
  EXPECT_NEAR(eff->get_value(), 1.0, 0.003);
  EXPECT_NEAR(slave.LastWristCommand().current_ff, -2.0, 0.003);
  EXPECT_NEAR(slave.LastWristCommand().kp, 0.004, 1e-5);
}

TEST(TestCanFdMotorSystem, RpBroadcastFourJoints)
{
  if (!VcanAvailable("vcan1")) {
    GTEST_SKIP() << "vcan1 not available";
  }
  auto info = MakeRpInfo("vcan1");
  FakeSlave slave("vcan1", 4, {});
  CanFdMotorSystem hardware;
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  auto state_interfaces = hardware.export_state_interfaces();
  auto command_interfaces = hardware.export_command_interfaces();
  ASSERT_EQ(hardware.on_configure(kTestState), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(kTestState), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(slave.rp_enabled.load());

  hardware_interface::CommandInterface * cmd[4];
  hardware_interface::StateInterface * pos[4];
  hardware_interface::StateInterface * vel[4];
  hardware_interface::StateInterface * eff[4];
  const std::vector<std::string> names{
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_pitch_joint",
    "left_elbow_pitch_joint",
  };
  for (int i = 0; i < 4; ++i) {
    cmd[i] = FindCommand(command_interfaces, names[i], "effort");
    pos[i] = FindState(state_interfaces, names[i], "position");
    vel[i] = FindState(state_interfaces, names[i], "velocity");
    eff[i] = FindState(state_interfaces, names[i], "effort");
    ASSERT_NE(cmd[i], nullptr);
    ASSERT_NE(pos[i], nullptr);
    ASSERT_NE(vel[i], nullptr);
    ASSERT_NE(eff[i], nullptr);
    cmd[i]->set_value(1.0);
  }

  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);

  // Second command cycle: the plugin holds the latest feedback position as the
  // MIT position setpoint with configured gains and feed-forward torque.
  ASSERT_EQ(hardware.write(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_EQ(hardware.read(kTestTime, kTestPeriod), hardware_interface::return_type::OK);
  EXPECT_TRUE(slave.GotRpBroadcast());
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(pos[i]->get_value(), 0.6 + 0.1 * i, 5e-4);
    EXPECT_NEAR(vel[i]->get_value(), 0.05, 0.01);
    EXPECT_NEAR(eff[i]->get_value(), 1.0, 0.02);
  }
  for (int i = 0; i < 4; ++i) {
    const auto slot = slave.LastRpSlot(i);
    EXPECT_NEAR(slot.position, 0.6 + 0.1 * i, 5e-4);
    EXPECT_NEAR(slot.velocity, 0.0, 0.02);
    EXPECT_NEAR(slot.kp, 36.0, 0.2);
    EXPECT_NEAR(slot.kd, 2.0, 1e-3);
    EXPECT_NEAR(slot.torque_ff, 1.0, 0.02);
  }
  for (int i = 4; i < 8; ++i) {
    const auto slot = slave.LastRpSlot(i);
    EXPECT_NEAR(slot.position, 0.0, 5e-4);
    EXPECT_NEAR(slot.torque_ff, 0.0, 0.02);
    EXPECT_NEAR(slot.kp, 0.0, 1e-6);
  }

  ASSERT_EQ(hardware.on_deactivate(kTestState), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(slave.rp_enabled.load());
}

}  // namespace
}  // namespace upper_body_canfd_hardware
