#include "motor_sim/ros/motor_sim_node.hpp"
#include "humanoid_gazebo/topology.hpp"
#include "motor_sim/protocol/motor_protocol.hpp"
#include "motor_sim/transport/frame_record.hpp"
#include "motor_sim/transport/serial_transport.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace motor_ros = motor_sim::ros;
namespace protocol = ::motor_sim::protocol;
namespace transport = ::motor_sim::transport;
using namespace std::chrono_literals;

namespace {

const std::string kMotorsYaml =
    std::string(TEST_SOURCE_DIR) + "/config/motors.yaml";

class PseudoTerminal {
public:
  PseudoTerminal() {
    char path[128]{};
    int slave = -1;
    if (::openpty(&master_, &slave, path, nullptr, nullptr) < 0) {
      throw std::runtime_error("openpty failed");
    }
    slave_path_ = path;
    ::close(slave);
    const int flags = ::fcntl(master_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(master_, F_SETFL, flags | O_NONBLOCK) < 0) {
      close_master();
      throw std::runtime_error("cannot make PTY master nonblocking");
    }
  }

  ~PseudoTerminal() { close_master(); }

  PseudoTerminal(const PseudoTerminal &) = delete;
  PseudoTerminal &operator=(const PseudoTerminal &) = delete;

  const std::string &slave_path() const noexcept { return slave_path_; }

  void send(const protocol::CanFrame &frame) {
    const auto record = transport::encode_frame_record(frame);
    std::size_t offset = 0U;
    while (offset < record.size()) {
      const auto count =
          ::write(master_, record.data() + offset, record.size() - offset);
      if (count <= 0) {
        throw std::runtime_error("cannot write PTY command");
      }
      offset += static_cast<std::size_t>(count);
    }
  }

  bool received_feedback() {
    std::array<char, 4096> bytes{};
    while (master_ >= 0) {
      const auto count = ::read(master_, bytes.data(), bytes.size());
      if (count <= 0) {
        break;
      }
      received_.append(bytes.data(), static_cast<std::size_t>(count));
    }
    return received_.find('\r') != std::string::npos;
  }

  std::size_t drain_feedback_records() {
    std::array<char, 4096> bytes{};
    while (master_ >= 0) {
      const auto count = ::read(master_, bytes.data(), bytes.size());
      if (count <= 0) {
        break;
      }
      received_.append(bytes.data(), static_cast<std::size_t>(count));
    }
    const auto records =
        static_cast<std::size_t>(std::count(received_.begin(), received_.end(), '\r'));
    const auto last = received_.rfind('\r');
    if (last != std::string::npos) {
      received_.erase(0U, last + 1U);
    }
    return records;
  }

  void close_master() noexcept {
    if (master_ >= 0) {
      ::close(master_);
      master_ = -1;
    }
  }

private:
  int master_{-1};
  std::string slave_path_;
  std::string received_;
};

protocol::CanFrame rp_enable_frame() {
  protocol::CanFrame frame;
  frame.id = protocol::kRpEnableId;
  frame.size = 8U;
  frame.data[0] = 1U;
  return frame;
}

protocol::CanFrame rp_command_frame(const double first_position,
                                    const bool legs) {
  protocol::CanFrame frame;
  frame.id = protocol::kRpCommandId;
  frame.fd = true;
  frame.brs = true;
  frame.size = 64U;
  protocol::MotorCommand neutral;
  neutral.enabled = true;
  for (std::size_t slot = 0U; slot < protocol::kRpBroadcastSlots; ++slot) {
    auto command = neutral;
    if ((!legs && slot == 5U) || (legs && (slot == 3U || slot == 7U))) {
      // 肘、膝的命令下限为正，避免量化后的零值触发 margin。
      command.position = 0.1;
    }
    const auto payload = protocol::pack_rp_slot(command);
    std::copy(payload.begin(), payload.end(), frame.data.begin() + slot * 8U);
  }
  auto first = neutral;
  first.position = first_position;
  const auto payload = protocol::pack_rp_slot(first);
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_enable_frame(const unsigned node) {
  const auto payload = protocol::pack_vaxor_control(
      true, false, protocol::CommandMode::Mit);
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_control_id(node);
  frame.size = payload.size();
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

protocol::CanFrame vaxor_command_frame(const unsigned node,
                                       const double motor_position) {
  protocol::MotorCommand command;
  command.enabled = true;
  command.position = motor_position;
  command.kp = 0.008;
  command.kd = 1.0;
  const auto payload = protocol::pack_vaxor_command(command);
  protocol::CanFrame frame;
  frame.id = protocol::vaxor_command_id(node);
  frame.fd = true;
  frame.brs = true;
  frame.size = payload.size();
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

bool spin_until(rclcpp::executors::SingleThreadedExecutor &executor,
                const std::function<bool()> &predicate,
                const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

}  // namespace

TEST(PtyIntegration, ThreeBusRoundTripAndDisconnect) {
  int argc = 0;
  char **argv = nullptr;
  rclcpp::init(argc, argv);

  PseudoTerminal left_arm;
  PseudoTerminal right_arm;
  PseudoTerminal legs;
  motor_ros::TransportMap transports;
  transports.emplace(
      "left_arm", std::make_unique<transport::SerialTransport>(
                      left_arm.slave_path(),
                      transport::TransportMode::UsbSerial, 3000000));
  transports.emplace(
      "right_arm", std::make_unique<transport::SerialTransport>(
                       right_arm.slave_path(),
                       transport::TransportMode::UsbSerial, 3000000));
  transports.emplace(
      "legs", std::make_unique<transport::SerialTransport>(
                  legs.slave_path(), transport::TransportMode::UsbSerial,
                  3000000));

  auto config = ::motor_sim::core::load_system_config(
      kMotorsYaml, humanoid_gazebo::humanoid_topology_contract());
  const auto joint_names = [&config] {
    std::vector<std::string> names;
    names.reserve(config.motors.size());
    for (const auto &motor : config.motors) {
      names.push_back(motor.joint);
    }
    return names;
  }();
  auto simulator = std::make_shared<motor_ros::MotorSimNode>(
      rclcpp::NodeOptions{}, std::move(config), std::move(transports));
  auto observer = std::make_shared<rclcpp::Node>("pty_integration_observer");
  auto joint_states = observer->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

  bool saw_nonzero_effort = false;
  bool disconnect_started = false;
  bool saw_zero_after_disconnect = false;
  std::vector<double> observed_efforts;
  std::vector<std::chrono::steady_clock::time_point> effort_arrivals;
  auto effort_subscription =
      observer->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/effort_controller/commands", 10,
          [&saw_nonzero_effort, &disconnect_started,
           &saw_zero_after_disconnect, &observed_efforts, &effort_arrivals](
              const std_msgs::msg::Float64MultiArray &message) {
            if (!disconnect_started) {
              effort_arrivals.push_back(std::chrono::steady_clock::now());
            }
            observed_efforts = message.data;
            saw_nonzero_effort =
                saw_nonzero_effort ||
                std::any_of(message.data.begin(), message.data.end(),
                            [](const double value) {
                              return std::abs(value) > 0.05;
                            });
            saw_zero_after_disconnect =
                saw_zero_after_disconnect ||
                (disconnect_started && message.data.size() == 30U &&
                 std::all_of(message.data.begin(), message.data.end(),
                             [](const double value) { return value == 0.0; }));
          });
  ASSERT_NE(effort_subscription, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(simulator);
  executor.add_node(observer);

  sensor_msgs::msg::JointState state;
  state.name = joint_names;
  state.position.assign(joint_names.size(), 0.0);
  state.velocity.assign(joint_names.size(), 0.0);
  ASSERT_TRUE(spin_until(
      executor,
      [&] {
        joint_states->publish(state);
        return joint_states->get_subscription_count() == 1U;
      },
      2s));

  left_arm.send(rp_enable_frame());
  left_arm.send(rp_command_frame(0.1, false));
  right_arm.send(vaxor_enable_frame(1U));
  // forearm 36:1，电机侧 3.6 rad 对应关节侧 0.1 rad。
  right_arm.send(vaxor_command_frame(1U, 3.6));
  legs.send(rp_enable_frame());
  legs.send(rp_command_frame(0.2, true));

  ASSERT_TRUE(spin_until(
      executor,
      [&] {
        joint_states->publish(state);
        return saw_nonzero_effort && left_arm.received_feedback() &&
               right_arm.received_feedback() && legs.received_feedback();
      },
      120ms));
  ASSERT_EQ(observed_efforts.size(), 30U);

  const char *endurance_value = std::getenv("HUMANOID_PTY_ENDURANCE_SECONDS");
  const long endurance_seconds =
      endurance_value == nullptr ? 0L : std::strtol(endurance_value, nullptr, 10);
  if (endurance_seconds > 0L) {
    std::array<std::chrono::steady_clock::time_point, 3> last_feedback{};
    const auto started = std::chrono::steady_clock::now();
    auto next_command = started;
    const auto deadline = started + std::chrono::seconds(endurance_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_command) {
        left_arm.send(rp_command_frame(0.1, false));
        right_arm.send(vaxor_command_frame(1U, 3.6));
        legs.send(rp_command_frame(0.2, true));
        joint_states->publish(state);
        next_command += 10ms;
      }
      executor.spin_some();
      if (left_arm.drain_feedback_records() > 0U) {
        last_feedback[0] = now;
      }
      if (right_arm.drain_feedback_records() > 0U) {
        last_feedback[1] = now;
      }
      if (legs.drain_feedback_records() > 0U) {
        last_feedback[2] = now;
      }
      ASSERT_FALSE(simulator->shutdown_requested());
      std::this_thread::sleep_for(1ms);
    }
    const auto finished = std::chrono::steady_clock::now();
    for (const auto feedback_at : last_feedback) {
      EXPECT_LT(finished - feedback_at, 200ms);
    }
    std::vector<double> jitter_ms;
    for (std::size_t index = 1U; index < effort_arrivals.size(); ++index) {
      const auto interval = effort_arrivals[index] - effort_arrivals[index - 1U];
      const double interval_ms =
          std::chrono::duration<double, std::milli>(interval).count();
      jitter_ms.push_back(std::abs(interval_ms - 10.0));
    }
    ASSERT_FALSE(jitter_ms.empty());
    std::sort(jitter_ms.begin(), jitter_ms.end());
    const auto p99_index = static_cast<std::size_t>(
        std::floor(0.99 * static_cast<double>(jitter_ms.size() - 1U)));
    const double p99_jitter_ms = jitter_ms[p99_index];
    EXPECT_LE(p99_jitter_ms, 2.0);
    std::cout << "PTY_ENDURANCE seconds=" << endurance_seconds
              << " effort_samples=" << effort_arrivals.size()
              << " p99_jitter_ms=" << p99_jitter_ms << std::endl;
  }

  disconnect_started = true;
  legs.close_master();
  ASSERT_TRUE(spin_until(
      executor,
      [&] {
        return simulator->shutdown_requested() && saw_zero_after_disconnect;
      },
      200ms));
  EXPECT_EQ(simulator->last_efforts(), std::vector<double>(30U, 0.0));

  executor.remove_node(observer);
  executor.remove_node(simulator);
  observer.reset();
  simulator.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}
