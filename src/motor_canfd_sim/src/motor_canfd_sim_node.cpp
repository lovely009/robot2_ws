#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace motor_canfd_sim {

namespace protocol = motor_canfd_drive::protocol;
using Clock = std::chrono::steady_clock;

struct JointSample {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct RpDriveState {
  bool enabled{false};
  protocol::JointCommand command{};
  Clock::time_point last_command{};
};

struct WristDriveState {
  bool enabled{false};
  protocol::WristMode mode{protocol::WristMode::Mit};
  double zero_offset{0.0};
  protocol::JointCommand command{};
  Clock::time_point last_command{};
  double current_a{0.0};
  uint8_t motor_type{0x01};
};

double clamp_symmetric(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

class MotorCanFdSimNode final : public rclcpp::Node {
public:
  MotorCanFdSimNode() : Node("motor_canfd_sim") {
    const auto left_interface = declare_parameter<std::string>("left_interface", "vcan0");
    const auto right_interface = declare_parameter<std::string>("right_interface", "vcan1");
    const auto allow_hardware = declare_parameter<bool>("allow_hardware_interfaces", false);
    const auto control_rate_hz = declare_parameter<double>("control_rate_hz", 500.0);
    const auto status_rate_hz = declare_parameter<double>("status_rate_hz", 50.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.15);
    rp_torque_limit_nm_ = declare_parameter<double>("rp_torque_limit_nm", 60.0);
    wrist_torque_constant_ =
        declare_parameter<double>("wrist_torque_constant_nm_per_amp", 0.35);
    wrist_current_limit_a_ = declare_parameter<double>("wrist_current_limit_a", 11.0);
    wrist_torque_limit_nm_ = declare_parameter<double>("wrist_torque_limit_nm", 5.0);
    wrist_position_gain_ =
        declare_parameter<double>("wrist_position_mode_gain_a_per_rad", 5.0);
    wrist_position_damping_ =
        declare_parameter<double>("wrist_position_mode_damping_a_per_rad_s", 0.08);
    wrist_velocity_gain_ =
        declare_parameter<double>("wrist_velocity_mode_gain_a_per_rad_s", 0.45);

    const auto rp_ids_raw = declare_parameter<std::vector<int64_t>>(
        "rp_ids", std::vector<int64_t>{1, 2, 3, 4});
    const auto wrist_nodes_raw = declare_parameter<std::vector<int64_t>>(
        "wrist_nodes", std::vector<int64_t>{1, 2, 3});
    const auto joint_names_raw = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"left_shoulder_yaw_joint", "left_shoulder_roll_joint",
         "left_shoulder_pitch_joint", "left_elbow_pitch_joint",
         "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
         "right_shoulder_yaw_joint", "right_shoulder_roll_joint",
         "right_shoulder_pitch_joint", "right_elbow_pitch_joint",
         "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint"});

    validate_parameters(control_rate_hz, status_rate_hz, rp_ids_raw,
                        wrist_nodes_raw, joint_names_raw);
    for (std::size_t index = 0; index < rp_ids_.size(); ++index) {
      rp_ids_[index] = static_cast<unsigned>(rp_ids_raw[index]);
    }
    for (std::size_t index = 0; index < wrist_nodes_.size(); ++index) {
      wrist_nodes_[index] = static_cast<unsigned>(wrist_nodes_raw[index]);
    }
    std::copy(joint_names_raw.begin(), joint_names_raw.end(), joint_names_.begin());

    const std::array<std::string, 2> interfaces{{left_interface, right_interface}};
    for (const auto &interface_name : interfaces) {
      if (!allow_hardware && interface_name.rfind("vcan", 0U) != 0U) {
        throw std::runtime_error(
            "refusing to run a motor emulator on '" + interface_name +
            "'; set allow_hardware_interfaces:=true only for intentional HIL");
      }
    }
    buses_[0] = std::make_unique<motor_canfd_drive::SocketCan>(interfaces[0]);
    buses_[1] = std::make_unique<motor_canfd_drive::SocketCan>(interfaces[1]);

    for (std::size_t arm = 0; arm < 2U; ++arm) {
      wrist_[arm][0].motor_type = 0x01;
      wrist_[arm][1].motor_type = 0x01;
      wrist_[arm][2].motor_type = 0x02;
    }

    effort_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50,
        std::bind(&MotorCanFdSimNode::on_joint_states, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz),
        std::bind(&MotorCanFdSimNode::control_tick, this));
    status_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / status_rate_hz),
        std::bind(&MotorCanFdSimNode::status_tick, this));

    RCLCPP_INFO(get_logger(),
                "C++ CAN-FD motor simulator ready: left=%s right=%s; RP 4 + wrist 3 per arm",
                left_interface.c_str(), right_interface.c_str());
  }

private:
  void validate_parameters(const double control_rate_hz, const double status_rate_hz,
                           const std::vector<int64_t> &rp_ids,
                           const std::vector<int64_t> &wrist_nodes,
                           const std::vector<std::string> &joint_names) {
    if (control_rate_hz <= 0.0 || status_rate_hz <= 0.0 || command_timeout_s_ <= 0.0) {
      throw std::invalid_argument("rates and command_timeout_s must be positive");
    }
    if (rp_torque_limit_nm_ <= 0.0 || wrist_torque_constant_ <= 0.0 ||
        wrist_current_limit_a_ <= 0.0 || wrist_torque_limit_nm_ <= 0.0) {
      throw std::invalid_argument("simulation current/torque limits must be positive");
    }
    if (rp_ids.size() != 4U || wrist_nodes.size() != 3U || joint_names.size() != 14U) {
      throw std::invalid_argument("rp_ids=4, wrist_nodes=3 and joint_names=14 are required");
    }

    std::array<int64_t, 4> sorted_rp{};
    std::copy(rp_ids.begin(), rp_ids.end(), sorted_rp.begin());
    std::sort(sorted_rp.begin(), sorted_rp.end());
    if (sorted_rp.front() < 1 || sorted_rp.back() > 8 ||
        std::adjacent_find(sorted_rp.begin(), sorted_rp.end()) != sorted_rp.end()) {
      throw std::invalid_argument("RP IDs must be unique values in [1, 8]");
    }

    std::array<int64_t, 3> sorted_wrist{};
    std::copy(wrist_nodes.begin(), wrist_nodes.end(), sorted_wrist.begin());
    std::sort(sorted_wrist.begin(), sorted_wrist.end());
    if (sorted_wrist.front() < 1 || sorted_wrist.back() > 60 ||
        std::adjacent_find(sorted_wrist.begin(), sorted_wrist.end()) != sorted_wrist.end()) {
      throw std::invalid_argument("wrist nodes must be unique values in [1, 60]");
    }
  }

  std::size_t find_joint(const std::string &name) const {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
    return it == joint_names_.end()
               ? joint_names_.size()
               : static_cast<std::size_t>(std::distance(joint_names_.begin(), it));
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr message) {
    for (std::size_t message_index = 0; message_index < message->name.size(); ++message_index) {
      const auto index = find_joint(message->name[message_index]);
      if (index >= samples_.size()) {
        continue;
      }
      if (message_index < message->position.size()) {
        samples_[index].position = message->position[message_index];
      }
      if (message_index < message->velocity.size()) {
        samples_[index].velocity = message->velocity[message_index];
      }
    }
  }

  void control_tick() {
    const auto now = Clock::now();
    try {
      drain_can(now);
      calculate_efforts(now);
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "CAN-FD simulation tick failed: %s", exception.what());
      return;
    }

    std_msgs::msg::Float64MultiArray output;
    output.data.reserve(samples_.size());
    for (const auto &sample : samples_) {
      output.data.push_back(sample.effort);
    }
    effort_publisher_->publish(output);
  }

  void drain_can(const Clock::time_point now) {
    for (std::size_t arm = 0; arm < buses_.size(); ++arm) {
      motor_canfd_drive::CanFrame frame;
      while (buses_[arm]->receive(frame)) {
        handle_frame(arm, frame, now);
      }
    }
  }

  void handle_frame(const std::size_t arm, const motor_canfd_drive::CanFrame &frame,
                    const Clock::time_point now) {
    if (frame.is_extended) {
      return;
    }
    if (frame.id == protocol::kRpEnableId && frame.size >= 1U) {
      const bool enabled = frame.data[0] != 0U;
      for (auto &drive : rp_[arm]) {
        drive.enabled = enabled;
      }
      return;
    }
    if (frame.id == protocol::kRpCommandId && frame.size == 64U) {
      for (std::size_t index = 0; index < rp_[arm].size(); ++index) {
        const auto offset = static_cast<std::size_t>(rp_ids_[index] - 1U) * 8U;
        rp_[arm][index].command =
            protocol::unpack_rp_slot(frame.data.data() + offset, 8U);
        rp_[arm][index].last_command = now;
      }
      send_rp_feedback(arm);
      return;
    }

    for (std::size_t index = 0; index < wrist_nodes_.size(); ++index) {
      const auto node = wrist_nodes_[index];
      if (frame.id == protocol::wrist_control_id(node) && frame.size == 8U) {
        handle_wrist_control(arm, index,
                             protocol::unpack_wrist_control(frame.data.data(), frame.size));
        return;
      }
      if (frame.id == protocol::wrist_command_id(node) && frame.size == 12U) {
        auto &drive = wrist_[arm][index];
        drive.command = protocol::unpack_wrist_command(frame.data.data(), frame.size);
        drive.last_command = now;
        send_wrist_reply(arm, index);
        return;
      }
    }
  }

  void handle_wrist_control(const std::size_t arm, const std::size_t wrist_index,
                            const protocol::WristControl &control) {
    auto &drive = wrist_[arm][wrist_index];
    if (!control.run) {
      drive.mode = control.mode;
      if (control.set_zero) {
        drive.zero_offset = samples_[arm * 7U + 4U + wrist_index].position;
      }
    }
    drive.enabled = control.run;
    if (!control.run) {
      drive.current_a = 0.0;
    }
  }

  void calculate_efforts(const Clock::time_point now) {
    for (std::size_t arm = 0; arm < 2U; ++arm) {
      for (std::size_t index = 0; index < 4U; ++index) {
        auto &sample = samples_[arm * 7U + index];
        const auto &drive = rp_[arm][index];
        double torque = 0.0;
        if (drive.enabled &&
            std::chrono::duration<double>(now - drive.last_command).count() <=
                command_timeout_s_) {
          torque = drive.command.kp * (drive.command.position - sample.position) +
                   drive.command.kd * (drive.command.velocity - sample.velocity) +
                   drive.command.feedforward;
          torque = clamp_symmetric(torque, rp_torque_limit_nm_);
        }
        sample.effort = torque;
      }

      for (std::size_t index = 0; index < 3U; ++index) {
        auto &sample = samples_[arm * 7U + 4U + index];
        auto &drive = wrist_[arm][index];
        double current = 0.0;
        if (drive.enabled &&
            std::chrono::duration<double>(now - drive.last_command).count() <=
                command_timeout_s_) {
          const auto relative_position = sample.position - drive.zero_offset;
          if (drive.mode == protocol::WristMode::Mit) {
            current = protocol::wrist_kp_to_current_gain(drive.command.kp) *
                          (drive.command.position - relative_position) +
                      protocol::wrist_kd_to_current_gain(drive.command.kd) *
                          (drive.command.velocity - sample.velocity) +
                      drive.command.feedforward;
          } else if (drive.mode == protocol::WristMode::Position) {
            current = wrist_position_gain_ *
                          (drive.command.position - relative_position) -
                      wrist_position_damping_ * sample.velocity;
          } else if (drive.mode == protocol::WristMode::Velocity) {
            current = wrist_velocity_gain_ *
                      (drive.command.velocity - sample.velocity);
          }
        }
        drive.current_a = clamp_symmetric(current, wrist_current_limit_a_);
        sample.effort = clamp_symmetric(
            drive.current_a * wrist_torque_constant_, wrist_torque_limit_nm_);
      }
    }
  }

  void send_rp_feedback(const std::size_t arm) {
    for (std::size_t index = 0; index < rp_ids_.size(); ++index) {
      const auto payload = protocol::pack_rp_state(samples_[arm * 7U + index]);
      if (!buses_[arm]->send(rp_ids_[index], payload.data(), payload.size(), true)) {
        RCLCPP_WARN(get_logger(), "failed to send RP simulated feedback");
      }
    }
  }

  void send_wrist_reply(const std::size_t arm, const std::size_t wrist_index) {
    const auto node = wrist_nodes_[wrist_index];
    const auto sample_index = arm * 7U + 4U + wrist_index;
    protocol::JointState wire_state{samples_[sample_index].position -
                                        wrist_[arm][wrist_index].zero_offset,
                                    samples_[sample_index].velocity,
                                    wrist_[arm][wrist_index].current_a};
    const auto payload = protocol::pack_wrist_reply(node, wire_state);
    if (!buses_[arm]->send(protocol::wrist_reply_id(node), payload.data(), payload.size(),
                           true)) {
      RCLCPP_WARN(get_logger(), "failed to send wrist simulated reply");
    }
  }

  void status_tick() {
    try {
      for (std::size_t arm = 0; arm < 2U; ++arm) {
        for (std::size_t index = 0; index < wrist_nodes_.size(); ++index) {
          const auto node = wrist_nodes_[index];
          const auto sample_index = arm * 7U + 4U + index;
          const auto &drive = wrist_[arm][index];
          protocol::JointState wire_state{samples_[sample_index].position - drive.zero_offset,
                                          samples_[sample_index].velocity,
                                          drive.current_a};
          const auto payload = protocol::pack_wrist_status(
              node, wire_state, 12.0, 0U, drive.enabled ? 4U : 3U, 0U,
              drive.mode, drive.motor_type);
          if (!buses_[arm]->send(protocol::wrist_status_id(node), payload.data(),
                                 payload.size(), true)) {
            RCLCPP_WARN(get_logger(), "failed to send wrist simulated status");
          }
        }
      }
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "status publisher failed: %s", exception.what());
    }
  }

  std::array<std::unique_ptr<motor_canfd_drive::SocketCan>, 2> buses_;
  std::array<std::array<RpDriveState, 4>, 2> rp_{};
  std::array<std::array<WristDriveState, 3>, 2> wrist_{};
  std::array<JointSample, 14> samples_{};
  std::array<unsigned, 4> rp_ids_{{1U, 2U, 3U, 4U}};
  std::array<unsigned, 3> wrist_nodes_{{1U, 2U, 3U}};
  std::array<std::string, 14> joint_names_{};

  double command_timeout_s_{0.15};
  double rp_torque_limit_nm_{60.0};
  double wrist_torque_constant_{0.35};
  double wrist_current_limit_a_{11.0};
  double wrist_torque_limit_nm_{5.0};
  double wrist_position_gain_{5.0};
  double wrist_position_damping_{0.08};
  double wrist_velocity_gain_{0.45};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace motor_canfd_sim

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motor_canfd_sim::MotorCanFdSimNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_canfd_sim"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
