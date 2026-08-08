#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor_canfd_drive/motor_canfd_driver.hpp"
#include "motor_canfd_drive/motor_protocol.hpp"
#include "motor_canfd_drive/protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"

namespace motor_canfd_sim {

namespace drive = motor_canfd_drive;
namespace protocol = motor_canfd_drive::protocol;
using Clock = std::chrono::steady_clock;

namespace {

drive::MotorParameterMap parse_driver_parameters(const YAML::Node &node,
                                                 const std::string &context) {
  drive::MotorParameterMap parameters;
  if (!node) {
    return parameters;
  }
  if (!node.IsMap()) {
    throw std::runtime_error(context + " parameters must be a YAML map");
  }
  for (const auto &entry : node) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error(context +
                               " parameters must contain scalar key/value pairs");
    }
    parameters[entry.first.Scalar()] = entry.second.Scalar();
  }
  return parameters;
}

void merge_parameters(drive::MotorParameterMap &target,
                      const drive::MotorParameterMap &overrides) {
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

drive::DriverConfig load_driver_config_yaml(const std::string &path) {
  if (path.empty()) {
    throw std::invalid_argument("motor_config_file is required");
  }
  const auto document = YAML::LoadFile(path);
  const auto root = document["motor_hardware"];
  if (!root || !root.IsMap()) {
    throw std::runtime_error("YAML root 'motor_hardware' is required");
  }

  drive::DriverConfig config;
  const auto buses = root["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("motor_hardware.buses must be a non-empty map");
  }
  for (const auto &entry : buses) {
    config.buses.push_back({entry.first.as<std::string>(),
                            entry.second["interface"].as<std::string>()});
  }

  std::unordered_map<std::string, drive::MotorParameterMap> type_parameters;
  const auto motor_types = root["motor_types"];
  if (motor_types) {
    if (!motor_types.IsMap()) {
      throw std::runtime_error("motor_hardware.motor_types must be a map");
    }
    for (const auto &entry : motor_types) {
      const auto type = entry.first.as<std::string>();
      type_parameters[type] = parse_driver_parameters(
          entry.second["parameters"], "motor type '" + type + "'");
    }
  }

  const auto motors = root["motors"];
  if (!motors || !motors.IsMap() || motors.size() == 0U) {
    throw std::runtime_error("motor_hardware.motors must be a non-empty map");
  }
  for (const auto &entry : motors) {
    drive::MotorConfig motor;
    motor.name = entry.first.as<std::string>();
    const auto node = entry.second;
    motor.joint_name = node["joint"].as<std::string>();
    motor.bus = node["bus"].as<std::string>();
    motor.type = node["type"].as<std::string>();
    motor.id = node["id"].as<unsigned>();
    const auto defaults = type_parameters.find(motor.type);
    if (defaults != type_parameters.end()) {
      motor.parameters = defaults->second;
    }
    merge_parameters(motor.parameters, parse_driver_parameters(
                                           node["parameters"],
                                           "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

double clamp_symmetric(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

double motor_parameter_as_double(const drive::MotorConfig &motor,
                                 const std::string &key) {
  const auto it = motor.parameters.find(key);
  if (it == motor.parameters.end()) {
    throw std::runtime_error("motor '" + motor.name + "' is missing parameter '" + key + "'");
  }
  std::size_t parsed = 0U;
  const double value = std::stod(it->second, &parsed);
  if (parsed != it->second.size() || !std::isfinite(value)) {
    throw std::runtime_error("motor '" + motor.name + "' parameter '" + key + "' is invalid");
  }
  return value;
}

} // namespace

struct JointSample {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct SimMotorState {
  bool enabled{false};
  protocol::JointCommand command{};
  Clock::time_point last_command{};
  protocol::WristMode mode{protocol::WristMode::Mit};
  double zero_offset{0.0};
  double current_a{0.0};
};

struct SimBus {
  drive::BusConfig config;
  std::unique_ptr<drive::SocketCan> socket;
  std::vector<std::size_t> motor_indices;
};

class MotorCanFdSimNode final : public rclcpp::Node {
public:
  MotorCanFdSimNode() : Node("motor_canfd_sim") {
    const auto config_file = declare_parameter<std::string>("motor_config_file", "");
    const auto allow_hardware =
        declare_parameter<bool>("allow_hardware_interfaces", false);
    const auto control_rate_hz = declare_parameter<double>("control_rate_hz", 500.0);
    const auto status_rate_hz = declare_parameter<double>("status_rate_hz", 50.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.15);
    rp_torque_limit_nm_ = declare_parameter<double>("rp_torque_limit_nm", 60.0);
    motor_2530_current_limit_a_ =
        declare_parameter<double>("motor_2530_current_limit_a", 11.0);
    motor_2530_torque_limit_nm_ =
        declare_parameter<double>("motor_2530_torque_limit_nm", 5.0);
    motor_2530_position_gain_ =
        declare_parameter<double>("motor_2530_position_mode_gain_a_per_rad", 5.0);
    motor_2530_position_damping_ =
        declare_parameter<double>("motor_2530_position_mode_damping_a_per_rad_s", 0.08);
    motor_2530_velocity_gain_ =
        declare_parameter<double>("motor_2530_velocity_mode_gain_a_per_rad_s", 0.45);

    validate_scalar_parameters(control_rate_hz, status_rate_hz);
    driver_config_ = load_driver_config_yaml(config_file);
    apply_bus_parameter_overrides();

    // Reuse the production driver's validation (registered types, ID ranges,
    // duplicate mappings, protocol-specific required parameters) without opening CAN.
    const drive::MotorCanfdDriver config_validator(driver_config_);
    static_cast<void>(config_validator);
    build_runtime(allow_hardware);

    samples_.resize(driver_config_.motors.size());
    motor_states_.resize(driver_config_.motors.size());
    for (std::size_t index = 0; index < driver_config_.motors.size(); ++index) {
      const auto &motor = driver_config_.motors[index];
      if (motor.type != drive::kRpMotorType && motor.type != drive::kMotor2530Type) {
        throw std::runtime_error("C++ simulator has no plant model for motor type '" +
                                 motor.type + "'");
      }
      if (!joint_to_motor_.emplace(motor.joint_name, index).second) {
        throw std::runtime_error("duplicate simulated joint '" + motor.joint_name + "'");
      }
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
                "generic C++ CAN-FD simulator ready: %zu buses, %zu motors",
                buses_.size(), driver_config_.motors.size());
  }

private:
  void validate_scalar_parameters(const double control_rate_hz,
                                  const double status_rate_hz) const {
    if (control_rate_hz <= 0.0 || status_rate_hz <= 0.0 || command_timeout_s_ <= 0.0 ||
        rp_torque_limit_nm_ <= 0.0 || motor_2530_current_limit_a_ <= 0.0 ||
        motor_2530_torque_limit_nm_ <= 0.0) {
      throw std::invalid_argument("simulation rates, timeout and limits must be positive");
    }
  }

  void apply_bus_parameter_overrides() {
    for (auto &bus : driver_config_.buses) {
      const auto parameter_name = "bus_override." + bus.name;
      const auto interface_name =
          declare_parameter<std::string>(parameter_name, bus.interface_name);
      if (interface_name.empty()) {
        throw std::invalid_argument("parameter '" + parameter_name + "' cannot be empty");
      }
      bus.interface_name = interface_name;
    }
  }

  void build_runtime(const bool allow_hardware) {
    std::unordered_map<std::string, std::size_t> bus_index;
    buses_.reserve(driver_config_.buses.size());
    for (const auto &config : driver_config_.buses) {
      if (!allow_hardware && config.interface_name.rfind("vcan", 0U) != 0U) {
        throw std::runtime_error(
            "refusing to run a motor emulator on '" + config.interface_name +
            "'; set allow_hardware_interfaces:=true only for intentional HIL");
      }
      SimBus bus;
      bus.config = config;
      bus.socket = std::make_unique<drive::SocketCan>(config.interface_name);
      bus_index.emplace(config.name, buses_.size());
      buses_.push_back(std::move(bus));
    }
    motor_to_bus_.resize(driver_config_.motors.size());
    for (std::size_t motor = 0; motor < driver_config_.motors.size(); ++motor) {
      const auto bus = bus_index.at(driver_config_.motors[motor].bus);
      motor_to_bus_[motor] = bus;
      buses_[bus].motor_indices.push_back(motor);
    }
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr message) {
    for (std::size_t message_index = 0; message_index < message->name.size(); ++message_index) {
      const auto it = joint_to_motor_.find(message->name[message_index]);
      if (it == joint_to_motor_.end()) {
        continue;
      }
      auto &sample = samples_[it->second];
      if (message_index < message->position.size()) {
        sample.position = message->position[message_index];
      }
      if (message_index < message->velocity.size()) {
        sample.velocity = message->velocity[message_index];
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

    // The current upper-body effort controller uses the same joint order as the
    // motor map in motor_canfd.yaml. If the robot model changes, update both maps.
    std_msgs::msg::Float64MultiArray output;
    output.data.reserve(samples_.size());
    for (const auto &sample : samples_) {
      output.data.push_back(sample.effort);
    }
    effort_publisher_->publish(output);
  }

  void drain_can(const Clock::time_point now) {
    for (std::size_t bus = 0; bus < buses_.size(); ++bus) {
      drive::CanFrame frame;
      while (buses_[bus].socket->receive(frame)) {
        handle_frame(bus, frame, now);
      }
    }
  }

  void handle_frame(const std::size_t bus, const drive::CanFrame &frame,
                    const Clock::time_point now) {
    if (frame.is_extended) {
      return;
    }
    if (frame.id == protocol::kRpEnableId && frame.size >= 1U) {
      const bool enabled = frame.data[0] != 0U;
      for (const auto motor : buses_[bus].motor_indices) {
        if (driver_config_.motors[motor].type == drive::kRpMotorType) {
          motor_states_[motor].enabled = enabled;
        }
      }
      return;
    }
    if (frame.id == protocol::kRpCommandId && frame.size == 64U) {
      bool has_rp = false;
      for (const auto motor : buses_[bus].motor_indices) {
        const auto &config = driver_config_.motors[motor];
        if (config.type != drive::kRpMotorType) {
          continue;
        }
        has_rp = true;
        const auto offset = static_cast<std::size_t>(config.id - 1U) * 8U;
        motor_states_[motor].command =
            protocol::unpack_rp_slot(frame.data.data() + offset, 8U);
        motor_states_[motor].last_command = now;
      }
      if (has_rp) {
        send_rp_feedback(bus);
      }
      return;
    }

    for (const auto motor : buses_[bus].motor_indices) {
      const auto &config = driver_config_.motors[motor];
      if (config.type != drive::kMotor2530Type) {
        continue;
      }
      if (frame.id == protocol::wrist_control_id(config.id) && frame.size == 8U) {
        handle_motor_2530_control(
            motor, protocol::unpack_wrist_control(frame.data.data(), frame.size));
        return;
      }
      if (frame.id == protocol::wrist_command_id(config.id) && frame.size == 12U) {
        auto &state = motor_states_[motor];
        state.command = protocol::unpack_wrist_command(frame.data.data(), frame.size);
        state.last_command = now;
        send_motor_2530_reply(motor);
        return;
      }
    }
  }

  void handle_motor_2530_control(const std::size_t motor,
                                 const protocol::WristControl &control) {
    auto &state = motor_states_[motor];
    if (!control.run) {
      state.mode = control.mode;
      if (control.set_zero) {
        state.zero_offset = samples_[motor].position;
      }
    }
    state.enabled = control.run;
    if (!control.run) {
      state.current_a = 0.0;
    }
  }

  void calculate_efforts(const Clock::time_point now) {
    for (std::size_t motor = 0; motor < driver_config_.motors.size(); ++motor) {
      const auto &config = driver_config_.motors[motor];
      auto &state = motor_states_[motor];
      auto &sample = samples_[motor];
      const bool fresh = state.enabled &&
                         std::chrono::duration<double>(now - state.last_command).count() <=
                             command_timeout_s_;
      if (config.type == drive::kRpMotorType) {
        double torque = 0.0;
        if (fresh) {
          torque = state.command.kp * (state.command.position - sample.position) +
                   state.command.kd * (state.command.velocity - sample.velocity) +
                   state.command.feedforward;
        }
        sample.effort = clamp_symmetric(torque, rp_torque_limit_nm_);
        continue;
      }

      double current = 0.0;
      if (fresh) {
        const auto relative_position = sample.position - state.zero_offset;
        if (state.mode == protocol::WristMode::Mit) {
          current = protocol::wrist_kp_to_current_gain(state.command.kp) *
                        (state.command.position - relative_position) +
                    protocol::wrist_kd_to_current_gain(state.command.kd) *
                        (state.command.velocity - sample.velocity) +
                    state.command.feedforward;
        } else if (state.mode == protocol::WristMode::Position) {
          current = motor_2530_position_gain_ *
                        (state.command.position - relative_position) -
                    motor_2530_position_damping_ * sample.velocity;
        } else if (state.mode == protocol::WristMode::Velocity) {
          current = motor_2530_velocity_gain_ *
                    (state.command.velocity - sample.velocity);
        }
      }
      state.current_a = clamp_symmetric(current, motor_2530_current_limit_a_);
      const auto torque_constant =
          motor_parameter_as_double(config, "torque_constant_nm_per_amp");
      sample.effort = clamp_symmetric(state.current_a * torque_constant,
                                      motor_2530_torque_limit_nm_);
    }
  }

  void send_rp_feedback(const std::size_t bus) {
    for (const auto motor : buses_[bus].motor_indices) {
      const auto &config = driver_config_.motors[motor];
      if (config.type != drive::kRpMotorType) {
        continue;
      }
      const auto payload = protocol::pack_rp_state(
          {samples_[motor].position, samples_[motor].velocity, samples_[motor].effort});
      if (!buses_[bus].socket->send(config.id, payload.data(), payload.size(), true)) {
        RCLCPP_WARN(get_logger(), "failed to send RP simulated feedback for %s",
                    config.name.c_str());
      }
    }
  }

  void send_motor_2530_reply(const std::size_t motor) {
    const auto &config = driver_config_.motors[motor];
    const auto &state = motor_states_[motor];
    const protocol::JointState wire_state{
        samples_[motor].position - state.zero_offset, samples_[motor].velocity,
        state.current_a};
    const auto payload = protocol::pack_wrist_reply(config.id, wire_state);
    auto &socket = *buses_[motor_to_bus_[motor]].socket;
    if (!socket.send(protocol::wrist_reply_id(config.id), payload.data(), payload.size(), true)) {
      RCLCPP_WARN(get_logger(), "failed to send 25/30 mm simulated reply for %s",
                  config.name.c_str());
    }
  }

  void status_tick() {
    try {
      for (std::size_t motor = 0; motor < driver_config_.motors.size(); ++motor) {
        const auto &config = driver_config_.motors[motor];
        if (config.type != drive::kMotor2530Type) {
          continue;
        }
        const auto &state = motor_states_[motor];
        const protocol::JointState wire_state{
            samples_[motor].position - state.zero_offset, samples_[motor].velocity,
            state.current_a};
        const auto payload = protocol::pack_wrist_status(
            config.id, wire_state, 12.0, 0U, state.enabled ? 4U : 3U, 0U,
            state.mode, 0x01U);
        auto &socket = *buses_[motor_to_bus_[motor]].socket;
        if (!socket.send(protocol::wrist_status_id(config.id), payload.data(), payload.size(),
                         true)) {
          RCLCPP_WARN(get_logger(), "failed to send 25/30 mm simulated status for %s",
                      config.name.c_str());
        }
      }
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "status publisher failed: %s", exception.what());
    }
  }

  drive::DriverConfig driver_config_;
  std::vector<SimBus> buses_;
  std::vector<std::size_t> motor_to_bus_;
  std::unordered_map<std::string, std::size_t> joint_to_motor_;
  std::vector<SimMotorState> motor_states_;
  std::vector<JointSample> samples_;

  double command_timeout_s_{0.15};
  double rp_torque_limit_nm_{60.0};
  double motor_2530_current_limit_a_{11.0};
  double motor_2530_torque_limit_nm_{5.0};
  double motor_2530_position_gain_{5.0};
  double motor_2530_position_damping_{0.08};
  double motor_2530_velocity_gain_{0.45};

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
