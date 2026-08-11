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

#include "motor_canfd_drive/motor_canfd_driver.hpp"
#include "motor_canfd_drive/motor_protocol.hpp"
#include "motor_canfd_drive/socketcan.hpp"
#include "motor_hardware_interface/motor_config.hpp"
#include "motor_canfd_sim/plant_model.hpp"
#include "motor_canfd_sim/sim_motor_handler.hpp"

namespace motor_canfd_sim {

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
    plant_ = std::make_unique<MotorPlantModel>(
        command_timeout_s_, rp_torque_limit_nm_, motor_2530_current_limit_a_,
        motor_2530_torque_limit_nm_, motor_2530_position_gain_,
        motor_2530_position_damping_, motor_2530_velocity_gain_);
    // 设备清单由调用方决定：本节点只接收配置路径，解析复用硬件接口的公开 API。
    driver_config_ = motor_hardware_interface::load_driver_config(config_file);
    apply_bus_parameter_overrides();

    // 复用真机 MotorCanfdDriver 的配置校验（协议注册、ID 范围、重复映射、必需参数）。
    // 这里只构造对象而不 open()，因此不会因为“检查配置”就占用任何 CAN 接口。
    const drive::MotorCanfdDriver config_validator(driver_config_);
    static_cast<void>(config_validator);
    build_runtime(allow_hardware);

    samples_.resize(driver_config_.motors.size());
    motor_states_.resize(driver_config_.motors.size());
    const auto handler_registry = SimMotorHandlerRegistry::with_builtin_handlers();
    for (std::size_t index = 0; index < driver_config_.motors.size(); ++index) {
      const auto &motor = driver_config_.motors[index];
      if (!handler_registry.contains(motor.type)) {
        throw std::runtime_error("C++ simulator has no motor handler for type '" +
                                 motor.type + "'");
      }
      if (!joint_to_motor_.emplace(motor.joint_name, index).second) {
        throw std::runtime_error("duplicate simulated joint '" + motor.joint_name + "'");
      }
      if (handlers_.count(motor.type) == 0U) {
        handlers_.emplace(motor.type, handler_registry.create(motor.type));
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
    // 同一份 YAML 真机可以写 can0/can1；仿真 launch 再通过 bus_override.*
    // 动态改为 vcan0/vcan1，不复制电机拓扑配置。
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
      // 安全保护：模拟器会主动“冒充电机”回复 CAN 帧，默认绝不允许接真实 can*。
      // 只有明确的 HIL 测试才应显式打开 allow_hardware_interfaces。
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
    // /joint_states 的排列顺序并不保证与 YAML motors 一致，所以按 joint_name 查找。
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
      for (std::size_t motor = 0; motor < driver_config_.motors.size(); ++motor) {
        samples_[motor].effort = plant_->compute_effort(
            driver_config_.motors[motor], motor_states_[motor], samples_[motor], now);
      }
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_logger(), "CAN-FD simulation tick failed: %s", exception.what());
      return;
    }

    // 当前半身机器人 effort_controller 的数组顺序与 motor_canfd.yaml 的 motors 顺序一致。
    // 如果以后修改 Gazebo controller joint 列表，必须同步检查这里的输出顺序。
    std_msgs::msg::Float64MultiArray output;
    output.data.reserve(samples_.size());
    for (const auto &sample : samples_) {
      output.data.push_back(sample.effort);
    }
    effort_publisher_->publish(output);
  }

  void drain_can(const Clock::time_point now) {
    // 每个 SocketCAN 都是非阻塞的：每个控制周期把当前积压帧排空后立即进入动力学计算。
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
    const SimHandlerContext context{driver_config_, buses_, motor_to_bus_,
                                    motor_states_, samples_, *plant_, get_logger()};
    for (const auto &entry : handlers_) {
      if (entry.second->handle_frame(context, bus, frame, now)) {
        return;
      }
    }
  }

  void status_tick() {
    const SimHandlerContext context{driver_config_, buses_, motor_to_bus_,
                                    motor_states_, samples_, *plant_, get_logger()};
    for (const auto &entry : handlers_) {
      entry.second->status_tick(context);
    }
  }

  drive::DriverConfig driver_config_;
  std::vector<SimBus> buses_;
  std::vector<std::size_t> motor_to_bus_;
  std::unordered_map<std::string, std::size_t> joint_to_motor_;
  std::unordered_map<std::string, std::unique_ptr<SimMotorHandler>> handlers_;
  std::vector<SimMotorState> motor_states_;
  std::vector<JointSample> samples_;
  std::unique_ptr<MotorPlantModel> plant_;

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
