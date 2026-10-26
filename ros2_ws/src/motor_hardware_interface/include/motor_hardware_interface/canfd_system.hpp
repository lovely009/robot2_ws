#pragma once

#include <chrono>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor_drive/motor_driver.hpp"
#include "motor_hardware_interface/motor_config.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_transmission/model.hpp"

namespace motor_hardware_interface {

// ROS 2 ros2_control 适配层：MotorDriver 负责执行器 I/O，robot_transmission
// 负责组级坐标换算。Model 由本类单线程持有。
class CanFdSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanFdSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

private:
  using TimePoint = std::chrono::steady_clock::time_point;

  bool load_config(const std::string &path, std::string &error);
  bool load_transmission_model(const std::string &path, std::string &error);
  bool apply_bus_overrides(std::string &error);
  bool build_joint_mapping(std::string &error);
  bool send_position_commands(bool zero_gains = false);
  bool receive_feedback(std::vector<bool> *observed, std::string &error);
  bool prepare_initial_commands(std::string &error);
  bool command_groups_complete() const;
  void close_driver() noexcept;

  MotorSystemConfig config_;
  std::unique_ptr<motor_drive::MotorDriver> driver_;
  std::unique_ptr<robot_transmission::Model> model_;

  // 执行器名键缓存；时间由收到的帧统一按 steady_clock 更新。
  robot_transmission::AxisMap latest_actuator_feedback_;
  std::map<std::string, std::chrono::steady_clock::time_point> motor_last_feedback_;

  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::vector<std::size_t> motor_to_driver_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;
  std::vector<TimePoint> last_feedback_;
  bool active_{false};
};

} // namespace motor_hardware_interface
