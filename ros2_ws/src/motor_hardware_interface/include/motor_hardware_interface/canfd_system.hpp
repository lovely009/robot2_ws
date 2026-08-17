#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor_drive/motor_driver.hpp"
#include "motor_hardware_interface/motor_config.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motor_hardware_interface {

// ROS 2 ros2_control 适配层：把 controller_manager 的 joint 接口翻译成
// MotorDriver 的通用命令/状态。这个类知道 ROS joint，但不处理私有协议字节。
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
  bool load_config(const std::string &path, std::string &error);
  bool apply_bus_overrides(std::string &error);
  bool build_joint_mapping(std::string &error);
  bool send_position_commands(bool zero_gains = false);
  void receive_feedback(std::vector<bool> *observed = nullptr);
  void close_driver() noexcept;

  MotorSystemConfig config_;
  std::unique_ptr<motor_drive::MotorDriver> driver_;
  // motor_to_joint_[m] = ros2_control joint 下标。映射按 joint_name 建立，
  // 不依赖“第几个电机就是第几个关节”的固定数组顺序。
  std::vector<std::size_t> motor_to_joint_;
  std::vector<motor_drive::protocol::JointState> motor_states_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;
  std::vector<std::chrono::steady_clock::time_point> last_feedback_;
  bool active_{false};
};

} // namespace motor_hardware_interface
