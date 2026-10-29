#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "can_bridge_msgs/msg/motor_command_array.hpp"
#include "can_bridge_msgs/msg/motor_control_state_array.hpp"
#include "can_bridge_msgs/msg/motor_motion_state_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_transmission/model.hpp"
#include "std_msgs/msg/empty.hpp"

namespace motor_hardware_interface {

class MicroRosSystem final : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MicroRosSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) override;
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
  using Motion = can_bridge_msgs::msg::MotorMotionStateArray;
  using Control = can_bridge_msgs::msg::MotorControlStateArray;
  using Command = can_bridge_msgs::msg::MotorCommandArray;

  struct Limit {
    std::optional<double> min;
    std::optional<double> max;
  };

  bool load_model(const std::string &path, std::string &error);
  bool build_interface_mapping(std::string &error);
  bool parse_limit(const std::string &value, std::optional<double> &bound, std::string &error) const;
  void on_motion(const Motion::SharedPtr message);
  void on_control(const Control::SharedPtr message);
  bool motion_current(const TimePoint &now, bool latch, std::string &error);
  bool control_current(const TimePoint &now, bool latch, std::string &error);
  bool controls_armed(const TimePoint &now, std::string &error) const;
  bool update_states(const TimePoint &now, bool latch, std::string &error);
  bool command_valid(std::string &error) const;
  bool transitioned_claims(const std::vector<std::string> &start_interfaces,
                           const std::vector<std::string> &stop_interfaces,
                           std::set<std::string> &next,
                           std::set<std::string> &started) const;
  void close_writer();
  bool fail(std::string error);
  bool publish(bool commanded, std::string &error);

  std::unique_ptr<robot_transmission::Model> model_;
  std::map<uint16_t, std::string> id_to_actuator_;
  std::set<uint16_t> expected_ids_;
  std::set<std::string> required_command_interfaces_;
  std::set<std::string> effort_command_interfaces_;
  std::set<std::string> claimed_command_interfaces_;
  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::vector<Limit> position_limits_;
  std::vector<Limit> velocity_limits_;
  std::vector<Limit> effort_limits_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::vector<double> command_effort_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Subscription<Motion>::SharedPtr motion_sub_;
  rclcpp::Subscription<Control>::SharedPtr control_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  std::optional<Motion> latest_motion_;
  std::optional<Control> latest_control_;
  TimePoint motion_received_{};
  TimePoint control_received_{};
  uint64_t motion_source_delay_ms_{0};
  bool motion_valid_{false};
  bool control_valid_{false};
  uint64_t control_session_{0};
  uint64_t session_time_floor_ns_{0};
  uint32_t motion_seq_{0};
  uint32_t control_seq_{0};
  uint32_t command_seq_{0};
  bool active_{false};
  bool armed_{false};
  bool writer_enabled_{false};
  bool faulted_{false};
};

} // namespace motor_hardware_interface
