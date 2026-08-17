#include "motor_hardware_interface/canfd_logic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace motor_hardware_interface {

bool map_motors_to_joints(const std::vector<std::string> &joint_names,
                          const std::vector<std::string> &motor_names,
                          const std::vector<std::string> &motor_joint_names,
                          std::vector<std::size_t> &motor_to_joint,
                          std::string &error) {
  if (motor_names.size() != motor_joint_names.size()) {
    error = "motor name/joint name lists have different sizes";
    return false;
  }
  if (joint_names.size() != motor_names.size()) {
    error = "ros2_control exposes " + std::to_string(joint_names.size()) +
            " joints but YAML maps " + std::to_string(motor_names.size()) +
            " motors";
    return false;
  }

  // 关节名必须唯一，才能建立 1:1 映射。
  std::unordered_map<std::string, std::size_t> joint_index;
  joint_index.reserve(joint_names.size());
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    if (!joint_index.emplace(joint_names[index], index).second) {
      error = "duplicate ros2_control joint '" + joint_names[index] + "'";
      return false;
    }
  }

  // motor_to_joint 是运行期唯一需要的快速映射；每个 joint 必须且只能被映射一次。
  motor_to_joint.assign(motor_names.size(), std::numeric_limits<std::size_t>::max());
  std::vector<bool> mapped(joint_names.size(), false);
  for (std::size_t motor = 0; motor < motor_names.size(); ++motor) {
    const auto it = joint_index.find(motor_joint_names[motor]);
    if (it == joint_index.end()) {
      error = "motor '" + motor_names[motor] + "' maps unknown ros2_control joint '" +
              motor_joint_names[motor] + "'";
      return false;
    }
    motor_to_joint[motor] = it->second;
    mapped[it->second] = true;
  }
  if (!std::all_of(mapped.begin(), mapped.end(), [](const bool value) { return value; })) {
    error = "every ros2_control joint must have exactly one motor mapping";
    return false;
  }
  return true;
}

std::size_t first_stale_feedback(
    const std::vector<std::chrono::steady_clock::time_point> &last_feedback,
    const std::chrono::steady_clock::time_point &now, double timeout_s) {
  for (std::size_t joint = 0; joint < last_feedback.size(); ++joint) {
    if (std::chrono::duration<double>(now - last_feedback[joint]).count() > timeout_s) {
      return joint;
    }
  }
  return last_feedback.size();
}

bool should_continue_waiting(const std::vector<bool> &observed,
                             const std::chrono::steady_clock::time_point &now,
                             const std::chrono::steady_clock::time_point &deadline) {
  const bool all_observed =
      std::all_of(observed.begin(), observed.end(), [](const bool seen) { return seen; });
  return !all_observed && now < deadline;
}

bool all_feedback_received(const std::vector<bool> &observed) {
  return std::all_of(observed.begin(), observed.end(), [](const bool seen) { return seen; });
}

std::vector<std::string> missing_joint_names(
    const std::vector<std::string> &joint_names, const std::vector<bool> &observed) {
  std::vector<std::string> missing;
  const auto count = std::min(joint_names.size(), observed.size());
  missing.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    if (!observed[index]) {
      missing.push_back(joint_names[index]);
    }
  }
  return missing;
}

bool take_observed_positions_as_initial_targets(
    const std::vector<std::string> &joint_names,
    const std::vector<bool> &observed, const std::vector<double> &positions,
    std::vector<double> &commands, std::string &error) {
  if (joint_names.size() != observed.size() || joint_names.size() != positions.size() ||
      joint_names.size() != commands.size()) {
    error = "joint name/observed/position/command lists have different sizes";
    return false;
  }
  for (std::size_t joint = 0; joint < joint_names.size(); ++joint) {
    if (!observed[joint] || !std::isfinite(positions[joint])) {
      error = "no initial feedback from joint '" + joint_names[joint] + "'";
      return false;
    }
    commands[joint] = positions[joint];
  }
  return true;
}

} // namespace motor_hardware_interface
