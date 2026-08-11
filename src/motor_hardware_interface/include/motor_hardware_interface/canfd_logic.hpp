#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace motor_hardware_interface {

// 纯逻辑函数：不依赖 ROS、ros2_control 或 SocketCAN，可直接在 gtest 中构造输入。
// 从 CanFdSystem 中抽出，保证"激活时序、反馈超时、关节映射"这些安全关键判断
// 有单元测试兜底，同时让适配层只保留 I/O 与 ROS 编排。

// 把电机清单按名称映射到 ros2_control 关节下标，并校验 1:1 覆盖。
// joint_names 是 ros2_control 暴露的关节名（按下标排列）；motor_names 与
// motor_joint_names 按电机下标排列（两者长度必须相同）。
// 校验：关节名无重复、每台电机的目标关节存在、且每个关节恰好被映射一次。
// 成功时 motor_to_joint[motor] 为关节下标；失败时返回 false 并在 error 中说明原因。
bool map_motors_to_joints(const std::vector<std::string> &joint_names,
                          const std::vector<std::string> &motor_names,
                          const std::vector<std::string> &motor_joint_names,
                          std::vector<std::size_t> &motor_to_joint,
                          std::string &error);

// 返回第一个反馈超时（now - last > timeout_s）的关节下标；
// 全部新鲜时返回 last_feedback.size()。使用 > 严格比较，等于超时阈值不算超时。
std::size_t first_stale_feedback(
    const std::vector<std::chrono::steady_clock::time_point> &last_feedback,
    const std::chrono::steady_clock::time_point &now, double timeout_s);

// 激活轮询是否继续：还有关节未观察到反馈，且当前时间未过 deadline。
bool should_continue_waiting(const std::vector<bool> &observed,
                             const std::chrono::steady_clock::time_point &now,
                             const std::chrono::steady_clock::time_point &deadline);

// 把已观察到且位置有限的关节反馈复制为初始位置目标（避免一使能就追错误目标）。
// 任一关节未观察到或位置非有限时返回 false，error 中给出首个问题关节名。
bool take_observed_positions_as_initial_targets(
    const std::vector<std::string> &joint_names,
    const std::vector<bool> &observed, const std::vector<double> &positions,
    std::vector<double> &commands, std::string &error);

} // namespace motor_hardware_interface
