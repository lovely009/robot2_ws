#pragma once

#include <string>
#include <vector>

#include "motor_drive/motor_driver.hpp"

namespace motor_hardware_interface {

// 适配层从拓扑文件（motor_driver.yaml，唯一入口）连同其引用的型号库
// （motor.yaml）一次性解析出的完整运行配置：底层 DriverConfig 之外，
// 还有 ROS 侧专属的反馈时序与每电机命令增益。
struct MotorSystemConfig {
  struct CommandGains {
    double kp{0.0};
    double kd{0.0};
  };

  motor_drive::DriverConfig driver;
  std::vector<CommandGains> motor_commands;
  double feedback_timeout_s{0.1};
  double initial_feedback_wait_s{0.1};
};

// 读取电机拓扑配置并解析为完整运行配置（单次 YAML 解析，YAML 世界的唯一边界）。
// 空路径、文件不可读或内容不符合 schema 时抛 std::runtime_error。
MotorSystemConfig load_system_config(const std::string &path);

// 薄封装：只返回底层驱动需要的 DriverConfig 部分，内部复用同一次解析。
motor_drive::DriverConfig load_driver_config(const std::string &path);

} // namespace motor_hardware_interface
