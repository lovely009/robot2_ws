#pragma once

#include <string>

#include "motor_canfd_drive/motor_canfd_driver.hpp"

namespace motor_hardware_interface {

// 读取电机拓扑配置（motor_canfd.yaml）并解析为纯 C++ DriverConfig。
// 本函数是 YAML 世界与底层驱动之间的唯一边界：motor_canfd_drive
// 不接触 YAML，只消费本函数产出的 DriverConfig。
// 空路径、文件不可读或内容不符合 schema 时抛 std::runtime_error。
motor_canfd_drive::DriverConfig load_driver_config(const std::string &path);

} // namespace motor_hardware_interface
