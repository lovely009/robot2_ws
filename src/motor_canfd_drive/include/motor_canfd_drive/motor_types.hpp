#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace motor_canfd_drive {

// 这里只描述“物理 CAN 拓扑”，刻意不出现 ROS 2 的 node/topic/controller，
// 也不出现 left_arm/right_arm 等机器人结构概念。这样底层驱动可以脱离 ROS 2 独立复用。
using MotorParameterMap = std::unordered_map<std::string, std::string>;

// 一个逻辑总线对应一个 Linux SocketCAN 接口，例如 left_arm -> can0。
struct BusConfig {
  std::string name;
  std::string interface_name;
};

// 一台电机只描述“它是谁、挂在哪、用什么协议”。
// parameters 是协议私有扩展参数，例如 25/30 mm 电机的 Nm/A 扭矩常数。
struct MotorConfig {
  std::string name;
  std::string joint_name;
  std::string bus;
  std::string type;
  unsigned id{0U};
  MotorParameterMap parameters;
};

// 总线数量和电机数量都由 vector 动态决定；这里没有固定 7/14 关节假设。
struct DriverConfig {
  std::vector<BusConfig> buses;
  std::vector<MotorConfig> motors;
};

} // namespace motor_canfd_drive
