#pragma once

#include <string>
#include <vector>

#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::core {

struct TimingConfig {
  unsigned control_hz{100U};
  unsigned vaxor_feedback_hz{50U};
  unsigned command_timeout_ms{150U};
};

struct BusConfig {
  std::string name;
};

struct TransmissionConfig {
  double gear_ratio{1.0};
  double efficiency{1.0};
  double direction{1.0};
  double zero_offset_rad{0.0};
};

struct JointLimits {
  double hard_lower_rad{0.0};
  double hard_upper_rad{0.0};
  double command_lower_rad{0.0};
  double command_upper_rad{0.0};
  double velocity_rad_s{0.0};
  double effort_nm{0.0};
};

struct PlantConfig {
  double kp{0.0};
  double kd{0.0};
  double effort_limit_nm{0.0};
  double torque_constant_nm_per_a{0.0};
  double current_limit_a{0.0};
};

struct MotorConfig {
  std::string joint;
  std::string bus;
  std::string protocol;
  unsigned id{0U};
  // CERB MIT τ_ff 的 12 位量化满量程（N·m），逐型号（R48 11.0 / R58 45.0）；
  // 与 plant.effort_limit_nm（plant 的力矩饱和）既不同义也不等值。缺省值取
  // kCerbDefaultMitTorqueLimitNm（R48），只保证不写该字段的旧配置行为不变。
  double mit_torque_limit_nm{
      ::motor_sim::protocol::kCerbDefaultMitTorqueLimitNm};
  TransmissionConfig transmission;
  JointLimits limits;
  PlantConfig plant;
};

struct SystemConfig {
  TimingConfig timing;
  std::vector<BusConfig> buses;
  std::vector<MotorConfig> motors;
};

struct RequiredMotorAddress {
  std::string joint;
  std::string bus;
  std::string protocol;
  unsigned id{0U};
};

struct TopologyContract {
  std::vector<std::string> buses;
  std::vector<RequiredMotorAddress> motors;
};

// 从 YAML 加载配置，并按 caller 提供的拓扑契约逐项校验 buses/motors 的顺序与地址。
// 机器人集成层使用此重载，保证 YAML 与 URDF/关节清单完全一致。
SystemConfig load_system_config(const std::string &path,
                                const TopologyContract &contract);

// 从 YAML 直接加载配置，不依赖外部拓扑契约。buses/motors 顺序完全由 YAML 决定。
// 通用独立入口使用此重载；调用方显式传入 config_file 与 bus→endpoint 映射，
// 电机数量、协议与 ID 以配置文件为准，不做任何伪造的默认拓扑。
SystemConfig load_system_config(const std::string &path);

}  // namespace motor_sim::core
