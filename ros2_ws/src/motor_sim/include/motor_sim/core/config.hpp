#pragma once

#include <string>
#include <vector>

#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::core {

struct TimingConfig {
  unsigned control_hz{100U};
  unsigned vaxor_feedback_hz{50U};
  unsigned command_timeout_ms{150U};
  // 逐台反馈周期（ms）。0 = 沿用 vaxor_feedback_hz 抽帧（旧行为）。
  unsigned feedback_period_ms{0U};
  // 逐台相位错开（ms）。0 = 不错开。
  double stagger_ms{0.0};
  // 假电机 MIT 律更新率（Hz）。0 = 跟随 control_hz。
  double effort_rate_hz{0.0};
  // CAN 主循环周期（ms）。0 = 跟随 control_hz。与 control_hz 不同义：
  // control_hz 驱动控制节拍，本字段驱动总线收发节拍。
  unsigned can_period_ms{0U};
};

// 总线传输类型。core 层不依赖 motor_transport（CMakeLists.txt:34-46 只链
// motor_protocol 与 yaml-cpp），故此处自定义枚举，由 ROS 层映射到
// transport::TransportMode（见 motor_sim_node.cpp 的 bus_transport_mode）。
enum class BusTransport { UsbSerial, SerialSlcan, SocketCan };

struct BusConfig {
  std::string name;
  BusTransport transport{BusTransport::UsbSerial};
  // socket_can 时为接口名（如 robot_vcan0），其余为设备路径。
  std::string endpoint;
  int baud_rate{3000000};            // 仅串口；socket_can 忽略
  int data_bitrate_index{-1};        // 仅 serial_slcan；-1 = 自动 V 探测
  bool probe_adapter{false};         // 仅 serial_slcan；E 取证
  bool allow_external_canfd{false};  // 仅 serial_slcan
  bool brs{true};                    // CAN FD 位率切换；false = 取证对照
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

// CERB SDO 易失对象的初值覆盖。真机上电初值：保护开关 0x2650:00 = 1（保护关）、
// 方向 0x2541:00 = 0（CW）。这两个对象不落盘，"进程重启 = 掉电复现"，
// 所以覆盖初值是复现真机特定场景的唯一手段。
struct SdoOverrides {
  unsigned protection_enabled{1U};  // 0x2650:00，仅 0/1
  unsigned direction{0U};           // 0x2541:00，0=CW 1=CCW
};

struct MotorConfig {
  std::string joint;
  std::string bus;
  std::string protocol;
  unsigned id{0U};
  // CERB MIT τ_ff 的 12 位量化满量程（N·m，厂家 2026-10-21：R58/R48 同 300.0）；
  // 与 plant.effort_limit_nm（plant 的力矩饱和）既不同义也不等值。缺省值取
  // kCerbDefaultMitTorqueLimitNm。
  double mit_torque_limit_nm{
      ::motor_sim::protocol::kCerbDefaultMitTorqueLimitNm};
  // SDO 从站的对端契约：固件靠 0x1008:00 的身份串绑型号、靠 0x6080:00 /
  // 0x2538:00 回读核验。三者必须同源。device_max_speed_rpm 是**模块侧共用值**
  // （R48/R58 同为 3325），逐型号的是 device_max_current_ma。
  std::string device_identity;
  int device_max_speed_rpm{0};
  int device_max_current_ma{0};
  TransmissionConfig transmission;
  JointLimits limits;
  PlantConfig plant;
  SdoOverrides sdo_overrides;
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
