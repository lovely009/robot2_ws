// motor_control 型号目录（ARD-0007 §8.2/§8.3 编译期"型号目录"）。
// 行值来源（仓库权威，不得凭空猜测）：
//  - ros2_ws/src/motor_hardware_interface/config/motor.yaml（Kt/减速比/效率/
//    额定与峰值扭矩/额定电流）
//  - 旧默认测试表（control_task.cpp，默认 MIT 增益）
//  - ARD §8.3 已确认型号基线（电压/编码器/限位能力）
//
// Vaxor 80 速比无完整规格，刻意不在目录中（ARD §8.3、禁止项 15）。
// TBD-CAN-03/TBD-CONFIG-01 关闭后补齐的量以 0/占位表示，不杜撰。

#include "motor_control_profile.h"

#include <stddef.h>

static const motor_protocol_model_profile_t kProfiles[] = {
    {
        // Vaxor 25mm 30 速比档（motor.yaml: vaxor_2530）
        .model_name = "vaxor_2530",
        .protocol = &motor_protocol_vaxor_adapter,
        .gear_ratio = 30.0f,
        .gear_efficiency = 0.70f,
        .torque_constant_nm_per_amp = 0.042f,
        .rated_torque_nm = 0.0f,  // 未确认，不参与钳制
        .peak_torque_nm = 0.0f,
        .rated_speed_rad_s = 0.0f,
        .supply_voltage_v = 15.0f, // 12~18V 标称中值（ARD §8.3）
        .default_kp = 0.008f,
        .default_kd = 1.0f,
        .mit_position_limit_rad = 0.0f, // vaxor 量化范围在 codec 内部钳制
        .mit_velocity_limit_rad_s = 0.0f,
        .mit_torque_limit_nm = 0.0f,
        .kp_max = 0.65535f, // 线协议量化上限（KP_SCALE 0.00001 × 0xffff）
        .kd_max = 655.35f,  // KD_SCALE 0.01 × 0xffff
        .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT |
                            MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE |
                            MOTOR_PROTOCOL_CAP_AUTONOMOUS_FEEDBACK,
    },
    {
        // Vaxor 25mm 50 速比档（motor.yaml: vaxor_2550）
        .model_name = "vaxor_2550",
        .protocol = &motor_protocol_vaxor_adapter,
        .gear_ratio = 50.0f,
        .gear_efficiency = 0.65f,
        .torque_constant_nm_per_amp = 0.042f,
        .rated_torque_nm = 0.0f,
        .peak_torque_nm = 0.0f,
        .rated_speed_rad_s = 0.0f,
        .supply_voltage_v = 15.0f,
        .default_kp = 0.008f,
        .default_kd = 1.0f,
        .mit_position_limit_rad = 0.0f,
        .mit_velocity_limit_rad_s = 0.0f,
        .mit_torque_limit_nm = 0.0f,
        .kp_max = 0.65535f,
        .kd_max = 655.35f,
        .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT |
                            MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE |
                            MOTOR_PROTOCOL_CAP_AUTONOMOUS_FEEDBACK,
    },
    {
        // CREB R48-101（motor.yaml: creb_r48_101；ARD §8.3 基线）
        .model_name = "creb_r48_101",
        .protocol = &motor_protocol_creb_adapter,
        .gear_ratio = 101.0f,
        .gear_efficiency = 1.0f,
        .torque_constant_nm_per_amp = 0.077f,
        .rated_torque_nm = 5.0f,
        .peak_torque_nm = 11.0f,
        .rated_speed_rad_s = 3.14159265f, // 30 RPM 关节侧
        .supply_voltage_v = 48.0f,
        .default_kp = 30.0f,
        .default_kd = 1.0f,
        .mit_position_limit_rad = 3.14159265f, // π rad（与真机参数页一致）
        .mit_velocity_limit_rad_s = 3.14159265f,
        .mit_torque_limit_nm = 11.0f,
        .kp_max = 500.0f, // MIT 量化上限 [0,500]
        .kd_max = 5.0f,   // [0,5]
        .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT |
                            MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE,
    },
    {
        // CREB R58-101（motor.yaml: creb_r58_101；ARD §8.3 基线）
        .model_name = "creb_r58_101",
        .protocol = &motor_protocol_creb_adapter,
        .gear_ratio = 101.0f,
        .gear_efficiency = 1.0f,
        .torque_constant_nm_per_amp = 0.086f,
        .rated_torque_nm = 20.0f,
        .peak_torque_nm = 45.0f,
        .rated_speed_rad_s = 3.14159265f,
        .supply_voltage_v = 48.0f,
        .default_kp = 30.0f,
        .default_kd = 1.0f,
        .mit_position_limit_rad = 3.14159265f,
        .mit_velocity_limit_rad_s = 3.14159265f,
        .mit_torque_limit_nm = 45.0f,
        .kp_max = 500.0f,
        .kd_max = 5.0f,
        .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT |
                            MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE,
    },
};

size_t motor_control_profile_count(void) {
  return sizeof(kProfiles) / sizeof(kProfiles[0]);
}

const motor_protocol_model_profile_t *motor_control_profile_at(size_t index) {
  if (index >= motor_control_profile_count()) {
    return NULL;
  }
  return &kProfiles[index];
}
