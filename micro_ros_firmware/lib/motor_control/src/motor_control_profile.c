// motor_control 型号目录（ARD-0007 §8.2/§8.3 编译期"型号目录"）。
// 行值来源（仓库权威，不得凭空猜测）：
//  - ros2_ws/src/motor_hardware_interface/config/motor.yaml（Kt/减速比/效率/
//    额定与峰值扭矩/额定电流、默认 MIT 增益）
//  - ARD §8.3 已确认型号基线（电压/编码器/限位能力）
//
// Vaxor 80 速比无完整规格，刻意不在目录中（ARD §8.3、禁止项 15）。
// TBD-CAN-03/TBD-CONFIG-01 关闭后补齐的量以 0/占位表示，不杜撰。

#include "motor_control_profile.h"

#include <stddef.h>
#include <string.h>

static const motor_protocol_model_profile_t kProfiles[] = {
  {
    // Vaxor 25mm 50 速比档（motor.yaml: vaxor_2550）
    .model_name = "vaxor_2550",
    .identity_string = NULL,     // vaxor 无身份读对象 → 协议内唯一型号兜底
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
    // CERB R48-101（motor.yaml: cerb_r48_101；ARD §8.3 基线）
    .model_name = "cerb_r48_101",
    // 设备 SDO 0x1008 回串（台架实测，与 R58 可区分）
    .identity_string = "R48LITE_V2",
    .protocol = &motor_protocol_cerb_adapter,
    .gear_ratio = 101.0f,
    .gear_efficiency = 1.0f,
    .torque_constant_nm_per_amp = 0.077f,
    .rated_torque_nm = 5.0f,
    .peak_torque_nm = 11.0f,
    // docs/CE-RB-R48-101-Lite-CO-I-参数规格.pdf：额定 1.3 A / 峰值 3.2 A
    .rated_current_amp = 1.3f,
    .peak_current_amp = 3.2f,
    .rated_speed_rad_s = (float)MOTOR_PROTOCOL_PI,     // 30 RPM 关节侧
    .supply_voltage_v = 48.0f,
    .default_kp = 30.0f,
    .default_kd = 1.0f,
    .mit_position_limit_rad = (float)MOTOR_PROTOCOL_PI,     // π rad（与真机参数页一致）
    .mit_velocity_limit_rad_s = (float)MOTOR_PROTOCOL_PI,
    .mit_torque_limit_nm = 11.0f,
    .kp_max = 500.0f,     // MIT 量化上限 [0,500]
    .kd_max = 5.0f,       // [0,5]
    .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT |
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE,
  },
  {
    // CERB R58-101（motor.yaml: cerb_r58_101；ARD §8.3 基线）
    .model_name = "cerb_r58_101",
    // 设备 SDO 0x1008 回串（台架实测）
    .identity_string = "R58LITE_V2",
    .protocol = &motor_protocol_cerb_adapter,
    .gear_ratio = 101.0f,
    .gear_efficiency = 1.0f,
    .torque_constant_nm_per_amp = 0.086f,
    .rated_torque_nm = 20.0f,
    .peak_torque_nm = 45.0f,
    // docs/CE-RB-R58-101-Lite-CO-I-参数规格.pdf：额定 3.5 A / 峰值 10.8 A
    .rated_current_amp = 3.5f,
    .peak_current_amp = 10.8f,
    .rated_speed_rad_s = (float)MOTOR_PROTOCOL_PI,
    .supply_voltage_v = 48.0f,
    .default_kp = 30.0f,
    .default_kd = 1.0f,
    .mit_position_limit_rad = (float)MOTOR_PROTOCOL_PI,
    .mit_velocity_limit_rad_s = (float)MOTOR_PROTOCOL_PI,
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

// 身份串比较：设备回串含尾随 NUL/空白填充（SDO 按对象声明长度回传），
// 一律剪掉后整串精确比较——不做前缀/大小写/模糊匹配，识别错了会把整机
// 换算比（减速比/Kt）配错，比不绑定危险得多。
static bool identity_matches(const char *identity_string, const uint8_t *identity,
    size_t length) {
  if (identity_string == NULL || identity == NULL) {
    return false;
  }
  size_t end = length;
  while (end > 0U && (identity[end - 1U] == '\0' || identity[end - 1U] == ' ' ||
      identity[end - 1U] == '\t' || identity[end - 1U] == '\r' ||
      identity[end - 1U] == '\n')) {
    --end;
  }
  const size_t expected = strlen(identity_string);
  if (end != expected) {
    return false;
  }
  return memcmp(identity, identity_string, expected) == 0;
}

const motor_protocol_model_profile_t *motor_control_profile_by_identity(
    const motor_protocol_adapter_t *protocol, const uint8_t *identity,
    size_t length) {
  if (protocol == NULL || identity == NULL || length == 0U) {
    return NULL;
  }
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const motor_protocol_model_profile_t *profile = &kProfiles[i];
    if (profile->protocol != protocol) {
      continue; // 协议不符不得绑定（与 bind 门禁一致）
    }
    if (identity_matches(profile->identity_string, identity, length)) {
      return profile;
    }
  }
  return NULL;
}

const motor_protocol_model_profile_t *motor_control_profile_sole(
    const motor_protocol_adapter_t *protocol) {
  if (protocol == NULL) {
    return NULL;
  }
  const motor_protocol_model_profile_t *sole = NULL;
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    if (kProfiles[i].protocol != protocol) {
      continue;
    }
    if (sole != NULL) {
      return NULL; // 目录内该协议多于一型 → 无从判定，不猜
    }
    sole = &kProfiles[i];
  }
  return sole;
}
