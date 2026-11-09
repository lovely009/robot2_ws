#ifndef MOTOR_CONTROL_PROFILE_H
#define MOTOR_CONTROL_PROFILE_H

// 编译期型号目录（ARD-0007 §8.2/§8.3）：这是"型号目录"而不是设备安装表——
// 行值来自仓库权威型号库
// ros2_ws/src/motor_hardware_interface/config/motor.yaml 与 ARD §8.3 基线。
//
// Vaxor 80 速比无完整规格，刻意不在目录中（ARD §8.3、禁止项 15）：
// 绑定必然失败，电机停留在不可控状态并出 PROFILE_REJECTED 事件。
//
// TBD-PROFILE-01：固件/上层 Profile 一致性检测机制未冻结；model_name 与
// ROS 型号库同名是当前一致性锚点。

#include <stddef.h>

#include "motor_protocol_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

// 目录条数与只读访问。
size_t motor_control_profile_count(void);
const motor_protocol_model_profile_t *motor_control_profile_at(size_t index);

// 按设备自报身份串查型号：协议与身份串双匹配，剪掉回串的尾随 NUL/空白后
// 整串精确比较（无前缀/模糊匹配）。未命中返回 NULL（不绑定，不可控）。
const motor_protocol_model_profile_t *motor_control_profile_by_identity(
  const motor_protocol_adapter_t *protocol, const uint8_t *identity,
  size_t length);

// 协议内唯一型号：目录内该协议只有一行才返回，多行返回 NULL（无从判定不猜）。
// 供无身份读对象的协议兜底绑定（如 vaxor）。
const motor_protocol_model_profile_t *motor_control_profile_sole(
  const motor_protocol_adapter_t *protocol);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_PROFILE_H
