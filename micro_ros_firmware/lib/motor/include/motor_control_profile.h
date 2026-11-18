#ifndef MOTOR_CONTROL_PROFILE_H
#define MOTOR_CONTROL_PROFILE_H

// 编译期型号目录，model_name 与 robot_description/config/motor.yaml 对应。
// VAXOR 80 速比缺少完整规格，绑定时报告 PROFILE_REJECTED。

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
