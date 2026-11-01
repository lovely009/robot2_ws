// 参数字段与设备槽序模型 + 值域校验（纯 C，无 rcl 依赖，host 可直测）。
#ifndef PARAM_FIELD_H
#define PARAM_FIELD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 每电机字段（顺序是 blob present 位图与 device_raw 槽索引的契约，勿重排）。
typedef enum {
  PARAM_FIELD_ENABLE = 0,
  PARAM_FIELD_KP,
  PARAM_FIELD_KD,
  PARAM_FIELD_POS_MIN,
  PARAM_FIELD_POS_MAX,
  PARAM_FIELD_VEL_LIMIT,
  PARAM_FIELD_TORQUE_LIMIT,
  // 0x2650:00：对外 1=保护生效，param_bridge 反转为厂家原值；持久化在 raw 槽 4。
  PARAM_FIELD_LIMIT_PROTECT,
  // 可下发字段由 param_bridge 的 kFields[] 显式列出，不以 COUNT 为注册上界。
  // 0x2541:00 方向：0=CW、1=CCW，原值直通，持久化在 raw 槽 5。
  // 新字段仅追加于 COUNT 前，不得重排既有位图与槽索引。
  PARAM_FIELD_DIRECTION,
  PARAM_FIELD_T_MAX,
  PARAM_FIELD_NODE_ID,
  PARAM_FIELD_COUNT
} param_field_t;

// 设备端原值槽位（位置下限/位置上限/最大转速/峰值电流 + v3 的安全开关/方向）。
// 顺序必须与 motor_protocol_adapter.h 的 MOTOR_PROTOCOL_PARAM_SLOT_* 一致——
// 两端跨库同序由 host 测试（test_param_store / test_motor_control）与
// storage_task 里的 _Static_assert 共同保证。
typedef enum {
  PARAM_DEVICE_SLOT_POS_MIN = 0,
  PARAM_DEVICE_SLOT_POS_MAX = 1,
  PARAM_DEVICE_SLOT_VEL_LIMIT = 2,
  PARAM_DEVICE_SLOT_TORQUE_LIMIT = 3,
  // v3：设备侧安全事实。位图 = 1u << slot，device_raw_present 是 u8（上限 8 槽）。
  PARAM_DEVICE_SLOT_SAFETY = 4,    // 0x2650:00（0=看门狗+限位保护开，1=关）
  PARAM_DEVICE_SLOT_DIRECTION = 5, // 0x2541（0=CW，1=CCW）
  PARAM_DEVICE_SLOT_T_MAX = 6,
  PARAM_DEVICE_SLOT_NODE_ID = 7,
  PARAM_DEVICE_SLOT_COUNT = 8
} param_device_slot_t;

// 标量校验：有限值；kp/kd ≥ 0；vel/torque > 0；limit_protect 只接受 0/1
// （布尔量，不做钳制）。超过 profile 上限时钳到 upper_limit
// （upper_limit <= 0 表示不钳），仍返回 true。
bool param_value_ok(param_field_t field, float value, float upper_limit,
    float *clamped);

// 位置限位成对校验：两者有限且 min < max。
bool param_position_limits_ok(float min_rad, float max_rad);

#ifdef __cplusplus
}
#endif

#endif // PARAM_FIELD_H
