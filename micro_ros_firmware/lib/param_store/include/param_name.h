// ROS 参数名模型（纯 C，无 rcl 依赖，host 可直测）：
// 电机参数名 m<id>.<field> 的唯一生成/解析处，以及值域校验。
// param_bridge 的 rcl 缝合层只做名字 <-> 内核动作的转换，逻辑全部在此。
#ifndef PARAM_NAME_H
#define PARAM_NAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_NAME_MAX_LEN 32U // rclc 名缓冲为 50，这里收紧留余量

// 每电机字段（顺序是 blob present 位图与参数注册顺序的契约，勿重排）。
typedef enum {
  PARAM_FIELD_ENABLE = 0,
  PARAM_FIELD_KP,
  PARAM_FIELD_KD,
  PARAM_FIELD_POS_MIN,
  PARAM_FIELD_POS_MAX,
  PARAM_FIELD_VEL_LIMIT,
  PARAM_FIELD_TORQUE_LIMIT,
  // 设备侧看门狗/限位保护开关（0x2650:00），0/1 数值：**1 = 保护生效**。
  // 收正向语义、由 param_bridge 反转成厂家原值（1 = 关）——设备里那个"1 = 关"
  // 是个反直觉陷阱，不能让它泄漏到操作员看到的数字上。
  // 追加在 COUNT 前：不重排既有字段（blob present 位图与注册顺序的契约）。
  // 值本身不入 blob 的 float 字段，它在 device_raw[槽 4] 里已有持久化。
  PARAM_FIELD_LIMIT_PROTECT,
  // 【B 方案实测】PID 六项（字典 0x2532..0x2537，UINT32 寄存器原值 0..32768）。
  // 追加在 COUNT 前，不重排既有字段（blob present 位图契约）。
  PARAM_FIELD_CURRENT_P,
  PARAM_FIELD_CURRENT_I,
  PARAM_FIELD_VELOCITY_P,
  PARAM_FIELD_VELOCITY_I,
  PARAM_FIELD_POSITION_P,
  PARAM_FIELD_POSITION_I,
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
  PARAM_DEVICE_SLOT_COUNT = 6
} param_device_slot_t;

typedef enum {
  PARAM_NAME_UNKNOWN = -1,
  PARAM_NAME_MOTOR = 0,
  PARAM_NAME_SYSTEM = 1
} param_name_kind_t;

// 生成 "m<id>.<field>"。返回 false = 缓冲不足。
bool param_name_build_motor(char *buf, size_t size, uint16_t motor_id,
                            param_field_t field);

// 解析电机参数名；"system.*" 返回 PARAM_NAME_SYSTEM（字段不回）。
param_name_kind_t param_name_parse(const char *name, uint16_t *motor_id,
                                   param_field_t *field);

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

#endif // PARAM_NAME_H
