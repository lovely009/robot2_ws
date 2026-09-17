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
  PARAM_FIELD_COUNT
} param_field_t;

// 设备端原值槽位（位置下限/位置上限/最大转速/峰值电流）。顺序必须与
// motor_protocol_adapter.h 的 MOTOR_PROTOCOL_PARAM_SLOT_* 一致——两端跨库
// 同序由 host 测试（test_param_store / test_motor_control）与 storage_task
// 里的 _Static_assert 共同保证。
typedef enum {
  PARAM_DEVICE_SLOT_POS_MIN = 0,
  PARAM_DEVICE_SLOT_POS_MAX = 1,
  PARAM_DEVICE_SLOT_VEL_LIMIT = 2,
  PARAM_DEVICE_SLOT_TORQUE_LIMIT = 3,
  PARAM_DEVICE_SLOT_COUNT = 4
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

// 标量校验：有限值；kp/kd ≥ 0；vel/torque > 0。超过 profile 上限时钳到
// upper_limit（upper_limit <= 0 表示不钳），仍返回 true。
bool param_value_ok(param_field_t field, float value, float upper_limit,
                    float *clamped);

// 位置限位成对校验：两者有限且 min < max。
bool param_position_limits_ok(float min_rad, float max_rad);

#ifdef __cplusplus
}
#endif

#endif // PARAM_NAME_H
