// 参数字段与设备槽序模型 + 值域校验（纯 C，无 rcl 依赖，host 可直测）。
//
// 原名 param_name.h，承载 m<id>.<field> 名的生成/解析；参数服务器 2026-09-21 退役、
// 参数面改话题后没人再拼/解参数名，那套 API 随之删除，本文件按剩余内容改名。
// 剩下两半仍被生产路径使用：两套槽序枚举 + 值域校验。
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
  // 设备侧看门狗/限位保护开关（0x2650:00），0/1 数值：**1 = 保护生效**。
  // 收正向语义、由 param_bridge 反转成厂家原值（1 = 关）——设备里那个"1 = 关"
  // 是个反直觉陷阱，不能让它泄漏到操作员看到的数字上。
  // 追加在 COUNT 前：不重排既有字段（blob present 位图与槽索引的契约）。
  // 值本身不入 blob 的 float 字段，它在 device_raw[槽 4] 里已有持久化。
  PARAM_FIELD_LIMIT_PROTECT,
  // 【已删除，不得回归】此处曾追加 PID 六项（0x2532..0x2537），被以 PARAM_FIELD_COUNT 为界的注册
  // 循环自动带进参数树，变成 6 个恒 NaN、不可写的死项，并把每电机项数从 5 顶到 11 → 门限虚高 →
  // 第 37 台起静默失败。PID 非安全项、驱动器自持，台架调参用离线 SDO 工具。
  // 教训：本枚举尾端是「全集」，而「可经参数面下发的集合」由 src/param_bridge.cpp 的 kFields[] 显式
  // 列出（线协议 field 序）——加字段必须同时改两处，别拿 PARAM_FIELD_COUNT 当上界。
  // 设备侧方向 0x2541:00：0 = CW、1 = CCW，**原值直通、不做语义反转**（没有 0x2650 那种「1 = 关」的
  // 反向陷阱），持久化在 device_raw[槽 5]；追加字段必须放在 COUNT 前——present 位图与槽索引是契约。
  PARAM_FIELD_DIRECTION,
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
