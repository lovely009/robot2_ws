// storage_projection.h 实现：纯 C，只依赖 motor_control 参数类型与 blob 格式。

#include "storage_projection.h"

#include <assert.h>
#include <math.h>
#include <string.h>

// 设备参数对象原值 ↔ 字段/参数对象常量的跨库顺序断言（本模块同时可见两个库的
// 头，放在这里做唯一一处交叉校验；顺序错了会把 mA 当位置限写进记录）。
static_assert(PARAM_DEVICE_SLOT_POS_MIN == MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN);
static_assert(PARAM_DEVICE_SLOT_POS_MAX == MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX);
static_assert(PARAM_DEVICE_SLOT_VEL_LIMIT ==
              MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX);
static_assert(PARAM_DEVICE_SLOT_TORQUE_LIMIT ==
              MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX);
static_assert(PARAM_DEVICE_SLOT_COUNT == MOTOR_PROTOCOL_PARAM_SLOT_COUNT);
static_assert(PARAM_DEVICE_SLOT_SAFETY == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
static_assert(PARAM_DEVICE_SLOT_DIRECTION ==
              MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
static_assert(PARAM_STORE_RECORD_COUNT == MOTOR_CONTROL_MAX_MOTORS,
              "blob 记录数必须与注册表容量一致");

void storage_projection_views_to_blob(
    param_store_blob_t *blob, const motor_control_param_view_t *views,
    size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const motor_control_param_view_t *v = &views[i];
    if (v->motor_id == 0U) {
      continue;
    }
    if (isfinite(v->kp)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_KP, v->kp);
    }
    if (isfinite(v->kd)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_KD, v->kd);
    }
    if (v->device_side_limits) {
      // 设备端原值优先；本次上电还没回读过设备时退回记录原值（保住比对基准）。
      const bool have_device = v->dev_raw_present != 0U;
      const uint8_t raw_present =
          have_device ? v->dev_raw_present : v->record_raw_present;
      const uint32_t *raw_values = have_device ? v->dev_raw : v->record_raw;
      const uint8_t pos_min_bit =
          (uint8_t)(1U << PARAM_DEVICE_SLOT_POS_MIN);
      const uint8_t pos_max_bit =
          (uint8_t)(1U << PARAM_DEVICE_SLOT_POS_MAX);
      const uint8_t vel_bit = (uint8_t)(1U << PARAM_DEVICE_SLOT_VEL_LIMIT);
      const uint8_t torque_bit =
          (uint8_t)(1U << PARAM_DEVICE_SLOT_TORQUE_LIMIT);
      // 位置限位成对落盘：只有两端都经设备核验才收录。
      if ((raw_present & (uint8_t)(pos_min_bit | pos_max_bit)) ==
              (uint8_t)(pos_min_bit | pos_max_bit) &&
          isfinite(v->pos_min_rad) && isfinite(v->pos_max_rad)) {
        param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MIN,
                             v->pos_min_rad);
        param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MAX,
                             v->pos_max_rad);
        param_store_blob_set_device_raw(
            blob, v->motor_id, PARAM_DEVICE_SLOT_POS_MIN,
            raw_values[PARAM_DEVICE_SLOT_POS_MIN]);
        param_store_blob_set_device_raw(
            blob, v->motor_id, PARAM_DEVICE_SLOT_POS_MAX,
            raw_values[PARAM_DEVICE_SLOT_POS_MAX]);
      }
      if ((raw_present & vel_bit) != 0U &&
          isfinite(v->velocity_limit_rad_s)) {
        param_store_blob_set(blob, v->motor_id, PARAM_FIELD_VEL_LIMIT,
                             v->velocity_limit_rad_s);
        param_store_blob_set_device_raw(
            blob, v->motor_id, PARAM_DEVICE_SLOT_VEL_LIMIT,
            raw_values[PARAM_DEVICE_SLOT_VEL_LIMIT]);
      }
      if ((raw_present & torque_bit) != 0U && isfinite(v->torque_limit_nm)) {
        param_store_blob_set(blob, v->motor_id, PARAM_FIELD_TORQUE_LIMIT,
                             v->torque_limit_nm);
        param_store_blob_set_device_raw(
            blob, v->motor_id, PARAM_DEVICE_SLOT_TORQUE_LIMIT,
            raw_values[PARAM_DEVICE_SLOT_TORQUE_LIMIT]);
      }
      continue; // 设备侧型号的限值到此为止（不落软限位分支）
    }
    if (isfinite(v->pos_min_rad)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MIN,
                           v->pos_min_rad);
    }
    if (isfinite(v->pos_max_rad)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MAX,
                           v->pos_max_rad);
    }
    if (isfinite(v->velocity_limit_rad_s)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_VEL_LIMIT,
                           v->velocity_limit_rad_s);
    }
    if (isfinite(v->torque_limit_nm)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_TORQUE_LIMIT,
                           v->torque_limit_nm);
    }
  }
}

size_t storage_projection_blob_to_boot(
    const param_store_blob_t *blob, motor_control_stored_params_t *out,
    size_t capacity) {
  size_t count = 0U;
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_t *rec = &blob->records[i];
    if ((rec->present == 0U && rec->device_raw_present == 0U) ||
        count >= capacity) {
      continue;
    }
    motor_control_stored_params_t *dst = &out[count++];
    memset(dst, 0, sizeof(*dst));
    dst->motor_id = rec->motor_id;
    float value = 0.0f;
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_KP, &value) &&
        param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_KD, &value)) {
      dst->has_kp_kd = true;
      dst->kp = rec->kp;
      dst->kd = rec->kd;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_POS_MIN,
                             &value) &&
        param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_POS_MAX,
                             &value)) {
      dst->has_position_limits = true;
      dst->pos_min_rad = rec->pos_min;
      dst->pos_max_rad = rec->pos_max;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_VEL_LIMIT,
                             &value)) {
      dst->has_velocity_limit = true;
      dst->velocity_limit_rad_s = rec->vel_limit;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_TORQUE_LIMIT,
                             &value)) {
      dst->has_torque_limit = true;
      dst->torque_limit_nm = rec->torque_limit;
    }
    // 设备端原值：上电比对的唯一依据（SI 值只用于显示与换算）。
    dst->device_raw_present = rec->device_raw_present;
    for (size_t slot = 0; slot < PARAM_DEVICE_SLOT_COUNT; ++slot) {
      dst->device_raw[slot] = rec->device_raw[slot];
    }
  }
  return count;
}
