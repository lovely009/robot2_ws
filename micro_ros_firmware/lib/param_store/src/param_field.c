// 参数值域校验。所有上限/符号判据集中在这里，param_bridge 的整条预校验直接用。

#include "param_field.h"

#include <math.h>

bool param_value_ok(param_field_t field, float value, float upper_limit,
    float *clamped) {
  if (!isfinite(value)) {
    return false;
  }
  switch (field) {
  case PARAM_FIELD_KP:
  case PARAM_FIELD_KD:
    if (value < 0.0f) {
      return false;
    }
    break;
  case PARAM_FIELD_VEL_LIMIT:
  case PARAM_FIELD_TORQUE_LIMIT:
    if (value <= 0.0f) {
      return false;
    }
    break;
  case PARAM_FIELD_LIMIT_PROTECT:
  case PARAM_FIELD_DIRECTION:
    // 布尔量：只认 0/1。**不参与下面的 upper_limit 钳制**——把 2 钳成 1
    // 会把"写错了"变成"静默关掉保护 / 静默改方向"，是这两个对象上最不该
    // 发生的失败模式。
    if (value != 0.0f && value != 1.0f) {
      return false;
    }
    if (clamped != NULL) {
      *clamped = value;
    }
    return true;
  default:
    break;   // enable/位置字段不走本函数（enable 是 bool，位置成对校验）
  }
  if (upper_limit > 0.0f && value > upper_limit) {
    value = upper_limit;
  }
  if (clamped != NULL) {
    *clamped = value;
  }
  return true;
}

bool param_position_limits_ok(float min_rad, float max_rad) {
  return isfinite(min_rad) && isfinite(max_rad) && min_rad < max_rad;
}
