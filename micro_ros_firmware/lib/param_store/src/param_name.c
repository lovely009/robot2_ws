#include "param_name.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *const kFieldNames[PARAM_FIELD_COUNT] = {
    "enable",         "kp",          "kd",         "pos_min",
    "pos_max",        "vel_limit",   "torque_limit"};

bool param_name_build_motor(char *buf, size_t size, uint16_t motor_id,
                            param_field_t field) {
  if (buf == NULL || size == 0U || motor_id < 1U || motor_id > 127U ||
      (int)field < 0 || field >= PARAM_FIELD_COUNT) {
    return false;
  }
  const int n = snprintf(buf, size, "m%u.%s", (unsigned)motor_id,
                         kFieldNames[field]);
  return n > 0 && (size_t)n < size;
}

param_name_kind_t param_name_parse(const char *name, uint16_t *motor_id,
                                   param_field_t *field) {
  if (name == NULL) {
    return PARAM_NAME_UNKNOWN;
  }
  if (strncmp(name, "system.", 7U) == 0) {
    return PARAM_NAME_SYSTEM;
  }
  if (name[0] != 'm') {
    return PARAM_NAME_UNKNOWN;
  }
  char *dot = strchr(name, '.');
  if (dot == NULL || dot == name + 1) {
    return PARAM_NAME_UNKNOWN;
  }
  unsigned id = 0U;
  if (sscanf(name + 1, "%u", &id) != 1 || id < 1U || id > 127U) {
    return PARAM_NAME_UNKNOWN;
  }
  // sscanf 允许 "7x.kp" 这类尾随，必须确认 'm' 与 '.' 之间全是数字。
  for (const char *p = name + 1; p < dot; ++p) {
    if (*p < '0' || *p > '9') {
      return PARAM_NAME_UNKNOWN;
    }
  }
  const char *suffix = dot + 1;
  for (int f = 0; f < PARAM_FIELD_COUNT; ++f) {
    if (strcmp(suffix, kFieldNames[f]) == 0) {
      if (motor_id != NULL) {
        *motor_id = (uint16_t)id;
      }
      if (field != NULL) {
        *field = (param_field_t)f;
      }
      return PARAM_NAME_MOTOR;
    }
  }
  return PARAM_NAME_UNKNOWN;
}

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
    default:
      break; // enable/位置字段不走本函数（enable 是 bool，位置成对校验）
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
