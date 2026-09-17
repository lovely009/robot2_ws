// 参数视图副本与 boot 存储参数套用（param_bridge / W25 存储任务支撑层）。
//
// 视图语义：
//  - kp/kd：未发 SET_GAINS 前为 NaN（has_kp_kd 未置）；
//  - Vaxor 限值取本地 soft_* 字段（各自 has 位）；
//  - CERB 限值取设备影子 dev_*（绑定后填 Profile 默认，REFRESH/写事务刷新）。
//
// 存储套用（boot blob）：只按 stored_apply_mask 在 Profile 绑定后分批生效；
// CERB 设备值只填影子，绝不主动写驱动器（设备真实值由 refresh 回读核验）。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// 绑定后首次设备影子：取 Profile 标定（±mit 位置限、额定转速、峰值扭矩）。
void motor_control_set_default_device_shadow(motor_control_entry_t *entry) {
  if (entry == NULL || entry->profile == NULL || entry->dev_limits_valid) {
    return;
  }
  const motor_protocol_model_profile_t *p = entry->profile;
  if ((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    return;
  }
  entry->dev_min_rad = -p->mit_position_limit_rad;
  entry->dev_max_rad = p->mit_position_limit_rad;
  entry->dev_max_velocity_rad_s =
      p->rated_speed_rad_s > 0.0f ? p->rated_speed_rad_s : NAN;
  entry->dev_max_torque_nm = p->peak_torque_nm > 0.0f ? p->peak_torque_nm : NAN;
  entry->dev_limits_valid = true;
}

static void fill_view(const motor_control_entry_t *entry,
                      motor_control_param_view_t *view) {
  memset(view, 0, sizeof(*view));
  view->motor_id = entry->motor_id;
  view->online = entry->online;
  view->bound = entry->profile != NULL;
  view->enable = entry->policy_enable;
  // 未显式设置增益时回退 Profile 默认值（命令打包同口径），使上层能成对
  // 读到有限的当前值；未绑定时仍为 NaN。
  if (entry->has_kp_kd) {
    view->kp = entry->kp;
    view->kd = entry->kd;
  } else if (entry->profile != NULL) {
    view->kp = entry->profile->default_kp;
    view->kd = entry->profile->default_kd;
  } else {
    view->kp = NAN;
    view->kd = NAN;
  }

  const bool device_side =
      entry->profile != NULL &&
      (entry->profile->capability_flags &
       MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U;
  view->device_side_limits = device_side;
  if (device_side) {
    view->pos_min_rad = entry->dev_limits_valid ? entry->dev_min_rad : NAN;
    view->pos_max_rad = entry->dev_limits_valid ? entry->dev_max_rad : NAN;
    view->velocity_limit_rad_s =
        entry->dev_limits_valid ? entry->dev_max_velocity_rad_s : NAN;
    view->torque_limit_nm =
        entry->dev_limits_valid ? entry->dev_max_torque_nm : NAN;
  } else {
    view->pos_min_rad = entry->has_soft_limits ? entry->soft_min_rad : NAN;
    view->pos_max_rad = entry->has_soft_limits ? entry->soft_max_rad : NAN;
    view->velocity_limit_rad_s =
        entry->has_soft_vel_limit ? entry->soft_max_velocity_rad_s : NAN;
    view->torque_limit_nm =
        entry->has_soft_torque_limit ? entry->soft_max_torque_nm : NAN;
  }
  view->limits_valid =
      isfinite(view->pos_min_rad) && isfinite(view->pos_max_rad) &&
      isfinite(view->velocity_limit_rad_s) && isfinite(view->torque_limit_nm);
}

size_t motor_control_copy_param_views(motor_control_context_t *ctx,
                                      motor_control_param_view_t *out,
                                      size_t capacity) {
  if (ctx == NULL || out == NULL || capacity == 0U) {
    return 0U;
  }
  size_t n = 0U;
  for (size_t i = 0; i < ctx->entry_count && n < capacity; ++i) {
    if (!ctx->entries[i].used) {
      continue;
    }
    fill_view(&ctx->entries[i], &out[n++]);
  }
  return n;
}

// ponytail: O(电机数×存储记录数) 线性查找；64 台 × 50Hz 边界仅 4k 比较，
// 真到三总线 40 台规模仍可忽略；若记录查找变热点再改为 motor_id 索引表。
static const motor_control_stored_params_t *find_stored(
    const motor_control_stored_params_t *records, size_t count,
    uint16_t motor_id) {
  for (size_t i = 0; i < count; ++i) {
    if (records[i].motor_id == motor_id) {
      return &records[i];
    }
  }
  return NULL;
}

// stored_apply_mask 编码：bit7 = 已见过该电机的存储记录（初始化一次）；
// bit0 = 待套增益，bit1 = 待套限值。套尽后低位清零、bit7 保留——之后的
// REFRESH 回读/设备写核验影子绝不能被重复 apply 覆盖。
#define STORED_MASK_SEEN 0x80U
#define STORED_MASK_GAINS 0x01U
#define STORED_MASK_LIMITS 0x02U

static void apply_stored_to_entry(motor_control_entry_t *entry,
                                  const motor_control_stored_params_t *rec) {
  const motor_protocol_model_profile_t *p = entry->profile;

  // 首次见到该记录：按字段存在性建立待套用位（只一次）。
  if ((entry->stored_apply_mask & STORED_MASK_SEEN) == 0U) {
    uint8_t mask = STORED_MASK_SEEN;
    if (rec->has_kp_kd) {
      mask |= STORED_MASK_GAINS;
    }
    if (rec->has_position_limits || rec->has_velocity_limit ||
        rec->has_torque_limit) {
      mask |= STORED_MASK_LIMITS;
    }
    entry->stored_apply_mask = mask;
  }
  if (p == NULL) {
    return; // 未绑定：保留待套用位，下一边界重试
  }
  uint8_t mask = entry->stored_apply_mask;
  if ((mask & (STORED_MASK_GAINS | STORED_MASK_LIMITS)) == 0U) {
    return; // 已套尽：后续影子变化（refresh/设备写）保持不动
  }

  if ((mask & STORED_MASK_GAINS) != 0U && rec->has_kp_kd) {
    entry->kp = rec->kp > p->kp_max ? p->kp_max : rec->kp;
    entry->kd = rec->kd > p->kd_max ? p->kd_max : rec->kd;
    entry->has_kp_kd = true;
  }
  if ((mask & STORED_MASK_LIMITS) != 0U) {
    if ((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
      // Vaxor：本地软件限位，立即参与打包前钳制。
      if (rec->has_position_limits) {
        entry->soft_min_rad = rec->pos_min_rad;
        entry->soft_max_rad = rec->pos_max_rad;
        entry->has_soft_limits = true;
      }
      if (rec->has_velocity_limit) {
        entry->soft_max_velocity_rad_s = rec->velocity_limit_rad_s;
        entry->has_soft_vel_limit = true;
      }
      if (rec->has_torque_limit) {
        entry->soft_max_torque_nm = rec->torque_limit_nm;
        entry->has_soft_torque_limit = true;
      }
    } else if ((p->capability_flags &
                MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U) {
      // CERB：只填设备影子，不发 SDO（设备真实值以 refresh 回读为准）。
      if (rec->has_position_limits) {
        entry->dev_min_rad = rec->pos_min_rad;
        entry->dev_max_rad = rec->pos_max_rad;
      }
      if (rec->has_velocity_limit) {
        entry->dev_max_velocity_rad_s = rec->velocity_limit_rad_s;
      }
      if (rec->has_torque_limit) {
        entry->dev_max_torque_nm = rec->torque_limit_nm;
      }
      entry->dev_limits_valid = true;
    }
  }
  entry->stored_apply_mask = STORED_MASK_SEEN; // 低位清零，绝不重复套用
}

void motor_control_apply_stored_params(
    motor_control_context_t *ctx,
    const motor_control_stored_params_t *records, size_t count,
    uint32_t now_ms) {
  (void)now_ms; // 当前套用不产生事件，暂不需要时基
  if (ctx == NULL || records == NULL || count == 0U) {
    return;
  }
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    // 无记录且无待套用位：跳过；有待套用位（记录只会在 boot 数组里持续传入）。
    const motor_control_stored_params_t *rec =
        find_stored(records, count, entry->motor_id);
    if (rec != NULL) {
      apply_stored_to_entry(entry, rec);
    }
  }
}
