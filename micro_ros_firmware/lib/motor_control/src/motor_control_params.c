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

// 绑定后首次设备影子：取 Profile 标定（±mit 位置限、额定转速、峰值扭矩）作
// 占位，来源标 PROFILE——设备真值要等上电回读（dev_init=PENDING 期间该台
// 不可运动，见 motor_control_dev_params_allow_motion）。
void motor_control_set_default_device_shadow(motor_control_entry_t *entry) {
  if (entry == NULL || entry->profile == NULL || entry->dev_limits_valid) {
    return;
  }
  const motor_protocol_model_profile_t *p = entry->profile;
  if ((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_NONE;
    entry->dev_init = MOTOR_CONTROL_DEV_INIT_NA; // 无设备对象可比对
    return;
  }
  entry->dev_min_rad = -p->mit_position_limit_rad;
  entry->dev_max_rad = p->mit_position_limit_rad;
  entry->dev_max_velocity_rad_s =
      p->rated_speed_rad_s > 0.0f ? p->rated_speed_rad_s : NAN;
  entry->dev_max_torque_nm = p->peak_torque_nm > 0.0f ? p->peak_torque_nm : NAN;
  entry->dev_limits_valid = true;
  entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_PROFILE;
  entry->dev_init = MOTOR_CONTROL_DEV_INIT_PENDING;
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
  view->limits_source =
      device_side ? entry->limits_source : MOTOR_CONTROL_LIMITS_SOURCE_NONE;
  view->dev_init = (uint8_t)entry->dev_init;
  // 设备端原值只在"来源=设备回读"时有效：其余来源（型号标定/Flash 记录）
  // 没有设备原值可给，页面据此区分"设备真值"与"离线回落值"。
  // 力矩列（A）用的是**限值系数**（规格书峰值对），与写事务的钳制同源；
  // 反馈侧的扭矩估计仍走 Kt×减速比×效率（见 motor_protocol.h 的说明）。
  view->torque_per_amp = entry->profile != NULL
                             ? motor_protocol_profile_limit_nm_per_amp(
                                   entry->profile)
                             : NAN;
  view->dev_raw_present = device_side ? entry->dev_raw_present : 0U;
  view->record_raw_present = device_side ? entry->record_raw_present : 0U;
  // 报告要的两张位图：已写未保存（含易失槽）与"记录对不上设备的哪几项"
  // （口径与 DEV_INIT 事件的 detail 相同，不另立一套判据）。
  view->record_dirty_slots = device_side ? entry->record_dirty_slots : 0U;
  view->dev_mismatch_mask =
      (device_side && entry->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH)
          ? motor_control_dev_mismatch_mask(entry)
          : 0U;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    view->dev_raw[slot] =
        (view->dev_raw_present & bit) != 0U ? entry->dev_raw[slot] : 0U;
    view->record_raw[slot] =
        (view->record_raw_present & bit) != 0U ? entry->record_raw[slot] : 0U;
    // 缺失量一律 NaN：0 会在页面上冒充"限位设成了 0"，那是个能让人按下去的值。
    view->dev_si[slot] =
        (view->dev_raw_present & bit) != 0U ? entry->dev_si[slot] : NAN;
    view->record_si[slot] =
        (view->record_raw_present & bit) != 0U ? entry->record_si[slot] : NAN;
  }
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

// 复杂度上限：O(电机数×存储记录数) 线性查找；64 台 × 50Hz 边界仅 4k 比较，
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

  // 两件事分开做，别互相挡：
  //  - boot_record_offered：只表示"本次调用已把记录（哪怕是"没有记录"）告知
  //    本表项"，设备参数回读据此才发起（config_step 0b）；
  //  - 记录内容捕获：只在真拿到记录时落袋一次（首次非 NULL 胜出）。若把它塞
  //    进 offered 的守卫里，先"声明无记录"再拿到记录的路径会把记录永久丢掉，
  //    比对基准随之消失——不一致就再也发现不了。
  // 记录内容不需要 Profile，也不受 mask 影响：它是上电比对基准与离线回落值。
  if (!entry->boot_record_offered) {
    entry->boot_record_offered = true;
  }
  if (rec != NULL && !entry->record_valid) {
    entry->record_valid = true;
    memcpy(entry->record_raw, rec->device_raw, sizeof(entry->record_raw));
    entry->record_raw_present = rec->device_raw_present;
    // 有设备参数的基线记录（device_raw_present 非零才算数：只存了 kp/kd 的
    // 记录对设备参数比对毫无用处，"无记录"是它诚实的结论）。
    // 已知标签漂移：保存一旦把保护槽写进 W25，**下一次**上电这里就看到
    // device_raw_present != 0 ⇒ 走"有记录"分支；若方向槽仍缺基线，结论会报
    // MISMATCH（点名方向）而不是 NO_RECORD。两者都冻结、都拒绝使能，行为完全
    // 一致，差的只是标签——别把它当成"记录莫名其妙冒出来了"。
    if (rec->device_raw_present != 0U) {
      entry->record_established = true;
    }
    // 记录侧的 SI 值：blob 里本来就存了（写设备时同一笔事务产生的），直接搬。
    // 缺字段的槽位保持 NaN。
    for (uint8_t i = 0U; i < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++i) {
      entry->record_si[i] = NAN;
    }
    if (rec->has_position_limits) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] = rec->pos_min_rad;
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX] = rec->pos_max_rad;
    }
    if (rec->has_velocity_limit) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX] =
          rec->velocity_limit_rad_s;
    }
    if (rec->has_torque_limit) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] =
          rec->torque_limit_nm;
    }
    // 两个安全槽是枚举/标志位，原值即 SI（与 adapter 的 passthrough 同口径）。
    entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] =
        (float)rec->device_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] =
        (float)rec->device_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  }

  // rec == NULL（本机对该台还没拿到记录）：只完成"已告知"这一次性动作，
  // 不碰 mask——把 SEEN 打在这里会让后到的真记录被当成"已套尽"，记录被静默
  // 丢弃（与上面 capture 同一类坑）。
  if (rec == NULL) {
    return;
  }

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
      // CERB：记录只作显示回落，绝不发 SDO 写；设备真值以回读为准。
      // 已经回读过设备的（source=DEVICE）不再被记录覆盖——设备为准。
      if (entry->limits_source != MOTOR_CONTROL_LIMITS_SOURCE_DEVICE) {
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
        entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_RECORD;
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
  // 注意 count == 0（Flash 全空）**不是**提前返回的理由：本次调用仍要向每个
  // 表项声明"记录已就位（哪怕是没有记录）"，上电设备参数回读才有发起时机
  // （见 motor_control_config_step 0b）。这里提前返回会让空 Flash 的板子
  // 永远停在 PENDING → 按"初始化完成才可运动"的口径整机不可使能。
  if (ctx == NULL || records == NULL) {
    return;
  }
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    // 无记录也要调用：本次调用同时承担"向表项声明记录已就位（含没有记录）"，
    // 上电设备参数回读据此才有发起时机（见 motor_control_config_step 0b）。
    // 有记录时按 mask 套用增益/限值；套尽后本函数是常数开销。
    const motor_control_stored_params_t *rec =
        find_stored(records, count, entry->motor_id);
    apply_stored_to_entry(entry, rec);
  }
}
