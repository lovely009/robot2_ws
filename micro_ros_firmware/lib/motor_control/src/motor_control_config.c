// motor_control 低频配置：Policy 本地项（增益/软件限位）与 request_id 事务。
//
// 事务规则：每电机至多一个未决事务，新请求立即 BUSY（不排队、不覆盖）；分阶段跨
// 1ms 服务节拍推进（每节拍至多一次迁移），绝不等待设备；可重试阶段自动重试 ≤2 次；
// 读回不一致不得返回 OK，失败不自动恢复运动授权。
//
// SET_MODE 门禁：事务会驱动厂家使能序列（vaxor stop/start 控制帧），故仅对已完成
// 身份确认、处于可配置生命周期且系统已授权的电机受理；模式是否被型号支持由 adapter
// 的阶段数回答（0 → UNSUPPORTED）。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// mailbox 冻结契约的 SET_MODE 值域（0=MIT，1=velocity，2=position）。
#define MOTOR_CONTROL_CONFIG_MODE_MAX 2U

// 身份读取事务的伪 request_id。事务不产生 CONFIG_RESULT（不是上层请求），
// 该值只用于 tx_request_id 的确定性初值；取高位置位以避免与 mailbox
// request_id（低频计数器）碰撞。
#define MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE 0x80000000U

// 身份读取失败后有界退避重试；读取成功但型号未知时不重试。
#define MOTOR_CONTROL_IDENTITY_READ_MAX_ATTEMPTS 3U
#define MOTOR_CONTROL_IDENTITY_READ_RETRY_MS 500U

// 事务发起（定义在提交入口段；config_step 的自动回读 / 身份读取都要用）。
static bool start_tx(motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms);

// 逐电机失能（Policy enable=false 与 SET_ENABLE(false) 共用）：清授权与目标 →
// 在动则发失能帧 → 仍在 ACTIVE/SAFE_STOP 的退回 READY_DISABLED 并出事件。
// ARD §36 冻结语义：复运动必须显式重发 enable=true，本函数不留自动恢复路径。
static void apply_entry_disable(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  entry->policy_enable = false;
  entry->motion_authorized = false;
  entry->has_target = false;
  if (entry->motion_enabled) {
    (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
    entry->motion_enabled = false;
  }
  entry->enable_pending = false;
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DISABLE,
        entry->motor_id, entry->bus_index,
        MOTOR_CONTROL_FAULT_NONE, 0U);
  }
}

// ---------------------------------------------------------------------------
// 事务收尾与步进
// ---------------------------------------------------------------------------

// 身份读取收尾：拓扑上等价于配置事务，但语义是"识别"而非"配置"——
// 不回 CONFIG_RESULT（上层没提过请求），只在成功时按身份串绑定型号。
static void finish_identity_read(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t result,
    uint32_t now_ms) {
  entry->tx_active = false;
  entry->tx_awaiting = false;

  const bool read_ok =
      result == MOTOR_PROTOCOL_CONFIG_OK && !entry->identity_read_overflow;
  if (!read_ok) {
    // 超时、ABORT 或溢出时退避重试，用尽后才定论。
    entry->identity_read_attempts++;
    if (entry->identity_read_attempts < MOTOR_CONTROL_IDENTITY_READ_MAX_ATTEMPTS) {
      entry->identity_read_retry_tick_ms =
          now_ms + MOTOR_CONTROL_IDENTITY_READ_RETRY_MS;
      return; // done 保持 false → 0) 的不变量复检会重发
    }
  }
  entry->identity_read_done = true;

  const motor_protocol_model_profile_t *profile = NULL;
  if (read_ok) {
    profile = motor_control_profile_by_identity(entry->adapter, entry->identity,
        entry->identity_size);
  }
  if (profile != NULL) {
    (void)motor_control_bind_profile_ptr(ctx, entry->motor_id, profile, now_ms);
    return;
  }
  // 身份不可得（重试用尽 / 目录无此串）：保持未绑定不可控，
  // 出 PROFILE_REJECTED 让上层看见"这台设备不认识"。
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
      entry->motor_id, entry->bus_index,
      MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
}

// 上电回读与固件记录按设备原值比对，避免 SI 换算系数误差掩盖不一致。
// 基线存在性取 record_established，不能用自动归一会更新的原值位图。
// 缺记录槽不比对，方向槽除外；方向易失，必须有有效基线。结论仅报告。

// 返回不一致槽位位图（1u << slot）；DEV_INIT 事件与参数视图共用。
uint8_t motor_control_dev_mismatch_mask(const motor_control_entry_t *entry) {
  uint8_t mask = 0U;
  // 方向易失；缺基线或基线非法即方向未知，不得猜为 CW。
  const uint8_t dir_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  if ((entry->record_raw_present & dir_bit) == 0U ||
      entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] > 1U) {
    mask |= dir_bit;
  }
  if (!entry->record_valid) {
    return mask; // 无记录：除方向基线外没有别的可比项
  }
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((entry->record_raw_present & bit) == 0U) {
      continue; // 记录里没写过这项：不构成不一致
    }
    if ((entry->dev_raw_present & bit) == 0U ||
        entry->dev_raw[slot] != entry->record_raw[slot]) {
      mask |= bit; // 记录有、设备没读到 → 也算对不上（无从确认一致）
    }
  }
  return mask;
}

static bool dev_params_match_record(const motor_control_entry_t *entry) {
  return motor_control_dev_mismatch_mask(entry) == 0U;
}

// 基线存在性不能取自动归一会写入的保护槽位图。
static bool entry_has_record(const motor_control_entry_t *entry) {
  return entry->record_established;
}

// 统一的比对结论（自动回读收尾与写事务后的重判共用）：
//   无记录 → NO_RECORD；有记录 → 逐槽位比 → OK / MISMATCH。
// 写事务会就地建立记录，所以"写过一项"就不再是 NO_RECORD——但那不等于通过：
// 记录没写齐（典型是方向无基线）仍判 MISMATCH 并由位图点名，不伪造 OK。
static motor_control_dev_init_t settle_dev_init(
    const motor_control_entry_t *entry) {
  if (!entry_has_record(entry)) {
    return MOTOR_CONTROL_DEV_INIT_NO_RECORD;
  }
  return dev_params_match_record(entry) ? MOTOR_CONTROL_DEV_INIT_OK
                                        : MOTOR_CONTROL_DEV_INIT_MISMATCH;
}

// 冲突冻结入口：停止 ACTIVE/SAFE_STOP 及使能整定窗中的电机，清授权与目标。
// 调用方负责补发对应事件。
static void freeze_entry_as_conflict(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t fault,
    uint32_t now_ms) {
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
      (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
      entry->motion_enabled)) {
    motor_control_soft_stop_entry(ctx, entry, now_ms);
  }
  entry->motion_enabled = false;
  entry->enable_pending = false;
  entry->has_target = false;
  entry->motion_authorized = false;
  entry->policy_enable = false;
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
  entry->unified_fault_code = fault;
  entry->last_failure_reason = fault;
}

// tx_active 复位后写入核对结论并发布事件。
static void apply_dev_init_result(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    motor_control_dev_init_t state,
    uint32_t now_ms) {
  // MISMATCH 携带不一致槽位；UNVERIFIED 携带本次未读到的槽位。
  const uint32_t detail =
      (state == MOTOR_CONTROL_DEV_INIT_MISMATCH)
          ? (uint32_t)motor_control_dev_mismatch_mask(entry)
          : (state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED)
                ? (uint32_t)(MOTOR_CONTROL_DEV_RAW_ALL & ~entry->refresh_slots)
                : 0U;
  const bool ready = (state == MOTOR_CONTROL_DEV_INIT_NA ||
      state == MOTOR_CONTROL_DEV_INIT_OK);
  const bool was_dev_block =
      entry->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM;
  entry->dev_init = state;
  if (ready) {
    if (was_dev_block) {
      entry->unified_fault_code = MOTOR_CONTROL_FAULT_NONE;
      entry->last_failure_reason = MOTOR_CONTROL_FAULT_NONE;
      if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
        entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY,
            entry->motor_id, entry->bus_index,
            MOTOR_CONTROL_FAULT_NONE, 0U);
      }
    }
  } else if (state == MOTOR_CONTROL_DEV_INIT_MISMATCH ||
      state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED ||
      state == MOTOR_CONTROL_DEV_INIT_NO_RECORD) {
    // 核对仅报告，不改生命周期、故障码或运动授权。
    // 警告：限位、速度和力矩可能未经核对，须以硬件急停与机械限位兜底。
    (void)ctx;
    (void)now_ms;
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DEV_INIT,
      entry->motor_id, entry->bus_index, (uint16_t)state,
      detail);
}

// 回读成功：影子取设备真值（设备为准），六个槽位的 SI 值同步落袋。
static void update_shadow_from_read(motor_control_entry_t *entry) {
  entry->dev_min_rad = entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
  entry->dev_max_rad = entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
  entry->dev_max_velocity_rad_s =
      entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
  entry->dev_max_torque_nm =
      entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX];
  entry->dev_limits_valid = true;
  entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_DEVICE;
}

// 写核验成功后同步内存记录的 raw/SI 并置脏；设备与 W25 落盘由 SAVE 触发。
static void update_record_after_write(motor_control_entry_t *entry) {
  const uint8_t pos_min_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN);
  const uint8_t pos_max_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX);
  // 本次事务实际写到的槽位（只写这些：REFRESH 的只读槽在 tx_write_raw 里恒为
  // 0，按 record_raw_present 全量刷会把它们抹成 0）。
  uint8_t written_mask = 0U;
  switch (entry->tx_kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
    written_mask = (uint8_t)(pos_min_bit | pos_max_bit);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT:
    // 保护开关核验成功后，新原值同时成为后续比对基线。
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_T_MAX:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX); break;
  case MOTOR_PROTOCOL_CONFIG_SET_NODE_ID:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID); break;
  case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
    break;
  default:
    return;
  }
  // 置存在位前比较原值；已有且相同不置脏，首次建立基线仍算变化。
  bool changed = false;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    if ((entry->record_raw_present & bit) == 0U ||
        entry->record_raw[slot] != entry->tx_write_raw[slot]) {
      changed = true; // 首次建立基线也算变化
    }
  }
  entry->record_raw_present |= written_mask;
  entry->dev_raw_present |= written_mask;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    // 刚写并读回核验过：设备与记录同值。
    entry->record_raw[slot] = entry->tx_write_raw[slot];
    entry->dev_raw[slot] = entry->tx_write_raw[slot];
    // 位置/速度/力矩取钳制后的请求 SI，安全槽原值直通；设备与记录显示同值。
    float si = NAN;
    switch (slot) {
    case MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN:
      si = entry->tx_request.position_limits.min_position_rad;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX:
      si = entry->tx_request.position_limits.max_position_rad;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX:
      si = entry->tx_request.max_velocity_rad_s;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX:
      si = entry->tx_request.max_torque_nm;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_SAFETY:
    case MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION:
    case MOTOR_PROTOCOL_PARAM_SLOT_T_MAX:
    case MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID:
      si = (float)entry->tx_write_raw[slot];
      break;
    default:
      break;
    }
    entry->record_si[slot] = si;
    entry->dev_si[slot] = si;
  }
  if (changed) {
    // 仅置脏；设备 0x2539 与 W25 写入均由 SAVE 触发。
    entry->record_dirty_slots |= written_mask;
  }
  // 仅上层写事务建立基线；内部归一事务 bit31=1，上层请求 bit31=0。
  if ((entry->tx_request_id & MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE) == 0U) {
    entry->record_established = true;
  }
  entry->record_valid = true;
}

// 设备写事务成功：用内核钳制后的实际请求（tx_request）更新设备影子，
// 保证影子 = 设备已核验值（读回核验通过才进这里）。
static void update_shadow_after_write(motor_control_entry_t *entry) {
  if (entry->profile == NULL ||
      (entry->profile->capability_flags &
      MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    return;
  }
  switch (entry->tx_kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
    entry->dev_min_rad = entry->tx_request.position_limits.min_position_rad;
    entry->dev_max_rad = entry->tx_request.position_limits.max_position_rad;
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    entry->dev_max_velocity_rad_s = entry->tx_request.max_velocity_rad_s;
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
    entry->dev_max_torque_nm = entry->tx_request.max_torque_nm;
    break;
  default:
    return;
  }
  entry->dev_limits_valid = true;
}

void motor_control_finish_tx(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t result,
    uint32_t now_ms) {
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    finish_identity_read(ctx, entry, result, now_ms);
    return;
  }
  // 用 request_id 区分归一与人工写入；失败退避重试，用尽报告 UNVERIFIED。
  bool boot_norm_failed_offline = false;
  if (entry->boot_norm_request_id != 0U &&
      entry->tx_request_id == entry->boot_norm_request_id) {
    entry->boot_norm_request_id = 0U;
    if (result == MOTOR_PROTOCOL_CONFIG_OK) {
      entry->boot_norm_stage =
          entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT
              ? MOTOR_CONTROL_BOOT_NORM_DIRECTION
              : MOTOR_CONTROL_BOOT_NORM_DONE;
    } else {
      entry->boot_norm_attempts++;
      if (entry->boot_norm_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS) {
        boot_norm_failed_offline = true;
      } else {
        entry->boot_norm_retry_tick_ms = now_ms + MOTOR_CONTROL_DEV_READ_RETRY_MS;
      }
    }
  }
  // 核对结论在 tx_active 复位后发布。
  bool dev_init_settled = false;
  motor_control_dev_init_t dev_state = MOTOR_CONTROL_DEV_INIT_NA;
  if (boot_norm_failed_offline) {
    // 归一失败不再自动回读比对，报告 UNVERIFIED。
    dev_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
    entry->dev_read_done = true;
    dev_init_settled = true;
  } else if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    entry->dev_read_attempts++; // 每次回读收尾都计入（成功也算一次尝试）
    // 回读成功的判据：全部槽位的原值位图到齐（每 ADVANCE 落袋一个槽，
    // 重复/缺失槽位图凑不齐 MOTOR_CONTROL_DEV_RAW_ALL）。
    if (result == MOTOR_PROTOCOL_CONFIG_OK &&
        entry->refresh_slots == MOTOR_CONTROL_DEV_RAW_ALL) {
      // 全部阶段到齐才提交：设备端原值位图一次性换成本次读到的槽位。
      entry->dev_raw_present = entry->refresh_slots;
      update_shadow_from_read(entry);
      dev_state = settle_dev_init(entry);
      entry->dev_read_done = true;
      dev_init_settled = true;
    } else {
      // 读取无副作用；退避重试用尽后报告 UNVERIFIED。
      if (entry->dev_read_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS) {
        dev_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
        entry->dev_read_done = true;
        dev_init_settled = true;
      } else {
        entry->dev_init = MOTOR_CONTROL_DEV_INIT_PENDING; // 未定论：等退避重试
        entry->dev_read_retry_tick_ms =
            now_ms + MOTOR_CONTROL_DEV_READ_RETRY_MS;
      }
    }
  } else if (result == MOTOR_PROTOCOL_CONFIG_OK) {
    update_shadow_after_write(entry);
    update_record_after_write(entry);
    // 写后重判：刚核验过的槽位已与记录同值，其余槽位仍按上次回读结论。
    if (entry->dev_read_done) {
      dev_state = settle_dev_init(entry);
      dev_init_settled = true;
    }
  }
  // 保存事务成功：脏位清除（设备侧落盘已发出；W25 由上层在同一动作里落）。
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH &&
      result == MOTOR_PROTOCOL_CONFIG_OK) {
    entry->record_dirty_slots = 0U;
  }
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_T_MAX && result != MOTOR_PROTOCOL_CONFIG_OK) {
    entry->dev_raw_present &= (uint8_t)~(1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX);
    entry->profile = NULL;
    entry->identity_read_done = false;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_IDENTIFYING;
  }
  entry->tx_active = false;
  entry->tx_awaiting = false;
  // 模式事务可能停在厂家 stop 阶段；无论结果均复位使能序列，后续重新发送。
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    entry->motion_enabled = false;
    entry->enable_pending = false;
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index, result,
      entry->tx_request_id);
  if (dev_init_settled) {
    apply_dev_init_result(ctx, entry, dev_state, now_ms);
  }
}

// 外部状态变化中止事务：配置回 UNCONFIRMED；身份读取不置 done、不发终态事件，
// 等表项重新可绑定后再发起。
void motor_control_abort_tx(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  if (ctx == NULL || entry == NULL || !entry->tx_active) {
    return;
  }
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    entry->tx_active = false;
    entry->tx_awaiting = false;
    entry->identity_size = 0U;
    entry->identity_read_overflow = false;
    return;
  }
  motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED, now_ms);
}

// 事务成功收尾：无确认手段的协议不得谎报 OK（ARD §20.5 / 禁止项 9）。
// 例外是读事务：确认就是设备回帧本身，与"写阶段无读回对象"判据无关。
static void finish_tx_success(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  const bool confirmed =
      entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY ||
      entry->adapter == NULL || !entry->adapter->config_verify_required;
  const uint16_t result = confirmed ? (uint16_t)MOTOR_PROTOCOL_CONFIG_OK
                                    : (uint16_t)MOTOR_PROTOCOL_CONFIG_UNCONFIRMED;
  motor_control_finish_tx(ctx, entry, result, now_ms);
}

// 同一物理总线至多一个已发未答阶段（exclude 自身除外），避免 SDO 应答错配。
// 无应答阶段不受此约束。
static bool bus_stage_in_flight(const motor_control_context_t *ctx,
    const motor_control_entry_t *exclude) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    const motor_control_entry_t *other = &ctx->entries[i];
    if (other == exclude || !other->used) {
      continue;
    }
    if (other->bus_index == exclude->bus_index && other->tx_active &&
        other->tx_awaiting) {
      return true;
    }
  }
  return false;
}

// 超时/重试判定先行，随后至多推进一个阶段（跨节拍步进，ARD §20.6）。
void motor_control_config_step(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }

  // 0) 每拍至多为一台在线、可绑定且未定论的电机发起身份读取；中止后可重启。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->identity_read_done || entry->tx_active) {
      continue; // adapter==NULL 防御：否则本拍 break 会挡住后续表项的识别
    }
    if (entry->identity_read_attempts > 0U &&
        (int32_t)(now_ms - entry->identity_read_retry_tick_ms) < 0) {
      continue; // 上一次读失败的退避窗内：到点再重试
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // ACTIVE/FAULT/OFFLINE/冲突态不识别（冲突锁存不得被利用）
    }
    motor_control_start_identity_read(ctx, entry, now_ms);
    break;
  }

  // 0a) 每拍至多一台归一，必须先于回读：保护生效、方向取 W25 有效记录。
  // 缺方向基线时跳过，不得猜为 CW；总线阶段仍按一问一答串行。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->tx_active ||
        entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE) {
      continue;
    }
    if (entry->profile == NULL ||
        (entry->profile->capability_flags &
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
      continue; // 未绑定 / 非设备侧型号（VAXOR）：无设备对象可写
    }
    if (!entry->boot_record_offered || entry->motion_enabled) {
      continue; // 记录先落袋；运动中不动安全边界（防御，理论上到不了）
    }
    if (entry->boot_norm_attempts > 0U &&
        (int32_t)(now_ms - entry->boot_norm_retry_tick_ms) < 0) {
      continue; // 失败退避窗内
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // 运动中/异常态不归一（故障恢复回 READY_DISABLED 后会复检补做）
    }

    motor_protocol_config_request_t request = {0};
    request.motor_id = entry->motor_id;
    if (entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT) {
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 2U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
      request.safety_protect_raw = 0U; // 厂家原值 0 = 保护生效
    } else {
      const uint8_t dir_bit =
          (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      const uint32_t dir_raw =
          entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
      if ((entry->record_raw_present & dir_bit) == 0U || dir_raw > 1U) {
        entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_DONE; // 不猜方向
        continue;
      }
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 3U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
      request.direction_raw = dir_raw;
    }
    entry->boot_norm_request_id = request.request_id;
    (void)start_tx(ctx, entry, &request, now_ms);
    if (!entry->tx_active) {
      // adapter 没有该 kind 的阶段表（设备侧型号却没有保护/方向对象）：
      // start_tx 只出了一条 UNSUPPORTED 事件、没起事务。按"无对象可写"收尾，
      // 否则每个节拍都会重开一次同样的空事务，而且 0b 永远等不到归一完成。
      entry->boot_norm_request_id = 0U;
      entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_DONE;
    }
    break;
  }

  // 0b) 绑定、记录与归一就绪后，每拍至多为一台发起回读。
  // 必须先归一再比对，否则设备上电初值会产生虚假的不一致。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->tx_active || entry->dev_read_done) {
      continue;
    }
    if (entry->profile == NULL ||
        (entry->profile->capability_flags &
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
      continue; // 未绑定 / 非设备侧型号（VAXOR）：无设备对象可比
    }
    if (!entry->boot_record_offered) {
      continue; // 2ms/500Hz 边界先套用 Flash 记录，再回读
    }
    if (entry->boot_norm_stage != MOTOR_CONTROL_BOOT_NORM_DONE) {
      continue; // 0a 还没收尾：先写定保护/方向，再回读比对（顺序是契约）
    }
    if (entry->dev_read_attempts > 0U &&
        (int32_t)(now_ms - entry->dev_read_retry_tick_ms) < 0) {
      continue; // 失败退避窗内：到点再重试
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // 运动中/异常态不主动回读（手动 REFRESH 仍可用）
    }
    motor_protocol_config_request_t request = {0};
    request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 1U;
    request.motor_id = entry->motor_id;
    request.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
    (void)start_tx(ctx, entry, &request, now_ms);
    break;
  }

  // 1) 等待响应超时：≤2 次自动重试，耗尽 → TIMEOUT。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->tx_active || !entry->tx_awaiting) {
      continue;
    }
    if ((int32_t)(now_ms - entry->tx_deadline_tick_ms) < 0) {
      continue;
    }
    entry->tx_retries++;
    if (entry->tx_retries > 2U) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_TIMEOUT, now_ms);
      continue;
    }
    entry->tx_awaiting = false; // 下方发送循环重发本阶段
  }

  // 2) 发送/推进：每节拍至多一个阶段迁移（无响应阶段逐节拍推进）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->tx_active || entry->tx_awaiting) {
      continue;
    }
    const motor_protocol_adapter_t *adapter = entry->adapter;
    if (adapter == NULL) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR, now_ms);
      continue;
    }
    if (bus_stage_in_flight(ctx, entry)) {
      continue; // 本总线还有一问没答：本拍不发，下一拍再看（一问一答）
    }
    motor_protocol_config_stage_info_t info = {0};
    if (adapter->config_stage_info == NULL ||
        !adapter->config_stage_info(entry->tx_stage, entry->profile,
        &entry->tx_request, &info)) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR, now_ms);
      continue;
    }
    if (info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT) {
      // 写阶段的设备端原值在此登记（重试重发同值，覆盖无副作用）。
      entry->tx_write_raw[info.param_slot] = info.param_raw;
    }
    motor_protocol_tx_frame_t frame = {0};
    if (adapter->config_encode_stage != NULL &&
        adapter->config_encode_stage(entry->tx_stage, entry->profile, entry->node,
        entry->bus_index, &entry->tx_request,
        &frame) &&
        frame.size > 0U) {
      frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
      if (!motor_control_push_tx(ctx, &frame)) {
        continue; // 缓冲满：下一节拍重试本阶段
      }
    }
    entry->last_commanded_tick_ms = now_ms;
    if (info.waits_response) {
      entry->tx_awaiting = true;
      entry->tx_deadline_tick_ms = now_ms + info.timeout_ms;
      continue;
    }
    entry->tx_stage++;
    entry->tx_retries = 0U;
    if (entry->tx_stage >= adapter->config_stage_count(entry->profile,
        &entry->tx_request)) {
      finish_tx_success(ctx, entry, now_ms);
    }
  }
}

// 响应帧裁决（on_raw_frame 调用）。返回 true 表示帧被事务消费
// （帧同时是反馈时，解码仍会在调用方继续执行）。
bool motor_control_config_on_frame(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_raw_frame_t *frame) {
  if (ctx == NULL || entry == NULL || !entry->tx_active || !entry->tx_awaiting) {
    return false;
  }
  const motor_protocol_adapter_t *adapter = entry->adapter;
  if (adapter == NULL || adapter->config_on_response == NULL) {
    return false;
  }
  motor_protocol_config_stage_info_t info = {0};
  if (!adapter->config_stage_info(entry->tx_stage, entry->profile,
      &entry->tx_request, &info) ||
      !info.waits_response) {
    return false;
  }
  // 读取类事务的载荷直接落进表项缓冲（容量固定，溢出由 adapter 置位，
  // finish 路径据此判读取失败）。按事务种类路由：身份串→identity，
  // 参数回读→refresh_stages（每阶段一个 motor_protocol_param_stage_t）。
  const bool is_refresh =
      entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  const size_t param_stage_bytes = sizeof(motor_protocol_param_stage_t);
  motor_protocol_read_buffer_t read;
  if (is_refresh) {
    read.data = (uint8_t *)entry->refresh_stages;
    read.capacity = sizeof(entry->refresh_stages);
    read.size = (size_t)entry->refresh_count * param_stage_bytes;
    read.overflow = false;
  } else {
    read.data = entry->identity;
    read.capacity = MOTOR_CONTROL_IDENTITY_CAPACITY;
    read.size = entry->identity_size;
    read.overflow = entry->identity_read_overflow;
  }
  const motor_protocol_stage_outcome_t outcome =
      adapter->config_on_response(entry->tx_stage, entry->profile,
      &entry->tx_request, frame, &read);
  if (is_refresh) {
    entry->refresh_count = (uint8_t)(read.size / param_stage_bytes);
  } else {
    entry->identity_size = read.size;
    entry->identity_read_overflow = read.overflow;
  }
  if (outcome == MOTOR_PROTOCOL_STAGE_ADVANCE ||
      outcome == MOTOR_PROTOCOL_STAGE_COMPLETE) {
    if (is_refresh && info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT &&
        entry->refresh_count > 0U) {
      // 本阶段刚追加的载荷（按阶段顺序追加，槽位由 adapter 声明）。
      const motor_protocol_param_stage_t *stage_payload =
          &entry->refresh_stages[entry->refresh_count - 1U];
      entry->dev_raw[info.param_slot] = stage_payload->raw;
      entry->dev_si[info.param_slot] = stage_payload->value_si;
      entry->refresh_slots |= (uint8_t)(1U << info.param_slot);
    } else if (info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT) {
      // 写阶段：登记本事务实际编码出的设备端原值（成功收尾时同步记录）。
      entry->tx_write_raw[info.param_slot] = info.param_raw;
    }
  }
  switch (outcome) {
  case MOTOR_PROTOCOL_STAGE_ADVANCE:
    entry->tx_stage++;
    entry->tx_retries = 0U;
    entry->tx_awaiting = false;
    if (entry->tx_stage >=
        adapter->config_stage_count(entry->profile, &entry->tx_request)) {
      finish_tx_success(ctx, entry, frame->rx_tick_ms);
    }
    return true;
  case MOTOR_PROTOCOL_STAGE_COMPLETE:
    // 设备提前给出终态（分段传输末段）：剩余阶段不再发送。
    finish_tx_success(ctx, entry, frame->rx_tick_ms);
    return true;
  case MOTOR_PROTOCOL_STAGE_IGNORE:
    // 非事务应答（同节点周期上报等）：不消费本帧，不计重试——调用方仍会
    // 用它走解码/反馈路径。
    return false;
  case MOTOR_PROTOCOL_STAGE_RETRY:
    // 无需回退缓冲：adapter 契约规定只在 ADVANCE/COMPLETE 前追加，判
    // RETRY 前不得写入（见 motor_protocol_adapter.h 的 config_on_response），
    // 因此已收下的前序分段不会被重试重复拼接或丢弃。
    entry->tx_retries++;
    entry->tx_awaiting = false;   // 读回不一致 → 重发本阶段（≤2 次）
    if (entry->tx_retries > 2U) {
      // 读回核验不一致：不得返回 OK（禁止项 9）→ UNCONFIRMED。
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
          frame->rx_tick_ms);
    }
    return true;
  case MOTOR_PROTOCOL_STAGE_FAIL:
  default:
    motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
        frame->rx_tick_ms);
    return true;
  }
}

// ---------------------------------------------------------------------------
// 提交入口
// ---------------------------------------------------------------------------

static bool start_tx(motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  const motor_protocol_adapter_t *adapter = entry->adapter;
  const uint8_t stages =
      adapter->config_stage_count != NULL
          ? adapter->config_stage_count(entry->profile, request)
          : 0U;
  if (stages == 0U) {
    // 厂家对象未决 / 协议不支持：统一 UNSUPPORTED（不猜厂家字节，TBD-CONFIG-01）。
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_UNSUPPORTED, request->request_id);
    return true;
  }
  entry->tx_active = true;
  entry->tx_awaiting = false;
  entry->tx_request_id = request->request_id;
  entry->tx_kind = request->kind;
  entry->tx_request = *request;
  entry->tx_stage = 0U;
  entry->tx_retries = 0U;
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    entry->refresh_count = 0U;
    entry->refresh_slots = 0U;
  }
  memset(entry->tx_write_raw, 0, sizeof(entry->tx_write_raw));
  return true;
}

// 身份读取发起（由 config_step 的不变量复检调用，每拍至多一台）。两级回退：
//   1) 协议有身份读对象 → 走事务机器读设备自报型号串（本函数只发起）；
//   2) 协议无身份读对象 → 目录内该协议唯一型号时直接绑定，多于一型则留
//      未绑定（不猜，保持不可控安全侧）。
void motor_control_start_identity_read(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    uint32_t now_ms) {
  if (ctx == NULL || entry == NULL || entry->adapter == NULL ||
      entry->identity_read_done) {
    return;
  }
  if (entry->tx_active) {
    return; // 事务在飞：下一拍由 config_step 的不变量复检
  }

  motor_protocol_config_request_t request = {0};
  request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE;
  request.motor_id = entry->motor_id;
  request.kind = MOTOR_PROTOCOL_CONFIG_READ_IDENTITY;

  const motor_protocol_adapter_t *adapter = entry->adapter;
  if (adapter->config_stage_count != NULL &&
      adapter->config_stage_count(entry->profile, &request) > 0U) {
    entry->identity_size = 0U;
    entry->identity_read_overflow = false;
    (void)start_tx(ctx, entry, &request, now_ms);
    return;
  }

  entry->identity_read_done = true;
  const motor_protocol_model_profile_t *sole =
      motor_control_profile_sole(adapter);
  if (sole != NULL) {
    (void)motor_control_bind_profile_ptr(ctx, entry->motor_id, sole, now_ms);
    return;
  }
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
      entry->motor_id, entry->bus_index,
      MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
}

// 设备侧限位/限值写必须在电机未运动时进行：厂家手册要求设置限位时电机失能；
// 运行中改动位置限位可能使当前位置立即越出新限位而触发厂家保护（0x0040/
// 0x0080）。与换绑门禁同口径——安全边界不得在运动中变动。
static bool device_limit_write_allowed(const motor_control_entry_t *entry) {
  return entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE &&
         entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_SAFE_STOP;
}

// 设备侧回读须等待归一完成，避免比较上电初值。
// 已有本会话定论时允许手动重读，包括归一失败后的恢复；非设备侧型号不受限。
static bool device_refresh_allowed(const motor_control_entry_t *entry) {
  if (entry->profile == NULL ||
      (entry->profile->capability_flags &
      MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    return true;
  }
  return entry->dev_read_done ||
         entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE;
}

static bool reject_invalid(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  ctx->config_rejected_count++;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry != NULL ? entry->motor_id : request->motor_id,
      entry != NULL ? entry->bus_index : 0U,
      MOTOR_PROTOCOL_CONFIG_INVALID, request->request_id);
  return false;
}

// 事务进行中的统一拒绝（一问一答：同一电机新请求直接 BUSY，不排队）。
static bool reject_busy(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  ctx->config_rejected_count++;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index,
      MOTOR_PROTOCOL_CONFIG_BUSY, request->request_id);
  return false;
}

// 仅原值已核验且等于 1 才算保护确知关闭；未知状态不触发此确认门。
static bool motor_control_unprotected_needs_confirm(
    const motor_control_entry_t *entry) {
  const uint8_t bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  return (entry->dev_raw_present & bit) != 0U &&
         entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U;
}

// 保护被手动关掉的台仍允许武装，但**每次**都要显式确认：关保护 = 该关节没有
// 任何限位在跑（CERB 是设备侧限位，固件没有软限位兜底）。判据取设备真值影子，
// 拒绝只进 last_failure_reason（不是设备故障，与 STALE_GENERATION 同待遇）。
static bool reject_if_unprotected_unconfirmed(
    motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request, uint32_t now_ms) {
  if (!request->enable || request->confirm_unprotected ||
      !motor_control_unprotected_needs_confirm(entry)) {
    return false;
  }
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED;
  (void)reject_invalid(ctx, entry, request, now_ms);
  return true;
}

// 策略落地：enable + MIT 增益一次原子落地，SET_POLICY 与 motor_control_apply_policy 共用，避免两条
// 路径的门禁与钳制漂移。**必须原子**：增益要先于使能到位，否则存在一个用旧 Kd 使能的窗口；拆成两次
// 调用会留下它，也会让「两笔只成一半」变成可观测状态。
// 代数门（防「周期重发自动复权」）：「未武装 → 武装」是一次新意图，policy_generation 必须严格前进
// （初值 0，故首次武装至少为 1）。冻结态与显式失能都会清掉 policy_enable，故无需在那些路径额外锁存。
// 它挡不住「每次都递增代数的周期发布者」——那与人工逐次授权在行为上不可区分，机制层面到此为止。
bool motor_control_apply_policy_entry(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  if (entry->profile == NULL) {
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (!isfinite(request->kp) || !isfinite(request->kd) ||
      request->kp < 0.0f || request->kd < 0.0f) {
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (request->enable && !entry->policy_enable &&
      request->policy_generation <= entry->policy_generation) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_STALE_GENERATION;
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (request->enable && !motor_control_dev_params_allow_motion(entry)) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (reject_if_unprotected_unconfirmed(ctx, entry, request, now_ms)) {
    return false;
  }
  // 增益先在本地落定（钳到型号上限），再动使能位——顺序即语义。
  entry->kp =
      request->kp > entry->profile->kp_max ? entry->profile->kp_max : request->kp;
  entry->kd =
      request->kd > entry->profile->kd_max ? entry->profile->kd_max : request->kd;
  entry->has_kp_kd = true;
  entry->policy_generation = request->policy_generation;
  if (request->enable) {
    entry->policy_enable = true;
    entry->motion_authorized = true;
  } else {
    apply_entry_disable(ctx, entry, now_ms);
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index,
      MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
  return true;
}

void motor_control_disable_mask(motor_control_context_t *ctx, uint64_t motor_mask,
    uint32_t now_ms) {
  if (ctx == NULL) return;
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (entry->used && entry->motor_id > 0U && entry->motor_id <= 64U &&
        (motor_mask & (UINT64_C(1) << (entry->motor_id - 1U))) != 0U) {
      entry->policy_enable = false;
      motor_control_soft_stop_entry(ctx, entry, now_ms);
    }
  }
}

void motor_control_report_config_result(motor_control_context_t *ctx,
    uint32_t request_id, uint16_t motor_id, uint16_t reason, uint32_t now_ms) {
  if (ctx == NULL || request_id == 0U) return;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      motor_id, 0U, reason, request_id);
}

bool motor_control_submit_config(motor_control_context_t *ctx,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  if (ctx == NULL || request == NULL || request->request_id == 0U) {
    return false;
  }
  motor_control_entry_t *entry = motor_control_find_entry(ctx, request->motor_id);
  if (entry == NULL) {
    ctx->config_rejected_count++;
    return false; // 未知电机：无表项可挂事件，仅计数
  }

  switch (request->kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_GAINS: {
    // 增益是低频 Policy（ARD §19.4）：本地生效，上限来自 Profile，
    // 不进入命令快照、不刷新命令 freshness。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!isfinite(request->kp) || !isfinite(request->kd) || request->kp < 0.0f ||
        request->kd < 0.0f) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    entry->kp = request->kp > entry->profile->kp_max ? entry->profile->kp_max
                                                       : request->kp;
    entry->kd = request->kd > entry->profile->kd_max ? entry->profile->kd_max
                                                       : request->kd;
    entry->has_kp_kd = true;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
    return true;
  }
  case MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT: {
    // 身份冲突锁存，仅重启可解除；DEV_PARAM 改走回读，不伪造一致。
    // 厂家故障走协议清错，下一状态帧更新生命周期；运动中禁止发送清错控制帧。
    if (entry->profile == NULL || !device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_CONFIG_CONFLICT) {
      return reject_invalid(ctx, entry, request, now_ms);   // 身份冲突不可解除
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM) {
      // DEV_PARAM 重读与手动 REFRESH 共用归一完成门。
      if (!device_refresh_allowed(entry)) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      motor_protocol_config_request_t refresh = *request;
      refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
      return start_tx(ctx, entry, &refresh, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_POLICY:
    return motor_control_apply_policy_entry(ctx, entry, request, now_ms);
  case MOTOR_PROTOCOL_CONFIG_SET_ENABLE: {
    // 逐电机 Enable（ARD §19）：本地策略位，非事务。
    // true = 重立运动授权（仍需新快照才能运动）；false = 立即退出运动：
    // 软停 + 失能帧 + 清授权位（ARD §36 冻结语义）。
    if (request->enable) {
      // 参数核对策略统一由 helper 决定；当前仅报告、不拦截。
      if (!motor_control_dev_params_allow_motion(entry)) {
        entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (reject_if_unprotected_unconfirmed(ctx, entry, request, now_ms)) {
        return false;
      }
      entry->policy_enable = true;
      entry->motion_authorized = true;
    } else {
      apply_entry_disable(ctx, entry, now_ms);
    }
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
    return true;
  }
  case MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS: {
    // 只读回读允许 ACTIVE 态；须已绑定且归一完成，避免比较上电初值。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!device_refresh_allowed(entry)) {
      return reject_busy(ctx, entry, request, now_ms);   // 归一未收尾：稍后重试
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_MODE: {
    if (request->mode > MOTOR_CONTROL_CONFIG_MODE_MAX) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    // 事务会驱动厂家使能序列：只对身份已确认、处于可配置生命周期且系统
    // 已授权的电机受理（未识别/未授权设备不得收到厂家使能帧）。
    if (entry->profile == NULL ||
        (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_SAFE_STOP)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!ctx->authorized) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  default:
    return reject_invalid(ctx, entry, request, now_ms);
  }
}
