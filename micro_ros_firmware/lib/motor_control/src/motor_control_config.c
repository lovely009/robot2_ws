// motor_control 低频配置：Policy 本地项（增益/软件限位）与 request_id 事务
// （ARD-0007 §19/§20/§26）。
//
// 事务规则（ARD §20）：每电机至多一个未决事务，新请求立即 BUSY（不排队、
// 不覆盖）；分阶段、跨 1ms 服务节拍推进（每节拍至多一个阶段迁移），绝不
// 等待设备；可重试阶段自动重试 ≤2 次；结果码统一七种；读回不一致不得
// 返回 OK（禁止项 9），失败不自动恢复运动授权。
//
// SET_MODE 门禁：事务会驱动厂家使能序列（vaxor stop/start 控制帧），因此
// 仅对已完成身份确认（Profile 已绑定）、处于可配置生命周期且系统已授权的
// 电机受理；模式值域为 mailbox 契约常量（0=MIT 1=velocity 2=position），
// 具体模式是否被型号支持由 adapter 的阶段数回答（0 → UNSUPPORTED）。
//
// OK 语义（禁止项 9）：无响应写阶段无法确认设备状态的协议
// （adapter->config_verify_required）不得返回 OK，统一收敛为 UNCONFIRMED。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// mailbox 冻结契约的 SET_MODE 值域（0=MIT，1=velocity，2=position）。
#define MOTOR_CONTROL_CONFIG_MODE_MAX 2U

// 身份读取事务的伪 request_id。事务不产生 CONFIG_RESULT（不是上层请求），
// 该值只用于 tx_request_id 的确定性初值；取高位置位以避免与 mailbox
// request_id（低频计数器）碰撞。
#define MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE 0x80000000U

// 事务发起（定义在提交入口段；config_step 的自动回读 / 身份读取都要用）。
static bool start_tx(motor_control_context_t *ctx, motor_control_entry_t *entry,
                     const motor_protocol_config_request_t *request,
                     uint32_t now_ms);

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
  entry->identity_read_done = true;

  const motor_protocol_model_profile_t *profile = NULL;
  if (result == MOTOR_PROTOCOL_CONFIG_OK && !entry->identity_read_overflow) {
    profile = motor_control_profile_by_identity(entry->adapter, entry->identity,
                                                entry->identity_size);
  }
  if (profile != NULL) {
    (void)motor_control_bind_profile_ptr(ctx, entry->motor_id, profile, now_ms);
    return;
  }
  // 身份不可得（超时/拒绝/回串超容量/目录无此串）：保持未绑定不可控，
  // 出 PROFILE_REJECTED 让上层看见"这台设备不认识"。
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
                           entry->motor_id, entry->bus_index,
                           MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
}

// ---------------------------------------------------------------------------
// 设备参数初始化（上电/重连回读设备 → 与固件记录比对）
//
// 口径（2026-09-17 定）：**参数以设备内为准，固件记录只作比对基准**。
//  - 绑定完成后自动回读一次设备六对象（0x607D:01/02、0x6080、0x2538、0x2650:01、0x2541）；
//  - 回读值与记录逐槽位比对，比的是**设备端原值**：关节侧 SI 两端都过同一套
//    Kt×减速比换算，系数配错时两边一起错、比对恒通过，等于没比；
//  - 不一致或回读失败 → 该台"未初始化"：冻结授权、不出运动帧，直到参数被
//    重写（写事务会同时更新设备与记录）或重连后重新回读；
//  - 记录里没有的槽位不参与比对（刚烧录、Flash 还空的电机不该整体卡住）。
// ---------------------------------------------------------------------------

// 记录与设备回读逐槽位比对（设备端原值）。返回值 = 不一致槽位位图
// （1u << MOTOR_PROTOCOL_PARAM_SLOT_*；0 = 一致）。位图随 DEV_INIT 事件带出去，
// 让上位机能直接说"是哪一项对不上"——否则操作者按页面写回一个槽位、别的槽位
// 仍不一致，结论还是未初始化，看起来就像"设置不生效"。
static uint8_t dev_params_mismatch_mask(const motor_control_entry_t *entry) {
  uint8_t mask = 0U;
  if (!entry->record_valid) {
    return 0U; // 本机无记录：没有比对基准
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
  return dev_params_mismatch_mask(entry) == 0U;
}

// 定论落地：写 dev_init，并按结论放行/冻结该台。调用点必须在 tx_active
// 复位之后（冻结路径与故障路径同形，避免事务重入）。
static void apply_dev_init_result(motor_control_context_t *ctx,
                                  motor_control_entry_t *entry,
                                  motor_control_dev_init_t state,
                                  uint32_t now_ms) {
  // 位图随事件带出去（1u<<slot；0 = 无此项信息），但两态语义不同：
  //  - MISMATCH：**对不上**的槽位（记录有、设备读到的值不同）；
  //  - UNVERIFIED：**没读到**的槽位（refresh_slots 是本次已落袋的槽位，
  //    取补集即缺失项）。0x2650:01 的读子索引是本分支唯一未经真机验证的对象
  //    （TBD-CONFIG-02），设备 abort 时若不给位图，操作者分不清是它没答还是
  //    总线抖动——位图让事件直接点名"哪个对象没到"。
  const uint32_t detail =
      (state == MOTOR_CONTROL_DEV_INIT_MISMATCH)
          ? (uint32_t)dev_params_mismatch_mask(entry)
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
             state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED) {
    // 未初始化（不一致 / 未能核对）：**运动中的走软停，不硬失能**——
    // 上电自检与写入都核验过，真出现不一致属于异常，此时按"停止运动但保留
    // 阻尼"处理（TBD-SAFETY-01 口径），不把力矩瞬间撤掉。
    // 静止态（READY_DISABLED 等）没有要停的东西，直接锁存结论。
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
        (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
         entry->motion_enabled)) {
      motor_control_soft_stop_entry(ctx, entry, now_ms);
    }
    // 与冲突/故障同口径冻结（ARD §36：失败不自动恢复运动授权）：结论未转好
    // 之前不得使能，恢复需显式 SET_ENABLE(true) + 新快照。
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->has_target = false;
    entry->motion_authorized = false;
    entry->policy_enable = false;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_DEV_PARAM;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
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

// 写事务成功：设备与固件记录一起更新（"写就是设备和驱动板一起写"）。
// 记录落 raw + SI，下次上电比对才不会把刚写过的值判成不一致。
static void update_record_after_write(motor_control_context_t *ctx,
                                      motor_control_entry_t *entry) {
  const uint8_t pos_min_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN);
  const uint8_t pos_max_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX);
  // 本次事务实际写到的槽位（只写这些：安全/方向是只读 REFRESH 槽，
  // tx_write_raw 里恒为 0，按 record_raw_present 全量刷会把它们抹成 0）。
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
    default:
      return;
  }
  entry->record_raw_present |= written_mask;
  entry->dev_raw_present |= written_mask;
  // 设备已核验写入 → 记录同步落盘（任务层响应）。不落盘就断电 = 下次上电
  // 比对必然不一致 = 该台未初始化，属于自找的停机。
  ctx->record_dirty = true;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    // 刚写并读回核验过：设备与记录同值。
    entry->record_raw[slot] = entry->tx_write_raw[slot];
    entry->dev_raw[slot] = entry->tx_write_raw[slot];
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
  // 设备参数定论（回读结果）延后到 tx_active 复位之后再落地：apply_dev_init_result
  // 会走冻结路径，事务若仍在飞会被重入。
  bool dev_init_settled = false;
  motor_control_dev_init_t dev_state = MOTOR_CONTROL_DEV_INIT_NA;
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    entry->dev_read_attempts++; // 每次回读收尾都计入（成功也算一次尝试）
    // 回读成功的判据：全部槽位的原值位图到齐（每 ADVANCE 落袋一个槽，
    // 重复/缺失槽位图凑不齐 MOTOR_CONTROL_DEV_RAW_ALL）。
    if (result == MOTOR_PROTOCOL_CONFIG_OK &&
        entry->refresh_slots == MOTOR_CONTROL_DEV_RAW_ALL) {
      // 全部阶段到齐才提交：设备端原值位图一次性换成本次读到的槽位。
      entry->dev_raw_present = entry->refresh_slots;
      update_shadow_from_read(entry);
      dev_state = dev_params_match_record(entry)
                      ? MOTOR_CONTROL_DEV_INIT_OK
                      : MOTOR_CONTROL_DEV_INIT_MISMATCH;
      entry->dev_read_done = true;
      dev_init_settled = true;
    } else {
      // 读不到设备：先退避重试，用尽尝试次数才定论 UNVERIFIED。读是纯只读
      // 事务，多问几次没有副作用；一次总线抖动就判"未核对"会让该台在
      // "初始化未完成不可使能"的口径下白白卡住。
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
    update_record_after_write(ctx, entry);
    // 写后重判：刚核验过的槽位已与记录同值，其余槽位仍按上次回读结论。
    if (entry->dev_read_done) {
      dev_state = dev_params_match_record(entry)
                      ? MOTOR_CONTROL_DEV_INIT_OK
                      : MOTOR_CONTROL_DEV_INIT_MISMATCH;
      dev_init_settled = true;
    }
  }
  entry->tx_active = false;
  entry->tx_awaiting = false;
  // 模式事务驱动过厂家使能序列（vaxor stop/start）：无论成败，使能序列
  // 重走——若事务异常停在 stop 态而内核仍记"已使能"，后续快照不再发使能帧，
  // 电机将哑到人工干预。强制复位只多一帧使能，两种结局都安全。
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

// 事务中止（离线 / 冲突 / 厂家故障 / 撤权路径共用）：不是事务自身走出的
// 裁决，而是"设备/会话状态变了，别等了"。
//  - 配置事务：回 UNCONFIRMED 终态，避免上层请求悬挂到自身超时。
//  - 身份读取：不回任何事件、也不置 identity_read_done——中止只说明本轮读
//    没走完，不代表设备不认识自己；表项一旦回到可绑定态（重连）会重新发起。
//    若在这里定论为"识别失败"，一次断链就能让电机在本次上电周期内永久
//    不可控。
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

// 同总线上是否已有别的表项"已发未答"（exclude 自身不计）。
//
// 厂家的配置事务（CERB 的 CANopen SDO）是**严格一问一答**：说明书里所有示例
// 都是一条请求对一条应答。并发发问会连环出事——设备按序处理请求，迟到的应答
// 与本阶段索引不符被判 RETRY，重发又把设备压得更久，最后表现为"一直超时"，
// 而重发的请求还会挤掉正常控制帧。故按总线串行：任一时刻一条总线上至多一个
// 未决阶段（新阶段发出前先让上一问落地）。无应答阶段（VAXOR 的本地策略）不受
// 影响；bus_index 相同的表项才是同一物理总线。
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

  // 0) 身份识别：不变量式重启——"在线 + 处于可绑定生命周期 + 本轮未定论 +
  // 无事务在飞"即发起（每拍至多一台，避免单拍突发撑开 1ms 节拍）。
  // 不变量而非一次性标志的理由：事务可能被离线/撤权/故障路径中止，中止
  // 不构成识别定论（见 motor_control_abort_tx），此刻若不复检就再没有
  // 发起时机——一次断链会让电机在本次上电周期内永久不可控。
  // 定论过的（identity_read_done）不再重试：认不出的设备不会因为多问几次
  // 就认识，重试只会刷屏事件环并占用总线。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->identity_read_done || entry->tx_active) {
      continue; // adapter==NULL 防御：否则本拍 break 会挡住后续表项的识别
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // ACTIVE/FAULT/OFFLINE/冲突态不识别（冲突锁存不得被利用）
    }
    motor_control_start_identity_read(ctx, entry, now_ms);
    break;
  }

  // 0b) 设备参数上电回读：绑定完成 + Flash 记录已落袋 + 在线 + 无事务在飞 +
  //     本会话未定论 → 自动发起一次 REFRESH_PARAMS。与身份识别同为不变量式
  //     复检（重连后 record/结论复位，自然重跑），每拍至多一台。
  //     记录先落袋再回读：比对基准必须在比对之前就位，否则"无记录可比"
  //     会把不一致静默判成 OK。
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
      continue; // 20ms 边界先套用 Flash 记录，再回读
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
      entry->tx_awaiting = false; // 读回不一致 → 重发本阶段（≤2 次）
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

// 策略落地：enable + MIT 增益，一次原子落地。SET_POLICY config kind 与批量
// 入口 motor_control_apply_policy 共用，避免两条路径的门禁与钳制漂移。
//
// 为什么必须原子：增益要先于使能到位，否则存在一个用旧 Kd 使能的窗口；拆成
// SET_GAINS + SET_ENABLE 两次调用会留下它，也会让「两笔只成一半」变成可观测
// 状态。
//
// 代数门（防「周期重发自动复权」）：「未武装 → 武装」是一次新意图，
// policy_generation 必须严格前进（初值 0，故首次武装至少要 1）。冻结态
// （FAULT/OFFLINE/CONFIG_CONFLICT/DEV_PARAM）与显式失能都会清掉
// policy_enable，所以本规则**不需要在那几条路径上额外锁存任何东西**。
// 它挡不住「每次都递增代数的周期发布者」：那与人工逐次授权在行为上不可
// 区分。ARD §36 的「显式」在机制层面到此为止，再往上靠上层自律。
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
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                           entry->motor_id, entry->bus_index,
                           MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
  return true;
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
      // 不进入 100 Hz 命令、不刷新命令 freshness。
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
    case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS: {
      if (!isfinite(request->position_limits.min_position_rad) ||
          !isfinite(request->position_limits.max_position_rad) ||
          request->position_limits.min_position_rad >=
              request->position_limits.max_position_rad) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      // 硬件参数表第三条：位置限位不得超过型号表的 MIT 量化范围
      // （CERB = ±π，与设备 0x607D 字典 ±32768 cnt 同域）。0 = 未确认不钳制。
      motor_protocol_config_request_t bounded = *request;
      const float pos_limit = entry->profile != NULL
                                  ? entry->profile->mit_position_limit_rad
                                  : 0.0f;
      if (pos_limit > 0.0f) {
        if (bounded.position_limits.min_position_rad < -pos_limit) {
          bounded.position_limits.min_position_rad = -pos_limit;
        }
        if (bounded.position_limits.max_position_rad > pos_limit) {
          bounded.position_limits.max_position_rad = pos_limit;
        }
      }
      if (entry->profile != NULL &&
          (entry->profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
        // 软件限位协议（Vaxor）：本地保存，打包前钳制（ARD §26.1）。
        entry->soft_min_rad = bounded.position_limits.min_position_rad;
        entry->soft_max_rad = bounded.position_limits.max_position_rad;
        entry->has_soft_limits = true;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
        return true;
      }
      // 设备侧限位（CERB）：必须走 request_id 事务 + 读回核验（ARD §26.2），
      // 且仅限未运动状态（见 device_limit_write_allowed）。
      if (!device_limit_write_allowed(entry)) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      return start_tx(ctx, entry, &bounded, now_ms); // 落事务的是钳制后的请求
    }
    case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT: {
      // 设备侧管理对象（仅 CERB）：0x6080 电机端最大转速 / 0x2538 电机端峰值
      // 电流，走 CANopen 快速 SDO 写 + 读回核验（ARD §20.4、§26.2）。
      // 请求为关节侧 SI；换算到电机端量由 adapter 依 profile 完成。
      if (entry->profile == NULL) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      motor_protocol_config_request_t clamped = *request;
      if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT) {
        if (!isfinite(request->max_velocity_rad_s) ||
            request->max_velocity_rad_s <= 0.0f) {
          return reject_invalid(ctx, entry, request, now_ms);
        }
        // 已确认额定转速的型号：限值不得高于额定（与 effort 峰值钳制同口径，
        // 不允许把驱动限值抬到型号标定之上）。
        const float rated = entry->profile->rated_speed_rad_s;
        if (rated > 0.0f && clamped.max_velocity_rad_s > rated) {
          clamped.max_velocity_rad_s = rated;
        }
      } else {
        if (!isfinite(request->max_torque_nm) || request->max_torque_nm <= 0.0f) {
          return reject_invalid(ctx, entry, request, now_ms);
        }
        const float peak = entry->profile->peak_torque_nm;
        if (peak > 0.0f && clamped.max_torque_nm > peak) {
          clamped.max_torque_nm = peak;
        }
        // 硬件参数表第二条：设备侧 0x2538 是**电气**上限，外部写入不得把设备里
        // 的电流推到规格书峰值电流之上。两条上限取更严者——今天两者等价
        // （限值系数就是峰值对），但任一列被改动后这条规则仍然成立。
        const float peak_amp = entry->profile->peak_current_amp;
        if (peak_amp > 0.0f) {
          const float cap_nm =
              peak_amp * motor_protocol_profile_limit_nm_per_amp(entry->profile);
          if (cap_nm > 0.0f && clamped.max_torque_nm > cap_nm) {
            clamped.max_torque_nm = cap_nm;
          }
        }
      }
      // 软件限位型号（VAXOR 无厂家管理对象）：本地保存，打包前钳制
      // （ARD §26.1，与位置软限位同口径）。只影响目标、不触发驱动器保护，
      // 故与增益一样**无生命周期门禁**——门禁只针对设备侧写。
      if ((entry->profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
        if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT) {
          entry->soft_max_velocity_rad_s = clamped.max_velocity_rad_s;
          entry->has_soft_vel_limit = true;
        } else {
          entry->soft_max_torque_nm = clamped.max_torque_nm;
          entry->has_soft_torque_limit = true;
        }
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
        return true;
      }
      // 设备侧（CERB）：0x6080 / 0x2538，SDO 写 + 读回核验；仅未运动时允许。
      if (!device_limit_write_allowed(entry)) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      return start_tx(ctx, entry, &clamped, now_ms);
    }
    case MOTOR_PROTOCOL_CONFIG_SET_ZERO: {
      // 设零点（字典 0x2531 写 1）：厂家要求**失能状态**下设置——已使能一律
      // 拒绝，不隐式失能（与设备侧限位同口径：安全边界不得在运动中变动）。
      // 命令式事务：只等写应答，不读回；也不更新设备影子/记录（零点不是本机
      // 记录的参数，它改变的是设备的位置基准）。
      // 注意：零点一变，设备里 0x607D 的限位也随之换了物理位置，操作者需要
      // 重新确认限位与机械行程（见 docs 决策日志）。
      if (entry->profile == NULL || !device_limit_write_allowed(entry)) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      return start_tx(ctx, entry, request, now_ms);
    }
    case MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT: {
      // 故障清除：只清「可清的」。三类故障的判据是**成因**（unified_fault_code）
      // 而不是生命周期——DEV_PARAM 与身份冲突都会把表项带进 CONFIG_CONFLICT，
      // 按生命周期判会把两者混为一谈。
      //  - 身份冲突锁存（latch_config_conflict）：ARD §9.3 刻意不可解除，复位
      //    只能整机重启 → 拒绝。
      //  - 设备参数未初始化（DEV_PARAM）：改走一次 REFRESH_PARAMS 事务，重跑
      //    回读与比对。真不一致仍然由 apply_dev_init_result 判 MISMATCH 保持
      //    CONFIG_CONFLICT——清除只负责「允许重走授权链」，不伪造一致。
      //  - 厂家故障：发起设备侧清错事务（协议无手段则 UNSUPPORTED）。设备清掉
      //    后下一帧状态帧走既有 FAULT → READY_DISABLED 路径收敛。
      // 清错帧是控制类帧（MIT 控制字），运动中的电机不得收到——与设备侧限位
      // 写同口径：安全边界不在运动中变动。
      if (entry->profile == NULL || !device_limit_write_allowed(entry)) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_CONFIG_CONFLICT) {
        return reject_invalid(ctx, entry, request, now_ms); // 身份冲突不可解除
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM) {
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
        // 设备参数初始化未完成（未回读/回读失败/与记录不一致）：使能请求直接
        // 拒绝并说明原因，不静默吞掉——页面要能立刻看到"为什么按不下去"。
        if (!motor_control_dev_params_allow_motion(entry)) {
          entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
          return reject_invalid(ctx, entry, request, now_ms);
        }
        entry->policy_enable = true;
        entry->motion_authorized = true;
      } else {
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
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                               entry->motor_id, entry->bus_index,
                               MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
      return true;
    }
    case MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS: {
      // 设备参数回读：换算系数来自 Profile（未绑定拒绝）；上传只读，
      // ACTIVE 态也允许（不加 device_limit_write_allowed 门禁）。
      if (entry->profile == NULL) {
        return reject_invalid(ctx, entry, request, now_ms);
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
