// motor_control v2 内核主体：初始化、会话/授权、10ms 控制边界、1ms 服务、
// TX 收集与状态/事件副本（ARD-0007 §8/§9/§11/§13；HANDOFF 阶段 A）。
//
// 所有权与边界：
//  - 本模块不依赖 ROS / HAL / FreeRTOS / control_mailbox（HANDOFF 阶段 A-3）；
//  - 厂家字节只经 motor_protocol 的 adapter 接口（ARD §24）；
//  - 一切时序判定用 MCU 单调时基毫秒（ARD §12），禁止 ROS 时间。
//
// 安全实现要点（对抗性审查后修订）：
//  - P0 帧（软停/失能）推送失败不静默丢弃：先驱逐最旧非 P0 帧腾位（§22.1
//    抢占），仍失败挂 p0_pending 由 service 逐节拍重发，新使能/MIT 成功后作废；
//  - 受控集合 = ACTIVE ∪ READY_DISABLED(已使能)（§13.1 DERIVED：使能整定窗内
//    的电机已在受控集合，缺席/30ms 级联同样必须停它）；
//  - apply 前置 staleness 门（§51.3）：age>30ms 不下发目标、age>100ms 撤权，
//    过期快照不记录、不复位级联去重标志。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// 内部工具
// ---------------------------------------------------------------------------

void motor_control_push_event(motor_control_context_t *ctx, uint32_t now_ms,
                              motor_control_event_type_t type, uint16_t motor_id,
                              uint8_t bus_index, uint16_t reason,
                              uint32_t request_id) {
  if (ctx == NULL) {
    return;
  }
  if (ctx->event_count >= MOTOR_CONTROL_MAX_EVENTS) {
    // 环满：丢最旧、保最新，丢弃计数可见（不静默）。
    memmove(&ctx->events[0], &ctx->events[1],
            (MOTOR_CONTROL_MAX_EVENTS - 1U) * sizeof(ctx->events[0]));
    ctx->event_count = MOTOR_CONTROL_MAX_EVENTS - 1U;
    ctx->event_dropped_count++;
  }
  motor_control_event_t *event = &ctx->events[ctx->event_count++];
  event->type = type;
  event->tick_ms = now_ms; // 事件携带 MCU 单调时序（ARD §18.2）
  event->motor_id = motor_id;
  event->bus_index = bus_index;
  event->reason = reason;
  event->request_id = request_id;
}

bool motor_control_push_tx(motor_control_context_t *ctx,
                           const motor_protocol_tx_frame_t *frame) {
  if (ctx == NULL || frame == NULL) {
    return false;
  }
  if (ctx->tx_count >= MOTOR_CONTROL_MAX_TX_PENDING) {
    ctx->tx_dropped_count++;
    return false;
  }
  ctx->tx_pending[ctx->tx_count++] = *frame;
  return true;
}

bool motor_control_push_tx_p0(motor_control_context_t *ctx,
                              motor_control_entry_t *entry,
                              const motor_protocol_tx_frame_t *frame) {
  if (ctx == NULL || frame == NULL) {
    return false;
  }
  if (ctx->tx_count < MOTOR_CONTROL_MAX_TX_PENDING) {
    ctx->tx_pending[ctx->tx_count++] = *frame;
    entry->p0_pending = false;
    entry->p0_kind = MOTOR_CONTROL_P0_NONE;
    return true;
  }
  // 满：驱逐最旧的非 P0 帧腾位（P0 抢占，ARD §22.1——被驱逐帧计入丢弃计数）。
  for (size_t i = 0; i < ctx->tx_count; ++i) {
    if (ctx->tx_pending[i].tx_class != MOTOR_PROTOCOL_TX_CLASS_P0) {
      memmove(&ctx->tx_pending[i], &ctx->tx_pending[i + 1],
              (ctx->tx_count - i - 1U) * sizeof(ctx->tx_pending[0]));
      ctx->tx_count--;
      ctx->tx_dropped_count++;
      ctx->tx_pending[ctx->tx_count++] = *frame;
      entry->p0_pending = false;
      entry->p0_kind = MOTOR_CONTROL_P0_NONE;
      return true;
    }
  }
  // 全是 P0（病态）：不丢任何安全帧——挂起重试，service 逐节拍补发。
  ctx->p0_retry_count++;
  return false;
}

motor_control_entry_t *motor_control_find_entry(motor_control_context_t *ctx,
                                                uint16_t motor_id) {
  if (ctx == NULL) {
    return NULL;
  }
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    if (ctx->entries[i].used && ctx->entries[i].motor_id == motor_id) {
      return &ctx->entries[i];
    }
  }
  return NULL;
}

// uint32 毫秒回绕安全比较：now 是否已超过 then 至少 threshold。
static bool tick_elapsed(uint32_t now, uint32_t then, uint32_t threshold) {
  return (uint32_t)(now - then) >= threshold;
}

// 统一 P0 失能帧生成 + 推送（撤权/离线/故障/冲突共用）。
static bool send_disable_p0(motor_control_context_t *ctx,
                            motor_control_entry_t *entry) {
  if (entry->adapter == NULL || entry->adapter->encode_disable == NULL) {
    return false;
  }
  motor_protocol_tx_frame_t frame = {0};
  if (!entry->adapter->encode_disable(entry->profile, entry->node,
                                      entry->bus_index, &frame)) {
    return false;
  }
  frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  if (motor_control_push_tx_p0(ctx, entry, &frame)) {
    return true;
  }
  entry->p0_pending = true;
  entry->p0_kind = MOTOR_CONTROL_P0_DISABLE;
  return false;
}

// ---------------------------------------------------------------------------
// 生命周期管理
// ---------------------------------------------------------------------------

void motor_control_init(motor_control_context_t *ctx,
                        const motor_protocol_adapter_t *const *adapters,
                        size_t adapter_count) {
  if (ctx == NULL) {
    return;
  }
  memset(ctx, 0, sizeof(*ctx));
  ctx->adapters = adapters;
  ctx->adapter_count = adapters != NULL ? adapter_count : 0U;
  // 安全默认：未授权、无快照。任务层启动时把 mailbox 的授权电平转发进来。
  ctx->authorized = false;
  ctx->require_new_snapshot = false;
}

// 撤销运动授权的统一安全路径（ARD §13.2/§13.3/§33）。
void motor_control_revoke_motion(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    if (entry->motion_enabled) {
      // 推送失败不静默：p0_pending 挂起，service 逐节拍重发失能帧。
      (void)send_disable_p0(ctx, entry);
    }
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->has_target = false;
    entry->motion_authorized = false;
    entry->tx_active = false; // 断链清配置事务（mailbox 同步清 config FIFO）
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DISABLE,
                               entry->motor_id, entry->bus_index,
                               MOTOR_CONTROL_FAULT_NONE, 0U);
    }
  }
  ctx->authorized = false;
  ctx->require_new_snapshot = true; // 恢复必须等新的完整快照（禁止复用旧目标）
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST,
                           0U, 0U, MOTOR_CONTROL_FAULT_NONE, 0U);
}

void motor_control_set_agent_connected(motor_control_context_t *ctx, bool connected,
                                       uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }
  if (connected) {
    ctx->agent_connected = true;
    ctx->agent_ever_connected = true;
    return;
  }
  ctx->agent_connected = false;
  if (ctx->agent_ever_connected) {
    // 曾连接后断链：立即协议级停机/失能并清授权，不等 100ms（ARD §13.2）。
    motor_control_revoke_motion(ctx, now_ms);
  }
}

bool motor_control_set_authorization(motor_control_context_t *ctx, bool authorized,
                                     uint32_t now_ms) {
  if (ctx == NULL) {
    return false;
  }
  if (authorized) {
    if (ctx->agent_ever_connected && !ctx->agent_connected) {
      return false; // 断链期间不得授权
    }
    ctx->authorized = true;
    // 全局 Enable：重立全部表项的运动授权（逐电机策略位不受影响；
    // 离线冻结的 policy_enable 仍需 MotorPolicy 显式重发——ARD §36）。
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      if (ctx->entries[i].used) {
        ctx->entries[i].motion_authorized = true;
      }
    }
    return true;
  }
  ctx->authorized = false;
  motor_control_revoke_motion(ctx, now_ms);
  return true;
}

bool motor_control_bind_profile(motor_control_context_t *ctx, uint16_t motor_id,
                                size_t profile_index, uint32_t now_ms) {
  if (ctx == NULL || profile_index >= motor_control_profile_count()) {
    return false;
  }
  return motor_control_bind_profile_ptr(ctx, motor_id,
                                        motor_control_profile_at(profile_index),
                                        now_ms);
}

bool motor_control_bind_profile_ptr(motor_control_context_t *ctx, uint16_t motor_id,
                                    const motor_protocol_model_profile_t *profile,
                                    uint32_t now_ms) {
  if (ctx == NULL || profile == NULL) {
    return false;
  }
  motor_control_entry_t *entry = motor_control_find_entry(ctx, motor_id);
  if (entry == NULL) {
    return false;
  }
  // 门禁：只允许未定型的发现/识别/就绪态换绑；运行中（ACTIVE/SAFE_STOP）与
  // 异常态（FAULT/OFFLINE/CONFIG_CONFLICT）一律拒绝，防止运动中换算比突变。
  if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
      entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
      entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
                             motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
    return false;
  }
  if (profile->protocol != entry->adapter) {
    // 协议身份不符：按能力冲突拒绝，不得绑定（ARD §9.3）。
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
                             motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
    return false;
  }
  entry->profile = profile;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_BOUND, motor_id,
                           entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  // 身份确认完成 → 识别态收敛到 READY_DISABLED（ARD §9：DISCOVERED→IDENTIFYING
  // →READY_DISABLED；绑定即识别来源，Vaxor-80 不在目录 → 永远无法到这）。
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING) {
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_IDENTIFYING;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY, motor_id,
                             entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 10ms 控制边界（ARD §11.3 / §51.3）
// ---------------------------------------------------------------------------

static bool snapshot_contains(const motor_control_command_snapshot_t *snapshot,
                              uint16_t motor_id, const motor_control_snapshot_target_t **out) {
  for (uint8_t i = 0; i < snapshot->target_count; ++i) {
    if (snapshot->targets[i].motor_id == motor_id) {
      *out = &snapshot->targets[i];
      return true;
    }
  }
  return false;
}

bool motor_control_apply_command_snapshot(
    motor_control_context_t *ctx, const motor_control_command_snapshot_t *snapshot,
    uint32_t now_ms) {
  if (ctx == NULL || snapshot == NULL) {
    return false;
  }

  // 校验（ARD §11.3 步骤 2-6 / §15.5）：非法快照整体拒绝，绝不部分采纳。
  bool rejected = false;
  uint16_t reject_reason = MOTOR_CONTROL_FAULT_NONE;
  if (snapshot->target_count > MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS) {
    rejected = true;
    reject_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
  }
  for (uint8_t i = 0; !rejected && i < snapshot->target_count; ++i) {
    const motor_control_snapshot_target_t *t = &snapshot->targets[i];
    if (!isfinite(t->position_rad) || !isfinite(t->velocity_rad_s) ||
        !isfinite(t->effort_nm)) {
      rejected = true; // 命令侧禁 NaN/Inf（TBD-ROS-01 当前取向：拒绝）
      reject_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
      break;
    }
    if (motor_control_find_entry(ctx, t->motor_id) == NULL) {
      rejected = true; // 不存在/未注册设备（ARD §15.5）
      reject_reason = MOTOR_CONTROL_FAULT_CONFIG_CONFLICT;
      break;
    }
    for (uint8_t j = (uint8_t)(i + 1U); j < snapshot->target_count; ++j) {
      if (snapshot->targets[j].motor_id == t->motor_id) {
        rejected = true; // 重复 motor_id
        reject_reason = MOTOR_CONTROL_FAULT_CONFIG_CONFLICT;
        break;
      }
    }
  }
  // 序号必须严格单调（回绕安全，ARD §11.3 步骤 2）。
  if (!rejected && ctx->have_snapshot &&
      (int32_t)(snapshot->sequence - ctx->last_snapshot_sequence) <= 0) {
    rejected = true;
    reject_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
  }
  if (rejected) {
    ctx->snapshot_rejected_count++;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_SNAPSHOT_REJECTED,
                             0U, 0U, reject_reason, 0U);
    return false;
  }

  // staleness 门（ARD §51.3：先判定会话/100ms/30ms，再做逐电机处理）。
  // 过期快照触发对应安全动作但不记录、不复位级联去重标志——它不是"新鲜证据"。
  const uint32_t snapshot_age = now_ms - snapshot->received_tick_ms;
  if (snapshot_age > MOTOR_CONTROL_SNAPSHOT_DISABLE_MS) {
    motor_control_revoke_motion(ctx, now_ms);
    ctx->snapshot_disable_done = true;
    ctx->snapshot_soft_stop_active = true;
    return true;
  }
  if (snapshot_age > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      motor_control_soft_stop_entry(ctx, &ctx->entries[i], now_ms);
    }
    ctx->snapshot_soft_stop_active = true;
    return true;
  }

  // 接受：freshness 基准 = mailbox 记录的本地接收 tick（ARD §12.2）。
  ctx->have_snapshot = true;
  ctx->last_snapshot_tick_ms = snapshot->received_tick_ms;
  ctx->last_snapshot_sequence = snapshot->sequence;
  ctx->require_new_snapshot = false;
  ctx->snapshot_soft_stop_active = false;
  ctx->snapshot_disable_done = false;
  ctx->snapshot_accepted_count++;

  // 受控集合逐台处理（ARD §11.3 步骤 10-12 / §51.3）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    const bool controlled = entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
                            entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP;
    const bool eligible_state =
        controlled || entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    if (!eligible_state) {
      continue;
    }

    const motor_control_snapshot_target_t *target = NULL;
    const bool present = snapshot_contains(snapshot, entry->motor_id, &target);
    if (!present) {
      // 受控集合缺席 → 软停（ARD §13.1，禁止沿用旧目标）。
      motor_control_soft_stop_entry(ctx, entry, now_ms);
      continue;
    }
    if (!ctx->authorized || ctx->require_new_snapshot) {
      continue; // 无授权/待新快照：不得产生运动帧
    }
    if (!entry->policy_enable) {
      continue; // 逐电机 Enable 关闭（MotorPolicy；恢复须显式 SET_ENABLE(true)）
    }
    if (!entry->motion_authorized) {
      continue; // 逐电机授权位已清（故障/离线/SET_ENABLE(false)）：不得自动恢复，
                // 必须显式 SET_ENABLE(true) 复立（ARD §36，禁止项"失败不自动恢复"）
    }
    if (entry->profile == NULL) {
      continue; // 未绑定型号：不可控
    }
    if (entry->tx_active && entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
      continue; // 模式事务独占该电机的使能序列（分阶段、不阻塞）
    }

    // Policy 软件限位（Vaxor，打包前钳制——ARD §26.1）；限位标志逐快照重算，
    // 不粘滞（目标回到限内即清除）。
    float target_pos = target->position_rad;
    float target_effort = target->effort_nm;
    const motor_protocol_model_profile_t *profile = entry->profile;
    entry->limit_flags &= ~(MOTOR_PROTOCOL_LIMIT_POS_MIN | MOTOR_PROTOCOL_LIMIT_POS_MAX |
                            MOTOR_PROTOCOL_LIMIT_EFFORT);
    if (entry->has_soft_limits &&
        (profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
      if (target_pos < entry->soft_min_rad) {
        target_pos = entry->soft_min_rad;
        entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_POS_MIN;
      } else if (target_pos > entry->soft_max_rad) {
        target_pos = entry->soft_max_rad;
        entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_POS_MAX;
      }
    }
    // Profile 峰值钳制（已确认型号才有峰值；0 = 未确认不钳）。
    if (profile->peak_torque_nm > 0.0f) {
      if (target_effort > profile->peak_torque_nm) {
        target_effort = profile->peak_torque_nm;
        entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_EFFORT;
      } else if (target_effort < -profile->peak_torque_nm) {
        target_effort = -profile->peak_torque_nm;
        entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_EFFORT;
      }
    }

    entry->has_target = true;
    entry->target_position_rad = target_pos;
    entry->target_velocity_rad_s = target->velocity_rad_s;
    entry->target_effort_nm = target_effort;
    // motion_authorized 不因快照出现而自动复立：撤销后只能由
    // set_authorization(true) 或 SET_ENABLE(true) 显式重立（ARD §36）。

    // 使能时序：未使能先发使能帧；有整定门的协议在本窗内不发运动目标。
    if (!entry->motion_enabled) {
      motor_protocol_tx_frame_t frame = {0};
      if (entry->adapter->encode_enable == NULL ||
          !entry->adapter->encode_enable(profile, entry->node, entry->bus_index,
                                         &frame)) {
        entry->protocol_error_count++;
        entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
        continue;
      }
      frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
      frame.tx_slot = (uint16_t)i; // P1 latest-wins 槽 = 表项索引（任务层路由用）
      if (!motor_control_push_tx(ctx, &frame)) {
        continue;
      }
      entry->motion_enabled = true;
      entry->last_commanded_tick_ms = now_ms;
      if (entry->adapter->enable_settle_ms > 0U) {
        entry->enable_pending = true;
        entry->enable_started_tick_ms = now_ms;
        continue;
      }
    }
    if (entry->enable_pending) {
      continue; // 整定窗内（service 到期后恢复）
    }

    motor_protocol_control_intent_t intent = {0};
    intent.position_rad = entry->target_position_rad;
    intent.velocity_rad_s = entry->target_velocity_rad_s;
    intent.effort_nm = entry->target_effort_nm;
    intent.kp = entry->has_kp_kd ? entry->kp : profile->default_kp;
    intent.kd = entry->has_kp_kd ? entry->kd : profile->default_kd;
    intent.enable = true;
    motor_protocol_tx_frame_t frame = {0};
    if (entry->adapter->encode_mit_command == NULL ||
        !entry->adapter->encode_mit_command(profile, entry->node, entry->bus_index,
                                            &intent, &frame)) {
      entry->protocol_error_count++;
      entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
      continue;
    }
    frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
    frame.tx_slot = (uint16_t)i; // P1 latest-wins 槽 = 表项索引（任务层路由用）
    if (motor_control_push_tx(ctx, &frame)) {
      // 新运动命令成功入队：过期的 P0 重试帧作废（新命令取代旧安全帧）。
      entry->p0_pending = false;
      entry->p0_kind = MOTOR_CONTROL_P0_NONE;
      entry->last_commanded_tick_ms = now_ms;
      if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE) {
        entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_ACTIVE,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_CONTROL_FAULT_NONE, 0U);
      }
    }
  }
  return true;
}

void motor_control_soft_stop_entry(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry, uint32_t now_ms) {
  if (entry == NULL || !entry->used) {
    return;
  }
  // 受控集合（ARD §13.1 DERIVED）：ACTIVE，或已使能但尚在使能序列中的
  // READY_DISABLED（CREB 使能整定窗内的电机同样必须被软停覆盖）。
  const bool in_controlled_set =
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
       entry->motion_enabled);
  if (in_controlled_set) {
    // 仅在进入软停的迁移沿发帧（去重，防 100Hz P0 刷屏）。
    if (entry->adapter != NULL && entry->profile != NULL &&
        entry->adapter->encode_soft_stop != NULL) {
      motor_protocol_tx_frame_t frame = {0};
      if (entry->adapter->encode_soft_stop(entry->profile, entry->node,
                                           entry->bus_index, &frame)) {
        frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
        if (!motor_control_push_tx_p0(ctx, entry, &frame)) {
          entry->p0_pending = true;
          entry->p0_kind = MOTOR_CONTROL_P0_SOFT_STOP;
        }
      } else {
        // 编码失败（Profile 缺失等病态）：同样挂起重试——安全帧不静默消失；
        // 重试仍失败时由 service 计数并放弃（与失能路径同语义）。
        entry->p0_pending = true;
        entry->p0_kind = MOTOR_CONTROL_P0_SOFT_STOP;
        entry->protocol_error_count++;
      }
    }
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_SAFE_STOP;
    entry->has_target = false;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_SOFT_STOP,
                             entry->motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_NONE, 0U);
  }
}

// ---------------------------------------------------------------------------
// 1ms 服务节拍（ARD §11.2 / §51.2 的运行时侧）
// ---------------------------------------------------------------------------

static void service_lifecycle_advance(motor_control_context_t *ctx, uint32_t now_ms) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    // 重连后的识别流（ARD §36）：DISCOVERED → IDENTIFYING；身份此前已确认且
    // Profile 仍在 → 识别立即完成，收敛 READY_DISABLED。
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->profile != NULL) {
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY,
                               entry->motor_id, entry->bus_index,
                               MOTOR_CONTROL_FAULT_NONE, 0U);
    }
  }
}

static void service_p0_retries(motor_control_context_t *ctx) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->p0_pending) {
      continue;
    }
    motor_protocol_tx_frame_t frame = {0};
    bool encoded = false;
    if (entry->p0_kind == MOTOR_CONTROL_P0_SOFT_STOP &&
        entry->adapter != NULL && entry->profile != NULL &&
        entry->adapter->encode_soft_stop != NULL) {
      encoded = entry->adapter->encode_soft_stop(entry->profile, entry->node,
                                                 entry->bus_index, &frame);
    } else if (entry->p0_kind == MOTOR_CONTROL_P0_DISABLE &&
               entry->adapter != NULL && entry->adapter->encode_disable != NULL) {
      encoded = entry->adapter->encode_disable(entry->profile, entry->node,
                                               entry->bus_index, &frame);
    } else {
      encoded = false;
    }
    if (!encoded) {
      entry->p0_pending = false; // 编码不了的安全帧不再重试（计数可见）
      entry->p0_kind = MOTOR_CONTROL_P0_NONE;
      entry->protocol_error_count++;
      continue;
    }
    frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
    if (motor_control_push_tx(ctx, &frame)) {
      entry->p0_pending = false;
      entry->p0_kind = MOTOR_CONTROL_P0_NONE;
    }
    // 仍满：保持挂起，下一节拍继续（不静默丢安全帧）。
  }
}

static void service_offline_checks(motor_control_context_t *ctx, uint32_t now_ms) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE) {
      continue;
    }
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
      // 冲突锁存不降级 OFFLINE：冲突态本就禁运动，且 §9.3 禁止任何
      // "离线→重连"周期被用来自动解除冲突。
      continue;
    }
    const motor_protocol_adapter_t *adapter = entry->adapter;
    if (adapter == NULL) {
      continue;
    }
    // OFFLINE 判定（2026-09-05 协议核实）：两协议均有与命令无关的 20ms 状态
    // 心跳（vaxor 20B 状态帧 / CREB 0x080 SYNC 上报帧），承担总线检测——
    // 心跳龄超阈值即离线。非自主心跳协议仅在"正在发命令却持续无反馈"时判定
    // （非受控期沉默是常态）。
    bool offline = false;
    if (adapter->autonomous_feedback) {
      offline = tick_elapsed(now_ms, entry->last_rx_tick_ms,
                             adapter->offline_timeout_ms);
    } else if (entry->last_commanded_tick_ms != 0U &&
               !tick_elapsed(now_ms, entry->last_commanded_tick_ms,
                             adapter->offline_timeout_ms) &&
               tick_elapsed(now_ms, entry->last_rx_tick_ms,
                            adapter->offline_timeout_ms)) {
      offline = true;
    }
    if (!offline) {
      continue;
    }
    // 撤销该电机的运动帧输出：正在发命令的设备必须先收到失能帧（§36）。
    if (entry->motion_enabled) {
      (void)send_disable_p0(ctx, entry);
    }
    entry->online = false;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_OFFLINE;
    entry->has_target = false;
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->motion_authorized = false; // 逐电机授权冻结（§36）
    entry->policy_enable = false;     // 冻结为 false：恢复需显式重发 Enable
    entry->tx_active = false;
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_OFFLINE;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_OFFLINE;
    ctx->require_new_snapshot = true; // 重连后旧授权/目标不复用（ARD §10.4）
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_OFFLINE,
                             entry->motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_OFFLINE, 0U);
  }
}

static void service_snapshot_freshness(motor_control_context_t *ctx, uint32_t now_ms) {
  if (!ctx->have_snapshot) {
    return;
  }
  const uint32_t age = now_ms - ctx->last_snapshot_tick_ms;
  if (age <= MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    ctx->snapshot_soft_stop_active = false;
    ctx->snapshot_disable_done = false;
    return;
  }
  if (age > MOTOR_CONTROL_SNAPSHOT_DISABLE_MS && !ctx->snapshot_disable_done) {
    // 100ms：协议级停机/失能 + 清运动授权（ARD §13.2，一次性升级）。
    motor_control_revoke_motion(ctx, now_ms);
    ctx->snapshot_disable_done = true;
    ctx->snapshot_soft_stop_active = true;
    return;
  }
  if (age > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS && !ctx->snapshot_soft_stop_active &&
      !ctx->snapshot_disable_done) {
    // 30ms：全体受控软停（一次性，直到新快照）。
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      motor_control_soft_stop_entry(ctx, &ctx->entries[i], now_ms);
    }
    ctx->snapshot_soft_stop_active = true;
  }
}

void motor_control_service(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }
  service_offline_checks(ctx, now_ms);
  service_snapshot_freshness(ctx, now_ms);
  service_lifecycle_advance(ctx, now_ms);
  service_p0_retries(ctx);

  // 使能整定到期（vendor 门，adapter 常量）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (entry->used && entry->enable_pending && entry->adapter != NULL &&
        tick_elapsed(now_ms, entry->enable_started_tick_ms,
                     entry->adapter->enable_settle_ms)) {
      entry->enable_pending = false;
    }
  }

  motor_control_config_step(ctx, now_ms);
}

// ---------------------------------------------------------------------------
// 发现探测与输出副本
// ---------------------------------------------------------------------------

bool motor_control_probe_discovery(motor_control_context_t *ctx, uint8_t bus_index) {
  if (ctx == NULL) {
    return false;
  }
  bool pushed = false;
  for (size_t i = 0; i < ctx->adapter_count; ++i) {
    const motor_protocol_adapter_t *adapter = ctx->adapters[i];
    if (adapter == NULL || adapter->encode_discovery_sync == NULL) {
      continue; // vaxor 靠自主上报，无探测帧
    }
    motor_protocol_tx_frame_t frame = {0};
    if (adapter->encode_discovery_sync(bus_index, &frame)) {
      pushed = motor_control_push_tx(ctx, &frame) || pushed;
    }
  }
  return pushed;
}

size_t motor_control_collect_tx(motor_control_context_t *ctx,
                                motor_protocol_tx_frame_t *out, size_t capacity) {
  if (ctx == NULL || out == NULL || capacity == 0U) {
    return 0U;
  }
  const size_t count = ctx->tx_count < capacity ? ctx->tx_count : capacity;
  memcpy(out, ctx->tx_pending, count * sizeof(ctx->tx_pending[0]));
  const size_t remainder = ctx->tx_count - count;
  if (remainder > 0U) {
    memmove(&ctx->tx_pending[0], &ctx->tx_pending[count],
            remainder * sizeof(ctx->tx_pending[0]));
  }
  ctx->tx_count = remainder;
  return count;
}

size_t motor_control_copy_motion_states(motor_control_context_t *ctx,
                                        motor_control_motion_state_t *out,
                                        size_t capacity, uint32_t now_ms) {
  if (ctx == NULL || out == NULL) {
    return 0U;
  }
  size_t count = 0U;
  for (size_t i = 0; i < ctx->entry_count && count < capacity; ++i) {
    const motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    motor_control_motion_state_t *state = &out[count++];
    state->motor_id = entry->motor_id;
    state->bus_index = entry->bus_index;
    state->lifecycle = entry->lifecycle;
    state->online = entry->online;
    state->position_rad = entry->has_measured ? entry->measured.position_rad : NAN;
    state->velocity_rad_s =
        entry->has_measured ? entry->measured.velocity_rad_s : NAN;
    state->effort_nm = entry->has_measured ? entry->measured.effort_nm : NAN;
    state->state_age_ms = entry->last_motion_rx_tick_ms != 0U
                              ? now_ms - entry->last_motion_rx_tick_ms
                              : 0xFFFFFFFFU;
  }
  return count;
}

size_t motor_control_copy_statuses(motor_control_context_t *ctx,
                                   motor_control_status_t *out, size_t capacity,
                                   uint32_t now_ms) {
  if (ctx == NULL || out == NULL) {
    return 0U;
  }
  size_t count = 0U;
  for (size_t i = 0; i < ctx->entry_count && count < capacity; ++i) {
    const motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used) {
      continue;
    }
    motor_control_status_t *status = &out[count++];
    status->motor_id = entry->motor_id;
    status->bus_index = entry->bus_index;
    status->lifecycle = entry->lifecycle;
    status->online = entry->online;
    status->temperature_c =
        entry->has_measured ? entry->measured.temperature_c : NAN;
    status->current_a = entry->has_measured ? entry->measured.current_a : NAN;
    status->voltage_v = entry->has_measured ? entry->measured.voltage_v : NAN;
    status->drive_mode = entry->has_measured ? entry->measured.drive_mode : 0U;
    status->unified_fault_code = entry->unified_fault_code;
    status->vendor_fault_code = entry->vendor_fault_code;
    status->protection_flags = entry->protection_flags;
    status->limit_flags = entry->limit_flags;
    status->state_age_ms = entry->last_motion_rx_tick_ms != 0U
                               ? now_ms - entry->last_motion_rx_tick_ms
                               : 0xFFFFFFFFU;
    status->transaction_state = entry->tx_active ? 1U : 0U;
    status->last_failure_reason = entry->last_failure_reason;
  }
  return count;
}

size_t motor_control_drain_events(motor_control_context_t *ctx,
                                  motor_control_event_t *out, size_t capacity) {
  if (ctx == NULL || out == NULL || capacity == 0U) {
    return 0U;
  }
  const size_t count = ctx->event_count < capacity ? ctx->event_count : capacity;
  memcpy(out, ctx->events, count * sizeof(ctx->events[0]));
  const size_t remainder = ctx->event_count - count;
  if (remainder > 0U) {
    memmove(&ctx->events[0], &ctx->events[count],
            remainder * sizeof(ctx->events[0]));
  }
  ctx->event_count = remainder;
  return count;
}
