// motor_control v2 内核主体：初始化、会话/授权、20ms 控制边界、1ms 服务、
// TX 收集与状态/事件副本（ARD-0007 §8/§9/§11/§13）。
//
// 所有权与边界：
//  - 本模块不依赖 ROS / HAL / FreeRTOS / control_mailbox；
//  - 厂家字节只经 motor_protocol 的 adapter 接口（ARD §24）；
//  - 一切时序判定用 MCU 单调时基毫秒（ARD §12），禁止 ROS 时间。
//
// 安全实现要点：
//  - P0 帧（软停/失能）推送失败不静默丢弃：先驱逐最旧非 P0 帧腾位（§22.1
//    抢占），仍失败挂 p0_pending 由 service 逐节拍重发；新 MIT 或使能帧
//    成功出队后作废——过期软停/失能重试帧不得晚于新安全意图出门；
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
  event->seq = ++ctx->event_seq;
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
  // 重试类别取 last_p0_kind（调用方推送前必须已设置），故各调用方不再
  // 各自复制「失败则置 p0_pending/p0_kind」的尾巴。
  ctx->p0_retry_count++;
  entry->p0_pending = true;
  entry->p0_kind = entry->last_p0_kind;
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
// 时间差判定：**带符号**比较，不是无符号。
// 帧的时戳由 ISR 盖，服务用任务里早先采样的 now——drain 期间跨过 1ms 边界
// 时会出现"帧戳 = now + 1"。无符号减法把这种"戳在未来"算成 0xFFFFFFFF，
// 于是 OFFLINE（差值 ≥ 100ms 阈值）立刻成立：该电机无端被判离线、清逐电机
// 授权、事务被判 UNCONFIRMED。带符号差值把它算成 -1ms = 刚到的新鲜帧，
// 语义正确；回绕安全性不变（只要两侧间隔 < 2^31 ms ≈ 24 天）。
static bool tick_elapsed(uint32_t now, uint32_t then, uint32_t threshold) {
  return (int32_t)(now - then) >= (int32_t)threshold;
}

// 下游（调度器 P0 FIFO 满 / 硬件拒收）回告重入队：帧由本内核
// 生成（tx_slot = 表项索引），任务层原样回传即可定位表项。
bool motor_control_requeue_p0(motor_control_context_t *ctx,
                              const motor_protocol_tx_frame_t *frame) {
  if (ctx == NULL || frame == NULL ||
      frame->tx_class != MOTOR_PROTOCOL_TX_CLASS_P0 ||
      frame->tx_slot >= ctx->entry_count) {
    return false;
  }
  motor_control_entry_t *entry = &ctx->entries[frame->tx_slot];
  if (!entry->used) {
    return false;
  }
  // 成功即重入队（含驱逐腾位路径）；失败（待发队列全为 P0 的病态）已由
  // push_tx_p0 转挂起重试，service_p0_retries 按 last_p0_kind 重新编码
  // 补发——不静默丢安全帧。
  return motor_control_push_tx_p0(ctx, entry, frame);
}

// 统一 P0 安全帧生成 + 推送（失能/软停共用）。撤权、离线、故障、冲突、
// 软停五条路径全部经此，不再各自复制编码与「失败挂起」逻辑。
//
// kind 会被记入 last_p0_kind：下游（TX 调度器/硬件）拒收触发 requeue_p0 时
// 据此重新编码补发，故每次推送前必须设置。此前三处外部调用漏设该字段，
// 会把一帧失能被按残留类别（可能已是 ENABLE）补发。
//
// 编码失败与队列满都不静默丢安全帧：挂 p0_pending 交 service 逐节拍重试
// 并计入 protocol_error_count（与 service_p0_retries 同一语义）。
bool motor_control_send_p0(motor_control_context_t *ctx,
                           motor_control_entry_t *entry, uint8_t kind) {
  if (ctx == NULL || entry == NULL || entry->adapter == NULL) {
    return false;
  }
  motor_protocol_tx_frame_t frame = {0};
  bool encoded = false;
  if (kind == MOTOR_CONTROL_P0_DISABLE) {
    encoded = entry->adapter->encode_disable != NULL &&
              entry->adapter->encode_disable(entry->profile, entry->node,
                                             entry->bus_index, &frame);
  } else if (kind == MOTOR_CONTROL_P0_SOFT_STOP) {
    // 软停编码需要 Profile 的力矩/限位量，缺失即视为编码失败。
    encoded = entry->profile != NULL &&
              entry->adapter->encode_soft_stop != NULL &&
              entry->adapter->encode_soft_stop(entry->profile, entry->node,
                                               entry->bus_index, &frame);
  }
  if (!encoded) {
    entry->p0_pending = true;
    entry->p0_kind = kind;
    entry->protocol_error_count++;
    return false;
  }
  frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  frame.tx_slot = (uint16_t)motor_control_entry_index(ctx, entry);
  entry->last_p0_kind = kind;
  return motor_control_push_tx_p0(ctx, entry, &frame);
}

// ---------------------------------------------------------------------------
// 生命周期管理
// ---------------------------------------------------------------------------

// 生产单例（库内静态分配；app 经 motor_control_instance() 只拿到不透明指针）。
static motor_control_context_t g_motor_control_instance;

motor_control_context_t *motor_control_instance(void) {
  return &g_motor_control_instance;
}

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
      (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
    }
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->has_target = false;
    entry->motion_authorized = false;
    // 中止进行中的事务（配置请求回 UNCONFIRMED；身份读取待重连重发）。
    motor_control_abort_tx(ctx, entry, now_ms);
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
  // 整机撤权同时作废轨迹：断链后不得留一条「未来还要动」的时间线
  // （任务层 mailbox 已在同一时刻清掉未取走的轨迹输入）。
  ctx->trajectory_active = false;
  ctx->trajectory.point_count = 0U;
  ctx->trajectory.target_count = 0U;
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
    // 全局授权重立非冻结态表项的逐电机授权位（断链/超时撤权的恢复路径）。
    // FAULT/OFFLINE/CONFIG_CONFLICT 冻结态不自动复立——逐电机恢复只能显式
    // SET_ENABLE(true) + 新鲜完整快照；其中跨总线身份冲突锁存无运行期解除
    // 路径，只能整机重启（ARD §36 安全语义）。
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      if (ctx->entries[i].used &&
          ctx->entries[i].lifecycle != MOTOR_CONTROL_LIFECYCLE_FAULT &&
          ctx->entries[i].lifecycle != MOTOR_CONTROL_LIFECYCLE_OFFLINE &&
          ctx->entries[i].lifecycle !=
              MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
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
    // 设备未发现：无表项（bus_index=0），仍回告 PROFILE_REJECTED，避免
    // 上层请求悬挂。
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
                             motor_id, 0U,
                             MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
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
  motor_control_set_default_device_shadow(entry);
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_BOUND, motor_id,
                           entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  // 身份确认完成 → 识别态收敛到 READY_DISABLED（ARD §9：DISCOVERED→IDENTIFYING
  // →READY_DISABLED；绑定即识别来源，Vaxor-80 不在目录 → 永远无法到这）。
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING) {
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY, motor_id,
                             entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 20ms 控制边界（ARD §11.3 / §51.3）
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 逐电机目标应用（快照通道与轨迹通道共用）
//
// 两条命令源（100Hz 完整快照 / 多点轨迹）必须共用同一套门禁与钳制：否则会
// 出现「快照走限位、轨迹不走」这类只有一条路径才有的安全缺口。
// ---------------------------------------------------------------------------

// 生命周期门：目标只作用于受控集合与使能序列窗口（ARD §13.1）。
static bool entry_target_eligible(const motor_control_entry_t *entry) {
  return entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
         entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
         entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
}

// 运动前置门（授权 / 逐电机 Enable / 逐电机运动授权 / 型号绑定 / 设备参数
// 结论 / 模式事务独占）。
static bool entry_may_move(const motor_control_context_t *ctx,
                           const motor_control_entry_t *entry) {
  if (!ctx->authorized || ctx->require_new_snapshot) {
    return false; // 无授权/待新证据：不得产生运动帧
  }
  if (!entry->policy_enable) {
    return false; // 逐电机 Enable 关闭（恢复须显式 SET_ENABLE(true)）
  }
  if (!entry->motion_authorized) {
    return false; // 授权位已清（故障/离线/撤权）：不得自动恢复（ARD §36）
  }
  if (entry->profile == NULL) {
    return false; // 未绑定型号：不可控
  }
  if (!motor_control_dev_params_allow_motion(entry)) {
    return false; // 设备参数未核对：该台不出运动帧
  }
  if (entry->tx_active && entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return false; // 模式事务独占该电机的使能序列
  }
  return true;
}

// 施加一个目标：限位/峰值钳制 → 使能序列 → MIT 组帧入队。
// 调用方必须先过 entry_target_eligible 与 entry_may_move。
static void entry_apply_target(motor_control_context_t *ctx,
                               motor_control_entry_t *entry, size_t index,
                               float position, float velocity, float effort,
                               uint32_t now_ms) {
  // Policy 软件限位（Vaxor，打包前钳制——ARD §26.1）；限位标志逐拍重算，
  // 不粘滞（目标回到限内即清除）。
  float target_pos = position;
  float target_vel = velocity;
  float target_effort = effort;
  const motor_protocol_model_profile_t *profile = entry->profile;
  entry->limit_flags &= ~(MOTOR_PROTOCOL_LIMIT_POS_MIN | MOTOR_PROTOCOL_LIMIT_POS_MAX |
                          MOTOR_PROTOCOL_LIMIT_VEL |
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
  // 软件速度限值（仅 CAP_LIMIT_SOFTWARE 型号；双向对称）。
  if (entry->has_soft_vel_limit &&
      (profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
    if (target_vel > entry->soft_max_velocity_rad_s) {
      target_vel = entry->soft_max_velocity_rad_s;
      entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_VEL;
    } else if (target_vel < -entry->soft_max_velocity_rad_s) {
      target_vel = -entry->soft_max_velocity_rad_s;
      entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_VEL;
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
  // 软件力矩限值（在峰值钳制之后，只可能更严——设置时已钳到型号峰值）。
  if (entry->has_soft_torque_limit &&
      (profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
    if (target_effort > entry->soft_max_torque_nm) {
      target_effort = entry->soft_max_torque_nm;
      entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_EFFORT;
    } else if (target_effort < -entry->soft_max_torque_nm) {
      target_effort = -entry->soft_max_torque_nm;
      entry->limit_flags |= MOTOR_PROTOCOL_LIMIT_EFFORT;
    }
  }

  entry->has_target = true;
  entry->target_position_rad = target_pos;
  entry->target_velocity_rad_s = target_vel;
  entry->target_effort_nm = target_effort;
  // motion_authorized 不因目标出现而自动复立：撤销后只能由
  // set_authorization(true) 或 SET_ENABLE(true) 显式重立（ARD §36）。

  // 使能时序：未使能先发使能帧；有整定门的协议在本窗内不发运动目标。
  if (!entry->motion_enabled) {
    motor_protocol_tx_frame_t enable_frame = {0};
    if (entry->adapter->encode_enable == NULL ||
        !entry->adapter->encode_enable(profile, entry->node, entry->bus_index,
                                       &enable_frame)) {
      entry->protocol_error_count++;
      entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
      return;
    }
    // 使能帧使用 P0，并先于同拍 P1 MIT 帧出队，避免两帧共享
    // latest-wins 槽时使能帧被覆盖。
    enable_frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
    entry->last_p0_kind = MOTOR_CONTROL_P0_ENABLE;
    enable_frame.tx_slot = (uint16_t)index; // 表项索引：P0 回告重入队按此定位
    if (!motor_control_push_tx(ctx, &enable_frame)) {
      return;
    }
    entry->motion_enabled = true;
    entry->last_commanded_tick_ms = now_ms;
    // 使能帧已出队：此前挂起的软停/失能重试帧是过期意图，不得在其后
    // 出门（否则重新使能的电机被旧失能帧再次打回，CERB 会输掉整个
    // 整定窗）。挂起中的使能帧（重入队转挂起）保持，由 service 补发。
    if (entry->p0_pending && entry->p0_kind != MOTOR_CONTROL_P0_ENABLE) {
      entry->p0_pending = false;
      entry->p0_kind = MOTOR_CONTROL_P0_NONE;
    }
    if (entry->adapter->enable_settle_ms > 0U) {
      entry->enable_pending = true;
      entry->enable_started_tick_ms = now_ms;
      return;
    }
  }
  if (entry->enable_pending) {
    return; // 整定窗内（service 到期后恢复）
  }

  motor_protocol_control_intent_t intent = {0};
  intent.position_rad = entry->target_position_rad;
  intent.velocity_rad_s = entry->target_velocity_rad_s;
  intent.effort_nm = entry->target_effort_nm;
  intent.kp = entry->has_kp_kd ? entry->kp : profile->default_kp;
  intent.kd = entry->has_kp_kd ? entry->kd : profile->default_kd;
  intent.enable = true;
  motor_protocol_tx_frame_t mit_frame = {0};
  if (entry->adapter->encode_mit_command == NULL ||
      !entry->adapter->encode_mit_command(profile, entry->node, entry->bus_index,
                                          &intent, &mit_frame)) {
    entry->protocol_error_count++;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
    return;
  }
  mit_frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  mit_frame.tx_slot = (uint16_t)index; // P1 latest-wins 槽 = 表项索引（任务层路由用）
  if (motor_control_push_tx(ctx, &mit_frame)) {
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

// 归属：轨迹声明集合内的电机由轨迹独占。纯派生（不落字段）——故障/离线/
// 冲突/设备参数未核对各自把表项带离可动生命周期，自动脱离轨迹。
static bool trajectory_owns(const motor_control_context_t *ctx,
                            uint16_t motor_id) {
  if (!ctx->trajectory_active) {
    return false;
  }
  for (uint16_t i = 0; i < ctx->trajectory.target_count; ++i) {
    if (ctx->trajectory.target_ids[i] == motor_id) {
      return true;
    }
  }
  return false;
}

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
  // 序号门已按 2026-09-15 台架决策移除：重放/乱序防护由新鲜度门
  // （30/100ms 级联）承担；last_snapshot_sequence 仅作诊断保留。
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

  // 受控集合逐台处理（ARD §11.3 步骤 10-12 / §51.3）。轨迹所属电机跳过
  // 整段：它是轨迹通道的，快照对它既不下发也不软停。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry_target_eligible(entry)) {
      continue;
    }
    if (trajectory_owns(ctx, entry->motor_id)) {
      continue;
    }

    const motor_control_snapshot_target_t *target = NULL;
    if (!snapshot_contains(snapshot, entry->motor_id, &target)) {
      // 受控集合缺席 → 软停（ARD §13.1，禁止沿用旧目标）。
      motor_control_soft_stop_entry(ctx, entry, now_ms);
      continue;
    }
    if (!entry_may_move(ctx, entry)) {
      continue;
    }
    entry_apply_target(ctx, entry, i, target->position_rad, target->velocity_rad_s,
                       target->effort_nm, now_ms);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 多点轨迹（MotorTrajectory）：在 20ms 控制边界上按时间线确定性插值
// ---------------------------------------------------------------------------

// 轨迹结构校验（与 mailbox 同规则，另加注册表存在性——内核不信任调用方：
// 白盒测试与未来入口都会直接喂这里）。整条拒绝，绝不部分采纳。
// 校验并把「首个非法路点下标」带出去——拒绝事件据此告诉发送方改哪一行。
// *bad_index = MOTOR_CONTROL_TRAJECTORY_BAD_INDEX_NONE 表示与路点无关
// （计数越界 / 归属重复 / 未注册 / 有归属无路点）。
static bool trajectory_validate(const motor_control_context_t *ctx,
                                const motor_control_trajectory_t *trajectory,
                                uint16_t *bad_index) {
  *bad_index = MOTOR_CONTROL_TRAJECTORY_BAD_INDEX_NONE;
  if (trajectory->point_count > MOTOR_CONTROL_MAX_TRAJECTORY_POINTS ||
      trajectory->target_count > MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS) {
    return false;
  }
  // 空轨迹 = 取消，合法载荷。
  if (trajectory->point_count == 0U && trajectory->target_count == 0U) {
    return true;
  }
  // 半条轨迹（有路点无归属 / 有归属无路点）没有可执行语义。
  if (trajectory->point_count == 0U || trajectory->target_count == 0U) {
    return false;
  }

  for (uint16_t i = 0; i < trajectory->target_count; ++i) {
    motor_control_entry_t *owner =
        motor_control_find_entry((motor_control_context_t *)ctx,
                                 trajectory->target_ids[i]);
    if (owner == NULL || !owner->used) {
      return false; // 不存在/未注册设备（与快照的未知 motor_id 同口径）
    }
    for (uint16_t j = (uint16_t)(i + 1U); j < trajectory->target_count; ++j) {
      if (trajectory->target_ids[i] == trajectory->target_ids[j]) {
        return false; // 声明集合重复
      }
    }
  }

  uint32_t seen_mask = 0U; // 每台「已有路点」位图
  // target_count == 32 时 1U<<32 是未定义行为，故位图只覆盖 ≤31 台。
  const bool need_mask = trajectory->target_count < 32U;
  uint32_t previous_time = 0U;
  for (uint16_t i = 0; i < trajectory->point_count; ++i) {
    const motor_control_trajectory_point_t *point = &trajectory->points[i];
    if (!isfinite(point->position_rad) || !isfinite(point->velocity_rad_s) ||
        !isfinite(point->effort_nm)) {
      *bad_index = i;
      return false; // 命令侧禁 NaN/Inf（TBD-ROS-01=A）
    }
    int slot = -1;
    for (uint16_t j = 0; j < trajectory->target_count && slot < 0; ++j) {
      if (trajectory->target_ids[j] == point->motor_id) {
        slot = (int)j;
      }
    }
    if (slot < 0) {
      *bad_index = i;
      return false; // motor_id 不在声明集合内
    }
    if (i > 0U) {
      if (point->time_from_start_ms < previous_time) {
        *bad_index = i;
        return false; // 时间轴必须非递减
      }
      // 时间非递减下，同一电机同一时刻的点必然相邻，只比前一点即可。
      const motor_control_trajectory_point_t *previous = &trajectory->points[i - 1U];
      if (previous->motor_id == point->motor_id &&
          previous->time_from_start_ms == point->time_from_start_ms) {
        *bad_index = i;
        return false; // 同一电机同一时刻重复路点（插值分母为 0）
      }
    }
    previous_time = point->time_from_start_ms;
    if (need_mask) {
      seen_mask |= (uint32_t)1U << (uint32_t)slot;
    }
  }
  if (need_mask) {
    return seen_mask == ((1U << trajectory->target_count) - 1U);
  }
  // 归属电机数 > 31：位图不够用，退回逐台复查（只在超大轨迹时走这条路径）。
  for (uint16_t i = 0; i < trajectory->target_count; ++i) {
    bool found = false;
    for (uint16_t j = 0; j < trajectory->point_count && !found; ++j) {
      found = trajectory->points[j].motor_id == trajectory->target_ids[i];
    }
    if (!found) {
      return false; // 声明了归属却没有任何路点
    }
  }
  return true;
}

// 段定位：取该电机「不晚于 elapsed 的最后一个路点」与其后的第一个路点。
// 返回 false = 该电机在轨迹里没有任何路点（合法载荷不会出现，防御用）。
static bool trajectory_segment(const motor_control_trajectory_t *trajectory,
                               uint16_t motor_id, uint32_t elapsed,
                               const motor_control_trajectory_point_t **left,
                               const motor_control_trajectory_point_t **right) {
  const motor_control_trajectory_point_t *lower = NULL;
  const motor_control_trajectory_point_t *upper = NULL;
  for (uint16_t i = 0; i < trajectory->point_count; ++i) {
    const motor_control_trajectory_point_t *point = &trajectory->points[i];
    if (point->motor_id != motor_id) {
      continue;
    }
    if (point->time_from_start_ms <= elapsed) {
      lower = point; // 时间有序：最后命中的即左端点
    } else if (upper == NULL) {
      upper = point; // 首个晚于 elapsed 的即右端点
    }
  }
  if (lower == NULL) {
    // elapsed 早于该电机首点（首点通常为 0）：钳首点保持，绝不外推。
    lower = upper;
    upper = NULL;
  }
  *left = lower;
  *right = upper;
  return lower != NULL;
}

// 释放归属：软停被移出集合的电机。轨迹结束/取消/被替换都是显式交接，
// 不靠快照隐式夺权（周期发布者会误杀轨迹）。
static void release_trajectory_motor(motor_control_context_t *ctx,
                                     uint16_t motor_id, uint32_t now_ms) {
  motor_control_entry_t *entry = motor_control_find_entry(ctx, motor_id);
  if (entry != NULL) {
    motor_control_soft_stop_entry(ctx, entry, now_ms);
  }
}

bool motor_control_apply_trajectory(motor_control_context_t *ctx,
                                    const motor_control_trajectory_t *trajectory,
                                    uint32_t now_ms) {
  if (ctx == NULL || trajectory == NULL) {
    return false;
  }
  uint16_t bad_index = MOTOR_CONTROL_TRAJECTORY_BAD_INDEX_NONE;
  if (!trajectory_validate(ctx, trajectory, &bad_index)) {
    ctx->trajectory_rejected_count++;
    // 字段重载（本事件只由结构非法触发，reason 放故障码没有第二重信息）：
    //   request_id = 被拒的 trajectory_id，reason = 首个非法路点下标。
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED,
                             0U, 0U, bad_index, trajectory->trajectory_id);
    return false;
  }

  // 取消（空轨迹）：释放全部所属电机并软停，不留归属残留。
  if (trajectory->point_count == 0U) {
    if (ctx->trajectory_active) {
      for (uint16_t i = 0; i < ctx->trajectory.target_count; ++i) {
        release_trajectory_motor(ctx, ctx->trajectory.target_ids[i], now_ms);
      }
    }
    ctx->trajectory_active = false;
    ctx->trajectory.point_count = 0U;
    ctx->trajectory.target_count = 0U;
    return true;
  }

  // 被替换：旧集合里不在新集合的电机立即释放（软停），其余由新轨迹接手。
  if (ctx->trajectory_active) {
    for (uint16_t i = 0; i < ctx->trajectory.target_count; ++i) {
      const uint16_t old_id = ctx->trajectory.target_ids[i];
      bool still_owned = false;
      for (uint16_t j = 0; j < trajectory->target_count && !still_owned; ++j) {
        still_owned = trajectory->target_ids[j] == old_id;
      }
      if (!still_owned) {
        release_trajectory_motor(ctx, old_id, now_ms);
      }
    }
  }

  ctx->trajectory = *trajectory; // 整条锁存：时间线要跨控制边界使用
  ctx->trajectory_active = true;
  ctx->trajectory_start_tick_ms = now_ms; // 起点 = 本地受理 tick（ARD §12.2）
  // 新轨迹是新证据：与快照受理同语义（清「待新证据」门）。
  ctx->require_new_snapshot = false;
  ctx->trajectory_accepted_count++;
  return true;
}

void motor_control_copy_trajectory_state(
    const motor_control_context_t *ctx,
    motor_control_trajectory_state_t *out, uint32_t now_ms) {
  if (ctx == NULL || out == NULL) {
    return;
  }
  const motor_control_trajectory_t *trajectory = &ctx->trajectory;
  out->trajectory_id = trajectory->trajectory_id;
  out->active = ctx->trajectory_active;
  out->target_count = ctx->trajectory_active ? trajectory->target_count : 0U;
  for (uint16_t i = 0U; i < out->target_count; ++i) {
    out->target_ids[i] = trajectory->target_ids[i];
  }
  // 路点时间非递减（mailbox 与内核各校验一次），故末点即总时长。
  const bool live = ctx->trajectory_active && trajectory->point_count > 0U;
  out->total_ms =
      live ? trajectory->points[trajectory->point_count - 1U].time_from_start_ms
           : 0U;
  out->elapsed_ms =
      ctx->trajectory_active ? now_ms - ctx->trajectory_start_tick_ms : 0U;
  out->completed = ctx->trajectory_active && out->elapsed_ms >= out->total_ms;
}

void motor_control_trajectory_step(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL || !ctx->trajectory_active) {
    return; // 无活动轨迹：O(1) 返回（未用轨迹 = 对既有行为零影响）
  }
  const motor_control_trajectory_t *trajectory = &ctx->trajectory;
  const uint32_t elapsed = now_ms - ctx->trajectory_start_tick_ms;
  for (uint16_t t = 0; t < trajectory->target_count; ++t) {
    motor_control_entry_t *entry =
        motor_control_find_entry(ctx, trajectory->target_ids[t]);
    if (entry == NULL || !entry->used || !entry_target_eligible(entry)) {
      continue; // 故障/离线/冲突/未绑定自动脱离轨迹（归属是派生的）
    }
    if (!entry_may_move(ctx, entry)) {
      continue; // 与快照路径同一套门禁
    }
    const motor_control_trajectory_point_t *left = NULL;
    const motor_control_trajectory_point_t *right = NULL;
    if (!trajectory_segment(trajectory, entry->motor_id, elapsed, &left, &right)) {
      continue; // 该电机在轨迹里没有路点（防御）
    }
    float position = left->position_rad;
    float velocity = 0.0f;
    const float effort = left->effort_nm; // 前馈段内保持左端点值
    if (right != NULL) {
      const uint32_t span_ms =
          right->time_from_start_ms - left->time_from_start_ms;
      if (span_ms > 0U) {
        const float alpha =
            (float)(elapsed - left->time_from_start_ms) / (float)span_ms;
        position = left->position_rad +
                   alpha * (right->position_rad - left->position_rad);
        // 速度取段斜率，不直通 point.velocity：上层常把该字段填 0，
        // 直通会让 MIT 律的 Kd 朝 0 速刹车（语义错误）。
        velocity = (right->position_rad - left->position_rad) /
                   ((float)span_ms / 1000.0f);
      }
    }
    entry_apply_target(ctx, entry, motor_control_entry_index(ctx, entry), position,
                       velocity, effort, now_ms);
  }
}

bool motor_control_apply_policy(motor_control_context_t *ctx,
                                const motor_control_policy_snapshot_t *snapshot,
                                uint32_t now_ms) {
  if (ctx == NULL || snapshot == NULL) {
    return false;
  }
  // 整条校验：非法即整条拒绝，绝不部分采纳（与命令快照同口径）。
  if (snapshot->target_count > MOTOR_CONTROL_MAX_MOTORS) {
    ctx->policy_rejected_count++;
    return false;
  }
  for (uint8_t i = 0U; i < snapshot->target_count; ++i) {
    const motor_control_policy_target_t *target = &snapshot->targets[i];
    if (!isfinite(target->kp) || !isfinite(target->kd) || target->kp < 0.0f ||
        target->kd < 0.0f) {
      ctx->policy_rejected_count++;
      return false;
    }
    for (uint8_t j = (uint8_t)(i + 1U); j < snapshot->target_count; ++j) {
      if (snapshot->targets[j].motor_id == target->motor_id) {
        ctx->policy_rejected_count++; // 同一台出现两次：哪一条算数没有定义
        return false;
      }
    }
  }
  ctx->policy_accepted_count++;
  // 逐台落地：**单台被拒不影响其余**——稀疏语义不是「整条连坐」，拒绝经
  // CONFIG_RESULT 事件可见（reason 说明是哪一类：代数未前进/设备参数未核对/
  // 未绑定）。
  for (uint8_t i = 0U; i < snapshot->target_count; ++i) {
    motor_control_entry_t *entry =
        motor_control_find_entry(ctx, snapshot->targets[i].motor_id);
    if (entry == NULL) {
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                               snapshot->targets[i].motor_id, 0U,
                               MOTOR_PROTOCOL_CONFIG_INVALID,
                               snapshot->request_id);
      continue;
    }
    motor_protocol_config_request_t request = {0};
    request.request_id = snapshot->request_id;
    request.motor_id = snapshot->targets[i].motor_id;
    request.kind = MOTOR_PROTOCOL_CONFIG_SET_POLICY;
    request.enable = snapshot->targets[i].enabled;
    request.kp = snapshot->targets[i].kp;
    request.kd = snapshot->targets[i].kd;
    request.policy_generation = snapshot->policy_generation;
    (void)motor_control_apply_policy_entry(ctx, entry, &request, now_ms);
  }
  return true;
}

void motor_control_soft_stop_entry(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry, uint32_t now_ms) {
  if (entry == NULL || !entry->used) {
    return;
  }
  // 受控集合（ARD §13.1 DERIVED）：ACTIVE，或已使能但尚在使能序列中的
  // READY_DISABLED（CERB 使能整定窗内的电机同样必须被软停覆盖）。
  const bool in_controlled_set =
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
       entry->motion_enabled);
  if (in_controlled_set) {
    // 仅在进入软停的迁移沿发帧（去重，防 50Hz P0 刷屏）。
    // 编码失败/队列满都挂起重试（安全帧不静默消失），计数在 helper 内。
    (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_SOFT_STOP);
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
    } else if (entry->p0_kind == MOTOR_CONTROL_P0_ENABLE &&
               entry->adapter != NULL && entry->profile != NULL &&
               entry->adapter->encode_enable != NULL) {
      encoded = entry->adapter->encode_enable(entry->profile, entry->node,
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
    frame.tx_slot = (uint16_t)i; // 循环变量即表项索引
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
    // 心跳（vaxor 20B 状态帧 / CERB 0x080 SYNC 上报帧），承担总线检测——
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
      (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
    }
    entry->online = false;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_OFFLINE;
    entry->has_target = false;
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->motion_authorized = false; // 逐电机授权冻结（§36）
    entry->policy_enable = false;     // 冻结为 false：恢复需显式重发 Enable
    // 中止进行中的事务（配置请求回 UNCONFIRMED；身份读取待重连重发）。
    motor_control_abort_tx(ctx, entry, now_ms);
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_OFFLINE;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_OFFLINE;
    ctx->require_new_snapshot = true; // 重连后旧授权/目标不复用（ARD §10.4）
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_OFFLINE,
                             entry->motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_OFFLINE, 0U);
  }
}

// 100ms 级联的收窄版本（有活动轨迹时用）：只停机快照通道的电机，不动
// 全局授权电平——轨迹电机必须继续按自己的时间线走。停机动作与全局撤权
// 对每台做的事逐条一致，差别只在作用域。
static void disable_snapshot_owned(motor_control_context_t *ctx, uint32_t now_ms) {
  bool disabled_motion = false;
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || trajectory_owns(ctx, entry->motor_id)) {
      continue;
    }
    if (entry->motion_enabled) {
      (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
      disabled_motion = true;
    }
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->has_target = false;
    entry->motion_authorized = false;
    motor_control_abort_tx(ctx, entry, now_ms);
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DISABLE,
                               entry->motor_id, entry->bus_index,
                               MOTOR_CONTROL_FAULT_NONE, 0U);
    }
  }
  if (disabled_motion) {
    // 快照通道的运动授权确实丢了：事件照发（诊断口径不变）。全局授权电平
    // 不动——它由会话电平每边界纯派生，与快照流是否活跃无关（ARD §33）。
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST,
                             0U, 0U, MOTOR_CONTROL_FAULT_NONE, 0U);
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
  // 轨迹在播期间，30/100ms 级联只作用于快照通道的电机：轨迹自带时长，
  // 「多久没收到快照」不是它的失效判据。轨迹电机仍受会话断链（立即全局
  // 撤权）与逐台故障/离线/冲突约束——那几条路径不经过这里。
  const bool holds_trajectory =
      ctx->trajectory_active && ctx->trajectory.target_count > 0U;
  if (age > MOTOR_CONTROL_SNAPSHOT_DISABLE_MS && !ctx->snapshot_disable_done) {
    // 100ms：协议级停机/失能 + 清运动授权（ARD §13.2，一次性升级）。
    if (holds_trajectory) {
      disable_snapshot_owned(ctx, now_ms);
    } else {
      motor_control_revoke_motion(ctx, now_ms);
    }
    ctx->snapshot_disable_done = true;
    ctx->snapshot_soft_stop_active = true;
    return;
  }
  if (age > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS && !ctx->snapshot_soft_stop_active &&
      !ctx->snapshot_disable_done) {
    // 30ms：受控软停（一次性，直到新快照）。轨迹所属电机不在其中。
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      if (trajectory_owns(ctx, ctx->entries[i].motor_id)) {
        continue;
      }
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

// 顺序确定化辅助：取下一个「未输出且 motor_id 最小」的表项，并在 taken 中
// 标记（taken 按表项索引、容量 MOTOR_CONTROL_MAX_MOTORS，由调用方清零）；
// 无可取时返回 NULL。
//
// 为什么需要：注册表顺序 = 发现序，会随上线顺序漂移，而状态/拓扑发布顺序
// 必须稳定（上层处理、人工对读都依赖）。n <= 64 的 O(n^2) 只出现在 10ms
// 桥接轮询路径，不在 1ms 实时环内。
//
// motion states / statuses / topology 三处共用本函数；勿再各自展开选择
// 循环——早期副本散落三处，其一还跨越模块边界直读 ctx->entries。
const motor_control_entry_t *motor_control_entry_take_sorted(
    const motor_control_context_t *ctx, bool *taken) {
  if (ctx == NULL || taken == NULL) {
    return NULL;
  }
  int best = -1;
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    if (!ctx->entries[i].used || taken[i]) {
      continue;
    }
    if (best < 0 || ctx->entries[i].motor_id < ctx->entries[best].motor_id) {
      best = (int)i;
    }
  }
  if (best < 0) {
    return NULL;
  }
  taken[best] = true;
  return &ctx->entries[best];
}

size_t motor_control_copy_motion_states(motor_control_context_t *ctx,
                                        motor_control_motion_state_t *out,
                                        size_t capacity, uint32_t now_ms) {
  if (ctx == NULL || out == NULL) {
    return 0U;
  }
  // 发布顺序确定化：按 motor_id 升序逐个选最小未输出条目。注册表顺序 =
  // 发现序，会随上线顺序漂移；上层处理与人工对读都需要稳定顺序。
  // n ≤ 64 的 O(n²) 选择只在 10ms 桥接轮询路径执行；缺席电机（未发现/
  // 已移除）自然不产生条目，不占位。
  bool taken[MOTOR_CONTROL_MAX_MOTORS] = {false};
  size_t count = 0U;
  for (;;) {
    if (count >= capacity) {
      break;
    }
    const motor_control_entry_t *entry =
        motor_control_entry_take_sorted(ctx, taken);
    if (entry == NULL) {
      break;
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
    // 命令源按归属派生（与仲裁同一判据）：轨迹所属即轨迹，否则看是否有
    // 在效目标。不发散出一份独立状态，免得与仲裁漂移。
    state->target_source =
        trajectory_owns(ctx, entry->motor_id)
            ? MOTOR_CONTROL_TARGET_SOURCE_TRAJECTORY
            : (entry->has_target ? MOTOR_CONTROL_TARGET_SOURCE_SNAPSHOT
                                 : MOTOR_CONTROL_TARGET_SOURCE_NONE);
  }
  return count;
}

size_t motor_control_copy_statuses(motor_control_context_t *ctx,
                                   motor_control_status_t *out, size_t capacity,
                                   uint32_t now_ms) {
  if (ctx == NULL || out == NULL) {
    return 0U;
  }
  // 发布顺序确定化：按 motor_id 升序（同 copy_motion_states 注释）。
  bool taken[MOTOR_CONTROL_MAX_MOTORS] = {false};
  size_t count = 0U;
  for (;;) {
    if (count >= capacity) {
      break;
    }
    const motor_control_entry_t *entry =
        motor_control_entry_take_sorted(ctx, taken);
    if (entry == NULL) {
      break;
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
    status->transaction_state = entry->tx_active ? 1U : 0U;
    status->last_failure_reason = entry->last_failure_reason;
  }
  return count;
}

void motor_control_copy_diagnostics(const motor_control_context_t *ctx,
                                    motor_control_diagnostics_t *out) {
  if (ctx == NULL || out == NULL) {
    return;
  }
  out->rx_frame_count = ctx->rx_frame_count;
  out->unknown_frame_count = ctx->unknown_frame_count;
  out->decode_error_count = ctx->decode_error_count;
  out->snapshot_accepted = ctx->snapshot_accepted_count;
  out->snapshot_rejected = ctx->snapshot_rejected_count;
  out->config_rejected = ctx->config_rejected_count;
  out->event_dropped = ctx->event_dropped_count;
  out->tx_dropped = ctx->tx_dropped_count;
  out->p0_retry = ctx->p0_retry_count;
  out->trajectory_accepted = ctx->trajectory_accepted_count;
  out->trajectory_rejected = ctx->trajectory_rejected_count;
  out->policy_accepted = ctx->policy_accepted_count;
  out->policy_rejected = ctx->policy_rejected_count;
}

bool motor_control_take_record_dirty(motor_control_context_t *ctx) {
  if (ctx == NULL || !ctx->record_dirty) {
    return false;
  }
  ctx->record_dirty = false;
  return true;
}

size_t motor_control_copy_topology(motor_control_context_t *ctx,
                                   motor_control_topology_entry_t *out,
                                   size_t capacity) {
  if (ctx == NULL || out == NULL) {
    return 0U;
  }
  // 发布顺序确定化：按 motor_id 升序（同 copy_motion_states 注释）。
  bool taken[MOTOR_CONTROL_MAX_MOTORS] = {false};
  size_t count = 0U;
  for (;;) {
    if (count >= capacity) {
      break;
    }
    const motor_control_entry_t *entry =
        motor_control_entry_take_sorted(ctx, taken);
    if (entry == NULL) {
      break;
    }
    motor_control_topology_entry_t *t = &out[count++];
    t->motor_id = entry->motor_id;
    t->bus_index = entry->bus_index;
    t->protocol = entry->adapter;
    t->model_name = entry->profile != NULL ? entry->profile->model_name : NULL;
    t->capability_flags =
        entry->profile != NULL ? entry->profile->capability_flags : 0U;
    t->online = entry->online;
    t->lifecycle = entry->lifecycle;
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
