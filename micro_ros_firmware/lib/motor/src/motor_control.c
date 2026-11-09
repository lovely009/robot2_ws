// motor_control 内核主体：初始化、会话/授权、2ms/500Hz 控制边界、1ms 服务、TX 收集与状态/事件副本。
//
// 不依赖 ROS/HAL/FreeRTOS/control_mailbox；厂家字节只经 motor_protocol 的 adapter 接口；一切时序
// 判定用 MCU 单调时基毫秒。
// 安全要点：P0 帧（软停/失能）推送失败不静默丢弃——先驱逐最旧非 P0 帧腾位，仍失败则挂 p0_pending
// 由 service 逐节拍重发，新 MIT/使能帧成功出队后作废（过期重试帧不得晚于新安全意图出门）。
// 受控集合 = ACTIVE ∪ READY_DISABLED(已使能)；apply 前置 staleness 门：age>30ms 不下发目标、
// age>100ms 撤权，过期快照不记录、不复位级联去重标志。

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
  // P1 is latest-wins at BOTH queue stages. apply_snapshot and hold_step
  // may run on the same boundary; retaining both copies makes a 37-axis batch
  // exceed the 40-frame drain and produces extra commands on the next 1ms tick.
  // Preserve the slot's place so replacing a busy axis cannot starve another.
  if (frame->tx_class == MOTOR_PROTOCOL_TX_CLASS_P1) {
    for (size_t i = 0U; i < ctx->tx_count; ++i) {
      motor_protocol_tx_frame_t *queued = &ctx->tx_pending[i];
      if (queued->tx_class == MOTOR_PROTOCOL_TX_CLASS_P1 &&
          queued->bus_index == frame->bus_index && queued->tx_slot == frame->tx_slot) {
        *queued = *frame;
        return true;
      }
    }
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
  // 全为 P0 时按 last_p0_kind 挂起重试；调用方推送前须设置类别。
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

// 带符号毫秒差支持 uint32 回绕，要求间隔 <2^31ms。
// ISR 帧戳可能比任务提前采样的 now 超前 1ms，须视为新鲜帧而非超时。
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

// 统一生成失能/软停帧；推送前设置 last_p0_kind，供下游拒收时重编码。
// 编码失败或队列满均挂起重试，并更新诊断计数。
bool motor_control_send_p0(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint8_t kind) {
  if (ctx == NULL || entry == NULL) {
    return false;
  }
  // 停机是同轴发送屏障：先使旧目标/旧使能失效，再尝试编码安全帧。
  // 只清本轴 P0/P1，P2 配置流与其他电机保持原顺序。下游取消独立锁存，
  // 不依赖安全帧是否能在本拍入队；否则队满时旧 P1 仍可能重新使能电机。
  const size_t slot = motor_control_entry_index(ctx, entry);
  size_t kept = 0U;
  for (size_t i = 0U; i < ctx->tx_count; ++i) {
    const motor_protocol_tx_frame_t *queued = &ctx->tx_pending[i];
    if (queued->tx_slot == slot &&
        queued->tx_class != MOTOR_PROTOCOL_TX_CLASS_P2) {
      continue;
    }
    ctx->tx_pending[kept++] = *queued;
  }
  ctx->tx_count = kept;
  ctx->tx_cancel_pending |= UINT64_C(1) << slot;
  entry->last_p0_kind = kind;
  motor_protocol_tx_frame_t frame = {0};
  bool encoded = false;
  if (entry->adapter != NULL && kind == MOTOR_CONTROL_P0_DISABLE) {
    encoded = entry->adapter->encode_disable != NULL &&
        entry->adapter->encode_disable(entry->profile, entry->node,
        entry->bus_index, &frame);
  } else if (entry->adapter != NULL && kind == MOTOR_CONTROL_P0_SOFT_STOP) {
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
  const bool was_authorized = ctx->authorized;
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
    // Revoking a host lease/session also disarms policy. The stored generation
    // remains the high-water mark, so a replay cannot arm the next lease.
    entry->policy_enable = false;
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
  if (was_authorized) {
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST,
        0U, 0U, MOTOR_CONTROL_FAULT_NONE, 0U);
  }
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

bool motor_control_is_authorized(const motor_control_context_t *ctx) {
  return ctx != NULL && ctx->authorized;
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
// 2ms/500Hz 控制边界（当前运行时配置）
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 逐电机目标应用（快照通道与轨迹通道共用）
//
// 两条命令源（完整快照 / 多点轨迹）必须共用同一套门禁与钳制：否则会
// 出现「快照走限位、轨迹不走」这类只有一条路径才有的安全缺口。
// ---------------------------------------------------------------------------

#define MOTOR_CONTROL_MOTION_FEEDBACK_TIMEOUT_MS 100U

// 真实运动反馈门：链路在线或 SDO 应答不能替代位置/速度测量。
static bool entry_motion_feedback_fresh(const motor_control_entry_t *entry,
    uint32_t now_ms) {
  if (!entry->has_measured || entry->last_motion_rx_tick_ms == 0U) {
    return false;
  }
  // ISR 帧戳可比任务采样超前数毫秒；最多容忍 2ms。真正的过期仍按
  // >100ms 拒绝，且用有符号差避免超前时间戳被无符号下溢成“超老”。
  const int32_t age_ms =
      (int32_t)(now_ms - entry->last_motion_rx_tick_ms);
  return age_ms >= -2 &&
         age_ms <= (int32_t)MOTOR_CONTROL_MOTION_FEEDBACK_TIMEOUT_MS;
}

// 生命周期门：目标只作用于受控集合与使能序列窗口（ARD §13.1）。
static bool entry_target_eligible(const motor_control_entry_t *entry) {
  return entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
         entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
         entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
}

// 运动前置门：全局授权、逐电机使能与授权、型号绑定、模式事务互斥。
// 设备参数核对仅报告，不参与运动授权。
static bool entry_may_move(const motor_control_context_t *ctx,
    const motor_control_entry_t *entry, uint32_t now_ms) {
  if (!ctx->authorized || ctx->require_new_snapshot) {
    return false; // 无授权/待新证据：不得产生运动帧
  }
  if (!entry->policy_enable) {
    return false; // 逐电机 Enable 关闭（恢复须显式 SET_ENABLE(true)）
  }
  if (!entry->motion_authorized) {
    return false; // 授权位已清（故障/离线/撤权）：不得自动恢复（ARD §36）
  }
  if (!entry_motion_feedback_fresh(entry, now_ms)) {
    return false; // 真实运动量缺失/过期：其它帧刷新 last_rx 也不得续租
  }
  if (entry->profile == NULL) {
    return false; // 未绑定型号：不可控
  }
  if (!motor_control_dev_params_allow_motion(entry)) {
    return false; // 无有效表项；参数核对结果不拦截运动
  }
  if (entry->tx_active && entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return false; // 模式事务独占该电机的使能序列
  }
  return true;
}

// MIT 组帧入队（用表项里**已钳制**的目标）。调用方必须先过
// entry_target_eligible / entry_may_move / enable_pending 三道门。
//
// supersede_p0 区分两种投递：
//  - true = 新目标（快照/轨迹路径）。新运动命令取代过期的安全帧（既有语义）。
//  - false = 边界保持重发（motor_control_hold_step）。同一目标的重复投递**不是
//    新证据**，不得把挂起的软停/失能重试取消掉——那会让一条迟到的 P0 永远发不
//    出去。重发路径在调用前已挡掉 p0_pending，这里的 false 是第二道锁。
static bool entry_push_mit(motor_control_context_t *ctx,
    motor_control_entry_t *entry, size_t index,
    uint32_t now_ms, bool supersede_p0) {
  const motor_protocol_model_profile_t *profile = entry->profile;
  motor_protocol_model_profile_t effective = *profile;
  const uint8_t t_bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX);
  if ((entry->dev_raw_present & t_bit) && entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX] > 0U)
    effective.mit_torque_limit_nm = (float)entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX];
  profile = &effective;
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
    return false;
  }
  mit_frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  mit_frame.tx_slot = (uint16_t)index; // P1 latest-wins 槽 = 表项索引（任务层路由用）
  if (!motor_control_push_tx(ctx, &mit_frame)) {
    return false;
  }
  if (supersede_p0) {
    // 过期的 P0 重试帧作废（新命令取代旧安全帧）。
    entry->p0_pending = false;
    entry->p0_kind = MOTOR_CONTROL_P0_NONE;
  }
  entry->last_commanded_tick_ms = now_ms;
  if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE) {
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_ACTIVE,
        entry->motor_id, entry->bus_index,
        MOTOR_CONTROL_FAULT_NONE, 0U);
  }
  return true;
}

// 施加一个目标：限位/峰值钳制 → 使能序列 → MIT 组帧入队。
// 调用方必须先过 entry_target_eligible 与 entry_may_move。
static void entry_apply_target(motor_control_context_t *ctx,
    motor_control_entry_t *entry, size_t index,
    float position, float velocity, float effort,
    uint32_t now_ms, bool emit) {
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

  if (!emit) {
    entry->fresh_target_pending = true;
    return; // admit the new values/deadline now; no physical motion until a boundary
  }
  entry->fresh_target_pending = false;

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
    // An explicitly fresh, authorized target can resume a soft stop, including
    // during the original vendor settle window. Keep that window unchanged;
    // reporting SOFT_STOP after accepting this new intent would make the host
    // latch a retired state until the first MIT output. A cached hold cannot
    // reach this path after soft stop because has_target was cleared.
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    }
    return; // 整定窗内（service 到期后恢复）
  }

  (void)entry_push_mit(ctx, entry, index, now_ms, true);
}

// 轨迹激活时，目标 ID 集合内的电机由轨迹独占；不在此判断生命周期或参数核对结果。
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

static bool apply_command_snapshot(
    motor_control_context_t *ctx, const motor_control_command_snapshot_t *snapshot,
    uint32_t now_ms, bool emit) {
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
  // 序号仅作诊断；时效门采用本地接收时间的 30/100ms 级联。
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

  // 同一个绝对截止贯穿接收、排队、执行；无效输入不刷新租约。
  const uint32_t remaining = snapshot->valid_until_tick_ms - now_ms;
  if (remaining == 0U || remaining > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    ++ctx->snapshot_rejected_count;
    return false;
  }

  // 接受：freshness 基准 = mailbox 记录的本地接收 tick（ARD §12.2）。
  ctx->have_snapshot = true;
  ctx->last_snapshot_tick_ms = snapshot->received_tick_ms;
  ctx->last_snapshot_sequence = snapshot->sequence;
  ctx->snapshot_deadline_tick_ms = snapshot->valid_until_tick_ms;
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
    if (!entry_may_move(ctx, entry, now_ms)) {
      continue;
    }
    entry_apply_target(ctx, entry, i, target->position_rad, target->velocity_rad_s,
        target->effort_nm, now_ms, emit);
  }
  return true;
}

bool motor_control_apply_command_snapshot(motor_control_context_t *ctx,
    const motor_control_command_snapshot_t *snapshot, uint32_t now_ms) {
  return apply_command_snapshot(ctx, snapshot, now_ms, true);
}
bool motor_control_admit_command_snapshot(motor_control_context_t *ctx,
    const motor_control_command_snapshot_t *snapshot, uint32_t now_ms) {
  return apply_command_snapshot(ctx, snapshot, now_ms, false);
}

// ---------------------------------------------------------------------------
// 多点轨迹（MotorTrajectory）：在 2ms/500Hz 控制边界上按时间线确定性插值
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
      // 同时间戳可交错不同电机，检查全部前点以防 1/2/1 形式漏检。
      for (uint16_t j = 0U; j < i; ++j) {
        const motor_control_trajectory_point_t *previous = &trajectory->points[j];
        if (previous->motor_id == point->motor_id &&
            previous->time_from_start_ms >= point->time_from_start_ms) {
          *bad_index = i;
          return false; // 同一电机时间必须严格递增
        }
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

// 非均匀单调三次 Hermite：受理时预计算切矢，控制边界只求值。
// 内部点取相邻三点的 Fritsch–Carlson 切矢（rad/s）：局部极值取 0，
// 否则按相邻段时长加权调和平均，并限幅到 3×min(|割线斜率|)，避免超调。
static float trajectory_monotone_tangent(
    const motor_control_trajectory_point_t *p0,
    const motor_control_trajectory_point_t *p1,
    const motor_control_trajectory_point_t *p2) {
  const float dt0 =
      (float)(p1->time_from_start_ms - p0->time_from_start_ms) / 1000.0f;
  const float dt1 =
      (float)(p2->time_from_start_ms - p1->time_from_start_ms) / 1000.0f;
  if (!(dt0 > 0.0f) || !(dt1 > 0.0f)) {
    return 0.0f; // 校验已保证同电机时间严格递增，这里只做防御
  }
  const float d0 = (p1->position_rad - p0->position_rad) / dt0;
  const float d1 = (p2->position_rad - p1->position_rad) / dt1;
  if (d0 * d1 <= 0.0f) {
    return 0.0f; // 局部极值（含任一割线为 0）：切矢 0，曲线在极值处停住
  }
  const float a0 = fabsf(d0);
  const float a1 = fabsf(d1);
  const float limit = 3.0f * (a0 < a1 ? a0 : a1);
  // 对不等时间间隔不能使用简单中心差分。权重来自相邻两段时长，
  // 加权调和平均在单调数据上保持正性并抑制短段造成的速度过冲。
  const float w1 = 2.0f * dt1 + dt0;
  const float w2 = dt1 + 2.0f * dt0;
  float m = (w1 + w2) / (w1 / d0 + w2 / d1);
  if (m > limit) {
    m = limit;
  } else if (m < -limit) {
    m = -limit;
  }
  return m;
}

// 为轨迹里每台电机的路点序列一次性算出全部切矢。按台分段流式推进：维护
// 「前一点 p0 / 当前点 p1」，见到下一点 p2 时就能定下 p1 的切矢；每台的首点
// 与末点恒为 0（端点零速）。复杂度与既有校验同量级，且只在受理时跑一次。
static void trajectory_compute_tangents(motor_control_context_t *ctx) {
  const motor_control_trajectory_t *trajectory = &ctx->trajectory;
  for (uint16_t i = 0; i < trajectory->point_count; ++i) {
    ctx->trajectory_tangent_rad_s[i] = 0.0f;
  }
  for (uint16_t m = 0; m < trajectory->target_count; ++m) {
    const uint16_t id = trajectory->target_ids[m];
    const motor_control_trajectory_point_t *p0 = NULL;
    const motor_control_trajectory_point_t *p1 = NULL;
    uint16_t i1 = 0U;
    for (uint16_t i = 0; i < trajectory->point_count; ++i) {
      const motor_control_trajectory_point_t *p2 = &trajectory->points[i];
      if (p2->motor_id != id) {
        continue;
      }
      if (p1 != NULL) {
        // p1 是首点（p0 == NULL，端点零速）或内部点（不等时间加权 + 限幅）。
        ctx->trajectory_tangent_rad_s[i1] =
            (p0 == NULL) ? 0.0f : trajectory_monotone_tangent(p0, p1, p2);
      }
      p0 = p1;
      p1 = p2;
      i1 = i;
    }
    if (p1 != NULL) {
      ctx->trajectory_tangent_rad_s[i1] = 0.0f; // 末点：端点零速
    }
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
  ctx->trajectory_agent_mode = false;
  ctx->trajectory_active = true;
  trajectory_compute_tangents(ctx); // 切矢只在此算一次，热路径只做求值
  ctx->trajectory_start_tick_ms = now_ms; // 起点 = 本地受理 tick（ARD §12.2）
  // 新轨迹是新证据：与快照受理同语义（清「待新证据」门）。
  ctx->require_new_snapshot = false;
  ctx->trajectory_accepted_count++;
  return true;
}

bool motor_control_trajectory_targets_healthy(const motor_control_context_t *ctx,
    const uint16_t *target_ids, uint16_t target_count, uint32_t now_ms) {
  if (ctx == NULL || target_ids == NULL || !ctx->authorized ||
      target_count == 0U || target_count > MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS)
    return false;
  for (uint16_t i = 0U; i < target_count; ++i) {
    const motor_control_entry_t *entry = motor_control_find_entry(
        (motor_control_context_t *)ctx, target_ids[i]);
    if (entry == NULL || !entry->used || !entry->online || entry->profile == NULL ||
        !entry_target_eligible(entry) || !entry->policy_enable ||
        !entry->motion_authorized || !entry->has_measured ||
        now_ms - entry->last_motion_rx_tick_ms > 100U ||
        (entry->tx_active && entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE))
      return false;
  }
  return true;
}

bool motor_control_apply_agent_trajectory(motor_control_context_t *ctx,
    const motor_control_trajectory_t *trajectory, uint32_t now_ms) {
  if (ctx == NULL || trajectory == NULL || ctx->trajectory_active ||
      !ctx->have_snapshot || now_ms - ctx->last_snapshot_tick_ms > 30U ||
      !motor_control_trajectory_targets_healthy(ctx, trajectory->target_ids,
          trajectory->target_count, now_ms)) return false;
  if (!motor_control_apply_trajectory(ctx, trajectory, now_ms)) return false;
  ctx->trajectory_agent_mode = true;
  return true;
}

bool motor_control_cancel_trajectory_id(motor_control_context_t *ctx,
    uint32_t trajectory_id, uint32_t now_ms) {
  if (ctx == NULL || !ctx->trajectory_active ||
      ctx->trajectory.trajectory_id != trajectory_id) return false;
  for (uint16_t i = 0U; i < ctx->trajectory.target_count; ++i) {
    release_trajectory_motor(ctx, ctx->trajectory.target_ids[i], now_ms);
  }
  ctx->trajectory_active = false;
  return true;
}

bool motor_control_trajectory_safety_pending(const motor_control_context_t *ctx,
    const uint16_t *target_ids, uint16_t target_count) {
  if (ctx == NULL || target_ids == NULL ||
      target_count > MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS) return true;
  for (uint16_t i = 0U; i < target_count; ++i) {
    const motor_control_entry_t *entry = motor_control_find_entry(
        (motor_control_context_t *)ctx, target_ids[i]);
    if (entry != NULL && entry->p0_pending) return true;
  }
  return false;
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
  if (ctx->trajectory_agent_mode &&
      (!ctx->have_snapshot || now_ms - ctx->last_snapshot_tick_ms > 30U ||
       !motor_control_trajectory_targets_healthy(ctx, trajectory->target_ids,
           trajectory->target_count, now_ms))) {
    (void)motor_control_cancel_trajectory_id(ctx, trajectory->trajectory_id, now_ms);
    return; // 一台反馈/策略失效中止整条，不允许剩余电机继续运行
  }
  const uint32_t elapsed = now_ms - ctx->trajectory_start_tick_ms;
  for (uint16_t t = 0; t < trajectory->target_count; ++t) {
    motor_control_entry_t *entry =
        motor_control_find_entry(ctx, trajectory->target_ids[t]);
    if (entry == NULL || !entry->used || !entry_target_eligible(entry)) {
      continue; // 故障/离线/冲突/未绑定自动脱离轨迹（归属是派生的）
    }
    if (!entry_may_move(ctx, entry, now_ms)) {
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
        // Hermite 插值保持段间位置与速度连续。
        const uint16_t li = (uint16_t)(left - trajectory->points);
        const uint16_t ri = (uint16_t)(right - trajectory->points);
        const float h = (float)span_ms / 1000.0f;
        const float tau =
            (float)(elapsed - left->time_from_start_ms) / (float)span_ms;
        const float t2 = tau * tau;
        const float t3 = t2 * tau;
        const float m0 = ctx->trajectory_tangent_rad_s[li];
        const float m1 = ctx->trajectory_tangent_rad_s[ri];
        position = (2.0f * t3 - 3.0f * t2 + 1.0f) * left->position_rad +
            (t3 - 2.0f * t2 + tau) * h * m0 +
            (-2.0f * t3 + 3.0f * t2) * right->position_rad +
            (t3 - t2) * h * m1;
        // 解析导数 v = (dx/dτ)/h —— 不直通 point.velocity（上层常填 0，
        // 直通会让 Kd 朝 0 速刹车）。
        const float dh00 = 6.0f * t2 - 6.0f * tau;
        const float dh10 = 3.0f * t2 - 4.0f * tau + 1.0f;
        const float dh01 = -6.0f * t2 + 6.0f * tau;
        const float dh11 = 3.0f * t2 - 2.0f * tau;
        velocity = (dh00 * left->position_rad + dh10 * h * m0 +
            dh01 * right->position_rad + dh11 * h * m1) / h;
      }
    }
    entry_apply_target(ctx, entry, motor_control_entry_index(ctx, entry), position,
        velocity, effort, now_ms, true);
  }
}

// 每控制边界零阶保持已受理的快照目标，命令率不依赖上行到达率。
// 仅保持 ≤30ms 的新鲜快照；撤权、故障等门禁不变，轨迹所属电机由同拍轨迹步进处理。
void motor_control_hold_step(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL || !ctx->have_snapshot) {
    return;
  }
  // 快照过期即停发：过了这条线由 service_snapshot_freshness 走软停/撤权，
  // 保持重发绝不能把它盖过去。
  const uint32_t remaining = ctx->snapshot_deadline_tick_ms - now_ms;
  if (remaining == 0U || remaining > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS ||
      (uint32_t)(now_ms - ctx->last_snapshot_tick_ms) >
          MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    return;
  }
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry_target_eligible(entry)) {
      continue;
    }
    // 轨迹机组由 trajectory_step 在同一拍发过（它是边界驱动的），跳过。
    if (trajectory_owns(ctx, entry->motor_id)) {
      continue;
    }
    // 只在"有已受理目标"时保持；缺席快照已在受理时软停并清 has_target。
    if (!entry->has_target) {
      continue;
    }
    if (entry->fresh_target_pending && entry_may_move(ctx, entry, now_ms)) {
      entry_apply_target(ctx, entry, i, entry->target_position_rad,
          entry->target_velocity_rad_s, entry->target_effort_nm, now_ms, true);
      continue;
    }
    // 两个"别的意图优先"的窗口：整定窗内不发运动目标；有挂起的 P0
    // （软停/失能/使能重试）时必须让安全帧先出去。
    if (entry->enable_pending || entry->p0_pending) {
      continue;
    }
    if (!entry_may_move(ctx, entry, now_ms)) {
      continue;
    }
    (void)entry_push_mit(ctx, entry, i, now_ms, false);
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
  // 稀疏策略逐台处理；单台拒绝不影响其余，结果经 CONFIG_RESULT 可见。
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
    // 显式确认位须传到保护关闭时的武装门禁。
    request.confirm_unprotected = snapshot->targets[i].confirm_unprotected;
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
    // 仅在进入软停的迁移沿发帧（去重，防 500Hz P0 刷屏）。
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
    // VAXOR/CERB 均有 20ms 状态心跳，按反馈龄判离线；
    // 非自主反馈协议仅在持续发命令而无反馈时判定。
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

bool motor_control_target_valid(const motor_control_context_t *ctx,
    uint16_t slot, uint32_t now_ms) {
  if (ctx == NULL || slot >= ctx->entry_count || !ctx->have_snapshot ||
      (uint32_t)(now_ms - ctx->last_snapshot_tick_ms) > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    return false;
  }
  const motor_control_entry_t *entry = &ctx->entries[slot];
  if (!entry->used || !entry->has_target || !entry_target_eligible(entry) ||
      !entry_may_move(ctx, entry, now_ms) || entry->p0_pending) return false;
  if (trajectory_owns(ctx, entry->motor_id)) return true;
  const uint32_t remaining = ctx->snapshot_deadline_tick_ms - now_ms;
  return remaining > 0U && remaining <= MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS;
}

void motor_control_stop_all(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) return;
  motor_control_revoke_motion(ctx, now_ms);
  // 新会话/安全停止后，稀疏新策略不能顺带复活未列出的旧电机策略。
  for (size_t i = 0; i < ctx->entry_count; ++i) ctx->entries[i].policy_enable = false;
}

// 租约断流对全部运动源生效；软停后轨迹不因心跳恢复而续播。
void motor_control_soft_stop_all(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) return;
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_soft_stop_entry(ctx, &ctx->entries[i], now_ms);
  }
  ctx->trajectory_active = false;
}

// 运动反馈过期与链路离线分开处置：保留在线链路，但该电机立即安全停机并使
// 恢复必须经过显式策略授权；SDO/状态帧刷新 last_rx 不得掩盖该事实。
static void service_motion_feedback_checks(motor_control_context_t *ctx,
    uint32_t now_ms) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online ||
        (!entry->has_target && !entry->motion_enabled &&
         entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE) ||
        entry_motion_feedback_fresh(entry, now_ms)) {
      continue;
    }
    motor_control_soft_stop_entry(ctx, entry, now_ms);
    entry->has_target = false;
    entry->motion_authorized = false;
    entry->policy_enable = false;
  }
}

static void service_snapshot_freshness(motor_control_context_t *ctx, uint32_t now_ms) {
  if (!ctx->have_snapshot) {
    return;
  }
  const uint32_t age = now_ms - ctx->last_snapshot_tick_ms;
  if (age > MOTOR_CONTROL_SNAPSHOT_DISABLE_MS && !ctx->snapshot_disable_done) {
    motor_control_revoke_motion(ctx, now_ms);
    ctx->snapshot_disable_done = true;
    ctx->snapshot_soft_stop_active = true;
    return;
  }
  if (age > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS && !ctx->snapshot_soft_stop_active &&
      !ctx->snapshot_disable_done) {
    motor_control_soft_stop_all(ctx, now_ms);
    ctx->snapshot_soft_stop_active = true;
    return;
  }
  const uint32_t remaining = ctx->snapshot_deadline_tick_ms - now_ms;
  if (remaining == 0U || remaining > MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS) {
    // 执行截止仅停止快照来源；轨迹仍由独立的 30ms 通信租约约束。
    for (size_t i = 0; i < ctx->entry_count; ++i) {
      if (!trajectory_owns(ctx, ctx->entries[i].motor_id)) {
        motor_control_soft_stop_entry(ctx, &ctx->entries[i], now_ms);
      }
    }
  }
}

void motor_control_service(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }
  service_offline_checks(ctx, now_ms);
  service_motion_feedback_checks(ctx, now_ms);
  service_snapshot_freshness(ctx, now_ms);
  if (ctx->trajectory_active && ctx->trajectory_agent_mode &&
      !motor_control_trajectory_targets_healthy(ctx, ctx->trajectory.target_ids,
          ctx->trajectory.target_count, now_ms)) {
    (void)motor_control_cancel_trajectory_id(ctx, ctx->trajectory.trajectory_id, now_ms);
  }
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

uint64_t motor_control_take_tx_cancellations(motor_control_context_t *ctx) {
  if (ctx == NULL) {
    return 0U;
  }
  const uint64_t slots = ctx->tx_cancel_pending;
  ctx->tx_cancel_pending = 0U;
  return slots;
}

uint64_t motor_control_take_zero_tx_cancellations(motor_control_context_t *ctx) {
  if (ctx == NULL) return 0U;
  const uint64_t slots = ctx->zero_tx_cancel_pending;
  ctx->zero_tx_cancel_pending = 0U;
  return slots;
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

// 取最小未输出 motor_id 并标记 taken；taken 由调用方按容量清零。
// 三类发布副本共用；n≤64 的 O(n²) 选择仅在 10ms 桥轮询运行。
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
  // 按 motor_id 升序输出，不受发现顺序影响。
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

static uint8_t control_lifecycle_of(const motor_control_entry_t *entry) {
  if (entry->profile == NULL) {
    return entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING
        ? MOTOR_CONTROL_CONTROL_LIFECYCLE_CONFIGURING
        : MOTOR_CONTROL_CONTROL_LIFECYCLE_UNCONFIGURED;
  }
  // 建档默认策略位不等于显式武装；旧路径的实际输出仍是已有激活证据。
  const bool activated = entry->policy_generation != 0U ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE;
  if (entry->online && entry->unified_fault_code == MOTOR_CONTROL_FAULT_NONE &&
      entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT &&
      entry->policy_enable && entry->motion_authorized && activated) {
    return MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVE;
  }
  return MOTOR_CONTROL_CONTROL_LIFECYCLE_INACTIVE;
}

static uint8_t execution_state_of(const motor_control_context_t *ctx,
    const motor_control_entry_t *entry, uint32_t now_ms) {
  if (!entry->online || entry->unified_fault_code != MOTOR_CONTROL_FAULT_NONE) {
    return MOTOR_CONTROL_EXECUTION_UNKNOWN;
  }
  if (entry->p0_pending && entry->p0_kind == MOTOR_CONTROL_P0_DISABLE) {
    return MOTOR_CONTROL_EXECUTION_DISABLING;
  }
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
    return MOTOR_CONTROL_EXECUTION_SOFT_STOP;
  }
  if (entry->enable_pending) {
    return MOTOR_CONTROL_EXECUTION_ENABLING;
  }
  if (!entry->has_target || !entry->motion_enabled) {
    return MOTOR_CONTROL_EXECUTION_IDLE;
  }
  if (trajectory_owns(ctx, entry->motor_id) && ctx->trajectory.point_count > 0U) {
    return now_ms - ctx->trajectory_start_tick_ms <
        ctx->trajectory.points[ctx->trajectory.point_count - 1U].time_from_start_ms
        ? MOTOR_CONTROL_EXECUTION_TRAJECTORY : MOTOR_CONTROL_EXECUTION_HOLDING;
  }
  if (entry->target_velocity_rad_s != 0.0f ||
      (entry->has_measured && (fabsf(entry->measured.velocity_rad_s) > 0.001f ||
      fabsf(entry->target_position_rad - entry->measured.position_rad) > 0.001f))) {
    return MOTOR_CONTROL_EXECUTION_COMMAND;
  }
  return MOTOR_CONTROL_EXECUTION_HOLDING;
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
    status->control_lifecycle = control_lifecycle_of(entry);
    status->execution_state = execution_state_of(ctx, entry, now_ms);
    status->policy_enabled = entry->policy_enable;
    status->motion_enabled = entry->motion_enabled;
    status->state_age_ms = entry->last_motion_rx_tick_ms != 0U
        ? now_ms - entry->last_motion_rx_tick_ms : UINT32_MAX;
    status->policy_generation = entry->policy_generation;
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
    // 设备侧安全事实：只暴露正向语义，厂家原值不越过协议层。
    const uint8_t safety_bit =
        (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
    const uint8_t dir_bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
    status->device_protect_known = (entry->dev_raw_present & safety_bit) != 0U;
    // 设备原值 0 = 保护生效，向外一律给正向布尔。
    status->device_protect_enabled =
        status->device_protect_known &&
        entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 0U;
    status->device_direction_known = (entry->dev_raw_present & dir_bit) != 0U;
    status->device_direction =
        (uint8_t)entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
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
