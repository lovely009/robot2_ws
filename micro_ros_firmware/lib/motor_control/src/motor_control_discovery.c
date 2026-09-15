// motor_control 发现与注册：原始帧 → adapter 归属 → 注册表条目 → 解码 →
// 生命周期/冲突/故障处理（ARD-0007 §9/§10/§21/§36）。
//
// 键与冲突：注册表以全局唯一 uint16 motor_id 为键（当前派生：= 厂家 node 号，
// 部署保证全局唯一；同号再现即冲突兜底）。同 motor_id 跨总线/跨协议身份不符
// → CONFIG_CONFLICT：禁运动、出事件、禁止"后到数据覆盖旧表项"（REV-08）。
// 冲突态穿越 OFFLINE/重连保持锁存——不存在自动解除路径（§9.3：不以任何
// 形式自动覆盖冲突，复位只能通过整机重启/未来的显式运维接口）。
//
// 重连（ARD §36/§10.4）：重新走 DISCOVERED 流（不直达 READY）；旧运动授权
// （逐电机 motion_authorized）与目标清除，Profile 保留时由 service 的识别流
// 推进收敛 READY_DISABLED。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// 冲突置位：禁运动 + P0 失能帧（推送失败挂起重试）+ 事件（只触发一次）。
static void latch_config_conflict(motor_control_context_t *ctx,
                                  motor_control_entry_t *entry, uint32_t now_ms) {
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
    return;
  }
  if (entry->motion_enabled && entry->adapter != NULL &&
      entry->adapter->encode_disable != NULL) {
    motor_protocol_tx_frame_t frame = {0};
    if (entry->adapter->encode_disable(entry->profile, entry->node,
                                       entry->bus_index, &frame)) {
      frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
      frame.tx_slot = (uint16_t)motor_control_entry_index(ctx, entry);
      if (!motor_control_push_tx_p0(ctx, entry, &frame)) {
        entry->p0_pending = true;
        entry->p0_kind = MOTOR_CONTROL_P0_DISABLE;
      }
    }
  }
  entry->motion_enabled = false;
  entry->enable_pending = false;
  entry->has_target = false;
  entry->motion_authorized = false;
  // policy_enable 一并冻结：即使全局授权与新快照随后到达，也必须等待
  // motion_authorized，若 policy_enable 残留 true，锁存清除后仅重发全局授权
  // + 新快照即自动复运动——违反 ARD §36「失败不自动恢复运动授权」。
  entry->policy_enable = false;
  // 中止配置事务并回 UNCONFIRMED 终态，确保请求收敛。
  if (entry->tx_active) {
    motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
                            now_ms);
  }
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
  entry->unified_fault_code = MOTOR_CONTROL_FAULT_CONFIG_CONFLICT;
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_CONFIG_CONFLICT;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_CONFLICT,
                           entry->motor_id, entry->bus_index,
                           MOTOR_CONTROL_FAULT_CONFIG_CONFLICT, 0U);
}

void motor_control_on_raw_frame(motor_control_context_t *ctx,
                                const motor_protocol_raw_frame_t *frame) {
  if (ctx == NULL || frame == NULL) {
    return;
  }
  ctx->rx_frame_count++;

  // 归属判定：逐 adapter 匹配（两协议 ID 段不重叠），并校验节点号范围。
  const motor_protocol_adapter_t *adapter = NULL;
  uint16_t node = 0U;
  for (size_t i = 0; i < ctx->adapter_count; ++i) {
    const motor_protocol_adapter_t *candidate = ctx->adapters[i];
    if (candidate == NULL || candidate->match_rx_frame == NULL) {
      continue;
    }
    uint16_t matched_node = 0U;
    if (candidate->match_rx_frame(frame, &matched_node) &&
        motor_protocol_adapter_valid_node(candidate, matched_node)) {
      adapter = candidate;
      node = matched_node;
      break;
    }
  }
  if (adapter == NULL) {
    ctx->unknown_frame_count++;
    return;
  }

  // 注册表定位/建档（motor_id 全局键；满表拒绝并出事件，不静默截断）。
  motor_control_entry_t *entry = motor_control_find_entry(ctx, node);
  const bool created = entry == NULL;
  bool needs_static_bind = created;
  if (created) {
    if (ctx->entry_count >= MOTOR_CONTROL_MAX_MOTORS) {
      motor_control_push_event(ctx, frame->rx_tick_ms,
                               MOTOR_CONTROL_EVENT_REGISTRY_FULL, node,
                               frame->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
      return;
    }
    entry = &ctx->entries[ctx->entry_count++];
    memset(entry, 0, sizeof(*entry));
    entry->used = true;
    entry->motor_id = node;
    entry->node = node;
    entry->bus_index = frame->bus_index;
    entry->adapter = adapter;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_DISCOVERED;
    entry->online = true;
    entry->policy_enable = true;      // 未收到 Policy 前默认使能（全局门仍生效）
    entry->motion_authorized = true;  // 全新设备的授权链路默认完整；
                                      // 故障/离线/撤权路径显式清除后不复立
    entry->last_rx_tick_ms = frame->rx_tick_ms;
    motor_control_push_event(ctx, frame->rx_tick_ms,
                             MOTOR_CONTROL_EVENT_DISCOVERED, node,
                             frame->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  } else if (entry->bus_index != frame->bus_index || entry->adapter != adapter) {
    // 同 motor_id 出现在别的总线/别的协议 → 冲突，不后到覆盖（REV-08）。
    latch_config_conflict(ctx, entry, frame->rx_tick_ms);
    return;
  } else if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
    // 冲突锁存：一致侧的帧只刷新在线观测，绝不自动解除冲突（§9.3）。
    entry->last_rx_tick_ms = frame->rx_tick_ms;
    return;
  } else if (!entry->online ||
             entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE) {
    // 重新上线：重新走 DISCOVERED 流（ARD §36），不直达 READY；
    // 旧运动授权/目标不复用（§10.4）；诊断残留一并清除。
    entry->online = true;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_DISCOVERED;
    entry->motion_enabled = false;
    entry->enable_pending = false;
    entry->has_target = false;
    entry->motion_authorized = false;
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_NONE;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_NONE;
    ctx->require_new_snapshot = true;
    needs_static_bind = true; // 重连后重新收敛绑定（profile 保留，幂等）
    motor_control_push_event(ctx, frame->rx_tick_ms,
                             MOTOR_CONTROL_EVENT_RECONNECTED, entry->motor_id,
                             entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  }

  entry->last_rx_tick_ms = frame->rx_tick_ms;
  if (!created && entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED) {
    // 非首帧的持续观测：发现 → 识别（身份确认由 Profile 绑定完成）。
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_IDENTIFYING;
  }

  // 编译期拓扑注入（下行 bind/policy/config/enable 话题已删）：建档与重连
  // 各尝试一次按名自动绑定，复用 bind 门禁与 PROFILE_*/READY 事件语义；
  // 表外电机（resolver 返回 NULL）保持未绑定不可控。绑定失败不逐帧重试——
  // 静态表是编译期事实，重试只会刷屏事件环（保持 IDENTIFYING，不可控安全侧）。
  if (needs_static_bind && ctx->model_resolver != NULL) {
    const char *model = ctx->model_resolver(node);
    if (model != NULL) {
      (void)motor_control_bind_profile_by_name(ctx, node, model,
                                               frame->rx_tick_ms);
    }
  }

  // 配置事务响应裁决（先于运动解码——读回帧同时携带反馈时两者都生效）。
  (void)motor_control_config_on_frame(ctx, entry, frame);

  // 解码 + SI 换算（Profile 未绑定时运动量为 NaN，状态/身份字段照常）。
  motor_protocol_measured_state_t measured = {0};
  if (adapter->decode_frame == NULL ||
      !adapter->decode_frame(entry->profile, node, frame, &measured)) {
    ctx->decode_error_count++;
    entry->protocol_error_count++;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROTOCOL_ERROR;
    entry->protection_flags |= MOTOR_PROTOCOL_PROT_DECODE_ERROR;
    return;
  }
  entry->measured = measured;
  entry->has_measured = true;
  if (measured.has_motion_state && isfinite(measured.position_rad)) {
    entry->last_motion_rx_tick_ms = frame->rx_tick_ms;
  }

  // 厂家故障：置位 → FAULT 禁运动 + P0 失能；清除 → 回 READY_DISABLED
  // （重新使能链路，目标已清；policy_enable 已随锁存冻结，恢复必须显式
  // SET_ENABLE(true)，ARD §36）。恢复规则细节待 TBD 关闭，此处为最小语义。
  if (measured.fault_active) {
    entry->vendor_fault_code =
        measured.vendor_fault_code != 0U ? measured.vendor_fault_code : 1U;
    entry->protection_flags |= MOTOR_PROTOCOL_PROT_VENDOR_FAULT;
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_VENDOR;
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
        entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      if (entry->motion_enabled) {
        motor_protocol_tx_frame_t frame = {0};
        if (entry->adapter->encode_disable != NULL &&
            entry->adapter->encode_disable(entry->profile, entry->node,
                                           entry->bus_index, &frame)) {
          frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
          frame.tx_slot = (uint16_t)motor_control_entry_index(ctx, entry);
          if (!motor_control_push_tx_p0(ctx, entry, &frame)) {
            entry->p0_pending = true;
            entry->p0_kind = MOTOR_CONTROL_P0_DISABLE;
          }
        }
      }
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_FAULT;
      entry->motion_enabled = false;
      entry->enable_pending = false;
      entry->has_target = false;
      entry->motion_authorized = false;
      // policy_enable 一并冻结（ARD §36）：故障清除后不得仅凭全局
      // 再授权 + 新快照自动复运动，必须显式 SET_ENABLE(true)。
      entry->policy_enable = false;
      // 中止配置事务并回 UNCONFIRMED 终态。
      if (entry->tx_active) {
        motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
                                frame->rx_tick_ms);
      }
      entry->last_failure_reason = MOTOR_CONTROL_FAULT_VENDOR;
      motor_control_push_event(ctx, frame->rx_tick_ms, MOTOR_CONTROL_EVENT_FAULT,
                               entry->motor_id, entry->bus_index,
                               MOTOR_CONTROL_FAULT_VENDOR, 0U);
    }
    return;
  }
  if ((entry->protection_flags & MOTOR_PROTOCOL_PROT_VENDOR_FAULT) != 0U) {
    entry->protection_flags &= ~MOTOR_PROTOCOL_PROT_VENDOR_FAULT;
    entry->vendor_fault_code = 0U;
    if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_VENDOR) {
      entry->unified_fault_code = MOTOR_CONTROL_FAULT_NONE;
    }
    if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT) {
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
      motor_control_push_event(ctx, frame->rx_tick_ms, MOTOR_CONTROL_EVENT_READY,
                               entry->motor_id, entry->bus_index,
                               MOTOR_CONTROL_FAULT_NONE, 0U);
    }
  }
}
