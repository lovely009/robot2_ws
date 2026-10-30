// motor_control 发现与注册：原始帧 → adapter 归属 → 注册表条目 → 解码 → 生命周期/冲突/故障处理。
//
// 注册表以全局唯一 uint16 motor_id 为键（当前派生自厂家 node 号，由部署保证全局唯一）。同 motor_id
// 跨总线/跨协议身份不符 → CONFIG_CONFLICT：禁运动、出事件，**禁止后到数据覆盖旧表项**；冲突态穿越
// OFFLINE/重连保持锁存，不存在自动解除路径（只能整机重启或未来的显式运维接口）。
// 重连重新走 DISCOVERED 流（不直达 READY）：旧运动授权与目标被清除，Profile 保留时由 service 的
// 识别流推进收敛到 READY_DISABLED。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// 冲突置位：禁运动 + P0 失能帧（推送失败挂起重试）+ 事件（只触发一次）。
static void latch_config_conflict(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
    return;
  }
  if (entry->motion_enabled) {
    // 失能帧生成/推送与失败挂起统一在 motor_control_send_p0 内。
    (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
  }
  entry->motion_enabled = false;
  entry->enable_pending = false;
  entry->has_target = false;
  entry->motion_authorized = false;
  // policy_enable 一并冻结：即使全局授权与新快照随后到达，也必须等待
  // motion_authorized，若 policy_enable 残留 true，锁存清除后仅重发全局授权
  // + 新快照即自动复运动——违反 ARD §36「失败不自动恢复运动授权」。
  entry->policy_enable = false;
  motor_control_abort_tx(ctx, entry, now_ms);
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
  const bool presence = adapter->is_discovery_frame == NULL ||
      adapter->is_discovery_frame(frame);
  motor_control_entry_t *entry = motor_control_find_entry(ctx, node);
  const bool created = entry == NULL;
  if (!presence) {
    if (created) {
      // 可归属但不是"设备存在"证据（CERB SDO 应答）：没有表项就无处裁决，
      // 且不得凭一次杂散事务应答凭空造出一台电机。
      ctx->unknown_frame_count++;
      return;
    }
    // 已知设备的事务应答：只刷新链路新鲜度并服务事务裁决，不推进生命周期、
    // 不解码——它不是周期状态帧，走解码只会把协议错误计数刷成噪声。
    entry->last_rx_tick_ms = frame->rx_tick_ms;
    (void)motor_control_config_on_frame(ctx, entry, frame);
    return;
  }
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
    entry->policy_enable = false;     // 新发现设备必须收到显式、带租约的 Policy。
    entry->motion_authorized = false;
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
    // 重连后重新识别：身份是设备事实，可能在换机/改接线后已变；旧 profile
    // 与旧读数一并作废（identity_read_done 复位 → service 复检时重发）。
    entry->identity_read_done = false;
    entry->identity_read_attempts = 0U;
    entry->identity_read_retry_tick_ms = 0U;
    entry->identity_size = 0U;
    entry->identity_read_overflow = false;
    entry->profile = NULL;
    // 设备参数**结论**一并作废：重新上线要重新回读设备并比对（换机/厂家工具
    // 都可能改过设备里的值）。旧影子的换算值属于旧型号，不许留用。
    // 注意只清"本轮会话结论"，**不清 Flash 记录**（record_*）：记录是电机侧
    // 的持久事实，且只在 boot 首次落袋时捕获一次——在这里清掉就再也补不回来，
    // 之后的重连都会变成"无记录可比"，不一致会被静默放过。
    entry->dev_limits_valid = false;
    entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_NONE;
    entry->dev_raw_present = 0U;
    entry->dev_read_done = false;
    entry->dev_read_attempts = 0U;
    entry->dev_read_retry_tick_ms = 0U;
    // 归一结论与设备影子同批作废：重连后要重新把易失对象写回定态。
    entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_PROTECT;
    entry->boot_norm_request_id = 0U;
    entry->boot_norm_attempts = 0U;
    entry->boot_norm_retry_tick_ms = 0U;
    entry->dev_init = MOTOR_CONTROL_DEV_INIT_PENDING;
    motor_control_push_event(ctx, frame->rx_tick_ms,
        MOTOR_CONTROL_EVENT_RECONNECTED, entry->motor_id,
        entry->bus_index, MOTOR_CONTROL_FAULT_NONE, 0U);
  }

  entry->last_rx_tick_ms = frame->rx_tick_ms;
  if (!created && entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED) {
    // 非首帧的持续观测：发现 → 识别（身份确认由 Profile 绑定完成）。
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_IDENTIFYING;
  }

  // 型号绑定（唯一来源：设备自报身份）不在本路径发起：识别由
  // motor_control_config_step 按不变量复检（在线 + 可绑定态 + 本轮未定论），
  // 覆盖建档、重连、以及被离线/撤权中止后的重发。

  // 配置事务响应裁决（先于运动解码）。返回 true = 帧已被事务消费：SDO 应答
  // （0x580|node）既非运动反馈、也不满足厂家反馈帧型，继续送解码只会把它
  // 计成协议错误并置 DECODE_ERROR 故障位——故消费后直接返回。
  // vaxor 无响应阶段（waits_response 恒 false），此分支对它不可达。
  if (motor_control_config_on_frame(ctx, entry, frame)) {
    return;
  }

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
        (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
      }
      entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_FAULT;
      entry->motion_enabled = false;
      entry->enable_pending = false;
      entry->has_target = false;
      entry->motion_authorized = false;
      // policy_enable 一并冻结（ARD §36）：故障清除后不得仅凭全局
      // 再授权 + 新快照自动复运动，必须显式 SET_ENABLE(true)。
      entry->policy_enable = false;
      motor_control_abort_tx(ctx, entry, frame->rx_tick_ms);
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
