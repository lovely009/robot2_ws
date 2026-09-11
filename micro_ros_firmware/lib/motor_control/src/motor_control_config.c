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

// ---------------------------------------------------------------------------
// 事务收尾与步进
// ---------------------------------------------------------------------------

void motor_control_finish_tx(motor_control_context_t *ctx,
                             motor_control_entry_t *entry, uint16_t result,
                             uint32_t now_ms) {
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
}
// 本函数也被撤权、离线、冲突和故障路径复用；这些路径均须回
// UNCONFIRMED 终态，避免上层请求悬挂。

// 事务成功收尾：无确认手段的协议不得谎报 OK（ARD §20.5 / 禁止项 9）。
static void finish_tx_success(motor_control_context_t *ctx,
                              motor_control_entry_t *entry, uint32_t now_ms) {
  const uint16_t result =
      entry->adapter != NULL && entry->adapter->config_verify_required
          ? (uint16_t)MOTOR_PROTOCOL_CONFIG_UNCONFIRMED
          : (uint16_t)MOTOR_PROTOCOL_CONFIG_OK;
  motor_control_finish_tx(ctx, entry, result, now_ms);
}

// 超时/重试判定先行，随后至多推进一个阶段（跨节拍步进，ARD §20.6）。
void motor_control_config_step(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
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
    motor_protocol_config_stage_info_t info = {0};
    if (adapter->config_stage_info == NULL ||
        !adapter->config_stage_info(entry->tx_stage, &entry->tx_request, &info)) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR, now_ms);
      continue;
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
  if (!adapter->config_stage_info(entry->tx_stage, &entry->tx_request, &info) ||
      !info.waits_response) {
    return false;
  }
  switch (adapter->config_on_response(entry->tx_stage, &entry->tx_request, frame)) {
    case MOTOR_PROTOCOL_STAGE_ADVANCE:
      entry->tx_stage++;
      entry->tx_retries = 0U;
      entry->tx_awaiting = false;
      if (entry->tx_stage >=
          adapter->config_stage_count(entry->profile, &entry->tx_request)) {
        finish_tx_success(ctx, entry, frame->rx_tick_ms);
      }
      return true;
    case MOTOR_PROTOCOL_STAGE_RETRY:
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
  return true;
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
      if (entry->profile != NULL &&
          (entry->profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
        // 软件限位协议（Vaxor）：本地保存，打包前钳制（ARD §26.1）。
        entry->soft_min_rad = request->position_limits.min_position_rad;
        entry->soft_max_rad = request->position_limits.max_position_rad;
        entry->has_soft_limits = true;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
        return true;
      }
      // 设备侧限位（CREB）：必须走 request_id 事务 + 读回核验（ARD §26.2）。
      if (entry->tx_active) {
        ctx->config_rejected_count++;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_BUSY, request->request_id);
        return false;
      }
      return start_tx(ctx, entry, request, now_ms);
    }
    case MOTOR_PROTOCOL_CONFIG_SET_ENABLE: {
      // 逐电机 Enable（ARD §19）：本地策略位，非事务。
      // true = 重立运动授权（仍需新快照才能运动）；false = 立即退出运动：
      // 软停 + 失能帧 + 清授权位（ARD §36 冻结语义）。
      if (request->enable) {
        entry->policy_enable = true;
        entry->motion_authorized = true;
      } else {
        entry->policy_enable = false;
        entry->motion_authorized = false;
        entry->has_target = false;
        if (entry->motion_enabled) {
          if (entry->adapter != NULL && entry->adapter->encode_disable != NULL) {
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
        ctx->config_rejected_count++;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_BUSY, request->request_id);
        return false;
      }
      return start_tx(ctx, entry, request, now_ms);
    }
    default:
      return reject_invalid(ctx, entry, request, now_ms);
  }
}
