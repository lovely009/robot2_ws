// CREB CAN-FD/MIT 协议 adapter（ARD-0007 §24 能力清单的 CREB 实现）。
//
// 单位约定（命令与反馈分向）：
//  - 命令（encode）：关节侧 rad/s 直通 MIT 12 位对称量化，上限 = Profile
//    mit_velocity_limit_rad_s（π）。命令侧不乘减速比——若按 ×gear 换算到
//    电机侧，正常关节速度（×101）将远超 π 满码饱和失真。
//  - 反馈（decode）：线协议速度为电机侧 rpm（codec 转 rad/s）→ ÷ 减速比
//    → 关节侧 rad/s；位置为关节侧原样；反馈 Iq(A) × Kt × 减速比 → 关节扭矩。
//  - MIT 量化范围（π/π/峰值）来自 Profile，与真机参数页一致。
//
// 能力边界（ARD §25 + 2026-09-05 协议核实）：0x080 SYNC 触发设备 20ms 周期
// 状态上报——上报帧即心跳（承担总线检测），因此 SYNC 心跳属管理性使用，
// 禁止作为 100Hz 控制触发（禁止项 14）；反馈帧（0x300+node）为命令的实时
// 状态。0x200 多控仅在显式非 MIT 模式下由 adapter 提供，本阶段不实现
// （禁止项 13）；设备侧限位必须走 request_id 事务 + 读回核验——厂家管理
// 对象未决（TBD-CONFIG-01），当前一律 UNSUPPORTED，不猜厂家字节。
//
// TBD 标注：软停 = MIT 阻尼帧保持使能（TBD-SAFETY-01 占位）；使能整定
// 100ms（既有经验行为）；OFFLINE = 5×20ms 心跳丢失（100ms）。

#include "motor_protocol_adapter.h"

#include <math.h>
#include <string.h>

#define CREB_FEEDBACK_ID_BASE 0x300U
#define CREB_COMMAND_ID_BASE 0x110U
#define CREB_SYNC_ID 0x080U

static bool creb_match_rx_frame(const motor_protocol_raw_frame_t *frame,
                                uint16_t *out_node) {
  if (frame == NULL || out_node == NULL || frame->is_extended) {
    return false;
  }
  if (frame->dlc != 12U || frame->can_id < CREB_FEEDBACK_ID_BASE + 1U) {
    return false;
  }
  const uint32_t offset = frame->can_id - CREB_FEEDBACK_ID_BASE;
  if (offset > 127U) {
    return false;
  }
  *out_node = (uint16_t)offset;
  return true;
}

static bool creb_decode_frame(const motor_protocol_model_profile_t *profile,
                              uint16_t node,
                              const motor_protocol_raw_frame_t *frame,
                              motor_protocol_measured_state_t *out) {
  if (profile != NULL && profile->protocol != &motor_protocol_creb_adapter) {
    return false;
  }
  if (frame == NULL || out == NULL || frame->is_extended ||
      frame->can_id != CREB_FEEDBACK_ID_BASE + node || frame->dlc != 12U) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->temperature_c = NAN; // 无温度来源
  out->voltage_v = NAN;
  out->position_rad = NAN;
  out->velocity_rad_s = NAN;
  out->effort_nm = NAN;

  // 故障位（data[11] bit5）帧本身有效：标注故障、运动量缺失，不当作坏帧丢。
  out->fault_active = (frame->data[11] & 0x20U) != 0U;
  if (out->fault_active) {
    out->vendor_fault_code = 1U;
    return true;
  }

  motor_protocol_joint_state_t wire = {0.0, 0.0, 0.0};
  if (!motor_protocol_unpack_creb_feedback(frame->data, frame->dlc, &wire)) {
    return false;
  }
  out->has_motion_state = true;
  out->position_rad = (float)wire.position; // 关节侧原样
  out->current_a = (float)wire.effort;      // codec 暂存 Iq(A)
  if (profile != NULL && profile->gear_ratio > 0.0f) {
    out->velocity_rad_s =
        (float)(wire.velocity / profile->gear_ratio); // 电机侧 → 关节侧
    out->effort_nm = (float)(wire.effort * profile->torque_constant_nm_per_amp *
                             profile->gear_ratio * profile->gear_efficiency);
  } else {
    out->velocity_rad_s = NAN;
    out->effort_nm = NAN;
  }
  return true;
}

static bool creb_encode_mit_command(const motor_protocol_model_profile_t *profile,
                                    uint16_t node, uint8_t bus_index,
                                    const motor_protocol_control_intent_t *intent,
                                    motor_protocol_tx_frame_t *out) {
  if (profile == NULL || profile->protocol != &motor_protocol_creb_adapter ||
      intent == NULL || out == NULL) {
    return false;
  }
  if (!isfinite(intent->position_rad) || !isfinite(intent->velocity_rad_s) ||
      !isfinite(intent->effort_nm) || !isfinite(intent->kp) ||
      !isfinite(intent->kd) || profile->mit_position_limit_rad <= 0.0f ||
      profile->mit_velocity_limit_rad_s <= 0.0f ||
      profile->mit_torque_limit_nm <= 0.0f) {
    return false;
  }
  // 命令线协议即关节侧（速度含内）：位置/速度/扭矩直通 MIT 量化，上限来自
  // Profile。命令速度不乘减速比（直通 π 上限量化），与反馈侧 rpm ÷gear
  // 分向相反——两侧行为由 test_creb_velocity_unit_contract 固定。
  motor_protocol_joint_command_t wire = {0};
  wire.position = intent->position_rad;
  wire.velocity = intent->velocity_rad_s;
  wire.kp = intent->kp;
  wire.kd = intent->kd;
  wire.feedforward = intent->effort_nm;
  const motor_protocol_creb_limits_t limits = {
      profile->mit_position_limit_rad,
      profile->mit_velocity_limit_rad_s,
      profile->mit_torque_limit_nm,
  };
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  out->can_id = CREB_COMMAND_ID_BASE + node;
  out->size = 12U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  motor_protocol_pack_creb_mit_frame(&wire, &limits, intent->enable, out->data);
  return true;
}

// 中性目标帧（使能/失能共用形状，控制字区分）。
static bool creb_encode_neutral(const motor_protocol_model_profile_t *profile,
                                uint16_t node, uint8_t bus_index, bool enable,
                                float kd, motor_protocol_tx_class_t tx_class,
                                motor_protocol_tx_frame_t *out) {
  motor_protocol_control_intent_t neutral = {0};
  neutral.kd = kd;
  neutral.enable = enable;
  if (!creb_encode_mit_command(profile, node, bus_index, &neutral, out)) {
    return false;
  }
  out->tx_class = tx_class;
  return true;
}

static bool creb_encode_enable(const motor_protocol_model_profile_t *profile,
                               uint16_t node, uint8_t bus_index,
                               motor_protocol_tx_frame_t *out) {
  // 使能 = 中性目标 + 使能位（0xcc）；运动目标待整定窗结束后下发。
  return creb_encode_neutral(profile, node, bus_index, true, 0.0f,
                             MOTOR_PROTOCOL_TX_CLASS_P1, out);
}

// TBD-SAFETY-01：软停暂取阻尼帧（零目标 + 默认 kd，保持使能）。
static bool creb_encode_soft_stop(const motor_protocol_model_profile_t *profile,
                                  uint16_t node, uint8_t bus_index,
                                  motor_protocol_tx_frame_t *out) {
  if (profile == NULL) {
    return false;
  }
  return creb_encode_neutral(profile, node, bus_index, true, profile->default_kd,
                             MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static bool creb_encode_disable(const motor_protocol_model_profile_t *profile,
                                uint16_t node, uint8_t bus_index,
                                motor_protocol_tx_frame_t *out) {
  // 失能 = 中性目标 + 控制字使能位清零（0x0c）。
  return creb_encode_neutral(profile, node, bus_index, false, 0.0f,
                             MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static bool creb_encode_discovery_sync(uint8_t bus_index,
                                       motor_protocol_tx_frame_t *out) {
  if (out == NULL) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
  out->can_id = CREB_SYNC_ID;
  out->size = 0U;
  out->is_fd = false;
  out->is_extended = false;
  out->is_brs = false;
  return true;
}

static uint8_t
creb_config_stage_count(const motor_protocol_model_profile_t *profile,
                        const motor_protocol_config_request_t *request) {
  if (request == NULL) {
    return 0U;
  }
  (void)profile;
  if (request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return 0U; // 限位写等管理对象未决（TBD-CONFIG-01）→ UNSUPPORTED
  }
  if (request->mode == (uint8_t)MOTOR_PROTOCOL_MODE_MIT) {
    return 1U; // 设备恒 MIT：单个空阶段，事务立即以 OK 收尾
  }
  return 0U; // 非 MIT 模式不受支持（ARD §25：CREB 默认且固定 MIT）
}

static bool creb_config_stage_info(uint8_t stage,
                                   const motor_protocol_config_request_t *request,
                                   motor_protocol_config_stage_info_t *out) {
  if (request == NULL || out == NULL || request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE ||
      stage >= 1U) {
    return false;
  }
  out->waits_response = false;
  out->timeout_ms = 0U;
  return true;
}

static bool creb_config_encode_stage(
    uint8_t stage, const motor_protocol_model_profile_t *profile, uint16_t node,
    uint8_t bus_index, const motor_protocol_config_request_t *request,
    motor_protocol_tx_frame_t *out) {
  if (request == NULL || out == NULL || request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE ||
      stage >= 1U) {
    return false;
  }
  (void)profile;
  (void)node;
  (void)bus_index;
  memset(out, 0, sizeof(*out)); // size 0 = 空阶段（无帧），仅推进状态机
  return true;
}

static motor_protocol_stage_outcome_t creb_config_on_response(
    uint8_t stage, const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response) {
  (void)stage;
  (void)request;
  (void)response;
  return MOTOR_PROTOCOL_STAGE_ADVANCE; // 无响应阶段，不可达
}

const motor_protocol_adapter_t motor_protocol_creb_adapter = {
    .protocol_name = "creb",
    .node_id_min = 1U,
    .node_id_max = 127U,
    .offline_timeout_ms = 100U, // 5×20ms SYNC 上报心跳（总线检测）
    .enable_settle_ms = 100U,   // 使能后 ≥100ms 才发运动目标（既有经验行为）
    .autonomous_feedback = true, // SYNC 上报帧 = 20ms 状态心跳（承担总线检测）
    .config_verify_required = false, // MIT 空阶段事务无设备写入，可报 OK
    .match_rx_frame = creb_match_rx_frame,
    .decode_frame = creb_decode_frame,
    .encode_mit_command = creb_encode_mit_command,
    .encode_enable = creb_encode_enable,
    .encode_soft_stop = creb_encode_soft_stop,
    .encode_disable = creb_encode_disable,
    .encode_discovery_sync = creb_encode_discovery_sync,
    .config_stage_count = creb_config_stage_count,
    .config_stage_info = creb_config_stage_info,
    .config_encode_stage = creb_config_encode_stage,
    .config_on_response = creb_config_on_response,
};
