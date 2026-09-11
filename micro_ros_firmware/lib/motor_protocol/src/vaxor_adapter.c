// Vaxor 25/30mm 协议 adapter（ARD-0007 §24 能力清单的 vaxor 实现）。
//
// 本文件是厂家字节布局、CAN-ID 推导、帧长与 FD 属性的唯一归属地：
// 内核只见 SI 关节侧物理量与本接口。编码/解码复用 motor_protocol.h 的
// 纯 codec（已由 host test 验证），本层只做"方向 + 单位 + ID"的缝合。
//
// 单位约定（与 codec 及 motor.yaml 一致）：
//  - 线协议位置/速度为电机轴（关节侧 × 减速比）；前馈字段为 Iq 电流 A。
//  - 关节扭矩 = Iq × Kt × 减速比 × 效率（反馈方向）；
//    Iq = τ ÷ (Kt × 减速比 × 效率)（命令方向，修正旧路径 N·m/A 混用）。
//
// 反馈模型（2026-09-05 与厂家行为核实，两部分）：
//  - 12B 回显帧 @ reply-id（0x701+4n）：控制帧的实时反馈，100Hz 跟随命令；
//  - 20B 状态帧 @ status-id（0x702+4n）：20ms 周期自主上报，承担总线检测
//    （心跳），并携带电压/故障/身份字段。
//
// TBD 标注：
//  - offline_timeout_ms = 100（5×20ms 自主上报周期，TBD-SAFETY-03 占位）；
//  - 软停 = MIT 阻尼帧 {0,0,0,default_kd,0} 且保持使能（TBD-SAFETY-01 占位，
//    厂家级语义关闭前不改安全边界）；
//  - 限位 = 仅软件限位（Profile/Policy），无设备侧限位（ARD §26.1）。

#include "motor_protocol_adapter.h"

#include <math.h>
#include <string.h>

// 反馈段 ID 基址（= node 0 槽位；node 合法范围 1..60，见 motor_protocol.h）。
#define VAXOR_REPLY_ID_BASE 0x701U
#define VAXOR_STATUS_ID_BASE 0x702U

static bool vaxor_match_rx_frame(const motor_protocol_raw_frame_t *frame,
                                 uint16_t *out_node) {
  if (frame == NULL || out_node == NULL || frame->is_extended) {
    return false;
  }
  // 反馈段：reply = 0x701 + 4n（12B）、status = 0x702 + 4n（20B），n ∈ 1..60
  // （基址 0x701/0x702 对应 node 0 的槽位，node 0 非法，由 node ≥ 1 排除）。
  uint32_t id = frame->can_id;
  if (frame->dlc == 12U && id > VAXOR_REPLY_ID_BASE &&
      (id - VAXOR_REPLY_ID_BASE) % 4U == 0U) {
    *out_node = (uint16_t)((id - VAXOR_REPLY_ID_BASE) / 4U);
    return true;
  }
  if (frame->dlc == 20U && id > VAXOR_STATUS_ID_BASE &&
      (id - VAXOR_STATUS_ID_BASE) % 4U == 0U) {
    *out_node = (uint16_t)((id - VAXOR_STATUS_ID_BASE) / 4U);
    return true;
  }
  return false;
}

static void vaxor_fill_si(const motor_protocol_model_profile_t *profile,
                          const motor_protocol_joint_state_t *wire,
                          motor_protocol_measured_state_t *out) {
  // 线协议值 = 电机轴量；effort 字段由 codec 暂存 Iq(A)。
  out->current_a = (float)wire->effort;
  if (profile == NULL || profile->gear_ratio <= 0.0f) {
    out->position_rad = NAN;
    out->velocity_rad_s = NAN;
    out->effort_nm = NAN;
    return;
  }
  out->position_rad = (float)(wire->position / profile->gear_ratio);
  out->velocity_rad_s = (float)(wire->velocity / profile->gear_ratio);
  out->effort_nm = (float)(wire->effort * profile->torque_constant_nm_per_amp *
                           profile->gear_ratio * profile->gear_efficiency);
}

static bool vaxor_decode_frame(const motor_protocol_model_profile_t *profile,
                               uint16_t node,
                               const motor_protocol_raw_frame_t *frame,
                               motor_protocol_measured_state_t *out) {
  if (profile != NULL && profile->protocol != &motor_protocol_vaxor_adapter) {
    return false;
  }
  if (frame == NULL || out == NULL || frame->is_extended) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->temperature_c = NAN; // 两协议反馈均无温度来源
  out->voltage_v = NAN;
  out->position_rad = NAN;
  out->velocity_rad_s = NAN;
  out->effort_nm = NAN;

  motor_protocol_joint_state_t wire = {0};
  if (frame->can_id == motor_protocol_vaxor_reply_id(node) &&
      frame->dlc == 12U) {
    // 回显帧恒 12B（实时反馈，100Hz 跟随命令）；20B 状态帧只在 status-id。
    if (!motor_protocol_unpack_vaxor_feedback(frame->data, frame->dlc, &wire)) {
      return false;
    }
    out->has_motion_state = true;
    vaxor_fill_si(profile, &wire, out);
    return true;
  }
  if (frame->can_id == motor_protocol_vaxor_status_id(node) &&
      frame->dlc == 20U) {
    motor_protocol_vaxor_status_t status = {0};
    if (!motor_protocol_unpack_vaxor_status(frame->data, frame->dlc, &status)) {
      return false;
    }
    // 状态帧前 10 字节与 reply 同布局，运动量同样有效。
    if (!motor_protocol_unpack_vaxor_feedback(frame->data, frame->dlc, &wire)) {
      return false;
    }
    out->has_motion_state = true;
    vaxor_fill_si(profile, &wire, out);
    out->is_status_frame = true;
    out->voltage_v = (float)status.voltage_v;
    out->vendor_fault_code = status.fault;
    out->vendor_state = status.motor_state;
    out->calibration_state = status.calibration_state;
    out->drive_mode = status.mode;
    out->motor_type = status.motor_type;
    out->fault_active = status.fault != 0U;
    return true;
  }
  return false;
}

static bool vaxor_encode_mit_command(const motor_protocol_model_profile_t *profile,
                                     uint16_t node, uint8_t bus_index,
                                     const motor_protocol_control_intent_t *intent,
                                     motor_protocol_tx_frame_t *out) {
  if (profile == NULL || profile->protocol != &motor_protocol_vaxor_adapter ||
      intent == NULL || out == NULL) {
    return false;
  }
  if (!isfinite(intent->position_rad) || !isfinite(intent->velocity_rad_s) ||
      !isfinite(intent->effort_nm) || !isfinite(intent->kp) ||
      !isfinite(intent->kd) || profile->gear_ratio <= 0.0f) {
    return false;
  }
  const double torque_per_amp = (double)profile->torque_constant_nm_per_amp *
                                profile->gear_ratio * profile->gear_efficiency;
  if (torque_per_amp <= 0.0) {
    return false; // Profile 换算参数不完整，拒绝生成运动帧
  }
  motor_protocol_joint_command_t wire = {0};
  wire.position = (double)intent->position_rad * profile->gear_ratio; // → 电机轴
  wire.velocity = (double)intent->velocity_rad_s * profile->gear_ratio;
  wire.kp = intent->kp;
  wire.kd = intent->kd;
  wire.feedforward = (double)intent->effort_nm / torque_per_amp; // N·m → Iq(A)
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  out->can_id = motor_protocol_vaxor_command_id(node);
  out->size = 12U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  motor_protocol_pack_vaxor_command(&wire, out->data);
  return true;
}

static bool vaxor_encode_control(uint16_t node, uint8_t bus_index, bool run,
                                 motor_protocol_tx_class_t tx_class,
                                 motor_protocol_tx_frame_t *out) {
  if (out == NULL) {
    return false;
  }
  const motor_protocol_vaxor_control_t control = {run, false,
                                                  MOTOR_PROTOCOL_MODE_MIT};
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = tx_class;
  out->can_id = motor_protocol_vaxor_control_id(node);
  out->size = 8U;
  out->is_fd = false;
  out->is_extended = false;
  out->is_brs = false;
  motor_protocol_pack_vaxor_control(&control, out->data);
  return true;
}

static bool vaxor_encode_enable(const motor_protocol_model_profile_t *profile,
                                uint16_t node, uint8_t bus_index,
                                motor_protocol_tx_frame_t *out) {
  (void)profile;
  return vaxor_encode_control(node, bus_index, true, MOTOR_PROTOCOL_TX_CLASS_P1,
                              out);
}

// TBD-SAFETY-01：软停暂取 MIT 阻尼帧（零目标 + 默认 kd，保持使能），
// 与既有经验行为一致；升级为 100ms 硬停的路径由内核负责。
static bool vaxor_encode_soft_stop(const motor_protocol_model_profile_t *profile,
                                   uint16_t node, uint8_t bus_index,
                                   motor_protocol_tx_frame_t *out) {
  if (profile == NULL || profile->protocol != &motor_protocol_vaxor_adapter ||
      out == NULL) {
    return false;
  }
  motor_protocol_control_intent_t damping = {0};
  damping.kd = profile->default_kd;
  damping.enable = true;
  if (!vaxor_encode_mit_command(profile, node, bus_index, &damping, out)) {
    return false;
  }
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  return true;
}

static bool vaxor_encode_disable(const motor_protocol_model_profile_t *profile,
                                 uint16_t node, uint8_t bus_index,
                                 motor_protocol_tx_frame_t *out) {
  (void)profile;
  if (!vaxor_encode_control(node, bus_index, false, MOTOR_PROTOCOL_TX_CLASS_P0,
                            out)) {
    return false;
  }
  return true;
}

static bool vaxor_encode_discovery_sync(uint8_t bus_index,
                                        motor_protocol_tx_frame_t *out) {
  (void)bus_index;
  (void)out;
  return false; // vaxor 靠 ~20ms 自主上报发现，无 SYNC 帧（ARD §25）
}

static uint8_t
vaxor_config_stage_count(const motor_protocol_model_profile_t *profile,
                         const motor_protocol_config_request_t *request) {
  if (request == NULL) {
    return 0U;
  }
  (void)profile;
  // 模式切换 = 两阶段无响应事务：stop(run=0,mode) → start(run=1,mode)
  // （与既有经验行为一致；若后续确认厂家有确认对象，在 adapter 内补齐）。
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return 2U;
  }
  return 0U; // 增益是本地 Policy；限位仅软件（不走设备事务）
}

static bool vaxor_config_stage_info(uint8_t stage,
                                    const motor_protocol_config_request_t *request,
                                    motor_protocol_config_stage_info_t *out) {
  if (request == NULL || out == NULL || request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE ||
      stage >= 2U) {
    return false;
  }
  out->waits_response = false;
  out->timeout_ms = 0U;
  return true;
}

static bool vaxor_config_encode_stage(
    uint8_t stage, const motor_protocol_model_profile_t *profile, uint16_t node,
    uint8_t bus_index, const motor_protocol_config_request_t *request,
    motor_protocol_tx_frame_t *out) {
  if (request == NULL || out == NULL || request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE ||
      stage >= 2U || request->mode > (uint8_t)MOTOR_PROTOCOL_MODE_POSITION) {
    return false;
  }
  (void)profile;
  const bool run = stage == 1U;
  if (!vaxor_encode_control(node, bus_index, run, MOTOR_PROTOCOL_TX_CLASS_P2, out)) {
    return false;
  }
  out->data[2] = request->mode; // control 帧第 3 字节为模式
  return true;
}

static motor_protocol_stage_outcome_t vaxor_config_on_response(
    uint8_t stage, const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response) {
  (void)stage;
  (void)request;
  (void)response;
  return MOTOR_PROTOCOL_STAGE_ADVANCE; // 无响应阶段，不可达
}

const motor_protocol_adapter_t motor_protocol_vaxor_adapter = {
    .protocol_name = "vaxor",
    .node_id_min = 1U,
    .node_id_max = 60U,
    .offline_timeout_ms = 100U, // TBD-SAFETY-03 占位：5×20ms 自主上报
    .enable_settle_ms = 0U,     // 使能后即可发运动目标（既有经验行为）
    .autonomous_feedback = true,
    .config_verify_required = true, // 模式切换无厂家确认对象（TBD-CONFIG-01）
    .match_rx_frame = vaxor_match_rx_frame,
    .decode_frame = vaxor_decode_frame,
    .encode_mit_command = vaxor_encode_mit_command,
    .encode_enable = vaxor_encode_enable,
    .encode_soft_stop = vaxor_encode_soft_stop,
    .encode_disable = vaxor_encode_disable,
    .encode_discovery_sync = vaxor_encode_discovery_sync,
    .config_stage_count = vaxor_config_stage_count,
    .config_stage_info = vaxor_config_stage_info,
    .config_encode_stage = vaxor_config_encode_stage,
    .config_on_response = vaxor_config_on_response,
};
