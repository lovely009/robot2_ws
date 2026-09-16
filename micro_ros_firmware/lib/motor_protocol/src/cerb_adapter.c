// CERB CAN-FD/MIT 协议 adapter（ARD-0007 §24 能力清单的 CERB 实现）。
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

#define CERB_FEEDBACK_ID_BASE 0x300U
#define CERB_COMMAND_ID_BASE 0x110U
#define CERB_SYNC_ID 0x080U

// CANopen SDO（厂家《CREB关节模组使用协议简易说明书》§4）：请求 0x600+node、
// 应答 0x580+node，均为 DLC 8 的 CANFD 加速标准帧（§3.1：1M 仲裁/5M 数据）。
// 身份读取对象 = 0x1008（厂家名称，分段上传），回串即型号身份。
#define CERB_SDO_REQUEST_ID_BASE 0x600U
#define CERB_SDO_RESPONSE_ID_BASE 0x580U
#define CERB_SDO_CMD_UPLOAD_INITIATE 0x40U // 上传发起（读请求）
#define CERB_SDO_CMD_UPLOAD_SEGMENT 0x60U  // 分段上传请求（bit4 = 切换位）
#define CERB_SDO_CMD_ABORT 0x80U
#define CERB_SDO_SCS_UPLOAD 0x40U       // 应答高 3 位 = 010（上传发起应答）
#define CERB_SDO_IDENTITY_INDEX 0x1008U // 厂家名称（设备身份串）
#define CERB_SDO_IDENTITY_SUBINDEX 0x00U
#define CERB_SDO_SEGMENT_BYTES 7U    // 分段上传每段有效载荷字节数
#define CERB_IDENTITY_MAX_SEGMENTS 4U // 4×7B = 28B（实测身份串 10 字符）
#define CERB_SDO_TIMEOUT_MS 20U      // 单阶段应答超时（设备响应在 ms 量级）

// 帧归属：反馈帧 0x300+node（12B）与 SDO 应答 0x580+node（8B）。
// node ∈ 1..127；两段各自用 offset ≤ 127 界定上界，互不重叠。
static bool cerb_match_rx_frame(const motor_protocol_raw_frame_t *frame,
                                uint16_t *out_node) {
  if (frame == NULL || out_node == NULL || frame->is_extended) {
    return false;
  }
  uint32_t base;
  if (frame->dlc == 12U) {
    base = CERB_FEEDBACK_ID_BASE;
  } else if (frame->dlc == 8U) {
    base = CERB_SDO_RESPONSE_ID_BASE;
  } else {
    return false;
  }
  if (frame->can_id <= base) {
    return false;
  }
  const uint32_t offset = frame->can_id - base;
  if (offset > 127U) {
    return false;
  }
  *out_node = (uint16_t)offset;
  return true;
}

// "设备存在"证据帧：只有 12B 周期状态上报能证明设备在线上。SDO 应答
// （0x580+node）是可归属帧但不是存在证据——否则一次杂散应答就能凭空建档。
static bool cerb_is_discovery_frame(const motor_protocol_raw_frame_t *frame) {
  return frame != NULL && !frame->is_extended && frame->dlc == 12U &&
         frame->can_id > CERB_FEEDBACK_ID_BASE &&
         (frame->can_id - CERB_FEEDBACK_ID_BASE) <= 127U;
}

static bool cerb_decode_frame(const motor_protocol_model_profile_t *profile,
                              uint16_t node,
                              const motor_protocol_raw_frame_t *frame,
                              motor_protocol_measured_state_t *out) {
  if (profile != NULL && profile->protocol != &motor_protocol_cerb_adapter) {
    return false;
  }
  if (frame == NULL || out == NULL || frame->is_extended ||
      frame->can_id != CERB_FEEDBACK_ID_BASE + node || frame->dlc != 12U) {
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
  if (!motor_protocol_unpack_cerb_feedback(frame->data, frame->dlc, &wire)) {
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

static bool cerb_encode_mit_command(const motor_protocol_model_profile_t *profile,
                                    uint16_t node, uint8_t bus_index,
                                    const motor_protocol_control_intent_t *intent,
                                    motor_protocol_tx_frame_t *out) {
  if (profile == NULL || profile->protocol != &motor_protocol_cerb_adapter ||
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
  // 分向相反——两侧行为由 test_cerb_velocity_unit_contract 固定。
  motor_protocol_joint_command_t wire = {0};
  wire.position = intent->position_rad;
  wire.velocity = intent->velocity_rad_s;
  wire.kp = intent->kp;
  wire.kd = intent->kd;
  wire.feedforward = intent->effort_nm;
  const motor_protocol_cerb_limits_t limits = {
      profile->mit_position_limit_rad,
      profile->mit_velocity_limit_rad_s,
      profile->mit_torque_limit_nm,
  };
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  out->can_id = CERB_COMMAND_ID_BASE + node;
  out->size = 12U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  motor_protocol_pack_cerb_mit_frame(&wire, &limits, intent->enable, out->data);
  return true;
}

// 中性目标帧（使能/失能共用形状，控制字区分）。
static bool cerb_encode_neutral(const motor_protocol_model_profile_t *profile,
                                uint16_t node, uint8_t bus_index, bool enable,
                                float kd, motor_protocol_tx_class_t tx_class,
                                motor_protocol_tx_frame_t *out) {
  motor_protocol_control_intent_t neutral = {0};
  neutral.kd = kd;
  neutral.enable = enable;
  if (!cerb_encode_mit_command(profile, node, bus_index, &neutral, out)) {
    return false;
  }
  out->tx_class = tx_class;
  return true;
}

static bool cerb_encode_enable(const motor_protocol_model_profile_t *profile,
                               uint16_t node, uint8_t bus_index,
                               motor_protocol_tx_frame_t *out) {
  // 使能 = 中性目标 + 使能位（0xcc）；运动目标待整定窗结束后下发。
  return cerb_encode_neutral(profile, node, bus_index, true, 0.0f,
                             MOTOR_PROTOCOL_TX_CLASS_P1, out);
}

// TBD-SAFETY-01：软停暂取阻尼帧（零目标 + 默认 kd，保持使能）。
static bool cerb_encode_soft_stop(const motor_protocol_model_profile_t *profile,
                                  uint16_t node, uint8_t bus_index,
                                  motor_protocol_tx_frame_t *out) {
  if (profile == NULL) {
    return false;
  }
  return cerb_encode_neutral(profile, node, bus_index, true, profile->default_kd,
                             MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static bool cerb_encode_disable(const motor_protocol_model_profile_t *profile,
                                uint16_t node, uint8_t bus_index,
                                motor_protocol_tx_frame_t *out) {
  // 失能 = 中性目标 + 控制字使能位清零（0x0c）。
  return cerb_encode_neutral(profile, node, bus_index, false, 0.0f,
                             MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static bool cerb_encode_discovery_sync(uint8_t bus_index,
                                       motor_protocol_tx_frame_t *out) {
  if (out == NULL) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
  out->can_id = CERB_SYNC_ID;
  out->size = 0U;
  out->is_fd = false;
  out->is_extended = false;
  out->is_brs = false;
  return true;
}

// ---------------------------------------------------------------------------
// 身份读取事务（READ_IDENTITY）：SDO 上传发起 + 分段上传
// ---------------------------------------------------------------------------

// 追加设备回传载荷；容量不足只置 overflow（由内核判失败，不静默截断）。
static void cerb_read_append(motor_protocol_read_buffer_t *buffer,
                             const uint8_t *data, size_t length) {
  if (buffer == NULL || buffer->data == NULL || data == NULL) {
    return;
  }
  for (size_t i = 0; i < length; ++i) {
    if (buffer->size >= buffer->capacity) {
      buffer->overflow = true;
      return;
    }
    buffer->data[buffer->size++] = data[i];
  }
}

// SDO 请求帧（0x600+node，DLC 8，CANFD 加速标准帧）。
static bool cerb_sdo_request(uint16_t node, uint8_t bus_index,
                             const uint8_t payload[8],
                             motor_protocol_tx_frame_t *out) {
  if (out == NULL || node < 1U || node > 127U) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P2; // 管理流量，非控制路径
  out->can_id = CERB_SDO_REQUEST_ID_BASE + node;
  out->size = 8U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  memcpy(out->data, payload, 8U);
  return true;
}

static uint8_t
cerb_config_stage_count(const motor_protocol_model_profile_t *profile,
                        const motor_protocol_config_request_t *request) {
  if (request == NULL) {
    return 0U;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    // 1 次上传发起 + 最多 4 个分段；设备提前给末段时 COMPLETE 提前收尾。
    return (uint8_t)(1U + CERB_IDENTITY_MAX_SEGMENTS);
  }
  (void)profile;
  if (request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return 0U; // 限位写等管理对象未决（TBD-CONFIG-01）→ UNSUPPORTED
  }
  if (request->mode == (uint8_t)MOTOR_PROTOCOL_MODE_MIT) {
    return 1U; // 设备恒 MIT：单个空阶段，事务立即以 OK 收尾
  }
  return 0U; // 非 MIT 模式不受支持（ARD §25：CERB 默认且固定 MIT）
}

static bool cerb_config_stage_info(uint8_t stage,
                                   const motor_protocol_config_request_t *request,
                                   motor_protocol_config_stage_info_t *out) {
  if (request == NULL || out == NULL) {
    return false;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    if (stage >= (uint8_t)(1U + CERB_IDENTITY_MAX_SEGMENTS)) {
      return false;
    }
    out->waits_response = true; // 每阶段都要等设备应答
    out->timeout_ms = CERB_SDO_TIMEOUT_MS;
    return true;
  }
  if (request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE || stage >= 1U) {
    return false;
  }
  out->waits_response = false;
  out->timeout_ms = 0U;
  return true;
}

static bool cerb_config_encode_stage(
    uint8_t stage, const motor_protocol_model_profile_t *profile, uint16_t node,
    uint8_t bus_index, const motor_protocol_config_request_t *request,
    motor_protocol_tx_frame_t *out) {
  if (request == NULL || out == NULL) {
    return false;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    if (stage >= (uint8_t)(1U + CERB_IDENTITY_MAX_SEGMENTS)) {
      return false;
    }
    uint8_t payload[8] = {0};
    if (stage == 0U) {
      // 上传发起：40 08 10 00（索引 0x1008 小端，子索引 0）
      payload[0] = CERB_SDO_CMD_UPLOAD_INITIATE;
      payload[1] = (uint8_t)(CERB_SDO_IDENTITY_INDEX & 0xFFU);
      payload[2] = (uint8_t)(CERB_SDO_IDENTITY_INDEX >> 8);
      payload[3] = CERB_SDO_IDENTITY_SUBINDEX;
    } else {
      // 分段上传请求：60/70 交替（bit4 = 切换位，从 0 起）。
      payload[0] =
          (uint8_t)(CERB_SDO_CMD_UPLOAD_SEGMENT |
                    (uint8_t)(((stage - 1U) & 1U) << 4));
    }
    return cerb_sdo_request(node, bus_index, payload, out);
  }
  if (request->kind != MOTOR_PROTOCOL_CONFIG_SET_MODE || stage >= 1U) {
    return false;
  }
  (void)profile;
  (void)node;
  (void)bus_index;
  memset(out, 0, sizeof(*out)); // size 0 = 空阶段（无帧），仅推进状态机
  return true;
}

static motor_protocol_stage_outcome_t cerb_config_on_response(
    uint8_t stage, const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response,
    motor_protocol_read_buffer_t *read_out) {
  if (request == NULL || response == NULL) {
    return MOTOR_PROTOCOL_STAGE_IGNORE;
  }
  if (request->kind != MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    return MOTOR_PROTOCOL_STAGE_ADVANCE; // 无响应阶段，不可达
  }
  // 归属过滤：同一节点的 12B 周期上报与 8B SDO 应答都会经归属匹配进来，
  // 只有 0x580+node 的 8B 帧才是本事务的应答。request->motor_id 在 adapter
  // 侧即解析后的厂家 node（内核键 = node，见 motor_protocol_adapter.h）。
  if (response->can_id != CERB_SDO_RESPONSE_ID_BASE + request->motor_id ||
      response->dlc < 8U) {
    return MOTOR_PROTOCOL_STAGE_IGNORE;
  }
  const uint8_t command = response->data[0];
  if ((command & 0xE0U) == CERB_SDO_CMD_ABORT) {
    return MOTOR_PROTOCOL_STAGE_FAIL; // 设备拒绝该对象
  }
  if (stage == 0U) {
    // 上传发起应答：scs=010。索引/子索引回带不一致 → 非本对象，读到的
    // 身份不可信，直接失败（不得拿别的对象去绑型号）。
    if ((command & 0xE0U) != CERB_SDO_SCS_UPLOAD) {
      return MOTOR_PROTOCOL_STAGE_RETRY;
    }
    if (response->data[1] != (uint8_t)(CERB_SDO_IDENTITY_INDEX & 0xFFU) ||
        response->data[2] != (uint8_t)(CERB_SDO_IDENTITY_INDEX >> 8) ||
        response->data[3] != CERB_SDO_IDENTITY_SUBINDEX) {
      return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    if ((command & 0x02U) != 0U) {
      // 加速传输：bit3-2 = 未用字节数，数据在 byte4..7。
      const uint8_t unused = (uint8_t)((command >> 2) & 0x03U);
      cerb_read_append(read_out, &response->data[4], (size_t)(4U - unused));
      return MOTOR_PROTOCOL_STAGE_COMPLETE;
    }
    return MOTOR_PROTOCOL_STAGE_ADVANCE; // 分段传输：等首个分段应答
  }
  // 分段上传应答：byte0 = 000 t nnn c（t=切换位、nnn=末段未用字节数、
  // c=末段位）；有效数据在 byte1..7。
  const uint8_t toggle = (uint8_t)((stage - 1U) & 1U);
  if ((command & 0x10U) != (uint8_t)(toggle << 4)) {
    return MOTOR_PROTOCOL_STAGE_RETRY; // 切换位不符（串段/乱序应答）
  }
  const bool last = (command & 0x01U) != 0U;
  const size_t unused = last ? (size_t)((command >> 1) & 0x07U) : 0U;
  if (unused > CERB_SDO_SEGMENT_BYTES) {
    return MOTOR_PROTOCOL_STAGE_FAIL; // 非法未用字节数
  }
  cerb_read_append(read_out, &response->data[1],
                   (size_t)CERB_SDO_SEGMENT_BYTES - unused);
  return last ? MOTOR_PROTOCOL_STAGE_COMPLETE : MOTOR_PROTOCOL_STAGE_ADVANCE;
}

const motor_protocol_adapter_t motor_protocol_cerb_adapter = {
    .protocol_name = "cerb",
    .node_id_min = 1U,
    .node_id_max = 127U,
    .offline_timeout_ms = 100U, // 5×20ms SYNC 上报心跳（总线检测）
    .enable_settle_ms = 100U,   // 使能后 ≥100ms 才发运动目标（既有经验行为）
    .autonomous_feedback = true, // SYNC 上报帧 = 20ms 状态心跳（承担总线检测）
    .config_verify_required = false, // MIT 空阶段事务无设备写入，可报 OK
    .match_rx_frame = cerb_match_rx_frame,
    .is_discovery_frame = cerb_is_discovery_frame,
    .decode_frame = cerb_decode_frame,
    .encode_mit_command = cerb_encode_mit_command,
    .encode_enable = cerb_encode_enable,
    .encode_soft_stop = cerb_encode_soft_stop,
    .encode_disable = cerb_encode_disable,
    .encode_discovery_sync = cerb_encode_discovery_sync,
    .config_stage_count = cerb_config_stage_count,
    .config_stage_info = cerb_config_stage_info,
    .config_encode_stage = cerb_config_encode_stage,
    .config_on_response = cerb_config_on_response,
};
