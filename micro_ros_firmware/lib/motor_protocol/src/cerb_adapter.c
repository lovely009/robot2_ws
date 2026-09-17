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
// 禁止作为 50Hz 控制触发（禁止项 14）；反馈帧（0x300+node）为命令的实时
// 状态。0x200 多控仅在显式非 MIT 模式下由 adapter 提供，本阶段不实现
// （禁止项 13）；设备侧限位必须走 request_id 事务 + 读回核验——厂家管理
// 对象未决（TBD-CONFIG-01），当前一律 UNSUPPORTED，不猜厂家字节。
//
// TBD 标注：软停 = MIT 阻尼帧保持使能（TBD-SAFETY-01 占位）；使能整定
// 100ms（既有经验行为）；OFFLINE = 5×20ms 心跳丢失（100ms）。

#include "motor_protocol_adapter.h"

#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// CANopen 快速 SDO（《CANopen-V0.9 字典表》「SDO 协议内容详解」）
//
// 帧格式（DLC 恒 8、CANFD 加速标准帧、数据小端）：
//   写请求 0x600|node : 23 idx_lo idx_hi sub d0 d1 d2 d3   （0x23 = 写 4 字节）
//   读请求 0x600|node : 40 idx_lo idx_hi sub 00 00 00 00   （0x40 = 上传请求）
//   写应答 0x580|node : 60 idx_lo idx_hi sub 00 00 00 00   （0x60 = 写成功）
//   读应答 0x580|node : 43 idx_lo idx_hi sub d0 d1 d2 d3   （0x43 = 上传 4 字节）
//   中止   0x580|node : 80 ...                              （设备拒绝）
//
// 管理对象全部 ≤4 字节，走快速 SDO，事务阶段数固定为「写 → 读回」成对，
// 写后必读回核验（ARD §20.4 / 禁止项 9）。设备身份对象 0x1008 超 4 字节，
// 走分段上传（见下 READ_IDENTITY 段）。
// ---------------------------------------------------------------------------
#define CERB_SDO_CMD_WRITE4 0x23U
#define CERB_SDO_CMD_READ4 0x40U
#define CERB_SDO_CMD_WRITE4_ACK 0x60U
#define CERB_SDO_CMD_READ4_ACK 0x43U
#define CERB_SDO_CMD_ABORT 0x80U

// 管理对象（主索引/子索引）与标度：
//   0x607D:01/02 负载位置下限/上限（INTEGER32，cnt）—— 字典 -32767..32768 cnt
//                对应 -180..180 度，即 65536 cnt / 360 度。
//   0x6080       电机最大转速（UNS32，电机端 RPM）
//   0x2538       电机峰值电流（UINT32，电机端 mA）
#define CERB_OBJ_POS_LIMIT 0x607DU
#define CERB_SUB_POS_LOWER 0x01U
#define CERB_SUB_POS_UPPER 0x02U
#define CERB_OBJ_MAX_SPEED 0x6080U
#define CERB_OBJ_MAX_CURRENT 0x2538U
#define CERB_SUB_DEFAULT 0x00U

#define CERB_SDO_TIMEOUT_MS 20U // 每阶段等待响应；超时自动重试 ≤2 次

static uint32_t cerb_read_le32(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

// 关节侧 rad → 0x607D 的 cnt（codec 反向换算的逆函数；标度唯一来源在 codec）。
static int32_t cerb_rad_to_limit_cnt(double rad) {
  return (int32_t)llround(rad * 180.0 / MOTOR_PROTOCOL_PI * (65536.0 / 360.0));
}

// 关节侧 rad/s → 0x6080 的电机端 RPM（对象是电机端量，必须乘减速比）。
static uint32_t cerb_joint_rad_s_to_motor_rpm(
    const motor_protocol_model_profile_t *profile, double joint_rad_s) {
  if (profile == NULL || !(profile->gear_ratio > 0.0f) || !(joint_rad_s > 0.0)) {
    return 0U;
  }
  const double motor_rad_s = joint_rad_s * (double)profile->gear_ratio;
  const double rpm = motor_rad_s * 60.0 / (2.0 * MOTOR_PROTOCOL_PI);
  return (uint32_t)llround(rpm);
}

// 关节侧 N·m → 0x2538 的电机端 mA。
// 关节扭矩 = Iq × Kt × gear × efficiency ⇒ Iq = τ / (Kt × gear × efficiency)。
static uint32_t cerb_joint_nm_to_motor_ma(
    const motor_protocol_model_profile_t *profile, double joint_nm) {
  if (profile == NULL || !(joint_nm > 0.0)) {
    return 0U;
  }
  const double denom = (double)profile->torque_constant_nm_per_amp *
                       (double)profile->gear_ratio *
                       (double)profile->gear_efficiency;
  if (!(denom > 0.0)) {
    return 0U;
  }
  return (uint32_t)llround(joint_nm / denom * 1000.0);
}

// CANopen SDO（厂家《CREB关节模组使用协议简易说明书》§4）：请求 0x600+node、
// 应答 0x580+node，均为 DLC 8 的 CANFD 加速标准帧（§3.1：1M 仲裁/5M 数据）。
// 身份读取对象 = 0x1008（厂家名称，分段上传），回串即型号身份。
#define CERB_SDO_REQUEST_ID_BASE 0x600U
#define CERB_SDO_RESPONSE_ID_BASE 0x580U
#define CERB_SDO_CMD_UPLOAD_INITIATE 0x40U // 上传发起（读请求）
#define CERB_SDO_CMD_UPLOAD_SEGMENT 0x60U  // 分段上传请求（bit4 = 切换位）
#define CERB_SDO_SCS_UPLOAD 0x40U       // 应答高 3 位 = 010（上传发起应答）
#define CERB_SDO_IDENTITY_INDEX 0x1008U // 厂家名称（设备身份串）
#define CERB_SDO_IDENTITY_SUBINDEX 0x00U
#define CERB_SDO_SEGMENT_BYTES 7U    // 分段上传每段有效载荷字节数
#define CERB_IDENTITY_MAX_SEGMENTS 4U // 4×7B = 28B（实测身份串 10 字符）

// 帧归属：反馈帧 0x300+node（12B）与 SDO 应答 0x580+node（8B）。
// node ∈ 1..127；两段各自用 offset ≤ 127 界定上界，互不重叠。
static bool cerb_match_rx_frame(const motor_protocol_raw_frame_t *frame,
                                uint16_t *out_node) {
  if (frame == NULL || out_node == NULL || frame->is_extended) {
    return false;
  }
  // SDO 应答（0x580|node，DLC 8）：配置事务的写应答/读回。
  // 不校验 FD 位：0x580|node 按 CANopen 定义即 SDO 应答段，FD 与否不改变
  // 归属；把它当硬条件会埋一类"总线属性一变就静默不匹配"的坑。
  if (frame->can_id >= MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE + 1U) {
    const uint32_t sdo_offset =
        frame->can_id - MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE;
    if (frame->dlc == 8U && sdo_offset <= 127U) {
      *out_node = (uint16_t)sdo_offset;
      return true;
    }
  }
  if (frame->dlc != 12U ||
      frame->can_id < MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE + 1U) {
    return false;
  }
  const uint32_t offset = frame->can_id - MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE;
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
         frame->can_id > MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE &&
         (frame->can_id - MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE) <= 127U;
}

static bool cerb_decode_frame(const motor_protocol_model_profile_t *profile,
                              uint16_t node,
                              const motor_protocol_raw_frame_t *frame,
                              motor_protocol_measured_state_t *out) {
  if (profile != NULL && profile->protocol != &motor_protocol_cerb_adapter) {
    return false;
  }
  if (frame == NULL || out == NULL || frame->is_extended ||
      frame->can_id != MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE + node ||
      frame->dlc != 12U) {
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
  // 命令 ID 走 motor_protocol.h 的规范推导（唯一来源，勿内联重算）。
  if (!motor_protocol_cerb_command_id(node, &out->can_id)) {
    return false;
  }
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
  out->can_id = MOTOR_PROTOCOL_CERB_SYNC_ID;
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
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    // 4 个加速 SDO upload：位置下限/上限、最大转速、峰值电流。
    return 4U;
  }
  (void)profile;
  switch (request->kind) {
    case MOTOR_PROTOCOL_CONFIG_SET_MODE:
      // 非 MIT 模式不受支持（ARD §25：CERB 默认且固定 MIT）。
      return request->mode == (uint8_t)MOTOR_PROTOCOL_MODE_MIT ? 1U : 0U;
    case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
      // 下限写+读回、上限写+读回。
      return 4U;
    case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
      // 写+读回。
      return 2U;
    default:
      // 增益/使能是纯内核本地项，不经设备事务。
      return 0U;
  }
}

// 阶段 → (对象索引, 子索引, 是否写, 目标原始值)。
// 阶段布局：位置限位 0=写下限 1=读下限 2=写上限 3=读上限；
// 速度/力矩 0=写 1=读回。返回 false = 阶段越界或该 kind 不走设备事务。
static bool cerb_sdo_stage_plan(
    uint8_t stage, const motor_protocol_model_profile_t *profile,
    const motor_protocol_config_request_t *request, uint16_t *out_index,
    uint8_t *out_sub, bool *out_is_write, uint32_t *out_value) {
  if (request == NULL || out_index == NULL || out_sub == NULL ||
      out_is_write == NULL || out_value == NULL) {
    return false;
  }
  const bool even = (stage % 2U) == 0U;
  switch (request->kind) {
    case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
      if (stage >= 4U) {
        return false;
      }
      *out_index = CERB_OBJ_POS_LIMIT;
      *out_sub = (stage < 2U) ? CERB_SUB_POS_LOWER : CERB_SUB_POS_UPPER;
      *out_is_write = even;
      *out_value = (uint32_t)cerb_rad_to_limit_cnt(
          (stage < 2U) ? request->position_limits.min_position_rad
                       : request->position_limits.max_position_rad);
      return true;
    case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
      if (stage >= 2U) {
        return false;
      }
      *out_index = CERB_OBJ_MAX_SPEED;
      *out_sub = CERB_SUB_DEFAULT;
      *out_is_write = even;
      *out_value = cerb_joint_rad_s_to_motor_rpm(profile,
                                                 request->max_velocity_rad_s);
      return true;
    case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
      if (stage >= 2U) {
        return false;
      }
      *out_index = CERB_OBJ_MAX_CURRENT;
      *out_sub = CERB_SUB_DEFAULT;
      *out_is_write = even;
      *out_value = cerb_joint_nm_to_motor_ma(profile, request->max_torque_nm);
      return true;
    default:
      return false;
  }
}

// REFRESH_PARAMS 只读阶段 → (对象, 子索引)。
static bool cerb_refresh_stage_plan(uint8_t stage, uint16_t *out_index,
                                    uint8_t *out_sub) {
  switch (stage) {
    case 0U:
      *out_index = CERB_OBJ_POS_LIMIT;
      *out_sub = CERB_SUB_POS_LOWER;
      return true;
    case 1U:
      *out_index = CERB_OBJ_POS_LIMIT;
      *out_sub = CERB_SUB_POS_UPPER;
      return true;
    case 2U:
      *out_index = CERB_OBJ_MAX_SPEED;
      *out_sub = CERB_SUB_DEFAULT;
      return true;
    case 3U:
      *out_index = CERB_OBJ_MAX_CURRENT;
      *out_sub = CERB_SUB_DEFAULT;
      return true;
    default:
      return false;
  }
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
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    if (stage >= 4U) {
      return false;
    }
    out->waits_response = true; // 4 个 upload，每阶段等应答
    out->timeout_ms = CERB_SDO_TIMEOUT_MS;
    return true;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    if (stage >= 1U) {
      return false;
    }
    out->waits_response = false; // 设备恒 MIT：空阶段，立即收尾
    out->timeout_ms = 0U;
    return true;
  }
  // profile 传 NULL：本函数只判阶段是否合法（不换算目标值），换算在 encode 阶段。
  uint16_t index = 0U;
  uint8_t sub = 0U;
  bool is_write = false;
  uint32_t value = 0U;
  if (!cerb_sdo_stage_plan(stage, NULL, request, &index, &sub, &is_write,
                           &value)) {
    return false;
  }
  out->waits_response = true; // 每个 SDO 阶段都等应答（写等 ack、读等回读）
  out->timeout_ms = CERB_SDO_TIMEOUT_MS;
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
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    uint16_t index = 0U;
    uint8_t sub = 0U;
    if (!cerb_refresh_stage_plan(stage, &index, &sub)) {
      return false;
    }
    uint8_t payload[8] = {0};
    payload[0] = CERB_SDO_CMD_READ4; // 0x40 上传请求
    payload[1] = (uint8_t)(index & 0xFFU);
    payload[2] = (uint8_t)(index >> 8U);
    payload[3] = sub;
    return cerb_sdo_request(node, bus_index, payload, out);
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    if (stage >= 1U) {
      return false;
    }
    memset(out, 0, sizeof(*out)); // size 0 = 空阶段（无帧），仅推进状态机
    return true;
  }
  uint16_t index = 0U;
  uint8_t sub = 0U;
  bool is_write = false;
  uint32_t value = 0U;
  if (!cerb_sdo_stage_plan(stage, profile, request, &index, &sub, &is_write,
                           &value)) {
    return false;
  }
  // 与身份读取共用同一请求构造：厂家默认「CANFD 加速 标准帧」（说明书
  // §3.1：1M 仲裁段 / 5M 数据段），SDO 一律不走经典帧。
  uint8_t payload[8] = {0};
  payload[0] = is_write ? CERB_SDO_CMD_WRITE4 : CERB_SDO_CMD_READ4;
  payload[1] = (uint8_t)(index & 0xFFU);
  payload[2] = (uint8_t)(index >> 8U);
  payload[3] = sub;
  if (is_write) {
    payload[4] = (uint8_t)(value & 0xFFU);
    payload[5] = (uint8_t)((value >> 8U) & 0xFFU);
    payload[6] = (uint8_t)((value >> 16U) & 0xFFU);
    payload[7] = (uint8_t)((value >> 24U) & 0xFFU);
  }
  return cerb_sdo_request(node, bus_index, payload, out);
}

static motor_protocol_stage_outcome_t cerb_config_on_response(
    uint8_t stage, const motor_protocol_model_profile_t *profile,
    const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response,
    motor_protocol_read_buffer_t *read_out) {
  if (request == NULL || response == NULL) {
    return MOTOR_PROTOCOL_STAGE_IGNORE;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
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
  // 设备参数回读（REFRESH_PARAMS）：4 个加速 upload 应答，回值换算成
  // 关节侧 SI 后按阶段顺序各追加一个 float（小端 4B）进读缓冲。
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    if (response->can_id != CERB_SDO_RESPONSE_ID_BASE + request->motor_id ||
        response->dlc < 8U) {
      return MOTOR_PROTOCOL_STAGE_IGNORE;
    }
    if ((response->data[0] & 0xE0U) == CERB_SDO_CMD_ABORT) {
      return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    uint16_t want_index = 0U;
    uint8_t want_sub = 0U;
    if (!cerb_refresh_stage_plan(stage, &want_index, &want_sub)) {
      return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    if (response->data[0] != CERB_SDO_CMD_READ4_ACK) {
      return MOTOR_PROTOCOL_STAGE_RETRY; // 非上传应答（迟到的写应答等）
    }
    const uint16_t got_index =
        (uint16_t)(response->data[1] | ((uint16_t)response->data[2] << 8U));
    if (got_index != want_index || response->data[3] != want_sub) {
      return MOTOR_PROTOCOL_STAGE_RETRY; // 非本阶段对象
    }
    if (profile == NULL) {
      return MOTOR_PROTOCOL_STAGE_FAIL; // 换算系数缺失（未绑定不得发起）
    }
    float value = 0.0f;
    const uint32_t raw = cerb_read_le32(&response->data[4]);
    switch (stage) {
      case 0U:
      case 1U: {
        // INTEGER32 cnt（负载端，与关节同轴）。
        const int32_t cnt = (int32_t)raw;
        value = (float)motor_protocol_cerb_limit_cnt_to_rad(cnt);
        break;
      }
      case 2U:
        value = (float)motor_protocol_cerb_motor_rpm_to_joint_rad_s(
            (double)profile->gear_ratio, raw);
        break;
      case 3U:
        value = (float)motor_protocol_cerb_motor_ma_to_joint_nm(
            (double)profile->torque_constant_nm_per_amp,
            (double)profile->gear_ratio, (double)profile->gear_efficiency, raw);
        break;
      default:
        return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    cerb_read_append(read_out, (const uint8_t *)&value, sizeof(value));
    return MOTOR_PROTOCOL_STAGE_ADVANCE;
  }
  // 设备侧管理对象：写应答 / 读回核验（0x607D、0x6080、0x2538）。
  if (response->dlc != 8U) {
    return MOTOR_PROTOCOL_STAGE_RETRY;
  }
  if (response->data[0] == CERB_SDO_CMD_ABORT) {
    return MOTOR_PROTOCOL_STAGE_FAIL; // 设备明确拒绝（含字典越界/不支持）
  }
  uint16_t index = 0U;
  uint8_t sub = 0U;
  bool is_write = false;
  uint32_t expected = 0U;
  if (!cerb_sdo_stage_plan(stage, profile, request, &index, &sub, &is_write,
                           &expected)) {
    return MOTOR_PROTOCOL_STAGE_FAIL;
  }
  const uint16_t got_index = (uint16_t)((uint16_t)response->data[1] |
                                        ((uint16_t)response->data[2] << 8U));
  if (got_index != index || response->data[3] != sub) {
    return MOTOR_PROTOCOL_STAGE_RETRY; // 非本阶段应答（可能是上一阶段的迟到帧）
  }
  if (is_write) {
    return response->data[0] == CERB_SDO_CMD_WRITE4_ACK
               ? MOTOR_PROTOCOL_STAGE_ADVANCE
               : MOTOR_PROTOCOL_STAGE_RETRY;
  }
  if (response->data[0] != CERB_SDO_CMD_READ4_ACK) {
    return MOTOR_PROTOCOL_STAGE_RETRY;
  }
  // 读回必须与写入值一致；不一致重发本阶段（≤2 次后内核收敛为 UNCONFIRMED）。
  return cerb_read_le32(&response->data[4]) == expected
             ? MOTOR_PROTOCOL_STAGE_ADVANCE
             : MOTOR_PROTOCOL_STAGE_RETRY;
}

const motor_protocol_adapter_t motor_protocol_cerb_adapter = {
    .protocol_name = "cerb",
    .node_id_min = 1U,
    .node_id_max = 127U,
    .offline_timeout_ms = 100U, // 5×20ms SYNC 上报心跳（总线检测）
    .enable_settle_ms = 100U,   // 使能后 ≥100ms 才发运动目标（既有经验行为）
    .autonomous_feedback = true, // SYNC 上报帧 = 20ms 状态心跳（承担总线检测）
    .config_verify_required = false, // 写阶段均为「写 → 读回」成对，读回核验即确认
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
