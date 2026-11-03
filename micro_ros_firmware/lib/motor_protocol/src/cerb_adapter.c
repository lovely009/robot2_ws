// CERB CAN-FD/MIT adapter：命令位置/速度/增益为关节侧，量化范围来自 Profile。
// 反馈速度由电机端 rpm 换算后除减速比；Iq × Kt × 减速比 × 效率得到关节转矩。
// 0x080 SYNC 只触发反馈，不作为运动控制边界；P1 按总线聚合为 0x210。
// 软停使用保持使能的 MIT 阻尼帧，设备配置事务必须关联 request_id 并核验读回。

#include "motor_protocol_adapter.h"

#include <math.h>
#include <string.h>

// ===== CERB：CANopen SDO、MIT/PDO、反馈与设备参数事务 =====
// 快速 SDO（《CANopen-V0.9 字典表》「SDO 协议内容详解」）：DLC 恒 8、CANFD 加速标准帧、数据小端。
// 写请求 0x600|node 首字节 0x23、读请求 0x40、写应答 0x580|node 首字节 0x60、读应答 0x43、中止 0x80。
// 管理对象全部 ≤4 字节，事务固定「写 → 读回核验」成对；身份对象 0x1008 超 4 字节走分段上传
// （见下 READ_IDENTITY 段）。
#define CERB_SDO_CCS_WRITE4 0x23U
#define CERB_SDO_CCS_UPLOAD_INITIATE 0x40U
#define CERB_SDO_SCS_WRITE_ACK 0x60U
#define CERB_SDO_SCS_UPLOAD_4BYTE 0x43U
#define CERB_SDO_CMD_ABORT 0x80U

// 管理对象（主索引/子索引）与标度：
//   0x607D:01/02 负载位置下限/上限（INTEGER32，cnt），65536 cnt/360 度。
//     子索引以字典和实测为准：:01 下限 -32768、:02 上限 +32768；说明书读取行相反。
//   0x6080 电机最大转速（UNS32，电机端 RPM）；0x2538 电机峰值电流（UINT32，电机端 mA）。
#define CERB_OBJ_POS_LIMIT 0x607DU
#define CERB_SUB_POS_LOWER 0x01U
#define CERB_SUB_POS_UPPER 0x02U
#define CERB_OBJ_MAX_SPEED 0x6080U
#define CERB_OBJ_MAX_CURRENT 0x2538U
// 0x2531「设置电机输出端零位」（字典表 RW，范围 0~2）：写 1 = 设置零位；
// 手册注「设置时电机需要在失能状态」。命令式对象：只等 SDO 写应答，不读回。
#define CERB_OBJ_SET_ZERO 0x2531U
#define CERB_SET_ZERO_CODE 1U
// 0x2539 保存码：1=电机参数（电流/速度/PI），2=正限位，4=负限位。
// 由独立 SAVE 事务触发；不保存则驱动器掉电恢复原值。
#define CERB_OBJ_SAVE_FLASH 0x2539U
#define CERB_SAVE_CODE_PARAMS 1U
#define CERB_SAVE_CODE_POS_UPPER 2U
#define CERB_SAVE_CODE_POS_LOWER 4U

// 0x2650:00：0=保护生效，1=关闭看门狗及限位；固件无软限位兜底。
// 手册称帧间隔 >500ms 自锁，实际看门狗行为须台架验证。
// 对象易失，0x2539 无效，须每次上电归一；仅 :00 可读，:01/:02 返回 ABORT 0x06090011。
#define CERB_OBJ_SAFETY_PROTECT 0x2650U
#define CERB_SUB_SAFETY_PROTECT 0x00U
// 0x2541：0=CW、1=CCW，仅改驱动力矩方向，不改位置坐标系、零点或限位。
// 警告：翻向可能使位置环正反馈，使能后向远离目标的方向运动。
#define CERB_OBJ_MOTION_DIR 0x2541U
#define CERB_OBJ_T_MAX 0x2546U
#define CERB_OBJ_NODE_ID 0x2530U

// 位置限位字典范围（0x607D:01/:02 均为 -32767~32768 cnt，默认 -32767/32768）：
// -π rad 折算是 -32768，比字典下限低 1 cnt，写下去可能被设备拒绝，故按字典钳。
#define CERB_POS_CNT_MIN (-32767)
#define CERB_POS_CNT_MAX (32768)
#define CERB_SUB_DEFAULT 0x00U

// SDO 阶段超时 30ms，至多重试 2 次；不是运动周期。
// 同总线必须一问一答串行，过短超时会造成重复请求拥塞。
#define CERB_SDO_TIMEOUT_MS 30U

static uint32_t cerb_read_le32(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

// 关节侧 rad → 0x607D 的 cnt（codec 反向换算的逆函数；标度唯一来源在 codec）。
// 结果按字典范围钳制：-π 折算为 -32768，超出字典下限 1 cnt。
static int32_t cerb_rad_to_limit_cnt(double rad) {
  const int32_t cnt =
      (int32_t)llround(rad * 180.0 / MOTOR_PROTOCOL_PI * (65536.0 / 360.0));
  if (cnt < CERB_POS_CNT_MIN) {
    return CERB_POS_CNT_MIN;
  }
  if (cnt > CERB_POS_CNT_MAX) {
    return CERB_POS_CNT_MAX;
  }
  return cnt;
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

// 关节侧 N·m → 0x2538 电机端 mA，优先用规格书峰值扭矩/电流比。
static uint32_t cerb_joint_nm_to_motor_ma(
    const motor_protocol_model_profile_t *profile, double joint_nm) {
  if (profile == NULL) {
    return 0U;
  }
  return motor_protocol_limit_joint_nm_to_ma(
      (double)motor_protocol_profile_limit_nm_per_amp(profile), joint_nm);
}

// CANopen SDO（厂家《CREB关节模组使用协议简易说明书》§4）：请求 0x600+node、
// 应答 0x580+node，均为 DLC 8 的 CANFD 加速标准帧（§3.1：1M 仲裁/5M 数据）。
// 身份读取对象 = 0x1008（厂家名称，分段上传），回串即型号身份。
#define CERB_SDO_CCS_UPLOAD_SEGMENT 0x60U  // 分段上传请求（bit4 = 切换位）
#define CERB_SDO_SCS_UPLOAD 0x40U       // 应答高 3 位 = 010（上传发起应答）
#define CERB_SDO_IDENTITY_INDEX 0x1008U // 厂家名称（设备身份串）
#define CERB_SDO_IDENTITY_SUBINDEX 0x00U
#define CERB_SDO_SEGMENT_BYTES 7U    // 分段上传每段有效载荷字节数
#define CERB_IDENTITY_MAX_SEGMENTS 4U // 4×7B = 28B（实测身份串 10 字符）

// 反馈帧 data[6..7] 错误代码（大端 uint16，含义查厂家「报错代码表」，固件只透传
// 不解释）。调用方保证帧形已成立（DLC=12）。
static uint32_t cerb_error_code(const motor_protocol_raw_frame_t *frame) {
  return ((uint32_t)frame->data[6] << 8U) | (uint32_t)frame->data[7];
}

// 帧归属：反馈帧 0x300+node（12B）与 SDO 应答 0x580+node（8B）。
// node ∈ 1..127；两段各自用 offset ≤ 127 界定上界，互不重叠。
static bool cerb_match_rx_frame(const motor_protocol_raw_frame_t *frame,
    uint16_t *out_node) {
  if (frame == NULL || out_node == NULL || frame->is_extended) {
    return false;
  }
  // SDO 归属按 ID 与 DLC 判断，不要求 FD 位。
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
  // 故障帧温度仍有效，须先于故障分支读取。
  out->temperature_c =
      motor_protocol_cerb_feedback_temperature_c(frame->data, frame->dlc);
  out->voltage_v = NAN;
  out->position_rad = NAN;
  out->velocity_rad_s = NAN;
  out->effort_nm = NAN;

  // 故障帧有效但运动量缺失；仅故障位有效时透传代码，避免残留代码阻止故障清除。
  out->fault_active = (frame->data[11] & 0x20U) != 0U;
  if (out->fault_active) {
    out->vendor_fault_code = cerb_error_code(frame);
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
      profile->mit_torque_limit_nm <= 0.0f ||
      profile->mit_kp_limit <= 0.0f || profile->mit_kd_limit <= 0.0f ||
      !isfinite(profile->mit_kp_limit) || !isfinite(profile->mit_kd_limit)) {
    return false;
  }
  // 命令为关节侧量，速度不乘减速比；反馈侧 RPM 换算才除以减速比。
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
    profile->mit_kp_limit,
    profile->mit_kd_limit,
  };
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  out->node = (uint8_t)node;
  if (motor_protocol_cerb_single_node_valid(node)) {
    // 命令 ID 走 motor_protocol.h 的规范推导（唯一来源，勿内联重算）。
    if (!motor_protocol_cerb_command_id(node, &out->can_id)) {
      return false;
    }
  } else if (motor_protocol_cerb_node_valid(node)) {
    // NodeID>15 的单轴 ID 有歧义：只产出 0x210 成员帧，聚合前不得上线。
    out->can_id = MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID;
    out->aggregate_only = true;
  } else {
    return false;
  }
  out->size = 12U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  if (!motor_protocol_pack_cerb_mit_frame(&wire, &limits, intent->enable,
      out->data)) {
    return false;
  }
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
  // P0 无聚合器；高节点必须直接展开为单槽 0x210，不得发送占位帧或歧义单轴 ID。
  if (out->aggregate_only) {
    motor_protocol_cerb_multi_axis_t axis = {0};
    axis.node = out->node;
    memcpy(axis.data, out->data, sizeof(axis.data));
    out->can_id = MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID;
    out->size = MOTOR_PROTOCOL_MAX_FRAME_BYTES;
    out->aggregate_only = false;
    if (!motor_protocol_pack_cerb_multi_mit(&axis, 1U, out->data)) {
      return false;
    }
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

// SYNC 使用 FD+BRS，保证也经过数据相位；DLC=0 的 CRC 等仍使用数据速率。
// 真机是否接受 FD SYNC 尚未验证，说明书仅明确 SDO 格式。
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
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
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
  out->can_id = MOTOR_PROTOCOL_CERB_SDO_REQUEST_ID_BASE + node;
  out->size = 8U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  memcpy(out->data, payload, 8U);
  return true;
}

// 配置阶段表统一描述对象、动作、保存码与原值槽；身份上传与空模式阶段单独处理。

enum {
  CERB_STAGE_SDO_WRITE, // 快速写 0x23，等写应答 0x60
  CERB_STAGE_SDO_READ,  // 快速上传 0x40，等 0x43 回读并与写值比对
  CERB_STAGE_SAVE,      // 写 0x2539 保存码，无应答，发完即推进
  CERB_STAGE_MIT_CLEAR, // MIT 控制字置 bit5 清错，无应答，发完即推进
};

// MIT 下行控制字 bit5 = 清错（巨蟹 CERB 通信协议；硬件资料入口见仓库 docs/README.md）。
// 固件的常规 MIT 组帧只置 bit7(使能)/bit6(抱闸释放)/bit4:1(模式)，从不置
// 这一位——它是唯一的主动清错手段。
#define CERB_MIT_CTRL_CLEAR_FAULT 0x20U

// 写阶段的取值来源（读/保存阶段为 NONE）。
enum {
  CERB_VALUE_NONE,
  CERB_VALUE_ZERO,       // 0x2531=1 常量
  CERB_VALUE_POS_LOWER,  // rad → cnt（位置下限）
  CERB_VALUE_POS_UPPER,
  CERB_VALUE_SPEED,      // 关节侧 rad/s → 电机端 RPM
  CERB_VALUE_CURRENT,    // 关节侧 N·m → 峰值电流 mA
  CERB_VALUE_PASSTHROUGH, // 读：原值即 SI（安全开关/方向，无单位换算）
  CERB_VALUE_SAFETY_RAW,  // 写/读：0x2650:00 原值（0=保护开，1=关），不换算
  CERB_VALUE_T_MAX_RAW,
  CERB_VALUE_NODE_ID_RAW,
  CERB_VALUE_DIRECTION_RAW, // 写/读：0x2541:00 原值（0=CW，1=CCW），不换算
};

typedef struct {
  uint8_t action;      // CERB_STAGE_*
  uint8_t value_kind;  // CERB_VALUE_*
  uint8_t param_slot;  // MOTOR_PROTOCOL_PARAM_SLOT_*（读回落袋/写登记槽位）
  uint16_t index;      // SDO 主索引
  uint8_t sub;         // SDO 子索引
  uint8_t save_code;   // CERB_STAGE_SAVE 的 0x2539 保存码（1/2/4）
} cerb_stage_desc_t;

#define CERB_DESC_WRITE(idx_, sub_, slot_, value_)                            \
  { CERB_STAGE_SDO_WRITE, value_, slot_, idx_, sub_, 0U }
#define CERB_DESC_READ(idx_, sub_, slot_, value_)                            \
  { CERB_STAGE_SDO_READ, value_, slot_, idx_, sub_, 0U }
#define CERB_DESC_SAVE(code_)                                                 \
  { CERB_STAGE_SAVE, CERB_VALUE_NONE, MOTOR_PROTOCOL_PARAM_SLOT_NONE,        \
    CERB_OBJ_SAVE_FLASH, CERB_SUB_DEFAULT, code_ }

static const cerb_stage_desc_t kStagesSetZero[1] = {
  CERB_DESC_WRITE(CERB_OBJ_SET_ZERO, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_NONE, CERB_VALUE_ZERO),
};

// 下限写/读回后再处理上限；设备 Flash 仅由独立 SAVE 事务保存。
static const cerb_stage_desc_t kStagesPositionLimits[4] = {
  CERB_DESC_WRITE(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_WRITE(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
};

static const cerb_stage_desc_t kStagesVelocityLimit[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
  CERB_DESC_READ(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
};

static const cerb_stage_desc_t kStagesTorqueLimit[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
  CERB_DESC_READ(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
};

// 清错帧无应答；是否清除由后续状态帧的 fault_active 判断。
static const cerb_stage_desc_t kStagesClearFault[1] = {
  { CERB_STAGE_MIT_CLEAR, CERB_VALUE_NONE, MOTOR_PROTOCOL_PARAM_SLOT_NONE, 0U,
    0U, 0U },
};

static const cerb_stage_desc_t kStagesRefresh[8] = {
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
  CERB_DESC_READ(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
  CERB_DESC_READ(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
  // 保护与方向直接回读原值。
  CERB_DESC_READ(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
      MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_PASSTHROUGH),
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_PASSTHROUGH),
  CERB_DESC_READ(CERB_OBJ_T_MAX, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_T_MAX, CERB_VALUE_PASSTHROUGH),
  CERB_DESC_READ(CERB_OBJ_NODE_ID, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID, CERB_VALUE_PASSTHROUGH),
};

// 保护易失：写后读回核验，不发保存码。
static const cerb_stage_desc_t kStagesSafetyProtect[2] = {
  CERB_DESC_WRITE(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
      MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_SAFETY_RAW),
  CERB_DESC_READ(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
      MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_SAFETY_RAW),
};

// 方向易失：写后读回核验，不发保存码。
// 读阶段必须用 DIRECTION_RAW 计算期望值，PASSTHROUGH 会使期望值恒为 CW。
static const cerb_stage_desc_t kStagesDirection[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
      MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
};

static const cerb_stage_desc_t kStagesTMax[2] = {
  CERB_DESC_WRITE(CERB_OBJ_T_MAX, 0, MOTOR_PROTOCOL_PARAM_SLOT_T_MAX, CERB_VALUE_T_MAX_RAW),
  CERB_DESC_READ(CERB_OBJ_T_MAX, 0, MOTOR_PROTOCOL_PARAM_SLOT_T_MAX, CERB_VALUE_T_MAX_RAW),
};
static const cerb_stage_desc_t kStagesNodeId[2] = {
  CERB_DESC_WRITE(CERB_OBJ_NODE_ID, 0, MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID, CERB_VALUE_NODE_ID_RAW),
  CERB_DESC_READ(CERB_OBJ_NODE_ID, 0, MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID, CERB_VALUE_NODE_ID_RAW),
};

// 保存按参数组（码 1）与限位组（码 4/2）执行；任一位置槽脏时保存上下限两项。
static const cerb_stage_desc_t kStagesSavePosBoth[2] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_LOWER),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_UPPER),
};

static const cerb_stage_desc_t kStagesSaveParams[1] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_PARAMS),
};

static const cerb_stage_desc_t kStagesSaveParamsAndPos[3] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_PARAMS),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_LOWER),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_UPPER),
};

static const cerb_stage_desc_t *cerb_save_table(
    const motor_protocol_config_request_t *request, uint8_t *count) {
  const bool params =
      (request->save_slots &
      ((1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX) |
      (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX))) != 0U;
  const bool pos =
      (request->save_slots &
      ((1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN) |
      (1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX))) != 0U;
  const cerb_stage_desc_t *table;
  if (params && pos) {
    table = kStagesSaveParamsAndPos;
    *count = 3U;
  } else if (params) {
    table = kStagesSaveParams;
    *count = 1U;
  } else if (pos) {
    table = kStagesSavePosBoth;
    *count = 2U;
  } else {
    // 仅易失槽变更：无设备保存码，由内核清脏并交上层落 W25。
    *count = 0U;
    return NULL;
  }
  return table;
}

// 事务 kind → 阶段表。READ_IDENTITY（分段上传）与 SET_MODE（空阶段）不在
// 表内，由三个接口特殊处理；返回 NULL 表示该 kind 不走设备事务。
static const cerb_stage_desc_t *cerb_stage_table(
    const motor_protocol_config_request_t *request, uint8_t *count) {
  switch (request->kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_ZERO:
    *count = (uint8_t)(sizeof(kStagesSetZero) / sizeof(kStagesSetZero[0]));
    return kStagesSetZero;
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
    *count = (uint8_t)(sizeof(kStagesPositionLimits) /
        sizeof(kStagesPositionLimits[0]));
    return kStagesPositionLimits;
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    *count = (uint8_t)(sizeof(kStagesVelocityLimit) /
        sizeof(kStagesVelocityLimit[0]));
    return kStagesVelocityLimit;
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
    *count = (uint8_t)(sizeof(kStagesTorqueLimit) /
        sizeof(kStagesTorqueLimit[0]));
    return kStagesTorqueLimit;
  case MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS:
    *count = (uint8_t)(sizeof(kStagesRefresh) / sizeof(kStagesRefresh[0]));
    return kStagesRefresh;
  case MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT:
    *count = (uint8_t)(sizeof(kStagesClearFault) /
        sizeof(kStagesClearFault[0]));
    return kStagesClearFault;
  case MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT:
    *count = (uint8_t)(sizeof(kStagesSafetyProtect) /
        sizeof(kStagesSafetyProtect[0]));
    return kStagesSafetyProtect;
  case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION:
    *count = (uint8_t)(sizeof(kStagesDirection) / sizeof(kStagesDirection[0]));
    return kStagesDirection;
  case MOTOR_PROTOCOL_CONFIG_SET_T_MAX:
    *count = 2; return kStagesTMax;
  case MOTOR_PROTOCOL_CONFIG_SET_NODE_ID:
    *count = 2; return kStagesNodeId;
  case MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH:
    return cerb_save_table(request, count);
  default:
    *count = 0U;
    return NULL;
  }
}

static const cerb_stage_desc_t *cerb_stage_at(
    const motor_protocol_config_request_t *request, uint8_t stage) {
  uint8_t count = 0U;
  const cerb_stage_desc_t *table = cerb_stage_table(request, &count);
  if (table == NULL || stage >= count) {
    return NULL;
  }
  return &table[stage];
}

// 写阶段的设备端原始值（cnt/RPM/mA/命令常量），按取值来源分发换算。
static uint32_t cerb_stage_write_value(
    uint8_t value_kind, const motor_protocol_model_profile_t *profile,
    const motor_protocol_config_request_t *request) {
  switch (value_kind) {
  case CERB_VALUE_ZERO:
    return CERB_SET_ZERO_CODE;
  case CERB_VALUE_POS_LOWER:
    return (uint32_t)cerb_rad_to_limit_cnt(
        request->position_limits.min_position_rad);
  case CERB_VALUE_POS_UPPER:
    return (uint32_t)cerb_rad_to_limit_cnt(
        request->position_limits.max_position_rad);
  case CERB_VALUE_SPEED:
    return cerb_joint_rad_s_to_motor_rpm(profile,
        request->max_velocity_rad_s);
  case CERB_VALUE_CURRENT:
    return cerb_joint_nm_to_motor_ma(profile, request->max_torque_nm);
  case CERB_VALUE_SAFETY_RAW:
    // 保护厂家原值直通；语义反转仅在 param_bridge。
    return request->safety_protect_raw;
  case CERB_VALUE_DIRECTION_RAW:
    return request->direction_raw;
  case CERB_VALUE_T_MAX_RAW:
    return request->t_max_raw;
  case CERB_VALUE_NODE_ID_RAW:
    return request->new_node_id;
  default:
    return 0U;
  }
}

// REFRESH 按槽位将设备原值换算为关节侧 SI。
static bool cerb_refresh_raw_to_si(uint8_t slot, uint32_t raw,
    const motor_protocol_model_profile_t *profile,
    float *out) {
  switch (slot) {
  case MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN:
  case MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX:
    // INTEGER32 cnt（负载端，与关节同轴）。
    *out = (float)motor_protocol_cerb_limit_cnt_to_rad((int32_t)raw);
    return true;
  case MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX:
    *out = (float)motor_protocol_cerb_motor_rpm_to_joint_rad_s(
        (double)profile->gear_ratio, raw);
    return true;
  case MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX:
    // 电流限值的读写共用限值系数，不使用反馈估计系数。
    *out = (float)motor_protocol_limit_ma_to_joint_nm(
        (double)motor_protocol_profile_limit_nm_per_amp(profile), raw);
    return true;
  case MOTOR_PROTOCOL_PARAM_SLOT_T_MAX:
    if (raw == 0 || raw > 16777215U) return false;
    *out = (float)raw; return true;
  case MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID:
    if (raw < 1 || raw > 127U) return false;
    *out = (float)raw; return true;
  case MOTOR_PROTOCOL_PARAM_SLOT_SAFETY:
  case MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION:
    // 原值透传：这两个是枚举/标志位，没有工程单位可换算。
    // value_si 仅供页面显示，裁决一律用 raw。
    *out = (float)raw;
    return true;
  default:
    return false;
  }
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
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    // 非 MIT 模式不受支持（ARD §25：CERB 默认且固定 MIT）。
    return request->mode == (uint8_t)MOTOR_PROTOCOL_MODE_MIT ? 1U : 0U;
  }
  uint8_t count = 0U;
  (void)cerb_stage_table(request, &count);
  (void)profile;
  return count;
}

static bool cerb_config_stage_info(uint8_t stage,
    const motor_protocol_model_profile_t *profile,
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
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    if (stage >= 1U) {
      return false;
    }
    out->waits_response = false; // 设备恒 MIT：空阶段，立即收尾
    out->timeout_ms = 0U;
    return true;
  }
  const cerb_stage_desc_t *desc = cerb_stage_at(request, stage);
  if (desc == NULL) {
    return false;
  }
  if (desc->action == CERB_STAGE_MIT_CLEAR) {
    out->waits_response = false; // 设备不对清错帧应答
    out->timeout_ms = 0U;
    out->param_raw_valid = false;
    out->param_slot = MOTOR_PROTOCOL_PARAM_SLOT_NONE;
    out->param_raw = 0U;
    return true;
  }
  out->timeout_ms = CERB_SDO_TIMEOUT_MS;
  if (desc->action == CERB_STAGE_SAVE) {
    out->waits_response = false; // 无应答：见 CERB_DESC_SAVE 的保存语义
    out->timeout_ms = 0U;
    out->param_raw_valid = false; // 保存码不是设备参数，不进比对/记录
    out->param_slot = MOTOR_PROTOCOL_PARAM_SLOT_NONE;
    out->param_raw = 0U;
    return true;
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    out->waits_response = true; // 6 个 upload，每阶段等应答
    out->param_raw_valid = true;
    out->param_slot = desc->param_slot;
    out->param_raw = 0U; // 原值由设备应答带回（读缓冲载荷）
    return true;
  }
  // 写事务的 SDO 阶段。
  out->waits_response = true; // 每个 SDO 阶段都等应答（写等 ack、读等回读）
  // set_zero 虽走 SDO 写，但写的是命令码不是设备参数，不进记录/比对。
  out->param_raw_valid = desc->action == CERB_STAGE_SDO_WRITE &&
      desc->param_slot != MOTOR_PROTOCOL_PARAM_SLOT_NONE;
  out->param_slot = out->param_raw_valid ? desc->param_slot
                                         : MOTOR_PROTOCOL_PARAM_SLOT_NONE;
  out->param_raw = cerb_stage_write_value(desc->value_kind, profile, request);
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
      payload[0] = CERB_SDO_CCS_UPLOAD_INITIATE;
      payload[1] = (uint8_t)(CERB_SDO_IDENTITY_INDEX & 0xFFU);
      payload[2] = (uint8_t)(CERB_SDO_IDENTITY_INDEX >> 8);
      payload[3] = CERB_SDO_IDENTITY_SUBINDEX;
    } else {
      // 分段上传请求：60/70 交替（bit4 = 切换位，从 0 起）。
      payload[0] =
          (uint8_t)(CERB_SDO_CCS_UPLOAD_SEGMENT |
          (uint8_t)(((stage - 1U) & 1U) << 4));
    }
    return cerb_sdo_request(node, bus_index, payload, out);
  }
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    if (stage >= 1U) {
      return false;
    }
    memset(out, 0, sizeof(*out)); // size 0 = 空阶段（无帧），仅推进状态机
    return true;
  }
  const cerb_stage_desc_t *desc = cerb_stage_at(request, stage);
  if (desc == NULL) {
    return false;
  }
  if (desc->action == CERB_STAGE_MIT_CLEAR) {
    // 清错不隐式使能；恢复运动仍须显式使能与新快照。
    float kd = profile != NULL ? profile->default_kd : 0.0f;
    if (!cerb_encode_neutral(profile, node, bus_index, false, kd,
        MOTOR_PROTOCOL_TX_CLASS_P2, out)) {
      return false;
    }
    out->data[0] |= CERB_MIT_CTRL_CLEAR_FAULT;
    return true;
  }
  uint8_t payload[8] = {0};
  if (desc->action == CERB_STAGE_SAVE) {
    // 保存无应答，发完即推进；设备写 Flash 可能阻塞控制循环。
    // 保存结果由下次上电参数比对检出。
    payload[0] = CERB_SDO_CCS_WRITE4;
    payload[1] = (uint8_t)(CERB_OBJ_SAVE_FLASH & 0xFFU);
    payload[2] = (uint8_t)(CERB_OBJ_SAVE_FLASH >> 8U);
    payload[3] = CERB_SUB_DEFAULT;
    payload[4] = desc->save_code;
    return cerb_sdo_request(node, bus_index, payload, out);
  }
  // 写/读 SDO：厂家默认「CANFD 加速 标准帧」（说明书 §3.1：1M 仲裁 /
  // 5M 数据），SDO 一律不走经典帧。
  payload[0] = desc->action == CERB_STAGE_SDO_WRITE
                   ? CERB_SDO_CCS_WRITE4
                   : CERB_SDO_CCS_UPLOAD_INITIATE;
  payload[1] = (uint8_t)(desc->index & 0xFFU);
  payload[2] = (uint8_t)(desc->index >> 8U);
  payload[3] = desc->sub;
  if (desc->action == CERB_STAGE_SDO_WRITE) {
    const uint32_t value =
        cerb_stage_write_value(desc->value_kind, profile, request);
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
    if (response->can_id != MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE + request->motor_id ||
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

  // 空模式阶段不会接收应答；直调时非 8B 返回 RETRY、8B 返回 FAIL。
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    return response->dlc == 8U ? MOTOR_PROTOCOL_STAGE_FAIL
                               : MOTOR_PROTOCOL_STAGE_RETRY;
  }

  const cerb_stage_desc_t *desc = cerb_stage_at(request, stage);
  if (desc == NULL) {
    return MOTOR_PROTOCOL_STAGE_FAIL;
  }
  const uint32_t sdo_response_id =
      MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE + request->motor_id;
  // 保存阶段无应答（发完即推进，内核不置 awaiting）：杂散帧不可能合法属于
  // 本阶段，显式 IGNORE，避免直调边界把它当 RETRY 触发 FLASH 重写。
  if (desc->action == CERB_STAGE_SAVE) {
    return MOTOR_PROTOCOL_STAGE_IGNORE;
  }

  // 设备参数回读（REFRESH_PARAMS）：6 个加速 upload，每阶段追加一个
  // motor_protocol_param_stage_t（设备端原值 + 关节侧换算值）进读缓冲。
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    if (response->can_id != sdo_response_id || response->dlc < 8U) {
      return MOTOR_PROTOCOL_STAGE_IGNORE;
    }
    if ((response->data[0] & 0xE0U) == CERB_SDO_CMD_ABORT) {
      return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    if (response->data[0] != CERB_SDO_SCS_UPLOAD_4BYTE) {
      return MOTOR_PROTOCOL_STAGE_RETRY; // 非上传应答（迟到的写应答等）
    }
    const uint16_t got_index =
        (uint16_t)(response->data[1] | ((uint16_t)response->data[2] << 8U));
    if (got_index != desc->index || response->data[3] != desc->sub) {
      return MOTOR_PROTOCOL_STAGE_RETRY; // 非本阶段对象
    }
    if (profile == NULL) {
      return MOTOR_PROTOCOL_STAGE_FAIL; // 换算系数缺失（未绑定不得发起）
    }
    float value = 0.0f;
    const uint32_t raw = cerb_read_le32(&response->data[4]);
    if (!cerb_refresh_raw_to_si(desc->param_slot, raw, profile, &value)) {
      return MOTOR_PROTOCOL_STAGE_FAIL;
    }
    const motor_protocol_param_stage_t payload = {.raw = raw,
                                                  .value_si = value};
    cerb_read_append(read_out, (const uint8_t *)&payload, sizeof(payload));
    return MOTOR_PROTOCOL_STAGE_ADVANCE;
  }

  uint32_t expected_id = sdo_response_id;
  if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_ZERO) {
    if (response->can_id != expected_id || response->dlc != 8U ||
        response->data[1] != (uint8_t)(CERB_OBJ_SET_ZERO & 0xFFU) ||
        response->data[2] != (uint8_t)(CERB_OBJ_SET_ZERO >> 8U) || response->data[3] != 0U) {
      return MOTOR_PROTOCOL_STAGE_IGNORE;
    }
    return response->data[0] == CERB_SDO_SCS_WRITE_ACK ? MOTOR_PROTOCOL_STAGE_ADVANCE
        : response->data[0] == CERB_SDO_CMD_ABORT ? MOTOR_PROTOCOL_STAGE_FAIL : MOTOR_PROTOCOL_STAGE_IGNORE;
  }
  if ((request->kind == MOTOR_PROTOCOL_CONFIG_SET_T_MAX || request->kind == MOTOR_PROTOCOL_CONFIG_SET_NODE_ID) &&
      response->can_id != expected_id && !(request->kind == MOTOR_PROTOCOL_CONFIG_SET_NODE_ID &&
      stage == 0 && response->can_id == MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE + request->new_node_id))
    return MOTOR_PROTOCOL_STAGE_IGNORE;
  if (response->dlc != 8U) {
    return MOTOR_PROTOCOL_STAGE_RETRY;
  }
  if (response->data[0] == CERB_SDO_CMD_ABORT) {
    return MOTOR_PROTOCOL_STAGE_FAIL; // 设备明确拒绝（含字典越界/不支持）
  }
  const uint16_t got_index = (uint16_t)((uint16_t)response->data[1] |
      ((uint16_t)response->data[2] << 8U));
  if (got_index != desc->index || response->data[3] != desc->sub) {
    return MOTOR_PROTOCOL_STAGE_RETRY; // 非本阶段应答（上一阶段迟到帧）
  }
  if (desc->action == CERB_STAGE_SDO_WRITE) {
    return response->data[0] == CERB_SDO_SCS_WRITE_ACK
               ? MOTOR_PROTOCOL_STAGE_ADVANCE
               : MOTOR_PROTOCOL_STAGE_RETRY;
  }
  if (response->data[0] != CERB_SDO_SCS_UPLOAD_4BYTE) {
    return MOTOR_PROTOCOL_STAGE_RETRY;
  }
  // 读回必须与写入值一致；不一致重发本阶段（≤2 次后内核收敛 UNCONFIRMED）。
  const uint32_t expected =
      cerb_stage_write_value(desc->value_kind, profile, request);
  return cerb_read_le32(&response->data[4]) == expected
             ? MOTOR_PROTOCOL_STAGE_ADVANCE
             : MOTOR_PROTOCOL_STAGE_RETRY;
}

// P1 逻辑帧 → (node, 9B MIT payload)。低节点按单轴 ID 反推；高节点
// （aggregate_only）携带显式 node，can_id 只是 0x210 占位、绝不直接上线。
static bool cerb_p1_member_node(const motor_protocol_tx_frame_t *frame,
    uint8_t *node) {
  if (frame == NULL || node == NULL) {
    return false;
  }
  if (frame->aggregate_only) {
    if (frame->can_id != MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID ||
        frame->is_extended || !frame->is_fd || !frame->is_brs ||
        frame->size != 12U || frame->node < 16U ||
        !motor_protocol_cerb_node_valid(frame->node) ||
        (frame->data[0] & 0x3fU) != 0x0cU) {
      return false;
    }
    *node = frame->node;
    return true;
  }
  return motor_protocol_cerb_single_mit_node(
      frame->can_id, frame->is_extended, frame->is_fd, frame->is_brs,
      frame->data, frame->size, node);
}

static size_t cerb_aggregate_p1_frames(
    const motor_protocol_tx_frame_t *frames, size_t available,
    motor_protocol_tx_frame_t *out) {
  if (frames == NULL || out == NULL || available == 0U) {
    return 0U;
  }

  const size_t limit = available < MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES
                           ? available
                           : MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES;
  motor_protocol_cerb_multi_axis_t axes[MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES] =
  {0};
  size_t count = 0U;
  for (size_t i = 0U; i < limit; ++i) {
    uint8_t node = 0U;
    const motor_protocol_tx_frame_t *frame = &frames[i];
    if (frame->bus_index != frames[0].bus_index ||
        frame->tx_class != MOTOR_PROTOCOL_TX_CLASS_P1 ||
        !cerb_p1_member_node(frame, &node)) {
      break;
    }
    bool duplicate = false;
    for (size_t axis = 0U; axis < count; ++axis) {
      if (axes[axis].node == node) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      break;
    }
    axes[count].node = node;
    memcpy(axes[count].data, frame->data, sizeof(axes[count].data));
    ++count;
  }
  // 单个低节点帧直接单轴发送更省总线；单个高节点没有合法单轴入口，
  // 必须包成一条 0x210 多轴帧（节点列表写明它自己）。
  if (count == 0U || (count < 2U && !frames[0].aggregate_only)) {
    return 0U;
  }

  motor_protocol_tx_frame_t aggregated = {0};
  aggregated.bus_index = frames[0].bus_index;
  aggregated.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  aggregated.tx_slot = UINT16_MAX; // 一个物理帧对应多个 latest-wins 槽。
  aggregated.can_id = MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID;
  aggregated.is_fd = true;
  aggregated.is_brs = true;
  aggregated.size = MOTOR_PROTOCOL_MAX_FRAME_BYTES;
  if (!motor_protocol_pack_cerb_multi_mit(axes, count, aggregated.data)) {
    return 0U;
  }
  *out = aggregated;
  return count;
}

const motor_protocol_adapter_t motor_protocol_cerb_adapter = {
  .protocol_name = "cerb",
  .node_id_min = 1U,
  .node_id_max = 127U,
  .offline_timeout_ms = 100U,   // 5×20ms SYNC 上报心跳（总线检测）
  .enable_settle_ms = 100U,     // 使能后 ≥100ms 才发运动目标（既有经验行为）
  .autonomous_feedback = true,   // SYNC 上报帧 = 20ms 状态心跳（承担总线检测）
  .config_verify_required = false,   // 写阶段均为「写 → 读回」成对，读回核验即确认
  .p1_batch_max = MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES,
  .match_rx_frame = cerb_match_rx_frame,
  .is_discovery_frame = cerb_is_discovery_frame,
  .decode_frame = cerb_decode_frame,
  .encode_mit_command = cerb_encode_mit_command,
  .encode_enable = cerb_encode_enable,
  .encode_soft_stop = cerb_encode_soft_stop,
  .encode_disable = cerb_encode_disable,
  .aggregate_p1_frames = cerb_aggregate_p1_frames,
  .encode_discovery_sync = cerb_encode_discovery_sync,
  .config_stage_count = cerb_config_stage_count,
  .config_stage_info = cerb_config_stage_info,
  .config_encode_stage = cerb_config_encode_stage,
  .config_on_response = cerb_config_on_response,
};
