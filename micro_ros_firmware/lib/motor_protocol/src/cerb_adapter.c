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
#define CERB_SDO_CCS_WRITE4 0x23U
#define CERB_SDO_CCS_UPLOAD_INITIATE 0x40U
#define CERB_SDO_SCS_WRITE_ACK 0x60U
#define CERB_SDO_SCS_UPLOAD_4BYTE 0x43U
#define CERB_SDO_CMD_ABORT 0x80U

// 管理对象（主索引/子索引）与标度：
//   0x607D:01/02 负载位置下限/上限（INTEGER32，cnt）—— 字典 -32767..32768 cnt
//                对应 -180..180 度，即 65536 cnt / 360 度。
//   子索引方向以字典表 + 真机为准，**不采信说明书**：说明书"读取正限位/读取负
//   限位"两行的子索引与它自己的设置行相反（设置正限位写 :02、负限位写 :01）——
//   照它读会把 -32568 当"正限位"。真机四台（2026-09-19）只读 SDO 实测
//   :01 = -32768、:02 = +32768。别照说明书"修正"下面两张表。
//   0x6080       电机最大转速（UNS32，电机端 RPM）
//   0x2538       电机峰值电流（UINT32，电机端 mA）
#define CERB_OBJ_POS_LIMIT 0x607DU
#define CERB_SUB_POS_LOWER 0x01U
#define CERB_SUB_POS_UPPER 0x02U
#define CERB_OBJ_MAX_SPEED 0x6080U
#define CERB_OBJ_MAX_CURRENT 0x2538U
// 0x2531「设置电机输出端零位」（字典表 RW，范围 0~2）：写 1 = 设置零位；
// 手册注「设置时电机需要在失能状态」。命令式对象：只等 SDO 写应答，不读回。
#define CERB_OBJ_SET_ZERO 0x2531U
#define CERB_SET_ZERO_CODE 1U
// 0x2539「Flash保存参数代码」（《CERB巨蟹智能-CANopen-V0.9字典表》字典表）：
//   1 = 断电保存**设置的电机参数**（最大电流/最大转速/PI 等）
//   2 = 设置电机**正限位**（0x607D:02）
//   4 = 设置电机**负限位**（0x607D:01）
// 手册另注明「0x607D 设置完成后需要保存 Flash」。
// **不按对应码保存 = 驱动器掉电回出厂值**：本机上电比对会因此判"不一致 →
// 未初始化"。所以写 0x2538/0x6080 之后发 1，写 :01 之后发 4、写 :02 之后发 2。
#define CERB_OBJ_SAVE_FLASH 0x2539U
#define CERB_SAVE_CODE_PARAMS 1U
#define CERB_SAVE_CODE_POS_UPPER 2U
#define CERB_SAVE_CODE_POS_LOWER 4U

// 0x2650 看门狗/限位保护开关（子索引 **0x00**）：
//   读   40 50 26 00 00 … → 反馈 43 50 26 00 0x …（0x 即当前状态）
//   开启 23 50 26 00 **00** …
//   关闭 23 50 26 00 **01** …
// 0 = 保护生效，1 = 关闭（官方 PR0002 §4.6 只列了"关闭"一条，开启/读法由
// owner 手工核实 + SLCAN 复现）。这是设备侧唯一的独立安全超时（帧间隔 >500ms
// 电机自锁），且关掉会连限位保护一起失效——CERB 型号的 capability_flags 是
// CAP_LIMIT_DEVICE_SIDE，固件没有软限位兜底。
// **易失**（2026-09-19 实测）：掉电后回到上电初值，0x2539 保存码对它无效
// （官方 §4.11：码 1 只保存 PI 参数和最大电流）。要"保护必须开着"须每次上电
// 显式发一次开启帧，不能指望设备记住。
// **读子索引实测**（2026-09-19）：:01 / :02 一律 ABORT 0x06090011（子索引不
// 存在），只有 :00 可读。此前按 0x01 读会让 REFRESH 恒挂在安全槽上 →
// UNCONFIRMED → UNVERIFIED，全部 CERB 都到不了 DEV_INIT_OK、永远无法使能。
#define CERB_OBJ_SAFETY_PROTECT 0x2650U
#define CERB_SUB_SAFETY_PROTECT 0x00U
// 0x2541「电机方向定义」：0=CW，1=CCW。**真机实测（2026-09-20）**：它只改驱动
// 的力矩/转动方向，**不改位置坐标系**——同一物理方向转动关节时，0x6064 的计数
// 方向不随 0x2541 变化（转两次同向，Δ 均为正：+2394 / +2573 cnt）。所以零点与
// cnt 限位**不镜像**，翻转后也**不需要**重新设零。真正的风险是驱动器自持的位置
// 环变成正反馈：使能后会朝远离目标的方向冲，直到撞上设备限位为止。
#define CERB_OBJ_MOTION_DIR 0x2541U

// 位置限位字典范围（0x607D:01/:02 均为 -32767~32768 cnt，默认 -32767/32768）：
// -π rad 折算是 -32768，比字典下限低 1 cnt，写下去可能被设备拒绝，故按字典钳。
#define CERB_POS_CNT_MIN (-32767)
#define CERB_POS_CNT_MAX (32768)
#define CERB_SUB_DEFAULT 0x00U

// 每阶段等待响应；超时自动重试 ≤2 次。
// 30ms 是"设备在跑 50Hz 控制时仍从容应答"的量级：厂家 SDO 是一问一答，
// 超时过短会重发、重发又让设备更慢，反而把整条读数链压成"一直超时"
// （2026-09-17 台架实测：20ms 下 node1 连续超时）。总线级串行见内核
// bus_stage_in_flight。
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

// 关节侧 N·m → 0x2538 的电机端 mA。
// 系数用**规格书峰值对**（峰值扭矩 ↔ 峰值电流）优先，见
// motor_protocol_profile_limit_nm_per_amp：写"峰值扭矩"就该正好写出"峰值电流"，
// 这样"外部修改不得超过硬件参数表"在扭矩与电流两个维度同时成立。
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
  // 温度先于故障分支取：data[8..9] 在故障帧里同样有效（规格书「执行器发给
  // 上位机」表），而过温本身就是故障的常见成因。
  out->temperature_c =
      motor_protocol_cerb_feedback_temperature_c(frame->data, frame->dlc);
  out->voltage_v = NAN;
  out->position_rad = NAN;
  out->velocity_rad_s = NAN;
  out->effort_nm = NAN;

  // 故障位（data[11] bit5）帧本身有效：标注故障、运动量缺失，不当作坏帧丢。
  // 错误代码只在故障帧取出：bit5 是「此刻在故障」的权威判据，代码是「是什么
  // 故障」的线索，且可能是锁存的历史值。无故障帧里也透传它，上游「非零 ⟺ 故障
  // 在册」的不变量就没了（motor_control_discovery 的清除分支依赖它）。
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
  out->can_id = MOTOR_PROTOCOL_CERB_SDO_REQUEST_ID_BASE + node;
  out->size = 8U;
  out->is_fd = true;
  out->is_extended = false;
  out->is_brs = true;
  memcpy(out->data, payload, 8U);
  return true;
}

// ---------------------------------------------------------------------------
// 配置事务阶段描述符表
//
// 除分段上传的 READ_IDENTITY 与 SET_MODE 空阶段外，所有事务的阶段知识
// （对象/子索引/读写/保存码/原值槽位）只存在于下表；stage_count/info/encode/
// on_response 四个接口都是这张表的解释器，新增一种事务 = 加一行表，不再
// 散改 8 个 switch。
// ---------------------------------------------------------------------------

enum {
  CERB_STAGE_SDO_WRITE, // 快速写 0x23，等写应答 0x60
  CERB_STAGE_SDO_READ,  // 快速上传 0x40，等 0x43 回读并与写值比对
  CERB_STAGE_SAVE,      // 写 0x2539 保存码，无应答，发完即推进
  CERB_STAGE_MIT_CLEAR, // MIT 控制字置 bit5 清错，无应答，发完即推进
};

// MIT 下行控制字 bit5 = 清错（ros2_ws/src/motor_drive/docs/cerb-series.md）。
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

// 下限写+读回、上限写+读回。**保存不在本表**：0x2539 是独立的保存事务
// （MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH），写到这里就落盘会让"只改没存"
// 变成不可观测的状态（见 param-authority-reversal-design.md §3.2）。
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

// REFRESH_PARAMS：6 个加速 upload（槽位与阶段顺序同源，内核按槽位落袋，
// 不再依赖阶段号硬编码）。末两个是 v3 设备侧安全事实，只读回读、写侧不提供。
// CLEAR_FAULT：单阶段清错帧，无应答。是否真清掉由设备后续状态帧的
// fault_active 判定（既有 FAULT → READY_DISABLED 路径），不引入新确认机制。
static const cerb_stage_desc_t kStagesClearFault[1] = {
  { CERB_STAGE_MIT_CLEAR, CERB_VALUE_NONE, MOTOR_PROTOCOL_PARAM_SLOT_NONE, 0U,
    0U, 0U },
};

static const cerb_stage_desc_t kStagesRefresh[6] = {
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
  CERB_DESC_READ(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
  CERB_DESC_READ(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
  // v3：设备侧安全事实。读回即比对基准，写侧不提供（恢复靠外部工具）。
  CERB_DESC_READ(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
                 MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_PASSTHROUGH),
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_PASSTHROUGH),
};

// 0x2650:00 看门狗/限位保护开关：写 + 读回核验，**不跟 0x2539 保存阶段**。
// 手册只给了关闭命令「23 50 26 00 01 00 00 00」，没给这个对象对应的保存码
// （对比 0x607D 那句明确写着"设置完成后需要保存 Flash"），所以掉电是否保持
// **实测无效**（2026-09-19）：发过码 1 后掉电，保护开关照样回退；官方 §4.11
// 明确码 1 只保存 PI 参数和最大电流，覆盖面不含本对象。读回核验照旧：写下去的值必须能
// 读回来，否则重发本阶段（≤2 次后内核收敛 UNCONFIRMED）。
static const cerb_stage_desc_t kStagesSafetyProtect[2] = {
  CERB_DESC_WRITE(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
                  MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_SAFETY_RAW),
  CERB_DESC_READ(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
                 MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_SAFETY_RAW),
};

// 0x2541:00 方向：写 + 读回核验，**不跟 0x2539 保存阶段**（易失，见上）。
// 翻转方向只改力矩方向、不改位置坐标系（见文件上方 0x2541 的实测注释）——
// 内核**不**据此冻结该台
// 直到重新设零（见 motor_control_config.c 的 ZERO_STALE）。
// 读阶段同样用 CERB_VALUE_DIRECTION_RAW（**不得复用 CERB_VALUE_PASSTHROUGH**）：
// 读回核验在 cerb_config_on_response 里就是从**本阶段的 value_kind** 反算期望
// 值（cerb_stage_write_value），PASSTHROUGH 落 default:return 0 → 期望恒为 CW，
// 写 CCW 会被永久判 RETRY/UNCONFIRMED。与 kStagesSafetyProtect 的读阶段
// 用 CERB_VALUE_SAFETY_RAW 同形。
static const cerb_stage_desc_t kStagesDirection[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                  MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
};

// 保存阶段表集合（SAVE_DEVICE_FLASH）。落盘粒度只有两组：参数组（码 1，
// 覆盖 PI 参数 + 最大电流 0x2538 + 最大转速 0x6080）与位置限位组（码 4/2）。
// 位置上下限在写事务里是同一笔（SET_POSITION_LIMITS 一次写两槽），所以脏掩码
// 实际上只出现"两组各自脏"或"两组都脏"；万一将来出现只脏一个位置槽的形状，
// 按"多存一个对象"兜底——多写一次 FLASH 只是慢，少写一次就是掉电丢参数。
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
    // 只脏易失槽（保护开关 / 方向）：没有保存码可发。count=0 让内核把这一步
    // 收敛成"清脏位 + 落 W25"，而不是每拍重开一个空事务。
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
      // 厂家原值直通：本层不做方向语义反转（不认"保护是否生效"，只认字节）。
      return request->safety_protect_raw;
    case CERB_VALUE_DIRECTION_RAW:
      return request->direction_raw;
    default:
      return 0U;
  }
}

// REFRESH 回读原值 → 关节侧 SI（按槽位选换算，不再按阶段号 switch）。
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
      // 0x2538 是电气电流上限：与写入侧同一套限值系数，保证"读到的 A"
      // 与"写回去的 A"是同一个数（换算系数换了也不会来回漂）。
      *out = (float)motor_protocol_limit_ma_to_joint_nm(
          (double)motor_protocol_profile_limit_nm_per_amp(profile), raw);
      return true;
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
    // 中性目标 + **不请求使能**（bit7=0）：故障态设备上「重新使能」的意图
    // 无意义，而且会把「清错」变成隐式的重新使能。清完仍要显式
    // SET_ENABLE(true) + 新快照才可能回运动（ARD §36）。
    motor_protocol_control_intent_t neutral = {0};
    neutral.kd = profile != NULL ? profile->default_kd : 0.0f;
    neutral.enable = false;
    if (!cerb_encode_mit_command(profile, node, bus_index, &neutral, out)) {
      return false;
    }
    out->data[0] |= CERB_MIT_CTRL_CLEAR_FAULT;
    return true;
  }
  uint8_t payload[8] = {0};
  if (desc->action == CERB_STAGE_SAVE) {
    // 保存阶段无应答：发完即推进。驱动器写 FLASH 期间可能阻塞自身控制
    // 循环，等 ack 只会把它和整条总线拖住；落盘成功与否由下次上电的
    // 设备参数比对检出（不一致 → 未初始化），比一次不可靠 ack 更硬。
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

  // SET_MODE 是空阶段（waits_response=false，内核不会投递应答）；保持旧版
  // 对杂散帧的边界口径：非 8B RETRY、8B FAIL（其事务无设备阶段计划）。
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

  // 写事务的 SDO 阶段（保存阶段无应答，不会进本函数）。
  // 长度检查（can_id 归属已由内核/match_rx_frame 预过滤；与旧实现一致，
  // 直调测试不设 can_id）。sdo_response_id 在 refresh 分支使用。
  (void)sdo_response_id;
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
