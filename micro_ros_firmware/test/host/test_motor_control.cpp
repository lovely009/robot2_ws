// motor_control ID 化运行时 host test（ARD-0007 §43.1 覆盖清单）。
//
// 覆盖：注册/发现、九态生命周期迁移、30/100ms 快照安全级联、断链立即停机、
// 受控缺席软停、重复/冲突 motor_id（冲突锁存穿越离线/重连）、Profile 绑定
// 与拒绝、OFFLINE/重连、逐电机 Enable 冻结与复立、厂家故障锁存且恢复需
// 显式重使能、配置事务（BUSY/UNSUPPORTED/超时/重试/结果码，撤权与离线
// 中止回 UNCONFIRMED）、增益与软件限位钳制、使能整定门、P0 重入队/满队
// 抢占、tx_slot 表项路由、过期挂起 P0 帧随使能作废、快照校验（NaN/重复
// ID/序号回绕）、容量 64、状态副本按 motor_id 升序输出、
// 事件环与 TX 收集/丢弃计数。全部黑盒：只走公共 API，不触碰 ctx 内部字段
// （生命周期断言经公共头的 entries 数组只读完成）。
//
// 时基说明：所有 tick 为 MCU 单调时基毫秒（uint32 回绕安全）。

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "motor_control_internal.h"
#include "motor_control_profile.h"

// ---------------------------------------------------------------------------
// 助手
// ---------------------------------------------------------------------------

static const motor_protocol_adapter_t *g_adapters[2];

static void bind_real_adapters() {
  g_adapters[0] = &motor_protocol_vaxor_adapter;
  g_adapters[1] = &motor_protocol_cerb_adapter;
}

static bool find_profile(const char *name, size_t *out_index) {
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    if (strcmp(motor_control_profile_at(i)->model_name, name) == 0) {
      *out_index = i;
      return true;
    }
  }
  return false;
}

static motor_protocol_raw_frame_t raw_frame(uint8_t bus, uint32_t id, size_t len,
                                            const uint8_t *data, uint32_t tick) {
  motor_protocol_raw_frame_t f = {};
  f.bus_index = bus;
  f.can_id = id;
  f.is_fd = true;
  f.dlc = (uint8_t)len;
  f.rx_tick_ms = tick;
  if (data != nullptr) {
    memcpy(f.data, data, len > MOTOR_PROTOCOL_MAX_FRAME_BYTES
                             ? MOTOR_PROTOCOL_MAX_FRAME_BYTES
                             : len);
  }
  return f;
}

// vaxor 12B 反馈帧（电机侧 SI，经 codec 打包成线协议）。
static motor_protocol_raw_frame_t vaxor_reply(uint16_t node, double motor_pos,
                                              double motor_vel, double current_a,
                                              uint32_t tick) {
  motor_protocol_joint_state_t wire = {motor_pos, motor_vel, current_a};
  uint8_t data[12];
  motor_protocol_pack_vaxor_reply(node, &wire, data);
  return raw_frame(0, motor_protocol_vaxor_reply_id(node), 12, data, tick);
}

// vaxor 20B 状态帧（fault 非零触发 fault_active）。
static motor_protocol_raw_frame_t vaxor_status(uint16_t node, uint32_t fault,
                                               uint32_t tick) {
  motor_protocol_joint_state_t wire = {0.0, 0.0, 0.0};
  uint8_t data[20];
  motor_protocol_pack_vaxor_status(node, &wire, 48.0, fault, 1, 0, 0, 1, data);
  return raw_frame(0, motor_protocol_vaxor_status_id(node), 20, data, tick);
}

// CERB 反馈帧：pos 1820 counts = 10°，vel 120 rpm（电机侧），Iq 2000 mA = 2 A。
static motor_protocol_raw_frame_t cerb_feedback(uint16_t node, uint32_t tick) {
  uint8_t data[12] = {};
  data[0] = 0x07; data[1] = 0x1c;
  data[2] = 0x00; data[3] = 0x78;
  data[4] = 0x07; data[5] = 0xd0;
  return raw_frame(0, 0x300U + node, 12, data, tick);
}

// CERB 故障帧：data[11] bit5 置位（厂家故障位，decode 判 fault_active）。
static motor_protocol_raw_frame_t cerb_fault(uint16_t node, uint32_t tick) {
  motor_protocol_raw_frame_t f = cerb_feedback(node, tick);
  f.data[11] |= 0x20U;
  return f;
}

static motor_control_command_snapshot_t make_snapshot(uint32_t seq, uint32_t tick) {
  motor_control_command_snapshot_t s = {};
  s.sequence = seq;
  s.received_tick_ms = tick;
  return s;
}

static void snapshot_add(motor_control_command_snapshot_t *s, uint16_t motor_id,
                         float pos, float vel, float effort) {
  s->targets[s->target_count].motor_id = motor_id;
  s->targets[s->target_count].position_rad = pos;
  s->targets[s->target_count].velocity_rad_s = vel;
  s->targets[s->target_count].effort_nm = effort;
  s->target_count++;
}

// 取走全部待发送帧（冲缓冲用）。
static size_t flush_tx(motor_control_context_t *ctx,
                       motor_protocol_tx_frame_t *out, size_t capacity) {
  return motor_control_collect_tx(ctx, out, capacity);
}

static size_t count_tx_class(motor_control_context_t *ctx,
                             motor_protocol_tx_class_t cls) {
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t total = 0;
  for (;;) {
    size_t n = motor_control_collect_tx(ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
    if (n == 0) {
      break;
    }
    for (size_t i = 0; i < n; ++i) {
      if (frames[i].tx_class == cls) {
        total++;
      }
    }
  }
  return total;
}

// 取指定电机的 DEV_INIT 事件（state + 不一致槽位位图）。drain 是消费性的，
// 同一条用例里要在别的 events_contain 之前调用。
static bool take_dev_init_event(motor_control_context_t *ctx, uint16_t motor_id,
                               uint16_t *out_state, uint32_t *out_detail) {
  motor_control_event_t events[64];
  bool found = false;
  size_t n;
  while ((n = motor_control_drain_events(ctx, events, 64)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_DEV_INIT &&
          events[i].motor_id == motor_id) {
        if (out_state != nullptr) *out_state = events[i].reason;
        if (out_detail != nullptr) *out_detail = events[i].request_id;
        found = true;
      }
    }
  }
  return found;
}

// 事件断言：循环 drain 至空，扫描全部事件（drain 是消费性的，不能只取一次）。
static bool events_contain(motor_control_context_t *ctx,
                           motor_control_event_type_t type, uint16_t motor_id) {
  motor_control_event_t events[64];
  bool found = false;
  size_t n;
  while ((n = motor_control_drain_events(ctx, events, 64)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == type &&
          (motor_id == 0 || events[i].motor_id == motor_id)) {
        found = true;
      }
    }
  }
  return found;
}

static bool has_result_event(motor_control_context_t *ctx, uint32_t request_id,
                             uint16_t reason) {
  motor_control_event_t events[64];
  bool found = false;
  size_t n;
  while ((n = motor_control_drain_events(ctx, events, 64)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT &&
          events[i].request_id == request_id && events[i].reason == reason) {
        found = true;
      }
    }
  }
  return found;
}


static const motor_control_param_view_t *view_of(motor_control_context_t *ctx,
                                                 uint16_t motor_id) {
  static motor_control_param_view_t cache[MOTOR_CONTROL_MAX_MOTORS];
  const size_t n = motor_control_copy_param_views(ctx, cache, MOTOR_CONTROL_MAX_MOTORS);
  for (size_t i = 0; i < n; ++i) {
    if (cache[i].motor_id == motor_id) {
      return &cache[i];
    }
  }
  return nullptr;
}

// 黑盒原则下仅存的只读窥视：生命周期断言经公共头 entries 数组完成。
static const motor_control_entry_t *entry_of(motor_control_context_t *ctx,
                                             uint16_t motor_id) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    if (ctx->entries[i].used && ctx->entries[i].motor_id == motor_id) {
      return &ctx->entries[i];
    }
  }
  assert(false && "entry not found");
  return nullptr;
}

// ---- 设备身份读取（CERB SDO 0x1008）测试侧驱动器 --------------------------
//
// 与固件 adapter 对向实现同一线协议：上传发起应答（0x41 + 索引/子索引 +
// 总字节数）与分段应答（byte0 = 000 t nnn c，数据在 byte1..7）。

static motor_protocol_raw_frame_t sdo_initiate_response(uint16_t node,
                                                        size_t total,
                                                        uint32_t tick) {
  uint8_t data[8] = {0x41, 0x08, 0x10, 0x00, 0, 0, 0, 0};
  data[4] = (uint8_t)(total & 0xFFU);
  data[5] = (uint8_t)((total >> 8) & 0xFFU);
  return raw_frame(0, 0x580U + node, 8, data, tick);
}

static motor_protocol_raw_frame_t sdo_segment_response(
    uint16_t node, uint8_t toggle, const uint8_t *payload, size_t payload_len,
    bool last, uint32_t tick) {
  uint8_t data[8] = {0};
  const size_t unused = last ? 7U - payload_len : 0U;
  data[0] = (uint8_t)(((toggle & 1U) << 4) | ((unused & 0x07U) << 1) |
                      (last ? 1U : 0U));
  memcpy(&data[1], payload, payload_len);
  return raw_frame(0, 0x580U + node, 8, data, tick);
}

// 加速传输应答（单帧回全量，n = 未用字节数）。CANopen 加速传输上限 4 字节，
// 故只用于短串用例。
static motor_protocol_raw_frame_t sdo_expedited_response(uint16_t node,
                                                         const char *text,
                                                         uint32_t tick) {
  const size_t len = strlen(text);
  assert(len <= 4U);
  uint8_t data[8] = {0};
  data[0] = (uint8_t)(0x43U | ((uint8_t)(4U - len) << 2));
  data[1] = 0x08;
  data[2] = 0x10;
  data[3] = 0x00;
  memcpy(&data[4], text, len);
  return raw_frame(0, 0x580U + node, 8, data, tick);
}

// 走完一次完整身份读取事务（事务每节拍至多推进一步，故逐段喂 service）。
static void feed_identity(motor_control_context_t *ctx, uint16_t node,
                          const char *identity, uint32_t tick) {
  const size_t len = strlen(identity);
  motor_control_service(ctx, tick); // 阶段 0：上传发起
  motor_protocol_raw_frame_t ack = sdo_initiate_response(node, len, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  size_t offset = 0U;
  uint8_t toggle = 0U;
  while (offset < len) {
    motor_control_service(ctx, tick + 2U + 2U * (uint32_t)toggle);
    const size_t chunk = (len - offset) > 7U ? 7U : (len - offset);
    const bool last = (offset + chunk) >= len;
    motor_protocol_raw_frame_t seg =
        sdo_segment_response(node, toggle, (const uint8_t *)identity + offset,
                             chunk, last, tick + 3U + 2U * (uint32_t)toggle);
    motor_control_on_raw_frame(ctx, &seg);
    offset += chunk;
    toggle = (uint8_t)((toggle + 1U) & 1U);
  }
}

// 完整使能链路（vaxor 路径）：反馈 → 自动绑定（协议内唯一型号）→ 连接 → 授权。
static void bring_vaxor_to_ready(motor_control_context_t *ctx, uint16_t motor_id,
                                 const char *profile_name, uint32_t tick) {
  motor_protocol_raw_frame_t f = vaxor_reply(motor_id, 0.0, 0.0, 0.0, tick);
  motor_control_on_raw_frame(ctx, &f);
  motor_control_service(ctx, tick + 1U); // 识别在服务节拍上发起
  size_t idx;
  assert(find_profile(profile_name, &idx));
  // vaxor 无身份读对象：内核按"协议内唯一型号"兜底绑定，无需上层干预。
  assert(entry_of(ctx, motor_id)->profile == motor_control_profile_at(idx));
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
  assert(entry_of(ctx, motor_id)->dev_init == MOTOR_CONTROL_DEV_INIT_NA);
}

// 完整使能链路（CERB 路径）：反馈 → 读设备自报身份 → 命中目录即绑定 →
// 连接 → 授权。
// ---------------------------------------------------------------------------
// 设备参数（CERB 0x607D:01/:02、0x6080、0x2538、0x2650:01、0x2541）：上电回读 → 与固件记录比对。
// ---------------------------------------------------------------------------

// 设备六个管理对象的顺序（与 adapter 的 REFRESH 阶段同序）。
static const uint16_t kDevIndex[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
    0x607DU, 0x607DU, 0x6080U, 0x2538U, 0x2650U, 0x2541U};
static const uint8_t kDevSub[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {1U, 2U, 0U,
                                                                0U, 1U, 0U};

static motor_protocol_raw_frame_t sdo_read_ack(uint16_t node, uint8_t stage,
                                               uint32_t value, uint32_t tick) {
  uint8_t payload[8] = {0x43U,
                        (uint8_t)(kDevIndex[stage] & 0xFFU),
                        (uint8_t)(kDevIndex[stage] >> 8U),
                        kDevSub[stage],
                        (uint8_t)(value & 0xFFU),
                        (uint8_t)((value >> 8U) & 0xFFU),
                        (uint8_t)((value >> 16U) & 0xFFU),
                        (uint8_t)((value >> 24U) & 0xFFU)};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x580U + node, 8, payload, tick);
  f.is_fd = false;
  return f;
}

static motor_protocol_raw_frame_t sdo_write_ack(uint16_t node, uint16_t index,
                                                uint8_t sub, uint32_t tick) {
  uint8_t payload[8] = {0x60U, (uint8_t)(index & 0xFFU),
                        (uint8_t)(index >> 8U), sub, 0U, 0U, 0U, 0U};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x580U + node, 8, payload, tick);
  f.is_fd = false;
  return f;
}

// 走完一次 REFRESH_PARAMS：逐阶段 service 出请求 → 喂对应应答。
// 返回结束时的时间戳。keepalive 为真时每阶段夹带周期心跳 + 新命令快照，
// 让"运行中回读"的用例在该事务期间保持 ACTIVE（否则 30ms 快照缺席级联会
// 先把电机软停，掩盖被测路径）。
// out_saw_p0：本事务期间是否观测到该节点的 P0 帧（软停等安全帧）。喂帧过程
// 会清空待发队列，事后 flush 看不到已被取走的帧，故在 drain 时顺手记录。
static uint32_t feed_device_params(motor_control_context_t *ctx, uint16_t node,
                                   const uint32_t raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT],
                                   uint32_t tick, bool keepalive,
                                   bool *out_saw_p0 = nullptr) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  uint32_t t = tick;
  for (uint8_t stage = 0U; stage < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++stage) {
    if (keepalive) {
      motor_control_command_snapshot_t snap = make_snapshot(1000U + stage, t);
      snapshot_add(&snap, node, 0.1f, 0.0f, 0.0f);
      (void)motor_control_apply_command_snapshot(ctx, &snap, t);
    }
    motor_control_service(ctx, t);
    const size_t drained =
        motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    for (size_t k = 0U; out_saw_p0 != nullptr && k < drained; ++k) {
      if (drain[k].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0 &&
          drain[k].can_id == 0x110U + node) {
        *out_saw_p0 = true;
      }
    }
    if (keepalive) {
      motor_protocol_raw_frame_t hb = cerb_feedback(node, t);
      motor_control_on_raw_frame(ctx, &hb);
    }
    motor_protocol_raw_frame_t ack =
        sdo_read_ack(node, stage, raw[stage], t + 1U);
    motor_control_on_raw_frame(ctx, &ack);
    t += 4U;
  }
  // 收尾那一拍也要看：末阶段应答在 on_raw_frame 里收尾，软停帧正是在此后的
  // 服务拍被取走——只看循环内的 drain 会漏掉它。
  motor_control_service(ctx, t);
  const size_t tail =
      motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (size_t k = 0U; out_saw_p0 != nullptr && k < tail; ++k) {
    if (drain[k].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0 &&
        drain[k].can_id == 0x110U + node) {
      *out_saw_p0 = true;
    }
  }
  return t;
}

// 只喂周期心跳、不回应答：让在飞的设备参数回读按阶段超时失败。
// 心跳是必须的——静默 100ms 会先把该台判离线，测的就不是超时路径了。
static uint32_t run_heartbeat_only(motor_control_context_t *ctx, uint16_t node,
                                   uint32_t t, uint32_t until_ms) {
  while ((int32_t)(until_ms - t) > 0) {
    motor_control_service(ctx, t);
    motor_protocol_raw_frame_t hb = cerb_feedback(node, t);
    motor_control_on_raw_frame(ctx, &hb);
    t += 20U;
  }
  return t;
}

// 走完一次 SET_TORQUE_LIMIT（写 + 读回核验）。
static uint32_t feed_torque_write(motor_control_context_t *ctx, uint16_t node,
                                  uint32_t ma, uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack =
      sdo_write_ack(node, 0x2538U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t rd = sdo_read_ack(node, 3U, ma, tick + 3U);
  motor_control_on_raw_frame(ctx, &rd);
  motor_control_service(ctx, tick + 4U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 4U;
}

// 造一条 Flash 记录（只含设备端原值 + 对应 SI 值由内核按 profile 比对）。
static motor_control_stored_params_t make_device_record(
    uint16_t motor_id, const uint32_t raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT],
    uint8_t raw_present) {
  motor_control_stored_params_t rec = {};
  rec.motor_id = motor_id;
  rec.has_position_limits = (raw_present & 0x03U) == 0x03U;
  rec.has_velocity_limit = (raw_present & 0x04U) != 0U;
  rec.has_torque_limit = (raw_present & 0x08U) != 0U;
  // v3：安全开关(0x2650:01)/方向(0x2541)两槽总是随记录落袋（出厂默认
  // 0=保护开 / 0=CW），上电比对据此也能发现设备侧保护被外部工具关掉。
  rec.device_raw_present =
      (uint8_t)(raw_present | (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY) |
                (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION));
  for (size_t i = 0; i < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++i) {
    rec.device_raw[i] = raw[i];
  }
  return rec;
}

// 限值对象的换算系数 = 规格书峰值对：45 N·m ÷ 10.8 A（R58）、11 ÷ 3.2（R48）。
// 故"峰值扭矩 ⇔ 峰值电流"精确成立：写峰值扭矩 45 N·m 落在 10800 mA。
#define kR58PeakMa 10800U
// 下限 -32768 cnt（int32 补码）＝ -180°，上限 +32768 cnt ＝ +180°；
// 3030 RPM 电机端 ＝ π rad/s 关节侧；10800 mA ＝ 45 N·m（规格书峰值）。
// v3 末两槽：保护开关 0（保护开）、方向 0（CW）——出厂默认。非 const：
// 安全槽用例需要临时改一台的期望值再还原。
static uint32_t kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
    0xFFFF8000U, 32768U, 3030U, kR58PeakMa, 0U, 0U};
// 关节侧 N·m/A（限值口径）。
#define kR58LimitNmPerAmp (45.0f / 10.8f)

// 设备参数初始化收尾（新流程的一部分）：把"本机无记录"告知内核，驱动自动
// 回读，再喂六个 SDO 应答。不做这一步该台停在 PENDING，按新口径不可使能。
static void complete_device_param_init(motor_control_context_t *ctx, uint16_t node,
                                       uint32_t tick) {
  motor_control_stored_params_t none = {};
  motor_control_apply_stored_params(ctx, &none, 0U, tick);
  (void)feed_device_params(ctx, node, kR58Raw, tick + 1U, false);
}

// 绑定完成 + 会话就绪，但**不做**设备参数回读：需要观察 PENDING /
// MISMATCH / UNVERIFIED 或自己驱动回读的用例用这个。
static void bring_cerb_to_bound(motor_control_context_t *ctx, uint16_t motor_id,
                                const char *profile_name, uint32_t tick) {
  motor_protocol_raw_frame_t f = cerb_feedback(motor_id, tick);
  motor_control_on_raw_frame(ctx, &f); // 建档即发起身份读取
  size_t idx;
  assert(find_profile(profile_name, &idx));
  const motor_protocol_model_profile_t *profile = motor_control_profile_at(idx);
  assert(profile->identity_string != NULL);
  feed_identity(ctx, motor_id, profile->identity_string, tick + 1U);
  assert(entry_of(ctx, motor_id)->profile == profile);
  // 事务期的 P2 请求帧清掉，避免污染后续 TX 断言。
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
}

// 上电收敛完整态：绑定 + 设备参数回读比对走完（本机无记录 → OK）。
// 上电流程要求"初始化完成才可运动"，常规用例（关心使能/运动）用这个。
static void bring_cerb_to_ready(motor_control_context_t *ctx, uint16_t motor_id,
                                const char *profile_name, uint32_t tick) {
  bring_cerb_to_bound(ctx, motor_id, profile_name, tick);
  complete_device_param_init(ctx, motor_id, tick + 1U);
  assert(entry_of(ctx, motor_id)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}

// ---------------------------------------------------------------------------
// mock adapter：事务超时/重试/BUSY 测试用（无真实厂家语义）。
// match: 0x600+node（node 1..8）；SET_MODE 事务 = 1 阶段等待响应 50ms，
// request_id 偶数 → 响应 ADVANCE，奇数 → RETRY（超过 2 次内核判失败）。
// ---------------------------------------------------------------------------

static bool mock_match(const motor_protocol_raw_frame_t *frame, uint16_t *out_node) {
  if (frame->can_id >= 0x600U && frame->can_id <= 0x607U && frame->dlc == 4U) {
    *out_node = (uint16_t)(frame->can_id - 0x600U);
    return true;
  }
  return false;
}

static bool mock_decode(const motor_protocol_model_profile_t *profile, uint16_t node,
                        const motor_protocol_raw_frame_t *frame,
                        motor_protocol_measured_state_t *out) {
  (void)profile;
  (void)frame;
  out->position_rad = (float)node;
  out->velocity_rad_s = 0.0f;
  out->effort_nm = 0.0f;
  out->current_a = 0.0f;
  out->has_motion_state = true;
  return true;
}

static bool mock_encode_mit(const motor_protocol_model_profile_t *profile,
                            uint16_t node, uint8_t bus_index,
                            const motor_protocol_control_intent_t *intent,
                            motor_protocol_tx_frame_t *out) {
  (void)profile;
  (void)intent;
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  out->can_id = 0x610U + node;
  out->size = 4;
  out->data[0] = (uint8_t)node;
  return true;
}

static bool mock_encode_simple(uint16_t node, uint8_t bus_index, uint32_t id_base,
                               motor_protocol_tx_class_t cls,
                               motor_protocol_tx_frame_t *out) {
  memset(out, 0, sizeof(*out));
  out->bus_index = bus_index;
  out->tx_class = cls;
  out->can_id = id_base + node;
  out->size = 1;
  return true;
}

static bool mock_encode_enable(const motor_protocol_model_profile_t *p, uint16_t node,
                               uint8_t bus, motor_protocol_tx_frame_t *out) {
  (void)p;
  return mock_encode_simple(node, bus, 0x640U, MOTOR_PROTOCOL_TX_CLASS_P1, out);
}

static bool mock_encode_soft_stop(const motor_protocol_model_profile_t *p,
                                  uint16_t node, uint8_t bus,
                                  motor_protocol_tx_frame_t *out) {
  (void)p;
  return mock_encode_simple(node, bus, 0x630U, MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static bool mock_encode_disable(const motor_protocol_model_profile_t *p, uint16_t node,
                                uint8_t bus, motor_protocol_tx_frame_t *out) {
  (void)p;
  return mock_encode_simple(node, bus, 0x620U, MOTOR_PROTOCOL_TX_CLASS_P0, out);
}

static uint8_t mock_stage_count(const motor_protocol_model_profile_t *profile,
                                const motor_protocol_config_request_t *request) {
  (void)profile;
  return request->kind == MOTOR_PROTOCOL_CONFIG_SET_MODE ? 1U : 0U;
}

static bool mock_stage_info(uint8_t stage,
                            const motor_protocol_model_profile_t *profile,
                            const motor_protocol_config_request_t *request,
                            motor_protocol_config_stage_info_t *out) {
  (void)stage;
  (void)profile;
  (void)request;
  out->waits_response = true;
  out->timeout_ms = 50U;
  return true;
}

static bool mock_stage_encode(uint8_t stage, const motor_protocol_model_profile_t *profile,
                              uint16_t node, uint8_t bus,
                              const motor_protocol_config_request_t *request,
                              motor_protocol_tx_frame_t *out) {
  (void)stage;
  (void)profile;
  (void)request;
  return mock_encode_simple(node, bus, 0x650U, MOTOR_PROTOCOL_TX_CLASS_P2, out);
}

static motor_protocol_stage_outcome_t mock_on_response(
    uint8_t stage, const motor_protocol_model_profile_t *profile,
    const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response,
    motor_protocol_read_buffer_t *read_out) {
  (void)stage;
  (void)profile;
  (void)request;
  (void)read_out;
  // 依响应内容裁决（非盲目）：data[0]==1 → ADVANCE，否则 RETRY。
  return response != nullptr && response->data[0] == 1U
             ? MOTOR_PROTOCOL_STAGE_ADVANCE
             : MOTOR_PROTOCOL_STAGE_RETRY;
}

static const motor_protocol_adapter_t g_mock_adapter = {
    "mock",
    1,
    8,
    1000U, // offline_timeout_ms
    0U,    // enable_settle_ms
    true,  // autonomous_feedback
    false, // config_verify_required
    mock_match,
    nullptr, // is_discovery_frame（mock 无事务应答帧，match 到的都是存在证据）
    mock_decode,
    mock_encode_mit,
    mock_encode_enable,
    mock_encode_soft_stop,
    mock_encode_disable,
    nullptr, // encode_discovery_sync
    mock_stage_count,
    mock_stage_info,
    mock_stage_encode,
    mock_on_response};

// mock 协议的测试 Profile（经指针版绑定 API 注入，不进生产型号目录）。
static const motor_protocol_model_profile_t g_mock_profile = {
    "mock_model",
    nullptr, // identity_string（mock 型号不在目录内，靠显式绑定注入）
    &g_mock_adapter,
    1.0f,
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    100.0f,
    10.0f,
    MOTOR_PROTOCOL_CAP_MODE_MIT};

// ---------------------------------------------------------------------------
// 用例
// ---------------------------------------------------------------------------

// 初始化：空注册表、未授权（安全默认——引导期授权电平由任务层转发）。
static void test_init_starts_unauthorized_and_empty() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  assert(ctx.entry_count == 0);
  assert(!ctx.authorized);
  assert(!ctx.have_snapshot);
  motor_protocol_tx_frame_t frames[4];
  assert(motor_control_collect_tx(&ctx, frames, 4) == 0);
}

// 生命周期：帧 → DISCOVERED → 识别 → READY_DISABLED（ARD §43.1 第 1 条）。
// vaxor 无身份读对象，识别即"协议内唯一型号"兜底，首帧一步到位；
// 需要等设备回串的协议（CERB）中间停在 DISCOVERED/IDENTIFYING，见
// test_identity_read_binds_and_unknown_stays_unbound 与
// test_identifying_state_observed。
static void test_frame_creates_discovered_then_bind_readies() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.entry_count == 1);
  // 帧到达只建档：识别是服务节拍上的动作（不占 RX 路径）。
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED);
  assert(entry_of(&ctx, 5)->profile == NULL);

  motor_control_service(&ctx, 101);
  size_t vaxor_2550;
  assert(find_profile("vaxor_2550", &vaxor_2550));
  assert(entry_of(&ctx, 5)->profile == motor_control_profile_at(vaxor_2550));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  // drain 是消费性的：一次取出全部事件再断言多个类型。
  motor_control_event_t seen[64];
  size_t seen_n = 0;
  size_t got;
  while ((got = motor_control_drain_events(&ctx, seen + seen_n, 64 - seen_n)) > 0) {
    seen_n += got;
  }
  bool have_discovered = false;
  bool have_bound = false;
  for (size_t i = 0; i < seen_n; ++i) {
    if (seen[i].type == MOTOR_CONTROL_EVENT_DISCOVERED && seen[i].motor_id == 5) {
      have_discovered = true;
    }
    if (seen[i].type == MOTOR_CONTROL_EVENT_PROFILE_BOUND && seen[i].motor_id == 5) {
      have_bound = true;
    }
  }
  assert(have_discovered);
  assert(have_bound);
}

// 容量 64：满表后拒绝并出 REGISTRY_FULL 事件（禁止静默截断）。
static void test_registry_capacity_64_rejects_overflow() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  // vaxor 节点 1..32（motor_id 1..32）+ CERB 节点 33..64（motor_id 33..64）。
  for (uint16_t node = 1; node <= 32; ++node) {
    motor_protocol_raw_frame_t f = vaxor_reply(node, 0.0, 0.0, 0.0, node * 10);
    motor_control_on_raw_frame(&ctx, &f);
  }
  for (uint16_t node = 33; node <= 64; ++node) {
    motor_protocol_raw_frame_t f = cerb_feedback(node, node * 10);
    motor_control_on_raw_frame(&ctx, &f);
  }
  assert(ctx.entry_count == MOTOR_CONTROL_MAX_MOTORS);

  // 第 65 台：CERB 节点 65（合法范围 1..127，motor_id 65 未占用）→ 满表拒绝。
  motor_protocol_raw_frame_t extra = cerb_feedback(65, 1000);
  motor_control_on_raw_frame(&ctx, &extra);
  assert(ctx.entry_count == MOTOR_CONTROL_MAX_MOTORS);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_REGISTRY_FULL, 0));
}

// 同一 motor_id 出现在不同总线 → CONFIG_CONFLICT、禁运动、不后到覆盖
// （ARD §9.3 / REV-08 / 禁止项 16）。
static void test_duplicate_motor_id_across_buses_conflicts() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f1 = cerb_feedback(7, 100); // bus 0
  motor_control_on_raw_frame(&ctx, &f1);
  motor_protocol_raw_frame_t f2 = cerb_feedback(7, 110);
  f2.bus_index = 1; // 同节点不同总线
  motor_control_on_raw_frame(&ctx, &f2);

  assert(ctx.entry_count == 1); // 后到帧不得另立新表项
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_CONFIG_CONFLICT, 7));
}

// 同总线同 motor_id 但协议身份不符 → CONFIG_CONFLICT（REV-08）。
static void test_identity_conflict_same_bus_different_protocol() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t cerb = cerb_feedback(7, 100); // motor_id 7（CERB）
  motor_control_on_raw_frame(&ctx, &cerb);
  // vaxor reply id 0x701+4*7 也解析出 node 7 —— 身份冲突。
  motor_protocol_raw_frame_t vaxor = vaxor_reply(7, 0.0, 0.0, 0.0, 110);
  motor_control_on_raw_frame(&ctx, &vaxor);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_CONFIG_CONFLICT, 7));
}

// 进入 ACTIVE 的完整前置链（ARD §9.2 / §43.1 第 2 条）：授权+Profile+新快照。
static void test_enter_active_requires_auth_profile_snapshot() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);
  size_t idx;
  assert(find_profile("vaxor_2550", &idx));
  assert(motor_control_bind_profile(&ctx, 5, idx, 110));

  // 未授权：快照不带来 ACTIVE，也不发运动帧。
  motor_control_set_agent_connected(&ctx, true, 120);
  motor_control_command_snapshot_t s = make_snapshot(1, 130);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 140));
  motor_control_service(&ctx, 150);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  // 授权 + 新快照 → ACTIVE：使能帧走 P0，先于 MIT 帧上总线，
  // MIT 帧走 P1；vaxor 无整定门，两帧同拍生成。一次排空统计两类计数。
  assert(motor_control_set_authorization(&ctx, true, 160));
  motor_control_command_snapshot_t s2 = make_snapshot(2, 165);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 170));
  motor_control_service(&ctx, 180);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  {
    motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
    size_t p0_count = 0, p1_count = 0, n;
    while ((n = motor_control_collect_tx(&ctx, frames,
                                         MOTOR_CONTROL_MAX_TX_PENDING)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0) {
          p0_count++;
        } else if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P1) {
          p1_count++;
        }
      }
    }
    assert(p0_count == 1); // 使能帧（P0）
    assert(p1_count == 1); // MIT 帧（P1）
  }
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_ACTIVE, 5));
}

// 下游因调度器 FIFO 满或硬件拒收而回告的 P0 帧必须重入队。
static void test_p0_requeue_after_downstream_failure() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  // 正常路径：内核生成的 P0 帧（tx_slot = 表项索引）原样回传 → 重入队。
  motor_protocol_tx_frame_t frame = {0};
  frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  frame.tx_slot = (uint16_t)(entry_of(&ctx, 5) - &ctx.entries[0]);
  frame.can_id = 0x123U;
  frame.size = 2U;
  assert(motor_control_requeue_p0(&ctx, &frame));
  motor_protocol_tx_frame_t out[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = motor_control_collect_tx(&ctx, out, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1);
  assert(out[0].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out[0].tx_slot == frame.tx_slot);

  // 队列满：填满非 P0 后回告 → 驱逐最旧非 P0 腾位（P0 抢占语义）。
  size_t profile_index;
  assert(find_profile("vaxor_2550", &profile_index));
  assert(motor_control_bind_profile(&ctx, 5, profile_index, 110));
  motor_control_set_agent_connected(&ctx, true, 120);
  motor_control_set_authorization(&ctx, true, 130);
  for (uint32_t tick = 150; tick < 150 + MOTOR_CONTROL_MAX_TX_PENDING; ++tick) {
    // 每拍刷新快照（防 30ms 陈旧级联）与反馈心跳（防 100ms 离线），MIT 逐拍
    // 入队直至填满待发队列。
    motor_control_command_snapshot_t fill = make_snapshot(tick, tick);
    snapshot_add(&fill, 5, 0.1f, 0.0f, 0.0f);
    assert(motor_control_apply_command_snapshot(&ctx, &fill, tick));
    motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, tick);
    motor_control_on_raw_frame(&ctx, &keepalive);
    motor_control_service(&ctx, tick);
  }
  const uint32_t dropped_before = ctx.tx_dropped_count;
  assert(motor_control_requeue_p0(&ctx, &frame));
  assert(ctx.tx_dropped_count == dropped_before + 1U); // 驱逐一个非 P0
  n = motor_control_collect_tx(&ctx, out, MOTOR_CONTROL_MAX_TX_PENDING);
  bool found_p0 = false;
  for (size_t i = 0; i < n; ++i) {
    if (out[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0) {
      found_p0 = true;
    }
  }
  assert(found_p0);
}

// 身份读取 → 自动绑定全链路：建档即读设备自报型号，命中目录绑定并收敛
// READY_DISABLED，事件可见；未命中目录（认不出的设备）保持不可控。
static void test_identity_read_binds_and_unknown_stays_unbound() {
  // (a) 命中：R48 自报串 → cerb_r48_101。
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(7, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(entry_of(&ctx, 7)->profile == NULL); // 首帧只建档，身份未回
  feed_identity(&ctx, 7, "R48LITE_V2", 101);
  size_t idx;
  assert(find_profile("cerb_r48_101", &idx));
  assert(entry_of(&ctx, 7)->profile == motor_control_profile_at(idx));
  assert(entry_of(&ctx, 7)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  {
    bool bound = false, ready = false;
    motor_control_event_t drained[64];
    size_t n;
    while ((n = motor_control_drain_events(&ctx, drained, 64)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (drained[i].motor_id != 7) {
          continue;
        }
        bound = bound || drained[i].type == MOTOR_CONTROL_EVENT_PROFILE_BOUND;
        ready = ready || drained[i].type == MOTOR_CONTROL_EVENT_READY;
      }
    }
    assert(bound);
    assert(ready);
  }

  // (b) 未命中：目录外自报串 → PROFILE_REJECTED + 保持不可控（不猜型号）。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  motor_protocol_raw_frame_t f2 = cerb_feedback(8, 100);
  motor_control_on_raw_frame(&ctx2, &f2);
  motor_control_event_t clear_buf[64];
  (void)motor_control_drain_events(&ctx2, clear_buf, 64);
  feed_identity(&ctx2, 8, "R99LITE_V9", 101);
  assert(entry_of(&ctx2, 8)->profile == NULL);
  motor_control_event_t ev = {};
  assert(motor_control_drain_events(&ctx2, &ev, 1) == 1);
  assert(ev.type == MOTOR_CONTROL_EVENT_PROFILE_REJECTED);
  assert(ev.motor_id == 8);
  assert(ev.bus_index == 0);
  assert(ev.reason == MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE);
}

// 身份串比较规则：尾随 NUL/空白剪掉后整串精确匹配，不做前缀/模糊匹配。
static void test_identity_string_match_is_exact() {
  const motor_protocol_adapter_t *cerb = &motor_protocol_cerb_adapter;
  const uint8_t exact[] = {'R', '4', '8', 'L', 'I', 'T', 'E', '_', 'V', '2'};
  assert(motor_control_profile_by_identity(cerb, exact, sizeof(exact)) != NULL);
  // 尾随填充（SDO 按对象声明长度回传常见形态）不影响判定。
  const uint8_t padded[] = {'R', '4', '8', 'L', 'I', 'T', 'E', '_',
                            'V', '2', 0, 0, 32};
  assert(motor_control_profile_by_identity(cerb, padded, sizeof(padded)) != NULL);
  // 前缀、大小写与多一字符都不许命中。
  const uint8_t prefix[] = {'R', '4', '8'};
  assert(motor_control_profile_by_identity(cerb, prefix, sizeof(prefix)) == NULL);
  const uint8_t lower[] = {'r', '4', '8', 'l', 'i', 't', 'e', '_',
                           'v', '2'};
  assert(motor_control_profile_by_identity(cerb, lower, sizeof(lower)) == NULL);
  const uint8_t longer[] = {'R', '4', '8', 'L', 'I', 'T', 'E', '_',
                            'V', '2', 'X'};
  assert(motor_control_profile_by_identity(cerb, longer, sizeof(longer)) == NULL);
  // 协议不符不得命中（同一身份串拿去 vaxor 上查）。
  assert(motor_control_profile_by_identity(&motor_protocol_vaxor_adapter, exact,
                                           sizeof(exact)) == NULL);
  assert(motor_control_profile_by_identity(cerb, NULL, 0U) == NULL);
  // 协议内唯一型号兜底：vaxor 只有一型 → 命中；cerb 有两型 → NULL（不猜）。
  assert(motor_control_profile_sole(&motor_protocol_vaxor_adapter) != NULL);
  assert(motor_control_profile_sole(&motor_protocol_cerb_adapter) == NULL);
}

// 身份读取失败的终态：不绑定、不逐帧重试（只发一次事务）；超时同理。
static void test_identity_read_failure_stays_unbound_without_retry() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(9, 100);
  motor_control_on_raw_frame(&ctx, &f);
  // 设备从容不应答（SDO 不实现/被滤掉）：每阶段 20ms 超时 ×≤2 次重试。
  // 期间维持 20ms 心跳，隔离"事务超时"与"心跳丢失判离线"两条路径。
  for (uint32_t t = 110U; t <= 260U; t += 10U) {
    if (t % 40U == 0U) {
      motor_protocol_raw_frame_t hb = cerb_feedback(9, t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
  }
  assert(entry_of(&ctx, 9)->online);
  assert(entry_of(&ctx, 9)->profile == NULL);
  // 读毕后不再发起新事务：排空待发队列后再看不再产生 SDO 请求帧。
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (uint32_t t = 270U; t <= 330U; t += 10U) {
    motor_protocol_raw_frame_t hb = cerb_feedback(9, t);
    motor_control_on_raw_frame(&ctx, &hb);
    motor_control_service(&ctx, t);
  }
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  for (size_t i = 0; i < n; ++i) {
    assert(frames[i].can_id < 0x600U || frames[i].can_id > 0x67FU); // 无 SDO 请求
  }
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 9));
}

// 重连重新识别：profile 不沿用旧读数，重新读身份后才回到可绑定态。
static void test_reconnect_reidentifies_device() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 4, "cerb_r58_101", 100);
  assert(entry_of(&ctx, 4)->profile != NULL);

  motor_control_service(&ctx, 400); // 心跳龄超 100ms → OFFLINE
  assert(entry_of(&ctx, 4)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);

  motor_protocol_raw_frame_t back = cerb_feedback(4, 500);
  motor_control_on_raw_frame(&ctx, &back);
  // 重连即作废旧身份：未读回前不可控（换成别的型号的话不得沿用旧换算比）。
  assert(entry_of(&ctx, 4)->profile == NULL);
  feed_identity(&ctx, 4, "R48LITE_V2", 501);
  size_t idx;
  assert(find_profile("cerb_r48_101", &idx));
  assert(entry_of(&ctx, 4)->profile == motor_control_profile_at(idx));
  assert(entry_of(&ctx, 4)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}

// 事务中止（撤权路径）不算"识别失败"：身份读取重连后仍会重新发起。
static void test_identity_read_abort_is_not_a_verdict() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_control_set_agent_connected(&ctx, true, 50);
  assert(motor_control_set_authorization(&ctx, true, 60));

  motor_protocol_raw_frame_t f = cerb_feedback(6, 100);
  motor_control_on_raw_frame(&ctx, &f); // 身份事务已发起，未应答
  // 断链撤权 → 中止事务（不判成败、不回 PROFILE_REJECTED）。
  motor_control_set_agent_connected(&ctx, false, 110);
  motor_control_event_t ev = {};
  size_t n = 0;
  bool rejected = false;
  while (motor_control_drain_events(&ctx, &ev, 1) == 1) {
    rejected = rejected || ev.type == MOTOR_CONTROL_EVENT_PROFILE_REJECTED;
    if (++n > 64U) {
      break;
    }
  }
  assert(!rejected);

  // 重连后重新识别仍然成功（若中止被误判为"识别失败"，这里就永远绑不上）。
  motor_protocol_raw_frame_t back = cerb_feedback(6, 200);
  motor_control_on_raw_frame(&ctx, &back);
  feed_identity(&ctx, 6, "R58LITE_V2", 201);
  size_t idx;
  assert(find_profile("cerb_r58_101", &idx));
  assert(entry_of(&ctx, 6)->profile == motor_control_profile_at(idx));
}

// SDO 应答可归属但不得凭空建档（只有 12B 状态上报是"设备存在"证据）；
// 已知设备的迟到/杂散应答只服务裁决，不得计入解码错误。
static void test_sdo_response_alone_creates_no_entry() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t ack = sdo_initiate_response(3, 10, 100);
  motor_control_on_raw_frame(&ctx, &ack);
  assert(ctx.entry_count == 0U);
  assert(ctx.unknown_frame_count == 1U);

  // 已知设备（已绑定、无未决事务）收到杂散 SDO 应答：静默丢弃。
  motor_protocol_raw_frame_t f = cerb_feedback(3, 200);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_service(&ctx, 201);
  feed_identity(&ctx, 3, "R48LITE_V2", 202);
  assert(entry_of(&ctx, 3)->profile != NULL);
  const uint32_t errors_before = ctx.decode_error_count;
  motor_protocol_raw_frame_t stray = sdo_initiate_response(3, 10, 400);
  motor_control_on_raw_frame(&ctx, &stray);
  assert(ctx.decode_error_count == errors_before);
  assert(entry_of(&ctx, 3)->profile != NULL);
}

// 事务等待期内到达的同节点周期上报必须被忽略（不是应答）：不推进阶段、
// 不计重试，且照常走解码供运动状态使用。
static void test_identity_read_ignores_heartbeat_frames() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(5, 100);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_service(&ctx, 101); // 阶段 0：上传发起，进入等待

  motor_protocol_raw_frame_t hb = cerb_feedback(5, 102);
  motor_control_on_raw_frame(&ctx, &hb); // 同节点周期上报
  assert(ctx.decode_error_count == 0U);  // 仍是正常反馈帧，不是协议错误
  assert(entry_of(&ctx, 5)->has_measured);
  // 心跳不得把事务推走：真实应答到达后仍从阶段 0 继续。
  motor_protocol_raw_frame_t ack = sdo_initiate_response(5, 10, 103);
  motor_control_on_raw_frame(&ctx, &ack);
  motor_control_service(&ctx, 104);
  motor_protocol_raw_frame_t seg =
      sdo_segment_response(5, 0, (const uint8_t *)"R58LITE_V2", 7, false, 105);
  motor_control_on_raw_frame(&ctx, &seg);
  motor_control_service(&ctx, 106);
  // 续段 = 身份串第 7..9 字节（"R58LITE" + "_V2"）。
  motor_protocol_raw_frame_t last =
      sdo_segment_response(5, 1, (const uint8_t *)"_V2", 3, true, 107);
  motor_control_on_raw_frame(&ctx, &last);
  size_t idx;
  assert(find_profile("cerb_r58_101", &idx));
  assert(entry_of(&ctx, 5)->profile == motor_control_profile_at(idx));
}

// 加速传输（≤4B 单帧回全量）走通且目录无此串 → 不绑定；
// 应答回带的索引不符 → 读到的内容不可信，直接判读失败。
static void test_identity_read_expedited_and_wrong_index() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(2, 100);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_service(&ctx, 101);
  motor_protocol_raw_frame_t ack = sdo_expedited_response(2, "R48V", 102);
  motor_control_on_raw_frame(&ctx, &ack);
  assert(entry_of(&ctx, 2)->profile == NULL);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 2));

  // 索引不符（设备回了别的对象 0x1000，与它 0x1008 的诉求无关）：不猜。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  motor_protocol_raw_frame_t f2 = cerb_feedback(3, 100);
  motor_control_on_raw_frame(&ctx2, &f2);
  motor_control_service(&ctx2, 101);
  uint8_t wrong[8] = {0x41, 0x00, 0x10, 0x00, 10, 0, 0, 0}; // 索引 0x1000
  motor_protocol_raw_frame_t wf = raw_frame(0, 0x580U + 3, 8, wrong, 102);
  motor_control_on_raw_frame(&ctx2, &wf);
  assert(entry_of(&ctx2, 3)->profile == NULL);
  assert(events_contain(&ctx2, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 3));
}

// 快照含未注册 motor_id → 整体拒绝（ARD §15.5：不存在/冲突设备必须拒绝）。
static void test_snapshot_with_unknown_motor_rejected() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 99, 0.1f, 0.0f, 0.0f); // 未注册
  assert(!motor_control_apply_command_snapshot(&ctx, &s, 130));
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_SNAPSHOT_REJECTED, 0));
}

// 受控集合缺席软停（ARD §13.1 / §43.1 第 7 条）。
static void test_snapshot_absence_soft_stops_motor() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  bring_vaxor_to_ready(&ctx, 9, "vaxor_2550", 110);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s, 9, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  // 下一快照缺席电机 5 → 软停（P0 阻尼帧 + SAFE_STOP）；电机 9 仍在控。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 150);
  snapshot_add(&s2, 9, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 160));
  motor_control_service(&ctx, 170);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(entry_of(&ctx, 9)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) == 1);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_SOFT_STOP, 5));

  // 重新出现 → 回 ACTIVE（授权未撤销，快照新鲜）。
  // 持续自主上报（20ms 周期），避免 100ms 无反馈被误判 OFFLINE。
  motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, 175);
  motor_control_on_raw_frame(&ctx, &keepalive);
  motor_control_command_snapshot_t s3 = make_snapshot(3, 180);
  snapshot_add(&s3, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s3, 9, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 190));
  motor_control_service(&ctx, 200);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 30ms 无新快照 → 全体受控软停；恢复快照后可回 ACTIVE（ARD §13.2）。
static void test_30ms_stale_snapshot_soft_stops() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125));
  motor_control_service(&ctx, 130);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // freshness 以 mailbox 接收 tick 为基准：age=30 仍新鲜（≤30），
  // age=31（>30）级联软停——精确踩规范边沿。
  motor_control_service(&ctx, 150);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_control_service(&ctx, 151);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  // P0 两帧：进入 ACTIVE 时的使能帧 + 本次软停帧。
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) == 2);

  // 新鲜快照回来 → 恢复（级联去重标志复位）。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 160);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 165));
  motor_control_service(&ctx, 170);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 100ms 无新快照 → 协议级停机/失能 + 清授权；恢复需新授权 + 新快照
// （ARD §13.2/§13.3 / §43.1 第 4 条）。
static void test_100ms_stale_disables_and_clears_authorization() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125));
  motor_control_service(&ctx, 130);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // 持续自主上报（设备仍在发反馈），仅快照停更：age=100 仍只是软停态，
  // age=101（>100）协议级停机/失能 + 清授权——精确踩规范边沿。
  motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, 219);
  motor_control_on_raw_frame(&ctx, &keepalive);
  motor_control_service(&ctx, 220); // age=100：30ms 级联已发软停，尚未升级
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(ctx.authorized);
  motor_control_service(&ctx, 221); // age=101：升级为协议级停机/失能
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(!ctx.authorized);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) >= 1); // 失能帧
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_DISABLE, 5));

  // 恢复：重新授权，但旧快照不作数 —— 必须等新快照。
  assert(motor_control_set_authorization(&ctx, true, 240));
  motor_control_service(&ctx, 250);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  motor_control_command_snapshot_t s2 = make_snapshot(2, 260);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 265));
  motor_control_service(&ctx, 270);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 断链立即停机（不等待 100ms，ARD §13.2/§33/§43.1 第 3 条）。
static void test_agent_disconnect_immediate_disable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125));
  motor_control_service(&ctx, 130);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  motor_control_set_agent_connected(&ctx, false, 135);
  assert(!ctx.authorized);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) >= 1);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST, 0)); // 全局事件
}

// 恢复不复用旧目标/旧授权：断链后即使重连+授权，没有新快照就不得出运动帧。
static void test_recovery_requires_fresh_snapshot_not_old_targets() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125));
  motor_control_service(&ctx, 130);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  motor_control_set_agent_connected(&ctx, false, 135);
  motor_control_set_agent_connected(&ctx, true, 200);
  assert(motor_control_set_authorization(&ctx, true, 205));
  // 500ms 内无任何新快照：不出运动帧，不回 ACTIVE。
  for (uint32_t t = 210; t <= 700; t += 10) {
    motor_control_service(&ctx, t);
    assert(entry_of(&ctx, 5)->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  }
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  // 设备重新上线（自主上报恢复）→ 重新发现流 → 等新快照。
  motor_protocol_raw_frame_t f2 = vaxor_reply(5, 0.0, 0.0, 0.0, 705);
  motor_control_on_raw_frame(&ctx, &f2);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING); // 重连帧非首帧：发现→识别即时推进
  motor_control_service(&ctx, 706); // 重新识别 → 绑定 → READY_DISABLED
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  // 循环期 100ms 级联已再次清权：按恢复语义重新授权（ARD §13.3）。
  assert(motor_control_set_authorization(&ctx, true, 708));
  // 循环期离线已冻结 policy_enable（ARD §36）：显式重发 Enable。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 9U;
  enable.motor_id = 5;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 709));
  motor_control_service(&ctx, 709);

  // 新快照到达后恢复。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 710);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 715));
  motor_control_service(&ctx, 720);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 快照校验：非法快照整体拒绝（NaN/重复 ID）。序号门已于 2026-09-15 移除
//（新鲜度门 30/100ms 级联承担重放/乱序防护），任意序号——包括回退/重复/
// uint32 回绕——均接受。
static void test_snapshot_validation() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));

  // 回退/重复序号：序号门已移除，一律接受（仍受新鲜度门约束）。
  motor_control_command_snapshot_t replay = make_snapshot(1, 140);
  snapshot_add(&replay, 5, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &replay, 150));

  motor_control_command_snapshot_t backward = make_snapshot(0, 150);
  snapshot_add(&backward, 5, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &backward, 155));

  // NaN 拒绝。
  motor_control_command_snapshot_t nan_s = make_snapshot(2, 160);
  snapshot_add(&nan_s, 5, NAN, 0.0f, 0.0f);
  assert(!motor_control_apply_command_snapshot(&ctx, &nan_s, 170));

  // 重复 motor_id 拒绝。
  motor_control_command_snapshot_t dup = make_snapshot(3, 180);
  snapshot_add(&dup, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&dup, 5, 0.1f, 0.0f, 0.0f);
  assert(!motor_control_apply_command_snapshot(&ctx, &dup, 190));

  // uint32 回绕与跳变：任意序号接受（独立上下文）。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  bring_vaxor_to_ready(&ctx2, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t high = make_snapshot(0xFFFFFFFFU, 200);
  snapshot_add(&high, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx2, &high, 210));
  motor_control_command_snapshot_t wrapped = make_snapshot(0, 220);
  snapshot_add(&wrapped, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx2, &wrapped, 230));
  motor_control_command_snapshot_t after_wrap = make_snapshot(1, 240);
  snapshot_add(&after_wrap, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx2, &after_wrap, 250));
  motor_control_command_snapshot_t back_low = make_snapshot(0, 260);
  snapshot_add(&back_low, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx2, &back_low, 270));
}

// OFFLINE（Vaxor：自主上报缺失即判龄）与重连（ARD §36/§43.1 第 12 条）。
static void test_vaxor_offline_and_reconnect() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125));
  motor_control_service(&ctx, 130);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // 100ms（vaxor 阈值占位，TBD-SAFETY-03）无任何帧 → OFFLINE。
  motor_control_service(&ctx, 235);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);
  assert(!entry_of(&ctx, 5)->online);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_OFFLINE, 5));

  // 重新上线 → 重新走发现/识别流（ARD §36），不直达 READY；
  // 旧授权/目标不得复用（§10.4）→ 需重新授权+新快照。
  motor_protocol_raw_frame_t f2 = vaxor_reply(5, 0.0, 0.0, 0.0, 300);
  motor_control_on_raw_frame(&ctx, &f2);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING); // 重连帧非首帧：发现→识别即时推进
  // 旧 profile 已在重连时作废（身份是设备事实，可能换机）；本拍重新识别
  // （vaxor 走"协议内唯一型号"兜底）→ 绑定 → READY_DISABLED。
  assert(entry_of(&ctx, 5)->profile == NULL);
  motor_control_service(&ctx, 302);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_RECONNECTED, 5));
  assert(motor_control_set_authorization(&ctx, true, 305)); // 重新授权
  // OFFLINE 冻结 policy_enable（ARD §36）：必须显式重发 Enable。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 6U;
  enable.motor_id = 5;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 307));
  motor_control_service(&ctx, 308);

  motor_control_command_snapshot_t s2 = make_snapshot(2, 310);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 315));
  motor_control_service(&ctx, 320);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// CERB OFFLINE（2026-09-05 协议核实）：0x080 SYNC 维持设备 20ms 状态上报，
// 上报帧即心跳（承担总线检测）——心跳在线时无命令也活着；心跳丢失
// ≥100ms（5×20ms，adapter 阈值）即 OFFLINE，与是否在发命令无关。
static void test_cerb_offline_by_heartbeat_loss() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  // 心跳在线（20ms 间隔的上报帧）→ 无命令也保持 READY，不误判离线。
  for (uint32_t t = 140; t <= 220; t += 20) {
    motor_protocol_raw_frame_t fb = cerb_feedback(7, t);
    motor_control_on_raw_frame(&ctx, &fb);
    motor_control_service(&ctx, t);
  }
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 心跳丢失 ≥100ms → OFFLINE（总线检测生效）。
  motor_control_service(&ctx, 340); // 末帧 220 → 120ms
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);
  assert(!entry_of(&ctx, 7)->online);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_OFFLINE, 7));
}

// CERB 使能整定门：使能帧后 100ms 内不发运动目标（厂商指引，adapter 常量）。
// 整定期间持续喂新快照，避免 30ms 级联抢先触发。
static void test_cerb_enable_settle_gate() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  // 整定期：只有使能帧（中性目标），状态仍 READY_DISABLED。
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1);
  assert(frames[0].can_id == 0x110U + 7U); // CERB 使能帧
  assert(frames[0].data[0] == 0xccU);      // 使能位

  // 整定期内的后续快照也不得产生 MIT 帧。
  // （各步喂 20ms 间隔心跳帧：CERB SYNC 上报帧承担总线检测，静默 100ms 即离线。）
  motor_control_command_snapshot_t s2 = make_snapshot(2, 150);
  snapshot_add(&s2, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 160));
  motor_protocol_raw_frame_t hb1 = cerb_feedback(7, 158);
  motor_control_on_raw_frame(&ctx, &hb1);
  motor_control_service(&ctx, 170);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  motor_control_command_snapshot_t s3 = make_snapshot(3, 180);
  snapshot_add(&s3, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 185));
  motor_protocol_raw_frame_t hb2 = cerb_feedback(7, 188);
  motor_control_on_raw_frame(&ctx, &hb2);
  motor_control_service(&ctx, 190); // 整定 60ms，仍在窗内
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  motor_control_command_snapshot_t s4 = make_snapshot(4, 220);
  snapshot_add(&s4, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s4, 225));
  motor_protocol_raw_frame_t hb3 = cerb_feedback(7, 228);
  motor_control_on_raw_frame(&ctx, &hb3);
  motor_control_service(&ctx, 235); // 整定 105ms ≥ 100ms → 到期
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  // 整定到期后的下一个边界 → MIT + ACTIVE。
  motor_control_command_snapshot_t s5 = make_snapshot(5, 245);
  snapshot_add(&s5, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s5, 250));
  motor_protocol_raw_frame_t hb4 = cerb_feedback(7, 252);
  motor_control_on_raw_frame(&ctx, &hb4);
  motor_control_service(&ctx, 260);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) >= 1);
}

// vaxor 无整定门：使能帧与 MIT 帧同一边界发出。
static void test_vaxor_no_settle() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n >= 2); // control(run=1) + MIT command
}

// 增益钳制：Policy 增益按 Profile 线协议上限钳制（上限来自 Profile，非核心常量）。
static void test_gains_clamped_to_profile() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  motor_protocol_config_request_t gains = {};
  gains.request_id = 1U;
  gains.motor_id = 5;
  gains.kind = MOTOR_PROTOCOL_CONFIG_SET_GAINS;
  gains.kp = 999.0f; // >> kp_max 0.65535
  gains.kd = 999.0f; // >> kd_max 655.35
  assert(motor_control_submit_config(&ctx, &gains, 150));
  motor_control_service(&ctx, 160);
  assert(has_result_event(&ctx, 1U, MOTOR_PROTOCOL_CONFIG_OK));
  // 冲掉 service 期间可能的软停帧（同为 12B 命令帧，避免混入下方断言）。
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  motor_control_command_snapshot_t s2 = make_snapshot(2, 170);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 180));
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool found_mit = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].size == 12U &&
        frames[i].can_id == motor_protocol_vaxor_command_id(5)) {
      motor_protocol_joint_command_t wire = {};
      assert(motor_protocol_unpack_vaxor_command(frames[i].data, 12, &wire));
      assert(fabs(wire.kp - 0.65535) < 1e-6); // 必须钳到上限（非静默丢增益）
      assert(wire.kd <= 655.35 + 0.02);
      found_mit = true;
    }
  }
  assert(found_mit);
}

// Vaxor 软件限位：打包前钳制 + limit_flags（ARD §26.1）。
static void test_vaxor_software_limits_clamp_before_pack() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  motor_protocol_config_request_t limits = {};
  limits.request_id = 1U;
  limits.motor_id = 5;
  limits.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  limits.position_limits.min_position_rad = -0.5f;
  limits.position_limits.max_position_rad = 0.5f;
  assert(motor_control_submit_config(&ctx, &limits, 150));
  motor_control_service(&ctx, 160);
  assert(has_result_event(&ctx, 1U, MOTOR_PROTOCOL_CONFIG_OK));
  // 冲掉 service 期间的软停帧（同为 12B 命令帧，避免混入下方断言）。
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  // 目标 1.0 rad 超上限 → 钳到 0.5（关节侧）→ 线协议位置 = 0.5×50 = 25 rad。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 170);
  snapshot_add(&s2, 5, 1.0f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 180));
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool found = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].size == 12U &&
        frames[i].can_id == motor_protocol_vaxor_command_id(5)) {
      motor_protocol_joint_command_t wire = {};
      assert(motor_protocol_unpack_vaxor_command(frames[i].data, 12, &wire));
      assert(fabs(wire.position - 25.0) < 0.01);
      found = true;
    }
  }
  assert(found);

  motor_control_status_t statuses[8];
  size_t m = motor_control_copy_statuses(&ctx, statuses, 8, 190);
  bool flagged = false;
  for (size_t i = 0; i < m; ++i) {
    if (statuses[i].motor_id == 5 &&
        (statuses[i].limit_flags & MOTOR_PROTOCOL_LIMIT_POS_MAX) != 0U) {
      flagged = true;
    }
  }
  assert(flagged);
}

// 配置事务（vaxor SET_MODE）：事务跨服务节拍逐阶段推进，期间该电机不出
// 运动帧；完成后出结果事件、运动恢复（ARD §20.4/§20.6）。
static void test_vaxor_mode_transaction_gates_motion() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  motor_protocol_config_request_t mode = {};
  mode.request_id = 7U;
  mode.motor_id = 5;
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = MOTOR_PROTOCOL_MODE_MIT;
  assert(motor_control_submit_config(&ctx, &mode, 150));

  // 边界 1：事务进行中 → 只有 P2 control(run=0) 阶段帧，无 MIT。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 152);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 155));
  motor_control_service(&ctx, 160); // 阶段 0 发出并推进
  size_t n1 = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n1 == 1);
  assert(frames[0].can_id == motor_protocol_vaxor_control_id(5));
  assert(frames[0].size == 8U);
  assert(frames[0].tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);
  assert(frames[0].data[0] == 0U); // run=false

  // 边界 2：事务仍在 → 无 MIT；阶段 1 发出后事务完成。
  motor_control_command_snapshot_t s3 = make_snapshot(3, 162);
  snapshot_add(&s3, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 165));
  motor_control_service(&ctx, 170);
  size_t n2 = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n2 == 1);
  assert(frames[0].data[0] == 1U); // run=true
  // vaxor 模式切换无厂家确认对象（config_verify_required）→ 收敛 UNCONFIRMED
  // 而非 OK（ARD §20.5 / 禁止项 9：不谎报"已确认"）。
  assert(has_result_event(&ctx, 7U, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED));

  // 边界 3：事务完成 → 先发使能帧重走使能序列，再发 MIT（事务以 stop/start
  // 驱动过厂家使能序列，内核不复用事务前的 motion_enabled——若事务异常停在
  // stop 态，内核误认为已使能会让电机哑掉；强制重使能对两种结局都安全）。
  motor_control_command_snapshot_t s4 = make_snapshot(4, 172);
  snapshot_add(&s4, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s4, 175));
  motor_control_service(&ctx, 180);
  size_t n3 = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n3 >= 2);
  assert(frames[0].can_id == motor_protocol_vaxor_control_id(5));
  assert(frames[0].data[0] == 1U); // 重使能：run=true
  assert(frames[1].can_id == motor_protocol_vaxor_command_id(5));
  assert(frames[1].size == 12U); // MIT 恢复
}

// CERB 限位写：CANopen 快速 SDO 四阶段（写下限/读下限/写上限/读上限）→ OK。
static void test_cerb_limits_sdo_transaction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  // CERB 位置限位改走 CANopen 快速 SDO：六阶段（写下限/读下限/写上限/读上限/
  // **写 FLASH 0x2539 两次：4=负限位、2=正限位**）全通才 OK；读回不一致不得
  // 返回 OK（ARD §20.4 / 禁止项 9）。
  // ±1.0 rad → cnt = ±10430（0x28BE），字典标度 65536 cnt/360°。
  motor_protocol_config_request_t limits = {};
  limits.request_id = 9U;
  limits.motor_id = 7;
  limits.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  limits.position_limits.min_position_rad = -1.0f;
  limits.position_limits.max_position_rad = 1.0f;
  assert(motor_control_submit_config(&ctx, &limits, 105));

  // SDO 应答帧（0x580|node = 0x587，DLC 8 经典帧）。
  auto sdo_response = [](uint8_t cmd, uint8_t sub, uint32_t value, uint32_t tick) {
    uint8_t payload[8] = {cmd, 0x7DU, 0x60U, sub,
                          (uint8_t)(value & 0xFFU),
                          (uint8_t)((value >> 8U) & 0xFFU),
                          (uint8_t)((value >> 16U) & 0xFFU),
                          (uint8_t)((value >> 24U) & 0xFFU)};
    motor_protocol_raw_frame_t f = raw_frame(0, 0x587U, 8, payload, tick);
    f.is_fd = false; // 器件按字典发经典帧（DLC 恒 0x08）
    return f;
  };
  const uint8_t kPosLowerSdoSub = 1U;
  const uint8_t kPosUpperSdoSub = 2U;
  const uint32_t kLowerCnt = 0xFFFFD742U; // -10430
  const uint32_t kUpperCnt = 0x000028BEU; // +10430

  // 保持快照新鲜，避免 30ms 软停干扰事务（与本用例前半段同一手法）。
  uint32_t t = 110; // 发现(100)后 100ms 内跑完四阶段，避免被判离线
  motor_protocol_tx_frame_t frames[16];
  // 阶段 0：写下限
  motor_control_service(&ctx, t);
  {
    const size_t n = motor_control_collect_tx(&ctx, frames, 16);
    bool found = false;
    for (size_t i = 0; i < n; ++i) {
      if (frames[i].can_id == 0x607U && frames[i].size == 8U &&
          frames[i].data[0] == 0x23U && frames[i].data[3] == kPosLowerSdoSub) {
        found = true;
        assert(frames[i].data[4] == 0x42U && frames[i].data[5] == 0xD7U);
      }
    }
    assert(found);
  }
  motor_protocol_raw_frame_t r = sdo_response(0x60U, kPosLowerSdoSub, 0U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  // 阶段 1：读回下限（值一致）
  t += 10;
  motor_control_service(&ctx, t);
  r = sdo_response(0x43U, kPosLowerSdoSub, kLowerCnt, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  // 阶段 2：写上限
  t += 10;
  motor_control_service(&ctx, t);
  r = sdo_response(0x60U, kPosUpperSdoSub, 0U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  // 阶段 3：读回上限（值一致）
  t += 10;
  motor_control_service(&ctx, t);
  r = sdo_response(0x43U, kPosUpperSdoSub, kUpperCnt, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  // 阶段 4/5：写 FLASH（0x2539，无应答）——字典表：4=负限位、2=正限位。
  // 不按对应码保存，驱动器掉电就回出厂值，下次上电比对必然判"不一致"。
  const uint32_t kSaveCodes[2] = {4U, 2U};
  for (size_t s = 0; s < 2U; ++s) {
    t += 10;
    motor_control_service(&ctx, t);
    const size_t n = motor_control_collect_tx(&ctx, frames, 16);
    bool found = false;
    for (size_t i = 0; i < n; ++i) {
      if (frames[i].can_id == 0x607U && frames[i].data[0] == 0x23U &&
          frames[i].data[1] == 0x39U && frames[i].data[4] == (uint8_t)kSaveCodes[s]) {
        found = true;
      }
    }
    assert(found);
  }
  assert(has_result_event(&ctx, 9U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(!entry_of(&ctx, 7)->tx_active);

  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  // 走完使能整定门（期间持续喂新快照防 30ms 级联、喂 20ms 心跳防误判离线）。
  uint32_t seq = 2;
  for (uint32_t t = 145; t <= 245; t += 10) {
    motor_control_command_snapshot_t settle = make_snapshot(seq++, t - 5);
    snapshot_add(&settle, 7, 0.1f, 0.0f, 0.0f);
    assert(motor_control_apply_command_snapshot(&ctx, &settle, t));
    if ((t - 145) % 20 == 0) { // SYNC 上报心跳（20ms 间隔）
      motor_protocol_raw_frame_t hb = cerb_feedback(7, t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
  }

  motor_control_command_snapshot_t s2 = make_snapshot(seq++, 260);
  snapshot_add(&s2, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 265));
  motor_control_service(&ctx, 270);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // 运动中再写设备侧限位 → 拒绝：厂家要求设置限位时电机失能；收窄限位会让
  // 当前位置立即越限而触发厂家保护（0x0040/0x0080）。安全边界不得运动中变动。
  motor_protocol_config_request_t late = limits;
  late.request_id = 10U;
  assert(!motor_control_submit_config(&ctx, &late, 290));
  assert(has_result_event(&ctx, 10U, MOTOR_PROTOCOL_CONFIG_INVALID));
  assert(!entry_of(&ctx, 7)->tx_active); // 拒绝即不启动事务

  // 设零点同口径：厂家要求失能状态设置，运动中一律拒绝（不隐式失能）。
  motor_protocol_config_request_t zero = {};
  zero.request_id = 11U;
  zero.motor_id = 7;
  zero.kind = MOTOR_PROTOCOL_CONFIG_SET_ZERO;
  assert(!motor_control_submit_config(&ctx, &zero, 295));
  assert(has_result_event(&ctx, 11U, MOTOR_PROTOCOL_CONFIG_INVALID));
  assert(!entry_of(&ctx, 7)->tx_active);

}

// CERB SET_MODE MIT：设备恒 MIT → 单个空阶段事务 → 立即 OK。
static void test_cerb_set_mode_mit_immediate_ok() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_protocol_config_request_t mode = {};
  mode.request_id = 3U;
  mode.motor_id = 7;
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = MOTOR_PROTOCOL_MODE_MIT;
  assert(motor_control_submit_config(&ctx, &mode, 110));
  motor_control_service(&ctx, 120);
  motor_control_service(&ctx, 130);
  assert(has_result_event(&ctx, 3U, MOTOR_PROTOCOL_CONFIG_OK));
}

// 故障路径：vaxor 状态帧故障字非零 → FAULT 禁运动；故障消失 → READY_DISABLED。
static void test_vendor_fault_latches_and_recovers() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  motor_protocol_raw_frame_t faulted = vaxor_status(5, 0x12345678, 150);
  motor_control_on_raw_frame(&ctx, &faulted);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_FAULT, 5));

  // 故障恢复（厂商故障位清零）→ READY_DISABLED，需走正常使能链路。
  motor_protocol_raw_frame_t cleared = vaxor_status(5, 0, 160);
  motor_control_on_raw_frame(&ctx, &cleared);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}

// mock adapter：事务响应 ADVANCE → OK；重试超限 → 失败；
// 未决期间新请求 BUSY；结果事件携带 request_id（ARD §20.3/§20.4）。
static void test_mock_transaction_advance_retry_busy_timeout() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  uint8_t payload[4] = {};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x603, 4, payload, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.entry_count == 1);
  motor_control_set_agent_connected(&ctx, true, 110);
  assert(motor_control_set_authorization(&ctx, true, 115));
  // 绑定 mock Profile（SET_MODE 门禁要求身份已确认）。
  assert(motor_control_bind_profile_ptr(&ctx, 3, &g_mock_profile, 118));

  // 偶数 request_id → 响应 ADVANCE → OK。
  motor_protocol_config_request_t ok_req = {};
  ok_req.request_id = 2U;
  ok_req.motor_id = 3;
  ok_req.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  assert(motor_control_submit_config(&ctx, &ok_req, 120));
  motor_control_service(&ctx, 130); // 发出阶段帧，进入等待
  // 未决期间重复请求 → BUSY（ARD §20.3：不排队、不覆盖）。
  motor_protocol_config_request_t busy_req = ok_req;
  busy_req.request_id = 4U;
  assert(!motor_control_submit_config(&ctx, &busy_req, 135));
  assert(has_result_event(&ctx, 4U, MOTOR_PROTOCOL_CONFIG_BUSY));
  // 响应到达 → ADVANCE → OK。
  uint8_t response_payload[4] = {1, 0, 0, 0};
  motor_protocol_raw_frame_t response =
      raw_frame(0, 0x603, 4, response_payload, 140);
  motor_control_on_raw_frame(&ctx, &response);
  assert(has_result_event(&ctx, 2U, MOTOR_PROTOCOL_CONFIG_OK));

  // 响应内容不匹配（data[0]≠1）→ 每次裁决 RETRY，超过 2 次自动重试
  // → 读回不一致失败（UNCONFIRMED，禁止项 9）。
  motor_protocol_config_request_t retry_req = {};
  retry_req.request_id = 5U;
  retry_req.motor_id = 3;
  retry_req.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  assert(motor_control_submit_config(&ctx, &retry_req, 150));
  uint8_t mismatch_payload[4] = {0, 0, 0, 0};
  motor_protocol_raw_frame_t mismatch =
      raw_frame(0, 0x603, 4, mismatch_payload, 160);
  motor_control_service(&ctx, 160);            // 发出 #1
  motor_control_on_raw_frame(&ctx, &mismatch); // RETRY #1
  motor_control_service(&ctx, 170);            // 重发 #2
  motor_control_on_raw_frame(&ctx, &mismatch); // RETRY #2
  motor_control_service(&ctx, 180);            // 重发 #3
  motor_control_on_raw_frame(&ctx, &mismatch); // 第 3 次 RETRY → 超限
  assert(has_result_event(&ctx, 5U, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED));

  // 无响应超时路径：每阶段 50ms 超时，≤2 次重试后 TIMEOUT。
  motor_protocol_config_request_t silent = {};
  silent.request_id = 8U;
  silent.motor_id = 3;
  silent.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  assert(motor_control_submit_config(&ctx, &silent, 200));
  motor_control_service(&ctx, 210); // 发出，deadline 260
  motor_control_service(&ctx, 255); // 未到期
  motor_control_service(&ctx, 261); // 超时重试 #1
  motor_control_service(&ctx, 311); // 超时重试 #2
  motor_control_service(&ctx, 361); // 重试耗尽 → TIMEOUT
  assert(has_result_event(&ctx, 8U, MOTOR_PROTOCOL_CONFIG_TIMEOUT));
}

// 发现探测：CERB 发 0x080 SYNC（P2）；仅 vaxor 注册表 → 拒绝。
static void test_probe_discovery_cerb_only() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  assert(motor_control_probe_discovery(&ctx, 1));
  motor_protocol_tx_frame_t frames[4];
  size_t n = flush_tx(&ctx, frames, 4);
  assert(n == 1);
  assert(frames[0].can_id == 0x080U);
  assert(frames[0].tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);

  motor_control_context_t ctx2;
  const motor_protocol_adapter_t *vaxor_only[1] = {&motor_protocol_vaxor_adapter};
  motor_control_init(&ctx2, vaxor_only, 1);
  assert(!motor_control_probe_discovery(&ctx2, 0));
}

// 事件环：环满挤掉并计数（不静默），drain 消费。
static void test_event_ring_drains_and_counts_drops() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  // 溢出环容量 8 条 → 满丢最旧、保最新，丢弃计数可见。
  // 事件源用「整快照拒绝」：每次非法快照恰好产出一条 SNAPSHOT_REJECTED，
  // 不受设备数限制。原先靠给 N 台设备各喂一帧首帧来灌环，但厂家 node 空间
  // 只有 1..127，环深提到 128 之后就喂不满了——那是测试手段的边界，不是
  // 产品的边界。
  const uint32_t overflow = 8U;
  const uint32_t pushed = MOTOR_CONTROL_MAX_EVENTS + overflow;
  motor_control_command_snapshot_t bad = make_snapshot(1U, 100U);
  bad.target_count = (uint8_t)(MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS + 1U);
  for (uint32_t i = 0; i < pushed; ++i) {
    assert(!motor_control_apply_command_snapshot(&ctx, &bad, 100U + i));
  }
  assert(ctx.snapshot_rejected_count == pushed); // 每次拒绝都出事件，不静默
  motor_control_event_t events[MOTOR_CONTROL_MAX_EVENTS];
  size_t total = 0;
  size_t n;
  while ((n = motor_control_drain_events(&ctx, events,
                                         MOTOR_CONTROL_MAX_EVENTS)) > 0) {
    total += n;
  }
  assert(total == MOTOR_CONTROL_MAX_EVENTS);
  assert(ctx.event_dropped_count == overflow);
}

// TX 收集：容量不足时余量保留（不丢），续取可得；P0 优先出队。
static void test_collect_tx_capacity_and_remainder() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  bring_vaxor_to_ready(&ctx, 6, "vaxor_2550", 110);
  motor_control_command_snapshot_t s = make_snapshot(1, 130);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s, 6, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 140));
  motor_control_service(&ctx, 150);

  motor_protocol_tx_frame_t frames[2];
  size_t n1 = flush_tx(&ctx, frames, 2);
  assert(n1 == 2);
  size_t n2 = flush_tx(&ctx, frames, 2);
  assert(n2 == 2);
  assert(ctx.tx_dropped_count == 0);
}

// 状态副本：state_age_ms 用单调时基；无反馈 = NaN。
static void test_motion_state_age_and_nan() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_protocol_raw_frame_t f = vaxor_reply(5, 1.0, 0.0, 0.0, 400);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_service(&ctx, 450);

  motor_control_motion_state_t states[4];
  size_t n = motor_control_copy_motion_states(&ctx, states, 4, 450);
  assert(n == 1);
  assert(states[0].motor_id == 5);
  assert(states[0].state_age_ms == 50);
  assert(fabs(states[0].position_rad - 1.0 / 50.0) < 1e-4);

  // 无反馈电机 → 位置可解（CERB 关节侧直通，无需 Profile），速度/扭矩
  // 依赖 Profile（未绑定 → NaN），仍在发现态。
  motor_protocol_raw_frame_t cf = cerb_feedback(9, 410);
  motor_control_on_raw_frame(&ctx, &cf);
  motor_control_service(&ctx, 460);
  motor_control_motion_state_t states2[8];
  size_t m = motor_control_copy_motion_states(&ctx, states2, 8, 460);
  assert(m == 2);
  for (size_t i = 0; i < m; ++i) {
    if (states2[i].motor_id == 9) {
      assert(fabs(states2[i].position_rad - 10.0 * M_PI / 180.0) < 1e-4);
      assert(std::isnan(states2[i].velocity_rad_s));
      assert(std::isnan(states2[i].effort_nm));
      assert(states2[i].lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED);
    }
  }
}

// 未知帧计数（不归属任何协议的 CAN ID、非法节点号）。
static void test_unknown_frame_counted() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = raw_frame(0, 0x123, 8, nullptr, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.unknown_frame_count == 1);
  assert(ctx.entry_count == 0);
}

// 诊断输出：统一故障码 + 厂家故障字透传 + 公共保护位。
static void test_status_copy_fields() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_protocol_raw_frame_t faulted = vaxor_status(5, 0xdeadbeef, 120);
  motor_control_on_raw_frame(&ctx, &faulted);
  motor_control_status_t statuses[4];
  size_t n = motor_control_copy_statuses(&ctx, statuses, 4, 130);
  assert(n == 1);
  assert(statuses[0].motor_id == 5);
  assert(statuses[0].unified_fault_code == MOTOR_CONTROL_FAULT_VENDOR);
  assert(statuses[0].vendor_fault_code == 0xdeadbeefU);
  assert((statuses[0].protection_flags & MOTOR_PROTOCOL_PROT_VENDOR_FAULT) != 0U);
  assert(statuses[0].lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT);
}

// Profile 绑定拒绝：越界索引 / 协议不符 → 拒绝 + 事件 + 保持不可控；
// 正确绑定后收敛 READY_DISABLED。
static void test_bind_profile_rejection() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);

  // 越界索引与协议不匹配都要拒绝。
  assert(!motor_control_bind_profile(&ctx, 5, motor_control_profile_count(), 110));
  size_t cerb_idx;
  assert(find_profile("cerb_r58_101", &cerb_idx));
  assert(!motor_control_bind_profile(&ctx, 5, cerb_idx, 115));
  assert(entry_of(&ctx, 5)->profile == nullptr);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 5));
  // 正确绑定仍可继续。
  size_t vaxor_idx;
  assert(find_profile("vaxor_2550", &vaxor_idx));
  assert(motor_control_bind_profile(&ctx, 5, vaxor_idx, 120));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}

// ---------------------------------------------------------------------------
// 安全边界回归覆盖
// ---------------------------------------------------------------------------

// 使能整定窗内的电机属于受控集合：快照缺席即软停。
static void test_settle_window_absence_soft_stops() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130)); // 使能帧，整定中
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(entry_of(&ctx, 7)->motion_enabled);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  // 空快照（合法）＝该电机缺席 → 整定窗内也必须软停（P0 阻尼帧 + SAFE_STOP）。
  motor_control_command_snapshot_t absent = make_snapshot(2, 150); // 0 目标
  assert(motor_control_apply_command_snapshot(&ctx, &absent, 160));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool found_damping = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0 &&
        frames[i].can_id == 0x110U + 7U && frames[i].size == 12U) {
      found_damping = true; // 软停阻尼帧（使能位保持、kd=默认阻尼）
    }
  }
  assert(found_damping);

  // 心跳保活（SYNC 上报帧承担总线检测，静默 100ms 即离线）。
  motor_protocol_raw_frame_t hb1 = cerb_feedback(7, 200);
  motor_control_on_raw_frame(&ctx, &hb1);
  motor_protocol_raw_frame_t hb2 = cerb_feedback(7, 290);
  motor_control_on_raw_frame(&ctx, &hb2);

  // 电机重新出现在新鲜快照 → 恢复 ACTIVE（整定到期由 service 清除后下发）。
  motor_control_command_snapshot_t s2 = make_snapshot(3, 305);
  snapshot_add(&s2, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 310)); // 整定未清，仅记录
  motor_control_service(&ctx, 315); // 整定到期（enable_pending 清除）
  motor_control_command_snapshot_t s3 = make_snapshot(4, 320);
  snapshot_add(&s3, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 325));
  motor_control_service(&ctx, 330);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 缓冲满时 P0 失能帧驱逐非 P0 帧入队（ARD §22.1），不得静默丢失。
static void test_p0_preempts_full_buffer() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  uint8_t payload[4] = {};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x603, 4, payload, 100);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_set_agent_connected(&ctx, true, 110);
  assert(motor_control_set_authorization(&ctx, true, 115));
  assert(motor_control_bind_profile_ptr(&ctx, 3, &g_mock_profile, 118));

  // 不 collect，连续边界把 tx_pending 堆满（每边界 1 帧 MIT）；
  // 期间持续喂 mock 自主上报帧，防止 1s 无反馈误判离线。
  uint32_t seq = 1;
  for (uint32_t t = 130; t < 130 + 200 * 10; t += 10) {
    motor_control_command_snapshot_t s = make_snapshot(seq++, t - 5);
    snapshot_add(&s, 3, 0.1f, 0.0f, 0.0f);
    assert(motor_control_apply_command_snapshot(&ctx, &s, t));
    uint8_t alive[4] = {};
    motor_protocol_raw_frame_t keepalive = raw_frame(0, 0x603, 4, alive, t - 2);
    motor_control_on_raw_frame(&ctx, &keepalive);
    motor_control_service(&ctx, t);
  }
  assert(ctx.tx_count == MOTOR_CONTROL_MAX_TX_PENDING);
  const uint32_t dropped_before = ctx.tx_dropped_count;
  assert(dropped_before > 0); // P1 溢出路径可见

  // 缓冲满时断链 → P0 失能帧必须仍然入队（驱逐一个非 P0 帧）。
  motor_control_set_agent_connected(&ctx, false, 2200);
  assert(ctx.tx_dropped_count > dropped_before); // 驱逐计入丢弃
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool found_disable = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].tx_class == MOTOR_PROTOCOL_TX_CLASS_P0 &&
        frames[i].can_id == 0x620U + 3U) {
      found_disable = true;
    }
  }
  assert(found_disable);
  assert(!entry_of(&ctx, 3)->p0_pending); // 未挂起重试（已成功入队）
}

// 过期快照门（ARD §51.3）：>30ms 软停、>100ms 撤权，且不记录、
// 不复位级联标志。
static void test_stale_snapshot_gates() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // 30<age≤100：软停级联，快照不被记录（序号 2 尚未被消费）。
  motor_control_command_snapshot_t stale30 = make_snapshot(2, 205);
  snapshot_add(&stale30, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &stale30, 240)); // age=35
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(ctx.last_snapshot_sequence == 1U); // 未记录

  // age>100：撤权 + 协议级失能。
  motor_control_command_snapshot_t stale100 = make_snapshot(3, 105);
  snapshot_add(&stale100, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &stale100, 260)); // age=155
  assert(!ctx.authorized);
  assert(ctx.require_new_snapshot);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 恢复链：新授权 + 序号 2 的新鲜快照（过期快照未消费序号）→ ACTIVE。
  assert(motor_control_set_authorization(&ctx, true, 270));
  motor_control_command_snapshot_t fresh = make_snapshot(2, 275);
  snapshot_add(&fresh, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &fresh, 280));
  motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, 285);
  motor_control_on_raw_frame(&ctx, &keepalive); // 自主上报保活，防误判离线
  motor_control_service(&ctx, 290);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// CONFIG_CONFLICT 锁存穿越 OFFLINE/重连，不自动解除（§9.3）。
static void test_conflict_survives_offline_reconnect() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t vaxor_first = vaxor_reply(7, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &vaxor_first);
  motor_protocol_raw_frame_t cerb_conflict = cerb_feedback(7, 110);
  motor_control_on_raw_frame(&ctx, &cerb_conflict); // 同 node 不同协议 → 冲突
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);

  // 长静默：冲突态不降级 OFFLINE（§9.3 —— 离线不得成为解除冲突的通道）。
  motor_control_service(&ctx, 300);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);

  // 设备回来 → 冲突保持锁存（不得自动解除）。
  motor_protocol_raw_frame_t vaxor_back = vaxor_reply(7, 0.0, 0.0, 0.0, 400);
  motor_control_on_raw_frame(&ctx, &vaxor_back);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  motor_control_service(&ctx, 410);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 识别只在服务节拍上发生：帧路径只推进 DISCOVERED → IDENTIFYING，不绑定
// （识别要发事务帧，属于服务路径的职责，RX 路径不做总线输出）。
// DISCOVERED→IDENTIFYING 中间态可观测（ARD §43.1 第 1 条）。
static void test_identifying_state_observed() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f1 = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f1);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED);
  motor_protocol_raw_frame_t f2 = vaxor_reply(5, 0.0, 0.0, 0.0, 120);
  motor_control_on_raw_frame(&ctx, &f2);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING);
}

// 事件携带 MCU 单调时戳（ARD §18.2）。
static void test_event_carries_tick() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 777);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_event_t events[64];
  size_t n;
  bool tick_ok = false;
  while ((n = motor_control_drain_events(&ctx, events, 64)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_DISCOVERED &&
          events[i].tick_ms == 777U) {
        tick_ok = true;
      }
    }
  }
  assert(tick_ok);
}

// P1 溢出丢弃计数可见（P0 路径由 test_p0_preempts_full_buffer 覆盖）。
static void test_tx_drop_counted() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  uint8_t payload[4] = {};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x603, 4, payload, 100);
  motor_control_on_raw_frame(&ctx, &f);
  motor_control_set_agent_connected(&ctx, true, 110);
  assert(motor_control_set_authorization(&ctx, true, 115));
  assert(motor_control_bind_profile_ptr(&ctx, 3, &g_mock_profile, 118));
  uint32_t seq = 1;
  // 迭代次数按容量推：本用例的意图是「灌满再灌一条」，写死数字会在容量
  // 调整后静默失去覆盖（本任务把它从 96 提到 160 时就撞上了）。
  const uint32_t pushes = MOTOR_CONTROL_MAX_TX_PENDING + 24U;
  for (uint32_t t = 130; t < 130 + pushes * 10; t += 10) {
    motor_control_command_snapshot_t s = make_snapshot(seq++, t - 5);
    snapshot_add(&s, 3, 0.1f, 0.0f, 0.0f);
    assert(motor_control_apply_command_snapshot(&ctx, &s, t));
    uint8_t alive[4] = {};
    motor_protocol_raw_frame_t keepalive = raw_frame(0, 0x603, 4, alive, t - 2);
    motor_control_on_raw_frame(&ctx, &keepalive);
    motor_control_service(&ctx, t);
  }
  assert(ctx.tx_dropped_count > 0); // 满即丢且计数（P1 路径）
}

// SET_MODE 门禁：未识别（无 Profile）电机不得受理（INVALID）。
static void test_set_mode_gating_on_unidentified() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f); // DISCOVERED，未绑定 Profile
  motor_protocol_config_request_t mode = {};
  mode.request_id = 11U;
  mode.motor_id = 5;
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = 0U;
  assert(!motor_control_submit_config(&ctx, &mode, 120));
  assert(has_result_event(&ctx, 11U, MOTOR_PROTOCOL_CONFIG_INVALID));
}

// 逐电机 Enable（MotorPolicy）：false 立即失能并冻结，
// true 重立授权（仍需新快照）；OFFLINE 冻结后必须显式重发 Enable。
static void test_policy_enable_disables_and_arms() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  while (flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
  }

  // enable=false：立即软停+失能，清逐电机授权，出 DISABLE 事件。
  motor_protocol_config_request_t disable = {};
  disable.request_id = 1U;
  disable.motor_id = 5;
  disable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  disable.enable = false;
  assert(motor_control_submit_config(&ctx, &disable, 150));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(!entry_of(&ctx, 5)->policy_enable);
  assert(!entry_of(&ctx, 5)->motion_authorized);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) >= 1); // 失能帧

  // 冻结中：新快照也不会恢复运动。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 170);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 180));
  motor_control_service(&ctx, 190);
  assert(entry_of(&ctx, 5)->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // enable=true：重立授权；下一个新鲜快照恢复 ACTIVE。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 2U;
  enable.motor_id = 5;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 200));
  assert(entry_of(&ctx, 5)->policy_enable);
  assert(entry_of(&ctx, 5)->motion_authorized);
  motor_control_command_snapshot_t s3 = make_snapshot(3, 210);
  snapshot_add(&s3, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 220));
  motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, 225);
  motor_control_on_raw_frame(&ctx, &keepalive); // 自主上报保活，防误判离线
  motor_control_service(&ctx, 230);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 厂家故障恢复语义：FAULT 清除回 READY_DISABLED 后，逐电机
// motion_authorized 已在故障锁存时清除——快照重现**不得**自动恢复运动；
// 必须显式 SET_ENABLE(true) 复立授权位（"失败不自动恢复运动授权"）。
// 全局重授权只重立非冻结态（会话派生授权的安全边界）：FAULT/OFFLINE/
// CONFIG_CONFLICT 表项的逐电机授权位不被复活，恢复只能复位控制板。
static void test_global_reauth_skips_frozen_lifecycles() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_protocol_raw_frame_t faulted = vaxor_status(5, 0x12345678, 125);
  motor_control_on_raw_frame(&ctx, &faulted);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT);
  (void)motor_control_set_authorization(&ctx, true, 130);
  assert(!entry_of(&ctx, 5)->motion_authorized); // FAULT 不复活
  assert(ctx.authorized);

  // 故障清除 → READY_DISABLED：全局位可重立，但 policy_enable 冻结仍挡命令。
  motor_protocol_raw_frame_t cleared = vaxor_status(5, 0, 135);
  motor_control_on_raw_frame(&ctx, &cleared);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  (void)motor_control_set_authorization(&ctx, true, 140);
  assert(entry_of(&ctx, 5)->motion_authorized);
  assert(!entry_of(&ctx, 5)->policy_enable); // 仍须复位才能运动
}

static void test_fault_recovery_requires_reauth() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 故障锁存：20B 状态帧故障字非零 → FAULT + P0 失能帧。
  motor_protocol_raw_frame_t faulted = vaxor_status(5, 0x12345678, 125);
  motor_control_on_raw_frame(&ctx, &faulted);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_FAULT, 5));
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING); // 冲走 P0 失能帧

  // 故障清除 → READY_DISABLED；逐电机授权位保持清除。
  motor_protocol_raw_frame_t cleared = vaxor_status(5, 0, 130);
  motor_control_on_raw_frame(&ctx, &cleared);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 新完整快照重现 → 不得自动恢复运动（授权位已清）。
  motor_control_command_snapshot_t s2 = make_snapshot(2, 135);
  snapshot_add(&s2, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 140));
  motor_control_service(&ctx, 145);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 0); // 无使能帧、无 MIT 帧

  // 显式 SET_ENABLE(true) 复立授权位 → 快照恢复正常运动。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 9U;
  enable.motor_id = 5;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 150));
  motor_control_command_snapshot_t s3 = make_snapshot(3, 155);
  snapshot_add(&s3, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 160));
  motor_control_service(&ctx, 165);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 故障/冲突锁存必须冻结 policy_enable。全局 set_authorization(true) 会
// 重立全部表项 motion_authorized，若 policy_enable 残留 true，故障清除后仅
// 重发全局授权 + 新快照即自动复运动——违反 ARD §36「失败不自动恢复」。
// 修复后：撤权→再授权 + 新快照 ⇒ 0 运动帧；显式 SET_ENABLE(true) 才恢复。
static void test_fault_recovery_no_motion_without_explicit_enable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 7, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 故障锁存 → 厂商故障位清零 → READY_DISABLED。
  motor_protocol_raw_frame_t faulted = vaxor_status(7, 0x12345678, 150);
  motor_control_on_raw_frame(&ctx, &faulted);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_FAULT);
  motor_protocol_raw_frame_t cleared = vaxor_status(7, 0, 160);
  motor_control_on_raw_frame(&ctx, &cleared);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 撤权 → 再授权（全局重立 motion_authorized）+ 新快照：修复前 policy_enable
  // 残留 true → 运动自动恢复（缺陷路径）；修复后必须 0 帧。
  assert(motor_control_set_authorization(&ctx, false, 170));
  assert(motor_control_set_authorization(&ctx, true, 175));
  motor_control_command_snapshot_t s2 = make_snapshot(2, 180);
  snapshot_add(&s2, 7, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 185));
  motor_control_service(&ctx, 190);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 0); // 无使能帧、无 MIT 帧：policy_enable 冻结生效

  // 显式 SET_ENABLE(true) → 快照恢复运动。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 21U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 200));
  motor_control_command_snapshot_t s3 = make_snapshot(3, 210);
  snapshot_add(&s3, 7, 0.3f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 215));
  motor_control_service(&ctx, 220);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 冲突锁存同样冻结 policy_enable：同 node 换协议（vaxor→cerb）即冲突。
  motor_protocol_raw_frame_t cross = cerb_feedback(7, 230);
  motor_control_on_raw_frame(&ctx, &cross);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(!entry_of(&ctx, 7)->policy_enable);
}

// 撤权/离线中止进行中配置事务时必须回 CONFIG_UNCONFIRMED 终态：
// 七码无「中止」码（msgs 冻结），UNCONFIRMED（写入效果无法确认）语义最诚实；
// 否则上层请求会悬挂到自身超时。
static void test_revoke_aborts_tx_with_unconfirmed() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  uint8_t payload[4] = {};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x602, 4, payload, 100); // node 2
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.entry_count == 1);
  assert(motor_control_bind_profile_ptr(&ctx, 2, &g_mock_profile, 110));
  motor_control_set_agent_connected(&ctx, true, 112);
  assert(motor_control_set_authorization(&ctx, true, 114));
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  motor_protocol_config_request_t mode_req = {};
  mode_req.request_id = 7U;
  mode_req.motor_id = 2;
  mode_req.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode_req.mode = 1U;
  assert(motor_control_submit_config(&ctx, &mode_req, 116));
  assert(entry_of(&ctx, 2)->tx_active);
  motor_control_service(&ctx, 118); // 发出阶段帧，进入等待
  assert(entry_of(&ctx, 2)->tx_awaiting);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 撤权 → 事务中止 → UNCONFIRMED 终态回告（带原 request_id）。
  assert(motor_control_set_authorization(&ctx, false, 150));
  assert(!entry_of(&ctx, 2)->tx_active);
  assert(has_result_event(&ctx, 7U, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED));
}

static void test_offline_aborts_tx_with_unconfirmed() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  uint8_t payload[4] = {};
  motor_protocol_raw_frame_t f = raw_frame(0, 0x602, 4, payload, 100); // node 2
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.entry_count == 1);
  assert(motor_control_bind_profile_ptr(&ctx, 2, &g_mock_profile, 110));
  motor_control_set_agent_connected(&ctx, true, 112);
  assert(motor_control_set_authorization(&ctx, true, 114));
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  motor_protocol_config_request_t mode_req = {};
  mode_req.request_id = 7U;
  mode_req.motor_id = 2;
  mode_req.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode_req.mode = 1U;
  assert(motor_control_submit_config(&ctx, &mode_req, 116));
  assert(entry_of(&ctx, 2)->tx_active);
  motor_control_service(&ctx, 118); // 发出阶段帧，进入等待
  assert(entry_of(&ctx, 2)->tx_awaiting);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 停心跳：服务节拍直接跳过事务 50ms 超时（否则 TIMEOUT 先收敛），离线检查
  // 先于 config_step 执行 → 事务被离线中止（而非 TIMEOUT）。
  motor_control_service(&ctx, 1200);
  assert(entry_of(&ctx, 2)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);
  assert(!entry_of(&ctx, 2)->tx_active);
  assert(has_result_event(&ctx, 7U, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED));
}

// 软停帧编码失败与失能路径同语义：挂起 p0_pending 由 service
// 重试；重试仍失败则计数并放弃（不静默丢安全帧）。
static bool mock_encode_soft_stop_fail(const motor_protocol_model_profile_t *p,
                                       uint16_t node, uint8_t bus,
                                       motor_protocol_tx_frame_t *out) {
  (void)p; (void)node; (void)bus; (void)out;
  return false;
}

static void test_soft_stop_encode_failure_pends_retry() {
  motor_control_context_t ctx;
  motor_protocol_adapter_t fail_adapter = g_mock_adapter;
  fail_adapter.encode_soft_stop = mock_encode_soft_stop_fail;
  motor_protocol_model_profile_t fail_profile = g_mock_profile;
  fail_profile.protocol = &fail_adapter;
  const motor_protocol_adapter_t *adapters[1] = {&fail_adapter};
  motor_control_init(&ctx, adapters, 1);
  motor_protocol_raw_frame_t f = {};
  f.bus_index = 0;
  f.can_id = 0x600U + 3U;
  f.dlc = 4U;
  f.rx_tick_ms = 100;
  f.data[0] = 1U;
  motor_control_on_raw_frame(&ctx, &f);
  assert(motor_control_bind_profile_ptr(&ctx, 3, &fail_profile, 100));
  motor_control_set_agent_connected(&ctx, true, 100);
  assert(motor_control_set_authorization(&ctx, true, 100));
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 3, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  assert(entry_of(&ctx, 3)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  // 编码失败 → 挂起 + SAFE_STOP（不静默丢帧）。
  motor_control_command_snapshot_t empty = make_snapshot(2, 130);
  assert(motor_control_apply_command_snapshot(&ctx, &empty, 135));
  const motor_control_entry_t *entry = entry_of(&ctx, 3);
  assert(entry->p0_pending);
  assert(entry->p0_kind == 1U); // 公共头注释：0=无，1=软停，2=失能，3=使能
  assert(entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  // service 重试：编码仍失败 → 放弃并计数（可见，不静默）。
  motor_control_service(&ctx, 140);
  assert(!entry_of(&ctx, 3)->p0_pending);
  assert(entry_of(&ctx, 3)->protocol_error_count >= 1);
}

// P1 槽位：MIT/使能帧携带表项索引槽（tx_slot），供任务层把帧路由到
// per-bus 调度器的 latest-wins 槽。两台电机（先 5 后 6）→ 槽 0/1。
static void test_tx_slot_maps_entry_index() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  bring_vaxor_to_ready(&ctx, 6, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s, 6, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool saw_mit_5 = false;
  bool saw_mit_6 = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].can_id == motor_protocol_vaxor_command_id(6)) {
      assert(frames[i].tx_slot == 1);
      saw_mit_6 = true;
    }
    if (frames[i].can_id == motor_protocol_vaxor_command_id(5)) {
      assert(frames[i].tx_slot == 0);
      saw_mit_5 = true;
    }
  }
  assert(saw_mit_5 && saw_mit_6);
}

// 内核生成的软停/失能 P0 帧必须携带表项索引（tx_slot），供下游调度器
// FIFO 满回告 motor_control_requeue_p0 按 tx_slot 定位表项，tx_slot 恒 0 会
// 把 p0_pending 错挂表项 0。断言主体用表项索引 ≥1 的电机（motor 6 = entry 1）。
static void test_p0_tx_slot_on_soft_stop_and_disable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  bring_vaxor_to_ready(&ctx, 6, "vaxor_2550", 100);
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s, 6, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 空快照 → 双双软停：P0 软停帧 tx_slot 各归其位（vaxor 软停复用命令 CAN-ID）。
  motor_control_command_snapshot_t empty = make_snapshot(2, 130);
  assert(motor_control_apply_command_snapshot(&ctx, &empty, 135));
  motor_control_service(&ctx, 140);
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool saw_soft_5 = false;
  bool saw_soft_6 = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].tx_class != MOTOR_PROTOCOL_TX_CLASS_P0) {
      continue;
    }
    if (frames[i].can_id == motor_protocol_vaxor_command_id(6)) {
      assert(frames[i].tx_slot == 1);
      saw_soft_6 = true;
    }
    if (frames[i].can_id == motor_protocol_vaxor_command_id(5)) {
      assert(frames[i].tx_slot == 0);
      saw_soft_5 = true;
    }
  }
  assert(saw_soft_5 && saw_soft_6);

  // 撤权 → 双双失能：P0 失能帧 tx_slot 各归其位（回告重入队按此定位表项）。
  // vaxor 失能帧走 encode_control → control CAN-ID（0x502+4n），与软停的
  // 命令 CAN-ID（0x501+4n）不同——两侧都要按各自 ID 断言 tx_slot。
  assert(motor_control_set_authorization(&ctx, false, 150));
  n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool saw_dis_5 = false;
  bool saw_dis_6 = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].tx_class != MOTOR_PROTOCOL_TX_CLASS_P0) {
      continue;
    }
    if (frames[i].can_id == motor_protocol_vaxor_control_id(6)) {
      assert(frames[i].tx_slot == 1);
      saw_dis_6 = true;
    }
    if (frames[i].can_id == motor_protocol_vaxor_control_id(5)) {
      assert(frames[i].tx_slot == 0);
      saw_dis_5 = true;
    }
  }
  assert(saw_dis_5 && saw_dis_6);
}

// 重编码路径：软停编码首拍失败挂起 → service 重编码补发，重编码帧同样
// 必须携带正确 tx_slot（service_p0_retries 曾恒 0）。两台 mock 电机（node 2/3
// = 表项 0/1），断言主体是表项 1。
static bool g_soft_stop_fail_countdown = false;

static bool mock_encode_soft_stop_fail_once(const motor_protocol_model_profile_t *p,
                                            uint16_t node, uint8_t bus,
                                            motor_protocol_tx_frame_t *out) {
  if (g_soft_stop_fail_countdown) {
    g_soft_stop_fail_countdown = false;
    (void)p; (void)node; (void)bus; (void)out;
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->bus_index = bus;
  out->tx_class = MOTOR_PROTOCOL_TX_CLASS_P1; // 内核会覆写为 P0
  out->can_id = 0x620U + node;
  out->size = 1U;
  return true;
}

static void test_p0_tx_slot_on_retry_reencode() {
  motor_control_context_t ctx;
  motor_protocol_adapter_t adapter = g_mock_adapter;
  adapter.encode_soft_stop = mock_encode_soft_stop_fail_once;
  motor_protocol_model_profile_t profile = g_mock_profile;
  profile.protocol = &adapter;
  const motor_protocol_adapter_t *adapters[1] = {&adapter};
  motor_control_init(&ctx, adapters, 1);

  // 两台 mock 电机：node 2 → 表项 0，node 3 → 表项 1（断言主体）。
  for (uint16_t node = 2; node <= 3; ++node) {
    motor_protocol_raw_frame_t f = {};
    f.bus_index = 0;
    f.can_id = 0x600U + node;
    f.dlc = 4U;
    f.rx_tick_ms = 100;
    f.data[0] = 1U;
    motor_control_on_raw_frame(&ctx, &f);
    assert(motor_control_bind_profile_ptr(&ctx, node, &profile, 100));
  }
  motor_control_set_agent_connected(&ctx, true, 100);
  assert(motor_control_set_authorization(&ctx, true, 100));
  motor_control_command_snapshot_t s = make_snapshot(1, 110);
  snapshot_add(&s, 2, 0.1f, 0.0f, 0.0f);
  snapshot_add(&s, 3, 0.2f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 115));
  motor_control_service(&ctx, 120);
  assert(entry_of(&ctx, 2)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(entry_of(&ctx, 3)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 首拍失败：空快照软停，node 2 编码失败挂起，node 3 编码成功出帧。
  g_soft_stop_fail_countdown = true;
  motor_control_command_snapshot_t empty = make_snapshot(2, 130);
  assert(motor_control_apply_command_snapshot(&ctx, &empty, 135));
  assert(entry_of(&ctx, 2)->p0_pending);
  motor_control_service(&ctx, 140); // 重编码 node 2 的挂起软停帧
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  bool saw_reencode_2 = false;
  bool saw_first_3 = false;
  for (size_t i = 0; i < n; ++i) {
    if (frames[i].tx_class != MOTOR_PROTOCOL_TX_CLASS_P0) {
      continue;
    }
    if (frames[i].can_id == 0x620U + 3U) {
      assert(frames[i].tx_slot == 1); // 首拍成功帧：表项 1（pre-fix 恒 0）
      saw_first_3 = true;
    }
    if (frames[i].can_id == 0x620U + 2U) {
      assert(frames[i].tx_slot == 0); // 重编码补发帧：表项 0
      saw_reencode_2 = true;
    }
  }
  assert(saw_first_3 && saw_reencode_2);
  assert(!entry_of(&ctx, 2)->p0_pending);
}

// 过期 P0 挂起帧必须在使能帧成功出队时作废：撤权期间队列满留下的挂起
// 失能重试帧，不得在重新使能后被 service 补发（否则刚使能的电机被旧
// 失能帧再次打回）。CERB 有 100ms 使能整定窗——使能与首帧 MIT 之间隔
// 多个 service 节拍，正是旧失能帧复发的窗口；无整定门协议使能与 MIT 同
// 边界出队，由 MIT 成功路径作废，不经过本窗口。填队列用回告重入队假帧；
// 溢出那次转挂起的类别随后会被撤权路径以 DISABLE 覆盖，不影响断言。
static void test_stale_pending_disable_cleared_by_enable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];

  // 使能 → 整定窗 → ACTIVE（与 test_cerb_enable_settle_gate 同节奏）。
  motor_control_command_snapshot_t s = make_snapshot(1, 120);
  snapshot_add(&s, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130));
  motor_control_service(&ctx, 140);
  motor_control_command_snapshot_t s2 = make_snapshot(2, 150);
  snapshot_add(&s2, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 160));
  motor_protocol_raw_frame_t hb1 = cerb_feedback(7, 158);
  motor_control_on_raw_frame(&ctx, &hb1);
  motor_control_service(&ctx, 170);
  motor_control_command_snapshot_t s3 = make_snapshot(3, 180);
  snapshot_add(&s3, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s3, 185));
  motor_protocol_raw_frame_t hb2 = cerb_feedback(7, 188);
  motor_control_on_raw_frame(&ctx, &hb2);
  motor_control_service(&ctx, 190);
  motor_control_command_snapshot_t s4 = make_snapshot(4, 220);
  snapshot_add(&s4, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s4, 225));
  motor_protocol_raw_frame_t hb3 = cerb_feedback(7, 228);
  motor_control_on_raw_frame(&ctx, &hb3);
  motor_control_service(&ctx, 235); // 整定 105ms ≥ 100ms → 到期
  motor_control_command_snapshot_t s5 = make_snapshot(5, 245);
  snapshot_add(&s5, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s5, 250));
  motor_protocol_raw_frame_t hb4 = cerb_feedback(7, 252);
  motor_control_on_raw_frame(&ctx, &hb4);
  motor_control_service(&ctx, 260);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // P0 待发队列填满（回告重入队假帧；容量 MAX_TX_PENDING → MAX+1 次
  // 调用才溢出，最后一次失败返回 false）。
  bool last_requeue = true;
  for (uint32_t k = 0; k <= MOTOR_CONTROL_MAX_TX_PENDING; ++k) {
    motor_protocol_tx_frame_t fill = {};
    fill.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
    fill.tx_slot = 0; // motor 7 = 表项 0
    fill.can_id = 0x700U;
    fill.size = 1U;
    last_requeue = motor_control_requeue_p0(&ctx, &fill);
  }
  assert(!last_requeue);

  // 撤权：失能帧推送失败（队列全 P0 无可驱逐）→ 转挂起重试。
  assert(motor_control_set_authorization(&ctx, false, 270));
  assert(entry_of(&ctx, 7)->p0_pending);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);

  // 重新授权 + 新完整快照 → 使能帧出队（整定窗开启）→ 过期挂起必须
  // 同时作废；否则 service 在整定窗内把旧失能帧补发出门。
  motor_protocol_raw_frame_t hb5 = cerb_feedback(7, 278);
  motor_control_on_raw_frame(&ctx, &hb5);
  assert(motor_control_set_authorization(&ctx, true, 280));
  motor_control_command_snapshot_t s6 = make_snapshot(6, 285);
  snapshot_add(&s6, 7, 0.15f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s6, 290));
  assert(!entry_of(&ctx, 7)->p0_pending); // 修复点：使能成功即作废
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING); // 取走使能帧
  motor_control_service(&ctx, 300);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) == 0);
  assert(!entry_of(&ctx, 7)->p0_pending);
}

// 状态副本按 motor_id 升序输出（注册表顺序 = 发现序，会随上线顺序漂移）。
// 注册顺序故意乱序（5、3、7），断言 motion/status 副本均升序；capacity
// 截断保留最小 id 前缀；缺席电机（未发现）不产生条目。
static void test_state_copies_sorted_by_motor_id() {
  motor_control_context_t ctx;
  const motor_protocol_adapter_t *adapters[1] = {&g_mock_adapter};
  motor_control_init(&ctx, adapters, 1);
  motor_protocol_model_profile_t profile = g_mock_profile;
  const uint16_t nodes[3] = {5U, 3U, 7U};
  for (size_t k = 0; k < 3; ++k) {
    motor_protocol_raw_frame_t f = {};
    f.bus_index = 0;
    f.can_id = 0x600U + nodes[k];
    f.dlc = 4U;
    f.rx_tick_ms = 100;
    f.data[0] = 1U;
    motor_control_on_raw_frame(&ctx, &f);
    assert(motor_control_bind_profile_ptr(&ctx, nodes[k], &profile, 100));
  }
  motor_control_motion_state_t states[8];
  size_t n = motor_control_copy_motion_states(&ctx, states, 8, 120);
  assert(n == 3);
  assert(states[0].motor_id == 3U && states[1].motor_id == 5U &&
         states[2].motor_id == 7U);
  motor_control_status_t statuses[8];
  size_t m = motor_control_copy_statuses(&ctx, statuses, 8, 120);
  assert(m == 3);
  assert(statuses[0].motor_id == 3U && statuses[1].motor_id == 5U &&
         statuses[2].motor_id == 7U);
  n = motor_control_copy_motion_states(&ctx, states, 2, 120);
  assert(n == 2 && states[0].motor_id == 3U && states[1].motor_id == 5U);
}


static void run_all_tests() {
  test_init_starts_unauthorized_and_empty();
  test_frame_creates_discovered_then_bind_readies();
  test_registry_capacity_64_rejects_overflow();
  test_duplicate_motor_id_across_buses_conflicts();
  test_identity_conflict_same_bus_different_protocol();
  test_enter_active_requires_auth_profile_snapshot();
  test_p0_requeue_after_downstream_failure();
  test_identity_read_binds_and_unknown_stays_unbound();
  test_identity_string_match_is_exact();
  test_identity_read_failure_stays_unbound_without_retry();
  test_reconnect_reidentifies_device();
  test_identity_read_abort_is_not_a_verdict();
  test_sdo_response_alone_creates_no_entry();
  test_identity_read_ignores_heartbeat_frames();
  test_identity_read_expedited_and_wrong_index();
  test_snapshot_with_unknown_motor_rejected();
  test_snapshot_absence_soft_stops_motor();
  test_30ms_stale_snapshot_soft_stops();
  test_100ms_stale_disables_and_clears_authorization();
  test_agent_disconnect_immediate_disable();
  test_recovery_requires_fresh_snapshot_not_old_targets();
  test_snapshot_validation();
  test_vaxor_offline_and_reconnect();
  test_cerb_offline_by_heartbeat_loss();
  test_cerb_enable_settle_gate();
  test_vaxor_no_settle();
  test_gains_clamped_to_profile();
  test_vaxor_software_limits_clamp_before_pack();
  test_vaxor_mode_transaction_gates_motion();
  test_cerb_limits_sdo_transaction();
  test_cerb_set_mode_mit_immediate_ok();
  test_vendor_fault_latches_and_recovers();
  test_mock_transaction_advance_retry_busy_timeout();
  test_probe_discovery_cerb_only();
  test_event_ring_drains_and_counts_drops();
  test_collect_tx_capacity_and_remainder();
  test_motion_state_age_and_nan();
  test_unknown_frame_counted();
  test_status_copy_fields();
  test_bind_profile_rejection();
  test_settle_window_absence_soft_stops();
  test_p0_preempts_full_buffer();
  test_stale_snapshot_gates();
  test_conflict_survives_offline_reconnect();
  test_identifying_state_observed();
  test_event_carries_tick();
  test_tx_drop_counted();
  test_set_mode_gating_on_unidentified();
  test_policy_enable_disables_and_arms();
  test_fault_recovery_requires_reauth();
  test_global_reauth_skips_frozen_lifecycles();
  test_fault_recovery_no_motion_without_explicit_enable();
  test_revoke_aborts_tx_with_unconfirmed();
  test_offline_aborts_tx_with_unconfirmed();
  test_soft_stop_encode_failure_pends_retry();
  test_tx_slot_maps_entry_index();
  test_p0_tx_slot_on_soft_stop_and_disable();
  test_p0_tx_slot_on_retry_reencode();
  test_stale_pending_disable_cleared_by_enable();
  test_state_copies_sorted_by_motor_id();
}


// CERB 设备参数回读（REFRESH_PARAMS）：6 个加速 upload（:01/:02/0x6080/
// 0x2538/0x2650:01/0x2541）全通后设备影子更新，param 视图反映换算后的关节侧值。
static void test_cerb_refresh_params_transaction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

  // 绑定后默认影子：±π、额定 π rad/s、峰值 45 N·m。
  const motor_control_param_view_t *d0 = view_of(&ctx, 7);
  assert(d0 != nullptr && d0->device_side_limits);
  assert(d0->limits_valid);
  assert(fabs(d0->pos_min_rad + MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(d0->pos_max_rad - MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(d0->velocity_limit_rad_s - MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(d0->torque_limit_nm - 45.0) < 1e-4);
  assert(d0->bound);
  // 未显式设置增益：视图回退 Profile 默认（cerb_r58_101：30/1）。
  assert(fabs(d0->kp - 30.0) < 1e-4 && fabs(d0->kd - 1.0) < 1e-4);

  motor_protocol_config_request_t req = {};
  req.request_id = 41U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &req, 105));

  auto sdo_read = [](uint8_t sub, uint16_t index, uint32_t value, uint32_t tick) {
    uint8_t payload[8] = {0x43U, (uint8_t)(index & 0xFFU),
                          (uint8_t)(index >> 8), sub,
                          (uint8_t)(value & 0xFFU),
                          (uint8_t)((value >> 8U) & 0xFFU),
                          (uint8_t)((value >> 16U) & 0xFFU),
                          (uint8_t)((value >> 24U) & 0xFFU)};
    motor_protocol_raw_frame_t f = raw_frame(0, 0x587U, 8, payload, tick);
    f.is_fd = false;
    return f;
  };
  uint32_t t = 110;
  // 阶段 0：读 :01 = 16384 cnt(+90°) → π/2
  motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t r = sdo_read(1U, 0x607DU, 16384U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  // 阶段 1：读 :02 = 32768 cnt(+180°) → π
  motor_control_service(&ctx, t);
  r = sdo_read(2U, 0x607DU, 32768U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  // 阶段 2：读 0x6080 = 600 RPM → 20π/101 rad/s
  motor_control_service(&ctx, t);
  r = sdo_read(0U, 0x6080U, 600U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  // 阶段 3：读 0x2538 = 1000 mA → 1 A × (45/10.8) = 4.1667 N·m（限值系数）
  motor_control_service(&ctx, t);
  r = sdo_read(0U, 0x2538U, 1000U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  // 阶段 4/5：v3 只读安全槽（0x2650:01 保护开关、0x2541 方向），原值透传。
  motor_control_service(&ctx, t);
  r = sdo_read(1U, 0x2650U, 0U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  motor_control_service(&ctx, t);
  r = sdo_read(0U, 0x2541U, 0U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);

  assert(has_result_event(&ctx, 41U, MOTOR_PROTOCOL_CONFIG_OK));
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v->limits_valid);
  assert(fabs(v->pos_min_rad - MOTOR_PROTOCOL_PI / 2.0) < 1e-4);
  assert(fabs(v->pos_max_rad - MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(v->velocity_limit_rad_s - 2.0 * MOTOR_PROTOCOL_PI * 10.0 / 101.0) < 1e-4);
  assert(fabs(v->torque_limit_nm - 45.0 / 10.8) < 1e-4);
}

// Vaxor 无设备参数可读：REFRESH 立即回 UNSUPPORTED，不发任何帧。
static void test_vaxor_refresh_params_unsupported() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

  motor_protocol_config_request_t req = {};
  req.request_id = 42U;
  req.motor_id = 5;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &req, 105));
  assert(has_result_event(&ctx, 42U, MOTOR_PROTOCOL_CONFIG_UNSUPPORTED));
  motor_protocol_tx_frame_t frames[8];
  assert(motor_control_collect_tx(&ctx, frames, 8) == 0U);
}

// CERB 限值写事务成功后，设备影子同步为内核钳制后的实际值。
static void test_cerb_device_write_updates_shadow() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

  motor_protocol_config_request_t req = {};
  req.request_id = 43U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  req.max_velocity_rad_s = 2.0f; // < 额定 π，不被钳
  assert(motor_control_submit_config(&ctx, &req, 105));

  auto ack = [](uint32_t tick) {
    // 2.0 rad/s × 101 × 60/(2π) ≈ 1929 RPM
    uint8_t payload[8] = {0x60U, 0x80U, 0x60U, 0x00U, 0, 0, 0, 0};
    return raw_frame(0, 0x587U, 8, payload, tick);
  };
  uint32_t t = 110;
  motor_control_service(&ctx, t);
  motor_protocol_tx_frame_t sent[8];
  assert(motor_control_collect_tx(&ctx, sent, 8) == 1U);
  assert(sent[0].data[0] == 0x23U && sent[0].data[1] == 0x80U);
  motor_protocol_raw_frame_t a = ack(t + 1);
  motor_control_on_raw_frame(&ctx, &a);
  t += 10;
  motor_control_service(&ctx, t);
  // 读回核验：回同样 RPM（1929）
  const uint32_t rpm = (uint32_t)llround(2.0 * 101.0 * 60.0 / (2.0 * MOTOR_PROTOCOL_PI));
  uint8_t rd[8] = {0x43U, 0x80U, 0x60U, 0x00U,
                   (uint8_t)(rpm & 0xFFU), (uint8_t)((rpm >> 8) & 0xFFU),
                   (uint8_t)((rpm >> 16) & 0xFFU), 0};
  motor_protocol_raw_frame_t rrf = raw_frame(0, 0x587U, 8, rd, t + 1);
  motor_control_on_raw_frame(&ctx, &rrf);
  t += 10;
  motor_control_service(&ctx, t); // 末阶段：写 FLASH（0x2539，无应答）后收尾
  (void)motor_control_collect_tx(&ctx, sent, 8U);
  assert(has_result_event(&ctx, 43U, MOTOR_PROTOCOL_CONFIG_OK));
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(fabs(v->velocity_limit_rad_s -
              motor_protocol_cerb_motor_rpm_to_joint_rad_s(101.0, rpm)) < 1e-4);
}

// 未绑定 Profile 的 CERB 不受理 REFRESH（换算需要系数）→ INVALID。
static void test_refresh_rejected_before_bind() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(7, 100);
  motor_control_on_raw_frame(&ctx, &f); // 建档，未喂身份 → 未绑定
  motor_control_service(&ctx, 110);
  motor_protocol_config_request_t req = {};
  req.request_id = 44U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(!motor_control_submit_config(&ctx, &req, 120));
  assert(has_result_event(&ctx, 44U, MOTOR_PROTOCOL_CONFIG_INVALID));
}


// boot 存储参数套用：CERB 只填设备影子不发 SDO；Vaxor 落本地软限位；
// 未绑定电机保留待套用位，绑定后的下一次 apply 生效。
static void test_stored_params_apply_after_bind() {
  // CERB：记录填影子（尚未回读设备 → 来源 RECORD），不发 SDO 帧；
  // 增益钳到 Profile 上限。
  {
    motor_control_context_t ctx;
    motor_control_init(&ctx, g_adapters, 2);
    bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
    while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

    motor_control_stored_params_t rec = {};
    rec.motor_id = 7;
    rec.has_kp_kd = true;
    rec.kp = 9999.0f;   // 超 kp_max=500
    rec.kd = 1.5f;
    rec.has_position_limits = true;
    rec.pos_min_rad = -0.7f;
    rec.pos_max_rad = 0.7f;
    rec.has_velocity_limit = true;
    rec.velocity_limit_rad_s = 2.0f;
    rec.has_torque_limit = true;
    rec.torque_limit_nm = 30.0f;
    motor_control_apply_stored_params(&ctx, &rec, 1, 120);

    const motor_control_param_view_t *v = view_of(&ctx, 7);
    assert(fabs(v->kp - 500.0) < 1e-4);
    assert(fabs(v->kd - 1.5) < 1e-4);
    assert(fabs(v->pos_min_rad + 0.7) < 1e-5);
    assert(fabs(v->pos_max_rad - 0.7) < 1e-5);
    assert(fabs(v->velocity_limit_rad_s - 2.0) < 1e-5);
    assert(fabs(v->torque_limit_nm - 30.0) < 1e-5);
    // 重复套用幂等（mask 已清零），且无 SDO 帧。
    motor_control_apply_stored_params(&ctx, &rec, 1, 130);
    motor_protocol_tx_frame_t frames[8];
    assert(motor_control_collect_tx(&ctx, frames, 8) == 0U);
  }
  // Vaxor：落本地软件限位，打包前钳制。
  {
    motor_control_context_t ctx;
    motor_control_init(&ctx, g_adapters, 2);
    bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
    while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

    motor_control_stored_params_t rec = {};
    rec.motor_id = 5;
    rec.has_position_limits = true;
    rec.pos_min_rad = -0.3f;
    rec.pos_max_rad = 0.3f;
    motor_control_apply_stored_params(&ctx, &rec, 1, 120);
    const motor_control_param_view_t *v = view_of(&ctx, 5);
    assert(!v->device_side_limits && v->limits_valid == false);
    assert(fabs(v->pos_min_rad + 0.3) < 1e-5);
    assert(fabs(v->pos_max_rad - 0.3) < 1e-5);
  }
  // 先 apply（未发现/未绑定）→ 发现绑定后再次 apply 才生效。
  {
    motor_control_context_t ctx;
    motor_control_init(&ctx, g_adapters, 2);
    motor_control_stored_params_t rec = {};
    rec.motor_id = 7;
    rec.has_kp_kd = true;
    rec.kp = 12.0f;
    rec.kd = 0.5f;
    motor_control_apply_stored_params(&ctx, &rec, 1, 50); // 无表项：空转
    motor_protocol_raw_frame_t f = cerb_feedback(7, 100);
    motor_control_on_raw_frame(&ctx, &f);
    motor_control_apply_stored_params(&ctx, &rec, 1, 105); // 已发现未绑定：保留
    assert(std::isnan(view_of(&ctx, 7)->kp));
    feed_identity(&ctx, 7, "R58LITE_V2", 106);
    while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}
    motor_control_apply_stored_params(&ctx, &rec, 1, 140); // 绑定后生效
    assert(fabs(view_of(&ctx, 7)->kp - 12.0) < 1e-4);
    assert(fabs(view_of(&ctx, 7)->kd - 0.5) < 1e-4);
  }
}


// 回归：boot 存储参数只在绑定后套用一次；之后的设备回读/写核验影子
// （REFRESH/SDO 写）不得被 20ms 边界的重复 apply 覆盖。
static void test_stored_apply_does_not_override_refreshed_shadow() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  // 用 bound 版（不做设备参数回读）：本用例先验证"记录是未回读时的回落值"，
  // 再验证"回读之后记录不得覆盖设备影子"。
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

  motor_control_stored_params_t rec = {};
  rec.motor_id = 7;
  rec.has_velocity_limit = true;
  rec.velocity_limit_rad_s = 1.5f;
  rec.has_torque_limit = true;
  rec.torque_limit_nm = 20.0f;
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  // 尚未回读设备：影子取记录（来源 = RECORD），页面显示的就是记录值。
  assert(fabs(view_of(&ctx, 7)->velocity_limit_rad_s - 1.5) < 1e-4);
  assert(fabs(view_of(&ctx, 7)->torque_limit_nm - 20.0) < 1e-4);
  assert(view_of(&ctx, 7)->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_RECORD);

  // 设备回读事务更新影子（模拟内核 refresh 成功路径：直接走一遍完整事务）。
  motor_protocol_config_request_t req = {};
  req.request_id = 51U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &req, 120));
  auto sdo = [](uint8_t sub, uint16_t index, uint32_t value, uint32_t tick) {
    uint8_t p[8] = {0x43, (uint8_t)(index & 0xFFU), (uint8_t)(index >> 8), sub,
                    (uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU),
                    (uint8_t)((value >> 16) & 0xFFU), (uint8_t)((value >> 24) & 0xFFU)};
    motor_protocol_raw_frame_t f = raw_frame(0, 0x587U, 8, p, tick);
    f.is_fd = false;
    return f;
  };
  uint32_t t = 125;
  motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f1 = sdo(1, 0x607D, 32768, t + 1);
  motor_control_on_raw_frame(&ctx, &f1);
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f2 = sdo(2, 0x607D, 32768, t + 1);
  motor_control_on_raw_frame(&ctx, &f2);
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f3 = sdo(0, 0x6080, 600, t + 1);
  motor_control_on_raw_frame(&ctx, &f3);
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f4 = sdo(0, 0x2538, 1000, t + 1);
  motor_control_on_raw_frame(&ctx, &f4);
  // v3：REFRESH 还有两个只读安全槽位（保护开关/方向），位图到齐才收尾。
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f5 = sdo(1, 0x2650, 0, t + 1);
  motor_control_on_raw_frame(&ctx, &f5);
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f6 = sdo(0, 0x2541, 0, t + 1);
  motor_control_on_raw_frame(&ctx, &f6);
  assert(has_result_event(&ctx, 51U, MOTOR_PROTOCOL_CONFIG_OK));

  // 回读之后：来源转为设备，记录不得再覆盖影子（设备为准）。
  assert(view_of(&ctx, 7)->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  motor_control_apply_stored_params(&ctx, &rec, 1, t + 20);
  motor_control_apply_stored_params(&ctx, &rec, 1, t + 40);
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(fabs(v->velocity_limit_rad_s - 2.0 * MOTOR_PROTOCOL_PI * 10.0 / 101.0) < 1e-4);
  assert(fabs(v->torque_limit_nm - 45.0 / 10.8) < 1e-3);
}


// 上电比对：记录已落袋后自动回读，一致 → OK（可运动）。
static void test_device_param_boot_verify_ok() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(entry_of(&ctx, 7)->boot_record_offered);

  // 自动回读：service 即发出第一个 SDO 读请求（0x600+node，P2）。
  motor_control_service(&ctx, 120);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U);
  assert(frames[0].can_id == 0x600U + 7U);
  assert(frames[0].tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);
  assert(frames[0].data[0] == 0x40U && frames[0].data[1] == 0x7DU); // 读 0x607D

  uint32_t t = feed_device_params(&ctx, 7, kR58Raw, 121, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(entry_of(&ctx, 7)->dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(entry_of(&ctx, 7)->dev_raw[3] == kR58PeakMa);
  assert(entry_of(&ctx, 7)->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_DEV_INIT, 7));

  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(v->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  assert(v->dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(v->dev_raw[3] == kR58PeakMa);
  assert(fabs(v->torque_limit_nm - 45.0) < 0.1);

  // 自动回读每会话只发一次：再 service 不再产生 SDO 请求。
  motor_control_service(&ctx, t + 10U);
  assert(flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) == 0U);
}

// 上电比对：设备里的峰值电流与记录不一致（台架实测 13300 mA vs 记录 10800 mA）
// → 未初始化：冻结授权、拒绝使能、不出运动帧；影子仍取设备真值。
static void test_device_param_mismatch_blocks_motion() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  const uint32_t device_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
      0xFFFF8000U, 32768U, 3030U, 13300U, 0U, 0U};
  (void)feed_device_params(&ctx, 7, device_raw, 120, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(e->dev_raw[3] == 13300U);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM);
  assert(!e->policy_enable);
  // 事件必须点名"是哪一项对不上"：记录里只有峰值电流(槽 3)与设备不同 →
  // 位图 = 1<<3。操作者据此知道该写哪一项，而不是盲写一通还看不到结果。
  uint16_t state = 0U;
  uint32_t detail = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &detail));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(detail == (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX));
  // 影子 = 设备真值（设备为准），不是记录值。
  assert(fabs(e->dev_max_torque_nm - 13.3 * kR58LimitNmPerAmp) < 0.1);

  // 使能被拒（显式回告，不静默吞掉）。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 71U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  motor_control_service(&ctx, 140);
  assert(!motor_control_submit_config(&ctx, &enable, 141));
  assert(has_result_event(&ctx, 71U, MOTOR_PROTOCOL_CONFIG_INVALID));

  // 快照也不得产生任何 MIT（P1）帧。
  motor_control_command_snapshot_t snap = make_snapshot(1U, 150);
  snapshot_add(&snap, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &snap, 152));
  motor_control_service(&ctx, 154);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 写事务 = 设备 + 固件记录一起写：写核验通过后记录同步、脏位置起、比对重判。
static void test_device_param_write_syncs_record_and_recovers() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  const uint32_t bad_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
      0xFFFF8000U, 32768U, 3030U, 13300U, 0U, 0U};
  (void)feed_device_params(&ctx, 7, bad_raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(!motor_control_take_record_dirty(&ctx));

  // 用户从参数页写回 45 N·m（峰值钳制上限）→ 规格书峰值电流 10800 mA。
  motor_protocol_config_request_t trq = {};
  trq.request_id = 81U;
  trq.motor_id = 7;
  trq.kind = MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT;
  trq.max_torque_nm = 100.0f; // 超峰值 → 内核钳到 45 N·m
  motor_control_service(&ctx, 140);
  assert(motor_control_submit_config(&ctx, &trq, 141));
  (void)feed_torque_write(&ctx, 7, kR58PeakMa, 142);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(has_result_event(&ctx, 81U, MOTOR_PROTOCOL_CONFIG_OK));
  // 记录与设备同步（原值口径），并请求落盘。
  assert(e->record_raw[3] == kR58PeakMa);
  assert((e->record_raw_present & 0x08U) != 0U);
  assert(e->dev_raw[3] == kR58PeakMa);
  // SI 限值随写核验同步进设备影子（record_* 镜像字段已删，原值仍在 record_raw）。
  assert(fabs(e->dev_max_torque_nm - 45.0) < 0.1);
  assert(motor_control_take_record_dirty(&ctx));

  // 记录与设备重新一致 → 解除未初始化，回到可运动态。
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_NONE);
  motor_protocol_config_request_t enable = {};
  enable.request_id = 82U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 160));
  assert(has_result_event(&ctx, 82U, MOTOR_PROTOCOL_CONFIG_OK));
}

// v3：安全开关（槽 4 = 0x2650:01）纳入上电比对——记录说"保护开(0)"、设备第 4
// 阶段回读说"保护已被关(1)" → MISMATCH，且位图须点名 bit4（否则上位机只能盲写）。
static void test_dev_init_flags_safety_mismatch() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  // 记录：安全槽 = 0（保护开）——即 kR58Raw[4] 的出厂默认值。
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  assert(rec.device_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 0U);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 设备回读：第 4 阶段（安全开关）返回 1 = 保护被关。临时改值后立即还原，
  // 免得污染后续共用 kR58Raw 的夹具。
  const uint32_t saved = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U;
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = saved;

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  // 位图须指出是 bit4（安全开关），不是别的槽位。
  uint16_t state = 0U;
  uint32_t detail = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &detail));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(detail == (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY));
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v != nullptr);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert((v->dev_raw_present & (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY)) != 0U);
  assert(v->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U);
  // 不一致 → 不可使能（与既有 MISMATCH 用例同口径）。
  assert(!e->policy_enable);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);

  // 一致场景（记录与设备都是 0）走既有 bring_cerb_to_ready → OK 已被覆盖。
  assert(saved == 0U);
}

// 回归（P8）：限值写事务只写它自己那几个槽位，不得把只读安全槽（4=保护开关 /
// 5=方向）清零。清零后记录落盘成 0，下次上电与设备(1)比对必然不一致 → 假
// CONFIG_CONFLICT，该台在无外部工具的情况下永久不可使能。
// 基线必须**非零**：基线为 0 时清零是空操作，这条路径根本测不出来。
static void test_limit_write_preserves_readonly_safety_slots() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  // 设备侧保护被外部工具关掉(1)、方向 CCW(1)：记录与设备同为非零基线。
  const uint32_t saved_safety = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
  const uint32_t saved_dir = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U;
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] = 1U;
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(!motor_control_take_record_dirty(&ctx));

  // 写峰值扭矩（写 + 读回核验）：本次事务只写了槽 3。
  motor_protocol_config_request_t trq = {};
  trq.request_id = 91U;
  trq.motor_id = 7;
  trq.kind = MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT;
  trq.max_torque_nm = 100.0f; // 超峰值 → 钳到 45 N·m = 10800 mA
  motor_control_service(&ctx, 140);
  assert(motor_control_submit_config(&ctx, &trq, 141));
  (void)feed_torque_write(&ctx, 7, kR58PeakMa, 142);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(has_result_event(&ctx, 91U, MOTOR_PROTOCOL_CONFIG_OK));
  // 只读槽必须原值保留——设备与记录两端都不许被写事务抹掉。
  assert(e->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U);
  assert(e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U);
  assert(e->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] == 1U);
  assert(e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] == 1U);
  // 写后重判仍一致（记录被抹成 0 前，这里会走成假不一致）。
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  // 端到端复现"下次上电"：记录(现已被污染) 与设备(仍为 1) 比对必须一致。
  assert((e->record_raw_present & (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY)) != 0U);
  assert(e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] ==
         e->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY]);

  // 视图也必须看到原值（上位机/页面读的就是这份）。
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v != nullptr);
  assert(v->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U);
  assert(v->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U);

  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = saved_safety;
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] = saved_dir;
}

// 回读失败（设备不答）：结论 UNVERIFIED——初始化未完成，**同样不可使能**，
// 结论出事件 + 错误码 41。恢复：重连重读或手动回读。
static void test_device_param_unverified_blocks_motion() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 只喂心跳、不回应答：单次尝试 = 阶段 0 连续 3 次超时即定论，尝试之间退避
  // 500ms；用尽 MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS 次才定论。
  uint32_t t = run_heartbeat_only(&ctx, 7, 120U, 3000U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_read_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED);
  assert(e->dev_read_done);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(!e->policy_enable);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_DEV_INIT, 7));

  // 初始化未完成：使能被拒，快照也不出 MIT。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 91U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(!motor_control_submit_config(&ctx, &enable, t));
  assert(has_result_event(&ctx, 91U, MOTOR_PROTOCOL_CONFIG_INVALID));
  motor_control_command_snapshot_t snap = make_snapshot(1U, t + 2U);
  snapshot_add(&snap, 7, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &snap, t + 4U));
  motor_control_service(&ctx, t + 6U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 首次回读失败 → 退避重试、期间仍 PENDING；重试成功即 OK，不退化 UNVERIFIED。
static void test_device_param_read_retry_recovers() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 第一次尝试：设备不应答（只喂心跳）→ 未定论，排下次重试。
  uint32_t t = run_heartbeat_only(&ctx, 7, 120U, 500U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_read_attempts == 1U);
  assert(!e->dev_read_done);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING); // 仍不可使能
  assert(!motor_control_dev_params_allow_motion(e));

  // 退避窗内继续喂心跳保持在线（静默 100ms 会被判离线，测的就不是重试路径），
  // 到点后设备正常应答 → 第二次尝试成功。
  const uint32_t retry_at = e->dev_read_retry_tick_ms;
  t = run_heartbeat_only(&ctx, 7, t, retry_at - 5U);
  (void)feed_device_params(&ctx, 7, kR58Raw, retry_at, false);
  assert(e->dev_read_attempts == 2U);
  assert(e->dev_read_done);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(motor_control_dev_params_allow_motion(e));
}

// 上电回读未收尾（PENDING）：同样不可使能——"初始化完成前不可使能"。
// 回读收尾后立即可使能（不等重连、不需人工干预）。
static void test_device_param_pending_blocks_enable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);

  motor_protocol_config_request_t enable = {};
  enable.request_id = 95U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  motor_control_service(&ctx, 112);
  assert(!motor_control_submit_config(&ctx, &enable, 113));
  assert(has_result_event(&ctx, 95U, MOTOR_PROTOCOL_CONFIG_INVALID));

  // 设备参数回读收尾 → OK → 使能被受理。
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(motor_control_submit_config(&ctx, &enable, 140));
  assert(has_result_event(&ctx, 95U, MOTOR_PROTOCOL_CONFIG_OK));
}

// 运行中（ACTIVE）手动回读发现不一致：走**软停**（阻尼帧、保留使能），
// 不硬失能；随后锁存未初始化结论。
static void test_device_param_runtime_mismatch_soft_stops() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);

  // 跑到 ACTIVE：使能 + 整定窗 100ms 内持续喂快照与心跳。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 96U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, 150));
  for (uint32_t i = 0U; i < 8U; ++i) {
    const uint32_t base = 152U + i * 20U;
    motor_control_command_snapshot_t snap = make_snapshot(1U + i, base);
    snapshot_add(&snap, 7, 0.1f, 0.0f, 0.0f);
    assert(motor_control_apply_command_snapshot(&ctx, &snap, base + 2U));
    motor_protocol_raw_frame_t hb = cerb_feedback(7, base + 3U);
    motor_control_on_raw_frame(&ctx, &hb);
    motor_control_service(&ctx, base + 4U);
  }
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  (void)count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1); // 清空 P1 待发

  // 手动回读，设备里的峰值电流与记录不符 → 软停而不是硬失能。
  motor_protocol_config_request_t refresh = {};
  refresh.request_id = 97U;
  refresh.motor_id = 7;
  refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &refresh, 320));
  const uint32_t bad_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
      0xFFFF8000U, 32768U, 3030U, 13300U, 0U, 0U};
  bool saw_p0 = false;
  (void)feed_device_params(&ctx, 7, bad_raw, 321, true, &saw_p0);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_SOFT_STOP, 7));
  assert(!e->policy_enable && !e->motion_authorized);
  // 软停帧（P0、MIT 阻尼）已发出——不是失能帧。
  assert(saw_p0);
}

// 记录里没写过的槽位不参与比对：只比记录声明过的字段。
static void test_device_param_partial_record_compare() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  // 记录只含峰值电流一项；设备的位置/转速与记录里的值无关。
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x08U);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  const uint32_t device_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {1024U, 2048U,
                                                                 100U,
                                                                 kR58PeakMa,
                                                                 0U, 0U};
  (void)feed_device_params(&ctx, 7, device_raw, 120, false);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(e->dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(e->record_raw_present ==
         (uint8_t)(0x08U | (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY) |
                   (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)));

  // 记录里声明的峰值电流对不上 → 未初始化（其余槽位不参与）。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  bring_cerb_to_bound(&ctx2, 9, "cerb_r58_101", 100);
  motor_control_stored_params_t rec2 = make_device_record(9, kR58Raw, 0x08U);
  motor_control_apply_stored_params(&ctx2, &rec2, 1, 110);
  const uint32_t bad_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {1024U, 2048U,
                                                             100U, 13300U, 0U,
                                                             0U};
  (void)feed_device_params(&ctx2, 9, bad_raw, 120, false);
  assert(entry_of(&ctx2, 9)->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
}

// 视图同时暴露固件记录（比对基准）：本次上电未成功回读设备时，存储投影靠它
// 保住基准，不能因为"读不到设备"就把记录抹掉。
static void test_device_param_view_exposes_record() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v->dev_raw_present == 0U);        // 还没回读设备
  assert(v->record_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);  // 记录基准在视图里可见
  assert(v->record_raw[3] == kR58PeakMa);
  assert(v->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_RECORD);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);

  // 回读成功后：设备端原值就位，记录仍保留（比对依据）。
  const uint32_t device_raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
      0xFFFF8000U, 32768U, 3030U, 13300U, 0U, 0U};
  (void)feed_device_params(&ctx, 7, device_raw, 120, false);
  v = view_of(&ctx, 7);
  assert(v->dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(v->dev_raw[3] == 13300U);
  assert(v->record_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(v->record_raw[3] == kR58PeakMa);
  assert(v->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
}

// 多台 CERB 并行上电回读：每台必须各发 6 个 SDO 读请求，对象顺序固定为
// 0x607D:01（位置下限）→ 0x607D:02（位置上限）→ 0x6080（最大转速）→
// 0x2538（峰值电流）→ 0x2650:01（保护开关）→ 0x2541（方向）。多台并行时同一拍
// 各台各推进一个阶段，总线上是"逐台交错"而非"逐台跑完再下一台"——按台过滤
// 日志时每台恰好 6 条读请求。
static void test_device_param_four_reads_per_motor() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  const uint16_t ids[4] = {1U, 2U, 3U, 4U};
  for (size_t i = 0; i < 4U; ++i) {
    bring_cerb_to_bound(&ctx, ids[i], "cerb_r58_101", 100U);
  }
  motor_control_stored_params_t rec[4];
  for (size_t i = 0; i < 4U; ++i) {
    rec[i] = make_device_record(ids[i], kR58Raw, 0x0FU);
  }
  motor_control_apply_stored_params(&ctx, rec, 4U, 110U);

  // 逐拍推进：每拍喂心跳保活 → service → 取帧（**同总线每拍至多一条 SDO
  // 请求**，一问一答）→ 立刻喂该请求的应答。
  uint8_t got_count[4] = {0};
  uint8_t got_index[4][MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {{0}};
  uint32_t t = 120U;
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  for (uint8_t round = 0U; round < 48U; ++round) {
    for (size_t i = 0; i < 4U; ++i) {
      motor_protocol_raw_frame_t hb = cerb_feedback(ids[i], t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
    const size_t n = motor_control_collect_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
    size_t requests = 0U;
    for (size_t k = 0; k < n; ++k) {
      if (frames[k].data[0] != 0x40U) {
        continue; // 只看上传（读）请求
      }
      requests++;
      const uint16_t node = (uint16_t)(frames[k].can_id - 0x600U);
      assert(node >= 1U && node <= 4U);
      const uint16_t index =
          (uint16_t)(frames[k].data[1] | ((uint16_t)frames[k].data[2] << 8U));
      const uint8_t sub = frames[k].data[3];
      size_t slot = 5U; // 0x2541 方向
      if (index == 0x607DU) {
        slot = (sub == 1U) ? 0U : 1U;
      } else if (index == 0x6080U) {
        slot = 2U;
      } else if (index == 0x2538U) {
        slot = 3U;
      } else if (index == 0x2650U) {
        slot = 4U;
      }
      assert(slot == got_count[node - 1U]); // 每台阶段顺序固定
      got_index[node - 1U][got_count[node - 1U]] = (uint8_t)index;
      got_count[node - 1U]++;
      motor_protocol_raw_frame_t ack =
          sdo_read_ack(node, (uint8_t)slot, kR58Raw[slot], t + 1U);
      motor_control_on_raw_frame(&ctx, &ack);
    }
    // 一问一答：同一总线上不允许出现两条同时在飞的 SDO 请求。
    assert(requests <= 1U);
    t += 2U;
  }
  for (size_t i = 0; i < 4U; ++i) {
    assert(got_count[i] == MOTOR_PROTOCOL_PARAM_SLOT_COUNT); // 每台恰好 6 条读
    assert(entry_of(&ctx, ids[i])->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
    assert(entry_of(&ctx, ids[i])->dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  }
}

// 重连：本轮设备参数结论作废重读，但 Flash 记录（比对基准）必须保留——
// 记录只在 boot 首次落袋时捕获，重连时清掉就再也补不回来，之后的不一致
// 会被当成"本机无记录"静默放过。
static void test_device_param_reconnect_keeps_record() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);

  // 心跳丢失 ≥100ms → OFFLINE；再收到反馈 → 重连（走重识别）。
  uint32_t t = 400U; // 末帧约在 136ms，留足 100ms 心跳龄
  motor_control_service(&ctx, t);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);
  motor_protocol_raw_frame_t hb = cerb_feedback(7, t + 1U);
  motor_control_on_raw_frame(&ctx, &hb);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING); // 结论作废，待重读
  assert(!e->dev_read_done);
  assert(e->record_valid);                    // 记录保留
  assert(e->record_raw_present == MOTOR_CONTROL_DEV_RAW_ALL);
  assert(e->record_raw[3] == kR58PeakMa);
  assert(e->dev_raw_present == 0U);           // 旧影子作废
}

// 外部写入不得超过**固定硬件参数表**（规格书）：请求超限时钳到型号表峰值，
// 且落到设备上的原值正好等于规格书的峰值电流 / 额定转速 / 位置量化范围。
// 覆盖三列：扭矩(=电流)、速度、位置。
static void test_device_write_clamped_to_hardware_table() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];

  // ① 扭矩请求 100 N·m（远超峰值 45）→ 钳到峰值，落到设备 = 峰值电流 10800 mA。
  motor_protocol_config_request_t trq = {};
  trq.request_id = 201U;
  trq.motor_id = 7;
  trq.kind = MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT;
  trq.max_torque_nm = 100.0f;
  assert(motor_control_submit_config(&ctx, &trq, 200));
  motor_control_service(&ctx, 201);
  const size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U && frames[0].can_id == 0x600U + 7U);
  assert(frames[0].data[0] == 0x23U && frames[0].data[1] == 0x38U); // 写 0x2538
  const uint32_t ma = (uint32_t)frames[0].data[4] |
                      ((uint32_t)frames[0].data[5] << 8U) |
                      ((uint32_t)frames[0].data[6] << 16U) |
                      ((uint32_t)frames[0].data[7] << 24U);
  assert(ma == kR58PeakMa); // 10800 mA = 规格书峰值电流
  (void)feed_torque_write(&ctx, 7, kR58PeakMa, 202);
  assert(fabs(entry_of(&ctx, 7)->dev_max_torque_nm - 45.0f) < 0.01f);
  assert(entry_of(&ctx, 7)->record_raw[3] == kR58PeakMa);

  // ② 速度请求 10 rad/s（远超额定 π）→ 钳到额定，落到设备 = 3030 RPM。
  motor_protocol_config_request_t vel = {};
  vel.request_id = 202U;
  vel.motor_id = 7;
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = 10.0f;
  assert(motor_control_submit_config(&ctx, &vel, 300));
  motor_control_service(&ctx, 301);
  size_t m = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(m == 1U && frames[0].data[0] == 0x23U && frames[0].data[1] == 0x80U); // 写 0x6080
  const uint32_t rpm = (uint32_t)frames[0].data[4] |
                       ((uint32_t)frames[0].data[5] << 8U) |
                       ((uint32_t)frames[0].data[6] << 16U) |
                       ((uint32_t)frames[0].data[7] << 24U);
  assert(rpm == 3030U); // 额定 30 RPM 输出 → 电机端 3030 RPM
  motor_control_service(&ctx, 302);
  motor_protocol_raw_frame_t ack = sdo_write_ack(7, 0x6080U, 0U, 303U);
  motor_control_on_raw_frame(&ctx, &ack);
  motor_control_service(&ctx, 304);
  (void)flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING); // 取走本阶段读回请求
  motor_protocol_raw_frame_t rd = sdo_read_ack(7, 2U, 3030U, 305U);
  motor_control_on_raw_frame(&ctx, &rd);
  assert(fabs(entry_of(&ctx, 7)->dev_max_velocity_rad_s - MOTOR_PROTOCOL_PI) < 1e-3f);
  // 末阶段：写 FLASH（0x2539，码 1）落盘。无应答，服务一拍即发出并收尾——
  // 不落盘的话驱动器一断电就回出厂值，下次上电比对必然判"不一致"。
  motor_control_service(&ctx, 306);
  m = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(m == 1U && frames[0].can_id == 0x600U + 7U);
  assert(frames[0].data[0] == 0x23U && frames[0].data[1] == 0x39U &&
         frames[0].data[2] == 0x25U && frames[0].data[4] == 0x01U);
  assert(!entry_of(&ctx, 7)->tx_active);

  // ③ 位置请求 ±300°（超 ±180° 量化范围）→ 钳到 ±π，落到设备 = ±32768 cnt。
  motor_protocol_config_request_t pos = {};
  pos.request_id = 203U;
  pos.motor_id = 7;
  pos.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  pos.position_limits.min_position_rad = -300.0f * (float)MOTOR_PROTOCOL_PI / 180.0f;
  pos.position_limits.max_position_rad = 300.0f * (float)MOTOR_PROTOCOL_PI / 180.0f;
  assert(motor_control_submit_config(&ctx, &pos, 400));
  motor_control_service(&ctx, 401);
  m = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(m == 1U && frames[0].data[0] == 0x23U && frames[0].data[3] == 0x01U);
  const int32_t lo_cnt = (int32_t)((uint32_t)frames[0].data[4] |
                                   ((uint32_t)frames[0].data[5] << 8U) |
                                   ((uint32_t)frames[0].data[6] << 16U) |
                                   ((uint32_t)frames[0].data[7] << 24U));
  // 钳到 ±π，再按字典范围钳 cnt：下限是 -32767（字典下限），不是 -32768。
  assert(lo_cnt == -32767);
}

// 设零点（字典 0x2531 写 1）：失能态受理，单阶段写 + 应答即 OK；不更新记录，
// 也不进设备参数比对（零点不是本机记录的参数）。
static void test_cerb_set_zero_transaction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  const uint32_t raw_before = entry_of(&ctx, 7)->dev_raw[3];
  const bool record_valid_before = entry_of(&ctx, 7)->record_valid;

  motor_protocol_config_request_t z = {};
  z.request_id = 301U;
  z.motor_id = 7;
  z.kind = MOTOR_PROTOCOL_CONFIG_SET_ZERO;
  assert(motor_control_submit_config(&ctx, &z, 110));
  motor_control_service(&ctx, 111);
  const size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U && frames[0].can_id == 0x600U + 7U);
  assert(frames[0].data[0] == 0x23U && frames[0].data[1] == 0x31U &&
         frames[0].data[2] == 0x25U && frames[0].data[4] == 0x01U);

  // SDO 写应答（0x60，索引 0x2531）→ 事务 OK。
  motor_protocol_raw_frame_t ack = sdo_write_ack(7, 0x2531U, 0U, 112U);
  motor_control_on_raw_frame(&ctx, &ack);
  assert(has_result_event(&ctx, 301U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(!entry_of(&ctx, 7)->tx_active);
  // 记录与设备端原值不受影响（零点不是本机记录的参数）。
  assert(entry_of(&ctx, 7)->dev_raw[3] == raw_before);
  assert(entry_of(&ctx, 7)->record_valid == record_valid_before);

  // 未绑定/无表项：拒绝。
  motor_protocol_config_request_t unknown = z;
  unknown.request_id = 302U;
  unknown.motor_id = 99U;
  assert(!motor_control_submit_config(&ctx, &unknown, 120));
}

// 非设备侧型号（VAXOR）与未回读：不参与比对，恒 NA。
static void test_device_param_na_for_software_limit_models() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_stored_params_t rec = {};
  rec.motor_id = 5;
  rec.has_kp_kd = true;
  rec.kp = 1.0f;
  rec.kd = 0.5f;
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  motor_control_service(&ctx, 120);
  assert(entry_of(&ctx, 5)->dev_init == MOTOR_CONTROL_DEV_INIT_NA);
  assert(entry_of(&ctx, 5)->dev_read_done == false); // 从不发起设备回读
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  assert(flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING) == 0U);
}


// copy_topology：按 motor_id 升序、含型号名/能力位/在线/生命周期；容量上界生效。
static void test_copy_topology() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 9, "vaxor_2550", 300);
  bring_cerb_to_bound(&ctx, 2, "cerb_r48_101", 100);
  bring_vaxor_to_ready(&ctx, 7, "vaxor_2550", 300);

  motor_control_topology_entry_t out[4] = {};
  assert(motor_control_copy_topology(&ctx, out, 4) == 3U);
  assert(out[0].motor_id == 2U && out[1].motor_id == 7U && out[2].motor_id == 9U);
  // 字段
  assert(out[0].protocol == &motor_protocol_cerb_adapter);
  assert(strcmp(out[0].model_name, "cerb_r48_101") == 0);
  assert((out[0].capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U);
  assert(out[1].protocol == &motor_protocol_vaxor_adapter);
  assert(out[1].model_name != nullptr);
  // 容量上界
  assert(motor_control_copy_topology(&ctx, out, 2) == 2U);
  // NULL 防护
  assert(motor_control_copy_topology(&ctx, nullptr, 4) == 0U);
}

// ---------------------------------------------------------------------------
// 多点轨迹（MotorTrajectory）：归属仲裁、段内插值、通道级 freshness
// ---------------------------------------------------------------------------

static motor_control_trajectory_t make_trajectory(uint32_t sequence,
                                                  uint32_t tick) {
  motor_control_trajectory_t t = {};
  t.sequence = sequence;
  t.received_tick_ms = tick;
  return t;
}

static void traj_add_motor(motor_control_trajectory_t *t, uint16_t motor_id) {
  t->target_ids[t->target_count] = motor_id;
  t->target_count++;
}

static void traj_add_point(motor_control_trajectory_t *t, uint32_t time_ms,
                           uint16_t motor_id, float pos, float effort) {
  t->points[t->point_count].time_from_start_ms = time_ms;
  t->points[t->point_count].motor_id = motor_id;
  t->points[t->point_count].position_rad = pos;
  t->points[t->point_count].velocity_rad_s = 0.0f;
  t->points[t->point_count].effort_nm = effort;
  t->point_count++;
}

// 两台 vaxor 经完整快照进 ACTIVE（轨迹用例的公共起点）。最后一份快照的
// received_tick_ms = tick + 20，是后续 freshness 级联的计时基准。
static void bring_two_active(motor_control_context_t *ctx, uint16_t first,
                             uint16_t second, uint32_t tick) {
  bring_vaxor_to_ready(ctx, first, "vaxor_2550", tick);
  bring_vaxor_to_ready(ctx, second, "vaxor_2550", tick + 10U);
  motor_control_command_snapshot_t s = make_snapshot(1U, tick + 20U);
  snapshot_add(&s, first, 0.0f, 0.0f, 0.0f);
  snapshot_add(&s, second, 0.0f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(ctx, &s, tick + 21U));
  assert(entry_of(ctx, first)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(entry_of(ctx, second)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// vaxor 自主心跳（离线阈值 100ms，跳时间时必须续上，否则测的是离线而不是
// 快照级联）。
static void feed_vaxor_heartbeat(motor_control_context_t *ctx, uint16_t first,
                                 uint16_t second, uint32_t tick) {
  motor_protocol_raw_frame_t f = vaxor_reply(first, 0.0, 0.0, 0.0, tick);
  motor_control_on_raw_frame(ctx, &f);
  f = vaxor_reply(second, 0.0, 0.0, 0.0, tick);
  motor_control_on_raw_frame(ctx, &f);
}

// 轨迹在播期间是所属电机的唯一命令权威：快照对它既不下发也不软停。
static void test_trajectory_owns_its_motors_against_snapshot() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&t, 1000U, 5, 0.5f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));
  motor_control_trajectory_step(&ctx, 200U);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(entry_of(&ctx, 5)->target_position_rad == 0.1f);

  // 快照只含 7：5 缺席也不得按 §13.1 软停（它归轨迹管）。
  motor_control_command_snapshot_t s = make_snapshot(2U, 205U);
  snapshot_add(&s, 7, 0.3f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 210U));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(entry_of(&ctx, 5)->has_target);
  assert(entry_of(&ctx, 5)->target_position_rad == 0.1f); // 目标仍来自轨迹
  assert(entry_of(&ctx, 7)->target_position_rad == 0.3f); // 7 照常跟快照
}

// 反向：轨迹集合外的电机仍受 §13.1「缺席即软停」约束。
static void test_snapshot_absence_soft_stops_only_unowned_motors() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));

  motor_control_command_snapshot_t empty = make_snapshot(2U, 205U);
  assert(motor_control_apply_command_snapshot(&ctx, &empty, 210U));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
}

// 段内线性插值 / 段斜率速度 / 末点钳制不外推 / 前馈段内保持。
static void test_trajectory_interpolates_and_clamps_at_end() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);
  traj_add_point(&t, 1000U, 5, 1.0f, 0.25f);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  motor_control_trajectory_step(&ctx, 300U); // elapsed 0 → 首点
  assert(entry_of(&ctx, 5)->target_position_rad == 0.0f);

  motor_control_trajectory_step(&ctx, 800U); // elapsed 500 → 中点
  assert(std::fabs(entry_of(&ctx, 5)->target_position_rad - 0.5f) < 1e-6f);
  // 速度取段斜率（1.0 rad / 1.0 s），不是路点里那个（这里填 0）的字段。
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s - 1.0f) < 1e-6f);
  assert(entry_of(&ctx, 5)->target_effort_nm == 0.0f); // 段内保持左端点

  motor_control_trajectory_step(&ctx, 9000U); // 越过末点 → 钳末点
  assert(entry_of(&ctx, 5)->target_position_rad == 1.0f);
  assert(entry_of(&ctx, 5)->target_velocity_rad_s == 0.0f);
  assert(entry_of(&ctx, 5)->target_effort_nm == 0.25f);
}

// 取消（空轨迹）与被替换：移出集合的电机立即软停释放。
static void test_trajectory_cancel_and_replacement_release_motors() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_motor(&t, 7);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&t, 0U, 7, 0.2f, 0.0f);
  traj_add_point(&t, 500U, 5, 0.3f, 0.0f);
  traj_add_point(&t, 500U, 7, 0.4f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));
  motor_control_trajectory_step(&ctx, 200U);
  assert(entry_of(&ctx, 5)->target_position_rad == 0.1f);
  assert(entry_of(&ctx, 7)->target_position_rad == 0.2f);

  // 替换成只声明 7 的轨迹 → 5 被移出集合，立即软停释放。
  motor_control_trajectory_t t2 = make_trajectory(2U, 300U);
  traj_add_motor(&t2, 7);
  traj_add_point(&t2, 0U, 7, 0.9f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t2, 300U));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(!entry_of(&ctx, 5)->has_target);
  motor_control_trajectory_step(&ctx, 300U);
  assert(entry_of(&ctx, 7)->target_position_rad == 0.9f);

  // 取消（空轨迹）→ 7 也软停释放。
  motor_control_trajectory_t cancel = make_trajectory(3U, 400U);
  assert(motor_control_apply_trajectory(&ctx, &cancel, 400U));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(!entry_of(&ctx, 7)->has_target);
}

// 快照流断但轨迹在播：30/100ms 级联只停机快照通道的电机，全局授权不动。
static void test_trajectory_survives_snapshot_timeout() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&t, 2000U, 5, 0.2f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));
  motor_control_trajectory_step(&ctx, 200U);

  const uint32_t last_snapshot = 120U; // bring_two_active 那份快照的接收 tick
  for (uint32_t dt = 10U; dt <= 150U; dt += 10U) {
    feed_vaxor_heartbeat(&ctx, 5, 7, last_snapshot + dt);
    motor_control_service(&ctx, last_snapshot + dt);
  }

  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(ctx.authorized); // 授权由会话电平派生，与快照流是否活跃无关
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST, 0));
}

// 回归：没有活动轨迹时，100ms 级联保持原有全局撤权语义。
static void test_snapshot_timeout_without_trajectory_revokes_globally() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  const uint32_t last_snapshot = 120U;
  for (uint32_t dt = 10U; dt <= 150U; dt += 10U) {
    feed_vaxor_heartbeat(&ctx, 5, 7, last_snapshot + dt);
    motor_control_service(&ctx, last_snapshot + dt);
  }

  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(!ctx.authorized);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST, 0));
}

// 整条拒绝：结构非法与未注册电机都不得部分采纳。
static void test_trajectory_rejects_malformed_and_unknown_motor() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  // 路点 motor_id 不在声明集合内
  motor_control_trajectory_t bad = make_trajectory(1U, 200U);
  traj_add_motor(&bad, 5);
  traj_add_point(&bad, 0U, 7, 0.1f, 0.0f);
  assert(!motor_control_apply_trajectory(&ctx, &bad, 200U));

  // 未注册电机
  motor_control_trajectory_t unknown = make_trajectory(2U, 210U);
  traj_add_motor(&unknown, 99);
  traj_add_point(&unknown, 0U, 99, 0.1f, 0.0f);
  assert(!motor_control_apply_trajectory(&ctx, &unknown, 210U));

  // 时间轴回退
  motor_control_trajectory_t back = make_trajectory(3U, 220U);
  traj_add_motor(&back, 5);
  traj_add_point(&back, 100U, 5, 0.1f, 0.0f);
  traj_add_point(&back, 50U, 5, 0.2f, 0.0f);
  assert(!motor_control_apply_trajectory(&ctx, &back, 220U));

  // 声明了归属却没有任何路点
  motor_control_trajectory_t empty_motor = make_trajectory(4U, 230U);
  traj_add_motor(&empty_motor, 5);
  traj_add_motor(&empty_motor, 7);
  traj_add_point(&empty_motor, 0U, 5, 0.1f, 0.0f);
  assert(!motor_control_apply_trajectory(&ctx, &empty_motor, 230U));

  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED, 0));
  // 被拒的轨迹没有接管任何电机：两台仍在快照通道上。
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(ctx.trajectory_active == false);
}

// 故障清除的设备侧动作：CERB 发 MIT 控制字 bit5（清错），Vaxor 无手段。
static void test_clear_fault_protocol_capability() {
  size_t idx = 0;
  assert(find_profile("cerb_r58_101", &idx));
  const motor_protocol_model_profile_t *cerb = motor_control_profile_at(idx);
  const motor_protocol_adapter_t *adapter = cerb->protocol;

  motor_protocol_config_request_t request = {};
  request.motor_id = 1U;
  request.kind = MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT;
  assert(adapter->config_stage_count(cerb, &request) == 1U);

  motor_protocol_config_stage_info_t info = {};
  assert(adapter->config_stage_info(0U, cerb, &request, &info));
  assert(!info.waits_response); // 设备不对清错帧应答：发完即推进
  assert(!info.param_raw_valid);

  motor_protocol_tx_frame_t frame = {};
  assert(adapter->config_encode_stage(0U, cerb, 3U, 0U, &request, &frame));
  assert(frame.size == 12U); // CAN FD 控制帧，非 SDO 管理帧
  // bit5 = 清错；bit7 = 0（不请求使能——故障态设备的使能意图无意义，
  // 而且会把「清错」变成隐式重新使能）；bit4:1 = 0x6（MIT）。
  assert((frame.data[0] & 0x20U) != 0U);
  assert((frame.data[0] & 0x80U) == 0U);
  assert((frame.data[0] & 0x1EU) == 0x0CU);

  // Vaxor 的控制帧没有清错位（Data0=运行/停止、Data1=设零点、Data2=模式），
  // 故 0 阶段——内核统一回 UNSUPPORTED，不猜厂家字节。
  assert(find_profile("vaxor_2550", &idx));
  assert(motor_control_profile_at(idx)->protocol->config_stage_count(
             motor_control_profile_at(idx), &request) == 0U);
}

// 内核故障清除入口：无清错手段的协议回 UNSUPPORTED（不静默吞掉）。
static void test_clear_fault_kernel_unsupported_without_protocol_support() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_protocol_config_request_t request = {};
  request.request_id = 0x1001U;
  request.motor_id = 5U;
  request.kind = MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT;
  assert(motor_control_submit_config(&ctx, &request, 110U));
  assert(has_result_event(&ctx, 0x1001U, MOTOR_PROTOCOL_CONFIG_UNSUPPORTED));
}

// ---------------------------------------------------------------------------
// 逐电机策略（SET_POLICY）：原子落地与代数门
// ---------------------------------------------------------------------------

static motor_protocol_config_request_t make_policy(uint32_t request_id,
                                                   uint16_t motor_id,
                                                   uint32_t generation,
                                                   bool enable, float kp,
                                                   float kd) {
  motor_protocol_config_request_t r = {};
  r.request_id = request_id;
  r.motor_id = motor_id;
  r.kind = MOTOR_PROTOCOL_CONFIG_SET_POLICY;
  r.policy_generation = generation;
  r.enable = enable;
  r.kp = kp;
  r.kd = kd;
  return r;
}

// 增益与使能一次落地；越限增益被钳到型号上限而不是拒绝。
static void test_policy_arms_atomically_with_clamped_gains() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  const motor_protocol_model_profile_t *profile = entry_of(&ctx, 5)->profile;
  assert(profile != nullptr && profile->kp_max > 0.0f);
  const float kd_sent = profile->kd_max * 0.5f;

  // 先显式撤销：建档默认 policy_enable=true，不先置 false 的话"武装"断言
  // 恒真，测不到任何东西。
  motor_protocol_config_request_t pre =
      make_policy(0x2000U, 5U, 1U, false, 0.0f, 0.0f);
  assert(motor_control_submit_config(&ctx, &pre, 105U));
  assert(!entry_of(&ctx, 5)->policy_enable);

  motor_protocol_config_request_t req =
      make_policy(0x2001U, 5U, 2U, true, profile->kp_max * 2.0f, kd_sent);
  assert(motor_control_submit_config(&ctx, &req, 110U));
  assert(has_result_event(&ctx, 0x2001U, MOTOR_PROTOCOL_CONFIG_OK));
  // 三项在同一笔里落地：不存在「已使能但 Kd 还是旧的」中间态。
  assert(entry_of(&ctx, 5)->policy_enable);
  assert(entry_of(&ctx, 5)->motion_authorized);
  assert(entry_of(&ctx, 5)->has_kp_kd);
  assert(entry_of(&ctx, 5)->kp == profile->kp_max); // 钳到上限
  assert(entry_of(&ctx, 5)->kd == kd_sent);         // 未超限则原值
}

// 「未武装 → 武装」需要代数前进：重复同一条策略不得把电机自动复权。
static void test_policy_generation_blocks_replay_rearm() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_protocol_config_request_t arm =
      make_policy(0x3001U, 5U, 1U, true, 10.0f, 1.0f);
  assert(motor_control_submit_config(&ctx, &arm, 110U));
  assert(entry_of(&ctx, 5)->policy_enable);

  motor_protocol_config_request_t disarm =
      make_policy(0x3002U, 5U, 1U, false, 10.0f, 1.0f);
  assert(motor_control_submit_config(&ctx, &disarm, 120U));
  assert(!entry_of(&ctx, 5)->policy_enable);

  // 同代数重新武装 = 「周期重发」的特征 → 拒绝，且不动武装状态。
  motor_protocol_config_request_t replay =
      make_policy(0x3003U, 5U, 1U, true, 10.0f, 1.0f);
  assert(!motor_control_submit_config(&ctx, &replay, 130U));
  assert(has_result_event(&ctx, 0x3003U, MOTOR_PROTOCOL_CONFIG_INVALID));
  assert(!entry_of(&ctx, 5)->policy_enable);
  assert(entry_of(&ctx, 5)->last_failure_reason ==
         MOTOR_CONTROL_FAULT_STALE_GENERATION);

  // 新意图（代数前进）→ 受理。
  motor_protocol_config_request_t fresh =
      make_policy(0x3004U, 5U, 2U, true, 10.0f, 1.0f);
  assert(motor_control_submit_config(&ctx, &fresh, 140U));
  assert(entry_of(&ctx, 5)->policy_enable);

  // 已武装时重复同代数是幂等的（不是「未武装 → 武装」的迁移），增益照常更新。
  // 取值从 profile 推：写死字面量会被钳到型号上限，断言就测不到「更新生效」。
  const float kp_new = entry_of(&ctx, 5)->profile->kp_max * 0.5f;
  motor_protocol_config_request_t again =
      make_policy(0x3005U, 5U, 2U, true, kp_new, 1.0f);
  assert(motor_control_submit_config(&ctx, &again, 150U));
  assert(entry_of(&ctx, 5)->kp == kp_new);
}

// 非法载荷与未绑定设备整笔拒绝，且不改变武装状态。
static void test_policy_rejects_invalid_and_unbound() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  // 先落一个已知增益，这样"拒绝有没有改状态"才可观测（policy_enable 默认
  // 就是 true，拿它当断言是恒真的）。
  const float kp_known = entry_of(&ctx, 5)->profile->kp_max * 0.5f;
  motor_protocol_config_request_t ok =
      make_policy(0x4000U, 5U, 1U, true, kp_known, 1.0f);
  assert(motor_control_submit_config(&ctx, &ok, 105U));
  assert(entry_of(&ctx, 5)->kp == kp_known);

  // 负增益：整笔拒绝、增益保持原值（不是"钳到 0"），代数也不前进。
  motor_protocol_config_request_t neg =
      make_policy(0x4001U, 5U, 2U, true, -1.0f, 1.0f);
  assert(!motor_control_submit_config(&ctx, &neg, 110U));
  assert(entry_of(&ctx, 5)->kp == kp_known);
  assert(entry_of(&ctx, 5)->policy_generation == 1U);

  // 未发现/未绑定的 motor_id：内核直接计数拒绝，不建表项。
  motor_protocol_config_request_t unbound =
      make_policy(0x4002U, 77U, 1U, true, 10.0f, 1.0f);
  assert(!motor_control_submit_config(&ctx, &unbound, 120U));
  assert(!has_result_event(&ctx, 0x4002U, MOTOR_PROTOCOL_CONFIG_OK));
}

// target_source：按归属派生（与仲裁同一判据，不发散出第二份状态）。
static void test_motion_state_reports_target_source() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_motion_state_t states[MOTOR_CONTROL_MAX_MOTORS];
  const auto source_of = [&](uint16_t id) {
    const size_t n = motor_control_copy_motion_states(
        &ctx, states, MOTOR_CONTROL_MAX_MOTORS, 200U);
    for (size_t i = 0; i < n; ++i) {
      if (states[i].motor_id == id) {
        return states[i].target_source;
      }
    }
    return (uint8_t)0xFFU;
  };

  // 两台都在快照通道上。
  assert(source_of(5U) == MOTOR_CONTROL_TARGET_SOURCE_SNAPSHOT);
  assert(source_of(7U) == MOTOR_CONTROL_TARGET_SOURCE_SNAPSHOT);

  // 轨迹只声明 5：5 变成轨迹源，7 不受影响。
  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));
  motor_control_trajectory_step(&ctx, 200U);
  assert(source_of(5U) == MOTOR_CONTROL_TARGET_SOURCE_TRAJECTORY);
  assert(source_of(7U) == MOTOR_CONTROL_TARGET_SOURCE_SNAPSHOT);

  // 取消 → 5 被软停释放，回到「无命令源」。
  motor_control_trajectory_t cancel = make_trajectory(2U, 300U);
  assert(motor_control_apply_trajectory(&ctx, &cancel, 300U));
  assert(source_of(5U) == MOTOR_CONTROL_TARGET_SOURCE_NONE);
}

// 事件序号单调递增：消费者据此判丢帧（累计丢弃数另见诊断话题）。
static void test_event_seq_is_monotonic() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_control_command_snapshot_t bad = make_snapshot(1U, 100U);
  bad.target_count = (uint8_t)(MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS + 1U);
  for (uint32_t i = 0; i < 5U; ++i) {
    assert(!motor_control_apply_command_snapshot(&ctx, &bad, 100U + i));
  }

  motor_control_event_t events[MOTOR_CONTROL_MAX_EVENTS];
  size_t total = 0;
  uint32_t previous = 0;
  bool first = true;
  size_t n;
  while ((n = motor_control_drain_events(&ctx, events,
                                         MOTOR_CONTROL_MAX_EVENTS)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (!first) {
        assert(events[i].seq == previous + 1U); // 无跳号 = 无丢弃
      }
      previous = events[i].seq;
      first = false;
      total++;
    }
  }
  assert(total == 5U);
}

// 诊断快照：内核计数器一次性可读（此前只存在于芯片里）。
static void test_diagnostics_counters_are_exposed() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_control_diagnostics_t d = {};
  motor_control_copy_diagnostics(&ctx, &d);
  assert(d.rx_frame_count == 0U && d.snapshot_rejected == 0U);

  motor_control_command_snapshot_t bad = make_snapshot(1U, 100U);
  bad.target_count = (uint8_t)(MOTOR_CONTROL_MAX_SNAPSHOT_TARGETS + 1U);
  assert(!motor_control_apply_command_snapshot(&ctx, &bad, 100U));

  motor_control_copy_diagnostics(&ctx, &d);
  assert(d.snapshot_rejected == 1U);  // 拒绝可见
  assert(d.event_dropped == 0U);      // 环未满
  assert(d.policy_accepted == 0U && d.trajectory_accepted == 0U);

  // NULL 防护：不得解引用空指针。
  motor_control_copy_diagnostics(&ctx, nullptr);
  motor_control_copy_diagnostics(nullptr, &d);
}

// 轨迹状态：id 回带、进度、播完标志（播完 ≠ 释放归属）。
static void test_trajectory_state_reports_progress() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_state_t st = {};
  motor_control_copy_trajectory_state(&ctx, &st, 200U);
  assert(!st.active && st.target_count == 0U);

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  t.trajectory_id = 0xABCDU;
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&t, 1000U, 5, 0.2f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));

  motor_control_copy_trajectory_state(&ctx, &st, 200U);
  assert(st.active && !st.completed);
  assert(st.trajectory_id == 0xABCDU); // 原样回带
  assert(st.target_count == 1U && st.target_ids[0] == 5U);
  assert(st.total_ms == 1000U && st.elapsed_ms == 0U);

  motor_control_copy_trajectory_state(&ctx, &st, 600U);
  assert(st.elapsed_ms == 400U && !st.completed);

  motor_control_copy_trajectory_state(&ctx, &st, 1500U);
  assert(st.completed && st.active); // 播完但归属仍在：电机在末点保持

  motor_control_trajectory_t cancel = make_trajectory(2U, 1600U);
  assert(motor_control_apply_trajectory(&ctx, &cancel, 1600U));
  motor_control_copy_trajectory_state(&ctx, &st, 1600U);
  assert(!st.active && !st.completed);
}

// 拒绝事件带 trajectory_id 与首个非法路点下标：并发多条轨迹时才分得清是谁、改哪行。
static void test_trajectory_reject_event_carries_id_and_index() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t bad = make_trajectory(1U, 200U);
  bad.trajectory_id = 0x1234U;
  traj_add_motor(&bad, 5);
  traj_add_point(&bad, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&bad, 0U, 5, 0.2f, 0.0f); // 同一电机同一时刻 → 下标 1 非法
  assert(!motor_control_apply_trajectory(&ctx, &bad, 200U));

  motor_control_event_t events[MOTOR_CONTROL_MAX_EVENTS];
  bool found = false;
  size_t n;
  while ((n = motor_control_drain_events(&ctx, events,
                                         MOTOR_CONTROL_MAX_EVENTS)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED) {
        assert(events[i].request_id == 0x1234U);
        assert(events[i].reason == 1U);
        found = true;
      }
    }
  }
  assert(found);

  // 与路点无关的拒绝（声明集合重复）→ 下标占位 0xFFFF。
  motor_control_trajectory_t dup = make_trajectory(2U, 300U);
  dup.trajectory_id = 0x5678U;
  traj_add_motor(&dup, 5);
  traj_add_motor(&dup, 5);
  traj_add_point(&dup, 0U, 5, 0.1f, 0.0f);
  assert(!motor_control_apply_trajectory(&ctx, &dup, 300U));
  bool found2 = false;
  while ((n = motor_control_drain_events(&ctx, events,
                                         MOTOR_CONTROL_MAX_EVENTS)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED &&
          events[i].request_id == 0x5678U) {
        assert(events[i].reason == MOTOR_CONTROL_TRAJECTORY_BAD_INDEX_NONE);
        found2 = true;
      }
    }
  }
  assert(found2);
}

int main() {
  bind_real_adapters();
  test_device_param_boot_verify_ok();
  test_device_param_mismatch_blocks_motion();
  test_dev_init_flags_safety_mismatch();
  test_device_param_write_syncs_record_and_recovers();
  test_limit_write_preserves_readonly_safety_slots();
  test_device_param_read_retry_recovers();
  test_device_param_unverified_blocks_motion();
  test_device_param_pending_blocks_enable();
  test_device_param_runtime_mismatch_soft_stops();
  test_device_param_partial_record_compare();
  test_device_param_view_exposes_record();
  test_device_write_clamped_to_hardware_table();
  test_cerb_set_zero_transaction();
  test_device_param_four_reads_per_motor();
  test_device_param_reconnect_keeps_record();
  test_device_param_na_for_software_limit_models();
  test_stored_apply_does_not_override_refreshed_shadow();
  test_stored_params_apply_after_bind();
  test_cerb_refresh_params_transaction();
  test_vaxor_refresh_params_unsupported();
  test_cerb_device_write_updates_shadow();
  test_refresh_rejected_before_bind();
  test_copy_topology();
  test_trajectory_owns_its_motors_against_snapshot();
  test_snapshot_absence_soft_stops_only_unowned_motors();
  test_trajectory_interpolates_and_clamps_at_end();
  test_trajectory_cancel_and_replacement_release_motors();
  test_trajectory_survives_snapshot_timeout();
  test_snapshot_timeout_without_trajectory_revokes_globally();
  test_trajectory_rejects_malformed_and_unknown_motor();
  test_clear_fault_protocol_capability();
  test_clear_fault_kernel_unsupported_without_protocol_support();
  test_policy_arms_atomically_with_clamped_gains();
  test_policy_generation_blocks_replay_rearm();
  test_policy_rejects_invalid_and_unbound();
  test_motion_state_reports_target_source();
  test_event_seq_is_monotonic();
  test_diagnostics_counters_are_exposed();
  test_trajectory_state_reports_progress();
  test_trajectory_reject_event_carries_id_and_index();
  run_all_tests();
  printf("test_motor_control: all passed\n");
  return 0;
}
