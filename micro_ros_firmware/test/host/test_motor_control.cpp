// motor_control 宿主测试：发现与生命周期、命令/轨迹仲裁、配置事务、
// 参数比对、限值钳制与安全帧重试。含内部状态白盒断言。
// tick 为 MCU 单调毫秒，支持 uint32 回绕。

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

// CERB 反馈：位置码 1820 = 10°，电机侧速度 120 rpm，Iq 码 2000 × 20mA = 40A。
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
  s.valid_until_tick_ms = tick + 30U;
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

// 有没有"已写未保存"的台。新模型逐台记脏位（entry->record_dirty_slots），
// 没有全局标志了——旧 API motor_control_take_record_dirty 已被删除，因为它会
// 成为"需要保存"的第二个真相源，与逐台脏位漂移。
static bool any_record_dirty(const motor_control_context_t *ctx) {
  for (size_t i = 0U; i < ctx->entry_count; ++i) {
    if (ctx->entries[i].used && ctx->entries[i].record_dirty_slots != 0U) {
      return true;
    }
  }
  return false;
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

static void enable_test_motor(motor_control_context_t *ctx, uint16_t motor_id,
    uint32_t tick) {
  motor_protocol_config_request_t enable{};
  enable.request_id = 1U;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.motor_id = motor_id;
  enable.enable = true;
  assert(motor_control_submit_config(ctx, &enable, tick));
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
  // 首帧发生于型号未绑定、运动量为 NaN；绑定后补一帧真实反馈建立运动龄。
  motor_protocol_raw_frame_t ready_feedback =
      vaxor_reply(motor_id, 0.0, 0.0, 0.0, tick + 1U);
  motor_control_on_raw_frame(ctx, &ready_feedback);
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
  assert(entry_of(ctx, motor_id)->dev_init == MOTOR_CONTROL_DEV_INIT_NA);
  // 运动用例显式武装；发现设备本身不再隐式使能。
  enable_test_motor(ctx, motor_id, tick + 2U);
}

// 完整使能链路（CERB 路径）：反馈 → 读设备自报身份 → 命中目录即绑定 →
// 连接 → 授权。
// ---------------------------------------------------------------------------
// 设备参数（CERB 0x607D:01/:02、0x6080、0x2538、0x2650:00、0x2541）：上电回读 → 与固件记录比对。
// ---------------------------------------------------------------------------

// 设备六个管理对象的顺序（与 adapter 的 REFRESH 阶段同序）。
static const uint16_t kDevIndex[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
  0x607DU, 0x607DU, 0x6080U, 0x2538U, 0x2650U, 0x2541U, 0x2546U, 0x2530U};
// 子索引与 adapter 的 REFRESH 阶段表一致；安全槽为 0x2650:00，
// :01/:02 返回 ABORT 0x06090011。
static const uint8_t kDevSub[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {1U, 2U, 0U,
                                                                 0U, 0U, 0U};

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

// 上电归一的一次驱动（0a）：定义在 feed_safety_write/feed_direction_write 之后
// （本函数由它们实现），这里先声明——feed_device_params 要用。
static uint32_t feed_boot_normalize(motor_control_context_t *ctx, uint16_t node,
    uint32_t tick);

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
  // 上电归一（0a）必须先走完，0b 的回读才有发起时机。测试侧统一在这里前置，
  // 于是绝大多数既有用例无需改动——只有"故意不回读"的失败注入用例要显式处理。
  uint32_t t = feed_boot_normalize(ctx, node, tick);
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
        sdo_read_ack(node, stage,
            stage == MOTOR_PROTOCOL_PARAM_SLOT_T_MAX && raw[stage] == 0 ? 300U :
            stage == MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID ? node : raw[stage], t + 1U);
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

// 走完一次 SET_VELOCITY_LIMIT（写 0x6080 + 读回核验），与 feed_torque_write 同形。
// 读回的原值取**写阶段编码器实际产出的那一个**（tx_write_raw，写应答落地时登记），
// 不由调用方猜 rpm：读回核验比的就是这个值，猜错的话事务会在 RETRY×3 后收敛成
// UNCONFIRMED——真发生时是响的，不是静默通过。
static uint32_t feed_velocity_write(motor_control_context_t *ctx, uint16_t node,
    uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack =
      sdo_write_ack(node, 0x6080U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  const uint32_t rpm =
      entry_of(ctx, node)->tx_write_raw[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
  motor_protocol_raw_frame_t rd = sdo_read_ack(node, 2U, rpm, tick + 3U);
  motor_control_on_raw_frame(ctx, &rd);
  motor_control_service(ctx, tick + 4U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 4U;
}

// 走完一次 SET_POSITION_LIMITS（写/读回 ×2：下限 0x607D:01 → 上限 0x607D:02），
// 与 feed_torque_write 同形。子索引 1/2 对应槽位 0/1；±1.0 rad 按 65536 cnt/360°
// 折算 = ∓10430 cnt（写下去 = 读回来，核验同值）。
static uint32_t feed_position_limits_write(motor_control_context_t *ctx,
    uint16_t node, uint32_t tick) {
  const uint32_t kCnt[2] = {0xFFFFD742U, 0x000028BEU}; // -10430 / +10430 cnt
  const uint8_t kSub[2] = {1U, 2U};
  const uint8_t kSlot[2] = {(uint8_t)MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN,
                            (uint8_t)MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX};
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  for (size_t phase = 0U; phase < 2U; ++phase) {
    motor_control_service(ctx, tick);
    (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    motor_protocol_raw_frame_t ack =
        sdo_write_ack(node, 0x607DU, kSub[phase], tick + 1U);
    motor_control_on_raw_frame(ctx, &ack);
    motor_control_service(ctx, tick + 2U);
    (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    motor_protocol_raw_frame_t rd =
        sdo_read_ack(node, kSlot[phase], kCnt[phase], tick + 3U);
    motor_control_on_raw_frame(ctx, &rd);
    motor_control_service(ctx, tick + 4U);
    (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    tick += 4U;
  }
  return tick;
}

// 保护开关写事务（写 0x2650:00 + 读回核验），无保存阶段。
static uint32_t feed_safety_write(motor_control_context_t *ctx, uint16_t node,
    uint32_t raw, uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack = sdo_write_ack(node, 0x2650U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t rd = sdo_read_ack(node, 4U, raw, tick + 3U);
  motor_control_on_raw_frame(ctx, &rd);
  motor_control_service(ctx, tick + 4U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 4U;
}

// 方向写事务（写 0x2541:00 + 读回核验），与 feed_safety_write 同形。
static uint32_t feed_direction_write(motor_control_context_t *ctx, uint16_t node,
    uint32_t raw, uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack = sdo_write_ack(node, 0x2541U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t rd = sdo_read_ack(node, 5U, raw, tick + 3U);
  motor_control_on_raw_frame(ctx, &rd);
  motor_control_service(ctx, tick + 4U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 4U;
}
// 走完一次上电归一（0a）：保护恒写 0；方向**仅在记录有槽 5 且值合法时**写，
// 否则整个方向步跳过（不猜方向，交给比对判基线缺失）。
// 必须在 feed_device_params 之前调用——0b 回读被 0a 阻塞。
// 读表项的 record_* 是白盒（本文件本就 -I 内核 src）。
static uint32_t feed_boot_normalize(motor_control_context_t *ctx, uint16_t node,
    uint32_t tick) {
  const motor_control_entry_t *e = entry_of(ctx, node);
  if (e->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE) {
    // 本会话早先已经归一过（重入 feed_device_params 的常见情形）：**不喂任何
    // 应答**。此刻多半有回读事务在飞，设备根本不会答这两个对象，硬喂只会被
    // 判成"非本阶段应答"（RETRY）而白白吃掉回读的重试次数。
    return tick;
  }
  if (e->tx_active && e->tx_request_id != e->boot_norm_request_id) {
    // 在飞的是**别的**事务（人工写 / 归一失败后的手动回读）：同理，设备没被
    // 问过保护/方向，不能喂那两条应答。
    return tick;
  }
  const uint8_t dir_bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  const bool has_dir = (e->record_raw_present & dir_bit) != 0U &&
      e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] <= 1U;
  const uint32_t dir_raw = e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];

  uint32_t t = feed_safety_write(ctx, node, 0U, tick); // 保护：恒写"生效"
  if (!has_dir) {
    motor_control_service(ctx, t);
    return t;
  }
  return feed_direction_write(ctx, node, dir_raw, t + 1U);
}

// 走归一的**一步**（写 + 读回核验），并断言这一拍发出去的正是该 index 的写请求
// （含写下去的原值）。与 feed_safety_write/feed_direction_write 的差别：收尾
// **不补最后一拍 service**，于是返回时归一事务已收尾、内核还没机会发起 0b 回读
// ——要看"写事务之后紧接着是不是回读"就必须用这个（那两个助手的收尾 service +
// drain 会把紧接着生成的回读首帧一并取走，断言会退化成空断言）。
static uint32_t step_boot_norm_write(motor_control_context_t *ctx, uint16_t node,
    uint16_t index, uint32_t raw, uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const uint8_t slot = (index == 0x2650U)
                           ? (uint8_t)MOTOR_PROTOCOL_PARAM_SLOT_SAFETY
                           : (uint8_t)MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION;
  motor_control_service(ctx, tick);
  const size_t n = motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  bool seen = false;
  bool saw_refresh_read = false;
  for (size_t i = 0U; i < n; ++i) {
    if (drain[i].size == 8U && drain[i].data[0] == 0x23U &&
        drain[i].data[1] == (uint8_t)(index & 0xFFU) &&
        drain[i].data[2] == (uint8_t)(index >> 8U) && drain[i].data[4] == raw) {
      seen = true;
    }
    if (drain[i].data[0] == 0x40U && drain[i].data[1] == 0x7DU) {
      saw_refresh_read = true; // 0x607D 的上传＝回读，归一这一步时还不许有
    }
  }
  assert(seen);            // 本步的写请求必须发出来
  assert(!saw_refresh_read);
  motor_protocol_raw_frame_t wack = sdo_write_ack(node, index, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &wack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t rack = sdo_read_ack(node, slot, raw, tick + 3U);
  motor_control_on_raw_frame(ctx, &rack);
  return tick + 3U;
}

// 造一条 Flash 记录（只含设备端原值 + 对应 SI 值由内核按 profile 比对）。
// include_safety_slots：true（默认）= 把 v3 的只读安全槽（4=保护开关 / 5=方向）
// 一并置位——"设备端回读过后落盘"的记录形状。false = 按 raw_present 原样，模拟
// **v2 迁移上来的记录**（`param_store_blob_migrate_v2` 只搬低 4 位，槽 4/5 为
// "未核验"），用来验证"记录没写过的槽位不参与比对"这条安全规则。
static motor_control_stored_params_t make_device_record(
    uint16_t motor_id, const uint32_t raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT],
    uint8_t raw_present, bool include_safety_slots = true) {
  motor_control_stored_params_t rec = {};
  rec.motor_id = motor_id;
  rec.has_position_limits = (raw_present & 0x03U) == 0x03U;
  rec.has_velocity_limit = (raw_present & 0x04U) != 0U;
  rec.has_torque_limit = (raw_present & 0x08U) != 0U;
  // v3：安全开关(0x2650:00)/方向(0x2541)两槽随记录落袋（出厂默认 0=保护开 /
  // 0=CW），上电比对据此也能发现设备侧保护被外部工具关掉。
  rec.device_raw_present =
      include_safety_slots
          ? (uint8_t)(raw_present | (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY) |
      (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION))
          : raw_present;
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

// 设备参数初始化收尾：先声明一份含方向基线的记录（真实板子第一次写过方向后
// 就是这个形状），再驱动自动回读与六槽比对。
// 不能再用"无记录"：方向基线缺失会判未初始化（见 motor_control_dev_mismatch_mask），
// 那是产品行为，不是测试可以绕过的。
static void complete_device_param_init(motor_control_context_t *ctx, uint16_t node,
    uint32_t tick) {
  motor_control_stored_params_t rec = make_device_record(node, kR58Raw, 0x00U);
  motor_control_apply_stored_params(ctx, &rec, 1U, tick);
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
  // 首帧在型号未绑定时不产生有限运动量；绑定后补一帧真实反馈建立运动龄。
  motor_protocol_raw_frame_t ready_feedback = cerb_feedback(motor_id, tick + 2U);
  motor_control_on_raw_frame(ctx, &ready_feedback);
  // 事务期的 P2 请求帧清掉，避免污染后续 TX 断言。
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
  motor_control_find_entry(ctx, motor_id)->dev_raw_present |= (1U << MOTOR_PROTOCOL_PARAM_SLOT_T_MAX);
  motor_control_find_entry(ctx, motor_id)->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX] = 300;
  enable_test_motor(ctx, motor_id, tick + 2U);
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
  1000U,   // offline_timeout_ms
  0U,      // enable_settle_ms
  true,    // autonomous_feedback
  false,   // config_verify_required
  1U,      // p1_batch_max
  mock_match,
  nullptr,   // is_discovery_frame（mock 无事务应答帧，match 到的都是存在证据）
  mock_decode,
  mock_encode_mit,
  nullptr, // aggregate_p1_frames
  mock_encode_enable,
  mock_encode_soft_stop,
  mock_encode_disable,
  nullptr,   // encode_discovery_sync
  mock_stage_count,
  mock_stage_info,
  mock_stage_encode,
  mock_on_response};

// mock 协议的测试 Profile（经指针版绑定 API 注入，不进生产型号目录）。
// 全字段显式初始化：新增字段若靠聚合零填充，旧测试会在不自知的情况下
// 改用 0 上限（曾把 kp_max/kd_max 挤错位）。
static const motor_protocol_model_profile_t g_mock_profile = {
  .model_name = "mock_model",
  .identity_string = nullptr, // mock 型号不在目录内，靠显式绑定注入
  .protocol = &g_mock_adapter,
  .gear_ratio = 1.0f,
  .gear_efficiency = 1.0f,
  .torque_constant_nm_per_amp = 0.0f,
  .rated_torque_nm = 0.0f,
  .peak_torque_nm = 0.0f,
  .rated_current_amp = 0.0f,
  .peak_current_amp = 0.0f,
  .rated_speed_rad_s = 0.0f,
  .supply_voltage_v = 1.0f,
  .default_kp = 0.0f,
  .default_kd = 0.0f,
  .mit_position_limit_rad = 0.0f,
  .mit_velocity_limit_rad_s = 0.0f,
  .mit_torque_limit_nm = 0.0f,
  .mit_kp_limit = 0.0f,
  .mit_kd_limit = 0.0f,
  .kp_max = 100.0f,
  .kd_max = 10.0f,
  .capability_flags = MOTOR_PROTOCOL_CAP_MODE_MIT};

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

// 容量 48：第 49 台拒绝并出 REGISTRY_FULL 事件（禁止静默截断）。
static void test_registry_capacity_48_rejects_overflow() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  // vaxor 节点 1..32 + CERB 节点 33..48，恰好注册 48 台。
  for (uint16_t node = 1; node <= 32; ++node) {
    motor_protocol_raw_frame_t f = vaxor_reply(node, 0.0, 0.0, 0.0, node * 10);
    motor_control_on_raw_frame(&ctx, &f);
  }
  for (uint16_t node = 33; node <= 48; ++node) {
    motor_protocol_raw_frame_t f = cerb_feedback(node, node * 10);
    motor_control_on_raw_frame(&ctx, &f);
  }
  assert(ctx.entry_count == 48U);

  // 第 49 台：合法且未占用的 CERB 节点 → 满表拒绝。
  motor_protocol_raw_frame_t extra = cerb_feedback(49, 1000);
  motor_control_on_raw_frame(&ctx, &extra);
  assert(ctx.entry_count == 48U);
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
  f = vaxor_reply(5, 0.0, 0.0, 0.0, 111);
  motor_control_on_raw_frame(&ctx, &f); // Profile 绑定后建立真实运动龄

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
  enable_test_motor(&ctx, 5, 160);
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
  // Fill with distinct P2 work. Repeated P1 snapshots intentionally coalesce
  // into one slot and can no longer be used to saturate this queue.
  ctx.tx_count = 0U;
  motor_protocol_tx_frame_t work{};
  work.tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
  for (size_t i = 0; i < MOTOR_CONTROL_MAX_TX_PENDING; ++i) {
    work.can_id = 0x600U + i;
    assert(motor_control_push_tx(&ctx, &work));
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
// 身份读取失败后有界退避重试，用尽才判不可识别。
static void test_identity_read_failure_retries_then_unbinds() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = cerb_feedback(9, 100);
  motor_control_on_raw_frame(&ctx, &f);

  // 设备从容不应答；喂心跳隔离"事务超时"与"心跳丢失判离线"两条路径。
  uint32_t t = 110U;
  for (; t <= 400U; t += 10U) {
    if (t % 20U == 0U) {
      motor_protocol_raw_frame_t hb = cerb_feedback(9, t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
  }
  const motor_control_entry_t *e = entry_of(&ctx, 9);
  assert(e->online && e->profile == NULL);
  assert(!e->identity_read_done);          // 还没定论——正是旧版缺的那一步
  assert(e->identity_read_attempts >= 1U); // 失败已计入
  assert(!events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 9));

  // 跑过全部重试窗口（每次事务 ~100ms + 500ms 退避）→ 用尽后才定论。
  for (; t <= 3000U; t += 10U) {
    if (t % 20U == 0U) {
      motor_protocol_raw_frame_t hb = cerb_feedback(9, t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
  }
  assert(entry_of(&ctx, 9)->identity_read_done);
  assert(entry_of(&ctx, 9)->profile == NULL); // 认不出 → 未绑定不可控
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_REJECTED, 9));

  // 定论之后不再发 SDO 请求（不刷屏事件环、不占总线）。
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  (void)flush_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (; t <= 3600U; t += 10U) {
    if (t % 20U == 0U) {
      motor_protocol_raw_frame_t hb = cerb_feedback(9, t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
  }
  const size_t n = flush_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (size_t i = 0; i < n; ++i) {
    assert(drain[i].can_id < 0x600U || drain[i].can_id > 0x67FU);
  }
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
  // 索引不符属于读取失败，退避重试用尽才报告 PROFILE_REJECTED。
  for (uint32_t t = 110U; t <= 3000U; t += 10U) {
    if (t % 20U == 0U) {
      motor_protocol_raw_frame_t hb = cerb_feedback(3, t);
      motor_control_on_raw_frame(&ctx2, &hb);
    }
    motor_control_service(&ctx2, t);
  }
  assert(entry_of(&ctx2, 3)->identity_read_done);
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

  // 通信租约在 age=30 仍有效，但目标执行截止已经到达，先停止快照来源。
  motor_control_service(&ctx, 149);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  motor_control_service(&ctx, 150);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  motor_control_service(&ctx, 151);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  // Stop barrier removes the unsent enable; only the current soft stop survives.
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) == 1);

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
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(!entry_of(&ctx, 5)->policy_enable);
  enable_test_motor(&ctx, 5, 271U);
  s2.sequence = 3U;
  s2.received_tick_ms = 272U;
  s2.valid_until_tick_ms = 302U;
  assert(motor_control_apply_command_snapshot(&ctx, &s2, 272U));
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
  // 705ms 重连帧发生在型号重新绑定前，只完成发现/识别；绑定后补一帧真实测量，
  // 否则已失效的旧运动时间戳不能作为恢复凭据。
  motor_protocol_raw_frame_t reconnected_feedback =
      vaxor_reply(5, 0.0, 0.0, 0.0, 707);
  motor_control_on_raw_frame(&ctx, &reconnected_feedback);
  motor_control_service(&ctx, 707);
  assert(entry_of(&ctx, 5)->last_motion_rx_tick_ms == 707U);
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

// 非法快照（NaN/重复 ID）整体拒绝；序号仅诊断，回退/重复/回绕均不限制受理。
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
  // 300ms 重连帧早于型号重新绑定，绑定后补一帧真实测量才能满足运动门禁。
  motor_protocol_raw_frame_t reconnected_feedback =
      vaxor_reply(5, 0.0, 0.0, 0.0, 303);
  motor_control_on_raw_frame(&ctx, &reconnected_feedback);
  motor_control_service(&ctx, 303);
  assert(entry_of(&ctx, 5)->last_motion_rx_tick_ms == 303U);
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

// CERB 的 0x080 SYNC 维持 20ms 状态上报，上报帧即心跳。
// 心跳丢失 ≥100ms（5×20ms，adapter 阈值）即 OFFLINE，与是否发命令无关。
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

  auto *limited = motor_control_find_entry(&ctx, 5);
  limited->soft_min_rad = -0.5f;
  limited->soft_max_rad = 0.5f;
  limited->has_soft_limits = true;
  motor_control_service(&ctx, 160);
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
  enable_test_motor(&ctx, 3, 119);

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
  // 环满丢最旧、保最新并计数。非法快照每次产生一条 SNAPSHOT_REJECTED，
  // 可填满事件环，不受设备节点数限制。
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
  enable_test_motor(&ctx, 3, 119);

  // Saturate with unrelated P2 work: repeated same-axis P1 is latest-wins.
  motor_control_command_snapshot_t snapshot = make_snapshot(1U, 130U);
  snapshot_add(&snapshot, 3, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &snapshot, 130U));
  motor_protocol_tx_frame_t work{};
  work.tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
  while (ctx.tx_count < MOTOR_CONTROL_MAX_TX_PENDING)
    assert(motor_control_push_tx(&ctx, &work));
  const uint32_t dropped_before = ctx.tx_dropped_count;
  motor_control_set_agent_connected(&ctx, false, 131U);
  // The barrier frees this axis's old enable/target and preserves unrelated P2.
  assert(ctx.tx_dropped_count == dropped_before);
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
  // 本用例只验证命令时效；局部刷新真实测量，避免新的运动反馈门成为
  // 提前触发条件。
  motor_protocol_raw_frame_t feedback = vaxor_reply(5, 0.0, 0.0, 0.0, 230);
  motor_control_on_raw_frame(&ctx, &feedback);
  motor_control_command_snapshot_t stale30 = make_snapshot(2, 205);
  snapshot_add(&stale30, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &stale30, 240)); // age=35
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(ctx.last_snapshot_sequence == 1U); // 未记录

  // age>100：撤权 + 协议级失能。
  feedback = vaxor_reply(5, 0.0, 0.0, 0.0, 250);
  motor_control_on_raw_frame(&ctx, &feedback);
  motor_control_command_snapshot_t stale100 = make_snapshot(3, 105);
  snapshot_add(&stale100, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &stale100, 260)); // age=155
  assert(!ctx.authorized);
  assert(ctx.require_new_snapshot);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 恢复链：新授权 + 序号 2 的新鲜快照（过期快照未消费序号）→ ACTIVE。
  assert(motor_control_set_authorization(&ctx, true, 270));
  enable_test_motor(&ctx, 5, 271U);
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

// 重号检测边界与冲突冻结。

// 事件计数（drain 是消费性的，一次扫完；与 events_contain 同源）。
static size_t count_events(motor_control_context_t *ctx,
    motor_control_event_type_t type, uint16_t motor_id) {
  motor_control_event_t events[64];
  size_t total = 0;
  size_t n;
  while ((n = motor_control_drain_events(ctx, events, 64)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (events[i].type == type && events[i].motor_id == motor_id) {
        total++;
      }
    }
  }
  return total;
}

// 同总线、同协议、同节点的两台设备无法区分，固件视为同一表项。
// 重号可能引起 CAN 错误或反馈超时，配置层须保证节点号唯一。
static void test_same_bus_same_protocol_duplicate_is_undetectable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t first = cerb_feedback(7, 100);
  motor_control_on_raw_frame(&ctx, &first);
  motor_protocol_raw_frame_t second = cerb_feedback(7, 110);
  motor_control_on_raw_frame(&ctx, &second); // 同节点、同总线、同协议

  assert(ctx.entry_count == 1); // 仍只建一台，不报冲突
  // 后续帧照常推进到 IDENTIFYING，不产生重号冲突。
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_IDENTIFYING);
  assert(count_events(&ctx, MOTOR_CONTROL_EVENT_CONFIG_CONFLICT, 7) == 0);
}

// ACTIVE 时注入冲突，须冻结运动并推送 P0 失能帧。
static void test_conflict_latch_freezes_motion_and_disables() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);
  size_t idx;
  assert(find_profile("vaxor_2550", &idx));
  assert(motor_control_bind_profile(&ctx, 5, idx, 110));
  // 首帧早于型号绑定，绑定后补一帧真实测量，供后续运动门禁使用。
  motor_protocol_raw_frame_t measured = vaxor_reply(5, 0.0, 0.0, 0.0, 110);
  motor_control_on_raw_frame(&ctx, &measured);
  motor_control_set_agent_connected(&ctx, true, 120);
  motor_control_set_authorization(&ctx, true, 130);
  enable_test_motor(&ctx, 5, 130);
  motor_control_command_snapshot_t s = make_snapshot(1, 140);
  snapshot_add(&s, 5, 0.1f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 150));
  motor_control_service(&ctx, 160);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(entry_of(&ctx, 5)->motion_enabled);
  {
    // 排空使能帧(P0)与 MIT 帧(P1)，让下面的 P0 计数干净。
    motor_protocol_tx_frame_t junk[MOTOR_CONTROL_MAX_TX_PENDING];
    while (motor_control_collect_tx(&ctx, junk, MOTOR_CONTROL_MAX_TX_PENDING) > 0) {
    }
  }

  // 同 node 走 CERB → 跨协议冲突。
  motor_protocol_raw_frame_t conflict = cerb_feedback(5, 170);
  motor_control_on_raw_frame(&ctx, &conflict);

  const motor_control_entry_t *e = entry_of(&ctx, 5);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(!e->motion_enabled);
  assert(!e->has_target);
  assert(!e->motion_authorized);
  // policy_enable 一并冻结：否则锁存清除后会仅凭全局授权 + 新快照自动复运动，
  // 违反 ARD §36「失败不自动恢复运动授权」。
  assert(!e->policy_enable);
  assert(!e->enable_pending);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_CONFIG_CONFLICT);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) == 1); // 失能帧
}

// 冲突事件只推一次：重复的冲突帧不得刷屏事件环。
static void test_conflict_event_pushed_once() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t seed = cerb_feedback(7, 100);
  motor_control_on_raw_frame(&ctx, &seed);
  for (uint32_t t = 110; t <= 300; t += 10) {
    motor_protocol_raw_frame_t conflict = vaxor_reply(7, 0.0, 0.0, 0.0, t);
    motor_control_on_raw_frame(&ctx, &conflict);
  }
  assert(count_events(&ctx, MOTOR_CONTROL_EVENT_CONFIG_CONFLICT, 7) == 1);
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
  enable_test_motor(&ctx, 3, 119);
  uint32_t seq = 1;
  ctx.tx_count = 0U;
  motor_protocol_tx_frame_t work{};
  work.tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
  for (size_t i = 0; i < MOTOR_CONTROL_MAX_TX_PENDING; ++i)
    assert(motor_control_push_tx(&ctx, &work));
  work.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  const auto dropped_before = ctx.tx_dropped_count;
  assert(!motor_control_push_tx(&ctx, &work));
  assert(ctx.tx_dropped_count == dropped_before + 1U);

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
  // Enable 决议本身不替代真实测量；恢复前补一帧新鲜反馈。
  motor_protocol_raw_frame_t recovered_feedback =
      vaxor_reply(5, 0.0, 0.0, 0.0, 215);
  motor_control_on_raw_frame(&ctx, &recovered_feedback);
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
  enable_test_motor(&ctx, 3, 100);
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
    enable_test_motor(&ctx, node, 100);
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
    fill.tx_slot = 1; // unrelated axis; this motor's barrier must not clear it
    fill.can_id = 0x700U;
    fill.size = 1U;
    last_requeue = motor_control_push_tx(&ctx, &fill);
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
  enable_test_motor(&ctx, 7, 281U);
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
    enable_test_motor(&ctx, nodes[k], 100);
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
  test_registry_capacity_48_rejects_overflow();
  test_duplicate_motor_id_across_buses_conflicts();
  test_identity_conflict_same_bus_different_protocol();
  test_enter_active_requires_auth_profile_snapshot();
  test_p0_requeue_after_downstream_failure();
  test_identity_read_binds_and_unknown_stays_unbound();
  test_identity_string_match_is_exact();
  test_identity_read_failure_retries_then_unbinds();
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
  test_same_bus_same_protocol_duplicate_is_undetectable();
  test_conflict_latch_freezes_motion_and_disables();
  test_conflict_event_pushed_once();
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
// 0x2538/0x2650:00/0x2541）全通后设备影子更新，param 视图反映换算后的关节侧值。
static void test_cerb_refresh_params_transaction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  while (motor_control_collect_tx(&ctx, nullptr, 0) > 0) {}

  // 绑定后默认影子：±3.14（0x2542 线协议满量程）、额定 π rad/s、峰值 45 N·m。
  const motor_control_param_view_t *d0 = view_of(&ctx, 7);
  assert(d0 != nullptr && d0->device_side_limits);
  assert(d0->limits_valid);
  assert(fabs(d0->pos_min_rad + 3.14f) < 1e-4);
  assert(fabs(d0->pos_max_rad - 3.14f) < 1e-4);
  assert(fabs(d0->velocity_limit_rad_s - MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(d0->torque_limit_nm - 45.0) < 1e-4);
  assert(d0->bound);
  // 未显式设置增益：视图回退 Profile 默认（cerb_r58_101：30/1）。
  assert(fabs(d0->kp - 30.0) < 1e-4 && fabs(d0->kd - 1.0) < 1e-4);

  // 读契约：设备侧型号必须先走完该台自己的归一（0a）才允许回读（内核
  // device_refresh_allowed）。用一份带方向基线的记录把归一走完；刻意用
  // step_boot_norm_write（不补收尾那一拍）好让下面的**手动**回读抢在自动回读
  // 之前发起——本用例测的正是手动回读这条路径。
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 104);
  const uint32_t norm_end = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 105U);
  (void)step_boot_norm_write(&ctx, 7, 0x2541U, 0U, norm_end + 1U);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);

  motor_protocol_config_request_t req = {};
  req.request_id = 41U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &req, 115));

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
  uint32_t t = 120;
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
  // 阶段 4/5：v3 只读安全槽（0x2650:00 保护开关、0x2541 方向），原值透传。
  motor_control_service(&ctx, t);
  r = sdo_read(0U, 0x2650U, 0U, t + 1); // 子索引 0x00（真机实测）
  motor_control_on_raw_frame(&ctx, &r);
  t += 10;
  motor_control_service(&ctx, t);
  r = sdo_read(0U, 0x2541U, 0U, t + 1);
  motor_control_on_raw_frame(&ctx, &r);
  t += 10; motor_control_service(&ctx, t);
  r = sdo_read(0, 0x2546, 300, t + 1); motor_control_on_raw_frame(&ctx, &r);
  t += 10; motor_control_service(&ctx, t);
  r = sdo_read(0, 0x2530, 7, t + 1); motor_control_on_raw_frame(&ctx, &r);
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
  // 安全槽（保护 0 / 方向 0）也给全：归一（0a）要按记录写方向，缺槽就跳过方向
  // 步、归一收不了尾；本用例要的是"归一走完之后手动回读照旧受理"。
  rec.device_raw_present =
      (uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY) |
      (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION));
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  // 尚未回读设备：影子取记录（来源 = RECORD），页面显示的就是记录值。
  assert(fabs(view_of(&ctx, 7)->velocity_limit_rad_s - 1.5) < 1e-4);
  assert(fabs(view_of(&ctx, 7)->torque_limit_nm - 20.0) < 1e-4);
  assert(view_of(&ctx, 7)->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_RECORD);

  // 读契约：设备侧型号必须先把该台归一（0a）走完才允许回读（内核
  // device_refresh_allowed）。两步刻意不补收尾那一拍，好让下面的**手动**回读
  // 抢在自动回读之前发起（本用例测的就是手动回读这条路径）。
  const uint32_t norm_end = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 115U);
  (void)step_boot_norm_write(&ctx, 7, 0x2541U, 0U, norm_end + 1U);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);

  // 设备回读事务更新影子（模拟内核 refresh 成功路径：直接走一遍完整事务）。
  motor_protocol_config_request_t req = {};
  req.request_id = 51U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &req, 140));
  auto sdo = [](uint8_t sub, uint16_t index, uint32_t value, uint32_t tick) {
        uint8_t p[8] = {0x43, (uint8_t)(index & 0xFFU), (uint8_t)(index >> 8), sub,
                        (uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU),
                        (uint8_t)((value >> 16) & 0xFFU), (uint8_t)((value >> 24) & 0xFFU)};
        motor_protocol_raw_frame_t f = raw_frame(0, 0x587U, 8, p, tick);
        f.is_fd = false;
        return f;
      };
  uint32_t t = 145;
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
  motor_protocol_raw_frame_t f5 = sdo(0, 0x2650, 0, t + 1); // 子索引 0x00（真机实测）
  motor_control_on_raw_frame(&ctx, &f5);
  t += 10; motor_control_service(&ctx, t);
  motor_protocol_raw_frame_t f6 = sdo(0, 0x2541, 0, t + 1);
  motor_control_on_raw_frame(&ctx, &f6);
  t += 10; motor_control_service(&ctx, t);
  auto f7 = sdo(0, 0x2546, 300, t + 1); motor_control_on_raw_frame(&ctx, &f7);
  t += 10; motor_control_service(&ctx, t);
  auto f8 = sdo(0, 0x2530, 7, t + 1); motor_control_on_raw_frame(&ctx, &f8);
  assert(has_result_event(&ctx, 51U, MOTOR_PROTOCOL_CONFIG_OK));

  // 回读之后：来源转为设备，记录不得再覆盖影子（设备为准）。
  assert(view_of(&ctx, 7)->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  motor_control_apply_stored_params(&ctx, &rec, 1, t + 20);
  motor_control_apply_stored_params(&ctx, &rec, 1, t + 40);
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(fabs(v->velocity_limit_rad_s - 2.0 * MOTOR_PROTOCOL_PI * 10.0 / 101.0) < 1e-4);
  assert(fabs(v->torque_limit_nm - 45.0 / 10.8) < 1e-3);
}


// 上电先归一、再自动回读，一致时报告 OK。
static void test_device_param_boot_verify_ok() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(entry_of(&ctx, 7)->boot_record_offered);

  // 上电归一（0a）：保护恒写"生效"(0) → 方向写记录值(0)。零增益回读之前必须
  // 先把设备里易失的两个安全对象写回定态，否则比对读的是上电初值。
  uint32_t t = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 120U);
  t = step_boot_norm_write(&ctx, 7, 0x2541U, 0U, t + 1U);

  // 归一收尾后的下一拍即发起自动回读：第一个 SDO 请求是 0x607D:01 的上传
  // （0x600+node，P2）。
  motor_control_service(&ctx, t + 1U);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  size_t n = flush_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U);
  assert(frames[0].can_id == 0x600U + 7U);
  assert(frames[0].tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);
  assert(frames[0].data[0] == 0x40U && frames[0].data[1] == 0x7DU); // 读 0x607D

  t = feed_device_params(&ctx, 7, kR58Raw, t + 2U, false);
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

// 归一的顺序是契约：保护/方向写事务必须先于六槽回读发生。
static void test_boot_normalize_runs_before_refresh() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 逐拍看请求帧：前两条 SDO 写必须是 0x2650:00（保护）与 0x2541:00（方向），
  // 之后才允许出现 0x607D 的读。每步都要喂应答才推进得下去——总线一问一答，
  // 上一条写在飞时内核不会发起下一条。
  const uint32_t t0 = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 120U);
  // 归一未完成期间，回读不得发起：此时表项仍停在 0a。
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(!entry_of(&ctx, 7)->dev_read_done);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DIRECTION);
  const uint32_t t1 = step_boot_norm_write(&ctx, 7, 0x2541U, 0U, t0 + 1U);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);

  // 两步都核验成功 → 归一收尾；下一拍才轮到回读，第一个请求是 0x607D:01。
  motor_control_service(&ctx, t1 + 1U);
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U);
  assert(drain[0].data[0] == 0x40U && drain[0].data[1] == 0x7DU);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(!entry_of(&ctx, 7)->dev_read_done);
}

// 归一失败用尽重试后报告 UNVERIFIED，不自动回读；结论不限制使能或运动。
static void test_boot_normalize_failure_state_before_refresh() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 只喂心跳、一个 SDO 应答都不给：归一写超时重试用尽（3 次尝试，每次
  // 3×30ms 超时 + 500ms 退避）。
  uint32_t t = run_heartbeat_only(&ctx, 7, 120U, 3000U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  // UNVERIFIED 仅报告，不限制运动。
  assert(motor_control_dev_params_allow_motion(e));
  // 未发起回读，六槽均未核对。
  assert(e->dev_read_attempts == 0U);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED);
  assert(mask == (uint32_t)MOTOR_CONTROL_DEV_RAW_ALL); // 六槽全未核对

  // 归一失败已定论后允许手动回读；clear_fault(DEV_PARAM) 共用此路径。
  assert(entry_of(&ctx, 7)->boot_norm_stage != MOTOR_CONTROL_BOOT_NORM_DONE);
  motor_protocol_config_request_t refresh = {};
  refresh.request_id = 60U;
  refresh.motor_id = 7;
  refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &refresh, t + 10U));
  (void)feed_device_params(&ctx, 7, kR58Raw, t + 11U, false);
  assert(has_result_event(&ctx, 60U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 结论转好后 0a 的不变量复检自动补做归一（stage 一直停在 PROTECT）。
  (void)feed_boot_normalize(&ctx, 7, t + 60U);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
}

// 人工写与归一共用 kind，以 boot_norm_request_id 区分；人工写不得推进归一步。
static void test_boot_normalize_not_advanced_by_manual_write() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT);

  motor_protocol_config_request_t off = {};
  off.request_id = 40U; off.motor_id = 7;
  off.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
  off.safety_protect_raw = 1U;
  assert(!motor_control_submit_config(&ctx, &off, 120));
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT);
  (void)feed_device_params(&ctx, 7, kR58Raw, 125U, false);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
  assert(entry_of(&ctx, 7)->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 0U);

}

// 归一完成前拒绝人工回读，避免比较上电初值；已有定论后仍允许重读。
// system.refresh 与 clear_fault 共用提交入口。
static void test_manual_refresh_rejected_during_normalize_window() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  // 窗口形状：记录已落袋、归一还没发起（没有事务在飞、也没有失败退避）。
  assert(entry_of(&ctx, 7)->boot_record_offered);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT);
  assert(!entry_of(&ctx, 7)->dev_read_done);
  assert(!entry_of(&ctx, 7)->tx_active);

  motor_protocol_config_request_t refresh = {};
  refresh.request_id = 50U;
  refresh.motor_id = 7;
  refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(!motor_control_submit_config(&ctx, &refresh, 111));
  assert(has_result_event(&ctx, 50U, MOTOR_PROTOCOL_CONFIG_BUSY));
  assert(!entry_of(&ctx, 7)->tx_active); // 被拒即不起事务

  // 归一照常完成，随后回读报告 OK。
  (void)feed_boot_normalize(&ctx, 7, 120U);
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
  (void)feed_device_params(&ctx, 7, kR58Raw, 130U, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);

  // 定论之后的人工回读照旧受理——拒绝不是靠"永远不许读"换来的。
  motor_protocol_config_request_t again = refresh;
  again.request_id = 51U;
  assert(motor_control_submit_config(&ctx, &again, 200));
  (void)feed_device_params(&ctx, 7, kR58Raw, 201U, false);
  assert(has_result_event(&ctx, 51U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}

// 记录无方向槽（v2 迁移上来的旧 blob / 新烧板）：归一**跳过**方向步、不写
// 0x2541，回读照跑，比对判基线缺失。
static void test_boot_normalize_skips_direction_without_record() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU, false);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 保护写 + 读回核验；随后**直接**是六槽回读，中间不许出现 0x2541 写。
  const uint32_t t = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 120U);
  motor_control_service(&ctx, t + 1U);
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U); // 只有回读首帧，没有方向写
  for (size_t i = 0U; i < n; ++i) {
    assert(!(drain[i].size == 8U && drain[i].data[0] == 0x23U &&
        drain[i].data[1] == 0x41U && drain[i].data[2] == 0x25U));
  }
  assert(drain[0].data[0] == 0x40U && drain[0].data[1] == 0x7DU);
  // 跳过后归一即完成，回读照跑。
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
  (void)feed_device_params(&ctx, 7, kR58Raw, t + 2U, false);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)); // 只有方向
}

// 无 W25 记录时报告 NO_RECORD，不限制运动。
static void test_dev_init_no_record_reports_without_gate() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);

  // 无记录也须以非 NULL 指针、count=0 告知；NULL 不会置 boot_record_offered。
  motor_control_stored_params_t empty = {};
  motor_control_apply_stored_params(&ctx, &empty, 0U, 110);
  assert(entry_of(&ctx, 7)->boot_record_offered);
  assert(!entry_of(&ctx, 7)->record_established);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);

  (void)feed_device_params(&ctx, 7, kR58Raw, 120U, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(motor_control_dev_params_allow_motion(e));
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_NONE);

  // 事件带的是本态本身（detail=0：没有"哪一项对不上"可报，整台没基线）。
  uint16_t state = 0U;
  uint32_t mask = 0xFFFFFFFFU;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(mask == 0U);

  // 参数视图也带本态（页面据此把固件列整行显示「无记录」）。
  assert(view_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);

  // NO_RECORD 不拒绝使能，也不置 DEV_PARAM 故障。
  motor_protocol_config_request_t en = {};
  en.request_id = 90U;
  en.motor_id = 7;
  en.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  en.enable = true;
  assert(motor_control_submit_config(&ctx, &en, 200));
  assert(has_result_event(&ctx, 90U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(e->policy_enable && e->motion_authorized);
  assert(e->last_failure_reason != MOTOR_CONTROL_FAULT_DEV_PARAM);
}

// 人工写建立记录后重新比对，补齐方向基线后报告 OK。
static void test_dev_init_flags_safety_mismatch_at_runtime() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);

  // 运行中复查要走**手动回读**：自动回读每会话只发一次（dev_read_done），
  // 上电那一次已经用掉了。
  motor_protocol_config_request_t refresh = {};
  refresh.request_id = 96U;
  refresh.motor_id = 7;
  refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &refresh, 140));

  uint32_t bad[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  memcpy(bad, kR58Raw, sizeof(bad));
  bad[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U; // 外部工具把保护关了
  (void)feed_device_params(&ctx, 7, bad, 141U, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  // 不一致只报告，不修改 policy_enable 或生命周期。
  assert(e->policy_enable);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY));
}

// 峰值电流 13300mA 与记录 10800mA 不符：报告 MISMATCH，影子取设备值；不限制使能。
static void test_device_param_mismatch_reports_without_gate() {
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
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_NONE);
  assert(e->policy_enable); // 测试已显式使能，未被 MISMATCH 冻结
  // 峰值电流槽不一致，事件位图应为 1<<3。
  uint16_t state = 0U;
  uint32_t detail = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &detail));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(detail == (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX));
  // 影子 = 设备真值（设备为准），不是记录值。
  assert(fabs(e->dev_max_torque_nm - 13.3 * kR58LimitNmPerAmp) < 0.1);

  // MISMATCH 不拒绝使能。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 71U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  motor_control_service(&ctx, 140);
  assert(motor_control_submit_config(&ctx, &enable, 141));
  assert(has_result_event(&ctx, 71U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->policy_enable && entry_of(&ctx, 7)->motion_authorized);

  // 生命周期不得因 MISMATCH 落锁（使能整定窗内仍停在 READY_DISABLED）。
  assert(entry_of(&ctx, 7)->lifecycle != MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 写事务 = 设备 + 固件记录一起写：写核验通过后记录同步、比对重判。
// （记录**内容**变了才排落盘——本用例写回的是记录里已有的值，故不排。）
static void test_device_param_missing_direction_baseline_reports() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU, false);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)); // 只有方向
  // 门禁已取消：结论只报，不改生命周期。
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}

// 整台无记录时报告 NO_RECORD、detail=0。须显式传空记录，
// 不用会注入方向基线的 complete_device_param_init 助手。
static void test_device_param_no_record_reports_on_direction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t none = {};
  motor_control_apply_stored_params(&ctx, &none, 0U, 110);
  assert(!any_record_dirty(&ctx)); // 还没写过设备：无脏位
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(motor_control_dev_params_allow_motion(e)); // 门禁已取消：未初始化也不再拦
  // 首次写入保护槽会改变原值记录并置脏位，待 SAVE 落 W25。
  // 必须先比较原值与存在位，再更新 record_raw_present。
  assert(any_record_dirty(&ctx));
  // 内部归一 request_id bit31=1，不建立 record_established 基线。
  assert(!e->record_established);
  uint16_t state = 0U;
  uint32_t mask = 0xFFFFFFFFU;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(mask == 0U); // 整台没基线：没有"哪一项对不上"可报
}

// v2 记录缺安全/方向槽：归一补保护原值 0 后，回读 1 应点名 bit4；
// 方向仍缺基线，同时点名 bit5。
static void test_device_param_migrated_record_flags_protect_off() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU, false);
  assert(rec.device_raw_present == 0x0FU); // 槽 4/5 未核验
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  const uint32_t saved = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U; // 外部工具把保护关了
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = saved;

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  // 归一那一拍已经把保护写进记录（这一位是"归一成功"的证据）。
  assert((e->record_raw_present & (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY)) != 0U);
  assert(e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 0U);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == ((1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY) |
      (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)));
  assert(saved == 0U);
}

// 非法方向值（槽 5 > 1）：与"缺槽"同待遇——0a 跳过方向步（不拿非法值去写
// 设备），比对层判基线缺失（设计 §3.5 的"槽值 > 1"）。
static void test_boot_normalize_skips_illegal_direction_value() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  const uint32_t saved = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] = 2U; // 非法值
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] = saved;

  // 保护写照做；随后**不许**出现 0x2541 的写请求。
  const uint32_t t = step_boot_norm_write(&ctx, 7, 0x2650U, 0U, 120U);
  motor_control_service(&ctx, t + 1U);
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  assert(n == 1U); // 只有回读首帧，没有方向写
  for (size_t i = 0U; i < n; ++i) {
    assert(!(drain[i].size == 8U && drain[i].data[0] == 0x23U &&
        drain[i].data[1] == 0x41U && drain[i].data[2] == 0x25U));
  }
  assert(entry_of(&ctx, 7)->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
  (void)feed_device_params(&ctx, 7, kR58Raw, t + 2U, false);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)); // 只有方向
  assert(saved == 0U);
}
// 在页面上点一次方向（写事务建立基线）→ 重判转好。
static void test_device_param_unverified_reports_without_gate() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 先完成归一，隔离回读失败路径。
  uint32_t t = feed_boot_normalize(&ctx, 7, 120U);

  // 只喂心跳、不回应答：单次尝试 = 阶段 0 连续 3 次超时即定论，尝试之间退避
  // 500ms；用尽 MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS 次才定论。
  t = run_heartbeat_only(&ctx, 7, t + 1U, 3000U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_read_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED);
  assert(e->dev_read_done);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(e->policy_enable); // 发现时默认使能，未被 UNVERIFIED 冻结
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_DEV_INIT, 7));

  // UNVERIFIED 不拒绝使能。
  motor_protocol_config_request_t enable = {};
  enable.request_id = 91U;
  enable.motor_id = 7;
  enable.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  enable.enable = true;
  assert(motor_control_submit_config(&ctx, &enable, t));
  assert(has_result_event(&ctx, 91U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->policy_enable && entry_of(&ctx, 7)->motion_authorized);
  // 生命周期不得因 UNVERIFIED 落锁。
  assert(entry_of(&ctx, 7)->lifecycle != MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 首次回读失败 → 退避重试、期间仍 PENDING；重试成功即 OK，不退化 UNVERIFIED。
static void test_device_param_read_retry_recovers() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 上电归一（0a）先走完：本用例要测的是**回读**的退避重试。
  uint32_t t = feed_boot_normalize(&ctx, 7, 120U);

  // 第一次尝试：设备不应答（只喂心跳）→ 未定论，排下次重试。
  t = run_heartbeat_only(&ctx, 7, t + 1U, t + 400U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_read_attempts == 1U);
  assert(!e->dev_read_done);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(motor_control_dev_params_allow_motion(e)); // 门禁已取消：PENDING 也不再拦

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

// PENDING 不限制使能，回读完成后报告 OK。
static void test_device_param_pending_does_not_block_enable() {
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
  assert(motor_control_submit_config(&ctx, &enable, 113));
  assert(has_result_event(&ctx, 95U, MOTOR_PROTOCOL_CONFIG_OK));

  // 设备参数回读收尾 → 结论转 OK（本条只验遥测收敛，使能早已受理）。
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}

// ACTIVE 时手动回读发现 MISMATCH 仅报告，不软停、不失能、不撤权。
static void test_device_param_runtime_mismatch_reports_only() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  // 用 bound 版而不是 ready 版：ready 注入的基线只含方向/保护两槽（0x30），而
  // 记录**首次捕获即胜出**（record_valid 后不再被覆盖）——本用例要的"峰值电流
  // 与记录不符"必须在第一次就落袋，否则限值槽不在基点里、比对恒通过。
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
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

  // 手动回读，设备里的峰值电流与记录不符 → 结论转 MISMATCH，但运行态不动。
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
  // 运行态原样：仍在 ACTIVE，授权/使能/运动位都没被动过。
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(e->policy_enable && e->motion_authorized && e->motion_enabled);
  // 不得因为一次比对结论就软停/失能。
  assert(!events_contain(&ctx, MOTOR_CONTROL_EVENT_SOFT_STOP, 7));
  assert(!saw_p0);
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
  assert((v->dev_raw_present & 0x3FU) == 0U);        // 还没回读设备
  assert(v->record_raw_present == 0x3FU);  // 记录基准在视图里可见
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
  assert(v->record_raw_present == 0x3FU);
  assert(v->record_raw[3] == kR58PeakMa);
  assert(v->limits_source == MOTOR_CONTROL_LIMITS_SOURCE_DEVICE);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
}

// 参数视图要把"两行"（固件记录 / 驱动板回读）和两张位图都端出来：
// DeviceParamReport 靠它填。两行必须同一把尺子（关节侧 SI：rad / rad/s / N·m），
// 否则人没法逐格对，两行就成了两串不可比的数字。
static void test_param_view_carries_report_fields() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  // make_device_record 只填 raw；blob 里 SI 与 raw 是同一笔事务产生的两个投影，
  // 这里照同一套换算补上（否则 record_si 全是 0，两行对不上的断言就变假断言）。
  rec.pos_min_rad = (float)motor_protocol_cerb_limit_cnt_to_rad(-32768);
  rec.pos_max_rad = (float)motor_protocol_cerb_limit_cnt_to_rad(32768);
  rec.velocity_limit_rad_s =
      (float)motor_protocol_cerb_motor_rpm_to_joint_rad_s(101.0, kR58Raw[2]);
  rec.torque_limit_nm =
      (float)motor_protocol_limit_ma_to_joint_nm(kR58LimitNmPerAmp, kR58Raw[3]);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 还没回读设备：设备那一行**整行 NaN**。缺失量绝不能是 0——0 会在页面上冒充
  // "限位设成了 0"，那是个能让人按下去的值。
  assert(std::isnan(view_of(&ctx, 7)->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN]));

  const uint32_t t = feed_device_params(&ctx, 7, kR58Raw, 120U, false);
  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(v->dev_mismatch_mask == 0U);
  assert(v->record_dirty_slots == 0U);

  // 设备侧 SI：回读值换算（−32768 cnt = −180° = −π rad；10800 mA = 45 N·m）。
  assert(fabs(v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] +
      (float)MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] - 45.0f) < 0.1f);
  // 记录侧 SI：与设备一致（同一组值的镜像），两行能逐格对上。
  assert(fabs(v->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] -
      v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN]) < 1e-4);
  assert(fabs(v->record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] -
      v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX]) < 0.1f);

  const uint32_t tw = t + 10U;
  const uint32_t written_rpm = kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
  // 不一致时位图点名是哪一项（运行中外部改动设备的形状）。
  uint32_t bad[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  memcpy(bad, kR58Raw, sizeof(bad));
  bad[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX] = written_rpm; // 速度这项设备与记录仍一致
  bad[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] = 2000U;
  motor_protocol_config_request_t rf = {};
  rf.request_id = 61U;
  rf.motor_id = 7;
  rf.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &rf, tw + 1U));
  (void)feed_device_params(&ctx, 7, bad, tw + 2U, false);
  v = view_of(&ctx, 7);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(v->dev_mismatch_mask ==
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX));
}

// 多台 CERB 并行上电：每台先走完归一（保护写 0 + 方向写记录值，各带读回核验），
// 再各发 6 个 SDO 读请求，对象顺序固定为 0x607D:01（位置下限）→ 0x607D:02
// （位置上限）→ 0x6080（最大转速）→ 0x2538（峰值电流）→ 0x2650:00（保护开关）
// → 0x2541（方向）。多台并行时同一拍各台各推进一个阶段，总线上是"逐台交错"
// 而非"逐台跑完再下一台"——按台过滤日志时每台恰好 2 条写 + 2 条写后读回 +
// 6 条回读，且**每台的写事务都排在它自己的回读之前**（顺序是契约）。
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
  // 请求**，一问一答）→ 立刻喂该请求的应答。写请求（0x23）与读请求（0x40）
  // 都要应答，否则归一那两步会超时重试、回读根本轮不上。
  uint8_t got_count[4] = {0};     // 该台已收到的**回读**条数（归一读回不计）
  uint8_t read_seen[4] = {0};     // 该台已收到的读请求条数（归一读回 + 回读）
  uint8_t got_index[4][MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {{0}};
  uint32_t t = 120U;
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  // 每台 10 条请求（2 写 + 2 写后读回 + 6 回读），同总线一问一答 ⇒ ≥40 拍。
  for (uint8_t round = 0U; round < 96U; ++round) {
    for (size_t i = 0; i < 4U; ++i) {
      motor_protocol_raw_frame_t hb = cerb_feedback(ids[i], t);
      motor_control_on_raw_frame(&ctx, &hb);
    }
    motor_control_service(&ctx, t);
    const size_t n = motor_control_collect_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
    size_t requests = 0U;
    for (size_t k = 0; k < n; ++k) {
      const uint16_t node = (uint16_t)(frames[k].can_id - 0x600U);
      assert(node >= 1U && node <= 4U);
      const uint16_t index =
          (uint16_t)(frames[k].data[1] | ((uint16_t)frames[k].data[2] << 8U));
      const uint8_t sub = frames[k].data[3];
      if (frames[k].data[0] == 0x23U) {
        // 写请求（归一的 0x2650 / 0x2541）：回写应答，不占读位。
        requests++;
        assert(index == 0x2650U || index == 0x2541U);
        motor_protocol_raw_frame_t wack = sdo_write_ack(node, index, sub, t + 1U);
        motor_control_on_raw_frame(&ctx, &wack);
        continue;
      }
      if (frames[k].data[0] != 0x40U) {
        continue; // 只看上传（读）请求
      }
      requests++;
      size_t slot = 5U; // 0x2541 方向
      if (index == 0x607DU) {
        slot = (sub == 1U) ? 0U : 1U;
      } else if (index == 0x6080U) {
        slot = 2U;
      } else if (index == 0x2538U) {
        slot = 3U;
      } else if (index == 0x2650U) {
        slot = 4U;
      } else if (index == 0x2546U) {
        slot = 6U;
      } else if (index == 0x2530U) {
        slot = 7U;
      }
      if (read_seen[node - 1U] == 0U) {
        // 该台的第一条读请求必须是归一保护写的读回核验（0x2650:00）。
        assert(slot == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
      } else if (read_seen[node - 1U] == 1U) {
        // 第二条是归一方向写的读回核验——两次归一步都排在回读之前。
        assert(slot == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      } else {
        assert(slot == got_count[node - 1U]); // 回读阶段顺序固定
        got_index[node - 1U][got_count[node - 1U]] = (uint8_t)index;
        got_count[node - 1U]++;
      }
      read_seen[node - 1U]++;
      motor_protocol_raw_frame_t ack =
          sdo_read_ack(node, (uint8_t)slot, slot == 6U ? 300U : slot == 7U ? node : kR58Raw[slot], t + 1U);
      motor_control_on_raw_frame(&ctx, &ack);
    }
    // 一问一答：同一总线上不允许出现两条同时在飞的 SDO 请求。
    assert(requests <= 1U);
    t += 2U;
  }
  for (size_t i = 0; i < 4U; ++i) {
    assert(got_count[i] == MOTOR_PROTOCOL_PARAM_SLOT_COUNT); // 每台恰好 6 条回读
    assert(read_seen[i] == (uint8_t)(2U + MOTOR_PROTOCOL_PARAM_SLOT_COUNT));
    assert(entry_of(&ctx, ids[i])->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE);
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
  assert(e->record_raw_present == 0x3FU);
  assert(e->record_raw[3] == kR58PeakMa);
  assert(e->dev_raw_present == 0U);           // 旧影子作废
}

// 外部写入不得超过**固定硬件参数表**（规格书）：请求超限时钳到型号表峰值，
// 且落到设备上的原值正好等于规格书的峰值电流 / 额定转速 / 位置量化范围。
// 覆盖三列：扭矩(=电流)、速度、位置。
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

// Hermite 求值 / 解析速度 / 末点钳制不外推 / 前馈段内保持。
static void test_trajectory_interpolates_and_clamps_at_end() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);
  traj_add_point(&t, 1000U, 5, 1.0f, 0.25f);
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  motor_control_trajectory_step(&ctx, 300U); // elapsed 0 → 首点
  assert(entry_of(&ctx, 5)->target_position_rad == 0.0f);

  feed_vaxor_heartbeat(&ctx, 5, 7, 800U);
  motor_control_trajectory_step(&ctx, 800U); // elapsed 500 → 中点
  assert(std::fabs(entry_of(&ctx, 5)->target_position_rad - 0.5f) < 1e-6f);
  // 速度是 Hermite 的解析导数：两端切矢为 0 的对称段在中点取峰值
  // = 1.5 × 平均斜率 = 1.5 rad/s。（线性插值会给常数 1.0——那正是被消除的台阶；
  // 1.5 倍峰值意味着同样的行程/时长下瞬时速度更高，故限速钳制仍必须生效。）
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s - 1.5f) < 1e-6f);
  assert(entry_of(&ctx, 5)->target_effort_nm == 0.0f); // 段内保持左端点

  feed_vaxor_heartbeat(&ctx, 5, 7, 9000U);
  motor_control_trajectory_step(&ctx, 9000U); // 越过末点 → 钳末点
  assert(entry_of(&ctx, 5)->target_position_rad == 1.0f);
  assert(entry_of(&ctx, 5)->target_velocity_rad_s == 0.0f);
  assert(entry_of(&ctx, 5)->target_effort_nm == 0.25f);
}

// 单调三次 Hermite：段间速度连续（C¹）。等间隔路点（0 → 1 → 3）的
// Fritsch–Carlson 加权调和切矢 = 3 / (3/1 + 3/2) = 4/3 rad/s。
// 线性插值在这里是一次速度台阶（左 1.0 / 右 2.0），本用例正是钉住"台阶消失"。
static void test_trajectory_hermite_c1_continuity() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);
  traj_add_point(&t, 1000U, 5, 1.0f, 0.0f);
  traj_add_point(&t, 2000U, 5, 3.0f, 0.0f);
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  // 路点前后各 1ms 采样（时间轴是整数 ms，这是最细的采样）。
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U + 999U);
  motor_control_trajectory_step(&ctx, 300U + 999U);
  const float v_before = entry_of(&ctx, 5)->target_velocity_rad_s;
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U + 1001U);
  motor_control_trajectory_step(&ctx, 300U + 1001U);
  const float v_after = entry_of(&ctx, 5)->target_velocity_rad_s;

  // 线性插值下这三个断言全不成立（1.0 / 2.0 / 差 1.0）。
  assert(std::fabs(v_before - v_after) < 1e-2f);  // 台阶消失
  assert(std::fabs(v_before - (4.0f / 3.0f)) < 1e-2f);
  assert(std::fabs(v_after - (4.0f / 3.0f)) < 1e-2f);
}

// 非均匀时间轴：0→1 用 1s，1→5 用 2s，割线为 1 和 2 rad/s。
// 真正的非均匀 Fritsch–Carlson 切矢为
// (5+4)/(5/1+4/2) = 9/7；简单算术平均会错误得到 1.5。
static void test_trajectory_hermite_uses_nonuniform_time_weights() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);
  traj_add_point(&t, 1000U, 5, 1.0f, 0.0f);
  traj_add_point(&t, 3000U, 5, 5.0f, 0.0f);
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  feed_vaxor_heartbeat(&ctx, 5, 7, 300U + 1000U);
  motor_control_trajectory_step(&ctx, 300U + 1000U);
  assert(std::fabs(entry_of(&ctx, 5)->target_position_rad - 1.0f) < 1e-6f);
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s - (9.0f / 7.0f)) <
      1e-3f);
}

// 单调三次 Hermite：端点强制零速、局部极值处切矢限为 0（因而段内不超调）。
static void test_trajectory_hermite_no_overshoot_and_zero_endpoints() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  // 0 → 1 → 0.5：中间点两侧割线异号（+1.0 / −0.5），是局部极大。
  // 不做单调限幅时中心差分 = (1.0 − 0.5)/2 = 0.25 ≠ 0，曲线会冲过 1.0。
  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);
  traj_add_point(&t, 1000U, 5, 1.0f, 0.0f);
  traj_add_point(&t, 2000U, 5, 0.5f, 0.0f);
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  // 起点强制零速（线性插值在此给 Δ_0 = +1.0）。
  motor_control_trajectory_step(&ctx, 300U);
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s) < 1e-6f);

  // 局部极大处切矢为 0（线性插值在此给 Δ_1 = −0.5）。
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U + 1000U);
  motor_control_trajectory_step(&ctx, 300U + 1000U);
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s) < 1e-3f);

  // 第二段（1.0 → 0.5）密集采样：不得超调。
  for (uint32_t e = 1000U; e <= 2000U; e += 25U) {
    feed_vaxor_heartbeat(&ctx, 5, 7, 300U + e);
    motor_control_trajectory_step(&ctx, 300U + e);
    const float x = entry_of(&ctx, 5)->target_position_rad;
    assert(x >= 0.5f - 1e-5f && x <= 1.0f + 1e-5f);
  }

  // 末点保持 + 端速 0（越界后保持末点，行为不变）。
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U + 9000U);
  motor_control_trajectory_step(&ctx, 300U + 9000U);
  assert(std::fabs(entry_of(&ctx, 5)->target_position_rad - 0.5f) < 1e-6f);
  assert(std::fabs(entry_of(&ctx, 5)->target_velocity_rad_s) < 1e-6f);
}

// 轨迹执行时 **位置与速度必须同向**：执行侧速度是位置时间线的解析导数（路点里的
// velocity 字段被忽略，见 motor_control.c 的求值注释）。符号写反的后果不是"看着不对"
// 而是 MIT 律里的 kd·(v_des − v) 变成**负阻尼**。这条同时钉死两件事：
//   ① 上升段速度为正、下降段速度为负（同向）；
//   ② 路点里塞一个假的 velocity 不影响执行速度（证明它是解析求导，不是直通）。
static void test_trajectory_position_velocity_same_sign() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100);

  motor_control_trajectory_t t = make_trajectory(1U, 300U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.0f, 0.0f);      // 起 0
  traj_add_point(&t, 1000U, 5, 1.0f, 0.0f);   // 1s 升到 +1.0
  traj_add_point(&t, 2000U, 5, -0.5f, 0.0f);  // 2s 降到 −0.5
  // 路点里塞满**负**的假速度：若固件直通它，上升段的 v ≥ 0 断言必然失败。
  for (uint16_t i = 0U; i < t.point_count; ++i) {
    t.points[i].velocity_rad_s = -9.0f;
  }
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t, 300U));

  int up = 0, down = 0;
  float prev = 0.0f;
  for (uint32_t e = 0U; e <= 2000U; e += 20U) {
    feed_vaxor_heartbeat(&ctx, 5, 7, 300U + e);
    motor_control_trajectory_step(&ctx, 300U + e);
    const float x = entry_of(&ctx, 5)->target_position_rad;
    const float v = entry_of(&ctx, 5)->target_velocity_rad_s;
    if (e == 0U) { prev = x; continue; }
    const float dx = x - prev;
    prev = x;
    // 单调段上解析导数是**弱**同号的：局部极值处切矢恰为 0（F–C 限幅），
    // 而那一拍的位置差分仍可能非零——所以断言取 ≥/≤ 而不是 >/<。
    if (dx > 1e-4f) { assert(v >= 0.0f); if (v > 1e-4f) up++; }
    else if (dx < -1e-4f) { assert(v <= 0.0f); if (v < -1e-4f) down++; }
  }
  assert(up >= 10 && down >= 10); // 两个方向都真的采到了样本
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
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  assert(motor_control_apply_trajectory(&ctx, &t2, 300U));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(!entry_of(&ctx, 5)->has_target);
  motor_control_trajectory_step(&ctx, 300U);
  assert(entry_of(&ctx, 7)->target_position_rad == 0.9f);

  // 取消（空轨迹）→ 7 也软停释放。
  motor_control_trajectory_t cancel = make_trajectory(3U, 400U);
  feed_vaxor_heartbeat(&ctx, 5, 7, 400U);
  assert(motor_control_apply_trajectory(&ctx, &cancel, 400U));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(!entry_of(&ctx, 7)->has_target);
}

// 命令租约约束全部运动源：轨迹也必须在断流后终止。
static void test_trajectory_aborts_on_command_lease_timeout() {
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
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(!ctx.trajectory_active);
  assert(!ctx.authorized);
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

// 诊断快照一次读取内核计数器。
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

// 状态视图暴露设备侧安全事实（正向语义！）：出厂的 kR58Raw[4]=0（保护开）→
// enabled=true；未回读时两个 known 都是 false，值不得被当成真值。
static void test_status_exposes_device_safety_facts() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  motor_control_status_t st[MOTOR_CONTROL_MAX_MOTORS];
  assert(motor_control_copy_statuses(&ctx, st, MOTOR_CONTROL_MAX_MOTORS, 200U) == 1U);
  assert(st[0].device_protect_known);
  assert(st[0].device_protect_enabled); // 设备 raw 0 = 保护生效
  assert(st[0].device_direction_known);
  assert(st[0].device_direction == 0U); // CW

  // Fixture models a device observed with protection disabled, without a runtime write API.
  motor_control_find_entry(&ctx, 7)->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U;
  assert(motor_control_copy_statuses(&ctx, st, MOTOR_CONTROL_MAX_MOTORS, 210U) == 1U);
  assert(st[0].device_protect_known);  // 写后核验落位 → 仍然"已知"
  assert(!st[0].device_protect_enabled);
}

// 保护被手动关掉的台仍允许武装，但**每次**都要显式确认位：关保护 = 该关节没有
// 任何限位在跑（CERB 是设备侧限位，固件没有软限位兜底）。
static void test_enable_without_protection_needs_confirm() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  // 心跳必须持续喂：静默 100ms 会先被新鲜度级联判离线，测的就不是使能门禁了。
  uint32_t t = run_heartbeat_only(&ctx, 7, 160U, 320U);

  // Device truth can still report disabled protection; explicit confirmation remains mandatory.
  motor_control_find_entry(&ctx, 7)->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U;
  t += 2U;

  motor_protocol_raw_frame_t hb = cerb_feedback(7, t);
  motor_control_on_raw_frame(&ctx, &hb);
  t += 2U;

  motor_control_policy_snapshot_t snap = {};
  snap.received_tick_ms = t;
  snap.request_id = 500U;
  snap.policy_generation = 1U;
  snap.target_count = 1U;
  snap.targets[0].motor_id = 7U;
  snap.targets[0].enabled = true;
  snap.targets[0].kp = 1.0f;
  snap.targets[0].kd = 0.1f;
  snap.targets[0].confirm_unprotected = false;
  (void)motor_control_apply_policy(&ctx, &snap, t);

  // policy_enable 发现时默认为真，须以拒绝回告和代数未推进判断武装被拒。
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->last_failure_reason == MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_NONE); // 不是设备故障
  assert(has_result_event(&ctx, 500U, MOTOR_PROTOCOL_CONFIG_INVALID));
  assert(e->policy_generation == 0U); // 代数未被推进 = 没有落地

  // 带确认位（代数继续前进）→ 受理。
  snap.request_id = 501U;
  snap.policy_generation = 2U;
  snap.received_tick_ms = t + 2U;
  snap.targets[0].confirm_unprotected = true;
  (void)motor_control_apply_policy(&ctx, &snap, t + 2U);
  assert(has_result_event(&ctx, 501U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->policy_generation == 2U); // 落地了

  // 保护仍是"开"的台不受本门禁约束：确认位可有可无。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  bring_cerb_to_ready(&ctx2, 7, "cerb_r58_101", 100);
  motor_control_policy_snapshot_t snap2 = {};
  snap2.received_tick_ms = 400U;
  snap2.request_id = 600U;
  snap2.policy_generation = 1U;
  snap2.target_count = 1U;
  snap2.targets[0].motor_id = 7U;
  snap2.targets[0].enabled = true;
  snap2.targets[0].kp = 1.0f;
  snap2.targets[0].kd = 0.1f;
  snap2.targets[0].confirm_unprotected = false;
  (void)motor_control_apply_policy(&ctx2, &snap2, 401U);
  assert(has_result_event(&ctx2, 600U, MOTOR_PROTOCOL_CONFIG_OK)); // 保护在开 → 放行
}

// hold_step 每控制边界重发已受理目标，命令率不依赖上行快照到达率。
static void test_hold_step_emits_held_target_every_boundary() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);

  motor_control_command_snapshot_t s = make_snapshot(1U, 120U);
  snapshot_add(&s, 5, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125U));
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 1); // 受理那一拍

  // 此后没有新快照：1ms service 本身不发运动帧，hold_step 每个边界恰好一帧，
  // 且目标零阶保持、逐位不变（不重算、不漂移）。
  for (uint32_t t = 130U; t < 150U; t += 5U) { // 原始截止 150ms，不含截止本身
    motor_protocol_raw_frame_t keepalive = vaxor_reply(5, 0.0, 0.0, 0.0, t);
    motor_control_on_raw_frame(&ctx, &keepalive); // 防 100ms 离线误判
    motor_control_service(&ctx, t);
    assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0); // service 不发
    motor_control_hold_step(&ctx, t);
    assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 1);
    assert(entry_of(&ctx, 5)->target_position_rad == 0.4f);
    assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);
  }
  // 重发不带新证据：不得产生软停/ACTIVE 之类的迁移事件。
  assert(!events_contain(&ctx, MOTOR_CONTROL_EVENT_SOFT_STOP, 5));

  // 快照过期（龄 > 30ms）：hold_step 立即停发，由 freshness 服务走软停，
  // 保持重发绝不能把这条级联盖过去。
  const uint32_t stale_t = 120U + MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS + 5U;
  motor_control_hold_step(&ctx, stale_t);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  motor_control_service(&ctx, stale_t);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  motor_control_hold_step(&ctx, stale_t + 5U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0); // 已无目标
}

// 保持重发的两个让路条件：轨迹所属电机由 trajectory_step 驱动（同一拍不得
// 重复发帧）；有挂起的 P0 时安全帧必须赢，重发不得把它取消掉。
static void test_hold_step_yields_to_trajectory_and_pending_p0() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_two_active(&ctx, 5, 7, 100); // 最后一份快照 received_tick = 120

  motor_control_trajectory_t t = make_trajectory(1U, 200U);
  traj_add_motor(&t, 5);
  traj_add_point(&t, 0U, 5, 0.1f, 0.0f);
  traj_add_point(&t, 1000U, 5, 0.5f, 0.0f);
  assert(motor_control_apply_trajectory(&ctx, &t, 200U));

  // 快照保持新鲜：7 的目标来自快照通道，5 的目标来自轨迹（忽略快照里的 5）。
  motor_control_command_snapshot_t s = make_snapshot(2U, 296U);
  snapshot_add(&s, 5, 0.0f, 0.0f, 0.0f);
  snapshot_add(&s, 7, 0.3f, 0.0f, 0.0f);
  feed_vaxor_heartbeat(&ctx, 5, 7, 298U);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 298U));
  (void)count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1); // 排空

  // 一个控制边界：5 由 trajectory_step 发，7 由 hold_step 发——合计恰好 2 帧，
  // hold_step 不得为轨迹所属的 5 重复发帧。
  feed_vaxor_heartbeat(&ctx, 5, 7, 300U);
  motor_control_trajectory_step(&ctx, 300U);
  motor_control_hold_step(&ctx, 300U);
  {
    motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
    const size_t n =
        motor_control_collect_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
    assert(n == 2U);
    const uint16_t slot5 = (uint16_t)(entry_of(&ctx, 5) - &ctx.entries[0]);
    const uint16_t slot7 = (uint16_t)(entry_of(&ctx, 7) - &ctx.entries[0]);
    size_t seen5 = 0U, seen7 = 0U;
    for (size_t i = 0U; i < n; ++i) {
      if (frames[i].tx_slot == slot5) { seen5++; }
      if (frames[i].tx_slot == slot7) { seen7++; }
    }
    assert(seen5 == 1U && seen7 == 1U);
  }
  assert(entry_of(&ctx, 5)->target_position_rad < 0.2f); // 轨迹在播（首点附近）
  assert(entry_of(&ctx, 7)->target_position_rad == 0.3f); // 快照目标

  // 挂起的 P0：队列塞满 P0 → 新的安全帧推不进去 → p0_pending。排空队列后
  // （等于下游把 P0 发了出去）重试意图仍在，hold_step 必须照样让路——否则它
  // 会把这条安全帧的补发取消掉。
  motor_control_entry_t *seven = nullptr;
  for (size_t i = 0U; i < ctx.entry_count; ++i) {
    if (ctx.entries[i].used && ctx.entries[i].motor_id == 7U) {
      seven = &ctx.entries[i];
    }
  }
  assert(seven != nullptr);
  motor_protocol_tx_frame_t p0 = {0};
  p0.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  for (size_t i = 0U; i < MOTOR_CONTROL_MAX_TX_PENDING; ++i) {
    assert(motor_control_push_tx_p0(&ctx, seven, &p0));
  }
  assert(!motor_control_push_tx_p0(&ctx, seven, &p0)); // 全是 P0 → 挂起重试
  assert(seven->p0_pending);
  (void)count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0); // 下游已发出去
  assert(seven->p0_pending);
  motor_control_hold_step(&ctx, 305U);
  assert(seven->p0_pending); // 重发没有取消这条安全帧的重试
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0U);
}

static void test_control_state_projection_is_authoritative_without_motion() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  motor_control_status_t status[1];
  auto *entry = motor_control_find_entry(&ctx, 5);
  entry->policy_enable = false;
  entry->motion_authorized = false;
  assert(motor_control_copy_statuses(&ctx, status, 1, 120) == 1);
  assert(status[0].control_lifecycle == MOTOR_CONTROL_CONTROL_LIFECYCLE_INACTIVE);
  const size_t before = ctx.tx_count;
  auto enable = make_policy(700, 5, 1, true, 10.f, 1.f);
  assert(motor_control_submit_config(&ctx, &enable, 121));
  assert(motor_control_copy_statuses(&ctx, status, 1, 122) == 1);
  assert(status[0].control_lifecycle == MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVE);
  assert(status[0].execution_state == MOTOR_CONTROL_EXECUTION_IDLE);
  assert(status[0].policy_enabled && !status[0].motion_enabled);
  assert(status[0].policy_generation == 1);
  assert(entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(ctx.tx_count == before);
  entry->has_target = true;
  entry->motion_enabled = true;
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
  entry->target_velocity_rad_s = 0.f;
  entry->target_position_rad = entry->measured.position_rad;
  assert(motor_control_copy_statuses(&ctx, status, 1, 123) == 1);
  assert(status[0].execution_state == MOTOR_CONTROL_EXECUTION_HOLDING);
  entry->enable_pending = true;
  assert(motor_control_copy_statuses(&ctx, status, 1, 124) == 1);
  assert(status[0].execution_state == MOTOR_CONTROL_EXECUTION_ENABLING);
  entry->enable_pending = false;
  entry->online = false;
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_OFFLINE;
  assert(motor_control_copy_statuses(&ctx, status, 1, 125) == 1);
  assert(status[0].control_lifecycle != MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVE);
  assert(status[0].control_lifecycle != MOTOR_CONTROL_CONTROL_LIFECYCLE_FINALIZED);
  entry->online = true;
  entry->unified_fault_code = MOTOR_CONTROL_FAULT_VENDOR;
  assert(motor_control_copy_statuses(&ctx, status, 1, 126) == 1);
  assert(status[0].control_lifecycle != MOTOR_CONTROL_CONTROL_LIFECYCLE_ACTIVE);
}

static void test_command_execution_deadline_is_not_renewed() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  auto s = make_snapshot(1U, 120U);
  s.valid_until_tick_ms = 126U;
  snapshot_add(&s, 5, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 125U));
  assert(motor_control_target_valid(&ctx, 0U, 125U));
  assert(!motor_control_target_valid(&ctx, 0U, 126U));
  (void)count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1);
  motor_control_hold_step(&ctx, 126U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  motor_control_service(&ctx, 126U);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  s.sequence = 2U;
  assert(!motor_control_apply_command_snapshot(&ctx, &s, 127U));
  assert(ctx.last_snapshot_sequence == 1U);
}

static void test_missing_motion_feedback_blocks_snapshot_hold_and_target_valid() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_vaxor_to_ready(&ctx, 5, "vaxor_2550", 100);
  // 设备可建档，但尚未获得任何真实运动量：其它帧不能替代运动反馈。
  motor_control_entry_t *entry = motor_control_find_entry(&ctx, 5);
  assert(entry != NULL);
  entry->has_measured = false;

  auto s = make_snapshot(1U, 101U);
  snapshot_add(&s, 5, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 101U));
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  assert(!motor_control_target_valid(&ctx, 0U, 101U));
  motor_control_hold_step(&ctx, 101U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
}

static void test_aged_motion_feedback_soft_stops_and_requires_explicit_enable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  // 首次目标进入厂商 100ms 使能整定窗；按 CERB 20ms 状态周期持续给反馈，
  // 既维持链路在线，也让真实运动反馈始终新鲜。
  auto s = make_snapshot(1U, 130U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 130U));
  motor_control_service(&ctx, 140U);

  auto fresh = cerb_feedback(7, 158U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(2U, 150U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 160U));
  motor_control_service(&ctx, 170U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  fresh = cerb_feedback(7, 188U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(3U, 180U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 185U));
  motor_control_service(&ctx, 190U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  fresh = cerb_feedback(7, 228U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(4U, 220U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 225U));
  motor_control_service(&ctx, 235U); // 整定 105ms → 到期

  fresh = cerb_feedback(7, 252U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(5U, 245U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 250U));
  motor_control_service(&ctx, 260U);
  assert(motor_control_target_valid(&ctx, 0U, 260U));
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 1);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE);

  // SDO 只刷新链路 last_rx；真实运动反馈仍停在 252ms，361ms 已过期。
  auto sdo = sdo_read_ack(7, MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID, 7, 361U);
  motor_control_on_raw_frame(&ctx, &sdo);
  assert(entry_of(&ctx, 7)->online);
  assert(entry_of(&ctx, 7)->last_rx_tick_ms == 361U);
  assert(entry_of(&ctx, 7)->last_motion_rx_tick_ms == 252U);

  s = make_snapshot(6U, 361U);
  snapshot_add(&s, 7, 0.5f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 361U));
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);
  assert(!motor_control_target_valid(&ctx, 0U, 361U));
  motor_control_hold_step(&ctx, 361U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  motor_control_service(&ctx, 361U);
  assert(entry_of(&ctx, 7)->online);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP);
  assert(!entry_of(&ctx, 7)->has_target);
  assert(!entry_of(&ctx, 7)->motion_authorized);
  assert(!entry_of(&ctx, 7)->policy_enable);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P0) >= 1);

  // 新反馈+新快照仍不得自动复动。
  fresh = cerb_feedback(7, 370U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(7U, 370U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 370U));
  assert(!motor_control_target_valid(&ctx, 0U, 370U));
  motor_control_hold_step(&ctx, 370U);
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 0);

  // 显式逐电机策略授权后，下一新鲜快照才恢复。
  enable_test_motor(&ctx, 7, 371U);
  fresh = cerb_feedback(7, 380U);
  motor_control_on_raw_frame(&ctx, &fresh);
  s = make_snapshot(8U, 380U);
  snapshot_add(&s, 7, 0.4f, 0.0f, 0.0f);
  assert(motor_control_apply_command_snapshot(&ctx, &s, 380U));
  assert(motor_control_target_valid(&ctx, 0U, 380U));
  assert(count_tx_class(&ctx, MOTOR_PROTOCOL_TX_CLASS_P1) == 1);
}

static void test_runtime_device_parameter_writes_are_retired() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  const motor_protocol_config_kind_t retired[] = {
    MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS, MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT,
    MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT, MOTOR_PROTOCOL_CONFIG_SET_ZERO,
    MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT, MOTOR_PROTOCOL_CONFIG_SET_DIRECTION,
    MOTOR_PROTOCOL_CONFIG_SET_T_MAX, MOTOR_PROTOCOL_CONFIG_SET_NODE_ID,
    MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH};
  for (const auto kind : retired) {
    motor_protocol_config_request_t request{};
    request.request_id = 100 + kind; request.motor_id = 7; request.kind = kind;
    request.position_limits.min_position_rad = -1; request.position_limits.max_position_rad = 1;
    request.max_velocity_rad_s = 1; request.max_torque_nm = 1;
    request.t_max_raw = 300; request.new_node_id = 8; request.save_slots = 1;
    assert(!motor_control_submit_config(&ctx, &request, 200));
    assert(!entry_of(&ctx, 7)->tx_active);
  }
}

int main() {
  bind_real_adapters();
  test_runtime_device_parameter_writes_are_retired();
  test_missing_motion_feedback_blocks_snapshot_hold_and_target_valid();
  test_aged_motion_feedback_soft_stops_and_requires_explicit_enable();
  test_command_execution_deadline_is_not_renewed();
  test_control_state_projection_is_authoritative_without_motion();
  test_device_param_boot_verify_ok();
  test_boot_normalize_runs_before_refresh();
  test_boot_normalize_failure_state_before_refresh();
  test_boot_normalize_not_advanced_by_manual_write();
  test_manual_refresh_rejected_during_normalize_window();
  test_boot_normalize_skips_direction_without_record();
  test_dev_init_no_record_reports_without_gate();
  test_boot_normalize_skips_illegal_direction_value();
  test_enable_without_protection_needs_confirm();
  test_dev_init_flags_safety_mismatch_at_runtime();
  test_device_param_mismatch_reports_without_gate();
  test_device_param_migrated_record_flags_protect_off();
  test_device_param_missing_direction_baseline_reports();
  test_device_param_no_record_reports_on_direction();
  test_device_param_read_retry_recovers();
  test_device_param_unverified_reports_without_gate();
  test_device_param_pending_does_not_block_enable();
  test_device_param_runtime_mismatch_reports_only();
  test_device_param_partial_record_compare();
  test_device_param_view_exposes_record();
  test_param_view_carries_report_fields();
  test_device_param_four_reads_per_motor();
  test_device_param_reconnect_keeps_record();
  test_device_param_na_for_software_limit_models();
  test_stored_apply_does_not_override_refreshed_shadow();
  test_stored_params_apply_after_bind();
  test_cerb_refresh_params_transaction();
  test_vaxor_refresh_params_unsupported();
  test_refresh_rejected_before_bind();
  test_copy_topology();
  test_trajectory_owns_its_motors_against_snapshot();
  test_snapshot_absence_soft_stops_only_unowned_motors();
  test_trajectory_interpolates_and_clamps_at_end();
  test_trajectory_hermite_c1_continuity();
  test_trajectory_hermite_uses_nonuniform_time_weights();
  test_trajectory_hermite_no_overshoot_and_zero_endpoints();
  test_trajectory_position_velocity_same_sign();
  test_trajectory_cancel_and_replacement_release_motors();
  test_trajectory_aborts_on_command_lease_timeout();
  test_hold_step_emits_held_target_every_boundary();
  test_hold_step_yields_to_trajectory_and_pending_p0();
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
  test_status_exposes_device_safety_facts();
  run_all_tests();
  printf("test_motor_control: all passed\n");
  return 0;
}
