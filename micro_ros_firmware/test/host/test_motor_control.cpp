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

#include "motor_control.h"
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

// 完整使能链路（vaxor 路径）：反馈 → 绑定 → 连接 → 授权。
static void bring_vaxor_to_ready(motor_control_context_t *ctx, uint16_t motor_id,
                                 const char *profile_name, uint32_t tick) {
  motor_protocol_raw_frame_t f = vaxor_reply(motor_id, 0.0, 0.0, 0.0, tick);
  motor_control_on_raw_frame(ctx, &f);
  size_t idx;
  assert(find_profile(profile_name, &idx));
  assert(motor_control_bind_profile(ctx, motor_id, idx, tick));
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
}

static void bring_cerb_to_ready(motor_control_context_t *ctx, uint16_t motor_id,
                                const char *profile_name, uint32_t tick) {
  motor_protocol_raw_frame_t f = cerb_feedback(motor_id, tick);
  motor_control_on_raw_frame(ctx, &f);
  size_t idx;
  assert(find_profile(profile_name, &idx));
  assert(motor_control_bind_profile(ctx, motor_id, idx, tick));
  motor_control_set_agent_connected(ctx, true, tick);
  assert(motor_control_set_authorization(ctx, true, tick));
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

static bool mock_stage_info(uint8_t stage, const motor_protocol_config_request_t *request,
                            motor_protocol_config_stage_info_t *out) {
  (void)stage;
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
    uint8_t stage, const motor_protocol_config_request_t *request,
    const motor_protocol_raw_frame_t *response) {
  (void)stage;
  (void)request;
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

// 生命周期：帧 → DISCOVERED → 绑定 → READY_DISABLED（ARD §43.1 第 1 条）。
static void test_frame_creates_discovered_then_bind_readies() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(ctx.entry_count == 1);
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_DISCOVERED);

  size_t vaxor_2550;
  assert(find_profile("vaxor_2550", &vaxor_2550));
  assert(motor_control_bind_profile(&ctx, 5, vaxor_2550, 110));
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

// 编译期拓扑 resolver（下行 MotorBind 话题删除后的绑定来源）：建档/重连
// 自动按名绑定；表外电机与 NULL resolver 保持未绑定（安全侧）。
static const char *bench_resolver(uint16_t motor_id) {
  return motor_id == 5 ? "vaxor_2550" : NULL;
}

static void test_static_model_resolver_auto_bind() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_control_set_model_resolver(&ctx, bench_resolver);

  // 建档即绑定：PROFILE_BOUND + READY，收敛 READY_DISABLED（events_contain
  // 是排空式断言，两个事件须在单次排空中同时确认）。
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f);
  assert(entry_of(&ctx, 5) != NULL);
  assert(entry_of(&ctx, 5)->profile != NULL);
  assert(entry_of(&ctx, 5)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  {
    bool bound = false, ready = false;
    motor_control_event_t drained[64];
    size_t n;
    while ((n = motor_control_drain_events(&ctx, drained, 64)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (drained[i].motor_id != 5) {
          continue;
        }
        bound = bound || drained[i].type == MOTOR_CONTROL_EVENT_PROFILE_BOUND;
        ready = ready || drained[i].type == MOTOR_CONTROL_EVENT_READY;
      }
    }
    assert(bound);
    assert(ready);
  }

  // 表外电机：不绑定，profile==NULL（不可控）。
  motor_protocol_raw_frame_t f9 = vaxor_reply(9, 0.0, 0.0, 0.0, 110);
  motor_control_on_raw_frame(&ctx, &f9);
  assert(entry_of(&ctx, 9) != NULL);
  assert(entry_of(&ctx, 9)->profile == NULL);

  // 离线 → 重连：resolver 在重连路径再次按名绑定收敛 READY_DISABLED。
  motor_control_service(&ctx, 300); // 100ms 无帧 → OFFLINE（vaxor 判龄）
  assert(entry_of(&ctx, 5)->lifecycle == MOTOR_CONTROL_LIFECYCLE_OFFLINE);
  motor_protocol_raw_frame_t f2 = vaxor_reply(5, 0.0, 0.0, 0.0, 400);
  motor_control_on_raw_frame(&ctx, &f2);
  assert(entry_of(&ctx, 5)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(events_contain(&ctx, MOTOR_CONTROL_EVENT_PROFILE_BOUND, 5));

  // NULL resolver（host 默认）：恢复显式绑定语义，不自动绑定。
  motor_control_context_t ctx2;
  motor_control_init(&ctx2, g_adapters, 2);
  motor_control_set_model_resolver(&ctx2, NULL);
  motor_protocol_raw_frame_t f5 = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx2, &f5);
  assert(entry_of(&ctx2, 5)->profile == NULL);
  // 新建表项首帧停在 DISCOVERED（识别推进在次帧）；关键是无 profile、未绑定。
  assert(entry_of(&ctx2, 5)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_DISCOVERED);
}

// MotorBind 过渡绑定通道的内核行为（按名绑定）。
static void test_bind_profile_by_name() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f); // 仅发现，未绑定

  // 未知名、空名或未发现设备：拒绝并回 PROFILE_REJECTED。
  assert(!motor_control_bind_profile_by_name(&ctx, 5, "vaxor_80", 110));
  assert(!motor_control_bind_profile_by_name(&ctx, 5, "", 110));
  assert(!motor_control_bind_profile_by_name(&ctx, 99, "vaxor_2550", 110));

  // 命中：绑定 + PROFILE_BOUND + 识别收敛 READY_DISABLED（events_contain
  // 是排空式断言，两个事件须在单次排空中同时确认）。
  assert(motor_control_bind_profile_by_name(&ctx, 5, "vaxor_2550", 120));
  assert(entry_of(&ctx, 5)->profile != NULL);
  assert(entry_of(&ctx, 5)->lifecycle ==
         MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  {
    bool bound = false, ready = false;
    motor_control_event_t drained[64];
    size_t n;
    while ((n = motor_control_drain_events(&ctx, drained, 64)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (drained[i].motor_id != 5) {
          continue;
        }
        bound = bound || drained[i].type == MOTOR_CONTROL_EVENT_PROFILE_BOUND;
        ready = ready || drained[i].type == MOTOR_CONTROL_EVENT_READY;
      }
    }
    assert(bound);
    assert(ready);
  }
}

// 绑定拒绝必须回 PROFILE_REJECTED 事件（MotorBind.msg 契约：结果经
// PROFILE_BOUND/PROFILE_REJECTED 回告）——未知名与设备未发现曾静默 false。
static void test_bind_rejection_emits_profile_rejected() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  motor_protocol_raw_frame_t f = vaxor_reply(5, 0.0, 0.0, 0.0, 100);
  motor_control_on_raw_frame(&ctx, &f); // 设备 5 已发现（bus 0）
  motor_control_event_t clear_buf[64];
  (void)motor_control_drain_events(&ctx, clear_buf, 64); // 清空引导 DISCOVERED 事件

  // (a) 已发现 + 未知名 → 事件 reason=3（PROFILE_UNAVAILABLE），bus=表项总线。
  assert(!motor_control_bind_profile_by_name(&ctx, 5, "vaxor_80", 110));
  motor_control_event_t ev = {};
  assert(motor_control_drain_events(&ctx, &ev, 1) == 1);
  assert(ev.type == MOTOR_CONTROL_EVENT_PROFILE_REJECTED);
  assert(ev.motor_id == 5);
  assert(ev.bus_index == 0);
  assert(ev.reason == MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE);

  // (b) 空名防御路径 → 同样回告。
  assert(!motor_control_bind_profile_by_name(&ctx, 5, "", 120));
  assert(motor_control_drain_events(&ctx, &ev, 1) == 1);
  assert(ev.type == MOTOR_CONTROL_EVENT_PROFILE_REJECTED);

  // (c) 未发现 id=99 + 已知名 → 事件 bus=0。
  assert(!motor_control_bind_profile_by_name(&ctx, 99, "vaxor_2550", 130));
  assert(motor_control_drain_events(&ctx, &ev, 1) == 1);
  assert(ev.type == MOTOR_CONTROL_EVENT_PROFILE_REJECTED);
  assert(ev.motor_id == 99);
  assert(ev.bus_index == 0);
  assert(ev.reason == MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE);

  // (d) 正常绑定不受影响 → PROFILE_BOUND。
  size_t idx;
  assert(find_profile("vaxor_2550", &idx));
  assert(motor_control_bind_profile(&ctx, 5, idx, 140));
  assert(motor_control_drain_events(&ctx, &ev, 1) == 1);
  assert(ev.type == MOTOR_CONTROL_EVENT_PROFILE_BOUND);
  assert(ev.motor_id == 5);
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
  motor_control_service(&ctx, 706); // 识别流推进（Profile 保留 → READY）
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
  motor_control_service(&ctx, 302); // 识别流推进（Profile 保留 → READY）
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

// CERB 限位写：厂家对象未决（TBD-CONFIG-01）→ UNSUPPORTED，不影响运动授权。
static void test_cerb_limits_unsupported() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
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

  motor_protocol_config_request_t limits = {};
  limits.request_id = 9U;
  limits.motor_id = 7;
  limits.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  limits.position_limits.min_position_rad = -1.0f;
  limits.position_limits.max_position_rad = 1.0f;
  assert(motor_control_submit_config(&ctx, &limits, 280));
  motor_control_service(&ctx, 290);
  assert(has_result_event(&ctx, 9U, MOTOR_PROTOCOL_CONFIG_UNSUPPORTED));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE); // 授权不变
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
  // 40 台发现 → 40 条 DISCOVERED 事件 > 环容量 32 → 丢弃 8。
  for (uint16_t node = 1; node <= 40; ++node) {
    motor_protocol_raw_frame_t f = vaxor_reply(node, 0.0, 0.0, 0.0, node);
    motor_control_on_raw_frame(&ctx, &f);
  }
  motor_control_event_t events[64];
  size_t total = 0;
  size_t n;
  while ((n = motor_control_drain_events(&ctx, events, 64)) > 0) {
    total += n;
  }
  assert(total == MOTOR_CONTROL_MAX_EVENTS);
  assert(ctx.event_dropped_count == 8U);
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
  for (uint32_t t = 130; t < 130 + 120 * 10; t += 10) {
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
  test_bind_profile_by_name();
  test_bind_rejection_emits_profile_rejected();
  test_static_model_resolver_auto_bind();
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
  test_cerb_limits_unsupported();
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

int main() {
  bind_real_adapters();
  run_all_tests();
  printf("test_motor_control: all passed\n");
  return 0;
}
