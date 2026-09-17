#include "control_mailbox.h"
#include "motor_protocol_adapter.h"

#include <cassert>

// 配置 kind 枚举契约：mailbox 与内核协议层枚举逐值相等
// （1..6、8、9，7 为内核专用不经 mailbox）——固件侧 motor_rt_task.cpp
// 同款 static_assert，host 侧双链锁死，任一侧改动枚举值即编译失败。
static_assert(static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_MODE) ==
                  static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_MODE),
              "mailbox/protocol config kind 契约不一致：SET_MODE");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_GAINS) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_GAINS),
    "mailbox/protocol config kind 契约不一致：SET_GAINS");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS),
    "mailbox/protocol config kind 契约不一致：SET_POSITION_LIMITS");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_ENABLE) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_ENABLE),
    "mailbox/protocol config kind 契约不一致：SET_ENABLE");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT),
    "mailbox/protocol config kind 契约不一致：SET_VELOCITY_LIMIT");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT),
    "mailbox/protocol config kind 契约不一致：SET_TORQUE_LIMIT");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS),
    "mailbox/protocol config kind 契约不一致：REFRESH_PARAMS");

#include <cmath>
#include <cstring>
#include <limits>

static control_command_snapshot_t make_snapshot(uint32_t sequence,
                                                uint32_t received_tick_ms,
                                                uint16_t motor_id,
                                                float position) {
  control_command_snapshot_t snapshot{};
  snapshot.sequence = sequence;
  snapshot.received_tick_ms = received_tick_ms;
  snapshot.target_count = 1U;
  snapshot.targets[0] = {motor_id, position, 0.0F, 0.0F};
  return snapshot;
}

static control_mailbox_config_request_t make_mode_request(uint32_t request_id,
                                                          uint16_t motor_id,
                                                          uint8_t mode) {
  control_mailbox_config_request_t request{};
  request.request_id = request_id;
  request.received_tick_ms = request_id * 10U;
  request.motor_id = motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_MODE;
  request.value.mode = mode;
  return request;
}

static void test_command_snapshot_is_complete_and_latest_wins() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  control_command_snapshot_t first =
      make_snapshot(7U, 100U, 42U, 1.5F);
  assert(control_mailbox_submit_command(&mailbox, &first));
  first.targets[0].position = 99.0F;

  control_command_snapshot_t out{};
  assert(control_mailbox_take_command(&mailbox, &out));
  assert(out.sequence == 7U);
  assert(out.received_tick_ms == 100U);
  assert(out.target_count == 1U);
  assert(out.targets[0].motor_id == 42U);
  assert(out.targets[0].position == 1.5F);
  assert(!control_mailbox_take_command(&mailbox, &out));

  const control_command_snapshot_t second =
      make_snapshot(8U, 110U, 43U, 2.5F);
  const control_command_snapshot_t third =
      make_snapshot(9U, 120U, 44U, 3.5F);
  assert(control_mailbox_submit_command(&mailbox, &second));
  assert(control_mailbox_submit_command(&mailbox, &third));
  assert(mailbox.command.overwrite_count == 1U);
  assert(control_mailbox_take_command(&mailbox, &out));
  assert(out.sequence == 9U);
  assert(out.targets[0].motor_id == 44U);
}

static void test_command_snapshot_accepts_64_targets_and_rejects_invalid_data() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  control_command_snapshot_t snapshot{};
  snapshot.sequence = 10U;
  snapshot.received_tick_ms = 200U;
  snapshot.target_count = CONTROL_COMMAND_MAILBOX_MAX_TARGETS;
  for (uint16_t index = 0U; index < snapshot.target_count; ++index) {
    snapshot.targets[index] = {static_cast<uint16_t>(index + 1U),
                               static_cast<float>(index), 0.0F, 0.0F};
  }
  assert(control_mailbox_submit_command(&mailbox, &snapshot));

  control_command_snapshot_t invalid = snapshot;
  invalid.target_count = CONTROL_COMMAND_MAILBOX_MAX_TARGETS + 1U;
  assert(!control_mailbox_submit_command(&mailbox, &invalid));

  invalid = snapshot;
  invalid.target_count = 2U;
  invalid.targets[1].motor_id = invalid.targets[0].motor_id;
  assert(!control_mailbox_submit_command(&mailbox, &invalid));

  invalid = snapshot;
  invalid.target_count = 1U;
  invalid.targets[0].position = std::numeric_limits<float>::quiet_NaN();
  assert(!control_mailbox_submit_command(&mailbox, &invalid));

  control_command_snapshot_t out{};
  assert(control_mailbox_take_command(&mailbox, &out));
  assert(out.target_count == CONTROL_COMMAND_MAILBOX_MAX_TARGETS);
  assert(out.targets[63].motor_id == 64U);
  assert(mailbox.command.invalid_snapshot_count == 3U);
}

static void test_config_is_neutral_bounded_fifo() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  for (uint32_t index = 0U;
       index < CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY; ++index) {
    const control_mailbox_config_request_t request =
        make_mode_request(index + 1U, static_cast<uint16_t>(index + 10U),
                          static_cast<uint8_t>(index));
    assert(control_mailbox_submit_config(&mailbox, &request));
  }
  const control_mailbox_config_request_t full_request =
      make_mode_request(99U, 99U, 1U);
  assert(!control_mailbox_submit_config(&mailbox, &full_request));
  assert(mailbox.config_count == CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY);
  assert(mailbox.config_overflow_count == 1U); // 满：消费慢于生产
  assert(mailbox.config_invalid_count == 0U);

  for (uint32_t index = 0U;
       index < CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY; ++index) {
    control_mailbox_config_request_t out{};
    assert(control_mailbox_take_config(&mailbox, &out));
    assert(out.request_id == index + 1U);
    assert(out.motor_id == index + 10U);
    assert(out.value.mode == index);
  }
  control_mailbox_config_request_t out{};
  assert(!control_mailbox_take_config(&mailbox, &out));

  const control_mailbox_config_request_t recycled =
      make_mode_request(100U, 100U, 2U);
  assert(control_mailbox_submit_config(&mailbox, &recycled));
  assert(control_mailbox_take_config(&mailbox, &out));
  assert(out.request_id == 100U);
}

static void test_config_validation_covers_semantic_shape_only() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  control_mailbox_config_request_t invalid = make_mode_request(0U, 1U, 0U);
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  invalid = make_mode_request(1U, 1U, 0U);
  invalid.kind = CONTROL_MAILBOX_CONFIG_SET_GAINS;
  invalid.value.gains = {std::numeric_limits<float>::quiet_NaN(), 1.0F};
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  invalid = make_mode_request(2U, 1U, 0U);
  invalid.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
  invalid.value.position_limits = {1.0F, 1.0F};
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  // 速度/力矩限值：NaN、非正一律拒（设备侧管理对象，仅 CERB）
  invalid = make_mode_request(3U, 1U, 0U);
  invalid.kind = CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
  invalid.value.max_velocity_rad_s = std::numeric_limits<float>::quiet_NaN();
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  invalid = make_mode_request(4U, 1U, 0U);
  invalid.kind = CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
  invalid.value.max_velocity_rad_s = 0.0F;
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  invalid = make_mode_request(5U, 1U, 0U);
  invalid.kind = CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT;
  invalid.value.max_torque_nm = -1.0F;
  assert(!control_mailbox_submit_config(&mailbox, &invalid));

  invalid = make_mode_request(6U, 1U, 0U);
  invalid.kind = static_cast<control_mailbox_config_kind_t>(99U);
  assert(!control_mailbox_submit_config(&mailbox, &invalid));
  assert(mailbox.config_invalid_count == 7U); // 非法请求单独可辨

  // 合法限值入队（正数、有限）
  control_mailbox_config_request_t good = make_mode_request(7U, 1U, 0U);
  good.kind = CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
  good.value.max_velocity_rad_s = 3.0F;
  assert(control_mailbox_submit_config(&mailbox, &good));
  good = make_mode_request(8U, 1U, 0U);
  good.kind = CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT;
  good.value.max_torque_nm = 20.0F;
  assert(control_mailbox_submit_config(&mailbox, &good));
  assert(mailbox.config_invalid_count == 7U); // 合法请求不计入非法计数
  assert(mailbox.config_overflow_count == 0U);
}

static void test_authorization_and_agent_edges_are_idempotent() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  const control_command_snapshot_t command =
      make_snapshot(1U, 10U, 1U, 0.1F);
  assert(control_mailbox_submit_command(&mailbox, &command));

  control_mailbox_request_authorization(&mailbox, false);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.authorization_disable_edge);
  assert(events.authorization_pending);
  assert(!events.authorization_requested);
  assert(!control_mailbox_take_events(&mailbox, &events));
  control_command_snapshot_t command_out{};
  assert(!control_mailbox_take_command(&mailbox, nullptr));
  assert(!control_mailbox_take_command(&mailbox, &command_out));
  assert(!control_mailbox_submit_command(&mailbox, &command));

  // 配置不属于运动授权；可以在禁能状态下进入低频事务队列。
  const control_mailbox_config_request_t config_while_disabled =
      make_mode_request(2U, 1U, 0U);
  assert(control_mailbox_submit_config(&mailbox, &config_while_disabled));

  control_mailbox_request_authorization(&mailbox, false);
  assert(!control_mailbox_take_events(&mailbox, &events));
  control_mailbox_request_authorization(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.authorization_pending);
  assert(events.authorization_requested);
  assert(control_mailbox_submit_command(&mailbox, &command));

  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_state_pending);
  assert(!events.agent_disconnect_edge);
  assert(control_mailbox_submit_command(&mailbox, &command));

  const control_mailbox_config_request_t config =
      make_mode_request(4U, 1U, 0U);
  assert(control_mailbox_submit_config(&mailbox, &config));
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_disconnect_edge);
  assert(events.authorization_disable_edge);
  assert(!events.authorization_requested);
  assert(!control_mailbox_take_command(&mailbox, &command_out));
  control_mailbox_config_request_t config_out{};
  assert(!control_mailbox_take_config(&mailbox, &config_out));

  assert(!control_mailbox_take_events(&mailbox, &events));
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(!control_mailbox_take_events(&mailbox, &events));
}

// 新增：授权/会话门拒绝的命令提交必须计数（拒绝必可见）。
static void test_command_gate_rejections_are_counted() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  const control_command_snapshot_t command =
      make_snapshot(1U, 10U, 1U, 0.1F);

  // 引导豁免：Agent 未上报前，授权默认允许 → 可提交。
  assert(control_mailbox_submit_command(&mailbox, &command));
  assert(mailbox.command_rejected_count == 0U);

  control_mailbox_request_authorization(&mailbox, false);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(!control_mailbox_submit_command(&mailbox, &command));
  assert(mailbox.command_rejected_count == 1U); // 授权门拒绝可见

  // 新语义（motor_enable 话题已删）：会话在线即授权——首连自动重立授权。
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_state_pending);
  assert(events.authorization_pending); // 撤权后首连重立授权边沿
  assert(control_mailbox_submit_command(&mailbox, &command));
  assert(mailbox.command_rejected_count == 1U); // 放行不计数

  // 显式置 true 已无变化（幂等）。
  control_mailbox_request_authorization(&mailbox, true);
  assert(!control_mailbox_take_events(&mailbox, &events));
  assert(control_mailbox_submit_command(&mailbox, &command));

  // 会话断开后命令门拒绝（授权由断链撤销）。
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(!control_mailbox_submit_command(&mailbox, &command));
  assert(mailbox.command_rejected_count == 2U);

  // 重连自动重立授权，命令重新放行。
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(control_mailbox_submit_command(&mailbox, &command));
  assert(mailbox.command_rejected_count == 2U);
}

// 新增：会话边沿矩阵（首连/断链/重连/重复状态）与 take_events 返回值语义。
static void test_session_edge_matrix() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  // 引导态：无边沿（返回 false），但仍带货电平。
  control_mailbox_events_t events{};
  assert(!control_mailbox_take_events(&mailbox, &events));
  assert(events.authorization_requested); // 引导豁免电平
  assert(!events.agent_connected_requested);

  // 首次连接：state_pending 边沿，无断链边沿。
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_state_pending);
  assert(events.agent_connected_requested);
  assert(!events.agent_disconnect_edge);

  // 重复同状态：无新边沿。
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(!control_mailbox_take_events(&mailbox, &events));

  // 从未连接就断（不可能走到，防御路径）：仅状态电平，无断链边沿。
  // ——此处 agent 已连接，走"连接后断开"主路径。
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_disconnect_edge);
  assert(events.agent_state_pending);
  assert(!events.agent_connected_requested);
  // 断链撤销授权：电平随边沿一起可见。
  assert(events.authorization_disable_edge);
  assert(!events.authorization_requested);

  // 会话门：断链期（重连前）config 提交被拒且计入会话计数。
  const control_mailbox_config_request_t config =
      make_mode_request(7U, 1U, 0U);
  assert(!control_mailbox_submit_config(&mailbox, &config));
  assert(mailbox.config_session_count == 1U);
  assert(mailbox.config_invalid_count == 0U);
  assert(mailbox.config_overflow_count == 0U);

  // 重连：授权随会话恢复自动重立（motor_enable 已删，授权=会话在线派生）；
  // state_pending + authorization_pending 同时可见，配置重新可排队。
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_take_events(&mailbox, &events));
  assert(events.agent_state_pending);
  assert(events.authorization_pending); // 重连重立授权边沿
  assert(events.authorization_requested);
  assert(control_mailbox_submit_config(&mailbox, &config));
  assert(mailbox.config_session_count == 1U);

  // 引导豁免前只见状态：新 mailbox 上"从未连接就断开"（防御路径）。
  control_mailbox_t fresh{};
  control_mailbox_init(&fresh);
  control_mailbox_request_agent_connected(&fresh, false);
  assert(control_mailbox_take_events(&fresh, &events));
  assert(events.agent_state_pending);
  assert(!events.agent_disconnect_edge);      // 从未连接 → 无断链边沿
  assert(events.authorization_requested);     // 引导豁免未被撤销
}

// SET_ENABLE：逐电机 Enable 经 config FIFO 传递（ARD §19）。
static void test_config_set_enable_queues() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  control_mailbox_config_request_t request{};
  request.request_id = 21U;
  request.motor_id = 5U;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_ENABLE;
  request.value.enable = true;
  assert(control_mailbox_submit_config(&mailbox, &request));
  control_mailbox_config_request_t out{};
  assert(control_mailbox_take_config(&mailbox, &out));
  assert(out.kind == CONTROL_MAILBOX_CONFIG_SET_ENABLE);
  assert(out.value.enable == true);
}

// 配置提交与命令路径同序：会话门先于数据校验——断链期的非法载荷
// 归因到 config_session_count，与 config_invalid_count 互斥不重复。
static void test_config_gate_precedes_validation() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  control_mailbox_request_agent_connected(&mailbox, true);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mailbox, &events));
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(control_mailbox_take_events(&mailbox, &events));

  control_mailbox_config_request_t invalid = make_mode_request(0U, 1U, 0U);
  assert(!control_mailbox_submit_config(&mailbox, &invalid));
  assert(mailbox.config_session_count == 1U); // 会话门优先归因
  assert(mailbox.config_invalid_count == 0U); // 不重复计数
  assert(mailbox.config_overflow_count == 0U);
}


// REFRESH_PARAMS：无载荷请求，只凭 request_id/motor_id 即可入队。
static void test_refresh_params_request_is_accepted() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  control_mailbox_request_agent_connected(&mailbox, true);

  control_mailbox_config_request_t request{};
  request.request_id = 1U;
  request.received_tick_ms = 10U;
  request.motor_id = 3U;
  request.kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
  assert(control_mailbox_submit_config(&mailbox, &request));

  control_mailbox_config_request_t taken{};
  assert(control_mailbox_take_config(&mailbox, &taken));
  assert(taken.kind == CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS);
  assert(taken.motor_id == 3U);
}

// SET_ZERO：同为无载荷请求。**注意**：mailbox 的 config_request_is_valid 是
// 白名单（default: false），新增 kind 必须同时加进去——漏加会让请求在 mailbox
// 就被拒（last_error=11），内核完全收不到、总线上一条帧都没有。
static void test_set_zero_request_is_accepted() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  control_mailbox_request_agent_connected(&mailbox, true);

  control_mailbox_config_request_t request{};
  request.request_id = 7U;
  request.received_tick_ms = 20U;
  request.motor_id = 5U;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_ZERO;
  assert(control_mailbox_submit_config(&mailbox, &request));
  assert(mailbox.config_invalid_count == 0U);

  control_mailbox_config_request_t taken{};
  assert(control_mailbox_take_config(&mailbox, &taken));
  assert(taken.kind == CONTROL_MAILBOX_CONFIG_SET_ZERO);
  assert(taken.motor_id == 5U);
}

int main() {
  test_refresh_params_request_is_accepted();
  test_set_zero_request_is_accepted();
  test_command_snapshot_is_complete_and_latest_wins();
  test_command_snapshot_accepts_64_targets_and_rejects_invalid_data();
  test_config_is_neutral_bounded_fifo();
  test_config_validation_covers_semantic_shape_only();
  test_authorization_and_agent_edges_are_idempotent();
  test_command_gate_rejections_are_counted();
  test_session_edge_matrix();
  test_config_set_enable_queues();
  test_config_gate_precedes_validation();
  return 0;
}
