#include "control_mailbox.h"

#include <cassert>
#include <cmath>
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
  assert(mailbox.config_rejected_count == 1U);

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

  invalid = make_mode_request(3U, 1U, 0U);
  invalid.kind = static_cast<control_mailbox_config_kind_t>(99U);
  assert(!control_mailbox_submit_config(&mailbox, &invalid));
  assert(mailbox.config_rejected_count == 4U);
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

int main() {
  test_command_snapshot_is_complete_and_latest_wins();
  test_command_snapshot_accepts_64_targets_and_rejects_invalid_data();
  test_config_is_neutral_bounded_fifo();
  test_config_validation_covers_semantic_shape_only();
  test_authorization_and_agent_edges_are_idempotent();
  return 0;
}
