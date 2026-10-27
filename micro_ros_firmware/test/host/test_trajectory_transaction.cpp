#include "trajectory_transaction.hpp"
#include "motor_control_internal.h"

#include <cassert>
#include <cstdio>

using trajectory_transaction::Coordinator;
using namespace trajectory_transaction;

static agent_trajectory_request_t load_request(uint32_t id = 1U,
    uint32_t trajectory_id = 1U) {
  agent_trajectory_request_t r{};
  r.session_id = 9U;
  r.request_id = id;
  r.trajectory_id = trajectory_id;
  r.valid_until_tick_ms = 1000U;
  r.operation = AGENT_TRAJECTORY_LOAD;
  r.segment_count = 1U;
  r.target_count = 1U;
  r.target_ids[0] = 1U;
  r.point_count = 2U;
  r.points[0] = {0U, 1U, 0.0f, 0.0f, 0.0f};
  r.points[1] = {100U, 1U, 1.0f, 0.0f, 0.0f};
  return r;
}

static agent_trajectory_request_t operation(uint32_t request_id,
    uint8_t op, uint32_t trajectory_id = 1U) {
  agent_trajectory_request_t r{};
  r.session_id = 9U;
  r.request_id = request_id;
  r.trajectory_id = trajectory_id;
  r.operation = op;
  r.valid_until_tick_ms = 1000U;
  return r;
}

static void test_load_ready_dedup_and_timeout() {
  Coordinator c;
  c.begin_session(9U);
  auto r = load_request();
  assert(c.load(r, 0U).state == AGENT_TRAJECTORY_ACCEPTED);
  assert(c.status().state == AGENT_TRAJECTORY_READY);
  assert(c.status().target_count == 1U && c.status().target_ids[0] == 1U);
  assert(c.load(r, 1U).segments_received == 1U);
  r.points[1].position = 2.0f;
  assert(c.load(r, 1U).reason == REASON_CONFLICT);
  c.poll(1000U);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);
  r = load_request();
  assert(c.load(r, 1001U).state == AGENT_TRAJECTORY_ACCEPTED);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED); // 重发不得重新组装
  assert(c.ack(9U, 1U, 1U));
  assert(c.status().target_count == 0U && c.status().target_ids[0] == 0U);
  assert(c.load(r, 1001U).state == AGENT_TRAJECTORY_RETIRED);
}

static void test_segments_capacity_and_timeline() {
  Coordinator c;
  c.begin_session(9U);
  auto r = load_request();
  r.segment_count = 8U;
  r.point_count = 32U;
  for (uint16_t s = 0; s < 8U; ++s) {
    r.request_id = s + 1U;
    r.segment_index = s;
    for (uint16_t p = 0; p < 32U; ++p) {
      r.points[p] = {static_cast<uint32_t>(s * 32U + p), 1U,
          static_cast<float>(p), 0.0f, 0.0f};
    }
    assert(c.load(r, s).state == AGENT_TRAJECTORY_ACCEPTED);
  }
  assert(c.status().state == AGENT_TRAJECTORY_READY);
  assert(c.assembly().point_count == 256U);
  assert(c.status().segments_received == 8U);

  c.begin_session(9U);
  r = load_request();
  r.segment_count = 2U;
  assert(c.load(r, 0U).state == AGENT_TRAJECTORY_ACCEPTED);
  r.request_id = 2U;
  assert(c.load(r, 1U).reason == REASON_ORDER);
  r.segment_index = 1U;
  assert(c.load(r, 1U).reason == REASON_TIMELINE);
  c.poll(1000U);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);

  c.begin_session(9U);
  r = load_request();
  r.target_count = 2U;
  r.target_ids[1] = 1U;
  assert(c.load(r, 0U).state == AGENT_TRAJECTORY_REJECTED);
  assert(c.status().request_high_water == 0U);
}

static void test_commit_cancel_and_retirement() {
  Coordinator c;
  c.begin_session(9U);
  auto r = load_request();
  assert(c.load(r, 0U).state == AGENT_TRAJECTORY_ACCEPTED);
  auto commit = operation(2U, AGENT_TRAJECTORY_COMMIT);
  assert(c.commit(commit, 0U).state == AGENT_TRAJECTORY_PENDING);
  assert(c.status().state == AGENT_TRAJECTORY_READY);
  auto next = load_request(3U, 2U);
  assert(c.load(next, 1U).reason == REASON_BUSY);
  auto cancel = operation(3U, AGENT_TRAJECTORY_CANCEL);
  assert(c.cancel(cancel).state == AGENT_TRAJECTORY_PENDING);
  motor_control_context_t ctx{};
  motor_control_init(&ctx, nullptr, 0U);
  c.boundary(&ctx, 5U, true);
  assert(c.status().state == AGENT_TRAJECTORY_CANCELED);
  assert(c.status().receipt.state == AGENT_TRAJECTORY_ACCEPTED);
  assert(c.commit(commit, 6U).state == AGENT_TRAJECTORY_RETIRED);
  assert(c.load(next, 6U).reason == REASON_CONFLICT);
  next.request_id = 4U;
  assert(c.load(next, 6U).state == AGENT_TRAJECTORY_ACCEPTED);
  assert(c.cancel(cancel).state == AGENT_TRAJECTORY_RETIRED);
  assert(c.status().trajectory_id == 2U);
  assert(!c.ack(8U, 4U, 2U));
  c.abort(REASON_STOPPED);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);
  c.begin_session(10U);
  assert(c.status().state == AGENT_TRAJECTORY_IDLE);
  assert(c.status().request_high_water == 0U);
}

static void test_commit_deadline_and_gate() {
  Coordinator c;
  c.begin_session(9U);
  auto r = load_request();
  c.load(r, 0U);
  auto commit = operation(2U, AGENT_TRAJECTORY_COMMIT);
  commit.valid_until_tick_ms = 5U;
  c.commit(commit, 0U);
  motor_control_context_t ctx{};
  motor_control_init(&ctx, nullptr, 0U);
  c.boundary(&ctx, 5U, true);
  assert(c.status().receipt.state == AGENT_TRAJECTORY_REJECTED);
  assert(c.status().receipt.reason == REASON_EXPIRED);
  assert(!ctx.trajectory_active);

  c.begin_session(9U);
  c.load(r, 0U);
  commit.valid_until_tick_ms = 1000U;
  c.commit(commit, 0U);
  c.boundary(&ctx, 5U, false);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);
  assert(!ctx.trajectory_active);
}

static void ready_kernel(motor_control_context_t &ctx, uint16_t count = 1U) {
  motor_control_init(&ctx, nullptr, 0U);
  ctx.authorized = true;
  ctx.have_snapshot = true;
  ctx.last_snapshot_tick_ms = 1U;
  ctx.snapshot_deadline_tick_ms = 31U;
  ctx.require_new_snapshot = false;
  ctx.entry_count = count;
  for (uint16_t i = 0U; i < count; ++i) {
    auto &entry = ctx.entries[i];
    entry.used = entry.online = entry.has_measured = true;
    entry.policy_enable = entry.motion_authorized = entry.motion_enabled = true;
    entry.lifecycle = MOTOR_CONTROL_LIFECYCLE_ACTIVE;
    entry.node = entry.motor_id = i + 1U;
    entry.adapter = &motor_protocol_vaxor_adapter;
    entry.profile = motor_control_profile_at(0U);
    entry.last_motion_rx_tick_ms = entry.last_rx_tick_ms = 1U;
  }
}

static void test_real_kernel_commit_completion_and_no_replay() {
  Coordinator c;
  c.begin_session(9U);
  motor_control_context_t ctx{};
  ready_kernel(ctx);
  c.load(load_request(), 1U);
  const auto commit = operation(2U, AGENT_TRAJECTORY_COMMIT);
  c.commit(commit, 1U);
  assert(!ctx.trajectory_active);
  c.boundary(&ctx, 5U, true);
  assert(ctx.trajectory_active);
  assert(c.status().state == AGENT_TRAJECTORY_RUNNING);
  assert(c.owns(1U));
  assert(!motor_control_cancel_trajectory_id(&ctx, 99U, 6U));
  assert(ctx.trajectory_active);
  ctx.last_snapshot_tick_ms = ctx.entries[0].last_motion_rx_tick_ms = 105U;
  c.boundary(&ctx, 105U, true);
  assert(c.status().state == AGENT_TRAJECTORY_SUCCEEDED);
  assert(!ctx.trajectory_active);
  assert(!c.owns(1U));
  assert(c.commit(commit, 106U).state == AGENT_TRAJECTORY_ACCEPTED);
  assert(ctx.trajectory_accepted_count == 1U);
  assert(c.ack(9U, 2U, 1U));
  assert(c.commit(commit, 107U).state == AGENT_TRAJECTORY_RETIRED);
}

static void test_one_stale_target_aborts_entire_group() {
  Coordinator c;
  c.begin_session(9U);
  motor_control_context_t ctx{};
  ready_kernel(ctx, 2U);
  auto r = load_request();
  r.target_count = 2U;
  r.target_ids[1] = 2U;
  r.point_count = 4U;
  r.points[1] = {0U, 2U, 0.0f, 0.0f, 0.0f};
  r.points[2] = {100U, 1U, 1.0f, 0.0f, 0.0f};
  r.points[3] = {100U, 2U, 1.0f, 0.0f, 0.0f};
  assert(c.load(r, 1U).state == AGENT_TRAJECTORY_ACCEPTED);
  c.commit(operation(2U, AGENT_TRAJECTORY_COMMIT), 1U);
  c.boundary(&ctx, 5U, true);
  assert(ctx.trajectory_active);
  ctx.last_snapshot_tick_ms = ctx.entries[0].last_motion_rx_tick_ms = 102U;
  motor_control_service(&ctx, 102U);
  assert(!ctx.trajectory_active); // 反馈过期在1ms服务拍就必须中止
  c.boundary(&ctx, 105U, true);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);
  assert(!ctx.entries[0].has_target);
}

static void test_cancel_waits_for_safe_output_queue() {
  Coordinator c;
  c.begin_session(9U);
  motor_control_context_t ctx{};
  ready_kernel(ctx);
  c.load(load_request(), 1U);
  c.commit(operation(2U, AGENT_TRAJECTORY_COMMIT), 1U);
  c.boundary(&ctx, 5U, true);
  ctx.tx_count = MOTOR_CONTROL_MAX_TX_PENDING;
  for (size_t i = 0U; i < ctx.tx_count; ++i)
    ctx.tx_pending[i].tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  const auto cancel = operation(3U, AGENT_TRAJECTORY_CANCEL);
  assert(c.cancel(cancel).state == AGENT_TRAJECTORY_PENDING);
  c.boundary(&ctx, 10U, false); // 无授权/无时钟仍可处理取消
  assert(ctx.entries[0].p0_pending);
  assert(c.status().state == AGENT_TRAJECTORY_RUNNING);
  assert(c.status().receipt.state == AGENT_TRAJECTORY_PENDING);
  motor_protocol_tx_frame_t frames[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_collect_tx(&ctx, frames, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_control_service(&ctx, 11U);
  c.boundary(&ctx, 15U, false);
  assert(c.status().state == AGENT_TRAJECTORY_CANCELED);
  assert(c.status().receipt.state == AGENT_TRAJECTORY_ACCEPTED);
}

static void test_kernel_rejects_interleaved_duplicate_time() {
  motor_control_context_t ctx{};
  ready_kernel(ctx, 2U);
  motor_control_trajectory_t trajectory{};
  trajectory.trajectory_id = 1U;
  trajectory.target_count = 2U;
  trajectory.target_ids[0] = 1U;
  trajectory.target_ids[1] = 2U;
  trajectory.point_count = 3U;
  trajectory.points[0] = {0U, 1U, 0.0f, 0.0f, 0.0f};
  trajectory.points[1] = {0U, 2U, 0.0f, 0.0f, 0.0f};
  trajectory.points[2] = {0U, 1U, 1.0f, 0.0f, 0.0f};
  assert(!motor_control_apply_agent_trajectory(&ctx, &trajectory, 5U));
  assert(!ctx.trajectory_active);
}

static void test_tick_wrap_and_request_exhaustion() {
  Coordinator c;
  c.begin_session(9U);
  auto r = load_request(AGENT_TRAJECTORY_MAX_ID, AGENT_TRAJECTORY_MAX_ID);
  r.segment_count = 2U;
  r.valid_until_tick_ms = 50U;
  assert(c.load(r, UINT32_MAX - 20U).state == AGENT_TRAJECTORY_ACCEPTED);
  r.valid_until_tick_ms = 500U;
  assert(c.load(r, 500U).state == AGENT_TRAJECTORY_ACCEPTED); // 不延长首段超时
  c.poll(978U);
  assert(c.status().state == AGENT_TRAJECTORY_LOADING);
  c.poll(979U);
  assert(c.status().state == AGENT_TRAJECTORY_ABORTED);
  assert(c.ack(9U, AGENT_TRAJECTORY_MAX_ID, AGENT_TRAJECTORY_MAX_ID));
  assert(c.status().request_high_water == AGENT_TRAJECTORY_MAX_ID);
  r = load_request(1U, 1U);
  assert(c.load(r, 0U).state == AGENT_TRAJECTORY_RETIRED);
}

int main() {
  test_load_ready_dedup_and_timeout();
  test_segments_capacity_and_timeline();
  test_commit_cancel_and_retirement();
  test_commit_deadline_and_gate();
  test_real_kernel_commit_completion_and_no_replay();
  test_one_stale_target_aborts_entire_group();
  test_cancel_waits_for_safe_output_queue();
  test_kernel_rejects_interleaved_duplicate_time();
  test_tick_wrap_and_request_exhaustion();
  std::printf("trajectory_transaction: PASS\n");
}
