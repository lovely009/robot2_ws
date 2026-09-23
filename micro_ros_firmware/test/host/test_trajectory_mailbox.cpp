// 轨迹 mailbox 契约测试：有界校验、分段拼接、latest-wins、取消语义、会话门。
// 全部为纯逻辑，host 直接编译固件真实 .c（无桩无复制）。

#include "control_mailbox.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

static control_trajectory_t make_trajectory(uint32_t sequence,
    uint32_t received_tick_ms) {
  control_trajectory_t trajectory{};
  trajectory.sequence = sequence;
  trajectory.received_tick_ms = received_tick_ms;
  trajectory.segment_index = 0U;
  trajectory.segment_count = 1U;
  return trajectory;
}

// 单电机两点轨迹：t=0 → position 1.0，t=100 → position 2.0。
static control_trajectory_t make_two_point(uint16_t motor_id) {
  control_trajectory_t trajectory = make_trajectory(1U, 1000U);
  trajectory.target_count = 1U;
  trajectory.target_ids[0] = motor_id;
  trajectory.point_count = 2U;
  trajectory.points[0] = {0U, motor_id, 1.0F, 0.0F, 0.0F};
  trajectory.points[1] = {100U, motor_id, 2.0F, 0.0F, 0.5F};
  return trajectory;
}

static void test_publish_take_roundtrip() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t taken{};
  assert(!control_trajectory_mailbox_take(&mailbox, &taken));

  const control_trajectory_t trajectory = make_two_point(7U);
  assert(control_trajectory_mailbox_publish(&mailbox, &trajectory));
  assert(mailbox.pending);

  assert(control_trajectory_mailbox_take(&mailbox, &taken));
  assert(!mailbox.pending); // take 清 pending
  assert(taken.sequence == 1U);
  assert(taken.received_tick_ms == 1000U);
  assert(taken.target_count == 1U && taken.target_ids[0] == 7U);
  assert(taken.point_count == 2U);
  assert(taken.points[1].time_from_start_ms == 100U);
  assert(taken.points[1].position == 2.0F);
  assert(taken.points[1].effort == 0.5F);

  assert(!control_trajectory_mailbox_take(&mailbox, &taken)); // 已取走
}

static void test_latest_wins() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  const control_trajectory_t first = make_two_point(7U);
  control_trajectory_t second = make_two_point(9U);
  second.sequence = 2U;

  assert(control_trajectory_mailbox_publish(&mailbox, &first));
  assert(control_trajectory_mailbox_publish(&mailbox, &second));
  assert(mailbox.overwrite_count == 1U); // 未取走即被覆盖

  control_trajectory_t taken{};
  assert(control_trajectory_mailbox_take(&mailbox, &taken));
  assert(taken.sequence == 2U && taken.target_ids[0] == 9U); // 取到最新
  assert(mailbox.overwrite_count == 1U); // take 不清计数
}

static void test_cancel_is_valid_payload() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  // 空轨迹 = 取消，必须能受理（否则取消动作会被当成非法载荷丢掉）。
  const control_trajectory_t cancel = make_trajectory(5U, 2000U);
  assert(control_trajectory_mailbox_publish(&mailbox, &cancel));
  control_trajectory_t taken{};
  assert(control_trajectory_mailbox_take(&mailbox, &taken));
  assert(taken.point_count == 0U && taken.target_count == 0U);
}

static void test_rejects_malformed() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);
  const control_trajectory_t good = make_two_point(7U);

  // 计数越界
  control_trajectory_t bad = good;
  bad.point_count = CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS + 1U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.target_count = CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS + 1U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // 半条轨迹：有路点没归属 / 有归属没路点，二者都不可执行
  bad = good;
  bad.target_count = 0U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.point_count = 0U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // motor_id 不在声明集合内
  bad = good;
  bad.points[1].motor_id = 8U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // target_ids 重复
  bad = good;
  bad.target_count = 2U;
  bad.target_ids[1] = bad.target_ids[0];
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // 时间轴回退
  bad = good;
  bad.points[1].time_from_start_ms = 0U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // 同一电机同一时刻重复路点（插值分母为 0）
  bad = good;
  bad.target_count = 2U;
  bad.target_ids[1] = 9U;
  bad.point_count = 3U;
  bad.points[0] = {0U, 7U, 1.0F, 0.0F, 0.0F};
  bad.points[1] = {0U, 9U, 1.0F, 0.0F, 0.0F};  // 不同电机共用 t=0：合法
  bad.points[2] = {0U, 9U, 2.0F, 0.0F, 0.0F};  // 同一电机再来一次 t=0：非法
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // 声明了归属却没有任何路点
  bad = good;
  bad.target_count = 2U;
  bad.target_ids[1] = 9U;
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  // NaN / Inf（命令侧禁，TBD-ROS-01=A）
  bad = good;
  bad.points[0].position = std::numeric_limits<float>::quiet_NaN();
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.points[1].velocity = std::numeric_limits<float>::infinity();
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.points[1].effort = -std::numeric_limits<float>::infinity();
  assert(!control_trajectory_mailbox_publish(&mailbox, &bad));

  assert(mailbox.invalid_trajectory_count == 12U);
  assert(!mailbox.pending); // 非法载荷一律不留痕迹
}

static void test_multiple_motors_share_timestamps() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  // 并行的关节本就同拍采样：不同电机共用同一 time_from_start_ms 必须合法，
  // 否则会逼上层去重时间轴。
  control_trajectory_t trajectory = make_trajectory(3U, 500U);
  trajectory.target_count = 2U;
  trajectory.target_ids[0] = 7U;
  trajectory.target_ids[1] = 9U;
  trajectory.point_count = 4U;
  trajectory.points[0] = {0U, 7U, 0.0F, 0.0F, 0.0F};
  trajectory.points[1] = {0U, 9U, 0.1F, 0.0F, 0.0F};
  trajectory.points[2] = {50U, 7U, 0.5F, 0.0F, 0.0F};
  trajectory.points[3] = {50U, 9U, 0.6F, 0.0F, 0.0F};
  assert(control_trajectory_mailbox_publish(&mailbox, &trajectory));

  control_trajectory_t taken{};
  assert(control_trajectory_mailbox_take(&mailbox, &taken));
  assert(taken.target_count == 2U && taken.point_count == 4U);
}

static void test_segments_assemble_atomically_and_merge_boundary_points() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t first = make_trajectory(10U, 1000U);
  first.trajectory_id = 77U;
  first.segment_index = 0U;
  first.segment_count = 2U;
  first.target_count = 2U;
  first.target_ids[0] = 7U;
  first.target_ids[1] = 9U;
  first.point_count = 2U;
  first.points[0] = {0U, 7U, 1.0F, 0.0F, 0.0F};
  first.points[1] = {100U, 7U, 2.0F, 0.0F, 0.5F};

  // 分段一只包含电机 7；完整性检查延迟到最后一段。
  assert(control_trajectory_mailbox_publish(&mailbox, &first));
  assert(!mailbox.pending);
  assert(mailbox.assembly_active);

  control_trajectory_t second = make_trajectory(11U, 1010U);
  second.trajectory_id = 77U;
  second.segment_index = 1U;
  second.segment_count = 2U;
  second.target_count = 2U;
  second.target_ids[0] = 9U;
  second.target_ids[1] = 7U; // 集合顺序变化，但集合本身相同
  second.point_count = 3U;
  second.points[0] = {100U, 7U, 2.0F, 0.0F, 0.5F}; // 重叠边界：合并
  second.points[1] = {100U, 9U, 3.0F, 0.0F, 0.0F};
  second.points[2] = {200U, 9U, 4.0F, 0.0F, 0.0F};

  assert(control_trajectory_mailbox_publish(&mailbox, &second));
  assert(mailbox.pending);
  assert(!mailbox.assembly_active);

  control_trajectory_t complete{};
  assert(control_trajectory_mailbox_take(&mailbox, &complete));
  assert(complete.trajectory_id == 77U);
  assert(complete.segment_index == 0U && complete.segment_count == 1U);
  assert(complete.point_count == 4U); // 重叠点只保留一次
  assert(complete.points[1].time_from_start_ms == 100U);
  assert(complete.points[2].motor_id == 9U);
  assert(complete.points[3].time_from_start_ms == 200U);
  assert(complete.sequence == 11U);
  assert(complete.received_tick_ms == 1010U);
}

static void test_segments_self_join_without_overlap_point() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t first = make_trajectory(20U, 2000U);
  first.trajectory_id = 78U;
  first.segment_index = 0U;
  first.segment_count = 2U;
  first.target_count = 1U;
  first.target_ids[0] = 7U;
  first.point_count = 2U;
  first.points[0] = {0U, 7U, 0.0F, 0.0F, 0.0F};
  first.points[1] = {100U, 7U, 1.0F, 0.0F, 0.0F};
  assert(control_trajectory_mailbox_publish(&mailbox, &first));

  control_trajectory_t second = make_trajectory(21U, 2010U);
  second.trajectory_id = 78U;
  second.segment_index = 1U;
  second.segment_count = 2U;
  second.target_count = 1U;
  second.target_ids[0] = 7U;
  second.point_count = 2U;
  // 没有共享时间点：固件必须把 100ms 与 150ms 直接拼成同一时间线。
  second.points[0] = {150U, 7U, 1.5F, 0.0F, 0.0F};
  second.points[1] = {250U, 7U, 2.0F, 0.0F, 0.0F};
  assert(control_trajectory_mailbox_publish(&mailbox, &second));

  control_trajectory_t complete{};
  assert(control_trajectory_mailbox_take(&mailbox, &complete));
  assert(complete.point_count == 4U);
  assert(complete.points[1].time_from_start_ms == 100U);
  assert(complete.points[2].time_from_start_ms == 150U);
  assert(complete.points[3].time_from_start_ms == 250U);
}

static void test_segment_conflict_keeps_published_state_untouched() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t first = make_trajectory(1U, 100U);
  first.trajectory_id = 88U;
  first.segment_index = 0U;
  first.segment_count = 2U;
  first.target_count = 1U;
  first.target_ids[0] = 7U;
  first.point_count = 2U;
  first.points[0] = {0U, 7U, 0.0F, 0.0F, 0.0F};
  first.points[1] = {100U, 7U, 1.0F, 0.0F, 0.0F};
  assert(control_trajectory_mailbox_publish(&mailbox, &first));

  control_trajectory_t conflicting = first;
  conflicting.sequence = 2U;
  conflicting.received_tick_ms = 200U;
  conflicting.segment_index = 1U;
  conflicting.points[0] = {100U, 7U, 9.0F, 0.0F, 0.0F};
  conflicting.points[1] = {200U, 7U, 10.0F, 0.0F, 0.0F};
  assert(!control_trajectory_mailbox_publish(&mailbox, &conflicting));
  assert(!mailbox.pending);
  assert(!mailbox.assembly_active);
  assert(mailbox.invalid_trajectory_count == 1U);
}

static void test_segment_order_and_total_capacity_are_bounded() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t first = make_trajectory(1U, 100U);
  first.trajectory_id = 99U;
  first.segment_index = 0U;
  first.segment_count = 3U;
  first.target_count = 1U;
  first.target_ids[0] = 7U;
  first.point_count = 1U;
  first.points[0] = {0U, 7U, 0.0F, 0.0F, 0.0F};
  assert(control_trajectory_mailbox_publish(&mailbox, &first));

  control_trajectory_t out_of_order = first;
  out_of_order.segment_index = 2U;
  out_of_order.points[0] = {200U, 7U, 2.0F, 0.0F, 0.0F};
  assert(!control_trajectory_mailbox_publish(&mailbox, &out_of_order));
  assert(!mailbox.assembly_active);
  assert(!mailbox.pending);

  control_trajectory_mailbox_init(&mailbox);
  first.segment_count = 2U;
  first.point_count = CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS;
  for (uint16_t i = 0U; i < first.point_count; ++i) {
    first.points[i] = {i, 7U, static_cast<float>(i), 0.0F, 0.0F};
  }
  assert(control_trajectory_mailbox_publish(&mailbox, &first));

  control_trajectory_t overflow = make_trajectory(2U, 200U);
  overflow.trajectory_id = 99U;
  overflow.segment_index = 1U;
  overflow.segment_count = 2U;
  overflow.target_count = 1U;
  overflow.target_ids[0] = 7U;
  overflow.point_count = 1U;
  overflow.points[0] = {CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS, 7U,
                        static_cast<float>(CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS),
                        0.0F, 0.0F};
  assert(!control_trajectory_mailbox_publish(&mailbox, &overflow));
  assert(!mailbox.assembly_active);
  assert(!mailbox.pending);
}

// 上层（web 示教）允许段边界重叠一行：原始分段载荷之和可超过 256，但去重
// 合并后恰好在容量内。容量必须按合并后的总量判，2026-09-22 实测 256 点
// 轨迹曾因此被误拒（页面报「未收到 trajectory_state 回执」）。
static void test_segment_capacity_counts_merged_points() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  control_trajectory_t first = make_trajectory(1U, 100U);
  first.trajectory_id = 101U;
  first.segment_index = 0U;
  first.segment_count = 2U;
  first.target_count = 1U;
  first.target_ids[0] = 7U;
  first.point_count = 128U;
  for (uint16_t i = 0U; i < first.point_count; ++i) {
    first.points[i] = {i, 7U, static_cast<float>(i), 0.0F, 0.0F};
  }
  assert(control_trajectory_mailbox_publish(&mailbox, &first));

  // 原始合计 128+129=257 > 256，t=127 是重叠边界点，合并后恰为 256。
  control_trajectory_t second = make_trajectory(2U, 200U);
  second.trajectory_id = 101U;
  second.segment_index = 1U;
  second.segment_count = 2U;
  second.target_count = 1U;
  second.target_ids[0] = 7U;
  second.point_count = 129U;
  for (uint16_t i = 0U; i < second.point_count; ++i) {
    const uint16_t t = static_cast<uint16_t>(127U + i);
    second.points[i] = {t, 7U, static_cast<float>(t), 0.0F, 0.0F};
  }
  assert(control_trajectory_mailbox_publish(&mailbox, &second));
  assert(mailbox.pending);
  assert(!mailbox.assembly_active);

  control_trajectory_t complete{};
  assert(control_trajectory_mailbox_take(&mailbox, &complete));
  assert(complete.point_count ==
      CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS);    // 合并后 256，重叠点只留一次
  assert(complete.points[127].time_from_start_ms == 127U);
  assert(complete.points[128].time_from_start_ms == 128U);

  // 去重后仍超 256：写入前的容量门必须整组拒绝。
  control_trajectory_mailbox_init(&mailbox);
  assert(control_trajectory_mailbox_publish(&mailbox, &first));
  control_trajectory_t overflow = make_trajectory(3U, 300U);
  overflow.trajectory_id = 102U;
  overflow.segment_index = 1U;
  overflow.segment_count = 2U;
  overflow.target_count = 1U;
  overflow.target_ids[0] = 7U;
  overflow.point_count = 130U; // 合并后 257
  for (uint16_t i = 0U; i < overflow.point_count; ++i) {
    const uint16_t t = static_cast<uint16_t>(127U + i);
    overflow.points[i] = {t, 7U, static_cast<float>(t), 0.0F, 0.0F};
  }
  assert(!control_trajectory_mailbox_publish(&mailbox, &overflow));
  assert(!mailbox.pending);
  assert(!mailbox.assembly_active);
  assert(mailbox.invalid_trajectory_count == 1U);
}

// 三段拼接 + 多电机 + 极短尾段：64 行 × 4 台 = 256 点，按 128 点/段切分并
// 重叠边界行 → [128, 128, 8]（第三段只剩两行）。这是 2026-09-22 之前网页发的
// 形状（网页现已改为一条轨迹一条消息，原因见 web_host 的 TRAJ_SEGMENT_POINTS
// 注释）；此处钉住拼接侧行为：合并后恰好 256 点必须被受理，且不留非法计数。
static void test_three_segments_with_two_row_tail_assemble_at_capacity() {
  static const uint16_t kMotors[4] = {1U, 2U, 3U, 4U};
  static const uint16_t kRowStart[3] = {0U, 31U, 62U};
  static const uint16_t kRowEnd[3] = {32U, 63U, 64U};

  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  for (uint16_t seg = 0U; seg < 3U; ++seg) {
    control_trajectory_t s = make_trajectory(seg + 1U, 1000U + seg);
    s.trajectory_id = 4242U;
    s.segment_index = seg;
    s.segment_count = 3U;
    s.target_count = 4U;
    for (uint16_t m = 0U; m < 4U; ++m) {
      s.target_ids[m] = kMotors[m];
    }
    uint16_t n = 0U;
    for (uint16_t row = kRowStart[seg]; row < kRowEnd[seg]; ++row) {
      const uint32_t t = static_cast<uint32_t>(row) * 100U; // 每行 wait=100ms
      for (uint16_t m = 0U; m < 4U; ++m) {
        s.points[n++] = {t, kMotors[m], 0.01F * static_cast<float>(row), 0.0F,
                         0.0F};
      }
    }
    s.point_count = n;
    assert(control_trajectory_mailbox_publish(&mailbox, &s));
  }

  assert(mailbox.pending);
  assert(!mailbox.assembly_active);
  control_trajectory_t complete{};
  assert(control_trajectory_mailbox_take(&mailbox, &complete));
  assert(complete.point_count == 256U);
  assert(complete.segment_index == 0U && complete.segment_count == 1U);
  assert(complete.points[255].time_from_start_ms == 6300U);
  assert(mailbox.invalid_trajectory_count == 0U);
  assert(mailbox.overwrite_count == 0U);
}

static void test_clear_keeps_diagnostics() {
  control_trajectory_mailbox_t mailbox{};
  control_trajectory_mailbox_init(&mailbox);

  const control_trajectory_t trajectory = make_two_point(7U);
  assert(control_trajectory_mailbox_publish(&mailbox, &trajectory));
  control_trajectory_mailbox_clear(&mailbox);
  assert(!mailbox.pending);

  control_trajectory_t taken{};
  assert(!control_trajectory_mailbox_take(&mailbox, &taken)); // 未取走的被撤销
}

static void test_session_gate_and_disconnect() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  const control_trajectory_t trajectory = make_two_point(7U);

  // 引导期（未见 Agent 状态）= 会话豁免，受理
  assert(control_mailbox_submit_trajectory(&mailbox, &trajectory));
  control_trajectory_t taken{};
  assert(control_mailbox_take_trajectory(&mailbox, &taken));

  // 撤权后拒绝，计入 trajectory_rejected_count
  control_mailbox_request_authorization(&mailbox, false);
  assert(!control_mailbox_submit_trajectory(&mailbox, &trajectory));
  assert(mailbox.trajectory_rejected_count == 1U);
  control_mailbox_request_authorization(&mailbox, true);

  // 断链清轨迹：不得留一条「未来还要动」的时间线
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_submit_trajectory(&mailbox, &trajectory));
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(!control_mailbox_take_trajectory(&mailbox, &taken));
  assert(mailbox.trajectory.pending == false);

  // 断链期提交走会话门，与载荷合法性不重复计数
  assert(!control_mailbox_submit_trajectory(&mailbox, &trajectory));
  assert(mailbox.trajectory_rejected_count == 2U);
  assert(mailbox.trajectory.invalid_trajectory_count == 0U);
}

int main() {
  test_publish_take_roundtrip();
  test_latest_wins();
  test_cancel_is_valid_payload();
  test_rejects_malformed();
  test_multiple_motors_share_timestamps();
  test_segments_assemble_atomically_and_merge_boundary_points();
  test_segments_self_join_without_overlap_point();
  test_segment_conflict_keeps_published_state_untouched();
  test_segment_order_and_total_capacity_are_bounded();
  test_segment_capacity_counts_merged_points();
  test_three_segments_with_two_row_tail_assemble_at_capacity();
  test_clear_keeps_diagnostics();
  test_session_gate_and_disconnect();
  return 0;
}
