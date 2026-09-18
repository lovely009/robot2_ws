// 轨迹 mailbox 契约测试：有界校验、latest-wins、取消语义、会话门。
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
  test_clear_keeps_diagnostics();
  test_session_gate_and_disconnect();
  return 0;
}
