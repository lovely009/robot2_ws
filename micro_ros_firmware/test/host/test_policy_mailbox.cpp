// 策略 mailbox 契约测试：有界校验、latest-wins、稀疏语义、会话门。
// 全部为纯逻辑，host 直接编译固件真实 .c（无桩无复制）。

#include "control_mailbox.h"

#include <cassert>
#include <cmath>
#include <limits>

static control_policy_snapshot_t make_snapshot(uint32_t request_id,
    uint32_t generation) {
  control_policy_snapshot_t s{};
  s.received_tick_ms = 1000U;
  s.request_id = request_id;
  s.policy_generation = generation;
  return s;
}

static void add_target(control_policy_snapshot_t *s, uint16_t motor_id,
    bool enabled, float kp, float kd) {
  s->targets[s->target_count] = {motor_id, enabled, kp, kd};
  s->target_count++;
}

static void test_publish_take_roundtrip() {
  control_policy_mailbox_t mailbox{};
  control_policy_mailbox_init(&mailbox);

  control_policy_snapshot_t taken{};
  assert(!control_policy_mailbox_take(&mailbox, &taken));

  control_policy_snapshot_t s = make_snapshot(7U, 3U);
  add_target(&s, 5U, true, 10.0F, 1.0F);
  assert(control_policy_mailbox_publish(&mailbox, &s));
  assert(control_policy_mailbox_take(&mailbox, &taken));
  assert(!mailbox.pending); // take 清 pending
  assert(taken.request_id == 7U && taken.policy_generation == 3U);
  assert(taken.target_count == 1U && taken.targets[0].motor_id == 5U);
  assert(taken.targets[0].enabled && taken.targets[0].kp == 10.0F);
  assert(!control_policy_mailbox_take(&mailbox, &taken));
}

static void test_latest_wins() {
  control_policy_mailbox_t mailbox{};
  control_policy_mailbox_init(&mailbox);
  control_policy_snapshot_t first = make_snapshot(1U, 1U);
  add_target(&first, 5U, true, 1.0F, 1.0F);
  control_policy_snapshot_t second = make_snapshot(2U, 2U);
  add_target(&second, 9U, true, 2.0F, 2.0F);

  assert(control_policy_mailbox_publish(&mailbox, &first));
  assert(control_policy_mailbox_publish(&mailbox, &second));
  assert(mailbox.overwrite_count == 1U);

  control_policy_snapshot_t taken{};
  assert(control_policy_mailbox_take(&mailbox, &taken));
  assert(taken.request_id == 2U && taken.targets[0].motor_id == 9U);
}

// 空策略是合法载荷（等价于无操作），不是非法——否则「什么都不改」这件事
// 反而会被当成错误。
static void test_empty_policy_is_valid() {
  control_policy_mailbox_t mailbox{};
  control_policy_mailbox_init(&mailbox);
  const control_policy_snapshot_t empty = make_snapshot(1U, 1U);
  assert(control_policy_mailbox_publish(&mailbox, &empty));
  control_policy_snapshot_t taken{};
  assert(control_policy_mailbox_take(&mailbox, &taken));
  assert(taken.target_count == 0U);
}

static void test_rejects_malformed() {
  control_policy_mailbox_t mailbox{};
  control_policy_mailbox_init(&mailbox);
  control_policy_snapshot_t good = make_snapshot(1U, 1U);
  add_target(&good, 5U, true, 1.0F, 1.0F);

  control_policy_snapshot_t bad = good;
  bad.target_count = CONTROL_POLICY_MAILBOX_MAX_TARGETS + 1U;
  assert(!control_policy_mailbox_publish(&mailbox, &bad));

  // 同一台出现两次：哪一条算数没有定义
  bad = good;
  add_target(&bad, 5U, false, 1.0F, 1.0F);
  assert(!control_policy_mailbox_publish(&mailbox, &bad));

  // 负增益 / NaN / Inf
  bad = good;
  bad.targets[0].kp = -1.0F;
  assert(!control_policy_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.targets[0].kd = std::numeric_limits<float>::quiet_NaN();
  assert(!control_policy_mailbox_publish(&mailbox, &bad));
  bad = good;
  bad.targets[0].kp = std::numeric_limits<float>::infinity();
  assert(!control_policy_mailbox_publish(&mailbox, &bad));

  assert(mailbox.invalid_policy_count == 5U);
  assert(!mailbox.pending); // 非法载荷一律不留痕迹
}

static void test_session_gate_and_disconnect() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);
  control_policy_snapshot_t s = make_snapshot(1U, 1U);
  add_target(&s, 5U, true, 1.0F, 1.0F);

  // 引导期（未见 Agent 状态）= 会话豁免，受理
  assert(control_mailbox_submit_policy(&mailbox, &s));
  control_policy_snapshot_t taken{};
  assert(control_mailbox_take_policy(&mailbox, &taken));

  control_mailbox_request_authorization(&mailbox, false);
  assert(!control_mailbox_submit_policy(&mailbox, &s));
  assert(mailbox.policy_rejected_count == 1U);
  control_mailbox_request_authorization(&mailbox, true);

  // 断链清策略：不得留一条「断链前声明的武装位」在恢复后被悄悄用上
  control_mailbox_request_agent_connected(&mailbox, true);
  assert(control_mailbox_submit_policy(&mailbox, &s));
  control_mailbox_request_agent_connected(&mailbox, false);
  assert(!control_mailbox_take_policy(&mailbox, &taken));
  assert(mailbox.policy.pending == false);

  assert(!control_mailbox_submit_policy(&mailbox, &s));
  assert(mailbox.policy_rejected_count == 2U);
  assert(mailbox.policy.invalid_policy_count == 0U);
}

int main() {
  test_publish_take_roundtrip();
  test_latest_wins();
  test_empty_policy_is_valid();
  test_rejects_malformed();
  test_session_gate_and_disconnect();
  return 0;
}
