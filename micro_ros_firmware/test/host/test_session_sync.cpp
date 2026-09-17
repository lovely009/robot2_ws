// session_sync 特征测试：先钉住既有控制边界的会话/授权行为，再替换实现。
#include "motor_control_internal.h"
#include "session_sync.h"

#include <cassert>

namespace {

motor_control_context_t g_ctx;

void reset() {
  motor_control_init(&g_ctx, nullptr, 0);
}

// 首边界（无边沿、电平=引导豁免 true）：授权建立、从未连接、无 LOST 事件。
static void test_boot_exempt() {
  reset();
  control_mailbox_t mb{};
  control_mailbox_init(&mb);
  control_mailbox_events_t events{};
  assert(!control_mailbox_take_events(&mb, &events)); // 引导期无边沿
  session_sync_apply(&g_ctx, &events,
                     control_mailbox_session_authorized(&mb), 10);
  assert(g_ctx.authorized);
  assert(!g_ctx.agent_connected && !g_ctx.agent_ever_connected);

  motor_control_event_t out[8] = {};
  assert(motor_control_drain_events(&g_ctx, out, 8) == 0U);
}

// 连接边沿：ever/connected 置位、授权在、无撤权事件。
static void test_connect() {
  reset();
  control_mailbox_t mb{};
  control_mailbox_init(&mb);
  control_mailbox_request_agent_connected(&mb, true);
  control_mailbox_events_t events{};
  assert(control_mailbox_take_events(&mb, &events));
  assert(events.agent_state_pending && events.agent_connected_requested);
  session_sync_apply(&g_ctx, &events,
                     control_mailbox_session_authorized(&mb), 20);
  assert(g_ctx.agent_connected && g_ctx.agent_ever_connected);
  assert(g_ctx.authorized);
  motor_control_event_t out[8] = {};
  assert(motor_control_drain_events(&g_ctx, out, 8) == 0U);
}

// 断连：两次 AUTHORIZATION_LOST（set_agent + set_authorization 各一次，
// 当前实现刻意保留的双事件），authorized=false、要新快照。
static void test_disconnect_double_lost() {
  reset();
  control_mailbox_t mb{};
  control_mailbox_init(&mb);
  // 先连
  control_mailbox_request_agent_connected(&mb, true);
  control_mailbox_events_t e{};
  (void)control_mailbox_take_events(&mb, &e);
  session_sync_apply(&g_ctx, &e, true, 20);

  // 再断（mailbox 清命令并撤销授权电平）
  control_mailbox_request_agent_connected(&mb, false);
  control_mailbox_events_t e2{};
  (void)control_mailbox_take_events(&mb, &e2);
  assert(e2.agent_state_pending && !e2.agent_connected_requested);
  session_sync_apply(&g_ctx, &e2,
                     control_mailbox_session_authorized(&mb), 40);
  assert(!g_ctx.agent_connected && !g_ctx.authorized);
  assert(g_ctx.require_new_snapshot);

  motor_control_event_t out[8] = {};
  size_t n = motor_control_drain_events(&g_ctx, out, 8);
  assert(n == 2U);
  for (size_t i = 0; i < n; ++i) {
    assert(out[i].type == MOTOR_CONTROL_EVENT_AUTHORIZATION_LOST);
  }
}

// 重连：授权恢复（无需显式重授权命令），但仍需新快照才能运动。
static void test_reconnect_reauthorizes() {
  reset();
  control_mailbox_t mb{};
  control_mailbox_init(&mb);
  control_mailbox_request_agent_connected(&mb, true);
  control_mailbox_events_t e{};
  (void)control_mailbox_take_events(&mb, &e);
  session_sync_apply(&g_ctx, &e, true, 20);
  control_mailbox_request_agent_connected(&mb, false);
  control_mailbox_events_t e2{};
  (void)control_mailbox_take_events(&mb, &e2);
  session_sync_apply(&g_ctx, &e2, false, 40);

  control_mailbox_request_agent_connected(&mb, true);
  control_mailbox_events_t e3{};
  (void)control_mailbox_take_events(&mb, &e3);
  session_sync_apply(&g_ctx, &e3,
                     control_mailbox_session_authorized(&mb), 60);
  assert(g_ctx.agent_connected && g_ctx.authorized);
  assert(g_ctx.require_new_snapshot); // 新快照由 apply_command_snapshot 清除
}

// 无连接边沿的每边界电平维持：在线每边界幂等重立授权。
static void test_steady_level() {
  reset();
  control_mailbox_events_t e{};
  // 引导后直连（测试直给电平 true + 连接边沿）
  control_mailbox_t mb{};
  control_mailbox_init(&mb);
  control_mailbox_request_agent_connected(&mb, true);
  (void)control_mailbox_take_events(&mb, &e);
  session_sync_apply(&g_ctx, &e, true, 20);
  // 连续无边沿边界
  control_mailbox_events_t empty{};
  session_sync_apply(&g_ctx, &empty, true, 40);
  session_sync_apply(&g_ctx, &empty, true, 60);
  assert(g_ctx.authorized);
  // 无事件、电平断（断连边沿已先处理后的后续边界）：幂等撤权
  session_sync_apply(&g_ctx, &empty, false, 80);
  assert(!g_ctx.authorized);
}

} // namespace

int main() {
  test_boot_exempt();
  test_connect();
  test_disconnect_double_lost();
  test_reconnect_reauthorizes();
  test_steady_level();
  return 0;
}
