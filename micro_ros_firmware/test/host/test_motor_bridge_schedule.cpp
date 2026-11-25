// 只替代 ROS/时钟端口；poll 函数由 CMake 从真实固件源码直接提取，不复制实现。
#include <cassert>
#include <cstdint>
#include <vector>

#define FIRMWARE_STATUS_REPORT_PERIOD_MS 200U
static constexpr uint32_t kStatusMinIntervalMs = 20U;
using rcl_ret_t = int;
static constexpr rcl_ret_t RCL_RET_OK = 0;
static constexpr rcl_ret_t RCL_RET_ERROR = 1;

static uint64_t g_session_id = 1;
static uint32_t tick = 0, g_status_tick = 0, g_status_sequence = 0;
static uint32_t fingerprint = 42, g_sent_status_fingerprint = 42;
static bool g_status_dirty = false, g_have_sent_status = true;
static bool g_active = true, g_publish_ok = true;
static int g_state_pub = 1;
static struct { uint32_t sequence = 0; } g_state_msg;
static std::vector<int> sent;

static bool ros_bridge_is_active() { return g_active; }
static uint32_t millis() { return tick; }
static void publish_events() {}
static void param_bridge_poll() {}
static void publish_motion(uint32_t) {}
static void fill_state() {}
static uint32_t status_fingerprint() { return fingerprint; }
static void serial_transport_send_begin() {}
static void serial_transport_send_end() {}
static void note_publish(rcl_ret_t) {}
static rcl_ret_t rcl_publish(int *publisher, const void *, const void *) {
  sent.push_back(*publisher);
  return g_publish_ok ? RCL_RET_OK : RCL_RET_ERROR;
}

#include "motor_bridge_poll.inc"

int main() {
  tick = 19; motor_bridge_poll(); assert(sent.empty());
  tick = 200; motor_bridge_poll();
  assert(sent == std::vector<int>{1} && g_status_tick == 200);

  fingerprint = 43; g_publish_ok = false;
  tick = 220; motor_bridge_poll();
  assert(g_status_dirty && g_sent_status_fingerprint == 42);
  assert(sent.size() == 2);
  g_publish_ok = true;
  tick = 240; motor_bridge_poll();
  assert(sent.size() == 3 && !g_status_dirty && g_sent_status_fingerprint == 43);

  g_active = false;
  tick = 500; motor_bridge_poll(); assert(sent.size() == 3);
  g_active = true; g_session_id = 0;
  tick = 700; motor_bridge_poll(); assert(sent.size() == 3);
}
