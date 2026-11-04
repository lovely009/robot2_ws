// 只替代 ROS/时钟端口；poll 函数由 CMake 从真实固件源码直接提取，不复制实现。
#include "reliable_schedule.hpp"
#include <cassert>
#include <vector>

static uint64_t g_session_id = 1;
static uint32_t tick = 0, g_status_sequence = 0;
static uint32_t fingerprint = 42, g_sent_status_fingerprint = 42;
static bool g_status_dirty = false, g_have_sent_status = true;
static ReliableSchedule g_schedule;
static int g_status_pub = 1, g_info_pub = 2;
static struct { uint32_t sequence = 0; } g_status_msg;
static int info;
static bool info_pending = false, success = true;
static std::vector<int> sent;
static uint32_t millis() { return tick; }
static void collect_events() {}
static void param_bridge_poll() {}
static void publish_motion(uint32_t) {}
static void fill_status() {}
static uint32_t status_fingerprint() { return fingerprint; }
static const int *param_bridge_info() { return info_pending ? &info : nullptr; }
static bool send_reliable(int *publisher, const void *) {
  sent.push_back(*publisher);
  return success;
}
static void param_bridge_info_sent(bool ok) { if (ok) info_pending = false; }

#include "motor_bridge_poll.inc"

int main() {
  tick = 19; motor_bridge_poll(); assert(sent.empty());
  info_pending = true;
  tick = 20; motor_bridge_poll();
  assert(sent == std::vector<int>{1} && g_schedule.status_success == 20);
  tick = 40; motor_bridge_poll();
  assert((sent == std::vector<int>{1, 1}));
  success = false;
  tick = 60; motor_bridge_poll();
  assert((sent == std::vector<int>{1, 1, 2}) && info_pending);
  tick = 69; motor_bridge_poll(); assert(sent.size() == 3);
  success = true;
  tick = 70; motor_bridge_poll();
  tick = 80; motor_bridge_poll();
  assert((sent == std::vector<int>{1, 1, 2, 1, 2}) && !info_pending);

  fingerprint = 43; success = false;
  tick = 90; motor_bridge_poll();
  assert(g_status_dirty && g_sent_status_fingerprint == 42);
  assert(g_schedule.status_success == 70 && g_schedule.status_attempt == 90);
  tick = 100; motor_bridge_poll(); assert(sent.size() == 6);
  success = true;
  tick = 110; motor_bridge_poll();
  assert(sent.size() == 7 && sent.back() == 1);
  assert(!g_status_dirty && g_sent_status_fingerprint == 43);
  assert(g_schedule.status_success == 110);
  g_session_id = 0;
  tick = 130; motor_bridge_poll(); assert(sent.size() == 7);
}
