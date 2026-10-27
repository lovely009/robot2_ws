#include "info_page.hpp"
#include <cassert>

using namespace info_page;
static Device devices[20]{};

static void test_revision_change_returns_retry() {
  Pager p;
  p.set_source(9, 1, 20);
  assert(p.request(9, 1, 0, 0) == Accepted);
  p.fill(devices, 20, 100, 1000000000);
  assert(p.page().count == 16 && p.page().next_cursor == 16 && !p.page().complete);
  p.sent(true);
  p.set_source(9, 2, 20);
  assert(p.request(9, 1, 1, 16) == Retry);
  assert(p.page().revision == 2 && p.page().count == 0);
}
static void test_repeat_page_no_hardware_read() {
  Pager p;
  devices[0].motor_id = 7;
  p.set_source(9, 1, 20);
  assert(p.request(9, 3, 0, 0) == Accepted);
  p.fill(devices, 20, 100, 1000000000);
  devices[0].motor_id = 8;
  assert(p.request(9, 3, 0, 0) == Accepted);
  assert(!p.needs_fill() && p.page().devices[0].motor_id == 7);
}
static void test_send_failure_retains_page() {
  Pager p;
  p.set_source(9, 1, 20);
  assert(p.request(9, 1, 0, 0) == Accepted);
  p.fill(devices, 20, 100, 0);
  p.sent(false);
  assert(p.pending() && p.page().tick_ms == 100);
  assert(p.request(9, 2, 0, 0) == Busy);
  p.sent(true);
  assert(!p.pending());
}
static void test_stale_session_rejected() {
  Pager p;
  p.set_source(9, 1, 20);
  assert(p.request(8, 1, 0, 0) == Retired);
  p.set_source(10, 1, 20);
  assert(!p.pending());
  assert(p.request(9, 1, 0, 0) == Retired);
}
int main() {
  test_revision_change_returns_retry();
  test_repeat_page_no_hardware_read();
  test_send_failure_retains_page();
  test_stale_session_rejected();
}
