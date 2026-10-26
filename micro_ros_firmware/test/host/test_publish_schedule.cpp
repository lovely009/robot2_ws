#include <cassert>
#include <cstdio>
#include "publish_schedule.h"

int main() {
  publish_schedule_state_t state = {};
  auto r = publish_schedule_step(true, true, true, true, true, true, &state);
  assert(r.reliable == PUBLISH_RELIABLE_CONTROL_STATES);
  r = publish_schedule_step(false, true, true, true, true, true, &state);
  assert(r.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
  publish_reliable_t last = PUBLISH_RELIABLE_NONE;
  int served[7] = {};
  for (int i = 0; i < 40; ++i) {
    r = publish_schedule_step(false, false, true, true, true, true, &state);
    last = r.spare_served;
    ++served[r.reliable];
  }
  assert(served[PUBLISH_RELIABLE_CONFIG_RESULT] == 10);
  assert(served[PUBLISH_RELIABLE_EVENT] == 10);
  assert(served[PUBLISH_RELIABLE_TOPOLOGY] == 10);
  assert(served[PUBLISH_RELIABLE_DEVICE_PARAM] == 10);
  for (int i = 0; i < 5; ++i) {
    r = publish_schedule_step(false, false, false, false, false, true, &state);
    assert(r.reliable == PUBLISH_RELIABLE_DEVICE_PARAM);
    last = r.spare_served;
  }
  r = publish_schedule_step(false, false, false, false, false, false, &state);
  assert(r.reliable == PUBLISH_RELIABLE_NONE);
  assert(r.spare_served == last);
  state = {};
  int slow[7] = {};
  for (int i = 0; i < 120; ++i) {
    r = publish_schedule_step(true, true, true, true, true, true, &state);
    ++slow[r.reliable];
  }
  for (int i = 1; i < 7; ++i) assert(slow[i] > 0);
  std::puts("test_publish_schedule: ok");
}
