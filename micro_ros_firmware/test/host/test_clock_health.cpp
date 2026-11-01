#include "clock_health.h"
#include <cassert>
#include <cstdint>

static void test_sync_expiry_30000ms() {
  clock_health_t clock{};
  clock_health_init(&clock);
  assert(!clock_health_valid(&clock, 0));
  clock_health_update(&clock, 100, true, 1000000000000LL);
  assert(clock_health_valid(&clock, 30100));
  assert(!clock_health_valid(&clock, 30101));
  uint64_t epoch = 99;
  assert(!clock_health_map_sample(&clock, 30101, 30101, &epoch));
  assert(epoch == 0);
}
static void test_jump_requires_second_stable_sync() {
  clock_health_t clock{};
  clock_health_init(&clock);
  clock_health_update(&clock, 0, true, 1000000000000LL);
  clock_health_update(&clock, 10000, true, 1000021000000LL);
  assert(!clock_health_valid(&clock, 10000));
  clock_health_update(&clock, 20000, false, 0);
  assert(!clock_health_valid(&clock, 20000));
  clock_health_update(&clock, 20001, true, 1000022000000LL);
  assert(clock_health_valid(&clock, 20001));
}
static void test_tick_wrap() {
  clock_health_t clock{};
  clock_health_init(&clock);
  constexpr uint32_t start = UINT32_MAX - 9;
  constexpr int64_t epoch = 1700000000000000000LL;
  clock_health_update(&clock, start, true, epoch - int64_t(start) * 1000000LL);
  uint64_t mapped = 0;
  assert(clock_health_map_sample(&clock, 5, 10, &mapped));
  assert(mapped == uint64_t(epoch + 15000000LL));
  // 回绕后成功同步仍是相同映射，不应被判为 49 天跳变。
  clock_health_update(&clock, 10, true, epoch + 20000000LL - 10000000LL);
  assert(clock_health_valid(&clock, 10));
}
static void test_failed_sync_keeps_invalid() {
  clock_health_t clock{};
  clock_health_init(&clock);
  clock_health_update(&clock, 1, false, 1000000000000LL);
  assert(!clock_health_valid(&clock, 1));
  assert(clock.failures == 1);
  clock_health_update(&clock, 2, true, 1000000000000LL);
  clock_health_update(&clock, 3, false, 0);
  assert(clock_health_valid(&clock, 3));
}
int main() {
  test_sync_expiry_30000ms();
  test_jump_requires_second_stable_sync();
  test_tick_wrap();
  test_failed_sync_keeps_invalid();
}
