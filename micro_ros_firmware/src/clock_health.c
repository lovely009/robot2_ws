#include "clock_health.h"
#include <stddef.h>
#include <string.h>

void clock_health_init(clock_health_t *clock) {
  if (clock != NULL) memset(clock, 0, sizeof(*clock));
}
void clock_health_update(clock_health_t *clock, uint32_t now_ms,
    bool sync_ok, int64_t offset_ns) {
  if (clock == NULL) return;
  if (!sync_ok || offset_ns <= 0) {
    ++clock->failures;
    return;
  }
  const uint64_t epoch = (uint64_t)offset_ns + (uint64_t)now_ms * 1000000ULL;
  bool stable = true;
  if (clock->have_sync) {
    const uint64_t expected = clock->sync_epoch_ns +
        (uint64_t)(uint32_t)(now_ms - clock->sync_tick_ms) * 1000000ULL;
    const uint64_t jump = epoch > expected ? epoch - expected : expected - epoch;
    stable = jump <= CLOCK_HEALTH_MAX_JUMP_NS;
  }
  clock->sync_tick_ms = now_ms;
  clock->sync_epoch_ns = epoch;
  clock->offset_ns = offset_ns;
  clock->have_sync = true;
  clock->stable = stable;
}
bool clock_health_valid(const clock_health_t *clock, uint32_t now_ms) {
  return clock != NULL && clock->have_sync && clock->stable &&
      (uint32_t)(now_ms - clock->sync_tick_ms) <= CLOCK_HEALTH_MAX_AGE_MS;
}
bool clock_health_map_sample(const clock_health_t *clock,
    uint32_t sample_tick_ms, uint32_t now_ms, uint64_t *epoch_ns) {
  if (epoch_ns == NULL) return false;
  *epoch_ns = 0;
  if (!clock_health_valid(clock, now_ms)) return false;
  const uint32_t sample_age = now_ms - sample_tick_ms;
  if (sample_age >= 0x80000000U) return false;
  const uint64_t now_epoch = clock->sync_epoch_ns +
      (uint64_t)(uint32_t)(now_ms - clock->sync_tick_ms) * 1000000ULL;
  const uint64_t age_ns = (uint64_t)sample_age * 1000000ULL;
  if (age_ns >= now_epoch) return false;
  *epoch_ns = now_epoch - age_ns;
  return true;
}
