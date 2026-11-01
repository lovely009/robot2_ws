#include "time_utils.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>

static clock_health_t g_clock;
void time_sync_reset(void) {
  taskENTER_CRITICAL();
  clock_health_init(&g_clock);
  taskEXIT_CRITICAL();
}
void time_sync_record(bool success, uint64_t epoch_ns) {
  taskENTER_CRITICAL();
  const uint32_t now = millis();
  clock_health_update(&g_clock, now, success && epoch_ns > 0,
      (int64_t)epoch_ns - (int64_t)now * 1000000LL);
  taskEXIT_CRITICAL();
}
void time_clock_snapshot(clock_health_t *out) {
  if (out == nullptr) return;
  taskENTER_CRITICAL();
  *out = g_clock;
  taskEXIT_CRITICAL();
}
bool time_clock_valid(void) {
  clock_health_t clock;
  time_clock_snapshot(&clock);
  return clock_health_valid(&clock, millis());
}
bool time_sample_epoch(uint32_t sample_tick_ms, uint64_t *epoch_ns) {
  clock_health_t clock;
  time_clock_snapshot(&clock);
  return clock_health_map_sample(&clock, sample_tick_ms, millis(), epoch_ns);
}
uint64_t time_epoch_now(void) {
  uint64_t epoch = 0;
  time_sample_epoch(millis(), &epoch);
  return epoch;
}
void fill_epoch_stamp(builtin_interfaces__msg__Time *stamp, uint64_t epoch_ns) {
  stamp->sec = (int32_t)(epoch_ns / 1000000000ULL);
  stamp->nanosec = (uint32_t)(epoch_ns % 1000000000ULL);
}
void fill_header_stamp(builtin_interfaces__msg__Time *stamp) {
  fill_epoch_stamp(stamp, time_epoch_now());
}
