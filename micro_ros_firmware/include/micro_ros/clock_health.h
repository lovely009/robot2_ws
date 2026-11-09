#ifndef CLOCK_HEALTH_H
#define CLOCK_HEALTH_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CLOCK_HEALTH_MAX_AGE_MS 30000U
#define CLOCK_HEALTH_MAX_JUMP_NS 20000000LL
typedef struct {
  uint32_t sync_tick_ms;
  uint32_t failures;
  int64_t offset_ns;
  uint64_t sync_epoch_ns;
  bool have_sync;
  bool stable;
} clock_health_t;
void clock_health_init(clock_health_t *clock);
// offset = 本次 epoch - 本次 uint32 MCU tick * 1e6；内部锚点处理 tick 回绕。
void clock_health_update(clock_health_t *clock, uint32_t now_ms,
    bool sync_ok, int64_t offset_ns);
bool clock_health_valid(const clock_health_t *clock, uint32_t now_ms);
bool clock_health_map_sample(const clock_health_t *clock,
    uint32_t sample_tick_ms, uint32_t now_ms, uint64_t *epoch_ns);
#ifdef __cplusplus
}
#endif
#endif
