// 同步健康与 MCU 单调时基映射；所有跨任务访问在短临界区内。
#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include <stdint.h>
#include <stdbool.h>
#include <builtin_interfaces/msg/time.h>
#include "clock_health.h"
#ifdef __cplusplus
extern "C" {
#endif
void time_sync_reset(void);
void time_sync_record(bool success, uint64_t epoch_ns);
void time_clock_snapshot(clock_health_t *out);
bool time_clock_valid(void);
bool time_sample_epoch(uint32_t sample_tick_ms, uint64_t *epoch_ns);
uint64_t time_epoch_now(void);
void fill_epoch_stamp(builtin_interfaces__msg__Time *stamp, uint64_t epoch_ns);
void fill_header_stamp(builtin_interfaces__msg__Time *stamp);
#ifdef __cplusplus
}
#endif
#endif
