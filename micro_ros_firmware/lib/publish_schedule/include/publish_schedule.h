#ifndef PUBLISH_SCHEDULE_H
#define PUBLISH_SCHEDULE_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define PUBLISH_SCHEDULE_TICK_MS 10U
#define PUBLISH_SCHEDULE_CONTROL_MS 50U
#define PUBLISH_SCHEDULE_STATUS_MS 100U
#include "firmware_config.h"
#define PUBLISH_SCHEDULE_MOTION_MS FIRMWARE_MOTOR_REPORT_PERIOD_MS
#if PUBLISH_SCHEDULE_MOTION_MS % PUBLISH_SCHEDULE_TICK_MS != 0U
#error "Motor report period must be a multiple of the bridge tick"
#endif

typedef enum {
  PUBLISH_RELIABLE_NONE = 0,
  PUBLISH_RELIABLE_CONTROL_STATES,
  PUBLISH_RELIABLE_MOTOR_STATUS,
  PUBLISH_RELIABLE_CONFIG_RESULT,
  PUBLISH_RELIABLE_EVENT,
  PUBLISH_RELIABLE_TOPOLOGY,
  PUBLISH_RELIABLE_DEVICE_PARAM,
} publish_reliable_t;

typedef struct {
  publish_reliable_t reliable;
  publish_reliable_t spare_served;
} publish_schedule_output_t;

typedef struct {
  publish_reliable_t last_spare;
  publish_reliable_t last_periodic;
  uint8_t periodic_streak;
} publish_schedule_state_t;

// 周期流连续最多占两轮；低速轮询时也为稀疏流保留有界发送机会。
publish_schedule_output_t publish_schedule_step(bool control_due, bool status_due,
    bool result_pending, bool event_pending, bool topology_due,
    bool device_pending, publish_schedule_state_t *state);
#ifdef __cplusplus
}
#endif
#endif
