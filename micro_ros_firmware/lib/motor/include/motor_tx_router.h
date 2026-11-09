// Protocol-aware P0/P1/P2 dequeue and physical CAN submission policy.

#ifndef MOTOR_TX_ROUTER_H
#define MOTOR_TX_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#include "can_tx_scheduler.h"
#include "motor_protocol_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_TX_ROUTER_LOGICAL_CAPACITY 40U
#define MOTOR_TX_ROUTER_PHYSICAL_BUDGET 16U

typedef bool (*motor_tx_router_submit_fn)(void *context, uint8_t bus_index,
    const can_tx_frame_t *frame);

typedef struct {
  // One shared scratch area is reused serially for the three bus schedulers.
  can_tx_frame_t logical_frames[MOTOR_TX_ROUTER_LOGICAL_CAPACITY];
  motor_protocol_tx_frame_t p1_lookahead[MOTOR_PROTOCOL_P1_BATCH_MAX];
} motor_tx_router_t;

typedef struct {
  uint32_t p0_logical_frames;
  uint32_t p1_logical_frames;
  uint32_t p2_logical_frames;
  uint32_t physical_attempts;
  uint32_t hardware_rejections;
  uint32_t p0_requeued;
  // aggregate_only 帧未能聚合而被拦下（不得原样上线）的次数。
  uint32_t aggregate_dropped;
} motor_tx_router_stats_t;

void motor_tx_router_init(motor_tx_router_t *router);

// Perform exactly one scheduler pump. While P0 is queued, only P0 is dequeued;
// the first rejected P0 stops this pump and it plus later P0 frames are restored
// to the FIFO front. P1/P2 remain queued until P0 drains. The caller owns the
// HAL callback and serializes calls (motor_rt_task is the sole owner).
motor_tx_router_stats_t motor_tx_router_pump(
  motor_tx_router_t *router, can_tx_scheduler_t *scheduler,
  uint8_t bus_index, motor_tx_router_submit_fn submit, void *context);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_TX_ROUTER_H
