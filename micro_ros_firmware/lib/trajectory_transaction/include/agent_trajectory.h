#ifndef AGENT_TRAJECTORY_H
#define AGENT_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>
#include "control_trajectory_mailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AGENT_TRAJECTORY_LOAD 0U
#define AGENT_TRAJECTORY_COMMIT 1U
#define AGENT_TRAJECTORY_CANCEL 2U
#define AGENT_TRAJECTORY_ACK 3U
#define AGENT_TRAJECTORY_PENDING 0U
#define AGENT_TRAJECTORY_ACCEPTED 1U
#define AGENT_TRAJECTORY_REJECTED 2U
#define AGENT_TRAJECTORY_RETIRED 3U
#define AGENT_TRAJECTORY_IDLE 0U
#define AGENT_TRAJECTORY_LOADING 1U
#define AGENT_TRAJECTORY_READY 2U
#define AGENT_TRAJECTORY_RUNNING 3U
#define AGENT_TRAJECTORY_SUCCEEDED 4U
#define AGENT_TRAJECTORY_CANCELED 5U
#define AGENT_TRAJECTORY_ABORTED 6U
#define AGENT_TRAJECTORY_SEGMENT_POINTS 32U
#define AGENT_TRAJECTORY_MAX_ID 0x3FFFFFFFU

typedef struct {
  uint64_t session_id;
  uint32_t request_id;
  uint32_t trajectory_id;
  uint32_t valid_until_tick_ms;
  uint32_t ack_request_id;
  uint32_t ack_terminal_trajectory_id;
  uint16_t segment_index;
  uint16_t segment_count;
  uint16_t target_count;
  uint16_t point_count;
  uint8_t operation;
  uint16_t target_ids[48];
  control_trajectory_point_t points[AGENT_TRAJECTORY_SEGMENT_POINTS];
} agent_trajectory_request_t;

typedef struct {
  uint32_t request_id;
  uint32_t trajectory_id;
  uint8_t operation;
  uint8_t state;
  uint16_t reason;
  uint16_t segments_received;
  uint16_t segment_count;
} agent_trajectory_receipt_t;

typedef struct {
  agent_trajectory_receipt_t receipt;
  uint32_t request_high_water;
  uint32_t trajectory_id;
  uint32_t elapsed_ms;
  uint32_t duration_ms;
  uint8_t state;
  uint16_t reason;
  uint16_t segments_received;
  uint16_t segment_count;
  uint16_t target_count;
  uint16_t target_ids[48]; // 终态保留，确认终态后清空；与状态同锁复制
} agent_trajectory_status_t;

// 只接收有界纯 C 请求；副作用仅由实时任务在控制边界执行。
bool motor_rt_submit_agent_trajectory(const agent_trajectory_request_t *request);
void motor_rt_copy_agent_trajectory(agent_trajectory_status_t *out);
bool motor_rt_agent_trajectory_owns(uint16_t motor_id);

#ifdef __cplusplus
}
#endif
#endif
