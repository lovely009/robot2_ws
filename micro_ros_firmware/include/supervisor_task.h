// 开发期软件任务监督：只观察三个业务状态机的进展，不执行复位或 IWDG 操作。

#ifndef SUPERVISOR_TASK_H
#define SUPERVISOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SUPERVISOR_TASK_CONTROL = 0,
  SUPERVISOR_TASK_SENSOR = 1,
  SUPERVISOR_TASK_MICRO_ROS = 2,
  SUPERVISOR_TASK_COUNT = 3
} supervisor_task_id_t;

enum {
  SUPERVISOR_FAULT_CONTROL_MISSED = (1u << 0),
  SUPERVISOR_FAULT_SENSOR_MISSED = (1u << 1),
  SUPERVISOR_FAULT_MICRO_ROS_STALLED = (1u << 2),
  SUPERVISOR_FAULT_IMU_DEGRADED = (1u << 3),
  SUPERVISOR_FAULT_FDCAN_NOT_READY = (1u << 4),
};

typedef struct {
  uint32_t fault_bitmask;
  uint32_t heartbeat[SUPERVISOR_TASK_COUNT];
  uint32_t miss_count[SUPERVISOR_TASK_COUNT];
  uint32_t last_check_ms;
} supervisor_diagnostics_t;

// 创建 100 ms、优先级 1 的监督任务；重复调用不重复建任务。
bool supervisor_task_start(void);

// 由任务在其状态机完成一次可观察进展时调用。ISR 不可调用。
void supervisor_heartbeat(supervisor_task_id_t task);

// 在短临界区内复制，提供给 GDB 或后续诊断消费方的一致快照。
void supervisor_get_diagnostics(supervisor_diagnostics_t *out);

#ifdef __cplusplus
}
#endif

#endif  // SUPERVISOR_TASK_H
