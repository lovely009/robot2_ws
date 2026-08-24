#ifndef SUPERVISOR_HEARTBEAT_H
#define SUPERVISOR_HEARTBEAT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// control 首次心跳之后的运行期最大允许无进展时间。
#define SUPERVISOR_CONTROL_RUNTIME_TIMEOUT_MS (100U)

// 单一任务的最近进展观测值。该结构仅由 supervisor task 写入。
typedef struct {
  uint32_t last_heartbeat;
  uint32_t last_progress_ms;
  bool observed;
} supervisor_heartbeat_state_t;

// 心跳为 0 时保持未观测，避免正常任务启动延时被误判。首个非零心跳记录
// 进展时刻；其后在未推进超过 timeout_ms 后报告 stalled。
// uint32_t 相减定义了 millis() 回绕后的经过时间。
bool supervisor_heartbeat_evaluate(supervisor_heartbeat_state_t *state,
                                   uint32_t heartbeat, uint32_t now_ms,
                                   uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif  // SUPERVISOR_HEARTBEAT_H
