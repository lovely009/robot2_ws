#ifndef PUBLISH_SCHEDULE_H
#define PUBLISH_SCHEDULE_H

// 上行发布调度（纯逻辑）：决定某一拍该发哪条 stream、哪条 reliable。
// 基准节拍固定 5 ms = 200 拍/秒；频率是调度意图，不等于测量新鲜度或闭环保证。
// 调用者负责按输出的类发布，并把 spare_served 原样保存为下一拍的 last_spare。

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PUBLISH_SCHEDULE_TICK_MS 5U
#define PUBLISH_SCHEDULE_TICKS_PER_SECOND 200U
#define PUBLISH_SCHEDULE_STATUS_PERIOD 20U  // MotorStatus 10 Hz
#define PUBLISH_SCHEDULE_TEMP_TICK_A 49U    // 2 Hz 温度拍（每 200 拍两次）
#define PUBLISH_SCHEDULE_TEMP_TICK_B 149U

typedef enum {
  PUBLISH_STREAM_NONE = 0,
  PUBLISH_STREAM_LOW_STATE,  // 偶数拍 = 100 Hz
  PUBLISH_STREAM_IMU,        // tick%4==1 且非温度拍 = 48 Hz
  PUBLISH_STREAM_IMU_TEMP,   // tick%200==49/149 = 2 Hz
} publish_stream_t;

typedef enum {
  PUBLISH_RELIABLE_NONE = 0,
  PUBLISH_RELIABLE_MOTOR_STATUS,  // tick%20==0 = 10 Hz，固定优先
  PUBLISH_RELIABLE_EVENT,
  PUBLISH_RELIABLE_TOPOLOGY,
  PUBLISH_RELIABLE_DEVICE_PARAM,
} publish_reliable_t;

typedef struct {
  publish_stream_t stream;
  publish_reliable_t reliable;
  // 本拍结束后的轮转位置：常规拍等于本拍服务类；MotorStatus 拍不服务 spare，
  // 原样回传 last_spare（位置不变）。调用者始终把它存为下一拍 last_spare。
  publish_reliable_t spare_served;
} publish_schedule_output_t;

// 每拍调用一次。pending 标志由调用者在拍前提供；last_spare 是上一拍的
// spare_served。三类可靠消息按轮转选取，单类独占时下一可用拍即出。
publish_schedule_output_t publish_schedule_step(uint32_t tick,
    bool event_pending, bool topology_pending, bool device_param_pending,
    publish_reliable_t last_spare);

#ifdef __cplusplus
}
#endif

#endif  // PUBLISH_SCHEDULE_H
