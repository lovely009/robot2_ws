#include "publish_schedule.h"

static publish_stream_t select_stream(uint32_t tick) {
  if ((tick & 1U) == 0U) {
    return PUBLISH_STREAM_LOW_STATE;
  }
  if ((tick & 3U) == 1U) {
    const uint32_t phase = tick % PUBLISH_SCHEDULE_TICKS_PER_SECOND;
    if (phase == PUBLISH_SCHEDULE_TEMP_TICK_A ||
        phase == PUBLISH_SCHEDULE_TEMP_TICK_B) {
      return PUBLISH_STREAM_IMU_TEMP;
    }
    return PUBLISH_STREAM_IMU;
  }
  return PUBLISH_STREAM_NONE;
}

static bool reliable_pending(publish_reliable_t kind, bool event_pending,
    bool topology_pending, bool device_param_pending) {
  switch (kind) {
    case PUBLISH_RELIABLE_EVENT: return event_pending;
    case PUBLISH_RELIABLE_TOPOLOGY: return topology_pending;
    case PUBLISH_RELIABLE_DEVICE_PARAM: return device_param_pending;
    default: return false;
  }
}

// 轮转只走过 pending 的类：last_spare 之后第一个 pending 即为本拍选择，
// 因此单类独占时不会空转；多类积压时按固定顺序轮转，长窗口内数量均衡。
// 调用者在 status 拍也必须回灌 spare_served，故本函数在 status 拍原样交回
// last_spare，轮转位置不因状态拍中断而清零。
static publish_reliable_t select_spare(bool event_pending,
    bool topology_pending, bool device_param_pending,
    publish_reliable_t last_spare) {
  static const publish_reliable_t kOrder[] = {
    PUBLISH_RELIABLE_EVENT,
    PUBLISH_RELIABLE_TOPOLOGY,
    PUBLISH_RELIABLE_DEVICE_PARAM,
  };
  uint32_t start = 0U;
  if (last_spare == PUBLISH_RELIABLE_EVENT) {
    start = 1U;
  } else if (last_spare == PUBLISH_RELIABLE_TOPOLOGY) {
    start = 2U;
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    const publish_reliable_t cand = kOrder[(start + i) % 3U];
    if (reliable_pending(cand, event_pending, topology_pending,
        device_param_pending)) {
      return cand;
    }
  }
  return PUBLISH_RELIABLE_NONE;
}

publish_schedule_output_t publish_schedule_step(uint32_t tick,
    bool event_pending, bool topology_pending, bool device_param_pending,
    publish_reliable_t last_spare) {
  publish_schedule_output_t out;
  out.stream = select_stream(tick);
  if (tick % PUBLISH_SCHEDULE_STATUS_PERIOD == 0U) {
    // 固定 10 Hz 状态优先；本拍不吃 spare 但保留轮转位置（原样交回 last_spare），
    // 积压最多顺延一拍，且不打断多类轮转。
    out.reliable = PUBLISH_RELIABLE_MOTOR_STATUS;
    out.spare_served = last_spare;
  } else {
    out.reliable = select_spare(event_pending, topology_pending,
        device_param_pending, last_spare);
    out.spare_served = out.reliable;
  }
  return out;
}
