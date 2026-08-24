// 时间戳工具实现：见 time_utils.h。

#include "time_utils.h"

#include <Arduino.h>
#include <rmw_microros/time_sync.h>

void fill_header_stamp(builtin_interfaces__msg__Time *stamp) {
  // 优先使用同步后的 epoch 时间；未同步时 rmw_uros_epoch_nanos() 返回 0，
  // 此时回退到 millis()（MCU uptime），避免发出 0 时间戳。
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (now_ns != 0) {
    stamp->sec = (int32_t)(now_ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(now_ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    stamp->sec = (int32_t)(ms / 1000u);
    stamp->nanosec = (uint32_t)((ms % 1000u) * 1000000u);
  }
}
