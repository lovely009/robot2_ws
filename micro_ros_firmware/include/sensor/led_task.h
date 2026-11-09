// WS2812 状态灯任务：按 micro-ROS 连接状态选颜色（策略函数 host 可测，
// 见 led_status_select；物理发送在 lib/board（WS2812 驱动）。
#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "micro_ros_link_state.h"

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} led_color_t;

#ifdef __cplusplus
extern "C" {
#endif

led_color_t led_status_select(micro_ros_link_state_t state,
    bool ever_connected, bool blink_on,
    led_color_t previous);

bool led_task_start(void);

#ifdef __cplusplus
}
#endif

#endif
