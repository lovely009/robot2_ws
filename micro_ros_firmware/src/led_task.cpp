#include "led_task.h"

// ---- host 可测：链路状态到颜色（不依赖 Arduino/FreeRTOS）----
led_color_t led_status_select(micro_ros_link_state_t state,
    bool ever_connected, bool blink_on,
    led_color_t previous) {
  switch (state) {
  case MICRO_ROS_LINK_CONNECT:
    return previous;
  case MICRO_ROS_LINK_RUN:
    return led_color_t{
      0U, static_cast<uint8_t>(blink_on ? 1U : 0U), 0U};
  case MICRO_ROS_LINK_LOST:
    return led_color_t{0U, 0U, 1U};
  case MICRO_ROS_LINK_WAIT_AGENT:
  default:
    return ever_connected ? led_color_t{0U, 0U, 1U}
                            : led_color_t{1U, 0U, 0U};
  }
}

// ---- WS2812 任务：板级依赖，host 单元测试编译时经 -DHOST_TEST 排除 ----
#ifndef HOST_TEST
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "micro_ros_task.h"
#include "ws2812.h"

static const uint32_t kLedTaskStack = 256U;
static const UBaseType_t kLedTaskPriority = 1U;
static const TickType_t kLedTaskPeriod = pdMS_TO_TICKS(50U);

static TaskHandle_t g_led_task_handle = nullptr;

static void led_task_fn(void *parameters) {
  (void)parameters;
  TickType_t last_wake = xTaskGetTickCount();
  led_color_t previous = {0xFFU, 0xFFU, 0xFFU};

  for (;;) {
    const led_color_t selected = led_status_select(
        micro_ros_task_get_link_state(), micro_ros_task_ever_connected(),
        ((millis() / 500U) & 1U) != 0U, previous);
    if (selected.r != previous.r || selected.g != previous.g ||
        selected.b != previous.b) {
      // 非阻塞提交：忙（上一帧 DMA 未完）时保留 previous，下个 50ms 周期重试
      if (ws2812_submit_rgb(selected.r, selected.g, selected.b)) {
        previous = selected;
      }
    }
    vTaskDelayUntil(&last_wake, kLedTaskPeriod);
  }
}

bool led_task_start(void) {
  if (g_led_task_handle != nullptr) {
    return true;
  }
  return xTaskCreate(led_task_fn, "led", kLedTaskStack, nullptr,
      kLedTaskPriority, &g_led_task_handle) == pdPASS;
}
#endif  // HOST_TEST
