// micro-ROS 连接管理任务实现：四态机见 micro_ros_task.h。
#include "micro_ros_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_platformio.h>

#include "ros_bridge.h"
#include "serial_transport.h"
#include "motor_rt_task.h"
#include "time_utils.h"
#include <rmw_microros/time_sync.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "本固件仅支持 serial 传输，请在 platformio.ini 设置 board_microros_transport = serial"
#endif

static const uint32_t kMicroRosTaskStack = 8192U;
static const UBaseType_t kMicroRosTaskPriority = 2U;

// 32 KiB 任务栈静态放入 RAM_D1 的 .bss，避免占用 pvPortMalloc/malloc
// 共用的 120 KiB DTCM 堆；motor_rt 动态栈仍位于 DTCM。
static StackType_t g_micro_ros_task_stack[kMicroRosTaskStack];
static StaticTask_t g_micro_ros_task_tcb;

static TaskHandle_t g_micro_ros_task_handle = nullptr;
static micro_ros_link_state_t g_link_state = MICRO_ROS_LINK_WAIT_AGENT;
static bool g_ever_connected = false;

// 链路状态由本任务写、led 任务与 ROS 桥读：进出都走短临界区。
static micro_ros_link_state_t link_state_get(void) {
  taskENTER_CRITICAL();
  const micro_ros_link_state_t state = g_link_state;
  taskEXIT_CRITICAL();
  return state;
}

static void link_state_set(micro_ros_link_state_t state) {
  taskENTER_CRITICAL();
  g_link_state = state;
  taskEXIT_CRITICAL();
}

static void micro_ros_task_fn(void *parameters) {
  (void)parameters;
  // 以上次唤醒时刻为基准维持 10ms 节拍，避免工作耗时累加到周期。
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last_sync = xTaskGetTickCount();
  for (;;) {
    switch (link_state_get()) {
    case MICRO_ROS_LINK_WAIT_AGENT:
      if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
        link_state_set(MICRO_ROS_LINK_CONNECT);
      }
      break;

    case MICRO_ROS_LINK_CONNECT:
      if (ros_bridge_create()) {
        // ever_connected 与状态在同一临界区发布：led_task 同时读这两个值。
        taskENTER_CRITICAL();
        g_ever_connected = true;
        g_link_state = MICRO_ROS_LINK_RUN;
        taskEXIT_CRITICAL();
        last_sync = xTaskGetTickCount();
      } else {
        ros_bridge_destroy();
        link_state_set(MICRO_ROS_LINK_WAIT_AGENT);
      }
      break;

    case MICRO_ROS_LINK_RUN:
      if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
        link_state_set(MICRO_ROS_LINK_LOST);
      } else {
        ros_bridge_spin();
        if ((TickType_t)(xTaskGetTickCount() - last_sync) >= pdMS_TO_TICKS(10000)) {
          // 校时结果显式进入健康门禁；旧非零 epoch 不代表同步仍健康。
          serial_transport_send_begin();
          const rmw_ret_t rc = rmw_uros_sync_session(2);
          serial_transport_send_end();
          time_sync_record(rc == RMW_RET_OK,
              rc == RMW_RET_OK ? rmw_uros_epoch_nanos() : 0);
          motor_rt_set_clock_valid(time_clock_valid());
          last_sync = xTaskGetTickCount();
        }
      }
      break;

    case MICRO_ROS_LINK_LOST:
    default:
      ros_bridge_destroy();
      link_state_set(MICRO_ROS_LINK_WAIT_AGENT);
      break;
    }
    // ping 最多等待 200ms × 3；落后一个周期时放弃追帧，避免饿死低优先级任务。
    if ((xTaskGetTickCount() - last_wake) < period) {
      vTaskDelayUntil(&last_wake, period);
    } else {
      // 必须在延时后重新取时刻，避免持续进入超时分支。
      vTaskDelay(period);
      last_wake = xTaskGetTickCount();
    }
  }
}

bool micro_ros_task_start(void) {
  if (g_micro_ros_task_handle != nullptr) {
    return true;
  }
  g_micro_ros_task_handle =
      xTaskCreateStatic(micro_ros_task_fn, "micro_ros", kMicroRosTaskStack,
      nullptr, kMicroRosTaskPriority, g_micro_ros_task_stack,
      &g_micro_ros_task_tcb);
  return g_micro_ros_task_handle != nullptr;
}

micro_ros_link_state_t micro_ros_task_get_link_state(void) {
  return link_state_get();
}

bool micro_ros_task_ever_connected(void) {
  bool ever_connected;
  taskENTER_CRITICAL();
  ever_connected = g_ever_connected;
  taskEXIT_CRITICAL();
  return ever_connected;
}