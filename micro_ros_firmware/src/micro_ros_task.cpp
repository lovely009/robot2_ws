// micro-ROS 连接管理任务实现：四态机见 micro_ros_task.h。
#include "micro_ros_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_platformio.h>

#include "ros_bridge.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "本固件仅支持 serial 传输，请在 platformio.ini 设置 board_microros_transport = serial"
#endif

static const uint32_t kMicroRosTaskStack = 8192U;
static const UBaseType_t kMicroRosTaskPriority = 2U;

// 任务栈**静态分配**：本工程 FreeRTOS 用 heap_useNewlib_ST，pvPortMalloc 与 newlib
// malloc 共用同一个 DTCM 堆（120 KiB，物理不可扩），任务栈与堆上常驻项互相挤。
// 这 32 KiB 放普通 .bss（RAM_D1）给堆腾地方：本任务不实时，走带 D-cache 的 AXI SRAM
// 无影响；motor_rt 的栈保持动态分配，继续留在 DTCM 的 0-wait 上。
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
        }
        break;

      case MICRO_ROS_LINK_LOST:
      default:
        ros_bridge_destroy();
        link_state_set(MICRO_ROS_LINK_WAIT_AGENT);
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
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