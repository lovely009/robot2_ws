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

static TaskHandle_t g_micro_ros_task_handle = nullptr;
static micro_ros_link_state_t g_link_state = MICRO_ROS_LINK_WAIT_AGENT;
static bool g_ever_connected = false;

static void micro_ros_task_fn(void *parameters) {
  (void)parameters;
  for (;;) {
    micro_ros_link_state_t state = MICRO_ROS_LINK_WAIT_AGENT;
    taskENTER_CRITICAL();
    state = g_link_state;
    taskEXIT_CRITICAL();

    switch (state) {
      case MICRO_ROS_LINK_WAIT_AGENT:
        if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
          taskENTER_CRITICAL();
          g_link_state = MICRO_ROS_LINK_CONNECT;
          taskEXIT_CRITICAL();
        }
        break;

      case MICRO_ROS_LINK_CONNECT:
        if (ros_bridge_create()) {
          taskENTER_CRITICAL();
          g_ever_connected = true;
          g_link_state = MICRO_ROS_LINK_RUN;
          taskEXIT_CRITICAL();
        } else {
          ros_bridge_destroy();
          taskENTER_CRITICAL();
          g_link_state = MICRO_ROS_LINK_WAIT_AGENT;
          taskEXIT_CRITICAL();
        }
        break;

      case MICRO_ROS_LINK_RUN:
        if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
          taskENTER_CRITICAL();
          g_link_state = MICRO_ROS_LINK_LOST;
          taskEXIT_CRITICAL();
        } else {
          ros_bridge_spin();
        }
        break;

      case MICRO_ROS_LINK_LOST:
      default:
        ros_bridge_destroy();
        taskENTER_CRITICAL();
        g_link_state = MICRO_ROS_LINK_WAIT_AGENT;
        taskEXIT_CRITICAL();
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool micro_ros_task_start(void) {
  if (g_micro_ros_task_handle != nullptr) {
    return true;
  }
  return xTaskCreate(micro_ros_task_fn, "micro_ros", kMicroRosTaskStack,
                     nullptr, kMicroRosTaskPriority,
                     &g_micro_ros_task_handle) == pdPASS;
}

micro_ros_link_state_t micro_ros_task_get_link_state(void) {
  micro_ros_link_state_t state;
  taskENTER_CRITICAL();
  state = g_link_state;
  taskEXIT_CRITICAL();
  return state;
}

bool micro_ros_task_ever_connected(void) {
  bool ever_connected;
  taskENTER_CRITICAL();
  ever_connected = g_ever_connected;
  taskEXIT_CRITICAL();
  return ever_connected;
}