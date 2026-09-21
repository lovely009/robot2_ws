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

// 任务栈**静态分配**：本工程 FreeRTOS 用 heap_useNewlib_ST，`pvPortMalloc` 与
// newlib `malloc` 共用同一个 DTCM 堆，而那个堆池总共只有 120 KiB
// （0x20000000 → _estack−8 KiB，DTCM 就 128 KiB，**不可能再扩**），任务栈与
// 堆上的常驻项互相挤。把这块 32 KiB 搬到 RAM_D1 的 .bss 就是为了给堆腾地方。
// *（历史：当初腾的对象是 rclc 参数槽——72 槽常驻 23.6 KB。参数服务器已于
// 2026-09-21 整体退役、参数面现在占 0 字节常驻堆，**但静态分配本身保留**：
// 它不占 DTCM，也没有理由搬回去。）*
// 放普通 .bss 而不是 DTCM：本任务不实时，走带 D-cache 的 AXI SRAM 没有影响；
// motor_rt 的栈保持动态分配，继续留在 DTCM 的 0-wait 上。
static StackType_t g_micro_ros_task_stack[kMicroRosTaskStack];
static StaticTask_t g_micro_ros_task_tcb;

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
  g_micro_ros_task_handle =
      xTaskCreateStatic(micro_ros_task_fn, "micro_ros", kMicroRosTaskStack,
                        nullptr, kMicroRosTaskPriority, g_micro_ros_task_stack,
                        &g_micro_ros_task_tcb);
  return g_micro_ros_task_handle != nullptr;
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