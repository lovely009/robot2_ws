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
  // 固定 10ms 节拍（100Hz）。vTaskDelay(10ms) 的语义是"做完本轮事再睡 10ms"，
  // 实际周期 = 本轮工作耗时 + 10ms ≈ 11ms → 有效节拍 ~90Hz；而 motion_states
  // 取 tick/2、imu 取 tick/1，于是一个报成 ~45Hz（文档标称 50Hz）、另一个 ~91Hz。
  // vTaskDelayUntil 以**上一次唤醒时刻**为基准推进，把工作耗时吸收进睡眠，
  // 周期恒为 10ms。motor_rt 的 1ms 服务用的是同一模式。
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);
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
    // 超时保护：ping 在链路异常时会阻塞到数百 ms（WAIT_AGENT/LOST 态下
    // 200ms×3 次尝试），此时 last_wake 已远落后于当前时刻。继续调
    // vTaskDelayUntil 会**连续立即返回**去追帧（空转），饿死低优先级任务；
    // 本任务优先级仅 2。落后达到一个周期就重新对齐，放弃追帧。
    if ((xTaskGetTickCount() - last_wake) < period) {
      vTaskDelayUntil(&last_wake, period);
    } else {
      // 重新对齐必须在**睡完之后**取时刻：若在 vTaskDelay 之前重置，下一轮
      // 的差值又会 ≥ 周期，于是永远走这一支、退化成原来的「工作耗时 + 10ms」，
      // 节拍与修复前无异（2026-10-13 首次修复即踩此坑，实测 imu 仍 90.5Hz）。
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