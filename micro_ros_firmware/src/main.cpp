// micro-ROS 入口（STM32H723VGT6, serial over USB CDC + FreeRTOS + IMU）
//
// 任务编排：
//   - systemInit() 统一完成板级硬件初始化（时钟/USB/状态灯）；
//   - start_task 创建 micro_ros / sensor / control 三个业务任务；
//   - vTaskStartScheduler() 接管调度后永不返回，loop() 仅作 Arduino 占位。
//
// 状态灯语义：上电红灯；micro-ROS 连接成功后绿灯 1Hz 闪烁；断连后蓝灯。
//
// 注释规范：中文写意图，micro-ROS / ROS 2 / FreeRTOS 等 API 保留英文。

#include <Arduino.h>
#include <HardwareSerial.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_platformio.h>

#include "ros_bridge.h"   // micro-ROS 节点生命周期
#include "supervisor_task.h"  // 开发期任务健康监督
#include "ws2812.h"       // 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7）
#include "sensor_task.h"   // 传感器任务（当前 BMI088 + Mahony + 温控 PID）
#include "control_task.h"  // 控制任务（当前三路 FDCAN 例程移植）

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "本固件仅支持 serial 传输，请在 platformio.ini 设置 board_microros_transport = serial"
#endif

// ---------------------------------------------------------------------------
// 板级配置
// ---------------------------------------------------------------------------
// micro-ROS 串口：USB CDC（PA11=USB_DM, PA12=USB_DP）。
// 启用 USB CDC 后 STM32duino 的 generic Serial 即映射为 SerialUSB（USBSerial，
// 继承 Stream），故用 auto& 引用以同时兼容 UART 与 USB CDC 两种后端。
static auto &kMicrorosSerial = Serial;
static const uint32_t kSerialBaud = 115200;  // USB CDC 忽略波特率，仅占位

static const uint32_t kStartTaskStack = 256;  // 开始任务
static const UBaseType_t kStartTaskPrio = 1;

static const uint32_t kMicroRosTaskStack = 8192;  // micro-ROS + RMW 需要较大栈
static const UBaseType_t kMicroRosTaskPrio = 2;
// 当前 board linker script 的 _Min_Stack_Size=0x400；STM32FreeRTOS 默认
// configMINIMAL_STACK_SIZE 语义为 0x400/8=128 words，timer task 为其两倍。
static const uint32_t kIdleTaskStackWords = 128U;
static const uint32_t kTimerTaskStackWords = 256U;

// configSUPPORT_STATIC_ALLOCATION=1 后，FreeRTOS 要求应用提供 idle/timer
// 内核任务的静态 TCB 与栈；业务任务仍沿用现有 xTaskCreate() 路径。
extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t **idle_tcb, StackType_t **idle_stack,
    uint32_t *idle_stack_size) {
  static StaticTask_t tcb;
  static StackType_t stack[kIdleTaskStackWords];
  *idle_tcb = &tcb;
  *idle_stack = stack;
  *idle_stack_size = kIdleTaskStackWords;
}

#if configUSE_TIMERS == 1
extern "C" void vApplicationGetTimerTaskMemory(
    StaticTask_t **timer_tcb, StackType_t **timer_stack,
    uint32_t *timer_stack_size) {
  static StaticTask_t tcb;
  static StackType_t stack[kTimerTaskStackWords];
  *timer_tcb = &tcb;
  *timer_stack = stack;
  *timer_stack_size = kTimerTaskStackWords;
}
#endif

// ---------------------------------------------------------------------------
// 统一硬件初始化
// ---------------------------------------------------------------------------
// 时钟（SystemClock_Config）、USB CDC、micro-ROS serial 传输、状态灯等板级
// 初始化在此一次性完成；IMU 初始化延后到调度器启动后的 sensor_task 内执行。
static void systemInit() {
  // 状态灯：初始化 SPI6（PA7=MOSI）驱动 WS2812，初始红灯
  ws2812_init();
  ws2812_set_rgb(1, 0, 0);  // 上电红灯

  kMicrorosSerial.begin(kSerialBaud);  // serial 传输：初始化 USB CDC
  set_microros_serial_transports(kMicrorosSerial);

  delay(1000);  // 等 USB 枚举稳定后再启动任务
}

// ---------------------------------------------------------------------------
// micro-ROS 任务：连接状态机（等 agent → 建实体 → 收发 → 断连清理）
// ---------------------------------------------------------------------------
enum class LinkState { kWaitAgent, kConnect, kRun, kLost };
static LinkState g_state = LinkState::kWaitAgent;
static bool g_ever_connected = false;  // 曾连上 agent：断连后保持蓝灯而非回到红灯

// 灯态跟随状态机：红=未连接，绿=连接成功（1Hz 闪烁），蓝=断连等待重连
static void update_link_led() {
  switch (g_state) {
    case LinkState::kWaitAgent:
      ws2812_set_rgb(g_ever_connected ? 0 : 1, 0, g_ever_connected ? 1 : 0);
      break;
    case LinkState::kConnect:
      break;  // 短暂建链态，保持上一颜色
    case LinkState::kRun:
      // 500ms 亮 / 500ms 灭，即 1Hz 绿灯闪烁
      ws2812_set_rgb(0, ((millis() / 500u) & 1u) ? 1 : 0, 0);
      break;
    case LinkState::kLost:
      ws2812_set_rgb(0, 0, 1);
      break;
  }
}

static void micro_ros_task(void *arg) {
  (void)arg;
  while (1) {
    switch (g_state) {
      case LinkState::kWaitAgent:
        // 探测 agent（200ms 超时 × 3 次）；agent 未启动时在此轮询等待
        if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
          g_state = LinkState::kConnect;
        }
        break;

      case LinkState::kConnect:
        if (ros_bridge_create()) {
          g_ever_connected = true;
          g_state = LinkState::kRun;
        } else {
          ros_bridge_destroy();  // 失败则清理，回等待态稍后重试
          g_state = LinkState::kWaitAgent;
        }
        break;

      case LinkState::kRun:
        // 心跳检测 + 服务会话：ping 探测 agent 存活，spin 驱动定时器发布 IMU。
        if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
          g_state = LinkState::kLost;  // 运行期 agent 丢失
        } else {
          ros_bridge_spin();
        }
        break;

      case LinkState::kLost:
      default:
        ros_bridge_destroy();
        g_state = LinkState::kWaitAgent;
        break;
    }
    update_link_led();
    // 表示连接状态机已完成一次迭代；不等同于 agent 已连接。
    supervisor_heartbeat(SUPERVISOR_TASK_MICRO_ROS);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 编排任务：micro-ROS 收发、传感器与控制任务相互独立，均在调度器启动后创建。
static void start_task(void *arg) {
  (void)arg;

  if (xTaskCreate(micro_ros_task, "micro_ros", kMicroRosTaskStack, nullptr,
                  kMicroRosTaskPrio, nullptr) != pdPASS) {
    Error_Handler();
  }
  if (!sensor_task_start()) {
    Error_Handler();
  }
  if (!control_task_start()) {
    Error_Handler();
  }
  if (!supervisor_task_start()) {
    Error_Handler();
  }
  // 编排完成后删除开始任务自身
  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Arduino 入口
// ---------------------------------------------------------------------------
void setup() {
  systemInit();  // 统一硬件初始化

  if (xTaskCreate(start_task, "start_task", kStartTaskStack, nullptr,
                  kStartTaskPrio, nullptr) != pdPASS) {
    Error_Handler();
  }

  // 启动调度器：接管 SysTick 后永不返回
  vTaskStartScheduler();

  // 理论上不可达；若 FreeRTOS 因配置错误返回此处，直接挂起
  while (1) {}
}

// Arduino 核心要求存在 loop()。STM32FreeRTOS 在 vApplicationIdleHook 里调用
// loop()，故这里只能放非阻塞代码（不得阻塞/延时，详见 STM32FreeRTOS.c）。
void loop() {}
