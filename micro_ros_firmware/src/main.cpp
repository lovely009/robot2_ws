// micro-ROS 入口（STM32H723VGT6, serial over USB CDC + FreeRTOS）
//
// 风格参考 STM32 例程的 gate 编排：
//   - systemInit() 统一完成板级硬件初始化
//   - 入口任务 micro_ros_task 先建立 micro-ROS 连接，连接成功后才派生业务任务
//   - vTaskStartScheduler() 接管调度后永不返回，loop() 仅作 Arduino 核心占位
//
// 注释规范：中文写意图，micro-ROS / ROS 2 / FreeRTOS API 保留英文。

#include <Arduino.h>
#include <HardwareSerial.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_platformio.h>

#include "ros_bridge.h"  // micro-ROS 节点生命周期（空工程骨架）
#include "ws2812.h"      // 板载 WS2812 彩灯驱动（SPI6 MOSI=PA7）
#include "imu.h"         // BMI088 IMU（姿态/温度）

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "本固件仅支持 serial 传输，请在 platformio.ini 设置 board_microros_transport = serial"
#endif

// ---------------------------------------------------------------------------
// 板级配置（单点可改，硬件映射只在这里改）
// ---------------------------------------------------------------------------
// micro-ROS 串口：USB CDC（PA11=USB_DM, PA12=USB_DP）。
// 启用 USB CDC 后 STM32duino 的 generic Serial 即映射为 SerialUSB（USBSerial，
// 继承 Stream），故用 auto& 引用以同时兼容 UART 与 USB CDC 两种后端。
static auto &kMicrorosSerial = Serial;
static const uint32_t kSerialBaud = 115200;  // USB CDC 忽略波特率，仅占位

// FreeRTOS 任务参数：micro-ROS 任务优先级高于业务任务，保证建链/收发不被饿死
static const uint32_t kMicroRosTaskStack = 8192;  // micro-ROS + RMW 需要较大栈
static const uint32_t kAppTaskStack = 2048;
static const UBaseType_t kMicroRosTaskPrio = 2;
static const UBaseType_t kAppTaskPrio = 1;

// ---------------------------------------------------------------------------
// 统一硬件初始化
// ---------------------------------------------------------------------------
// 时钟（SystemClock_Config）、USB CDC、micro-ROS serial 传输、状态灯等板级
// 初始化在此一次性完成；后续只负责任务编排。
static void systemInit() {
  // 状态灯：初始化 SPI6（PA7=MOSI）驱动 WS2812，初始熄灭
  ws2812_init();
  ws2812_set_rgb(0, 0, 0);

  // serial 传输：初始化 USB CDC 并绑定 micro-ROS（framing 模式）。
  kMicrorosSerial.begin(kSerialBaud);
  set_microros_serial_transports(kMicrorosSerial);

  delay(1000);  // 等 USB 枚举稳定后再启动任务
}

// ---------------------------------------------------------------------------
// 业务任务（IMU）
// ---------------------------------------------------------------------------
// 业务任务只负责采样，发布统一放到 micro-ROS 任务内，保证 XRCE 访问单线程。
static imu_data_t g_imu_shared;
static bool g_app_created = false;  // 连接成功后只派生一次业务任务

static void app_task(void *arg) {
  (void)arg;
  // 先注册数据源，再阻塞等待 BMI088 就绪：即使传感器未应答，定时器也持续
  // 发布（初始化期间为 0），保证 micro-ROS 会话与数据链路不断。
  ros_bridge_set_imu_source(&g_imu_shared);  // 定时器据此发布最新样本
  imu_init();  // 阻塞等待 BMI088 就绪；agent 任务优先级更高，不受影响
  while (1) {
    imu_update();  // 内部按 5ms 周期采样并更新姿态/温度
    if (imu_ready()) {
      imu_get_data(&g_imu_shared);
    }
  }
}

// ---------------------------------------------------------------------------
// micro-ROS 任务：连接状态机（等 agent → 建实体 → 收发 → 断连清理）
// ---------------------------------------------------------------------------
enum class LinkState { kWaitAgent, kConnect, kRun, kLost };
static LinkState g_state = LinkState::kWaitAgent;

// 开始函数：先等连接成功，再进入后续多任务调度（派生业务任务），随后持续收发
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
          // 连接成功后才进入多任务调度：派生一次业务任务（IMU 采样）
          if (!g_app_created) {
            if (xTaskCreate(app_task, "app", kAppTaskStack, nullptr,
                            kAppTaskPrio, nullptr) != pdPASS) {
              Error_Handler();
            }
            g_app_created = true;
          }
          ros_bridge_spin();
        }
        break;

      case LinkState::kLost:
      default:
        ros_bridge_destroy();
        g_state = LinkState::kWaitAgent;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------------------------------------------------------------------
// Arduino 入口
// ---------------------------------------------------------------------------
void setup() {
  systemInit();  // 统一硬件初始化

  // 开始函数：micro-ROS 建链，连接成功后再派生业务任务
  if (xTaskCreate(micro_ros_task, "micro_ros", kMicroRosTaskStack, nullptr,
                  kMicroRosTaskPrio, nullptr) != pdPASS) {
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
