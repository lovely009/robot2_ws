// micro-ROS 固件入口：初始化板级资源，创建业务任务并启动 FreeRTOS。

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_platformio.h>

#include "dma.h"
#include "led_task.h"
#include "micro_ros_task.h"
#include "serial_transport.h"
#include "motor_rt_task.h"
#include "sensor_task.h"
#include "storage_task.h"
#include "ws2812.h"
#include "board_irq.h"

// 防回归：栈溢出检测必须经项目级 STM32FreeRTOSConfig.h 生效；若该文件被
// 删除或未被 __has_include 命中，此处直接编译失败而非静默降级。
#if configCHECK_FOR_STACK_OVERFLOW != 2
#error "configCHECK_FOR_STACK_OVERFLOW must be 2 (see include/STM32FreeRTOSConfig.h)"
#endif
static_assert(BOARD_ISR_SYSCALL_THRESHOLD ==
    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
    "board_irq.h 阈值必须与 FreeRTOS syscall 门槛一致");

static const uint32_t START_STK_SIZE = 256U;
static const UBaseType_t START_TASK_PRIO = 1U;
static TaskHandle_t StartTask_Handler = nullptr;

static auto &kMicroRosSerial = Serial;
static const uint32_t kSerialBaud = 115200U;
static const uint32_t kIdleTaskStackWords = 128U;
static const uint32_t kTimerTaskStackWords = 256U;

// FreeRTOS 配置为静态分配 idle task 内存；缓冲区必须具有静态生命周期，避免调度器运行后依赖堆分配。
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
// 软件定时器启用时，由应用提供 timer service task 的静态控制块和栈。
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

// 栈溢出 fail-safe（configCHECK_FOR_STACK_OVERFLOW=2，STM32FreeRTOSConfig.h）：
// 栈已不可信，只记录任务名指针供 GDB 观察后关中断停机；不得调用任何依赖
// 栈深度或内核服务的函数（Serial/WS2812 均不可用）。
static const char *volatile g_stack_overflow_task = nullptr;

extern "C" void vApplicationStackOverflowHook(TaskHandle_t pxTask,
    char *pcTaskName) {
  (void)pxTask;
  g_stack_overflow_task = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
void initialize_system(void) {
  // 板级 DMA/BDMA 控制器资源（时钟+NVIC，lib/dma）先于一切设备传输初始化；
  // 设备配置归各设备库自持（lib/ws2812 SPI6+BDMA、lib/imu SPI2），
  // 「DMA 先于设备初始化」的官方时序由本函数保证。
  MX_DMA_Init();

  // 状态灯必须在 led task 启动前完成底层初始化；运行期仅由 led task 写入。
  ws2812_init();

  // micro-ROS 使用 USB CDC 暴露的 Serial 作为串行传输。USB CDC 下波特率参数不决定物理链路速率，但 begin() 仍用于启动串口设备。
  kMicroRosSerial.begin(kSerialBaud);
  serial_transport_setup(kMicroRosSerial);

  // 给 USB 枚举和主机侧串口就绪预留时间；此处调度器尚未启动，可使用 delay()。
  delay(1000U);
}

static void start_task(void *pv_parameters);

static void start_task(void *pv_parameters) {
  (void)pv_parameters;

  // 先创建状态指示与通信任务，再启动传感器和控制任务。各 start 函数只负责创建长期运行的业务任务；任一创建失败都进入统一故障处理，避免部分系统运行。
  if (!led_task_start()) {
    Error_Handler();
  }
  if (!micro_ros_task_start()) {
    Error_Handler();
  }
  if (!sensor_task_start()) {
    Error_Handler();
  }
  // 存储任务先于 motor_rt：同步加载 W25 blob（无芯片则降级），motor_rt
  // 启动时快照 boot 参数。Flash 擦除只在 storage(P1) 任务内发生。
  if (!storage_task_start()) {
    Error_Handler();
  }
  if (!motor_rt_task_start()) {
    Error_Handler();
  }

  // 子任务创建完成后释放一次性的启动任务资源。
  vTaskDelete(StartTask_Handler);
}

void setup() {
  // setup() 只完成不依赖 FreeRTOS 调度器的板级初始化。BMI088 等内部使用vTaskDelay 的初始化必须留在对应业务任务中执行。
  initialize_system();

  // 通过低优先级启动任务统一创建业务任务，确保其初始化发生在调度器启动后。
  if (xTaskCreate(start_task, "start_task", START_STK_SIZE, nullptr,
      START_TASK_PRIO, &StartTask_Handler) != pdPASS) {
    Error_Handler();
  }

  vTaskStartScheduler();

  // 正常情况下调度器不会返回；若返回则停留在此处，禁止继续执行未受调度的业务。
  for (;;) {}
}

// STM32FreeRTOS 将 Arduino loop() 作为 idle hook 调用，因此必须保持非阻塞且不承载业务逻辑；所有周期性工作均由独立任务负责。
void loop() {
}
