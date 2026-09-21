// 传感器业务任务（移植自官方 CtrBoard-H7_IMU_TempCtrl 编排）：osDelay(500) →
// heater_init() 启 TIM3 PWM → BMI088_init() → 等 ACC_INT(PE10) → BMI088_read →
// heater PID → CCR4。本工程改用 Arduino attachInterrupt()，加热 PWM 在 heater 模块。
// 后续其他传感器也在此任务编排。

#include "sensor_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "imu.h"
#include "heater.h"
#include "imu_snapshot.h"

// 任务参数：栈 512 字（2KB），优先级 2（与 micro_ros 收发任务相同，低于 motor_rt）
// IMU 为非关键路径（姿态/温度仅上 ROS）：优先级降到 motor_rt 之下，消除对
// 1ms 控制节拍的抢占；栈减半，台架 TBD-RT-01 复测，溢出由 stack overflow
// hook（main.cpp）兜底。功能不变：DRDY 采样/Mahony/PID/加热/发布原样。
static const uint32_t kSensorTaskStack = 512;
static const UBaseType_t kSensorTaskPrio = 2;
static const uint16_t kSensorStartDelayMs = 500;
static const uint16_t kImuInitRetryMs = 1000;

static SemaphoreHandle_t g_imu_sem = nullptr;
static imu_snapshot_t g_imu_snapshot = {};
static bool g_drdy_attached = false;
static heater_pid_state_t g_heater_pid = {};

static void accel_isr() {
  BaseType_t higher_priority_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(g_imu_sem, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void sensor_task_fn(void *arg) {
  (void)arg;

  // 官方 TempCtrl 任务先延时 500ms，再启动加热 PWM
  vTaskDelay(pdMS_TO_TICKS(kSensorStartDelayMs));
  heater_init();

  // 初始化失败不阻塞电机域：每秒单次重试并让出调度机会。
  while (!imu_try_init()) {
    heater_set_duty(0U);
    vTaskDelay(pdMS_TO_TICKS(kImuInitRetryMs));
  }

  // 仅在首次成功后挂 EXTI，重试不会重复注册中断。
  if (!g_drdy_attached) {
    attachInterrupt(PE10, accel_isr, RISING);
    g_drdy_attached = true;
  }

  while (1) {
    if (xSemaphoreTake(g_imu_sem, portMAX_DELAY) == pdTRUE && imu_ready()) {
      if (!imu_update()) {
        heater_set_duty(0U);
        continue;
      }
      // 写非活动缓冲不影响 ROS 读取；只在切换索引/序号时关闭中断。
      imu_data_t *next = imu_snapshot_begin_write(&g_imu_snapshot);
      imu_get_data(next);
      taskENTER_CRITICAL();
      imu_snapshot_publish(&g_imu_snapshot);
      taskEXIT_CRITICAL();
      const float ccr = heater_pid_update(&g_heater_pid, next->temp);
      heater_set_duty(static_cast<uint16_t>(ccr));
    }
  }
}

bool sensor_task_start(void) {
  if (g_imu_sem != nullptr) {
    return true;
  }

  g_imu_sem = xSemaphoreCreateBinary();
  if (g_imu_sem == nullptr) {
    return false;
  }
  // xSemaphoreCreateBinary 初始为空；首个 imu_update 必须由 ACC_INT DRDY 唤醒。
  imu_snapshot_init(&g_imu_snapshot);

  // 共享 DMA 控制器资源已由 main 初始化；BMI088 的阻塞 SPI2 配置归
  // lib/imu 自持，当前不占用 DMA。
  return xTaskCreate(sensor_task_fn, "sensor", kSensorTaskStack, nullptr,
                     kSensorTaskPrio, nullptr) == pdPASS;
}

bool sensor_task_get_imu_snapshot(imu_data_t *out, uint32_t *sequence) {
  bool copied = false;
  taskENTER_CRITICAL();
  copied = imu_snapshot_copy(&g_imu_snapshot, out, sequence);
  taskEXIT_CRITICAL();
  return copied;
}

