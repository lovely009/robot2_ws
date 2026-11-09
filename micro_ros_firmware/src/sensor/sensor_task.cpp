// 传感器任务：延时启动加热，按 BMI088 DRDY 采样并更新姿态与温控。

#include "sensor_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "imu.h"
#include "heater.h"
#include "imu_snapshot.h"
#include "time_utils.h"

// 栈 512 字（2KB），优先级低于 motor_rt，避免抢占 1ms 控制节拍。
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

  // 延时 500ms 后启动加热 PWM。
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
      const uint32_t sample_tick = millis();
      uint64_t sample_epoch = 0;
      const bool clock_valid = time_sample_epoch(sample_tick, &sample_epoch);
      if (!imu_update()) {
        heater_set_duty(0U);
        continue;
      }
      // 写非活动缓冲不影响 ROS 读取；只在切换索引/序号时关闭中断。
      imu_data_t *next = imu_snapshot_begin_write(&g_imu_snapshot);
      imu_get_data(next);
      taskENTER_CRITICAL();
      // BMI088_read 每次成功更新都会实际读取温度寄存器，独立记录读取序号。
      imu_snapshot_publish_timed(&g_imu_snapshot, sample_tick, sample_epoch,
          clock_valid, true);
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
  // lib/board（IMU 驱动）自持，当前不占用 DMA。
  return xTaskCreate(sensor_task_fn, "sensor", kSensorTaskStack, nullptr,
      kSensorTaskPrio, nullptr) == pdPASS;
}

bool sensor_task_get_imu_sample(imu_data_t *out, sensor_sample_metadata_t *metadata) {
  taskENTER_CRITICAL();
  const bool copied = imu_snapshot_copy_sample(&g_imu_snapshot, out, metadata);
  taskEXIT_CRITICAL();
  return copied;
}
bool sensor_task_get_temperature(float *out, sensor_sample_metadata_t *metadata) {
  taskENTER_CRITICAL();
  const bool copied = imu_snapshot_copy_temperature(&g_imu_snapshot, out, metadata);
  taskEXIT_CRITICAL();
  return copied;
}

bool sensor_task_get_imu_snapshot(imu_data_t *out, uint32_t *sequence) {
  bool copied = false;
  taskENTER_CRITICAL();
  copied = imu_snapshot_copy(&g_imu_snapshot, out, sequence);
  taskEXIT_CRITICAL();
  return copied;
}

