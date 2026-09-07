// 传感器业务任务（官方 CtrBoard-H7_IMU_TempCtrl 任务编排移植）。
//
// 官方流程：osDelay(500) → 启动 TIM3 PWM → BMI088_init() →
// 等 ACC_INT(PE10) 二值信号量 → BMI088_read → 温度 PID → CCR4。
// 本工程使用 Arduino attachInterrupt()/HardwareTimer，并保留 micro-ROS 发布；
// 任务命名为 sensor，后续其他传感器也在此任务编排。

#include "sensor_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>

#include "imu.h"
#include "imu_snapshot.h"

// 任务参数：栈 1024 字（4KB），优先级高于 micro_ros 收发任务
// IMU 为非关键路径（姿态/温度仅上 ROS）：优先级降到 motor_rt 之下，消除对
// 1ms 控制节拍的抢占；栈减半，台架 TBD-RT-01 复测，溢出由 stack overflow
// hook（main.cpp）兜底。功能不变：DRDY 采样/Mahony/PID/加热/发布原样。
static const uint32_t kSensorTaskStack = 512;
static const UBaseType_t kSensorTaskPrio = 2;
static const uint16_t kSensorStartDelayMs = 500;
static const uint16_t kImuInitRetryMs = 1000;

// 温度 PID（官方 imu_temp_ctrl.c 原值）
static const float kDesTemp = 40.0f;
static const float kPidKp = 100.0f;
static const float kPidKi = 50.0f;
static const float kPidKd = 10.0f;
static const float kMaxOut = 500.0f;

static SemaphoreHandle_t g_imu_sem = nullptr;
static volatile bool g_force_stop = false;
static imu_snapshot_t g_imu_snapshot = {};
static volatile uint32_t g_imu_init_error_count = 0U;
static volatile bool g_imu_healthy = false;
static bool g_drdy_attached = false;

static HardwareTimer &heater_timer() {
  static HardwareTimer tim3(TIM3);
  return tim3;
}

static void heater_init() {
  // 官方 TIM3：PSC=24-1、ARR=10000-1，APB1=120MHz → PWM 500Hz，初始占空比 0
  heater_timer().setPWM(4, PB1_ALT1, 500, 0);
}

static void heater_set_duty(uint16_t duty) {
  heater_timer().setCaptureCompare(4, duty);
}

static void accel_isr() {
  BaseType_t higher_priority_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(g_imu_sem, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void gyro_isr() {
  // 官方仅配置 GYRO_INT(PE12)，回调留空
}

static void temp_pid_run(float temp) {
  static float err = 0.0f;
  static float err_l = 0.0f;
  static float err_ll = 0.0f;

  float out = kPidKp * (kDesTemp - temp) +
              kPidKi * ((kDesTemp - temp) + err_l + err_ll) +
              kPidKd * ((kDesTemp - temp) - err_l);
  err_ll = err_l;
  err_l = err;
  err = kDesTemp - temp;

  if (out > kMaxOut) {
    out = kMaxOut;
  }
  if (out < 0.0f) {
    out = 0.0f;
  }
  if (g_force_stop) {
    out = 0.0f;
  }
  heater_set_duty(static_cast<uint16_t>(out));
}

static void sensor_task_fn(void *arg) {
  (void)arg;

  // 官方 TempCtrl 任务先延时 500ms，再启动加热 PWM
  vTaskDelay(pdMS_TO_TICKS(kSensorStartDelayMs));
  heater_init();

  // 初始化失败不阻塞电机域：每秒单次重试并让出调度机会。
  while (!imu_try_init()) {
    g_imu_healthy = false;
    ++g_imu_init_error_count;
    heater_set_duty(0U);
    vTaskDelay(pdMS_TO_TICKS(kImuInitRetryMs));
  }
  g_imu_healthy = true;

  // 仅在首次成功后挂 EXTI，重试不会重复注册中断。
  if (!g_drdy_attached) {
    attachInterrupt(PE10, accel_isr, RISING);
    attachInterrupt(PE12, gyro_isr, RISING);
    g_drdy_attached = true;
  }

  while (1) {
    if (xSemaphoreTake(g_imu_sem, portMAX_DELAY) == pdTRUE && imu_ready()) {
      if (!imu_update()) {
        g_imu_healthy = false;
        heater_set_duty(0U);
        continue;
      }
      // 写非活动缓冲不影响 ROS 读取；只在切换索引/序号时关闭中断。
      imu_data_t *next = imu_snapshot_begin_write(&g_imu_snapshot);
      imu_get_data(next);
      taskENTER_CRITICAL();
      imu_snapshot_publish(&g_imu_snapshot);
      taskEXIT_CRITICAL();
      g_imu_healthy = true;
      temp_pid_run(next->temp);
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
  g_imu_init_error_count = 0U;
  g_imu_healthy = false;

  // DMA1 时钟/流 NVIC 已由 main 板级初始化（initialize_system）先行使能
  // （官方时序：DMA 先于 SPI2）；SPI2 设备配置归 lib/imu 自持。
  return xTaskCreate(sensor_task_fn, "sensor", kSensorTaskStack, nullptr,
                     kSensorTaskPrio, nullptr) == pdPASS;
}

void imu_set_force_stop(bool stop) {
  g_force_stop = stop;
}

bool sensor_task_get_imu_snapshot(imu_data_t *out, uint32_t *sequence) {
  bool copied = false;
  taskENTER_CRITICAL();
  copied = imu_snapshot_copy(&g_imu_snapshot, out, sequence);
  taskEXIT_CRITICAL();
  return copied;
}

uint32_t sensor_task_get_imu_init_error_count(void) {
  uint32_t count = 0U;
  taskENTER_CRITICAL();
  count = g_imu_init_error_count;
  taskEXIT_CRITICAL();
  return count;
}

bool sensor_task_imu_healthy(void) {
  bool healthy = false;
  taskENTER_CRITICAL();
  healthy = g_imu_healthy;
  taskEXIT_CRITICAL();
  return healthy;
}
