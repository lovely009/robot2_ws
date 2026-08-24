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
#include "imu_bridge.h"

// 任务参数：栈 1024 字（4KB），优先级高于 micro_ros 收发任务
static const uint32_t kSensorTaskStack = 1024;
static const UBaseType_t kSensorTaskPrio = 4;
static const uint16_t kSensorStartDelayMs = 500;

// 温度 PID（官方 imu_temp_ctrl.c 原值）
static const float kDesTemp = 40.0f;
static const float kPidKp = 100.0f;
static const float kPidKi = 50.0f;
static const float kPidKd = 10.0f;
static const float kMaxOut = 500.0f;

static SemaphoreHandle_t g_imu_sem = nullptr;
static volatile bool g_force_stop = false;
static imu_data_t g_imu_shared = {};

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

  // BMI088 初始化会阻塞重试；此时调度器已运行，vTaskDelay 合法
  imu_init();

  // 传感器就绪后再挂 EXTI，避免初始化期间的空信号量
  attachInterrupt(PE10, accel_isr, RISING);
  attachInterrupt(PE12, gyro_isr, RISING);

  // 供 micro-ROS 定时器读取最新样本；agent 未连接时静默跳过发布
  imu_bridge_set_source(&g_imu_shared);

  while (1) {
    if (xSemaphoreTake(g_imu_sem, portMAX_DELAY) == pdTRUE && imu_ready()) {
      imu_update();
      imu_get_data(&g_imu_shared);
      temp_pid_run(g_imu_shared.temp);
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
  // 官方 TempCtrl 用 vSemaphoreCreateBinary 创建，初始即为可用状态；
  // 现代 xSemaphoreCreateBinary 初始为空，补一次 Give 保持同样首拍行为。
  xSemaphoreGive(g_imu_sem);

  return xTaskCreate(sensor_task_fn, "sensor", kSensorTaskStack, nullptr,
                     kSensorTaskPrio, nullptr) == pdPASS;
}

void imu_set_force_stop(bool stop) {
  g_force_stop = stop;
}
