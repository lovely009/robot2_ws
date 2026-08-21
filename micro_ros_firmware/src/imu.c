// IMU 业务聚合模块：BMI088 读取 + Mahony 姿态 + 温度 PID 加热。
// 温度控制从例程 imu_task.c 移植：TIM3_CH4(PB1) 输出 PWM 驱动加热，闭环恒温。
// unit：gyro °/s→rad/s、accel g→m/s^2，供 micro-ROS 按 sensor_msgs 发布。

#include "imu.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "stm32h7xx_hal.h"
#include "bmi088_driver.h"
#include "MahonyAHRS.h"

#define IMU_SAMPLE_PERIOD_MS 5  // 与 MahonyAHRS sampleFreq=200 对齐

// 温度 PID 参数（例程 imu_task.c 原值）
#define DES_TEMP 40.0f
#define PID_KP 100.0f
#define PID_KI 50.0f
#define PID_KD 10.0f
#define MAX_OUT 500

static float g_gyro[3];
static float g_accel[3];
static float g_temp;
static float g_quat[4];
static float g_angle[3];
static volatile bool g_ready = false;

static float g_err = 0, g_err_l = 0, g_err_ll = 0;
static TIM_HandleTypeDef g_htim3;

static void heater_init(void);
static void temp_pid_run(float temp);

void imu_init(void) {
  heater_init();

  // 自旋等待传感器完成初始化（例程 while(BMI088_init()) 行为）
  while (BMI088_init() != BMI088_NO_ERROR) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  g_quat[0] = 1.0f;
  g_quat[1] = g_quat[2] = g_quat[3] = 0.0f;
  g_ready = true;
}

void imu_update(void) {
  if (!g_ready) {
    vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    return;
  }

  BMI088_read(g_gyro, g_accel, &g_temp);

  // 姿态解算（IMU 模式：陀螺 + 加速度，无磁力计）
  MahonyAHRSupdateIMU(g_quat, g_gyro[0], g_gyro[1], g_gyro[2], g_accel[0],
                      g_accel[1], g_accel[2]);

  // 四元数 → 欧拉角（yaw/pitch/roll，rad）
  g_angle[0] = atan2f(2.0f * (g_quat[0] * g_quat[3] + g_quat[1] * g_quat[2]),
                      2.0f * (g_quat[0] * g_quat[0] + g_quat[1] * g_quat[1]) - 1.0f);
  g_angle[1] = asinf(-2.0f * (g_quat[1] * g_quat[3] - g_quat[0] * g_quat[2]));
  g_angle[2] = atan2f(2.0f * (g_quat[0] * g_quat[1] + g_quat[2] * g_quat[3]),
                      2.0f * (g_quat[0] * g_quat[0] + g_quat[3] * g_quat[3]) - 1.0f);

  temp_pid_run(g_temp);
  vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
}

void imu_get_data(imu_data_t *out) {
  // 单位换算：°/s → rad/s；g → m/s^2
  const float kDeg2Rad = 3.14159265358979f / 180.0f;
  const float kGravity = 9.80665f;
  for (int i = 0; i < 3; i++) {
    out->gyro[i] = g_gyro[i] * kDeg2Rad;
    out->accel[i] = g_accel[i] * kGravity;
    out->quat[i] = g_quat[i];
    out->angle[i] = g_angle[i];
  }
  out->quat[3] = g_quat[3];
  out->temp = g_temp;
}

bool imu_ready(void) { return g_ready; }

static void heater_init(void) {
  // TIM3_CH4 = PB1（例程预设 PB1，PSC=24-1, ARR=10000-1）
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &gpio);

  g_htim3.Instance = TIM3;
  g_htim3.Init.Prescaler = 24 - 1;
  g_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  g_htim3.Init.Period = 10000 - 1;
  g_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&g_htim3) != HAL_OK) {
    Error_Handler();
  }
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&g_htim3, &oc, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(&g_htim3, TIM_CHANNEL_4);
}

static void temp_pid_run(float temp) {
  float err = DES_TEMP - temp;
  float out = PID_KP * err + PID_KI * (err + g_err_l + g_err_ll) +
              PID_KD * (err - g_err_l);
  g_err_ll = g_err_l;
  g_err_l = g_err;
  g_err = err;
  if (out > MAX_OUT) out = MAX_OUT;
  if (out < 0) out = 0.0f;
  g_htim3.Instance->CCR4 = (uint16_t)out;
}
