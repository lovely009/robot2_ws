// IMU 传感器聚合模块：BMI088 读取 + Mahony 姿态解算。
// 加热 PWM 与温度 PID 在 sensor_task.cpp 中实现（官方 TempCtrl 任务层）。
// unit：gyro °/s→rad/s、accel g→m/s^2，供 micro-ROS 按 sensor_msgs 发布。

#include "imu.h"
#include <math.h>
#include "bmi088_driver.h"
#include "bmi088_middleware.h"
#include "MahonyAHRS.h"

static float g_gyro[3];
static float g_accel[3];
static float g_temp;
static float g_quat[4];
static float g_angle[3];
static volatile bool g_ready = false;

static void reset_attitude_state(void) {
  for (int i = 0; i < 3; ++i) {
    g_gyro[i] = 0.0f;
    g_accel[i] = 0.0f;
    g_angle[i] = 0.0f;
  }
  g_temp = 0.0f;
  g_quat[0] = 1.0f;
  g_quat[1] = g_quat[2] = g_quat[3] = 0.0f;
}

bool imu_try_init(void) {
  // BMI088_delay_ms 内部使用 vTaskDelay，因此本函数只能在调度器启动后调用。
  // 每次调用只尝试一次，重试节奏由 sensor_task 管理。
  g_ready = false;
  if (BMI088_init() != BMI088_NO_ERROR) {
    return false;
  }

  reset_attitude_state();
  g_ready = true;
  return true;
}

void imu_update(void) {
  if (!g_ready) {
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
