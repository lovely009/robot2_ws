#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 对外发布的 IMU 数据（SI 单位，符合 ROS2 sensor_msgs/Imu 约定）
typedef struct {
  float gyro[3];   // 角速度 rad/s
  float accel[3];  // 线加速度 m/s^2
  float temp;      // 芯片温度 °C
  float quat[4];   // 姿态四元数 (w,x,y,z)
  float angle[3];  // 欧拉角 yaw/pitch/roll（rad）
} imu_data_t;

// 尝试初始化 BMI088 一次；必须在 FreeRTOS 任务内调用。
// 失败由 sensor_task 延后重试，避免阻塞其他任务。
bool imu_try_init(void);

// 读一次传感器并更新姿态；须在 sensor_task 的 EXTI 信号量唤醒后调用。
// 本次传输失败时返回 false，调用方不得发布该样本。
bool imu_update(void);

// 拷贝当前数据（供发布层读取）
void imu_get_data(imu_data_t *out);

bool imu_ready(void);

#ifdef __cplusplus
}
#endif

#endif  // IMU_H
