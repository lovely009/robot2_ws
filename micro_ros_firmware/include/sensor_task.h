// 传感器业务任务接口（官方 CtrBoard-H7_IMU_TempCtrl 任务编排移植）。
// 当前承载 IMU（BMI088 初始化、EXTI 信号量、Mahony、温度 PID/加热 PWM），
// 后续新增传感器统一挂到此任务。

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 创建传感器任务（只能调用一次；重复调用返回 true 且不重复建任务）
bool sensor_task_start(void);

// 强制停止加热（官方 TempCtrl forceStop 语义；true 时 PID 输出置 0）
void imu_set_force_stop(bool stop);

#ifdef __cplusplus
}
#endif

#endif  // SENSOR_TASK_H
