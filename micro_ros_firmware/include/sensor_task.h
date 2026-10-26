// 传感器业务任务接口（官方 CtrBoard-H7_IMU_TempCtrl 任务编排移植）。
// 当前承载 IMU（BMI088 初始化、EXTI 信号量、Mahony、温度 PID/加热 PWM），
// 后续新增传感器统一挂到此任务。

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建传感器任务（只能调用一次；重复调用返回 true 且不重复建任务）
bool sensor_task_start(void);

// 原子复制最新的完整 IMU 快照；尚无有效样本时返回 false。
bool sensor_task_get_imu_snapshot(imu_data_t *out, uint32_t *sequence);


#ifdef __cplusplus
}
#endif

#endif  // SENSOR_TASK_H
