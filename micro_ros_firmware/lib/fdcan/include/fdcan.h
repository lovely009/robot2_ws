// FDCAN 句柄与初始化接口（官方 CtrBoard-H7_FDCAN 例程移植）。
// 三路 FDCAN 的时钟/引脚/中断在 src/fdcan.c 的 MspInit 内完成。

#ifndef FDCAN_H
#define FDCAN_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);

#ifdef __cplusplus
}
#endif

#endif  // FDCAN_H
