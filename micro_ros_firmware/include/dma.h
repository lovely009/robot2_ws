// DMA1 驱动接口（官方 CtrBoard-H7_IMU_TempCtrl 例程移植）。
// 仅启用 DMA1 时钟与中断；SPI2 RX/TX 流配置在 HAL_SPI_MspInit 内完成。

#ifndef DMA_H
#define DMA_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif  // DMA_H
