// DMA 共享资源门卫：管理 DMA1、BDMA/DMAMUX2 的控制器时钟与通道中断优先级，记录编译期固定的
// 资源占用。**设备句柄与传输配置归设备库**——WS2812 自持 BDMA Channel0 的句柄、IRQ handler 与位于
// .bdma_noinit/RAM_D3 的帧缓冲；CAN4 使用 DMA1 Stream0/1 和 D2 暂存区；BMI088 走阻塞 SPI2。
// DMA 缓冲的 cache 约束由非缓存 MPU 区域承担（.dma_noinit/.bdma_noinit），设备库无需手工维护 cache。

#ifndef DMA_H
#define DMA_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化共享 DMA 控制器资源。由 main 在设备初始化前调用。
void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif  // DMA_H
