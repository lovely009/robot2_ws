// DMA 共享资源门卫（ARD-0007 §28/§30）：
//   1. 管理 DMA1、BDMA/DMAMUX2 控制器时钟与通道中断优先级；
//   2. 记录编译期固定的资源占用，设备句柄和传输配置仍归设备库；
//   3. 统一 DMA 缓冲的 cache/MPU 约束。当前未启用 D-cache，无需维护。
//
// 当前占用：DMA1 Stream0/1 空闲；WS2812 使用 BDMA Channel0，句柄、IRQ
// handler 与位于 .bdma_noinit/RAM_D3 的帧缓冲均归 lib/ws2812。
// BMI088 使用阻塞 SPI2，不占用 DMA。设备库之间禁止互相 include 或调用。

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
