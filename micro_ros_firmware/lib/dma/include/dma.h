// DMA 共享资源门卫（ARD-0007 §28/§30：共享 DMA 控制器、通道配置、IRQ 资源
// 管理；不做 IMU/WS2812 等设备业务与采样策略）。
//
// 职责边界：
//  1. DMA1 控制器资源：时钟、NVIC 优先级（含 FreeRTOS FromISR 合规断言）。
//  2. SPI MspInit 分发器：HAL_SPI_MspInit 是全局唯一弱回调入口，各 SPI
//     外设（当前仅 SPI2/BMI088）在此按 Instance 分发——新 SPI 外设（如
//     WS2812 走 SPI6，ARD §30.2）在此新增 case，禁止在业务库内私写 MspInit。
//  3. 流占用表：DMA 流的分配关系编译期固定、本文件可见（见 dma.c 顶部表）。
//
// cache/MPU 政策声明位（ARD §30.3）：当前全部 DMA 配置处于"已链接未使能"
// 状态（BMI088 收发走阻塞 HAL_SPI），无 DMA 缓冲，无需 cache 维护。未来任何
// 外设启用 DMA 传输时，必须在此层声明其缓冲的 MPU 区域或 TX/RX 边界的
// cache 维护方式，禁止把 cache 语义下沉到协议/业务代码。

#ifndef DMA_H
#define DMA_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// DMA1 时钟与流中断优先级。HAL_SPI_Init（来自 bmi088_middleware.c）触发
// 本库内的 HAL_SPI_MspInit 完成 SPI2 引脚/时钟源/DMA 流链接。
void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif  // DMA_H
