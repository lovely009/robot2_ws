// DMA 共享资源门卫——只管控制器级资源（ARD-0007 §28/§30）：
//   1. DMA1 与 BDMA/DMAMUX2 时钟、通道/流中断优先级（含 FreeRTOS FromISR
//      合规断言）。
//   2. 流占用登记（见 dma.c 顶部表）：分配关系编译期固定、仅登记不配置。
//   3. cache/MPU 政策声明位（ARD §30.3）：任何设备启用 DMA 传输前，其
//      缓冲的 MPU 区域或 TX/RX 边界 cache 维护方式必须在此层声明，
//      禁止把 cache 语义下沉到协议/业务代码。当前声明：本工程未启用
//      cache（H723 无 D-cache），DMA 缓冲无需维护。
//
// 低耦合高内聚口径（2026-09-07）：设备 SPI/DMA 配置归各设备库——lib/imu
// 自持 SPI2 的 HAL_SPI_MspInit、流句柄与 handler（与 lib/fdcan 自持
// HAL_FDCAN_MspInit 同构）；跨设备接线在任务层完成（sensor_task_start
// 先调 MX_DMA_Init 再触发 IMU 初始化）。lib/imu 为阻塞收发、不用 DMA
// （2026-09-07 移除句柄/链接）；WS2812 走 SPI6 + BDMA（ARD §30.2）：
// BDMA_Ch0 句柄/handler 归 lib/ws2812 自持，帧缓冲在 .bdma_noinit
// （RAM_D3——BDMA 仅能访问 DTCM/SRAM4）。库与库之间禁止互相 include
// 或调用。
//
// 职责边界：不做 IMU/WS2812 等设备业务与采样策略；不定义任何 SPI 外设。

#ifndef DMA_H
#define DMA_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// DMA1 时钟与流中断优先级（Stream0/1 已登记给 SPI2/BMI088，句柄与 handler
// 归 lib/imu 自持）。由任务层在设备初始化前调用（官方时序：DMA 先于 SPI2）。
void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif  // DMA_H
