// DMA/BDMA 占用表（编译期固定；新占用在此登记并更新本表）：
//   DMA1_Stream0/1 → 空闲（BMI088 使用阻塞 SPI2）
//   BDMA1_Ch0      → SPI6_TX（WS2812；句柄与 handler 归 lib/ws2812，
//                    缓冲位于 .bdma_noinit/RAM_D3）
//   其余空闲。
//
// 优先级合规：DMA 流中断优先级必须 ≥ BOARD_ISR_SYSCALL_THRESHOLD
//（数值更大=紧迫度更低，不得抢占 FreeRTOS 内核临界区）——当前不调用 FromISR
// API，但未来 DMA 回调一旦使用 RTOS 服务，此断言阻止静默违规。

#include "dma.h"
#include "board_irq.h"

// DMA 中断优先级沿用官方 TempCtrl：6/0（低于 syscall 门槛 5，数值更大）。
#define DMA_STREAM_IRQ_PRIORITY 6U

void MX_DMA_Init(void) {
  // 与 FreeRTOS FromISR 的兼容性在编译期钉死（禁止项：静默违反 RTOS 约定）。
  _Static_assert(DMA_STREAM_IRQ_PRIORITY >= BOARD_ISR_SYSCALL_THRESHOLD,
                 "DMA IRQ priority must not preempt the FreeRTOS syscall threshold");

  __HAL_RCC_DMA1_CLK_ENABLE(); // 当前 DMA1 无占用（lib/imu 不用 DMA）；控制器时钟为未来设备保留

  // BDMA（WS2812 SPI6_TX，lib/ws2812 自持句柄与 handler）。H723 的 DMAMUX2
  // 挂在 BDMA 同一 AHB4 时钟域，无独立时钟门，BDMAEN 一个位管两者。
  __HAL_RCC_BDMA_CLK_ENABLE();
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, DMA_STREAM_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
}
