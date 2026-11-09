// DMA/BDMA 占用表（编译期固定；新占用在此登记并更新本表）：
//   DMA1_Stream0/1 → 空闲（BMI088 使用阻塞 SPI2）
//   BDMA1_Ch0      → SPI6_TX（WS2812；句柄与 handler 归 lib/board（WS2812 驱动），
//                    缓冲位于 .bdma_noinit/RAM_D3）
//   其余空闲。
//
// DMA 中断优先级须 ≥ BOARD_ISR_SYSCALL_THRESHOLD（数值越大，紧迫度越低），
// 不得抢占 FreeRTOS 内核临界区。

#include "dma.h"
#include "board_irq.h"

// DMA IRQ 优先级 6/0，低于 syscall 门槛 5。
#define DMA_STREAM_IRQ_PRIORITY 6U

void MX_DMA_Init(void) {
  // 编译期校验 FreeRTOS FromISR 优先级约束。
  _Static_assert(DMA_STREAM_IRQ_PRIORITY >= BOARD_ISR_SYSCALL_THRESHOLD,
      "DMA IRQ priority must not preempt the FreeRTOS syscall threshold");

  __HAL_RCC_DMA1_CLK_ENABLE(); // 当前 DMA1 无占用（lib/board 的 IMU 驱动不用 DMA）；控制器时钟为未来设备保留

  // BDMA（WS2812 SPI6_TX，lib/board（WS2812 驱动） 自持句柄与 handler）。H723 的 DMAMUX2
  // 挂在 BDMA 同一 AHB4 时钟域，无独立时钟门，BDMAEN 一个位管两者。
  __HAL_RCC_BDMA_CLK_ENABLE();
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, DMA_STREAM_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
}
