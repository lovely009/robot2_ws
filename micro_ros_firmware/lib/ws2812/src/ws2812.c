// WS2812 彩灯驱动（SPI6 + BDMA 编码，官方 CtrBoard-H7_WS2812 例程同源方案）
//
// 原理：
//   - SPI6 内核时钟 = HSE 24MHz（SystemClock_Config 已设 RCC_SPI6CLKSOURCE_HSE），
//     预分频 /4 → SPI 时钟 6MHz
//   - 每 8 个 SPI 位编码 1 个 WS2812 位：0→0x60（2 高 6 低 ≈ 333ns 高），
//     1→0x78（4 高 4 低 ≈ 667ns 高），位周期 1.33us（WS2812 容差内）
//   - 24 位 GRB 颜色 = 24 字节；帧尾 100 字节 0x00 低电平复位（>50us）
//
// DMA（2026-09-07，替代旧「阻塞发送 + 关中断 ~165us + 101 次 HAL 调用」实现）：
//   - H723 上 SPI6 的 DMA 请求只接 DMAMUX2/BDMA（DMA1/2 无此请求线）；
//     BDMA 仅能访问 DTCM/SRAM4 → 帧缓冲置于 RAM_D3 的 .bdma_noinit 段
//     （项目级链接脚本 ldscript/stm32h723vgt6.ld；NOLOAD 段由本文件清零）。
//   - 控制器级资源（BDMA/DMAMUX2 时钟、Channel0 NVIC）归 lib/dma 的
//     MX_DMA_Init，由 main 板级初始化在 ws2812_init 之前调用（前置契约）。
//   - 发送 = 单次 HAL_SPI_Transmit_DMA；完成链两级，handler 均由本文件
//     自持（与 lib/imu / lib/fdcan 同构）：BDMA_Channel0 IRQ 里 HAL 收完
//     缓冲并使能 SPI EOT 中断，SPI6 EOT IRQ 里 HAL 清标志、置 READY。
//     SPI6 侧缺失则 State 永久 BUSY_TX——首帧后 LED 冻结红灯（2026-09-08
//     修复）。不关中断——DMA 流不被任务/ISR 抢占打断，灯序与高优先级
//     活动互不影响。
//   - 忙（上一帧未完）时 submit 返回 false，调用方下个周期重试。
//   - cache/MPU（dma.h 政策声明位）：本工程未启用 cache（H723 无 D-cache），
//     DMA 缓冲无需维护。
//
// 注：此前用 SPI1 @2.125MHz + 3 位编码（0→0x80 高 470ns）卡在 WS2812 阈值
// （~0.5us）边缘，0 位被误判为 1 → 全白。官方 6MHz + 8 位编码 0 位仅 333ns，安全。

#include "ws2812.h"
#include "stm32_def.h"      // Error_Handler
#include "stm32h7xx_hal.h"  // HAL SPI6 / BDMA / GPIO
#include <string.h>

#define WS2812_FRAME_COLOR_BYTES 24U  // GRB 24 位 × 8 SPI 位
#define WS2812_FRAME_RESET_BYTES 100U // 帧尾低电平复位 >50us
#define WS2812_FRAME_BYTES \
  (WS2812_FRAME_COLOR_BYTES + WS2812_FRAME_RESET_BYTES)

// BDMA 仅能访问 DTCM/SRAM4：帧缓冲必须落在 RAM_D3（.bdma_noinit，见链接脚本）
static uint8_t g_ws2812_frame[WS2812_FRAME_BYTES]
    __attribute__((section(".bdma_noinit"), aligned(4)));

static SPI_HandleTypeDef hspi6;
static DMA_HandleTypeDef hdma_spi6_tx;

void ws2812_init(void) {
  // SPI6 内核时钟源已在 SystemClock_Config 设为 HSE（24MHz）
  __HAL_RCC_SPI6_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA5=SPI6_SCK, PA7=SPI6_MOSI（AF8）
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &gpio);

  // SPI6_TX DMA：BDMA1 Channel0（DMAMUX2 请求 12）。控制器级资源（时钟/NVIC）
  // 已由 lib/dma 的 MX_DMA_Init 使能——前置契约见文件头。
  hdma_spi6_tx.Instance = BDMA_Channel0;
  hdma_spi6_tx.Init.Request = BDMA_REQUEST_SPI6_TX;
  hdma_spi6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi6_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi6_tx.Init.Mode = DMA_NORMAL;
  hdma_spi6_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_spi6_tx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(&hspi6, hdmatx, hdma_spi6_tx);

  // HAL 把 SPI 的 CFG1/CFG2 寄存器一次性写入，故需显式配置关键字段；
  // 其余字段（CRC/TIMode/NSSPolarity 等）默认 0，结构体清零后省略。
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 24MHz/4 = 6MHz
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK) {
    Error_Handler();
  }

  // 两级完成链第二级：SPI6 EOT 中断（handler 见文件尾）。优先级与 BDMA
  // 同为 6，≥ FreeRTOS 可调用域边界（configLIBRARY_MAX_SYSCALL=5）。
  HAL_NVIC_SetPriority(SPI6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SPI6_IRQn);

  // .bdma_noinit 为 NOLOAD 段，startup 不清零：帧尾复位字节必须恒 0x00
  memset(g_ws2812_frame, 0, sizeof(g_ws2812_frame));
}

bool ws2812_submit_rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (hspi6.State != HAL_SPI_STATE_READY) {
    return false; // 上一帧 DMA 未完：调用方下个周期重试
  }
  // WS2812 数据顺序为 GRB；0→0x60, 1→0x78（8 SPI 位/WS2812 位）。
  // 帧尾 100 字节复位在 init 时清零后不再改动。
  for (int i = 0; i < 8; i++) {
    g_ws2812_frame[7 - i] = (((g >> i) & 0x01) ? 0x78 : 0x60);
    g_ws2812_frame[15 - i] = (((r >> i) & 0x01) ? 0x78 : 0x60);
    g_ws2812_frame[23 - i] = (((b >> i) & 0x01) ? 0x78 : 0x60);
  }
  return HAL_SPI_Transmit_DMA(&hspi6, g_ws2812_frame,
                              WS2812_FRAME_BYTES) == HAL_OK;
}

// 发送完成中断两级链（handler 均归设备库自持，与 lib/imu / lib/fdcan
// 的回调同构）：BDMA TC = HAL 收完缓冲、使能 EOT 中断；SPI6 EOT = HAL
// 清 EOT/TXTF 标志、置 READY。缺任一级则链断（SPI6 侧缺失曾是 LED
// 冻结红灯的根因——EOT 挂起无人清，submit 永远撞 BUSY_TX）。
void BDMA_Channel0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi6_tx);
}

void SPI6_IRQHandler(void) {
  HAL_SPI_IRQHandler(&hspi6);
}
