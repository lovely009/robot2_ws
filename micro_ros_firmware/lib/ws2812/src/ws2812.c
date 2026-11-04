// WS2812 彩灯驱动：SPI6 @ 6MHz，以 8 个 SPI 位编码 1 个 WS2812 位。
// 24 字节 GRB 数据后追加 100 字节低电平复位。SPI6_TX 使用 BDMA Channel0；
// 帧缓冲必须位于 BDMA 可访问的 .bdma_noinit/RAM_D3，且由本文件显式清零。
// MX_DMA_Init 须先初始化控制器时钟和 NVIC。发送使用单次 DMA，忙时由调用方
// 下周期重试；BDMA 完成后仍需 SPI6 EOT 中断将 HAL 状态恢复为 READY。

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
  // 已由 src/dma.c 的 MX_DMA_Init 使能——前置契约见文件头。
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

// BDMA TC 完成缓冲搬运；SPI6 EOT 清发送标志并将 HAL 状态置回 READY。
void BDMA_Channel0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi6_tx);
}

void SPI6_IRQHandler(void) {
  HAL_SPI_IRQHandler(&hspi6);
}
