// DMA1 配置（官方 CtrBoard-H7_IMU_TempCtrl 例程移植）。
// 官方由 MX_DMA_Init 开启 DMA1 时钟/中断，并在 HAL_SPI_MspInit 中配置
// SPI2_RX = DMA1_Stream0、SPI2_TX = DMA1_Stream1 并链接到 hspi2。
// 本工程 SPI2 句柄由 bmi088_middleware.c 提供，收发仍走官方阻塞调用，
// DMA 仅保留配置与中断转发路径。

#include "dma.h"
#include "stm32_def.h"

static DMA_HandleTypeDef hdma_spi2_rx;
static DMA_HandleTypeDef hdma_spi2_tx;

void MX_DMA_Init(void) {
  // DMA 中断优先级沿用官方 TempCtrl：6/0
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance != SPI2) {
    return;
  }

  // SPI2 时钟源与官方一致：PLL（在 SPI2 首次初始化时兜底配置）
  RCC_PeriphCLKInitTypeDef periph_clk = {};
  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  periph_clk.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // PC1=SPI2_MOSI, PC2_C=SPI2_MISO, PB13=SPI2_SCK（AF5）
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &gpio);

  // SPI2_RX：DMA1_Stream0，外设→内存，字节对齐，Normal 模式
  hdma_spi2_rx.Instance = DMA1_Stream0;
  hdma_spi2_rx.Init.Request = DMA_REQUEST_SPI2_RX;
  hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi2_rx.Init.Mode = DMA_NORMAL;
  hdma_spi2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx);

  // SPI2_TX：DMA1_Stream1，内存→外设，字节对齐，Normal 模式
  hdma_spi2_tx.Instance = DMA1_Stream1;
  hdma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;
  hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi2_tx.Init.Mode = DMA_NORMAL;
  hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(hspi, hdmatx, hdma_spi2_tx);

  // 保留官方中断使能；阻塞收发路径不会触发 SPI 事件
  HAL_NVIC_SetPriority(SPI2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

void DMA1_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

void DMA1_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}
