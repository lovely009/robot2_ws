// DMA 共享资源门卫实现（职责与资源占用表见 dma.h）。
//
// DMA 流占用表（编译期固定；新占用必须在此登记并更新本表）：
//   DMA1_Stream0 → SPI2_RX（BMI088 加速度计/陀螺仪，lib/imu）
//   DMA1_Stream1 → SPI2_TX（BMI088 加速度计/陀螺仪，lib/imu）
//   其余流空闲。SPI6（WS2812 候选，ARD §30.2）未分配。
//
// 优先级合规：DMA 流中断优先级必须 ≥（数值上不高于触发优先级门槛）
// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY——当前不调用 FromISR API，
// 但未来 DMA 回调一旦使用 RTOS 服务，此断言阻止静默违规（同 lib/fdcan 的
// FDCAN_FREERTOS_IRQ_PRIORITY 断言做法）。

#include "dma.h"
#include "stm32_def.h"
#include <STM32FreeRTOS.h> // configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（断言用）

// DMA 中断优先级沿用官方 TempCtrl：6/0（低于 syscall 门槛 5，数值更大）。
#define DMA_STREAM_IRQ_PRIORITY 6U

static DMA_HandleTypeDef hdma_spi2_rx;
static DMA_HandleTypeDef hdma_spi2_tx;

static void spi2_msp_init(SPI_HandleTypeDef *hspi);

void MX_DMA_Init(void) {
  // 与 FreeRTOS FromISR 的兼容性在编译期钉死（禁止项：静默违反 RTOS 约定）。
  _Static_assert(DMA_STREAM_IRQ_PRIORITY >=
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
                 "DMA IRQ priority must not preempt the FreeRTOS syscall threshold");

  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, DMA_STREAM_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, DMA_STREAM_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

// SPI MspInit 分发器：HAL 的该回调全局唯一。当前仅 SPI2（BMI088）；
// 新 SPI 外设在此新增分支并同步更新 dma.c 顶部的流占用表。
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  if (hspi == NULL) {
    return;
  }
  // Instance 为外设寄存器结构指针，不能用 switch（非整型）。
  if (hspi->Instance == SPI2) {
    spi2_msp_init(hspi);
  }
  // 未登记的 SPI：不初始化（外设自负责报错）。
}

static void spi2_msp_init(SPI_HandleTypeDef *hspi) {
  // SPI2 时钟源与官方一致：PLL（在 SPI2 首次初始化时兜底配置）
  RCC_PeriphCLKInitTypeDef periph_clk = {0};
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
  HAL_NVIC_SetPriority(SPI2_IRQn, DMA_STREAM_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

void DMA1_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

void DMA1_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}
