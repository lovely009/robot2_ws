#include "can4_spi.h"
#include "stm32h7xx_hal.h"
#include "clock.h"
#include <string.h>

static SPI_HandleTypeDef spi3;
enum { TRANSFER_TIMEOUT_US = 100 };

// DMA1 不能访问 DTCM，调用方的栈也可能在可缓存内存。D2 暂存区由 MPU 设为非缓存。
static uint8_t dma_tx[CAN4_SPI_MAX_BYTES] __attribute__((section(".dma_noinit"), aligned(4)));
static uint8_t dma_rx[CAN4_SPI_MAX_BYTES] __attribute__((section(".dma_noinit"), aligned(4)));
static volatile enum { IDLE, WAIT_TASK, WAIT_EVENT, WAIT_DMA_TAIL, COMPLETE, FAILED } dma_state;
static volatile struct can4_dma_diagnostics {
  uint32_t reads, writes, timeouts, max_eot_us;
} g_can4_dma_diag __attribute__((section(".dtcm_noinit")));

#define SPI_ERRORS (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_TIFRE)
#define SPI_CLEAR (SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_UDRC | SPI_IFCR_TIFREC)
#define DMA_ERRORS (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF1 | DMA_LISR_DMEIF1)
#define DMA_DONE (DMA_LISR_TCIF0 | DMA_LISR_TCIF1)
#define DMA_CLEAR (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 | \
                   DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)

bool can4_spi_init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  GPIO_InitTypeDef pin = {0};
  pin.Pin = GPIO_PIN_9; // H1.1 nCS
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &pin);

  pin.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12; // SCK, MISO, MOSI
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &pin);

  pin.Pin = GPIO_PIN_14; // H1.9 INT，低有效，在任务内按电平读取
  pin.Mode = GPIO_MODE_INPUT;
  pin.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &pin);

  spi3.Instance = SPI3;
  spi3.Init.Mode = SPI_MODE_MASTER;
  spi3.Init.Direction = SPI_DIRECTION_2LINES;
  spi3.Init.DataSize = SPI_DATASIZE_8BIT;
  spi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  spi3.Init.NSS = SPI_NSS_SOFT;
  // PLL1Q 240 MHz / 16 = 15 MHz，低于 40 MHz 模块的勘误上限 17 MHz。
  spi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  spi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  spi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
  CLEAR_BIT(SPI3->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  dma_state = IDLE;
  if (HAL_SPI_Init(&spi3) != HAL_OK) return false;
  static bool g_dma_diag_initialized = false;
  if (!g_dma_diag_initialized) {
    g_can4_dma_diag = (struct can4_dma_diagnostics){0};
    g_dma_diag_initialized = true;
  }

  // Stream0/1 对应 DMAMUX1 通道 0/1，字节宽、地址自增、普通模式。
  DMAMUX1_Channel0->CCR = DMA_REQUEST_SPI3_RX;
  DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI3_TX;
  DMA1_Stream0->PAR = (uintptr_t)&SPI3->RXDR;
  DMA1_Stream1->PAR = (uintptr_t)&SPI3->TXDR;
  DMA1_Stream0->M0AR = (uintptr_t)dma_rx;
  DMA1_Stream1->M0AR = (uintptr_t)dma_tx;
  DMA1_Stream0->FCR = DMA1_Stream1->FCR = 0U; // direct mode

  __HAL_RCC_TIM23_CLK_ENABLE();
  // 板级时钟校验 HCLK=240 MHz、APB1=/2，所以 TIM23 内核时钟等于 HCLK。
  TIM23->PSC = HAL_RCC_GetHCLKFreq() / 1000000U - 1U;
  TIM23->ARR = TRANSFER_TIMEOUT_US - 1U;
  TIM23->EGR = TIM_EGR_UG;
  TIM23->SR = 0U;
  return true;
}

static bool transfer_polled(const uint8_t *tx, uint8_t *rx, uint16_t length) {
  // 短寄存器访问；最多预装 8 字节，给 RX FIFO 保留余量。
  const uint32_t start_us = getCurrentMicros();
  uint16_t sent = 0, received = 0;
  MODIFY_REG(SPI3->CR2, SPI_CR2_TSIZE, length);
  SPI3->IFCR = SPI_CLEAR;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  SET_BIT(SPI3->CR1, SPI_CR1_SPE);
  SET_BIT(SPI3->CR1, SPI_CR1_CSTART);
  bool complete = false;
  do {
    uint32_t status = SPI3->SR;
    if (status & SPI_ERRORS) break;
    if ((status & SPI_SR_RXP) && received < length)
      rx[received++] = *(volatile uint8_t *)&SPI3->RXDR;
    if ((status & SPI_SR_TXP) && sent < length && sent - received < 8)
      *(volatile uint8_t *)&SPI3->TXDR = tx[sent++];
    if (received == length && (status & SPI_SR_EOT)) {
      complete = true;
      break;
    }
  } while ((uint32_t)(getCurrentMicros() - start_us) < TRANSFER_TIMEOUT_US);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
  SPI3->IFCR = SPI_CLEAR;
  return complete;
}

// SPI3 EOT 与 TIM23 同一抢占优先级。先关两个事件源再唤醒，早到的通知也可被任务取走。
static void finish_dma_from_isr(void) {
  bool wake_task = dma_state == WAIT_TASK;
  if (!wake_task && dma_state != WAIT_EVENT) return;
  uint32_t status = SPI3->SR;
  uint32_t flags = DMA1->LISR;
  uint32_t elapsed = TIM23->CNT;
  // 先观察 EOT 和两路 TC，最后查 UIF；二者同时挂起时按超时处理。
  bool expired = (TIM23->SR & TIM_SR_UIF) != 0U;
  bool valid_eot = !expired && (status & (SPI_SR_EOT | SPI_ERRORS)) == SPI_SR_EOT &&
                   !(flags & DMA_ERRORS);
  bool wait_tail = valid_eot && (flags & DMA_DONE) != DMA_DONE;
  SPI3->IER = 0U;
  TIM23->DIER = 0U;
  if (!wait_tail) {
    TIM23->CR1 = TIM_CR1_OPM;
    TIM23->SR = 0U;
  }
  if (!valid_eot) {
    if (expired) ++g_can4_dma_diag.timeouts;
    CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
  } else if (elapsed > g_can4_dma_diag.max_eot_us) {
    g_can4_dma_diag.max_eot_us = elapsed;
  }
  GPIOB->BSRR = GPIO_PIN_9; // EOT 或超时，均及时释放 nCS
  // H723 的 EOT 请求保持到 IFCR 确认。只关 IER 会反复进 SPI3 IRQ，任务无法收尾。
  SPI3->IFCR = SPI_CLEAR;
  if (!valid_eot) dma_state = FAILED;
  else dma_state = wait_tail ? WAIT_DMA_TAIL : COMPLETE;
  __DSB();
  if (wake_task) can4_spi_wake_from_isr();
  else __SEV();
}

void SPI3_IRQHandler(void) {
  if (SPI3->SR & SPI3->IER) finish_dma_from_isr();
}

void TIM23_IRQHandler(void) {
  if ((TIM23->DIER & TIM_DIER_UIE) && (TIM23->SR & TIM_SR_UIF)) finish_dma_from_isr();
}

static bool stop_streams(void) {
  CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_EN);
  CLEAR_BIT(DMA1_Stream1->CR, DMA_SxCR_EN);
  const uint32_t start = getCurrentMicros();
  while ((DMA1_Stream0->CR | DMA1_Stream1->CR) & DMA_SxCR_EN) {
    if ((uint32_t)(getCurrentMicros() - start) >= 8U) return false;
  }
  return true;
}

static bool transfer_dma(const uint8_t *tx, uint8_t *rx, uint16_t length, bool may_block) {
  // 上次故障后仍未停流时，不能复用 .dma_noinit 缓冲或重写 NDTR。
  if ((DMA1_Stream0->CR | DMA1_Stream1->CR) & DMA_SxCR_EN) return false;
  memcpy(dma_tx, tx, length);
  __DMB();
  DMA1->LIFCR = DMA_CLEAR;
  DMA1_Stream0->NDTR = DMA1_Stream1->NDTR = length;
  DMA1_Stream0->CR = DMA_SxCR_MINC | DMA_SxCR_PL_0 | DMA_SxCR_EN;
  DMA1_Stream1->CR = DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_EN;

  MODIFY_REG(SPI3->CR2, SPI_CR2_TSIZE, length);
  SET_BIT(SPI3->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
  SPI3->IFCR = SPI_CLEAR;
  TIM23->CNT = 0U;
  TIM23->SR = 0U;
  NVIC_ClearPendingIRQ(SPI3_IRQn);
  NVIC_ClearPendingIRQ(TIM23_IRQn);
  dma_state = may_block ? WAIT_TASK : WAIT_EVENT;
  SPI3->IER = SPI_IER_EOTIE | SPI_IER_OVRIE | SPI_IER_MODFIE | SPI_IER_UDRIE | SPI_IER_TIFREIE;
  TIM23->DIER = TIM_DIER_UIE;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
  SET_BIT(SPI3->CR1, SPI_CR1_SPE);
  TIM23->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
  SET_BIT(SPI3->CR1, SPI_CR1_CSTART);
  __set_PRIMASK(primask);

  if (may_block) {
    can4_spi_wait_dma();
  } else {
    // TX 从调度器出队到 UINC|TXREQ 之间不让低优先级任务插入。
    while (dma_state == WAIT_EVENT) __WFE();
  }
  bool ok = dma_state == COMPLETE;
  while (dma_state == WAIT_DMA_TAIL) {
    // EOT 早于最后一次内存写时，TIM23 继续计时，覆盖任务重新获得 CPU 的时间。
    uint32_t flags = DMA1->LISR;
    if (TIM23->SR & TIM_SR_UIF) {
      ++g_can4_dma_diag.timeouts;
      break;
    }
    if (flags & DMA_ERRORS) break;
    if ((flags & DMA_DONE) == DMA_DONE) {
      ok = true;
      break;
    }
  }
  TIM23->CR1 = TIM_CR1_OPM;
  TIM23->SR = 0U;
  CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
  CLEAR_BIT(SPI3->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
  if (!stop_streams()) ok = false;
  SPI3->IFCR = SPI_CLEAR;
  dma_state = IDLE;
  __DMB();
  if (ok) {
    memcpy(rx, dma_rx, length);
    if (may_block) ++g_can4_dma_diag.reads;
    else ++g_can4_dma_diag.writes;
  }
  return ok;
}

bool can4_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t length, bool may_block) {
  if (!tx || !rx || !length || length > CAN4_SPI_MAX_BYTES) return false;
  if (length < CAN4_SPI_DMA_MIN_BYTES) return transfer_polled(tx, rx, length);
  return transfer_dma(tx, rx, length, may_block);
}

bool can4_irq_asserted(void) {
  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_RESET;
}

void can4_spi_power_set(bool on) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef pin = {0};
  if (!on) {
    TIM23->CR1 = TIM_CR1_OPM;
    TIM23->DIER = 0U;
    TIM23->SR = 0U;
    SPI3->IER = 0U;
    CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
    CLEAR_BIT(SPI3->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
    (void)stop_streams();
    SPI3->IFCR = SPI_CLEAR;
    dma_state = IDLE;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    // 断电期间拉低输出并断开上拉，防止经 H1 引脚 ESD 二极管反向寄生供电导致芯片无法彻底复位。
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_12, GPIO_PIN_RESET);
    pin.Pin = GPIO_PIN_9;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pin);

    pin.Pin = GPIO_PIN_10 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &pin);

    pin.Pin = GPIO_PIN_11;
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &pin);

    pin.Pin = GPIO_PIN_14;
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &pin);
  } else {
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_NOPULL;
    pin.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &pin);
    pin.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &pin);
    pin.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &pin);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  }
}

void can4_init_delay_ms(uint32_t ms) {
  HAL_Delay(ms);
}
