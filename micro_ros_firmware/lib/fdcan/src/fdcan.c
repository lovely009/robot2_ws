// FDCAN 外设驱动（官方 CtrBoard-H7_FDCAN 例程移植 + 电机协议适配）。
//
// 官方生成代码：三路 FDCAN（FDCAN1=PD0/PD1、FDCAN2=PB5/PB6、FDCAN3=PD12/PD13），
// FD + BRS、标称 1Mbps / 数据 5Mbps。时钟源在官方例程为 PLL1Q=80MHz，
// 本工程为不动 BMI088 的 PLL1Q=240MHz(SPI2)，改用 PLL2Q=80MHz 供 FDCAN。
// 消息 RAM：RX/TX 元素扩到 64B（CAN-FD 满载帧；vaxor 反馈 12/20B），只保留
// RX FIFO0 + TX FIFO，每路按 HAL 计算占 300 个 32 位字，偏移 0/300/600。
// 中断转发沿用 HAL：FDCANx_IT0/IT1_IRQHandler -> HAL_FDCAN_IRQHandler。

#include <STM32FreeRTOS.h>

#include "fdcan.h"
#include "stm32_def.h"  // Error_Handler 宏（_Error_Handler(file, line)）

#define FDCAN_FREERTOS_IRQ_PRIORITY 5U

_Static_assert(
    FDCAN_FREERTOS_IRQ_PRIORITY >=
        configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
    "FDCAN IRQ priority must permit FreeRTOS FromISR APIs");

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

// 三路共用一个 FDCAN 内核时钟，计数器保证只开关一次
static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

static void fdcan_periph_clk_config(void) {
  // PLL2：HSE 24MHz / 2 * 40 = 480MHz VCO，Q=6 -> 80MHz FDCAN 内核时钟
  RCC_PeriphCLKInitTypeDef periph_clk = {0};
  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  periph_clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  periph_clk.PLL2.PLL2M = 2;
  periph_clk.PLL2.PLL2N = 40;
  periph_clk.PLL2.PLL2P = 2;
  periph_clk.PLL2.PLL2Q = 6;
  periph_clk.PLL2.PLL2R = 2;
  periph_clk.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  periph_clk.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  periph_clk.PLL2.PLL2FRACN = 0;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
    Error_Handler();
  }
}

void MX_FDCAN1_Init(void) {
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 12; // 16tq@80MHz = 5Mbps，与 FDCAN2/3 一致（审查 C3）
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 4;
  hfdcan1.Init.ExtFiltersNbr = 4;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

void MX_FDCAN2_Init(void) {
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = ENABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 16;
  hfdcan2.Init.NominalTimeSeg1 = 63;
  hfdcan2.Init.NominalTimeSeg2 = 16;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 2;
  hfdcan2.Init.DataTimeSeg1 = 12;
  hfdcan2.Init.DataTimeSeg2 = 3;
  hfdcan2.Init.MessageRAMOffset = 300;
  hfdcan2.Init.StdFiltersNbr = 4;
  hfdcan2.Init.ExtFiltersNbr = 4;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

void MX_FDCAN3_Init(void) {
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = ENABLE;
  hfdcan3.Init.NominalPrescaler = 1;
  hfdcan3.Init.NominalSyncJumpWidth = 16;
  hfdcan3.Init.NominalTimeSeg1 = 63;
  hfdcan3.Init.NominalTimeSeg2 = 16;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 2;
  hfdcan3.Init.DataTimeSeg1 = 12;
  hfdcan3.Init.DataTimeSeg2 = 3;
  hfdcan3.Init.MessageRAMOffset = 600;
  hfdcan3.Init.StdFiltersNbr = 4;
  hfdcan3.Init.ExtFiltersNbr = 4;
  hfdcan3.Init.RxFifo0ElmtsNbr = 8;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {
  GPIO_InitTypeDef gpio = {0};
  configASSERT(FDCAN_FREERTOS_IRQ_PRIORITY >=
               configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  fdcan_periph_clk_config();

  if (fdcanHandle->Instance == FDCAN1) {
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    // PD0=FDCAN1_RX, PD1=FDCAN1_TX（AF9）
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  } else if (fdcanHandle->Instance == FDCAN2) {
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    // PB5=FDCAN2_RX, PB6=FDCAN2_TX（AF9）
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  } else if (fdcanHandle->Instance == FDCAN3) {
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    // PD12=FDCAN3_RX, PD13=FDCAN3_TX（AF5）
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_FDCAN3;
    HAL_GPIO_Init(GPIOD, &gpio);

    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, FDCAN_FREERTOS_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle) {
  if (fdcanHandle->Instance == FDCAN1) {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  } else if (fdcanHandle->Instance == FDCAN2) {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
  } else if (fdcanHandle->Instance == FDCAN3) {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13);
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn);
  }
}

void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN2_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan2);
}

void FDCAN2_IT1_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan2);
}

void FDCAN3_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan3);
}

void FDCAN3_IT1_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan3);
}
