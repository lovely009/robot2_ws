// FDCAN 外设驱动（官方 CtrBoard-H7_FDCAN 例程移植 + 电机协议适配）。
//
// 官方生成代码：三路 FDCAN（FDCAN1=PD0/PD1、FDCAN2=PB5/PB6、FDCAN3=PD12/PD13），
// FD + BRS、标称 1Mbps / 数据 5Mbps。时钟源在官方例程为 PLL1Q=80MHz，
// 本工程为不动 BMI088 的 PLL1Q=240MHz(SPI2)，改用 PLL2Q=80MHz 供 FDCAN。
// 消息 RAM：RX/TX 元素扩到 64B（CAN-FD 满载帧；vaxor 反馈 12/20B），只保留
// RX FIFO0 + TX FIFO，每路按 HAL 计算占 444 个 32 位字，偏移 0/444/888。
// 中断转发沿用 HAL：FDCANx_IT0/IT1_IRQHandler -> HAL_FDCAN_IRQHandler。

#include "board_irq.h"

#include "fdcan.h"
#include "stm32_def.h"  // Error_Handler 宏（_Error_Handler(file, line)）

// 台架自答模式（platformio.ini 的 FDCAN_SELF_ACK_BENCH，默认关）：HAL 的
// EXTERNAL_LOOPBACK = TEST.LBCK + MON，控制器自答自己的帧——空总线（无
// ACK 节点）也能把 SYNC/命令发上总线供第三方观察，错误计数不再累积。
// MON 代价：本节点对总线上其他节点只收不应——真实电机/完整台架必须关闭。
#ifdef FDCAN_SELF_ACK_BENCH
#define FDCAN_INIT_MODE FDCAN_MODE_EXTERNAL_LOOPBACK
#else
#define FDCAN_INIT_MODE FDCAN_MODE_NORMAL
#endif

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

// 三路共用一个 FDCAN 内核时钟，计数器保证只开关一次
static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

// 三路 FDCAN 的位时序、元素尺寸与滤波配额完全一致，仅 Instance 与消息 RAM
// 偏移不同（每路 444 个 32 位字：RX FIFO0 8 深 + TX FIFO 16 深，64B 元素）。
// 集中在此，避免三份拷贝各自漂移——历史上 DataTimeSeg1 是靠注释维持一致的。
//
// 位时序（FDCAN 时钟 = PLL2Q 80MHz，system_clock 单点配置）：
//   仲裁：N=1, SJW=16, Seg1=63, Seg2=16 → 80 tq/bit → 1.0 Mbps
//   数据：N=1, SJW=2,  Seg1=12, Seg2=3  → 16 tq/bit → 5.0 Mbps
// 数值是台架实测基线，无示波器/总线分析仪不得改动。
#define FDCAN_ARB_BITRATE_PRESCALER 1U
#define FDCAN_ARB_SJW_TQ             16U
#define FDCAN_ARB_SEG1_TQ            63U
#define FDCAN_ARB_SEG2_TQ            16U
#define FDCAN_DATA_BITRATE_PRESCALER 1U
#define FDCAN_DATA_SJW_TQ            2U
#define FDCAN_DATA_SEG1_TQ           12U
#define FDCAN_DATA_SEG2_TQ           3U

// HAL 消息 RAM：标准过滤器 4 字、扩展过滤器 4×2 字，64B 元素每个 18 字。
#define FDCAN_MESSAGE_RAM_WORDS_PER_BUS \
  (4U + 4U * 2U + 8U * FDCAN_DATA_BYTES_64 + 16U * FDCAN_DATA_BYTES_64)
_Static_assert(FDCAN_MESSAGE_RAM_WORDS_PER_BUS == 444U,
    "FDCAN 消息 RAM 布局计算必须与 HAL 一致");
_Static_assert(3U * FDCAN_MESSAGE_RAM_WORDS_PER_BUS < 2560U,
    "三路 FDCAN 消息 RAM 不得超过 10KB");
static void fdcan_apply_init(FDCAN_HandleTypeDef *handle,
    FDCAN_GlobalTypeDef *instance,
    uint32_t message_ram_offset) {
  handle->Instance = instance;
  handle->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  handle->Init.Mode = FDCAN_INIT_MODE;
  handle->Init.AutoRetransmission = ENABLE;
  handle->Init.TransmitPause = DISABLE;
  handle->Init.ProtocolException = ENABLE;
  handle->Init.NominalPrescaler = FDCAN_ARB_BITRATE_PRESCALER;
  handle->Init.NominalSyncJumpWidth = FDCAN_ARB_SJW_TQ;
  handle->Init.NominalTimeSeg1 = FDCAN_ARB_SEG1_TQ;
  handle->Init.NominalTimeSeg2 = FDCAN_ARB_SEG2_TQ;
  handle->Init.DataPrescaler = FDCAN_DATA_BITRATE_PRESCALER;
  handle->Init.DataSyncJumpWidth = FDCAN_DATA_SJW_TQ;
  handle->Init.DataTimeSeg1 = FDCAN_DATA_SEG1_TQ;
  handle->Init.DataTimeSeg2 = FDCAN_DATA_SEG2_TQ;
  handle->Init.MessageRAMOffset = message_ram_offset;
  handle->Init.StdFiltersNbr = 4;
  handle->Init.ExtFiltersNbr = 4;
  handle->Init.RxFifo0ElmtsNbr = 8;
  handle->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  handle->Init.RxFifo1ElmtsNbr = 0;
  handle->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  handle->Init.RxBuffersNbr = 0;
  handle->Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  handle->Init.TxEventsNbr = 0;
  handle->Init.TxBuffersNbr = 0;
  handle->Init.TxFifoQueueElmtsNbr = 16;
  handle->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  handle->Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(handle) != HAL_OK) {
    Error_Handler();
  }
}

// 消息 RAM 偏移必须按路递增且不重叠（见文件头说明）。
void MX_FDCAN1_Init(void) {
  fdcan_apply_init(&hfdcan1, FDCAN1, 0U);
}

void MX_FDCAN2_Init(void) {
  fdcan_apply_init(&hfdcan2, FDCAN2, FDCAN_MESSAGE_RAM_WORDS_PER_BUS);
}

void MX_FDCAN3_Init(void) {
  fdcan_apply_init(&hfdcan3, FDCAN3, 2U * FDCAN_MESSAGE_RAM_WORDS_PER_BUS);
}

// 板级引脚/中断向量表：三路只有端口、引脚、AF 与 IT0/IT1 不同，其余
// （AF 推挽、无上下拉、VeryHigh 速度、同一 NVIC 优先级）完全相同。
typedef struct {
  FDCAN_GlobalTypeDef *instance;
  GPIO_TypeDef *port;
  uint32_t pins;
  uint32_t alternate;
  IRQn_Type it0;
  IRQn_Type it1;
} fdcan_board_pin_t;

static const fdcan_board_pin_t kFdcanPins[] = {
  // FDCAN1=PD0/PD1（AF9）
  {FDCAN1, GPIOD, GPIO_PIN_0 | GPIO_PIN_1, GPIO_AF9_FDCAN1, FDCAN1_IT0_IRQn,
   FDCAN1_IT1_IRQn},
  // FDCAN2=PB5/PB6（AF9）
  {FDCAN2, GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_AF9_FDCAN2, FDCAN2_IT0_IRQn,
   FDCAN2_IT1_IRQn},
  // FDCAN3=PD12/PD13（AF5）
  {FDCAN3, GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_AF5_FDCAN3,
   FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn},
};

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {
  GPIO_InitTypeDef gpio = {0};
  for (size_t i = 0; i < sizeof(kFdcanPins) / sizeof(kFdcanPins[0]); ++i) {
    const fdcan_board_pin_t *row = &kFdcanPins[i];
    if (fdcanHandle->Instance != row->instance) {
      continue;
    }
    // 三路共用一个 FDCAN 内核时钟，计数保证只开一次
    if (HAL_RCC_FDCAN_CLK_ENABLED++ == 0U) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }
    // 引脚时钟：FDCAN2 在 GPIOB，FDCAN1/3 在 GPIOD（重复使能幂等）
    if (row->port == GPIOB) {
      __HAL_RCC_GPIOB_CLK_ENABLE();
    } else {
      __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    gpio.Pin = row->pins;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = row->alternate;
    HAL_GPIO_Init(row->port, &gpio);

    HAL_NVIC_SetPriority(row->it0, BOARD_ISR_SYSCALL_THRESHOLD, 0);
    HAL_NVIC_EnableIRQ(row->it0);
    HAL_NVIC_SetPriority(row->it1, BOARD_ISR_SYSCALL_THRESHOLD, 0);
    HAL_NVIC_EnableIRQ(row->it1);
    return;
  }
  // 未登记的外设实例：不初始化（由调用方负责报错）。
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
