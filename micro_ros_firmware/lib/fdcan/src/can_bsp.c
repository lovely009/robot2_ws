// FDCAN 板级收发层（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植后按
// ARD-0007 §21/§22 收敛）：
//  - RX：ISR 只取硬件 FIFO 帧、盖单调时戳、写每总线 can_rx_ring（SPSC），
//    不解码、不建运行时状态；
//  - TX：每总线 can_tx_scheduler，pump 结果经 can_bsp_submit_tx 进硬件 FIFO；
//  - 统计：ring/调度器自带计数 + 本层硬件级错误计数（ARD §32.1）。
//  - 所有权：motor_rt_task 是 RX ring 唯一消费者，调试桥不得形成第二条 RX 路径。

#include "can_bsp.h"
#include "stm32_def.h" // Error_Handler 宏（_Error_Handler(file, line)）+ HAL_GetTick

#include <string.h>

static FDCAN_HandleTypeDef *const kCanHandles[CAN_RX_BUS_COUNT] = {
    &hfdcan1, &hfdcan2, &hfdcan3};

static can_rx_ring_t g_rx_rings[CAN_RX_BUS_COUNT];
static can_tx_scheduler_t g_tx_schedulers[CAN_TX_BUS_COUNT];
static volatile uint32_t g_bus_stats[CAN_RX_BUS_COUNT][CAN_BSP_STAT_COUNT];

can_rx_ring_t *can_bsp_rx_ring(uint8_t bus_index) {
  if (bus_index >= CAN_RX_BUS_COUNT) {
    return NULL;
  }
  return &g_rx_rings[bus_index];
}

can_tx_scheduler_t *can_bsp_tx_scheduler(uint8_t bus_index) {
  if (bus_index >= CAN_TX_BUS_COUNT) {
    return NULL;
  }
  return &g_tx_schedulers[bus_index];
}

uint32_t can_bsp_bus_stat(uint8_t bus_index, uint8_t stat_index) {
  if (bus_index >= CAN_RX_BUS_COUNT || stat_index >= CAN_BSP_STAT_COUNT) {
    return 0U;
  }
  return g_bus_stats[bus_index][stat_index];
}

// FD 帧仅允许 0-8/12/16/20/24/32/48/64 字节；DLC 代码即 HAL 宏值
static uint32_t fdcan_bytes_to_dlc(uint32_t len) {
  switch (len) {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    case 8: return FDCAN_DLC_BYTES_8;
    case 12: return FDCAN_DLC_BYTES_12;
    case 16: return FDCAN_DLC_BYTES_16;
    case 20: return FDCAN_DLC_BYTES_20;
    case 24: return FDCAN_DLC_BYTES_24;
    case 32: return FDCAN_DLC_BYTES_32;
    case 48: return FDCAN_DLC_BYTES_48;
    case 64: return FDCAN_DLC_BYTES_64;
    default: return 0xFFFFFFFFU;
  }
}

static uint32_t fdcan_dlc_to_bytes(uint32_t dlc) {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:
    case FDCAN_DLC_BYTES_1:
    case FDCAN_DLC_BYTES_2:
    case FDCAN_DLC_BYTES_3:
    case FDCAN_DLC_BYTES_4:
    case FDCAN_DLC_BYTES_5:
    case FDCAN_DLC_BYTES_6:
    case FDCAN_DLC_BYTES_7:
    case FDCAN_DLC_BYTES_8:
      return dlc;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
  }
}

void can_filter_init(void) {
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x00;
  filter.FilterID2 = 0x00;

  // Mask=0：不过滤任何标准 ID（发现/热插拔依赖全收，TBD-CAN-02 再收敛）；
  // 非匹配/远程帧仍按例程拒绝。
  for (uint32_t i = 0; i < 3; i++) {
    if (HAL_FDCAN_ConfigFilter(kCanHandles[i], &filter) != HAL_OK) {
      Error_Handler();
    }
    HAL_FDCAN_ConfigGlobalFilter(kCanHandles[i], FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  }
}

void bsp_can_init(void) {
  for (uint8_t bus = 0U; bus < CAN_RX_BUS_COUNT; ++bus) {
    can_rx_ring_init(&g_rx_rings[bus]);
  }
  for (uint8_t bus = 0U; bus < CAN_TX_BUS_COUNT; ++bus) {
    can_tx_scheduler_init(&g_tx_schedulers[bus]);
  }
  memset((void *)g_bus_stats, 0, sizeof(g_bus_stats));
  can_filter_init();
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);
  const uint32_t rx_notifications =
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
      FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
  HAL_FDCAN_ActivateNotification(&hfdcan1, rx_notifications, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, rx_notifications, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, rx_notifications, 0);
  // 错误状态（EW/EP/BUS_OFF）计数——台架 T12 故障恢复路径需要可见性。
  const uint32_t error_notifications =
      FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF;
  HAL_FDCAN_ActivateNotification(&hfdcan1, error_notifications, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, error_notifications, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, error_notifications, 0);
}

bool can_bsp_submit_tx(uint8_t bus_index, const can_tx_frame_t *frame) {
  if (bus_index >= CAN_TX_BUS_COUNT || frame == NULL ||
      frame->size > CAN_TX_FRAME_BYTES) {
    return false;
  }
  const uint32_t dlc = fdcan_bytes_to_dlc(frame->size);
  if (dlc == 0xFFFFFFFFU) {
    can_tx_scheduler_note_hw_reject(&g_tx_schedulers[bus_index]);
    return false;
  }
  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier = frame->can_id;
  tx.IdType = frame->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = dlc;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = frame->is_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx.FDFormat = frame->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0;
  if (HAL_FDCAN_AddMessageToTxFifoQ(kCanHandles[bus_index], &tx,
                                    (uint8_t *)frame->data) != HAL_OK) {
    // 硬件 FIFO 满/拒绝：记账后交由上层策略（调度器下轮重泵）。
    can_tx_scheduler_note_hw_reject(&g_tx_schedulers[bus_index]);
    return false;
  }
  return true;
}

static int8_t can_bus_index(FDCAN_HandleTypeDef *hfdcan) {
  for (uint8_t bus_index = 0U; bus_index < CAN_RX_BUS_COUNT; ++bus_index) {
    if (hfdcan == kCanHandles[bus_index]) {
      return (int8_t)bus_index;
    }
  }
  return -1;
}

// 单调时基毫秒（ISR 上下文安全）：HAL_GetTick 读 32 位对齐 volatile uwTick，
// 单加载无读改写，Cortex-M7 上原子。必须与内核 millis()（= HAL_GetTick）同基——
// 内核用帧时戳与 now_ms 做帧龄比较（离线判定/状态新鲜度），两侧若不同基
// （如 FreeRTOS tick，起点为调度器启动）会引入启动期恒定偏移导致误判。
static uint32_t isr_now_ms(void) {
  return HAL_GetTick();
}

static bool fdcan_receive_into_frame(FDCAN_HandleTypeDef *hfdcan,
                                     uint8_t bus_index, uint32_t rx_tick_ms,
                                     can_rx_frame_t *frame) {
  FDCAN_RxHeaderTypeDef rx = {0};
  *frame = (can_rx_frame_t){0};
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, frame->data) !=
      HAL_OK) {
    return false;
  }
  frame->bus_index = bus_index;
  frame->can_id = rx.Identifier;
  frame->dlc = (uint8_t)fdcan_dlc_to_bytes(rx.DataLength);
  frame->is_extended = (rx.IdType == FDCAN_EXTENDED_ID);
  frame->is_fd = (rx.FDFormat == FDCAN_FD_CAN);
  frame->is_brs = (rx.BitRateSwitch == FDCAN_BRS_ON);
  frame->rx_tick_ms = rx_tick_ms;
  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  int8_t bus_index = can_bus_index(hfdcan);
  if (bus_index < 0) {
    return;
  }
  const uint8_t bus = (uint8_t)bus_index;

  const uint32_t handled_notifications =
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
      FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
  if ((RxFifo0ITs & handled_notifications) == 0U) {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
    g_bus_stats[bus][CAN_BSP_STAT_HW_FIFO_LOST]++;
  }

  // 硬件 FIFO 深度为 8；只处理进入回调时已挂起的帧，后到帧由下一次 IRQ
  // 继续承接，避免持续满载时在单次 ISR 中无界循环。
  const uint32_t rx_tick_ms = isr_now_ms();
  uint32_t pending_frames =
      HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
  for (uint32_t index = 0U; index < pending_frames; ++index) {
    can_rx_frame_t frame = {0};
    if (!fdcan_receive_into_frame(hfdcan, bus, rx_tick_ms, &frame)) {
      g_bus_stats[bus][CAN_BSP_STAT_HAL_READ_ERROR]++;
      break;
    }
    // SPSC 入环：满则丢新帧并计数（can_rx_ring 内部完成，ARD §21.3）。
    (void)can_rx_ring_push_from_isr(&g_rx_rings[bus], &frame);
  }
}

// 错误状态事件（EW/EP/BUS_OFF）：仅计数，恢复策略待台架 T12 数据。
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  int8_t bus_index = can_bus_index(hfdcan);
  if (bus_index < 0) {
    return;
  }
  if (ErrorStatusITs != 0U) {
    g_bus_stats[(uint8_t)bus_index][CAN_BSP_STAT_ERROR_STATUS]++;
  }
}
