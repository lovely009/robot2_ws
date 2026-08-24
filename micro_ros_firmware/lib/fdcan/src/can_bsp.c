// FDCAN 板级收发层（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植）。
// 官方只给 FDCAN1 配了接收过滤器；本工程对三路都配"全收"过滤器，
// 使 fdcan2/3 的回调同样可用（发送/接收行为与例程保持一致）。

#include <STM32FreeRTOS.h>

#include "can_bsp.h"
#include "stm32_def.h"  // Error_Handler 宏（_Error_Handler(file, line)）

static FDCAN_HandleTypeDef *const kCanHandles[3] = {&hfdcan1, &hfdcan2, &hfdcan3};
static const UBaseType_t kRxQueueCapacities[CAN_RX_BUS_COUNT] = {
    CAN1_RX_QUEUE_CAPACITY,
    CAN_OTHER_RX_QUEUE_CAPACITY,
    CAN_OTHER_RX_QUEUE_CAPACITY,
};

static StaticQueue_t g_rx_queue_controls[CAN_RX_BUS_COUNT];
static uint8_t g_can1_rx_storage[CAN1_RX_QUEUE_CAPACITY *
                                 sizeof(can_rx_frame_t)];
static uint8_t g_can2_rx_storage[CAN_OTHER_RX_QUEUE_CAPACITY *
                                 sizeof(can_rx_frame_t)];
static uint8_t g_can3_rx_storage[CAN_OTHER_RX_QUEUE_CAPACITY *
                                 sizeof(can_rx_frame_t)];
static uint8_t *const kRxQueueStorage[CAN_RX_BUS_COUNT] = {
    g_can1_rx_storage,
    g_can2_rx_storage,
    g_can3_rx_storage,
};
static QueueHandle_t g_rx_queues[CAN_RX_BUS_COUNT] = {NULL, NULL, NULL};
static volatile can_rx_diagnostics_t g_rx_diagnostics[CAN_RX_BUS_COUNT] = {0};

static void can_rx_queues_init(void) {
  for (uint8_t bus_index = 0U; bus_index < CAN_RX_BUS_COUNT; ++bus_index) {
    if (g_rx_queues[bus_index] != NULL) {
      continue;
    }
    g_rx_queues[bus_index] = xQueueCreateStatic(
        kRxQueueCapacities[bus_index], sizeof(can_rx_frame_t),
        kRxQueueStorage[bus_index], &g_rx_queue_controls[bus_index]);
    configASSERT(g_rx_queues[bus_index] != NULL);
  }
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

  // Mask=0：不过滤任何标准 ID；非匹配/远程帧仍按例程拒绝
  for (uint32_t i = 0; i < 3; i++) {
    if (HAL_FDCAN_ConfigFilter(kCanHandles[i], &filter) != HAL_OK) {
      Error_Handler();
    }
    HAL_FDCAN_ConfigGlobalFilter(kCanHandles[i], FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  }

  // 官方仅在 FDCAN1 配 FIFO0 水位 1，保持原样
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

void bsp_can_init(void) {
  can_rx_queues_init();
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
}

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, const uint8_t *data, uint32_t len) {
  return fdcanx_send_frame(hfdcan, id, data, len, false, true);
}

uint8_t fdcanx_send_frame(hcan_t *hfdcan, uint32_t id, const uint8_t *data,
                          uint32_t len, bool is_extended, bool is_fd) {
  uint32_t dlc = fdcan_bytes_to_dlc(len);
  if (dlc == 0xFFFFFFFFU) {
    return 1;
  }

  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier = id;
  tx.IdType = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = dlc;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = is_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx.FDFormat = is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, (uint8_t *)data) != HAL_OK) {
    return 1;
  }
  return 0;
}

uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf) {
  uint32_t id32 = 0;
  bool is_extended = false;
  bool is_fd = false;
  uint8_t len = fdcanx_receive_frame(hfdcan, &id32, buf, &is_extended, &is_fd);
  *rec_id = (uint16_t)id32;
  return len;
}

uint8_t fdcanx_receive_frame(hcan_t *hfdcan, uint32_t *rec_id, uint8_t *buf,
                             bool *is_extended, bool *is_fd) {
  FDCAN_RxHeaderTypeDef rx = {0};
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK) {
    return 0;
  }
  *rec_id = rx.Identifier;
  *is_extended = (rx.IdType == FDCAN_EXTENDED_ID);
  *is_fd = (rx.FDFormat == FDCAN_FD_CAN);
  return (uint8_t)fdcan_dlc_to_bytes(rx.DataLength);
}

bool can_rx_queue_pop(uint8_t bus_index, can_rx_frame_t *out) {
  if (bus_index >= CAN_RX_BUS_COUNT || out == NULL ||
      g_rx_queues[bus_index] == NULL) {
    return false;
  }
  return xQueueReceive(g_rx_queues[bus_index], out, 0U) == pdPASS;
}

uint32_t can_rx_queue_pending(uint8_t bus_index) {
  if (bus_index >= CAN_RX_BUS_COUNT || g_rx_queues[bus_index] == NULL) {
    return 0U;
  }
  return (uint32_t)uxQueueMessagesWaiting(g_rx_queues[bus_index]);
}

void can_rx_get_diagnostics(uint8_t bus_index, can_rx_diagnostics_t *out) {
  if (out == NULL) {
    return;
  }
  if (bus_index >= CAN_RX_BUS_COUNT) {
    *out = (can_rx_diagnostics_t){0};
    return;
  }
  taskENTER_CRITICAL();
  out->rx_count = g_rx_diagnostics[bus_index].rx_count;
  out->overflow_count = g_rx_diagnostics[bus_index].overflow_count;
  out->hal_receive_error_count =
      g_rx_diagnostics[bus_index].hal_receive_error_count;
  taskEXIT_CRITICAL();
}

static int8_t can_bus_index(FDCAN_HandleTypeDef *hfdcan) {
  for (uint8_t bus_index = 0U; bus_index < CAN_RX_BUS_COUNT; ++bus_index) {
    if (hfdcan == kCanHandles[bus_index]) {
      return (int8_t)bus_index;
    }
  }
  return -1;
}

static bool fdcan_receive_into_frame(FDCAN_HandleTypeDef *hfdcan,
                                     uint8_t bus_index,
                                     can_rx_frame_t *frame) {
  FDCAN_RxHeaderTypeDef rx = {0};
  *frame = (can_rx_frame_t){0};
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, frame->data) !=
      HAL_OK) {
    return false;
  }
  frame->bus_index = bus_index;
  frame->id = rx.Identifier;
  frame->dlc = (uint8_t)fdcan_dlc_to_bytes(rx.DataLength);
  frame->is_extended = (rx.IdType == FDCAN_EXTENDED_ID);
  frame->is_fd = (rx.FDFormat == FDCAN_FD_CAN);
  return true;
}

static void can_rx_queue_push_from_isr(uint8_t bus_index,
                                       const can_rx_frame_t *frame,
                                       BaseType_t *higher_priority_task_woken) {
  QueueHandle_t queue = g_rx_queues[bus_index];
  configASSERT(queue != NULL);
  ++g_rx_diagnostics[bus_index].rx_count;
  if (xQueueSendFromISR(queue, frame, higher_priority_task_woken) == pdPASS) {
    return;
  }

  can_rx_frame_t discarded = {0};
  BaseType_t receive_result =
      xQueueReceiveFromISR(queue, &discarded, higher_priority_task_woken);
  configASSERT(receive_result == pdPASS);
  if (receive_result != pdPASS) {
    return;
  }
  ++g_rx_diagnostics[bus_index].overflow_count;
  BaseType_t send_result =
      xQueueSendFromISR(queue, frame, higher_priority_task_woken);
  configASSERT(send_result == pdPASS);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  int8_t bus_index = can_bus_index(hfdcan);
  if (bus_index < 0) {
    return;
  }

  const uint32_t handled_notifications =
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
      FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
  if ((RxFifo0ITs & handled_notifications) == 0U) {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
    ++g_rx_diagnostics[(uint8_t)bus_index].hal_receive_error_count;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;
  // 硬件 FIFO 深度为 8；只处理进入回调时已挂起的帧，后到帧由下一次 IRQ
  // 继续承接，避免持续满载时在单次 ISR 中无界循环。
  uint32_t pending_frames =
      HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
  for (uint32_t index = 0U; index < pending_frames; ++index) {
    can_rx_frame_t frame = {0};
    if (!fdcan_receive_into_frame(hfdcan, (uint8_t)bus_index, &frame)) {
      ++g_rx_diagnostics[(uint8_t)bus_index].hal_receive_error_count;
      break;
    }
    can_rx_queue_push_from_isr((uint8_t)bus_index, &frame,
                               &higher_priority_task_woken);
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
