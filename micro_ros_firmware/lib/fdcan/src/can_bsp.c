// FDCAN 板级收发层（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植）。
// 官方只给 FDCAN1 配了接收过滤器；本工程对三路都配"全收"过滤器，
// 使 fdcan2/3 的回调同样可用（发送/接收行为与例程保持一致）。

#include "can_bsp.h"
#include "stm32_def.h"  // Error_Handler 宏（_Error_Handler(file, line)）

static FDCAN_HandleTypeDef *const kCanHandles[3] = {&hfdcan1, &hfdcan2, &hfdcan3};

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
  can_filter_init();
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
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

// CAN2/3 暂未接入业务，保留最近一帧即可
static uint8_t rx_data2[64] = {0};
static uint8_t rx_data3[64] = {0};
static uint16_t rec_id2 = 0;
static uint16_t rec_id3 = 0;

// CAN1 最近一帧快照：中断写入，can1_rx_snapshot() 在任务侧读取
static can_rx_frame_t g_can1_frame = {0};
volatile uint32_t can1_rx_counter = 0;

static void fdcan1_rx_callback(void) {
  g_can1_frame.dlc =
      fdcanx_receive_frame(&hfdcan1, &g_can1_frame.id, g_can1_frame.data,
                           &g_can1_frame.is_extended, &g_can1_frame.is_fd);
  can1_rx_counter++;
}

static void fdcan2_rx_callback(void) {
  fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
}

static void fdcan3_rx_callback(void) {
  fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
}

uint32_t can1_rx_snapshot(can_rx_frame_t *out) {
  // 关中断保证 64B 数据与元数据/序号一致；调用方是任务上下文，开销可接受
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t seq = can1_rx_counter;
  if (out != NULL) {
    *out = g_can1_frame;
  }
  __set_PRIMASK(primask);
  return seq;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }
  if (hfdcan == &hfdcan1) {
    fdcan1_rx_callback();
  } else if (hfdcan == &hfdcan2) {
    fdcan2_rx_callback();
  } else if (hfdcan == &hfdcan3) {
    fdcan3_rx_callback();
  }
}
