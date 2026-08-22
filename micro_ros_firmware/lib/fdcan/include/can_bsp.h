// FDCAN 板级收发接口（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植）。
// 发送支持标准 ID 的 FD 帧（BRS），接收经中断回调落到模块内静态缓冲区；
// 业务层通过 can1_rx_snapshot() 取最新帧，不直接触碰裸全局变量。

#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

// CAN1 最近一帧接收快照（中断写入，业务任务轮询读取）
typedef struct {
  uint32_t id;
  uint8_t dlc;
  bool is_extended;
  bool is_fd;
  uint8_t data[64];
} can_rx_frame_t;

#ifdef __cplusplus
extern "C" {
#endif

void bsp_can_init(void);
void can_filter_init(void);

// 返回 0=成功，1=失败（DLC 不支持或 TX FIFO 满）
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, const uint8_t *data, uint32_t len);

// 通用发送：支持标准/扩展 ID、经典/FD 帧；fdcanx_send_data 为其 FD 标准帧特例
uint8_t fdcanx_send_frame(hcan_t *hfdcan, uint32_t id, const uint8_t *data,
                          uint32_t len, bool is_extended, bool is_fd);

// 从 RX FIFO0 取一帧；返回数据长度，无帧返回 0
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);

// 通用接收：额外返回扩展/ FD 标志；fdcanx_receive 为兼容封装
uint8_t fdcanx_receive_frame(hcan_t *hfdcan, uint32_t *rec_id, uint8_t *buf,
                             bool *is_extended, bool *is_fd);

// 取 CAN1 最新接收帧快照；返回自增序号，调用方记录上次序号以判断新帧。
// 在中断内复制，可安全地从任务上下文调用。
uint32_t can1_rx_snapshot(can_rx_frame_t *out);

// 调试计数：CAN1 接收帧总数（GDB 可读）
extern volatile uint32_t can1_rx_counter;

#ifdef __cplusplus
}
#endif

#endif  // CAN_BSP_H
