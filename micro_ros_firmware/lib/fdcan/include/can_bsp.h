// FDCAN 板级收发接口（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植）。
// 发送支持标准 ID 的 FD 帧（BRS），接收经中断回调落到三路静态队列；
// control task 是主队列的唯一消费者。

#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "can_rx_queue.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

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

#ifdef __cplusplus
}
#endif

#endif  // CAN_BSP_H
