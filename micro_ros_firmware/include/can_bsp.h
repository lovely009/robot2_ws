// FDCAN 板级收发接口（官方 CtrBoard-H7_FDCAN 例程 bsp_fdcan.c 移植）。
// 发送支持标准 ID 的 FD 帧（BRS），接收经中断回调落到静态缓冲区。

#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

// CAN1 最近一帧接收元数据（供 micro-ROS 调试桥轮询；数据仍在 rx_data1）
typedef struct {
  uint32_t id;
  uint8_t dlc;
  bool is_extended;
  bool is_fd;
} can_rx_meta_t;

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

// 中断回调内读取，供业务轮询 rx_data1/2/3
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

// CAN1 调试桥：回调自增计数器，micro-ROS 定时器按计数变化消费最新帧
extern volatile uint32_t can1_rx_counter;
extern can_rx_meta_t can1_rx_meta;

extern uint8_t rx_data1[64];
extern uint8_t rx_data2[64];
extern uint8_t rx_data3[64];
extern uint16_t rec_id1;
extern uint16_t rec_id2;
extern uint16_t rec_id3;

#ifdef __cplusplus
}
#endif

#endif  // CAN_BSP_H
