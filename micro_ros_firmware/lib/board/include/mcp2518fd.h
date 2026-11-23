#ifndef MCP2518FD_H
#define MCP2518FD_H

#include "can_rx_ring.h"
#include "can_tx_scheduler.h"

// 上电通过内部回环验证 SPI、消息 RAM、FIFO 与 INT，再切换到正常模式。
// 所有 SPI 操作均由 motor_rt 执行。某一路离线不会中断主控启动。
bool mcp2518fd_init(void);
bool mcp2518fd_ready(void);
bool mcp2518fd_transmit(const can_tx_frame_t *frame, uint8_t dlc);
void mcp2518fd_drain(can_rx_ring_t *ring, uint32_t now_ms);
uint32_t mcp2518fd_rx_overflows(void);

#endif
