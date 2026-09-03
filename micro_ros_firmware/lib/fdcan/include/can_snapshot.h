// CAN RX 最新帧快照；由 CAN 数据层维护，消费者只读取副本。

#ifndef CAN_SNAPSHOT_H
#define CAN_SNAPSHOT_H

#include "can_rx_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void can_snapshot_store(uint8_t bus_index, const can_rx_frame_t *frame);
uint32_t can_snapshot_read(uint8_t bus_index, can_rx_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif  // CAN_SNAPSHOT_H