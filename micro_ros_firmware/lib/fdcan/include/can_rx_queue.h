// FDCAN RX 队列的纯数据接口；不依赖 HAL，便于业务层与 host 测试复用。

#ifndef CAN_RX_QUEUE_H
#define CAN_RX_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_RX_BUS_COUNT 3U
#define CAN1_RX_QUEUE_CAPACITY 32U
#define CAN_OTHER_RX_QUEUE_CAPACITY 8U

typedef struct {
  uint8_t bus_index;
  uint32_t id;
  uint8_t dlc;
  bool is_extended;
  bool is_fd;
  uint8_t data[64];
} can_rx_frame_t;

typedef struct {
  uint32_t rx_count;
  uint32_t overflow_count;
  uint32_t hal_receive_error_count;
} can_rx_diagnostics_t;

#ifdef __cplusplus
extern "C" {
#endif

// 主队列只允许 control task 消费；无帧或参数非法时返回 false。
bool can_rx_queue_pop(uint8_t bus_index, can_rx_frame_t *out);

// 返回当前积压帧数；参数非法或队列尚未初始化时返回 0。
uint32_t can_rx_queue_pending(uint8_t bus_index);

void can_rx_get_diagnostics(uint8_t bus_index, can_rx_diagnostics_t *out);

#ifdef __cplusplus
}
#endif

#endif  // CAN_RX_QUEUE_H
