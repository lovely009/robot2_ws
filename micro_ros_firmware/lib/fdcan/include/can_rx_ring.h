// 每总线静态 RX 环（ARD-0007 §21.1/§21.2）：FDCAN ISR 为唯一生产者，
// motor_rt_task 为唯一消费者（SPSC 无锁）。
//
// 并发约定：push 只在 ISR 上下文调用（只推进 head）；pop/pending 只在任务
// 上下文调用（只推进 tail）。head/tail 均 32 位自然对齐，Cortex-M7 上
// 单加载/存储原子。环满丢弃**新帧**并计数，不移动 tail，因为 ISR 侧不能
// 回收仍可能由消费者读取的槽位（ARD §51.1）。
//
// 帧元素含 ISR 盖章的 MCU 单调接收时戳（rx_tick_ms，与内核 millis() 同一
// HAL 计数器——见 can_bsp.c isr_now_ms）与 is_brs 属性（ARD §21.2）；
// bus_index 由 ISR 填写。

#ifndef CAN_RX_RING_H
#define CAN_RX_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_RX_BUS_COUNT 3U
#define CAN_RX_RING_CAPACITY 64U // ARD §21.1：初始深度 64，仅台架数据可调
#define CAN_RX_FRAME_BYTES 64U

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  bool is_extended;
  bool is_fd;
  bool is_brs;
  uint8_t dlc;
  uint32_t rx_tick_ms; // ISR 盖章：HAL/millis 同一单调时基毫秒（勿用 FreeRTOS
                       // tick——其起点为调度器启动，与 millis 存在启动期恒定偏移）
  uint8_t data[CAN_RX_FRAME_BYTES];
} can_rx_frame_t;

typedef struct {
  can_rx_frame_t slots[CAN_RX_RING_CAPACITY];
  volatile uint32_t head; // ISR 写序号（单调递增，槽位 = head % CAPACITY）
  volatile uint32_t tail; // 任务读序号（单调递增，槽位 = tail % CAPACITY）
  // 诊断（ARD §21.4）：投递总数与环满丢弃数，统计对账 rx_count = 弹出 + 在环。
  uint32_t rx_count;
  uint32_t overflow_count;
} can_rx_ring_t;

// 静态初始化（零态即可用，此函数用于显式复位与可读性）。
void can_rx_ring_init(can_rx_ring_t *ring);

// ISR 侧入环。返回 false = 环满（新帧被丢弃，overflow_count 已自增）。
bool can_rx_ring_push_from_isr(can_rx_ring_t *ring, const can_rx_frame_t *frame);

// 任务侧批量弹出（最多 max_count 帧），返回实际弹出数。
uint32_t can_rx_ring_pop(can_rx_ring_t *ring, can_rx_frame_t *out, uint32_t max_count);

// 任务侧查询在环帧数。
uint32_t can_rx_ring_pending(const can_rx_ring_t *ring);

#ifdef __cplusplus
}
#endif

#endif // CAN_RX_RING_H
