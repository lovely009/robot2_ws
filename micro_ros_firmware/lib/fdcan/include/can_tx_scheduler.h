// 每总线 TX 软件调度器（ARD-0007 §22 / §37）：P0 安全 > P1 周期控制 >
// P2 管理流量。每总线一个实例（三路硬件独立调度）。
//
// 语义：
//  - P0（停机/失能/使能/急停/安全帧）：最高优先级，pump 时先于一切出队；
//    上层（运行时内核）已按电机去重，本层为有界 FIFO，满则丢新并计数；
//    P2 队列积压绝不影响 P0 出队（§22.1 MUST）。
//  - P1（100Hz MIT）：每个电机槽位 latest-wins——新目标覆盖未发送旧目标
//    （slot 为电机在总线上的槽索引，0..CAN_TX_P1_SLOTS-1），无 FIFO 历史
//    （禁止项 5）。pump 只发送"脏"槽位一次。
//  - P2（发现/配置/管理）：有界 FIFO + 每次 pump 配额上限，绝不让 P2
//    挤占 P0/P1（TBD-CAN-01：配额值为具名占位常量）。
//
// pump() 返回本轮应提交到硬件 FIFO 的帧（调用方按序 HAL_FDCAN_AddMessage）。
// 纯 C、无 HAL/FreeRTOS 依赖，可宿主测试；硬件忙/拒绝的反馈由调用方
// 统计后通过 can_tx_scheduler_note_result() 记账。

#ifndef CAN_TX_SCHEDULER_H
#define CAN_TX_SCHEDULER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TX_BUS_COUNT 3U
#define CAN_TX_P1_SLOTS 64U        // 每电机一个 latest-wins 槽（对齐注册表容量）
#define CAN_TX_P0_CAPACITY 16U     // P0 FIFO（上层已去重，容量可小）
#define CAN_TX_P2_CAPACITY 16U     // P2 FIFO（ARD §22.3 有界）
#define CAN_TX_P2_PUMP_QUOTA 2U    // 每次 pump 最多出队 P2 帧数（TBD-CAN-01 占位）
#define CAN_TX_FRAME_BYTES 64U

typedef struct {
  uint32_t can_id;
  bool is_extended;
  bool is_fd;
  bool is_brs;
  uint8_t size;
  uint8_t data[CAN_TX_FRAME_BYTES];
} can_tx_frame_t;

typedef struct {
  can_tx_frame_t queue[CAN_TX_P0_CAPACITY];
  uint8_t head;
  uint8_t count;
  uint32_t dropped_count; // 满丢新帧计数
} can_tx_p0_t;

typedef struct {
  can_tx_frame_t slots[CAN_TX_P1_SLOTS];
  bool dirty[CAN_TX_P1_SLOTS];
  uint32_t replaced_count; // 新目标覆盖未发送旧目标的次数（§32.1 p1_replaced）
} can_tx_p1_t;

typedef struct {
  can_tx_frame_t queue[CAN_TX_P2_CAPACITY];
  uint8_t head;
  uint8_t count;
  uint32_t dropped_count;
} can_tx_p2_t;

typedef struct {
  can_tx_p0_t p0;
  can_tx_p1_t p1;
  can_tx_p2_t p2;
  uint32_t p0_sent;
  uint32_t p1_sent;
  uint32_t p2_sent;
  uint32_t tx_hw_reject_count; // 调用方经 note_result 记账的硬件拒绝
} can_tx_scheduler_t;

void can_tx_scheduler_init(can_tx_scheduler_t *sched);

// P0：入队（满 = 丢新帧 + dropped_count，返回 false；任务层将失败回告内核
// motor_control_requeue_p0 重入队，确保安全帧不静默丢弃）。
bool can_tx_scheduler_submit_p0(can_tx_scheduler_t *sched,
                                const can_tx_frame_t *frame);

// 硬件拒收的安全帧回插 P0 队首（下一 pump 先于一切出队；队满返回 false，
// 正常调用时序下不会发生，因为刚从 P0 弹出一帧）。
bool can_tx_scheduler_requeue_p0_front(can_tx_scheduler_t *sched,
                                       const can_tx_frame_t *frame);

// P1：写入电机槽位（latest-wins）。槽脏且未发送时再写 → replaced_count++。
bool can_tx_scheduler_submit_p1(can_tx_scheduler_t *sched, uint16_t slot,
                                const can_tx_frame_t *frame);

// P2：入队（满 = 丢新帧 + dropped_count，返回 false）。
bool can_tx_scheduler_submit_p2(can_tx_scheduler_t *sched,
                                const can_tx_frame_t *frame);

// 出队本轮待发送帧：先排空 P0，再发全部脏 P1 槽，最后 P2 受配额约束。
// 返回实际出队帧数；容量不足时余量保留（P0 绝不因容量被跳过——容量不足
// 时优先保证 P0/P1，P2 留在队列）。p0_count 非空时写入 out 前缀中的 P0 帧
// 数，供调用方在硬件拒收时识别安全帧并回插。
uint32_t can_tx_scheduler_pump(can_tx_scheduler_t *sched, can_tx_frame_t *out,
                               uint32_t capacity, uint32_t *p0_count);

// 硬件提交结果记账（调用方在 HAL_FDCAN_AddMessageToTxFifoQ 失败时调用）。
void can_tx_scheduler_note_hw_reject(can_tx_scheduler_t *sched);

#ifdef __cplusplus
}
#endif

#endif // CAN_TX_SCHEDULER_H
