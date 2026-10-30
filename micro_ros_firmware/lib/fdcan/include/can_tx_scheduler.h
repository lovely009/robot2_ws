// 每总线 TX 软件调度器：P0 安全 > P1 周期控制 > P2 管理流量，每总线一个实例。
// P0（停机/失能/使能/急停）最高优先级、pump 先于一切出队，**P2 积压绝不影响它**；P1（周期
// MIT）每电机槽位 latest-wins、无 FIFO 历史，只发「脏」槽位；P2（发现/配置/管理）有界 FIFO +
// 每次 pump 配额上限。pump() 返回本轮该提交硬件 FIFO 的帧，纯 C、无 HAL/FreeRTOS 依赖、
// 可宿主测试，硬件忙/拒收由调用方经 can_tx_scheduler_note_result() 记账。

#ifndef CAN_TX_SCHEDULER_H
#define CAN_TX_SCHEDULER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TX_BUS_COUNT 3U
#include "firmware_config.h"
#define CAN_TX_P1_SLOTS FIRMWARE_MAX_MOTORS // 每电机一个 latest-wins 槽
#define CAN_TX_P0_CAPACITY 32U     // P0 FIFO，预留每总线 16 台同时撤权的余量。
#define CAN_TX_P2_CAPACITY 16U     // P2 FIFO（ARD §22.3 有界）
#define CAN_TX_P2_PUMP_QUOTA 2U    // 每次 pump 最多出队 P2 帧数（TBD-CAN-01 占位）
#define CAN_TX_FRAME_BYTES 64U

typedef struct {
  uint32_t can_id;
  // 稳定的内核表项槽位；P0/P1 用于同轴取消，P2 忽略。
  uint16_t tx_slot;
  bool is_extended;
  bool is_fd;
  bool is_brs;
  uint8_t size;
  // 协议层聚合透传字段（调度器不解释，只随帧搬运）：CERB 高 NodeID 的
  // 逻辑帧携带显式节点号并标记 aggregate_only，聚合器据此生成 0x210 帧。
  uint8_t node;
  bool aggregate_only;
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
  uint32_t tx_hw_reject_count; // 调用方经 note_hw_reject 记账的硬件拒绝
} can_tx_scheduler_t;

void can_tx_scheduler_init(can_tx_scheduler_t *sched);

// 停机意图使同轴旧 P0（含使能）与 P1 失效；保留其他轴及 P2 的顺序。
// 在路由新帧之前调用，不能等停机帧成功入队/送达才取消旧目标。
void can_tx_scheduler_cancel_slots(can_tx_scheduler_t *sched, uint64_t slots);

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
// 时优先保证 P0/P1，P2 留在队列）。p1_limit 限制本轮 P1 出队数，
// 可为 P2 留出物理发送名额。p0_count/p1_count 非空时返回各自
// 在 out 中的帧数，供调用方区分安全帧、可合并的控制帧和管理帧。
uint32_t can_tx_scheduler_pump(can_tx_scheduler_t *sched, can_tx_frame_t *out,
    uint32_t capacity, uint32_t p1_limit,
    uint32_t *p0_count,
    uint32_t *p1_count);

// 硬件提交结果记账（调用方在 HAL_FDCAN_AddMessageToTxFifoQ 失败时调用）。
void can_tx_scheduler_note_hw_reject(can_tx_scheduler_t *sched);

#ifdef __cplusplus
}
#endif

#endif // CAN_TX_SCHEDULER_H
