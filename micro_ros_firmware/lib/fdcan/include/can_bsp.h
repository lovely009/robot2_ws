// FDCAN 板级接口：初始化、过滤器、IRQ、RX ring、TX 调度与硬件统计。
// ISR 只取帧、盖单调时戳并写 SPSC ring；motor_rt_task 是唯一消费者。
// TX 经 P0/P1/P2 调度后提交硬件；本层不做协议解码、ROS 或策略裁决。

#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "can_rx_ring.h"
#include "can_tx_scheduler.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 RX ring/TX 调度器、过滤器、启动三路硬件并激活中断
// （RXFIFO0 新帧/满/丢失 + 错误状态）。single-init 约定：仅在上电初始化路径
// 调用一次，无重入保护，不在重连时重建。
void bsp_can_init(void);
void can_filter_init(void);

// 总线 RX ring 访问（任务侧批量弹出；ISR 在内部写入）。
can_rx_ring_t *can_bsp_rx_ring(uint8_t bus_index);

// 总线 TX 调度器访问（上层路由 P0/P1/P2 帧后 pump）。
can_tx_scheduler_t *can_bsp_tx_scheduler(uint8_t bus_index);

// 提交一帧到该总线硬件 TX FIFO（pump 结果逐帧调用）。
// 失败（DLC 不支持/FIFO 满）自动计入该总线调度器的 tx_hw_reject_count。
bool can_bsp_submit_tx(uint8_t bus_index, const can_tx_frame_t *frame);


// 每总线硬件 RX FIFO0 丢帧计数；FIFO 深度为 8，丢失不可恢复。
uint32_t can_bsp_rx_hw_lost(uint8_t bus_index);

// 每总线成功提交到硬件 TX FIFO 的帧数。
uint32_t can_bsp_tx_frame_count(uint8_t bus_index);

#ifdef __cplusplus
}
#endif

#endif // CAN_BSP_H
