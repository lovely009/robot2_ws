// FDCAN 板级收发接口（ARD-0007 §21/§22/§28：三路硬件 init/过滤器/IRQ/
// RX ring/TX 调度/低层统计归 lib/fdcan；不做协议解码、ROS、策略）。
//
// 数据路径：ISR 只从硬件 FIFO 取帧、盖 MCU 单调时戳、写入该总线 can_rx_ring
// （SPSC 无锁，不解码）；motor_rt_task 每服务节拍经 can_rx_ring_pop 批量弹出，
// TX 经每总线 can_tx_scheduler（P0/P1/P2）pump 后由 can_bsp_submit_tx 提交硬件。
//
// 本层不做：协议解码、ROS、策略裁决、第二个 RX 消费者（ARD §28.1）。

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


// 每总线硬件 RX FIFO0 的 MESSAGE_LOST 计数：FIFO0 只有 8 深，一旦丢帧
// **不可恢复**——这是协议层唯一会真丢数据的地方，必须能看见。
// 注意：必须留在 extern "C" 块**内**——放外面会变成 C++ 链接，
// 与 C 编译的定义对不上（undefined reference）。
uint32_t can_bsp_rx_hw_lost(uint8_t bus_index);

// 每总线**已提交给硬件**的发送帧数：在 HAL_FDCAN_AddMessageToTxFifoQ 成功处计。
// 这是"板子发了多少帧"的直接证据（此前只有失败计数，判断发送频率只能靠推断）。
uint32_t can_bsp_tx_frame_count(uint8_t bus_index);

#ifdef __cplusplus
}
#endif

#endif // CAN_BSP_H
