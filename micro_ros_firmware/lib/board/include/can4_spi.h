// DM-MC-Board02 扩展板 H1，SPI3 由 motor_rt 任务独占。
#ifndef CAN4_SPI_H
#define CAN4_SPI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { CAN4_SPI_MAX_BYTES = 77, CAN4_SPI_DMA_MIN_BYTES = 32 };

bool can4_spi_init(void);
// 只有控制门禁之前的 RX 消息 RAM 读取可挂起任务。TX 提交期间保留调用方优先级。
bool can4_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t length, bool may_block);
// 由独占 SPI3 的 motor_rt 实现。一次可挂起的 DMA 事务对应一次通知。
void can4_spi_wait_dma(void);
void can4_spi_wake_from_isr(void);
bool can4_irq_asserted(void);
void can4_init_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif
