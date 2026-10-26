// W25Q64（OCTOSPI2）原始读写驱动 + geometry 纯逻辑（host 可测）。
// 唯一所有者是 storage 任务：上电即初始化，其余任务严禁直接访问本驱动。

#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>

#include "w25q64_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  W25Q64_OK = 0,
  W25Q64_ERROR_ARGUMENT = -1,
  W25Q64_ERROR_HAL = -2,
  W25Q64_ERROR_TIMEOUT = -3,
  W25Q64_ERROR_ID = -4,
  W25Q64_ERROR_STATE = -5,
} w25q64_status_t;

#define W25Q64_JEDEC_ID 0xEF4017U

w25q64_status_t w25q64_init(void);
w25q64_status_t w25q64_reset(void);
uint32_t w25q64_read_id(void);
w25q64_status_t w25q64_read(uint32_t address, uint8_t *data, uint32_t size);
w25q64_status_t w25q64_program_page(uint32_t address, const uint8_t *data,
    uint32_t size);
w25q64_status_t w25q64_write(uint32_t address, const uint8_t *data,
    uint32_t size);
w25q64_status_t w25q64_erase_sector(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif  // W25Q64_H
