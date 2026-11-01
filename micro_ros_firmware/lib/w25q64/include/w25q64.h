// W25Q64 boot-only read driver. No program or erase operations.
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
#ifdef __cplusplus
}
#endif

#endif  // W25Q64_H
