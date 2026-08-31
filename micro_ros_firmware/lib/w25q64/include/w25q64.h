#ifndef W25Q64_H_
#define W25Q64_H_

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
w25q64_status_t w25q64_erase_block_32k(uint32_t address);
w25q64_status_t w25q64_erase_block_64k(uint32_t address);
w25q64_status_t w25q64_chip_erase(void);
w25q64_status_t w25q64_memory_mapped_mode(void);

#ifdef __cplusplus
}
#endif

#endif  // W25Q64_H_
