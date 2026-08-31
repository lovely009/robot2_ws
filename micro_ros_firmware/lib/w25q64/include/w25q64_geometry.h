#ifndef W25Q64_GEOMETRY_H_
#define W25Q64_GEOMETRY_H_

#include <stdbool.h>
#include <stdint.h>

#define W25Q64_FLASH_SIZE 0x800000U
#define W25Q64_PAGE_SIZE 256U
#define W25Q64_SECTOR_SIZE 4096U

#ifdef __cplusplus
extern "C" {
#endif

bool w25q64_range_valid(uint32_t address, uint32_t size);
bool w25q64_page_program_valid(uint32_t address, uint32_t size);
bool w25q64_erase_address_valid(uint32_t address, uint32_t erase_size);

#ifdef __cplusplus
}
#endif

#endif  // W25Q64_GEOMETRY_H_
