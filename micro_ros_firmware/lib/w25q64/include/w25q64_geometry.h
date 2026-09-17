// W25Q64 几何常量（纯值，host 可测）：全部以字节为单位。
// Flash 8MB；页编程 256B；4KB 扇区是最小擦除单位（storage A/B 各占一扇区）。
#ifndef W25Q64_GEOMETRY_H
#define W25Q64_GEOMETRY_H

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

#endif  // W25Q64_GEOMETRY_H
