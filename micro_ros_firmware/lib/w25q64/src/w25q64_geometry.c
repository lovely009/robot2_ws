#include "w25q64_geometry.h"

bool w25q64_range_valid(uint32_t address, uint32_t size) {
  return address <= W25Q64_FLASH_SIZE && size <= W25Q64_FLASH_SIZE - address;
}

bool w25q64_page_program_valid(uint32_t address, uint32_t size) {
  if (size == 0U || size > W25Q64_PAGE_SIZE ||
      !w25q64_range_valid(address, size)) {
    return false;
  }

  return address / W25Q64_PAGE_SIZE ==
         (address + size - 1U) / W25Q64_PAGE_SIZE;
}

bool w25q64_erase_address_valid(uint32_t address, uint32_t erase_size) {
  if (erase_size != W25Q64_SECTOR_SIZE && erase_size != 32768U &&
      erase_size != 65536U) {
    return false;
  }

  return address % erase_size == 0U &&
         w25q64_range_valid(address, erase_size);
}
