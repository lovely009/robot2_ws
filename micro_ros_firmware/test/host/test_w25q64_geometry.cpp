#include "w25q64_geometry.h"

#include <cassert>

int main() {
  assert(w25q64_range_valid(0U, 0U));
  assert(w25q64_range_valid(W25Q64_FLASH_SIZE - 1U, 1U));
  assert(!w25q64_range_valid(W25Q64_FLASH_SIZE, 1U));
  assert(!w25q64_range_valid(W25Q64_FLASH_SIZE - 1U, 2U));

  assert(w25q64_page_program_valid(0U, W25Q64_PAGE_SIZE));
  assert(w25q64_page_program_valid(255U, 1U));
  assert(!w25q64_page_program_valid(255U, 2U));
  assert(!w25q64_page_program_valid(0U, W25Q64_PAGE_SIZE + 1U));

  assert(w25q64_erase_address_valid(0U, W25Q64_SECTOR_SIZE));
  assert(w25q64_erase_address_valid(W25Q64_SECTOR_SIZE,
      W25Q64_SECTOR_SIZE));
  assert(!w25q64_erase_address_valid(1U, W25Q64_SECTOR_SIZE));
  assert(!w25q64_erase_address_valid(W25Q64_FLASH_SIZE,
      W25Q64_SECTOR_SIZE));
  assert(w25q64_erase_address_valid(0U, 32768U));
  assert(w25q64_erase_address_valid(0U, 65536U));
  assert(!w25q64_erase_address_valid(0U, 8192U));
  return 0;
}
