#include <array>
#include <cassert>
#include <cstring>
#include "storage_boot.h"
#include "param_store_blob.h"
#include "w25q64.h"

static std::array<uint8_t, 0x4000> flash;
static bool available = true;
extern "C" w25q64_status_t w25q64_init(void) {
  return available ? W25Q64_OK : W25Q64_ERROR_ID;
}
extern "C" w25q64_status_t w25q64_read(uint32_t address, uint8_t *out, uint32_t size) {
  if (address > flash.size() || size > flash.size() - address) return W25Q64_ERROR_ARGUMENT;
  std::memcpy(out, flash.data() + address, size);
  return W25Q64_OK;
}
extern "C" w25q64_status_t w25q64_erase_sector(uint32_t address) {
  if (!available) return W25Q64_ERROR_ID;
  if ((address & 0xfffU) != 0U || address + 0x1000U > flash.size()) return W25Q64_ERROR_ARGUMENT;
  std::memset(flash.data() + address, 0xff, 0x1000U);
  return W25Q64_OK;
}
extern "C" w25q64_status_t w25q64_write(uint32_t address, const uint8_t *data, uint32_t size) {
  if (!available) return W25Q64_ERROR_ID;
  if (address > flash.size() || size > flash.size() - address) return W25Q64_ERROR_ARGUMENT;
  for (uint32_t i = 0; i < size; ++i) flash[address + i] &= data[i];
  return W25Q64_OK;
}
static void slot(uint32_t address, uint32_t sequence, uint32_t direction) {
  param_store_blob_t blob{};
  assert(param_store_blob_reset(&blob, sizeof(blob)));
  assert(param_store_blob_set_device_raw(&blob, 7, PARAM_DEVICE_SLOT_DIRECTION, direction));
  param_store_blob_finalize(&blob, sequence);
  std::memcpy(flash.data() + address, &blob, sizeof(blob));
}
static void expect_direction(uint32_t direction) {
  const auto before = flash;
  storage_boot_load();
  size_t count = 0;
  const auto *records = storage_boot_params(&count);
  assert(count == 1 && records[0].motor_id == 7);
  assert(records[0].device_raw[PARAM_DEVICE_SLOT_DIRECTION] == direction);
  assert(flash == before);
}
int main() {
  flash.fill(0xff);
  slot(0, 10, 0); slot(0x2000, 11, 1);
  expect_direction(1);
  flash[0x2000] ^= 1;
  expect_direction(0);
  flash.fill(0xff);
  slot(0, UINT32_MAX, 0); slot(0x2000, 1, 1);
  expect_direction(1);
  flash.fill(0xff);
  slot(0x1000, 3, 1);
  expect_direction(1);
  available = false;
  storage_boot_load();
  size_t count = 99;
  storage_boot_params(&count);
  assert(count == 0);

  // Test 37-motor W25 dual-slot save + boot load roundtrip
  available = true;
  flash.fill(0xff);
  storage_boot_load();
  std::array<motor_control_param_view_t, 37> views37{};
  for (uint16_t i = 0; i < 37; ++i) {
    views37[i].motor_id = static_cast<uint16_t>(i + 1U);
    views37[i].record_raw_present = (1U << PARAM_DEVICE_SLOT_DIRECTION) | (1U << PARAM_DEVICE_SLOT_VEL_LIMIT);
    views37[i].record_raw[PARAM_DEVICE_SLOT_DIRECTION] = i % 2U;
    views37[i].record_raw[PARAM_DEVICE_SLOT_VEL_LIMIT] = 1000U + i;
  }
  assert(storage_save_records(views37.data(), views37.size()));
  storage_boot_load();
  size_t loaded_count = 0;
  const auto *loaded = storage_boot_params(&loaded_count);
  assert(loaded_count == 37);
  assert(loaded[0].motor_id == 1 && loaded[36].motor_id == 37);
  assert(loaded[36].device_raw[PARAM_DEVICE_SLOT_VEL_LIMIT] == 1036U);
}
