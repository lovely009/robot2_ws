#include <cassert>
#include <cstdint>
#include <cstring>
#include "firmware_settings.h"
#include "param_store_blob.h"

int main() {
  const firmware_settings_t defaults = FIRMWARE_SETTINGS_DEFAULT;
  auto settings = defaults;
  const int64_t allowed[][7] = {{10, 20, 50, 100}, {20, 25, 50},
    {10, 20, 25, 50}, {10, 20, 25, 50, 100}, {1, 2}, {50, 100, 125, 200, 250, 500, 1000}};
  for (unsigned field = 0; field < FIRMWARE_SETTINGS_COUNT; ++field) {
    for (auto value : allowed[field]) {
      if (!value) continue;
      assert(firmware_settings_set(&settings, field, value));
      assert(firmware_settings_get(&settings, field) == value);
      assert(1000 % value == 0);
      if (field < 5) assert((1000 / value) % 10 == 0);
    }
    const int64_t invalid[] = {-1, 0, 3, 256, 65536, INT64_MAX};
    for (int64_t value : invalid) {
      const auto before = settings;
      assert(!firmware_settings_set(&settings, field, value));
      assert(firmware_settings_equal(&before, &settings));
    }
  }
  assert(!firmware_settings_set(&settings, FIRMWARE_SETTINGS_COUNT, 1));
  const int64_t invalid_control[] = {1, 49, 999, 1001, 2000};
  for (const int64_t value : invalid_control)
    assert(!firmware_settings_set(&settings, 5, value));
  assert(!firmware_settings_set(nullptr, 0, 10));
  uint32_t words[3]{};
  assert(!firmware_settings_unpack(words, &settings));
  assert(firmware_settings_set(&settings, 1, 25));
  assert(firmware_settings_set(&settings, 5, 1000));
  firmware_settings_pack(&settings, words);
  auto restored = defaults;
  assert(firmware_settings_unpack(words, &restored));
  assert(firmware_settings_equal(&settings, &restored));
  assert(restored.motor_command_hz == 1000);
  const auto before = restored;
  words[2] |= 0xff000000U;
  assert(!firmware_settings_unpack(words, &restored));
  assert(firmware_settings_equal(&before, &restored));
  firmware_settings_pack(&settings, words);
  words[1] = 20U;
  assert(!firmware_settings_unpack(words, &restored));

  // SYS1 迁移保留原频率，新增控制频率恢复默认 200Hz。
  words[0] = UINT32_C(0x31535953);
  assert(firmware_settings_unpack(words, &restored));
  assert(restored.agent_ping_hz == 20 && restored.motion_hz == 25);
  assert(restored.motor_command_hz == FIRMWARE_MOTOR_COMMAND_HZ);
  words[1] |= 0x10000U;
  assert(!firmware_settings_unpack(words, &restored));
  words[0] = 0;
  assert(!firmware_settings_unpack(words, &restored));

  alignas(8) static uint8_t bytes[PARAM_STORE_BLOB_SIZE];
  auto *blob = param_store_blob_reset(bytes, sizeof(bytes));
  assert(blob && param_store_blob_set(blob, 1, PARAM_FIELD_KP, 3));
  firmware_settings_pack(&settings, blob->reserved);
  param_store_blob_finalize(blob, 7);
  assert(param_store_blob_validate(blob));
  assert(firmware_settings_unpack(blob->reserved, &restored));
  float kp;
  assert(param_store_blob_get(blob, 1, PARAM_FIELD_KP, &kp) && kp == 3);
  blob->reserved[2] ^= 1U;
  assert(!param_store_blob_validate(blob));
  assert(param_store_blob_reset(bytes, sizeof(bytes)));
  restored = defaults;
  assert(!firmware_settings_unpack(blob->reserved, &restored));
  assert(firmware_settings_equal(&defaults, &restored));
}
