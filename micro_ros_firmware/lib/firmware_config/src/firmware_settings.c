#include "firmware_settings.h"
#include <stddef.h>

const char *const firmware_setting_names[FIRMWARE_SETTINGS_COUNT] = {
  "system.agent_ping_hz", "system.motion_report_hz",
  "system.status_report_hz", "system.imu_report_hz", "system.temperature_report_hz",
  "system.motor_command_hz"};
bool firmware_settings_set(firmware_settings_t *s, unsigned field, int64_t value) {
  if (!s) return false;
  switch (field) {
  case 0:
    if (value != 10 && value != 20 && value != 50 && value != 100) return false;
    s->agent_ping_hz = (uint16_t)value; break;
  case 1:
    if (value != 20 && value != 25 && value != 50) return false;
    s->motion_hz = (uint8_t)value; break;
  case 2:
    if (value != 10 && value != 20 && value != 25 && value != 50) return false;
    s->status_hz = (uint8_t)value; break;
  case 3:
    if (value != 10 && value != 20 && value != 25 && value != 50 && value != 100) return false;
    s->imu_hz = (uint8_t)value; break;
  case 4:
    if (value != 1 && value != 2) return false;
    s->temperature_hz = (uint8_t)value; break;
  case 5:
    if (value < 50 || value > 1000 || 1000 % value != 0) return false;
    s->motor_command_hz = (uint16_t)value; break;
  default: return false;
  }
  return true;
}
int64_t firmware_settings_get(const firmware_settings_t *s, unsigned field) {
  switch (field) {
  case 0: return s->agent_ping_hz;
  case 1: return s->motion_hz;
  case 2: return s->status_hz;
  case 3: return s->imu_hz;
  case 4: return s->temperature_hz;
  case 5: return s->motor_command_hz;
  default: return -1;
  }
}
bool firmware_settings_equal(const firmware_settings_t *a, const firmware_settings_t *b) {
  for (unsigned i = 0; i < FIRMWARE_SETTINGS_COUNT; ++i)
    if (firmware_settings_get(a, i) != firmware_settings_get(b, i)) return false;
  return true;
}
void firmware_settings_pack(const firmware_settings_t *s, uint32_t words[3]) {
  words[0] = UINT32_C(0x32535953); // SYS2：沿用 v3 头部预留区，仍受整 blob CRC 保护。
  words[1] = s->agent_ping_hz | ((uint32_t)s->motor_command_hz << 16);
  words[2] = (uint32_t)s->motion_hz | ((uint32_t)s->status_hz << 8) |
      ((uint32_t)s->imu_hz << 16) | ((uint32_t)s->temperature_hz << 24);
}
bool firmware_settings_unpack(const uint32_t words[3], firmware_settings_t *s) {
  if (!words || !s) return false;
  const bool legacy = words[0] == UINT32_C(0x31535953);
  if (legacy ? words[1] > UINT16_MAX : words[0] != UINT32_C(0x32535953)) return false;
  firmware_settings_t candidate = FIRMWARE_SETTINGS_DEFAULT;
  const int64_t values[] = {words[1] & 0xffffU,
    words[2] & 0xffU, (words[2] >> 8) & 0xffU, (words[2] >> 16) & 0xffU, words[2] >> 24,
    legacy ? FIRMWARE_MOTOR_COMMAND_HZ : words[1] >> 16};
  for (unsigned i = 0; i < FIRMWARE_SETTINGS_COUNT; ++i)
    if (!firmware_settings_set(&candidate, i, values[i])) return false;
  *s = candidate;
  return true;
}
