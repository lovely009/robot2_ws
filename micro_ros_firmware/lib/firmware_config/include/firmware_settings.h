#ifndef FIRMWARE_SETTINGS_H
#define FIRMWARE_SETTINGS_H
#include <stdbool.h>
#include <stdint.h>
#include "firmware_config.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint16_t agent_ping_hz;
  uint8_t motion_hz, status_hz, imu_hz, temperature_hz;
  uint16_t motor_command_hz;
} firmware_settings_t;
#define FIRMWARE_SETTINGS_DEFAULT {100U, 50U, 10U, 100U, 2U, FIRMWARE_MOTOR_COMMAND_HZ}
#define FIRMWARE_SETTINGS_COUNT 6U
extern const char *const firmware_setting_names[FIRMWARE_SETTINGS_COUNT];
bool firmware_settings_set(firmware_settings_t *settings, unsigned field, int64_t value);
int64_t firmware_settings_get(const firmware_settings_t *settings, unsigned field);
bool firmware_settings_equal(const firmware_settings_t *a, const firmware_settings_t *b);
void firmware_settings_pack(const firmware_settings_t *settings, uint32_t words[3]);
bool firmware_settings_unpack(const uint32_t words[3], firmware_settings_t *settings);
#ifdef __cplusplus
}
#endif
#endif
