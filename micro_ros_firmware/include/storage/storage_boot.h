#ifndef STORAGE_BOOT_H
#define STORAGE_BOOT_H
#include <stdbool.h>
#include <stddef.h>
#include "motor_control.h"
#ifdef __cplusplus
extern "C" {
#endif
void storage_boot_load(void);
const motor_control_stored_params_t *storage_boot_params(size_t *count);
bool storage_save_records(const motor_control_param_view_t *views, size_t count);
#ifdef __cplusplus
}
#endif
#endif
