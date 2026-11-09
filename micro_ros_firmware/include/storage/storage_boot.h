#ifndef STORAGE_BOOT_H
#define STORAGE_BOOT_H
#include <stddef.h>
#include "motor_control.h"
#ifdef __cplusplus
extern "C" {
#endif
void storage_boot_load(void);
const motor_control_stored_params_t *storage_boot_params(size_t *count);
#ifdef __cplusplus
}
#endif
#endif
