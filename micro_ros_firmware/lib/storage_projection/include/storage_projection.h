// 旧 W25 电机记录的只读启动投影，无硬件依赖。
#ifndef STORAGE_PROJECTION_H
#define STORAGE_PROJECTION_H

#include <stddef.h>

#include "motor_control.h"
#include "param_store_blob.h"

#ifdef __cplusplus
extern "C" {
#endif

// blob → boot 记录（写入 out，最多 capacity 条，返回实际条数）。
// 字段存在性来自 present/device_raw_present 位图。
// blob 为 NULL，或 capacity>0 且 out 为 NULL 时返回 0。
size_t storage_projection_blob_to_boot(
  const param_store_blob_t *blob, motor_control_stored_params_t *out,
  size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
