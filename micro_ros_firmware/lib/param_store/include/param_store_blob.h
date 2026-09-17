// W25Q64 持久化参数 blob（纯 C，无 HAL 依赖，host 可直测）。
//
// 单 blob = 32B 头 + 64×32B 记录 = 2080B，放在一个 4KB sector 内。
// 双 sector A/B 的选槽/seq 比较由 storage 任务负责，本模块只管布局与 CRC。
#ifndef PARAM_STORE_BLOB_H_
#define PARAM_STORE_BLOB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "param_name.h"

#ifdef __cplusplus
extern "C" {
#endif

// 小端序字节 'D','M','P',0x01。
#define PARAM_STORE_MAGIC 0x01504D44U
#define PARAM_STORE_VERSION 1U
#define PARAM_STORE_RECORD_COUNT 64U
#define PARAM_STORE_HEADER_SIZE 32U
#define PARAM_STORE_BLOB_SIZE \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * 32U)

typedef struct {
  uint32_t present; // 位 = 1u << param_field_t（bit0 enable 永不置位）
  uint16_t motor_id;
  uint16_t reserved;
  float kp;
  float kd;
  float pos_min;
  float pos_max;
  float vel_limit;
  float torque_limit;
} param_store_record_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t seq;
  uint32_t record_count;
  uint32_t crc32;
  uint32_t reserved[3]; // 头补齐 32B，保证记录 4B 对齐
  param_store_record_t records[PARAM_STORE_RECORD_COUNT];
} param_store_blob_t;

// 在给定存储上初始化为空 blob（magic/version/count 就位，seq=0，CRC 有效）。
// 缓冲不足返回 NULL。
param_store_blob_t *param_store_blob_reset(void *storage, size_t size);

// 校验 magic/version/header_size/record_count 与 CRC32。
bool param_store_blob_validate(const param_store_blob_t *blob);

// 写一个字段（稀疏占用记录槽，满 64 条不同电机返回 false）。
bool param_store_blob_set(param_store_blob_t *blob, uint16_t motor_id,
                          param_field_t field, float value);
// 读一个字段；未保存返回 false。
bool param_store_blob_get(const param_store_blob_t *blob, uint16_t motor_id,
                          param_field_t field, float *value);

// 写 seq 并重算 CRC（落盘前最后一步）。
void param_store_blob_finalize(param_store_blob_t *blob, uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif // PARAM_STORE_BLOB_H_
