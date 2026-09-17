// W25Q64 持久化参数 blob（纯 C，无 HAL 依赖，host 可直测）。
//
// 单 blob = 32B 头 + 64×48B 记录 = 3104B，放在一个 4KB sector 内（v2）。
// 双 sector A/B 的选槽/seq 比较由 storage 任务负责，本模块只管布局与 CRC。
//
// v2 相对 v1（32B 记录）增加设备端原值：设备参数以设备内为准，固件记录是
// 上电比对的基准，比对必须比原值——关节侧 SI 会与换算系数一起错，比对恒
// 通过。v1 blob 仍可读：param_store_blob_migrate_v1() 就地升级。
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
#define PARAM_STORE_VERSION 2U
#define PARAM_STORE_VERSION_LEGACY 1U
#define PARAM_STORE_RECORD_COUNT 64U
#define PARAM_STORE_HEADER_SIZE 32U
#define PARAM_STORE_RECORD_SIZE 48U
#define PARAM_STORE_LEGACY_RECORD_SIZE 32U
#define PARAM_STORE_BLOB_SIZE \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * PARAM_STORE_RECORD_SIZE)
#define PARAM_STORE_LEGACY_BLOB_SIZE \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * \
   PARAM_STORE_LEGACY_RECORD_SIZE)

typedef struct {
  uint32_t present; // 位 = 1u << param_field_t（bit0 enable 永不置位）
  uint16_t motor_id;
  // 设备端原值存在性：位 = 1u << param_device_slot_t。有位的槽位
  // device_raw 就是设备里存的值（写核验或回读得来），上电比对照它。
  uint8_t device_raw_present;
  uint8_t reserved;
  float kp;
  float kd;
  float pos_min;
  float pos_max;
  float vel_limit;
  float torque_limit;
  uint32_t device_raw[PARAM_DEVICE_SLOT_COUNT]; // cnt/cnt/RPM/mA
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

// 设备端原值：写/读单个槽位（写会把该槽位计入 device_raw_present）。
bool param_store_blob_set_device_raw(param_store_blob_t *blob, uint16_t motor_id,
                                     param_device_slot_t slot, uint32_t raw);
bool param_store_blob_get_device_raw(const param_store_blob_t *blob,
                                     uint16_t motor_id, param_device_slot_t slot,
                                     uint32_t *raw);
// 该电机记录里的设备端原值位图；无记录返回 0。
uint8_t param_store_blob_device_raw_present(const param_store_blob_t *blob,
                                            uint16_t motor_id);

// 旧版（v1，32B 记录、无设备端原值）blob → v2。校验 v1 头与 CRC，逐字段搬运，
// device_raw_present 保持 0（升级后首次上电不做设备参数比对，直到参数被重写）。
// 成功返回 out，失败返回 NULL（调用方按"无有效 blob"处理）。
param_store_blob_t *param_store_blob_migrate_v1(void *legacy_src, size_t src_size,
                                                void *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // PARAM_STORE_BLOB_H_
