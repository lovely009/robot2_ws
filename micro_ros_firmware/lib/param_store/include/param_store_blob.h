// W25Q64 持久化参数 blob（纯 C，无 HAL 依赖，host 可直测）。
// 单 blob = 32B 头 + 64×120B 记录 = 7712B；每槽占 2 个 4KB 扇区（槽 A 0x0000–0x1FFF、槽 B
// 0x2000–0x3FFF）——选槽与 seq 比较归 storage 任务，本模块只管布局与 CRC。
// v3 使用 8 个设备原值槽（现用 6）；身份与 MIT 标度字段预留，写入尚未实现。
// 支持校验并迁移 v1/v2 布局。
#ifndef PARAM_STORE_BLOB_H
#define PARAM_STORE_BLOB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "param_field.h"

#ifdef __cplusplus
extern "C" {
#endif

// 小端序字节 'D','M','P',0x01。
#define PARAM_STORE_MAGIC 0x01504D44U
#define PARAM_STORE_VERSION 3U
#define PARAM_STORE_VERSION_V2 2U
#define PARAM_STORE_VERSION_LEGACY 1U
#define PARAM_STORE_RECORD_COUNT 64U
#define PARAM_STORE_HEADER_SIZE 32U
#define PARAM_STORE_RECORD_SIZE 120U
#define PARAM_STORE_RECORD_SIZE_V2 48U
#define PARAM_STORE_LEGACY_RECORD_SIZE 32U
#define PARAM_STORE_BLOB_SIZE \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * PARAM_STORE_RECORD_SIZE)
#define PARAM_STORE_BLOB_SIZE_V2 \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * PARAM_STORE_RECORD_SIZE_V2)
#define PARAM_STORE_LEGACY_BLOB_SIZE \
  (PARAM_STORE_HEADER_SIZE + PARAM_STORE_RECORD_COUNT * \
  PARAM_STORE_LEGACY_RECORD_SIZE)

typedef struct {
  uint32_t present; // 位 = 1u << param_field_t（bit0 enable 永不置位）
  uint16_t motor_id;
  // 设备端原值存在性：位 = 1u << param_device_slot_t。有位的槽位
  // device_raw 就是设备里存的值（写核验或回读得来），上电比对照它。
  uint8_t device_raw_present;
  uint8_t reserved0;
  float kp;
  float kd;
  float pos_min;
  float pos_max;
  float vel_limit;
  float torque_limit;
  // 原值槽现用 6 个，另留 2 个；身份与来源链预留，当前写入路径保持全零。
  uint32_t device_raw[8]; // cnt/cnt/RPM/mA/0x2650:00/0x2541
  char model_name[16];     // 型号目录名，NUL 结尾；空 = 未绑定
  uint32_t identity_hash;  // 设备 0x1008 自报串的 CRC32；0 = 未核验
  // MIT 全量程预留，当前写入路径未实现；0 表示未知。
  float mit_pos_limit_rad;
  float mit_vel_limit_rad_s;
  float mit_torque_limit_nm;
  uint8_t profile_version; // 型号目录版本（预留，未实现）
  uint8_t reserved[23];    // 后续项：零点 epoch / 跨零 / 运行时长 / MIT 实测标度
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

// v1（32B 记录、无设备原值）迁移至当前版本；校验头与 CRC，原值存在位保持 0。
// 成功返回 out，失败返回 NULL。
param_store_blob_t *param_store_blob_migrate_v1(void *legacy_src, size_t src_size,
    void *out, size_t out_size);

// v2（48B 记录、device_raw 4 槽）→ v3。校验 v2 头与 CRC，逐字段搬运；
// device_raw[4..7] 清零（新增的安全/方向槽为"未核验"，与 v1→v2 时
// device_raw_present = 0 同口径）；型号身份与 MIT 标度一律写 0 = 未知。
// 成功返回 out，失败返回 NULL（调用方按"无有效 blob"处理）。
param_store_blob_t *param_store_blob_migrate_v2(void *v2_src, size_t src_size,
    void *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // PARAM_STORE_BLOB_H
