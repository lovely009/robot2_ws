#include "param_store_blob.h"

#include <string.h>

_Static_assert(sizeof(param_store_record_t) == PARAM_STORE_RECORD_SIZE,
    "record 必须 120B（v3）");
_Static_assert(sizeof(param_store_blob_t) == PARAM_STORE_BLOB_SIZE,
    "blob 布局尺寸漂移（v3 = 32 + 64*120）");

// CRC32/IEEE 802.3（与 zlib 同多项式；软件按位，2KB 只在 boot/save 各算一次）。
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
    }
  }
  return ~crc;
}

// CRC 覆盖集：header（crc 字段置零）+ 全部记录。v1/v2/v3 三种记录布局共用同一
// 定义——加/改头字段时只有这里要动，否则某个布局会静默校验失败退回默认值。
static uint32_t crc_over_blob(const uint8_t *header, const uint8_t *records,
    size_t records_bytes) {
  uint8_t header_copy[PARAM_STORE_HEADER_SIZE];
  memcpy(header_copy, header, PARAM_STORE_HEADER_SIZE);
  memset(header_copy + 16, 0, sizeof(uint32_t)); // crc32 字段偏移 = 16
  uint32_t crc = crc32_update(0U, header_copy, PARAM_STORE_HEADER_SIZE);
  return crc32_update(crc, records, records_bytes);
}

static uint32_t blob_crc(const param_store_blob_t *blob) {
  return crc_over_blob((const uint8_t *)blob, (const uint8_t *)blob->records,
      sizeof(blob->records));
}

param_store_blob_t *param_store_blob_reset(void *storage, size_t size) {
  if (storage == NULL || size < PARAM_STORE_BLOB_SIZE) {
    return NULL;
  }
  param_store_blob_t *blob = (param_store_blob_t *)storage;
  memset(blob, 0, PARAM_STORE_BLOB_SIZE);
  blob->magic = PARAM_STORE_MAGIC;
  blob->version = PARAM_STORE_VERSION;
  blob->header_size = PARAM_STORE_HEADER_SIZE;
  blob->record_count = PARAM_STORE_RECORD_COUNT;
  param_store_blob_finalize(blob, 0U);
  return blob;
}

bool param_store_blob_validate(const param_store_blob_t *blob) {
  if (blob == NULL || blob->magic != PARAM_STORE_MAGIC ||
      blob->version != PARAM_STORE_VERSION ||
      blob->header_size != PARAM_STORE_HEADER_SIZE ||
      blob->record_count != PARAM_STORE_RECORD_COUNT) {
    return false;
  }
  return blob->crc32 == blob_crc(blob);
}

// 已占用记录 = 有字段位或有设备端原值位；空槽两者皆空（motor_id 可能有
// 残留，判定不看它——否则"先写原值后写字段"会占出两条记录）。
static bool record_occupied(const param_store_record_t *rec) {
  return rec->present != 0U || rec->device_raw_present != 0U;
}

// 取记录；没有就占用第一个空槽（空槽 = 无字段位且无设备端原值位，motor_id
// 可能有残留故不入判定——否则"先写原值后写字段"会占出两条记录）。表满返回 NULL。
static param_store_record_t *find_or_alloc_record(param_store_blob_t *blob,
    uint16_t motor_id) {
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    if (record_occupied(&blob->records[i]) &&
        blob->records[i].motor_id == motor_id) {
      return &blob->records[i];
    }
  }
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    if (!record_occupied(&blob->records[i])) {
      blob->records[i].motor_id = motor_id;
      return &blob->records[i];
    }
  }
  return NULL;
}

// 只读查找（占用判定同上）。
static const param_store_record_t *find_record_const(
    const param_store_blob_t *blob, uint16_t motor_id) {
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_t *rec = &blob->records[i];
    if (record_occupied(rec) && rec->motor_id == motor_id) {
      return rec;
    }
  }
  return NULL;
}

// 不落在 float 字段里的参数：`enable` 是策略位（不持久化），`limit_protect`
// 的值持久化在 device_raw[槽 4]（见 param_store_blob_set_device_raw）——
// 这两个走本组 get/set 只会拿到 false，不会静默丢值。
bool param_store_blob_set(param_store_blob_t *blob, uint16_t motor_id,
    param_field_t field, float value) {
  if (blob == NULL || motor_id == 0U) {
    return false;
  }
  if (field == PARAM_FIELD_ENABLE || field == PARAM_FIELD_LIMIT_PROTECT ||
      field >= PARAM_FIELD_COUNT) {
    return false;
  }
  param_store_record_t *rec = find_or_alloc_record(blob, motor_id);
  if (rec == NULL) {
    return false; // 64 条记录全占用
  }
  switch (field) {
  case PARAM_FIELD_KP: rec->kp = value; break;
  case PARAM_FIELD_KD: rec->kd = value; break;
  case PARAM_FIELD_POS_MIN: rec->pos_min = value; break;
  case PARAM_FIELD_POS_MAX: rec->pos_max = value; break;
  case PARAM_FIELD_VEL_LIMIT: rec->vel_limit = value; break;
  case PARAM_FIELD_TORQUE_LIMIT: rec->torque_limit = value; break;
  default: return false;
  }
  rec->present |= (uint32_t)1U << (unsigned)field;
  return true;
}

bool param_store_blob_get(const param_store_blob_t *blob, uint16_t motor_id,
    param_field_t field, float *value) {
  if (blob == NULL || value == NULL || field == PARAM_FIELD_ENABLE ||
      field == PARAM_FIELD_LIMIT_PROTECT || field >= PARAM_FIELD_COUNT) {
    return false;
  }
  const param_store_record_t *rec = find_record_const(blob, motor_id);
  if (rec == NULL || (rec->present & ((uint32_t)1U << (unsigned)field)) == 0U) {
    return false;
  }
  switch (field) {
  case PARAM_FIELD_KP: *value = rec->kp; return true;
  case PARAM_FIELD_KD: *value = rec->kd; return true;
  case PARAM_FIELD_POS_MIN: *value = rec->pos_min; return true;
  case PARAM_FIELD_POS_MAX: *value = rec->pos_max; return true;
  case PARAM_FIELD_VEL_LIMIT: *value = rec->vel_limit; return true;
  case PARAM_FIELD_TORQUE_LIMIT: *value = rec->torque_limit; return true;
  default: return false;
  }
}

void param_store_blob_finalize(param_store_blob_t *blob, uint32_t seq) {
  if (blob == NULL) {
    return;
  }
  blob->seq = seq;
  blob->crc32 = 0U;
  blob->crc32 = blob_crc(blob);
}

bool param_store_blob_set_device_raw(param_store_blob_t *blob, uint16_t motor_id,
    param_device_slot_t slot, uint32_t raw) {
  if (blob == NULL || motor_id == 0U || slot >= PARAM_DEVICE_SLOT_COUNT) {
    return false;
  }
  param_store_record_t *rec = find_or_alloc_record(blob, motor_id);
  if (rec == NULL) {
    return false; // 64 条记录全占用
  }
  rec->device_raw[(unsigned)slot] = raw;
  rec->device_raw_present |= (uint8_t)(1U << (unsigned)slot);
  return true;
}

bool param_store_blob_get_device_raw(const param_store_blob_t *blob,
    uint16_t motor_id, param_device_slot_t slot,
    uint32_t *raw) {
  if (blob == NULL || raw == NULL || slot >= PARAM_DEVICE_SLOT_COUNT) {
    return false;
  }
  const param_store_record_t *rec = find_record_const(blob, motor_id);
  if (rec == NULL ||
      (rec->device_raw_present & (uint8_t)(1U << (unsigned)slot)) == 0U) {
    return false;
  }
  *raw = rec->device_raw[(unsigned)slot];
  return true;
}

uint8_t param_store_blob_device_raw_present(const param_store_blob_t *blob,
    uint16_t motor_id) {
  if (blob == NULL) {
    return 0U;
  }
  const param_store_record_t *rec = find_record_const(blob, motor_id);
  return rec != NULL ? rec->device_raw_present : 0U;
}

// v1 记录（32B，无设备端原值）：仅供读取迁移，不参与写入。
typedef struct {
  uint32_t present;
  uint16_t motor_id;
  uint16_t reserved;
  float kp;
  float kd;
  float pos_min;
  float pos_max;
  float vel_limit;
  float torque_limit;
} param_store_record_v1_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t seq;
  uint32_t record_count;
  uint32_t crc32;
  uint32_t reserved[3];
  param_store_record_v1_t records[PARAM_STORE_RECORD_COUNT];
} param_store_blob_v1_t;

_Static_assert(sizeof(param_store_record_v1_t) == PARAM_STORE_LEGACY_RECORD_SIZE,
    "v1 record 必须 32B");
_Static_assert(sizeof(param_store_blob_v1_t) == PARAM_STORE_LEGACY_BLOB_SIZE,
    "v1 blob 尺寸漂移");

static uint32_t blob_v1_crc(const param_store_blob_v1_t *blob) {
  return crc_over_blob((const uint8_t *)blob, (const uint8_t *)blob->records,
      sizeof(blob->records));
}

param_store_blob_t *param_store_blob_migrate_v1(void *legacy_src, size_t src_size,
    void *out, size_t out_size) {
  if (legacy_src == NULL || out == NULL ||
      src_size < PARAM_STORE_LEGACY_BLOB_SIZE ||
      out_size < PARAM_STORE_BLOB_SIZE) {
    return NULL;
  }
  const param_store_blob_v1_t *src = (const param_store_blob_v1_t *)legacy_src;
  if (src->magic != PARAM_STORE_MAGIC ||
      src->version != PARAM_STORE_VERSION_LEGACY ||
      src->header_size != PARAM_STORE_HEADER_SIZE ||
      src->record_count != PARAM_STORE_RECORD_COUNT ||
      src->crc32 != blob_v1_crc(src)) {
    return NULL;
  }
  param_store_blob_t *dst = param_store_blob_reset(out, out_size);
  if (dst == NULL) {
    return NULL;
  }
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_v1_t *from = &src->records[i];
    if (from->present == 0U) {
      continue;
    }
    param_store_record_t *to = &dst->records[i];
    to->present = from->present;
    to->motor_id = from->motor_id;
    to->device_raw_present = 0U; // v1 没有设备端原值：升级后不做比对
    to->kp = from->kp;
    to->kd = from->kd;
    to->pos_min = from->pos_min;
    to->pos_max = from->pos_max;
    to->vel_limit = from->vel_limit;
    to->torque_limit = from->torque_limit;
  }
  param_store_blob_finalize(dst, src->seq); // seq 沿用，活动槽选择不受影响
  return dst;
}

// v2 记录（48B，device_raw 4 槽、无型号身份/MIT 标度）：仅供读取迁移，不参与写入。
typedef struct {
  uint32_t present;
  uint16_t motor_id;
  uint8_t device_raw_present;
  uint8_t reserved0;
  float kp;
  float kd;
  float pos_min;
  float pos_max;
  float vel_limit;
  float torque_limit;
  uint32_t device_raw[4];
} param_store_record_v2_t;

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t seq;
  uint32_t record_count;
  uint32_t crc32;
  uint32_t reserved[3];
  param_store_record_v2_t records[PARAM_STORE_RECORD_COUNT];
} param_store_blob_v2_t;

_Static_assert(sizeof(param_store_record_v2_t) == PARAM_STORE_RECORD_SIZE_V2,
    "v2 record 必须 48B");
_Static_assert(sizeof(param_store_blob_v2_t) == PARAM_STORE_BLOB_SIZE_V2,
    "v2 blob 尺寸漂移");

static uint32_t blob_v2_crc(const param_store_blob_v2_t *blob) {
  return crc_over_blob((const uint8_t *)blob, (const uint8_t *)blob->records,
      sizeof(blob->records));
}

param_store_blob_t *param_store_blob_migrate_v2(void *v2_src, size_t src_size,
    void *out, size_t out_size) {
  if (v2_src == NULL || out == NULL || src_size < PARAM_STORE_BLOB_SIZE_V2 ||
      out_size < PARAM_STORE_BLOB_SIZE) {
    return NULL;
  }
  const param_store_blob_v2_t *src = (const param_store_blob_v2_t *)v2_src;
  if (src->magic != PARAM_STORE_MAGIC ||
      src->version != PARAM_STORE_VERSION_V2 ||
      src->header_size != PARAM_STORE_HEADER_SIZE ||
      src->record_count != PARAM_STORE_RECORD_COUNT ||
      src->crc32 != blob_v2_crc(src)) {
    return NULL; // 头或 CRC 不合格：不动 out，调用方按"无有效 blob"处理
  }
  param_store_blob_t *dst = param_store_blob_reset(out, out_size);
  if (dst == NULL) {
    return NULL;
  }
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_v2_t *from = &src->records[i];
    if (from->present == 0U && from->device_raw_present == 0U) {
      continue; // 空槽（与 record_occupied 同判定）
    }
    param_store_record_t *to = &dst->records[i];
    to->present = from->present;
    to->motor_id = from->motor_id;
    // 只搬低 4 位：v2 合法的写路径 slot >= 4 即拒，位 4/5 恒 0；掩码是为了
    // 对"CRC 合法但高位被置位"的畸形 blob 也强制"未核验 = 位 0"这一约定，
    // 而不是假设它成立（否则安全/方向槽会被谎报为"已核验且值为 0"）。
    to->device_raw_present = (uint8_t)(from->device_raw_present & 0x0FU);
    to->kp = from->kp;
    to->kd = from->kd;
    to->pos_min = from->pos_min;
    to->pos_max = from->pos_max;
    to->vel_limit = from->vel_limit;
    to->torque_limit = from->torque_limit;
    for (uint32_t s = 0; s < 4U; ++s) {
      to->device_raw[s] = from->device_raw[s];
    }
    // device_raw[4..7]、model_name、identity_hash、mit_*、profile_version、
    // reserved 均为 0（reset 已清零）——语义是"未知/未核验"。
  }
  param_store_blob_finalize(dst, src->seq); // seq 沿用，活动槽选择不受影响
  return dst;
}
