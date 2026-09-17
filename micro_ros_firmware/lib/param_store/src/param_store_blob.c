#include "param_store_blob.h"

#include <string.h>

_Static_assert(sizeof(param_store_record_t) == 32U, "record 必须 32B");
_Static_assert(sizeof(param_store_blob_t) == PARAM_STORE_BLOB_SIZE,
               "blob 布局尺寸漂移");

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

static uint32_t blob_crc(const param_store_blob_t *blob) {
  // CRC 覆盖：header（crc 字段置零）+ 全部记录。
  uint8_t header_copy[PARAM_STORE_HEADER_SIZE];
  memcpy(header_copy, blob, PARAM_STORE_HEADER_SIZE);
  memset(header_copy + 16, 0, sizeof(uint32_t)); // crc32 字段偏移 = 16
  uint32_t crc = crc32_update(0U, header_copy, PARAM_STORE_HEADER_SIZE);
  return crc32_update(crc, (const uint8_t *)blob->records,
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

static param_store_record_t *find_record(param_store_blob_t *blob,
                                         uint16_t motor_id) {
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    if (blob->records[i].present != 0U &&
        blob->records[i].motor_id == motor_id) {
      return &blob->records[i];
    }
  }
  return NULL;
}

bool param_store_blob_set(param_store_blob_t *blob, uint16_t motor_id,
                          param_field_t field, float value) {
  if (blob == NULL || motor_id == 0U) {
    return false;
  }
  if (field == PARAM_FIELD_ENABLE || field >= PARAM_FIELD_COUNT) {
    return false;
  }
  param_store_record_t *rec = find_record(blob, motor_id);
  if (rec == NULL) {
    for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
      if (blob->records[i].present == 0U) {
        rec = &blob->records[i];
        rec->motor_id = motor_id;
        break;
      }
    }
    if (rec == NULL) {
      return false;
    }
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
      field >= PARAM_FIELD_COUNT) {
    return false;
  }
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_t *rec = &blob->records[i];
    if (rec->present == 0U || rec->motor_id != motor_id) {
      continue;
    }
    if ((rec->present & ((uint32_t)1U << (unsigned)field)) == 0U) {
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
  return false;
}

void param_store_blob_finalize(param_store_blob_t *blob, uint32_t seq) {
  if (blob == NULL) {
    return;
  }
  blob->seq = seq;
  blob->crc32 = 0U;
  blob->crc32 = blob_crc(blob);
}
