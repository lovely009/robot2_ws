// W25Q64 boot read & runtime cache sync (dual-slot A/B, 2 sectors per slot).
#include "storage_boot.h"
#include "storage_projection.h"
#include "param_store_blob.h"
#include "w25q64.h"

#include <math.h>

static constexpr uint32_t kSlotBase[2] = {0U, 0x2000U};
static constexpr uint32_t kLegacySlotBBase = 0x1000U;
static uint32_t slot_base(uint8_t slot) { return kSlotBase[slot & 1U]; }
static motor_control_stored_params_t g_boot_records[PARAM_STORE_RECORD_COUNT];
static size_t g_boot_count;
static uint8_t g_active_slot = 0U;
static uint32_t g_active_seq = 0U;
static bool g_storage_ready = false;
alignas(8) static uint8_t g_blob_scratch[PARAM_STORE_BLOB_SIZE];

static bool read_blob_at(uint32_t addr, uint8_t *buf) {
  if (w25q64_read(addr, buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    return false;
  }
  if (param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf))) {
    return true;
  }
  alignas(8) static uint8_t v2_buf[PARAM_STORE_BLOB_SIZE_V2];
  if (w25q64_read(addr, v2_buf, sizeof(v2_buf)) == W25Q64_OK &&
      param_store_blob_migrate_v2(v2_buf, sizeof(v2_buf), buf,
      PARAM_STORE_BLOB_SIZE) != nullptr &&
      param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf))) {
    return true;
  }
  alignas(8) static uint8_t legacy_buf[PARAM_STORE_LEGACY_BLOB_SIZE];
  if (w25q64_read(addr, legacy_buf, sizeof(legacy_buf)) != W25Q64_OK) {
    return false;
  }
  return param_store_blob_migrate_v1(legacy_buf, sizeof(legacy_buf), buf,
      PARAM_STORE_BLOB_SIZE) != nullptr &&
         param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf));
}

static bool read_slot(uint8_t slot, uint8_t *buf) {
  return read_blob_at(slot_base(slot), buf);
}

static void load_boot_params() {
  uint8_t *scratch = g_blob_scratch;
  const bool ok_a = read_slot(0U, scratch);
  const uint32_t seq_a =
      ok_a ? reinterpret_cast<const param_store_blob_t *>(scratch)->seq : 0U;
  const bool ok_b = read_slot(1U, scratch);
  const uint32_t seq_b =
      ok_b ? reinterpret_cast<const param_store_blob_t *>(scratch)->seq : 0U;

  bool use_b;
  if (ok_a && ok_b) {
    use_b = (int32_t)(seq_b - seq_a) > 0;
  } else if (ok_b) {
    use_b = true;
  } else if (ok_a) {
    use_b = false;
  } else {
    if (!read_blob_at(kLegacySlotBBase, scratch)) {
      return;
    }
    const param_store_blob_t *adopted =
        reinterpret_cast<const param_store_blob_t *>(scratch);
    g_active_slot = 0U;
    g_active_seq = adopted->seq;
    g_boot_count = storage_projection_blob_to_boot(
        adopted, g_boot_records, PARAM_STORE_RECORD_COUNT);
    return;
  }
  if (!use_b && !read_slot(0U, scratch)) {
    if (!read_slot(1U, scratch)) {
      return;
    }
    use_b = true;
  }
  const param_store_blob_t *chosen =
      reinterpret_cast<const param_store_blob_t *>(scratch);
  g_active_slot = use_b ? 1U : 0U;
  g_active_seq = chosen->seq;
  g_boot_count = storage_projection_blob_to_boot(chosen, g_boot_records,
      PARAM_STORE_RECORD_COUNT);
}

void storage_boot_load(void) {
  g_boot_count = 0;
  g_active_slot = 0U;
  g_active_seq = 0U;
  g_storage_ready = (w25q64_init() == W25Q64_OK);
  if (g_storage_ready) load_boot_params();
}

const motor_control_stored_params_t *storage_boot_params(size_t *count) {
  if (count) *count = g_boot_count;
  return g_boot_records;
}

bool storage_save_records(const motor_control_param_view_t *views, size_t count) {
  if (!g_storage_ready || (views == nullptr && count > 0U)) {
    return false;
  }
  param_store_blob_t *blob = param_store_blob_reset(g_blob_scratch, sizeof(g_blob_scratch));
  if (blob == nullptr) {
    return false;
  }
  for (size_t i = 0U; i < count && i < PARAM_STORE_RECORD_COUNT; ++i) {
    const auto &v = views[i];
    if (v.motor_id == 0U) continue;
    if (isfinite(v.kp) && isfinite(v.kd)) {
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_KP, v.kp);
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_KD, v.kd);
    }
    if (isfinite(v.pos_min_rad) && isfinite(v.pos_max_rad)) {
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_POS_MIN, v.pos_min_rad);
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_POS_MAX, v.pos_max_rad);
    }
    if (isfinite(v.velocity_limit_rad_s)) {
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_VEL_LIMIT, v.velocity_limit_rad_s);
    }
    if (isfinite(v.torque_limit_nm)) {
      (void)param_store_blob_set(blob, v.motor_id, PARAM_FIELD_TORQUE_LIMIT, v.torque_limit_nm);
    }
    for (uint8_t slot = 0U; slot < PARAM_DEVICE_SLOT_COUNT; ++slot) {
      if ((v.record_raw_present & (1U << slot)) != 0U) {
        (void)param_store_blob_set_device_raw(
            blob, v.motor_id, static_cast<param_device_slot_t>(slot), v.record_raw[slot]);
      }
    }
  }
  const uint32_t next_seq = g_active_seq + 1U;
  param_store_blob_finalize(blob, next_seq);
  const uint8_t target_slot = static_cast<uint8_t>(g_active_slot ^ 1U);
  const uint32_t base = slot_base(target_slot);
  if (w25q64_erase_sector(base) != W25Q64_OK ||
      w25q64_erase_sector(base + W25Q64_SECTOR_SIZE) != W25Q64_OK ||
      w25q64_write(base, g_blob_scratch, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    return false;
  }
  if (!read_slot(target_slot, g_blob_scratch) ||
      reinterpret_cast<const param_store_blob_t *>(g_blob_scratch)->seq != next_seq) {
    return false;
  }
  g_active_slot = target_slot;
  g_active_seq = next_seq;
  return true;
}
