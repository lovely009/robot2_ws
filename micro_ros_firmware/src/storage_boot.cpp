// Boot-only read of existing motor records. No Flash writes, erases or runtime task.
#include "storage_boot.h"
#include "storage_projection.h"
#include "param_store_blob.h"
#include "w25q64.h"

static constexpr uint32_t kSlotBase[2] = {0U, 0x2000U};
static constexpr uint32_t kLegacySlotBBase = 0x1000U;
static uint32_t slot_base(uint8_t slot) { return kSlotBase[slot & 1U]; }
static motor_control_stored_params_t g_boot_records[PARAM_STORE_RECORD_COUNT];
static size_t g_boot_count;
alignas(8) static uint8_t g_blob_scratch[PARAM_STORE_BLOB_SIZE];

// 优先校验 v3，否则尝试 v2（48B/记录）、v1（32B/记录）迁移；迁移后须重新校验。
static bool read_blob_at(uint32_t addr, uint8_t *buf) {
  if (w25q64_read(addr, buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    return false;
  }
  if (param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf))) {
    return true;
  }
  // v2 就地迁移。
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
  // 完整读取并校验后才比较 seq，必要时重读选中槽，避免使用损坏的头。
  uint8_t *scratch = g_blob_scratch;
  const bool ok_a = read_slot(0U, scratch);
  const uint32_t seq_a =
      ok_a ? reinterpret_cast<const param_store_blob_t *>(scratch)->seq : 0U;
  const bool ok_b = read_slot(1U, scratch);
  const uint32_t seq_b =
      ok_b ? reinterpret_cast<const param_store_blob_t *>(scratch)->seq : 0U;

  bool use_b;
  if (ok_a && ok_b) {
    use_b = (int32_t)(seq_b - seq_a) > 0; // 回绕按差值比较
  } else if (ok_b) {
    use_b = true;
  } else if (ok_a) {
    use_b = false;
  } else {
    // 两新槽无效时尝试旧槽 B（0x1000，位于新槽 A 的擦除范围）。
    if (!read_blob_at(kLegacySlotBBase, scratch)) {
      return; // 两新槽 + 旧槽 B 皆无效：保持 g_boot_count = 0
    }
    const param_store_blob_t *adopted =
        reinterpret_cast<const param_store_blob_t *>(scratch);
    g_boot_count = storage_projection_blob_to_boot(
        adopted, g_boot_records, PARAM_STORE_RECORD_COUNT);
    return;
  }
  // 胜者是 B 时它已在 scratch 里；是 A 时需要重读（顺带重新校验）。
  if (!use_b && !read_slot(0U, scratch)) {
    if (!read_slot(1U, scratch)) {
      return;
    }
    use_b = true;
  }
  const param_store_blob_t *chosen =
      reinterpret_cast<const param_store_blob_t *>(scratch);
  g_boot_count = storage_projection_blob_to_boot(chosen, g_boot_records,
      PARAM_STORE_RECORD_COUNT);
}


void storage_boot_load(void) {
  g_boot_count = 0;
  if (w25q64_init() == W25Q64_OK) load_boot_params();
}
const motor_control_stored_params_t *storage_boot_params(size_t *count) {
  if (count) *count = g_boot_count;
  return g_boot_records;
}
