// W25Q64 双槽存储：每槽 2 个 4KB 扇区，写非活动槽，启动选 CRC 有效且 seq 最新者。
// g_boot_records 仅在任务创建前加载；save 在内核临界区拍快照，退出后擦写。
// 结果与 request_id 由存储任务写、micro-ROS 读，须在同一短临界区内成对访问。

#include "storage_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <string.h>

#include "storage_projection.h"
#include "param_store_blob.h"
#include "w25q64.h"
#include "firmware_parameters.h"

static const UBaseType_t kStorageTaskPrio = 1U; // 与 led 同级最低；Flash 擦除
                                                // 阻塞期高优先级任务可抢占
static const configSTACK_DEPTH_TYPE kStorageTaskStack = 1024U; // 4KB（HAL OSPI AutoPolling 深栈）

// 每槽 8KB，v3 blob 为 7712B；constexpr 供槽边界断言索引。
static constexpr uint32_t kSlotBase[2] = {0x000000U, 0x00002000U};
static const uint32_t kSlotSectors = 2U;

// blob 不得跨槽；w25q64_range_valid 仅检查整片 8MB，不检查槽边界。
static_assert(PARAM_STORE_BLOB_SIZE <= kSlotSectors * W25Q64_SECTOR_SIZE,
    "blob 超出槽容量（2 个 4KB 扇区），整写会破坏下一个槽");

// 槽 B 必须位于槽 A 的擦除范围之外，避免擦除破坏双槽冗余。
static_assert(kSlotBase[1] >= kSlotSectors * W25Q64_SECTOR_SIZE,
    "槽 B 与槽 A 的擦除范围重叠，双槽冗余失效");

// 旧槽 B 位于 0x1000；仅当两个新槽均无效时尝试读取。
static const uint32_t kLegacySlotBBase = W25Q64_SECTOR_SIZE;

static uint32_t slot_base(uint8_t slot) {
  return kSlotBase[slot & 1U];
}

static TaskHandle_t g_task = nullptr;
static bool g_started = false;
static bool g_flash_ok = false;
static storage_param_snapshot_fn_t g_snapshot_fn = nullptr;

void storage_task_set_param_snapshot(storage_param_snapshot_fn_t fn) {
  g_snapshot_fn = fn;
}
static uint8_t g_active_slot = 0; // 0=A 1=B（无有效 blob 默认 A）
static uint32_t g_seq = 0U;

// boot 投影（start 阶段填充，运行期不变；save 的结果下次启动才生效）。
static motor_control_stored_params_t
    g_boot_records[PARAM_STORE_RECORD_COUNT];
static size_t g_boot_count = 0U;

// boot 加载与 save 不重叠，共用整 blob 缓冲；加载必须早于任务创建。
// 按 param_store_blob_t 访问须 8 字节对齐，避免 M7 非对齐 STRD 故障。
alignas(8) static uint8_t g_blob_scratch[PARAM_STORE_BLOB_SIZE];
// 读回核验按页做：不需要 7712B 的第二份全量缓冲。
alignas(8) static uint8_t g_verify_page[W25Q64_PAGE_SIZE];
static volatile storage_save_status_t g_result = STORAGE_SAVE_IDLE;
static volatile uint32_t g_result_request_id = 0U;

static void publish_result(uint32_t request_id, storage_save_status_t result) {
  taskENTER_CRITICAL();
  g_result_request_id = request_id;
  g_result = result;
  taskEXIT_CRITICAL();
}

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
  configASSERT(g_task == nullptr);
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
    // 采用后必须标记活动槽为 A，使下次 save 写 B(0x2000)，避免擦除唯一有效数据。
    if (!read_blob_at(kLegacySlotBBase, scratch)) {
      return; // 两新槽 + 旧槽 B 皆无效：保持 g_boot_count = 0
    }
    g_active_slot = 0U;
    const param_store_blob_t *adopted =
        reinterpret_cast<const param_store_blob_t *>(scratch);
    g_seq = adopted->seq;
    firmware_parameters_load(adopted->reserved);
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
  g_active_slot = use_b ? 1U : 0U;
  const param_store_blob_t *chosen =
      reinterpret_cast<const param_store_blob_t *>(scratch);
  g_seq = chosen->seq;
  firmware_parameters_load(chosen->reserved);
  g_boot_count = storage_projection_blob_to_boot(chosen, g_boot_records,
      PARAM_STORE_RECORD_COUNT);
}

// 拍快照 → 打包 → 擦写非活动槽 → 按页读回核验 → 翻转活动槽。
static void perform_save(uint32_t request_id) {
  if (!g_flash_ok || g_snapshot_fn == nullptr) {
    publish_result(request_id, STORAGE_SAVE_FAIL);
    return;
  }
  // 缓冲超过本任务 4KB 栈，须静态分配；仅 storage_task_fn 调用并独占。
  static motor_control_param_view_t views[MOTOR_CONTROL_MAX_MOTORS];
  // 提供者在本任务上下文、内核临界区内复制快照。
  const size_t n = g_snapshot_fn(views, MOTOR_CONTROL_MAX_MOTORS);

  param_store_blob_t *blob =
      param_store_blob_reset(g_blob_scratch, sizeof(g_blob_scratch));
  if (blob == nullptr) {
    publish_result(request_id, STORAGE_SAVE_FAIL);
    return;
  }
  storage_projection_views_to_blob(blob, views, n);
  const auto settings = firmware_parameters_snapshot();
  firmware_settings_pack(&settings, blob->reserved);
  g_seq++;
  param_store_blob_finalize(blob, g_seq);

  const uint8_t target = (uint8_t)(g_active_slot ^ 1U);
  const uint32_t addr = slot_base(target);
  // 擦除与编程可能阻塞数百 ms 到 ~1s/扇区（AutoPolling）；本任务 P1，
  // motor_rt(P3)/micro_ros(P2) 可抢占，IWDG 由 motor_rt 独立喂。
  // 每槽擦除 2 个扇区。
  for (uint32_t s = 0U; s < kSlotSectors; ++s) {
    if (w25q64_erase_sector(addr + s * W25Q64_SECTOR_SIZE) != W25Q64_OK) {
      publish_result(request_id, STORAGE_SAVE_FAIL); // 旧槽仍是当前槽，启动仍可加载
      return;
    }
  }
  if (w25q64_write(addr, g_blob_scratch, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    publish_result(request_id, STORAGE_SAVE_FAIL);
    return;
  }
  // 按页读回核验（7712B = 30 整页 + 32B 残页）。
  for (uint32_t off = 0U; off < PARAM_STORE_BLOB_SIZE; off += W25Q64_PAGE_SIZE) {
    const uint32_t chunk =
        (PARAM_STORE_BLOB_SIZE - off) < W25Q64_PAGE_SIZE
            ? (PARAM_STORE_BLOB_SIZE - off)
            : W25Q64_PAGE_SIZE;
    if (w25q64_read(addr + off, g_verify_page, chunk) != W25Q64_OK ||
        memcmp(g_blob_scratch + off, g_verify_page, chunk) != 0) {
      publish_result(request_id, STORAGE_SAVE_FAIL);
      return;
    }
  }
  g_active_slot = target;
  firmware_parameters_saved(&settings);
  publish_result(request_id, STORAGE_SAVE_OK);
}

static void storage_task_fn(void *pv_parameters) {
  (void)pv_parameters;
  for (;;) {
    uint32_t request_id = 0U;
    (void)xTaskNotifyWait(0U, 0U, &request_id, portMAX_DELAY);
    perform_save(request_id);
  }
}

bool storage_task_start(void) {
  if (g_started) {
    return true;
  }
  // W25 唯一所有者：在此初始化（库内自完成 OSPI 时钟/GPIO/HAL 初始化）。
  // 无芯片/ID 不符 → 降级：无持久化继续启动，save 回 FAIL。
  g_flash_ok = (w25q64_init() == W25Q64_OK);
  if (g_flash_ok) {
    load_boot_params();
  }
  if (xTaskCreate(storage_task_fn, "storage", kStorageTaskStack, nullptr,
      kStorageTaskPrio, &g_task) != pdPASS) {
    return false;
  }
  g_started = true;
  return true;
}

void storage_task_request_save(uint32_t request_id) {
  if (!g_started || request_id == 0U) {
    return;
  }
  // 任务通知值携带请求令牌。重复请求不排队，但不会丢失当前请求与结果的关联。
  (void)xTaskNotify(g_task, request_id, eSetValueWithOverwrite);
}

storage_save_status_t storage_take_result(uint32_t *request_id) {
  taskENTER_CRITICAL();
  const storage_save_status_t result = g_result;
  const uint32_t result_request_id = g_result_request_id;
  g_result = STORAGE_SAVE_IDLE;
  g_result_request_id = 0U;
  taskEXIT_CRITICAL();
  if (request_id != nullptr) {
    *request_id = result_request_id;
  }
  return result;
}

const motor_control_stored_params_t *storage_boot_params(size_t *count) {
  if (count != nullptr) {
    *count = g_boot_count;
  }
  return g_boot_records;
}
