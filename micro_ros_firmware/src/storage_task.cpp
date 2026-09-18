// W25Q64 参数存储任务实现（协议见 storage_task.h）。
//
// 双槽 A/B，每槽 2 个 4KB 扇区（W25Q64 最小擦除单位），各放一个 blob：
// v3 为 7712B = 32B 头 + 64×120B 记录，槽 A 0x0000–0x1FFF、槽 B 0x2000–0x3FFF
// （v2 记录 48B、v1 旧格式 32B，仍可读，见 read_slot 的迁移链）。save 总写非
// 活动槽，boot 选 CRC 有效且 seq 最大者——erase/write 中掉电至少保留一个好槽
// （擦 A 的扇区不碰 B）。
//
// 并发：g_boot_records 只在 start 的加载阶段写（任务创建前）；save 路径用
// motor_rt 临界区拍参数快照后离开临界区再擦写；结果单字由 micro_ros 任务
// 读（take_result）与本任务写，可视为原子单字读写。

#include "storage_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <string.h>

#include "storage_projection.h"
#include "param_store_blob.h"
#include "w25q64.h"

static const UBaseType_t kStorageTaskPrio = 1U; // 与 led 同级最低；Flash 擦除
                                                // 阻塞期高优先级任务可抢占
static const configSTACK_DEPTH_TYPE kStorageTaskStack = 1024U; // 4KB（HAL OSPI AutoPolling 深栈）

// 双槽 A/B，每槽 2 个 4KB 扇区（W25Q64 最小擦除单位），各放一个 blob：
// v3 为 7712B = 32B 头 + 64×120B 记录。save 总写非活动槽，boot 选 CRC 有效且
// seq 最大者——erase/write 中掉电至少保留一个好槽（擦 A 的扇区不碰 B）。
static const uint32_t kSlotBase[2] = {0x000000U, 0x00002000U};
static const uint32_t kSlotSectors = 2U;

static uint32_t slot_base(uint8_t slot) { return kSlotBase[slot & 1U]; }

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

// 唯一一块整 blob 暂存：boot 阶段（任务创建前）读入胜者，save 阶段打包写出。
// 两者无时间重叠——load_boot_params() 只在 storage_task_start() 内调用一次，
// 且发生在 xTaskCreate 之前，其入口有 configASSERT(g_task == nullptr) 把关。
// 必须 8 字节对齐：上层按 param_store_blob_t 结构体访问，-Os 会用 STRD，
// 非对齐在 M7 上直接 UNDEFINSTR。
alignas(8) static uint8_t g_blob_scratch[PARAM_STORE_BLOB_SIZE];
// 读回核验按页做：不需要 7712B 的第二份全量缓冲。
alignas(8) static uint8_t g_verify_page[W25Q64_PAGE_SIZE];
static volatile storage_save_status_t g_result = STORAGE_SAVE_IDLE;

// 读取一个槽到调用方缓冲并落到 v3 布局：优先按 v3 校验；不是 v3 时再依次尝试
// v2（48B 记录）与 v1（升级前写入的 32B 记录格式）就地迁移——升级不该静默丢
// 掉用户的 kp/kd、限值与设备端原值记录。
static bool read_slot(uint8_t slot, uint8_t *buf) {
  if (w25q64_read(slot_base(slot), buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    return false;
  }
  if (param_store_blob_validate(
          reinterpret_cast<const param_store_blob_t *>(buf))) {
    return true;
  }
  // v2 就地迁移（升级不该静默丢掉用户的记录）。
  alignas(8) static uint8_t v2_buf[PARAM_STORE_BLOB_SIZE_V2];
  if (w25q64_read(slot_base(slot), v2_buf, sizeof(v2_buf)) == W25Q64_OK &&
      param_store_blob_migrate_v2(v2_buf, sizeof(v2_buf), buf,
                                  PARAM_STORE_BLOB_SIZE) != nullptr) {
    return true;
  }
  alignas(8) static uint8_t legacy_buf[PARAM_STORE_LEGACY_BLOB_SIZE];
  if (w25q64_read(slot_base(slot), legacy_buf, sizeof(legacy_buf)) != W25Q64_OK) {
    return false;
  }
  return param_store_blob_migrate_v1(legacy_buf, sizeof(legacy_buf), buf,
                                     PARAM_STORE_BLOB_SIZE) != nullptr;
}

static void load_boot_params() {
  // 单缓冲：不做「只读头选胜者」的捷径——那会拿未经校验的 seq 做裁决，
  // 头损坏时可能选错槽。改为顺序整读 + 逐个 validate + 必要时重读赢家。
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
    return; // 两槽皆无效：保持 g_boot_count = 0
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
  g_boot_count = storage_projection_blob_to_boot(chosen, g_boot_records,
                                                PARAM_STORE_RECORD_COUNT);
}

// 拍快照 → 打包 → 擦写非活动槽 → 按页读回核验 → 翻转活动槽。
static void perform_save() {
  if (!g_flash_ok || g_snapshot_fn == nullptr) {
    g_result = STORAGE_SAVE_FAIL;
    return;
  }
  // **静态而非栈上**：64 × 80 B = 5,120 B，而本任务栈只有 4 KB
  // （kStorageTaskStack = 1024 words）。放在栈上必然溢出 → FreeRTOS
  // 栈溢出 fail-safe（main.cpp 的 hook：关中断死循环）→ IWDG 无人喂 →
  // ~500ms 后复位。台架现象就是「一落盘就重启」。
  // 本任务独占该缓冲（perform_save 只由 storage_task_fn 调用）。
  static motor_control_param_view_t views[MOTOR_CONTROL_MAX_MOTORS];
  // 经注入的提供者拍快照（实现在 motor_rt：进内核临界区，仍在本任务
  // 上下文执行——与旧的直接互调逐字同构）。
  const size_t n = g_snapshot_fn(views, MOTOR_CONTROL_MAX_MOTORS);

  param_store_blob_t *blob =
      param_store_blob_reset(g_blob_scratch, sizeof(g_blob_scratch));
  if (blob == nullptr) {
    g_result = STORAGE_SAVE_FAIL;
    return;
  }
  storage_projection_views_to_blob(blob, views, n);
  g_seq++;
  param_store_blob_finalize(blob, g_seq);

  const uint8_t target = (uint8_t)(g_active_slot ^ 1U);
  const uint32_t addr = slot_base(target);
  // 擦除与编程可能阻塞数百 ms 到 ~1s/扇区（AutoPolling）；本任务 P1，
  // motor_rt(P3)/micro_ros(P2) 可抢占，IWDG 由 motor_rt 独立喂。
  // v3 每槽 2 个扇区，擦除时间约为原来的两倍（实测值见 validation plan）。
  for (uint32_t s = 0U; s < kSlotSectors; ++s) {
    if (w25q64_erase_sector(addr + s * W25Q64_SECTOR_SIZE) != W25Q64_OK) {
      g_result = STORAGE_SAVE_FAIL; // 旧槽仍是当前槽，启动仍可加载
      return;
    }
  }
  if (w25q64_write(addr, g_blob_scratch, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    g_result = STORAGE_SAVE_FAIL;
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
      g_result = STORAGE_SAVE_FAIL;
      return;
    }
  }
  g_active_slot = target;
  g_result = STORAGE_SAVE_OK;
}

static void storage_task_fn(void *pv_parameters) {
  (void)pv_parameters;
  for (;;) {
    xTaskNotifyWait(0U, 0U, nullptr, portMAX_DELAY);
    perform_save();
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

void storage_task_request_save() {
  if (!g_started) {
    return;
  }
  // eSetBits 语义下重复通知合并：保存进行中的脉冲不排队堆积。
  (void)xTaskNotify(g_task, 0U, eNoAction);
}

storage_save_status_t storage_take_result(void) {
  const storage_save_status_t result = g_result;
  g_result = STORAGE_SAVE_IDLE;
  return result;
}

const motor_control_stored_params_t *storage_boot_params(size_t *count) {
  if (count != nullptr) {
    *count = g_boot_count;
  }
  return g_boot_records;
}
