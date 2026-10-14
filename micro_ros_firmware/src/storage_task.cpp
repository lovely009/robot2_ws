// W25Q64 参数存储任务（协议见 storage_task.h）。
// 双槽 A/B、每槽 2 个 4KB 扇区（W25Q64 最小擦除单位）：save 总写非活动槽，boot 选 CRC 有效且 seq
// 最大者——擦写中掉电至少保留一个好槽。升级兼容：旧布局的槽 B 在 0x1000、新布局下落进槽 A 的擦除
// 范围，两个新槽都读不到时才兜底试一次，免得奇数存档次数的板子升级后丢光参数。
// 并发：g_boot_records 只在 start 的加载阶段写（任务创建前）；save 在 motor_rt 临界区内拍快照、离开
// 临界区再擦写；结果状态与 request_id 由本任务写、micro-ROS 任务读，必须成对进出短临界区。

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

// 双槽 A/B 的基址：每槽 2 个 4KB 扇区，v3 blob 占 7712B（见文件头）。
// constexpr 而非 const：下面的槽不重叠断言要在编译期索引 kSlotBase[1]。
static constexpr uint32_t kSlotBase[2] = {0x000000U, 0x00002000U};
static const uint32_t kSlotSectors = 2U;

// 一个 blob 必须装得进一个槽。超了的话 perform_save 的整写会越过槽尽头写进
// 下一个槽——w25q64_range_valid 只兜整片 8MB，不兜槽边界，会静默破坏另一个
// 槽里的有效 blob。7712 对 8192 还剩 480B（记录尺寸涨到 127B 以上就越界）。
static_assert(PARAM_STORE_BLOB_SIZE <= kSlotSectors * W25Q64_SECTOR_SIZE,
    "blob 超出槽容量（2 个 4KB 扇区），整写会破坏下一个槽");

// 槽区不重叠：槽 B 的基址必须在槽 A 的擦除范围之外。上面那条只保证 blob 装得进
// 一个槽，不保证两槽首尾相接——kSlotBase 一旦手滑改小（如把 B 写成 0x1000），
// 擦 A 会连带擦掉 B，双槽冗余静默失效。范围由 W25Q64_SECTOR_SIZE 与 kSlotSectors
// 派生，改布局时这条会先炸。
static_assert(kSlotBase[1] >= kSlotSectors * W25Q64_SECTOR_SIZE,
    "槽 B 与槽 A 的擦除范围重叠，双槽冗余失效");

// 升级前旧布局的槽 B 基址（旧 kSectorB = W25Q64_SECTOR_SIZE = 0x1000）。
// 新布局下 0x1000 落在槽 A 的擦除范围内，read_slot() 永远读不到它，
// 只有 load_boot_params() 的兜底分支会碰（见那里的说明）。
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

// 唯一一块整 blob 暂存：boot 阶段（任务创建前）读入胜者，save 阶段打包写出，两者
// 时间上不重叠（load_boot_params 只在 storage_task_start 内、xTaskCreate 之前）。
// 必须 8 字节对齐：上层按 param_store_blob_t 访问，-Os 会用 STRD，非对齐在 M7 上
// 直接 UNDEFINSTR。
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

// 读 addr 处的 blob 到调用方缓冲并落到 v3 布局：优先按 v3 校验；不是 v3 时再
// 依次尝试 v2（48B 记录）与 v1（升级前写入的 32B 记录格式）就地迁移——升级不
// 该静默丢掉用户的 kp/kd、限值与设备端原值记录。迁移结果**再 validate 一遍**
// （设计 §6 要求）：这条 true 是调用方对该缓冲唯一的信任来源。
static bool read_blob_at(uint32_t addr, uint8_t *buf) {
  if (w25q64_read(addr, buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK) {
    return false;
  }
  if (param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf))) {
    return true;
  }
  // v2 就地迁移（升级不该静默丢掉用户的记录）。
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
    // 两新槽皆无效：可能是从旧布局升级上来的板子。旧槽 B 在 0x1000，落在新槽 A 的
    // 擦除范围内，read_slot() 再也读不到它——旧固件下存档次数为**奇数**的板子若不
    // 在这里兜底，会静默丢掉全部持久化参数。只在新布局两槽都失败时试一次。
    //
    // 采用时 g_active_slot 必须是 0：数据物理上位于 0x1000（槽 A 的擦除范围内），
    // 下次 save 只能写槽 B(0x2000)，而 g_active_slot = 0 正好给出 target = 1。
    // **不能**把它当成 slot 1 的内容返回——那会让 use_b = 1，首次存档就擦掉刚采用的数据。
    if (!read_blob_at(kLegacySlotBBase, scratch)) {
      return; // 两新槽 + 旧槽 B 皆无效：保持 g_boot_count = 0
    }
    g_active_slot = 0U;
    const param_store_blob_t *adopted =
        reinterpret_cast<const param_store_blob_t *>(scratch);
    g_seq = adopted->seq;
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
  g_boot_count = storage_projection_blob_to_boot(chosen, g_boot_records,
      PARAM_STORE_RECORD_COUNT);
}

// 拍快照 → 打包 → 擦写非活动槽 → 按页读回核验 → 翻转活动槽。
static void perform_save(uint32_t request_id) {
  if (!g_flash_ok || g_snapshot_fn == nullptr) {
    publish_result(request_id, STORAGE_SAVE_FAIL);
    return;
  }
  // **静态而非栈上**：64 × 80 B = 5,120 B，本任务栈只有 4 KB，放栈上必然溢出 →
  // 栈溢出 fail-safe（main.cpp 的 hook：关中断死循环）→ IWDG 无人喂 → ~500ms 复位。
  // 台架现象就是「一落盘就重启」。本任务独占该缓冲（perform_save 只由 storage_task_fn 调）。
  static motor_control_param_view_t views[MOTOR_CONTROL_MAX_MOTORS];
  // 经注入的提供者拍快照（实现在 motor_rt：进内核临界区，仍在本任务
  // 上下文执行——与旧的直接互调逐字同构）。
  const size_t n = g_snapshot_fn(views, MOTOR_CONTROL_MAX_MOTORS);

  param_store_blob_t *blob =
      param_store_blob_reset(g_blob_scratch, sizeof(g_blob_scratch));
  if (blob == nullptr) {
    publish_result(request_id, STORAGE_SAVE_FAIL);
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
