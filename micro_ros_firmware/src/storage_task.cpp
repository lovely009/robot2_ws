// W25Q64 参数存储任务实现（协议见 storage_task.h）。
//
// 双 sector A/B（各放一个 2080B blob，地址 0x0/0x1000）：save 总写非活动槽，
// boot 选 CRC 有效且 seq 最大者——erase/write 中掉电至少保留一个好槽。
//
// 并发：g_boot_records 只在 start 的加载阶段写（任务创建前）；save 路径用
// motor_rt 临界区拍参数快照后离开临界区再擦写；结果单字由 micro_ros 任务
// 读（take_result）与本任务写，可视为原子单字读写。

#include "storage_task.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <string.h>

#include "motor_rt_task.h"
#include "param_store_blob.h"
#include "w25q64.h"

static const UBaseType_t kStorageTaskPrio = 1U; // 与 led 同级最低；Flash 擦除
                                                // 阻塞期高优先级任务可抢占
static const configSTACK_DEPTH_TYPE kStorageTaskStack = 1024U; // 4KB（HAL OSPI AutoPolling 深栈）

static const uint32_t kSectorA = 0x000000U;
static const uint32_t kSectorB = 0x001000U;

static TaskHandle_t g_task = nullptr;
static bool g_started = false;
static bool g_flash_ok = false;
static uint8_t g_active_sector = 0; // 0=A 1=B（无有效 blob 默认 A）
static uint32_t g_seq = 0U;

// boot 投影（start 阶段填充，运行期不变；save 的结果下次启动才生效）。
static motor_control_stored_params_t
    g_boot_records[PARAM_STORE_RECORD_COUNT];
static size_t g_boot_count = 0U;

// save 暂存与读回校验缓冲（静态，避免擦写路径上的栈压力与动态分配）。
// 必须 4 字节对齐：上层按 param_store_blob_t 结构体访问，-Os 会用 STRD，
// 非对齐在 M7 上直接 UNDEFINSTR（uint8_t 数组默认只保证 1 字节对齐）。
alignas(8) static uint8_t g_write_buf[PARAM_STORE_BLOB_SIZE];
alignas(8) static uint8_t g_verify_buf[PARAM_STORE_BLOB_SIZE];
static volatile storage_save_status_t g_result = STORAGE_SAVE_IDLE;

static uint32_t sector_address(uint8_t slot) {
  return slot == 0U ? kSectorA : kSectorB;
}

// 把内核参数视图投影成 blob 字段（只收录有限值；enable 不持久化）。
static void project_views_to_blob(param_store_blob_t *blob,
                                  const motor_control_param_view_t *views,
                                  size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const motor_control_param_view_t *v = &views[i];
    if (v->motor_id == 0U) {
      continue;
    }
    if (isfinite(v->kp)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_KP, v->kp);
    }
    if (isfinite(v->kd)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_KD, v->kd);
    }
    if (isfinite(v->pos_min_rad)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MIN,
                           v->pos_min_rad);
    }
    if (isfinite(v->pos_max_rad)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_POS_MAX,
                           v->pos_max_rad);
    }
    if (isfinite(v->velocity_limit_rad_s)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_VEL_LIMIT,
                           v->velocity_limit_rad_s);
    }
    if (isfinite(v->torque_limit_nm)) {
      param_store_blob_set(blob, v->motor_id, PARAM_FIELD_TORQUE_LIMIT,
                           v->torque_limit_nm);
    }
  }
}

// blob → boot 记录（每电机一条；字段存在性来自 present 位图）。
static void project_blob_to_boot(const param_store_blob_t *blob) {
  g_boot_count = 0U;
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_t *rec = &blob->records[i];
    if (rec->present == 0U || g_boot_count >= PARAM_STORE_RECORD_COUNT) {
      continue;
    }
    motor_control_stored_params_t *out = &g_boot_records[g_boot_count++];
    memset(out, 0, sizeof(*out));
    out->motor_id = rec->motor_id;
    float value = 0.0f;
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_KP, &value) &&
        param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_KD, &value)) {
      out->has_kp_kd = true;
      out->kp = rec->kp;
      out->kd = rec->kd;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_POS_MIN,
                             &value) &&
        param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_POS_MAX,
                             &value)) {
      out->has_position_limits = true;
      out->pos_min_rad = rec->pos_min;
      out->pos_max_rad = rec->pos_max;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_VEL_LIMIT,
                             &value)) {
      out->has_velocity_limit = true;
      out->velocity_limit_rad_s = rec->vel_limit;
    }
    if (param_store_blob_get(blob, rec->motor_id, PARAM_FIELD_TORQUE_LIMIT,
                             &value)) {
      out->has_torque_limit = true;
      out->torque_limit_nm = rec->torque_limit;
    }
  }
}

// 读取一个 sector 的 blob；有效返回指针（指向 static 读缓冲外的调用方缓冲）。
static bool read_slot(uint8_t slot, uint8_t *buf) {
  if (w25q64_read(sector_address(slot), buf, PARAM_STORE_BLOB_SIZE) !=
      W25Q64_OK) {
    return false;
  }
  return param_store_blob_validate(
      reinterpret_cast<const param_store_blob_t *>(buf));
}

static void load_boot_params() {
  alignas(8) static uint8_t slot_a[PARAM_STORE_BLOB_SIZE];
  alignas(8) static uint8_t slot_b[PARAM_STORE_BLOB_SIZE];
  const bool ok_a = read_slot(0, slot_a);
  const bool ok_b = read_slot(1, slot_b);
  const param_store_blob_t *blob_a =
      reinterpret_cast<const param_store_blob_t *>(slot_a);
  const param_store_blob_t *blob_b =
      reinterpret_cast<const param_store_blob_t *>(slot_b);
  const param_store_blob_t *chosen = nullptr;
  if (ok_a && ok_b) {
    // seq 单调递增；回绕按"差值"比较（无符号溢出安全）。
    if ((int32_t)(blob_b->seq - blob_a->seq) > 0) {
      chosen = blob_b;
      g_active_sector = 1U;
    } else {
      chosen = blob_a;
      g_active_sector = 0U;
    }
  } else if (ok_a) {
    chosen = blob_a;
    g_active_sector = 0U;
  } else if (ok_b) {
    chosen = blob_b;
    g_active_sector = 1U;
  }
  if (chosen != nullptr) {
    g_seq = chosen->seq;
    project_blob_to_boot(chosen);
  }
}

// 拍快照 → 打包 → 擦写非活动槽 → 读回 CRC 核验 → 翻转活动槽。
static void perform_save() {
  if (!g_flash_ok) {
    g_result = STORAGE_SAVE_FAIL;
    return;
  }
  motor_control_param_view_t views[MOTOR_CONTROL_MAX_MOTORS];
  const size_t n =
      motor_rt_copy_param_views(views, MOTOR_CONTROL_MAX_MOTORS);

  param_store_blob_t *blob =
      param_store_blob_reset(g_write_buf, sizeof(g_write_buf));
  if (blob == nullptr) {
    g_result = STORAGE_SAVE_FAIL;
    return;
  }
  project_views_to_blob(blob, views, n);
  g_seq++;
  param_store_blob_finalize(blob, g_seq);

  const uint8_t target = (uint8_t)(g_active_sector ^ 1U);
  const uint32_t addr = sector_address(target);
  // 擦除与编程可能阻塞数百 ms 到 ~1s（AutoPolling）；本任务 P1，
  // motor_rt(P3)/micro_ros(P2) 可抢占，IWDG 由 motor_rt 独立喂。
  if (w25q64_erase_sector(addr) != W25Q64_OK ||
      w25q64_write(addr, g_write_buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK ||
      w25q64_read(addr, g_verify_buf, PARAM_STORE_BLOB_SIZE) != W25Q64_OK ||
      memcmp(g_write_buf, g_verify_buf, PARAM_STORE_BLOB_SIZE) != 0) {
    g_result = STORAGE_SAVE_FAIL; // 旧槽仍是当前槽，启动仍可加载
    return;
  }
  g_active_sector = target;
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
