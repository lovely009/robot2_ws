# W25 参数 blob v3 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **勘误（2026-09-19，真机台架）**：本计划全程把安全开关建模成 `0x2650:01`，是**错的**。
> 四台真机上 `0x2650` 只有子索引 **`0x00`** 可读，`:01`/`:02` 回 ABORT `0x06090011`
> （子索引不存在）——手册写命令 `23 50 26 00 01 …` 里 `00` 才是子索引，后面的 `01`
> 是数据。按 `:01` 读会让 `REFRESH_PARAMS` 恒挂在安全槽上，全部 CERB 到不了
> `DEV_INIT_OK`、永远无法使能。代码、模拟器与 host 测试都已改为 `0x00`。
> 见[真机台架记录](../../../docs/reports/2026-09-19-cerb-real-hardware-bench.md)与
> [validation plan](motor-runtime-validation.md) 的 TBD-CONFIG-02。
> **下文保留当时原文**（含各任务里的代码片段与命令），不再逐行改写；
> 读到时请按本勘误换算子索引。

**Goal:** 把 W25 参数 blob 从 v2（48 B 记录 / 每槽 1 扇区）升到 v3（120 B 记录 / 每槽 2 扇区），
并把设备侧安全事实（`0x2650:01` 看门狗+限位保护、`0x2541` 方向）纳入上电比对与拒绝使能门禁。

**Architecture:** 记录尾部留 `reserved[23]`，后续项不再需要格式迁移；新事实做成两个
`device_raw` 槽位而非新造 flags，从而零改动复用现有的落袋 / 位图 / 比对 / 事件位图通路；
boot 裁决砍掉「只读头选胜者」捷径，改顺序整读 + 必要时重读赢家，单缓冲。

**Tech Stack:** C17 / C++17（`lib/param_store`、`lib/motor_control`、`lib/motor_protocol`）、
PlatformIO（STM32H723VGT6）、host 测试用 CTest + `assert`（**无 gtest**）、W25Q64 OSPI。

**Spec:** `micro_ros_firmware/docs/plans/active/w25-blob-v3-design.md`

## Global Constraints

- 记录尺寸 **120 B**；blob = 32 B 头 + 64×120 B = **7712 B**；槽 A = `0x0000–0x1FFF`，
  槽 B = `0x2000–0x3FFF`；`W25Q64_SECTOR_SIZE = 4096`，页 256 B。
- `PARAM_STORE_RECORD_COUNT == MOTOR_CONTROL_MAX_MOTORS == 64` —— **不得改动**
  （`src/storage_projection.c:18` 的 `static_assert` 会拦）。
- `PARAM_DEVICE_SLOT_COUNT` 4 → 6；`device_raw_present` 是 `uint8_t`，**上限 8 槽**。
- `PARAM_DEVICE_SLOT_*` 与 `MOTOR_PROTOCOL_PARAM_SLOT_*` **两侧必须同序同步**
  （`storage_projection.c:11-17` 的跨库 `static_assert` 兜底）。
- `param_field_t` 仍是 7 项且**不要改**——它是 blob `present` 位图契约。
- host 测试风格：`#include <cassert>`，`namespace { void test_xxx() {...} }`，
  `main()` 里逐个调用，末尾 `printf("test_xxx: all passed\n"); return 0;`。**不用 gtest**。
- 固件构建：`~/.platformio/penv/bin/pio run`（`pio` 不在 PATH）。
- host 测试：`cmake -S test/host -B .host-build && cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure`。
- 改 `platformio.ini` 编译宏或 `ldscript/*.ld` 后必须 `pio run -t clean`——**本计划两者都不动，不要 clean**。
- 提交前缀：`feat:` / `fix:` / `refactor:` / `docs:` / `test:`。

---

### Task 1: v3 记录布局与常量

改掉尺寸常量与 `param_store_record_t`。这一步之后旧测试里对 48 B / 3104 B 的隐含假设会暴露出来，
所以本任务**同时**把 `param_store_blob.c` 的 `_Static_assert` 与 `test_param_store.cpp` 的尺寸断言一起改到自洽。

**Files:**
- Modify: `lib/param_store/include/param_store_blob.h`
- Modify: `lib/param_store/src/param_store_blob.c:5-8`
- Test: `test/host/test_param_store.cpp`

**Interfaces:**
- Consumes: 无（首个任务）
- Produces: `PARAM_STORE_VERSION = 3`、`PARAM_STORE_RECORD_SIZE = 120`、
  `PARAM_STORE_BLOB_SIZE = 7712`、`PARAM_STORE_VERSION_V2 = 2`、
  `PARAM_STORE_RECORD_SIZE_V2 = 48`、`PARAM_STORE_BLOB_SIZE_V2 = 3104`；
  `param_store_record_t`（尾部新增 `model_name[16]` / `identity_hash` / `mit_*` 三标度 /
  `profile_version` / `reserved[23]`，`device_raw` 由 `[4]` 扩到 `[8]`）。

- [ ] **Step 1: 写失败测试**

在 `test/host/test_param_store.cpp` 的 `namespace {` 内、`test_blob_roundtrip_and_crc` 之前插入：

```cpp
// v3 布局尺寸契约：改动尺寸必须先动这里的断言，防止静默漂移。
void test_blob_v3_layout_size() {
  static_assert(sizeof(param_store_record_t) == 120U, "v3 记录必须 120B");
  static_assert(PARAM_STORE_RECORD_SIZE == 120U, "v3 记录尺寸常量");
  static_assert(PARAM_STORE_BLOB_SIZE == 7712U, "v3 blob = 32 + 64*120");
  static_assert(PARAM_STORE_VERSION == 3U, "v3 版本号");
  // 槽区：每槽 2 个 4KB 扇区，blob 必须装得下。
  assert(PARAM_STORE_BLOB_SIZE <= 2U * 4096U);
  // 字段偏移契约（迁移与页读回都依赖它）。
  param_store_record_t rec{};
  assert(reinterpret_cast<uint8_t *>(&rec.present) == reinterpret_cast<uint8_t *>(&rec));
  assert(reinterpret_cast<uint8_t *>(&rec.device_raw) - reinterpret_cast<uint8_t *>(&rec) == 32);
  assert(reinterpret_cast<uint8_t *>(&rec.model_name) - reinterpret_cast<uint8_t *>(&rec) == 64);
  assert(reinterpret_cast<uint8_t *>(&rec.identity_hash) - reinterpret_cast<uint8_t *>(&rec) == 80);
  assert(reinterpret_cast<uint8_t *>(&rec.mit_pos_limit_rad) - reinterpret_cast<uint8_t *>(&rec) == 84);
  assert(reinterpret_cast<uint8_t *>(&rec.profile_version) - reinterpret_cast<uint8_t *>(&rec) == 96);
  assert(sizeof(rec.device_raw) == 32U);
  assert(sizeof(rec.reserved) == 23U);
}
```

在 `main()` 里 `test_blob_roundtrip_and_crc();` 之前加 `test_blob_v3_layout_size();`。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 2>&1 | tail -20
```
Expected: 编译失败，`static assertion failed: v3 记录必须 120B`（以及 `PARAM_STORE_BLOB_SIZE` 相关断言）。

- [ ] **Step 3: 改常量与结构体**

`lib/param_store/include/param_store_blob.h`，把版本/尺寸段替换为：

```c
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
```

同文件的 `param_store_record_t` 替换为：

```c
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
  // v3：槽位 4→8（现用 6，留 2 给后续设备对象如 0x6081/0x6060）。
  uint32_t device_raw[8]; // cnt/cnt/RPM/mA/0x2650:01/0x2541
  // v3 新增：型号身份与来源链。
  char model_name[16];     // 型号目录名，NUL 结尾；空 = 未绑定
  uint32_t identity_hash;  // 设备 0x1008 自报串的 CRC32；0 = 未核验
  // v3 新增：写记录时固件假定的 MIT 全量程（来源 = 型号目录）。
  // 值 0 = 未知（迁移来的记录）。用途：让「固件升级改了 profile」可被发现。
  float mit_pos_limit_rad;
  float mit_vel_limit_rad_s;
  float mit_torque_limit_nm;
  uint8_t profile_version; // 型号目录版本
  uint8_t reserved[23];    // 后续项：零点 epoch / 跨零 / 运行时长 / MIT 实测标度
} param_store_record_t;
```

`lib/param_store/src/param_store_blob.c:5-8` 的两条 `_Static_assert` 的**文案**改为：

```c
_Static_assert(sizeof(param_store_record_t) == PARAM_STORE_RECORD_SIZE,
               "record 必须 120B（v3）");
_Static_assert(sizeof(param_store_blob_t) == PARAM_STORE_BLOB_SIZE,
               "blob 布局尺寸漂移（v3 = 32 + 64*120）");
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build .host-build -j2 2>&1 | tail -5 && ctest --test-dir .host-build --output-on-failure -R test_param_store
```
Expected: PASS，输出 `test_param_store: all passed`。

- [ ] **Step 5: 提交**

```bash
git add lib/param_store/include/param_store_blob.h lib/param_store/src/param_store_blob.c test/host/test_param_store.cpp
git commit -m "feat(fw): blob v3 记录布局 120B（device_raw 扩至 8 槽 + 型号身份 + MIT 假定标度）"
```

---

### Task 2: 设备槽位枚举扩到 6 并两侧同步

**Files:**
- Modify: `lib/param_store/include/param_name.h:33-39`
- Modify: `lib/motor_protocol/include/motor_protocol_adapter.h:223-228`
- Modify: `src/storage_projection.c:11-17`（增补两条 `static_assert`）
- Test: `test/host/test_param_store.cpp`

**Interfaces:**
- Consumes: Task 1 的 `device_raw[8]` 与 `device_raw_present`
- Produces: `PARAM_DEVICE_SLOT_SAFETY = 4`、`PARAM_DEVICE_SLOT_DIRECTION = 5`、
  `PARAM_DEVICE_SLOT_COUNT = 6`；对应的 `MOTOR_PROTOCOL_PARAM_SLOT_SAFETY = 4`、
  `MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION = 5`、`MOTOR_PROTOCOL_PARAM_SLOT_COUNT = 6`

- [ ] **Step 1: 写失败测试**

在 `test/host/test_param_store.cpp` 的 `test_blob_device_raw_roundtrip` 之后插入：

```cpp
// v3 新增两个设备槽位（0x2650:01 安全开关 / 0x2541 方向）：可独立于四个限值槽
// 落袋，位图互不干扰，越界拒绝。
void test_blob_device_raw_safety_slots() {
  static uint8_t storage[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob = param_store_blob_reset(storage, sizeof(storage));
  assert(PARAM_DEVICE_SLOT_COUNT == 6U);
  assert(PARAM_DEVICE_SLOT_COUNT == MOTOR_PROTOCOL_PARAM_SLOT_COUNT);
  assert(PARAM_DEVICE_SLOT_SAFETY == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  assert(PARAM_DEVICE_SLOT_DIRECTION == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  // 只写安全槽（不写限值）：记录必须存在且只有该位。
  assert(param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_SAFETY, 1U));
  assert(param_store_blob_device_raw_present(blob, 7) ==
         (uint8_t)(1U << PARAM_DEVICE_SLOT_SAFETY));
  // 再加方向槽，两位并存。
  assert(param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_DIRECTION, 0U));
  assert(param_store_blob_device_raw_present(blob, 7) ==
         (uint8_t)((1U << PARAM_DEVICE_SLOT_SAFETY) |
                   (1U << PARAM_DEVICE_SLOT_DIRECTION)));
  // 值 0 也是有效记录（方向 CW = 0），不能与"未核验"混淆。
  uint32_t raw = 123U;
  assert(param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_DIRECTION, &raw));
  assert(raw == 0U);
  // 与四个限值槽共存、互不覆盖。
  assert(param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_VEL_LIMIT, 3030U));
  assert(param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_SAFETY, &raw) && raw == 1U);
  assert(param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_VEL_LIMIT, &raw) && raw == 3030U);
  // 越界（COUNT 与 8 之间的空槽、以及 >= 8）都拒绝。
  assert(!param_store_blob_set_device_raw(blob, 7, PARAM_DEVICE_SLOT_COUNT, 1U));
  assert(!param_store_blob_set_device_raw(blob, 7, (param_device_slot_t)78, 1U));
  assert(!param_store_blob_get_device_raw(blob, 7, PARAM_DEVICE_SLOT_COUNT, &raw));
  // device_raw_present 是 u8：满位图 0x3F 不溢出。
  assert(param_store_blob_device_raw_present(blob, 7) <= 0x3FU);
}
```

在 `main()` 里加 `test_blob_device_raw_safety_slots();`。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -20
```
Expected: 编译失败，`'PARAM_DEVICE_SLOT_SAFETY' was not declared`。

- [ ] **Step 3: 改两侧枚举**

`lib/param_store/include/param_name.h`：

```c
typedef enum {
  PARAM_DEVICE_SLOT_POS_MIN = 0,
  PARAM_DEVICE_SLOT_POS_MAX = 1,
  PARAM_DEVICE_SLOT_VEL_LIMIT = 2,
  PARAM_DEVICE_SLOT_TORQUE_LIMIT = 3,
  // v3：设备侧安全事实。位图 = 1u << slot，device_raw_present 是 u8（上限 8 槽）。
  PARAM_DEVICE_SLOT_SAFETY = 4,    // 0x2650:01（0=看门狗+限位保护开，1=关）
  PARAM_DEVICE_SLOT_DIRECTION = 5, // 0x2541（0=CW，1=CCW）
  PARAM_DEVICE_SLOT_COUNT = 6
} param_device_slot_t;
```

`lib/motor_protocol/include/motor_protocol_adapter.h`：

```c
#define MOTOR_PROTOCOL_PARAM_SLOT_NONE 0xFFU
#define MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN 0U
#define MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX 1U
#define MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX 2U
#define MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX 3U
// v3：设备侧安全事实（无 SI 换算，原值透传）。
#define MOTOR_PROTOCOL_PARAM_SLOT_SAFETY 4U    // 0x2650:01
#define MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION 5U // 0x2541
#define MOTOR_PROTOCOL_PARAM_SLOT_COUNT 6U
```

`src/storage_projection.c` 在现有四条之后补：

```c
static_assert(PARAM_DEVICE_SLOT_SAFETY == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
static_assert(PARAM_DEVICE_SLOT_DIRECTION == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
```

- [ ] **Step 4: 跑全部 host 测试**

```bash
cmake --build .host-build -j2 2>&1 | tail -5 && ctest --test-dir .host-build --output-on-failure
```
Expected: 全部 PASS（测试数不变，本任务只加用例）。

> 若 `test_motor_control` 或 `test_motor_protocol_adapter` 因数组变大而断言失败，
> 先看是不是某处硬编码了 4 个槽位——那是本任务**应当**暴露的东西，就地改成按
> `MOTOR_PROTOCOL_PARAM_SLOT_COUNT` 迭代。

- [ ] **Step 5: 提交**

```bash
git add lib/param_store/include/param_name.h lib/motor_protocol/include/motor_protocol_adapter.h src/storage_projection.c test/host/test_param_store.cpp
git commit -m "feat(fw): 设备槽位扩到 6，新增安全开关(0x2650:01)与方向(0x2541)"
```

---

### Task 3: v2 → v3 迁移与链式迁移

**Files:**
- Modify: `lib/param_store/include/param_store_blob.h`（声明 `param_store_blob_migrate_v2`）
- Modify: `lib/param_store/src/param_store_blob.c`（v2 镜像结构 + CRC + 迁移实现）
- Test: `test/host/test_param_store.cpp`

**Interfaces:**
- Consumes: Task 1/2 的常量、`param_store_record_t`、`param_store_blob_reset()`、`param_store_blob_finalize()`
- Produces: `param_store_blob_t *param_store_blob_migrate_v2(void *v2_src, size_t src_size, void *out, size_t out_size)`
  —— 校验 v2 头 + v2 CRC 后逐字段搬到 v3；失败返回 `NULL`

- [ ] **Step 1: 写失败测试**

在 `test/host/test_param_store.cpp` 的 `test_blob_migrate_v1` 之后插入：

```cpp
// 手搓一个合法 v2 blob（48B 记录）供迁移测试。
static void build_v2_blob(uint8_t *buf) {
  std::memset(buf, 0, PARAM_STORE_BLOB_SIZE_V2);
  auto *hdr = reinterpret_cast<uint32_t *>(buf);
  hdr[0] = PARAM_STORE_MAGIC;
  hdr[1] = PARAM_STORE_VERSION_V2 | (PARAM_STORE_HEADER_SIZE << 16);
  hdr[2] = 9U; // seq
  hdr[3] = PARAM_STORE_RECORD_COUNT;
  const size_t rec = PARAM_STORE_HEADER_SIZE;
  auto *p32 = reinterpret_cast<uint32_t *>(buf + rec);
  p32[0] = (1U << PARAM_FIELD_KP) | (1U << PARAM_FIELD_KD) |
           (1U << PARAM_FIELD_POS_MIN) | (1U << PARAM_FIELD_POS_MAX) |
           (1U << PARAM_FIELD_VEL_LIMIT) | (1U << PARAM_FIELD_TORQUE_LIMIT);
  auto *p16 = reinterpret_cast<uint16_t *>(buf + rec);
  p16[2] = 5U;                        // motor_id
  buf[rec + 6] = (uint8_t)((1U << PARAM_DEVICE_SLOT_POS_MIN) |
                           (1U << PARAM_DEVICE_SLOT_TORQUE_LIMIT)); // device_raw_present
  auto *pf = reinterpret_cast<float *>(buf + rec);
  pf[2] = 30.0f;  // kp
  pf[3] = 1.0f;   // kd
  pf[4] = -3.14f; // pos_min
  pf[5] = 3.14f;  // pos_max
  pf[6] = 3.0f;   // vel_limit
  pf[7] = 45.0f;  // torque_limit
  // v2 里 device_raw 在偏移 32，共 4 槽。
  auto *praw = reinterpret_cast<uint32_t *>(buf + rec + 32);
  praw[PARAM_DEVICE_SLOT_POS_MIN] = (uint32_t)(int32_t)-32767;
  praw[PARAM_DEVICE_SLOT_TORQUE_LIMIT] = 10800U;
  // v2 CRC：头（crc 字段清零）+ 64×48B 记录。
  auto crc32 = [](uint32_t crc, const uint8_t *d, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
      crc ^= d[i];
      for (int b = 0; b < 8; ++b) {
        crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
      }
    }
    return ~crc;
  };
  uint8_t header_copy[PARAM_STORE_HEADER_SIZE];
  std::memcpy(header_copy, buf, PARAM_STORE_HEADER_SIZE);
  std::memset(header_copy + 16, 0, sizeof(uint32_t));
  uint32_t crc = crc32(0U, header_copy, PARAM_STORE_HEADER_SIZE);
  crc = crc32(crc, buf + PARAM_STORE_HEADER_SIZE,
              PARAM_STORE_RECORD_COUNT * PARAM_STORE_RECORD_SIZE_V2);
  hdr[4] = crc;
}

void test_blob_migrate_v2() {
  static uint8_t v2[PARAM_STORE_BLOB_SIZE_V2];
  build_v2_blob(v2);
  static uint8_t out[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob =
      param_store_blob_migrate_v2(v2, sizeof(v2), out, sizeof(out));
  assert(blob != nullptr);
  assert(blob->version == PARAM_STORE_VERSION && blob->seq == 9U);
  assert(param_store_blob_validate(blob));
  // 用户数据一字不丢。
  float v = 0.0f;
  assert(param_store_blob_get(blob, 5, PARAM_FIELD_KP, &v) && v == 30.0f);
  assert(param_store_blob_get(blob, 5, PARAM_FIELD_TORQUE_LIMIT, &v) && v == 45.0f);
  assert(param_store_blob_get(blob, 5, PARAM_FIELD_VEL_LIMIT, &v) && v == 3.0f);
  // 四个旧设备槽原值保留，新两个槽是"未核验"。
  uint32_t raw = 0U;
  assert(param_store_blob_get_device_raw(blob, 5, PARAM_DEVICE_SLOT_POS_MIN, &raw));
  assert(raw == (uint32_t)(int32_t)-32767);
  assert(param_store_blob_get_device_raw(blob, 5, PARAM_DEVICE_SLOT_TORQUE_LIMIT, &raw));
  assert(raw == 10800U);
  assert(!param_store_blob_get_device_raw(blob, 5, PARAM_DEVICE_SLOT_SAFETY, &raw));
  assert(!param_store_blob_get_device_raw(blob, 5, PARAM_DEVICE_SLOT_DIRECTION, &raw));
  // v3 新字段：型号身份与 MIT 标度一律为"未知"。
  assert(blob->records[0].model_name[0] == '\0');
  assert(blob->records[0].identity_hash == 0U);
  assert(blob->records[0].mit_torque_limit_nm == 0.0f);
  assert(blob->records[0].profile_version == 0U);
  // reserved 尾必须全零（后续项靠"全零 = 未使用"识别）。
  for (size_t i = 0; i < sizeof(blob->records[0].reserved); ++i) {
    assert(blob->records[0].reserved[i] == 0U);
  }
  // CRC 损坏 → 拒绝迁移。
  auto *hdr = reinterpret_cast<uint32_t *>(v2);
  hdr[4] ^= 0xFFU;
  assert(param_store_blob_migrate_v2(v2, sizeof(v2), out, sizeof(out)) == nullptr);
}

// v1 → v3 必须经 v2 中转（链式），且 v1 的设备端原值位图仍为 0。
void test_blob_migrate_chain_v1_to_v3() {
  static uint8_t legacy[PARAM_STORE_LEGACY_BLOB_SIZE];
  std::memset(legacy, 0, sizeof(legacy));
  auto *hdr = reinterpret_cast<uint32_t *>(legacy);
  hdr[0] = PARAM_STORE_MAGIC;
  hdr[1] = PARAM_STORE_VERSION_LEGACY | (PARAM_STORE_HEADER_SIZE << 16);
  hdr[2] = 3U;
  hdr[3] = PARAM_STORE_RECORD_COUNT;
  const size_t rec = PARAM_STORE_HEADER_SIZE;
  auto *p32 = reinterpret_cast<uint32_t *>(legacy + rec);
  p32[0] = (1U << PARAM_FIELD_KP);
  auto *p16 = reinterpret_cast<uint16_t *>(legacy + rec);
  p16[2] = 4U;
  auto *pf = reinterpret_cast<float *>(legacy + rec);
  pf[2] = 7.5f; // kp
  auto crc32 = [](uint32_t crc, const uint8_t *d, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
      crc ^= d[i];
      for (int b = 0; b < 8; ++b) {
        crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
      }
    }
    return ~crc;
  };
  uint8_t hc[PARAM_STORE_HEADER_SIZE];
  std::memcpy(hc, legacy, PARAM_STORE_HEADER_SIZE);
  std::memset(hc + 16, 0, sizeof(uint32_t));
  uint32_t crc = crc32(0U, hc, PARAM_STORE_HEADER_SIZE);
  crc = crc32(crc, legacy + PARAM_STORE_HEADER_SIZE,
              PARAM_STORE_RECORD_COUNT * PARAM_STORE_LEGACY_RECORD_SIZE);
  hdr[4] = crc;

  static uint8_t mid[PARAM_STORE_BLOB_SIZE_V2];
  assert(param_store_blob_migrate_v1(legacy, sizeof(legacy), mid, sizeof(mid)) != nullptr);
  static uint8_t out[PARAM_STORE_BLOB_SIZE];
  param_store_blob_t *blob =
      param_store_blob_migrate_v2(mid, sizeof(mid), out, sizeof(out));
  assert(blob != nullptr && param_store_blob_validate(blob));
  float v = 0.0f;
  assert(param_store_blob_get(blob, 4, PARAM_FIELD_KP, &v) && v == 7.5f);
  assert(param_store_blob_device_raw_present(blob, 4) == 0U);
}
```

在 `main()` 里加 `test_blob_migrate_v2();` 与 `test_blob_migrate_chain_v1_to_v3();`。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -20
```
Expected: 编译失败，`'param_store_blob_migrate_v2' was not declared`。

- [ ] **Step 3: 实现迁移**

`lib/param_store/include/param_store_blob.h` 在 `param_store_blob_migrate_v1` 声明后追加：

```c
// v2（48B 记录、device_raw 4 槽）→ v3。校验 v2 头与 CRC，逐字段搬运；
// device_raw[4..7] 清零（新增的安全/方向槽为"未核验"，与 v1→v2 时
// device_raw_present = 0 同口径）；型号身份与 MIT 标度一律写 0 = 未知。
// 成功返回 out，失败返回 NULL（调用方按"无有效 blob"处理）。
param_store_blob_t *param_store_blob_migrate_v2(void *v2_src, size_t src_size,
                                                void *out, size_t out_size);
```

`lib/param_store/src/param_store_blob.c` 在 `param_store_blob_migrate_v1` 之后追加：

```c
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
  uint8_t header_copy[PARAM_STORE_HEADER_SIZE];
  memcpy(header_copy, blob, PARAM_STORE_HEADER_SIZE);
  memset(header_copy + 16, 0, sizeof(uint32_t));
  uint32_t crc = crc32_update(0U, header_copy, PARAM_STORE_HEADER_SIZE);
  return crc32_update(crc, (const uint8_t *)blob->records,
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
    return NULL;
  }
  param_store_blob_t *dst = param_store_blob_reset(out, out_size);
  if (dst == NULL) {
    return NULL;
  }
  for (uint32_t i = 0; i < PARAM_STORE_RECORD_COUNT; ++i) {
    const param_store_record_v2_t *from = &src->records[i];
    if (from->present == 0U && from->device_raw_present == 0U) {
      continue;
    }
    param_store_record_t *to = &dst->records[i];
    to->present = from->present;
    to->motor_id = from->motor_id;
    to->device_raw_present = from->device_raw_present;
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
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build .host-build -j2 2>&1 | tail -5 && ctest --test-dir .host-build --output-on-failure -R test_param_store
```
Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add lib/param_store/include/param_store_blob.h lib/param_store/src/param_store_blob.c test/host/test_param_store.cpp
git commit -m "feat(fw): blob v2→v3 迁移，形成 v1→v2→v3 链"
```

---

### Task 4: cerb adapter 新增两个只读阶段

`kStagesRefresh` 从 4 阶段扩到 6，两个新阶段的 SI 值就是原值本身（无换算）。

**Files:**
- Modify: `lib/motor_protocol/src/cerb_adapter.c`（对象宏 ~:51-67、`CERB_VALUE_*` 枚举、
  `kStagesRefresh`、`cerb_refresh_raw_to_si`）
- Test: `test/host/test_motor_protocol_adapter.cpp`

**Interfaces:**
- Consumes: Task 2 的 `MOTOR_PROTOCOL_PARAM_SLOT_SAFETY` / `_DIRECTION`
- Produces: `kStagesRefresh[6]`，末两个阶段的 `param_slot` 为 4 / 5，
  读回时 `motor_protocol_param_stage_t.value_si == (float)raw`

- [ ] **Step 1: 写失败测试**

在 `test/host/test_motor_protocol_adapter.cpp` 末尾（`} // namespace` 之前）加：

```cpp
// REFRESH 回读必须覆盖新建的两个安全槽位，且原值透传（无 SI 换算）。
void test_cerb_refresh_covers_safety_slots() {
  const motor_protocol_model_profile_t *profile = find_profile("cerb_r58_101");
  assert(profile != nullptr);
  motor_protocol_config_request_t request = {};
  request.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  uint8_t count = 0U;
  const cerb_stage_desc_t *table = cerb_stage_table(&request, &count);
  assert(table != nullptr && count == 6U);
  assert(table[4].action == CERB_STAGE_SDO_READ);
  assert(table[4].index == 0x2650U && table[4].sub == 0x01U);
  assert(table[4].param_slot == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  assert(table[5].action == CERB_STAGE_SDO_READ);
  assert(table[5].index == 0x2541U && table[5].sub == 0x00U);
  assert(table[5].param_slot == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  // 原值透传：raw=1 → value_si=1.0（不做任何单位换算）。
  float si = 0.0f;
  assert(cerb_refresh_raw_to_si(MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, 1U, profile, &si));
  assert(si == 1.0f);
  assert(cerb_refresh_raw_to_si(MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, 0U, profile, &si));
  assert(si == 0.0f);
  // 未知槽位仍拒绝。
  assert(!cerb_refresh_raw_to_si(MOTOR_PROTOCOL_PARAM_SLOT_COUNT, 1U, profile, &si));
}
```

在 `main()` 里加 `test_cerb_refresh_covers_safety_slots();`。

> 注：`cerb_stage_table` 与 `cerb_refresh_raw_to_si` 目前是 `static`，测试文件看不到。
> 该文件已有**直测 adapter 内部函数**的先例（`test_motor_protocol_adapter.cpp:346`
> 就用了 `profile->mit_position_limit_rad` 等 profile 内部字段），所以按该文件既有的
> 包含方式来即可——若既有做法是 `#include` 源文件或另设测试专用导出，就沿用；
> **不要**为了测试把这两个函数改成非 static。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -20
```
Expected: 断言失败（`count == 6U` 不成立，当前是 4）或编译失败（宏未定义）。

- [ ] **Step 3: 实现**

`lib/motor_protocol/src/cerb_adapter.c` 在 `CERB_OBJ_SAVE_FLASH` 定义之后加：

```c
// 【历史片段·勘误】子索引实为 0x00，且开启/读法已确认（见 cerb_adapter.c）。
// 0x2650:01「关闭看门狗和限位保护」（厂家协议说明书条目；手册只给了写命令
// 23 50 26 00 01 …）：写 1 = 关闭看门狗 + 限位保护，0 = 保护开。这是设备侧
// 唯一的独立安全超时（帧间隔 >500ms 电机自锁），且关掉会连限位保护一起失效
// ——CERB 型号的 capability_flags 是 CAP_LIMIT_DEVICE_SIDE，固件没有软限位兜底。
// 读子索引 0x01 未经台架实测，见 validation plan。
#define CERB_OBJ_SAFETY_PROTECT 0x2650U
#define CERB_SUB_SAFETY_PROTECT 0x01U
// 0x2541「电机方向定义」：0=CW，1=CCW。方向一翻，记录里所有 cnt 限位与零点的
// 含义整体镜像。
#define CERB_OBJ_MOTION_DIR 0x2541U
```

在 `CERB_VALUE_*` 枚举里追加：

```c
  CERB_VALUE_PASSTHROUGH, // 读：原值即 SI（安全开关/方向，无单位换算）
```

修改 `kStagesRefresh`：

```c
static const cerb_stage_desc_t kStagesRefresh[6] = {
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
  CERB_DESC_READ(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
  CERB_DESC_READ(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
  // v3：设备侧安全事实。读回即比对基准，写侧不提供（恢复靠外部工具）。
  CERB_DESC_READ(CERB_OBJ_SAFETY_PROTECT, CERB_SUB_SAFETY_PROTECT,
                 MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, CERB_VALUE_PASSTHROUGH),
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_PASSTHROUGH),
};
```

`cerb_refresh_raw_to_si` 的 `switch (slot)` 加两个 case：

```c
    case MOTOR_PROTOCOL_PARAM_SLOT_SAFETY:
    case MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION:
      // 原值透传：这两个是枚举/标志位，没有工程单位可换算。
      // value_si 仅供页面显示，裁决一律用 raw。
      *out = (float)raw;
      return true;
```

- [ ] **Step 4: 扩测试夹具的「4 个设备对象」硬编码**

`test/host/test_motor_control.cpp` 里有一组按顺序写死 4 个设备的夹具，REFRESH 变 6 阶段后
**必须同步扩到 6**，否则既有的 `feed_device_params()` 流程会错位：

```cpp
// :298-299 扩到 6（顺序必须与 kStagesRefresh 一致）
static const uint16_t kDevIndex[6] = {0x607DU, 0x607DU, 0x6080U, 0x2538U,
                                      0x2650U, 0x2541U};
static const uint8_t kDevSub[6] = {1U, 2U, 0U, 0U, 1U, 0U};
// :426 扩到 6：前 4 个不变，新增项喂"保护开 + 方向 CW"（出厂默认）
static const uint32_t kR58Raw[6] = {0xFFFF8000U, 32768U, 3030U, kR58PeakMa,
                                    0U, 0U};
```

同文件里按 `4U` / `4` 迭代这些数组的地方（`feed_device_params`、`make_device_record`、
`sdo_read_ack` 的阶段判定）一并改为按 `MOTOR_PROTOCOL_PARAM_SLOT_COUNT` 迭代。
`make_device_record()` 造记录时把两个新槽也填进 `device_raw[]` 与 `device_raw_present`
（值取 `kR58Raw[4]` / `kR58Raw[5]`），否则默认路径会因"记录里没有这两个槽"而跳过比对。

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build .host-build -j2 2>&1 | tail -20 && ctest --test-dir .host-build --output-on-failure
```
Expected: 全部 PASS。若 `test_motor_control` 出现 `dev_init` 相关断言失败，多半是
Step 4 的某个 `4U` 没改干净——按 `MOTOR_PROTOCOL_PARAM_SLOT_COUNT` 全局搜一遍。

- [ ] **Step 6: 提交**

```bash
git add lib/motor_protocol/src/cerb_adapter.c test/host/test_motor_protocol_adapter.cpp test/host/test_motor_control.cpp
git commit -m "feat(fw): cerb REFRESH 回读纳入 0x2650:01 保护开关与 0x2541 方向"
```

---

### Task 5: 安全语义落定（比对位图 + 事件注释）

Task 2 扩了 `PARAM_SLOT_COUNT` 之后，`motor_control_config.c:86-92` 的比对循环与
`:201` 的写登记循环会自动覆盖新槽位——**本任务主要是把它钉住**，并修消息注释。

**Files:**
- Modify: `micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg`（`EVENT_DEV_INIT` 注释）
- Test: `test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 2 的槽位枚举、Task 4 的 REFRESH 六阶段
- Produces: `motor_control_entry_t.dev_raw[4]`/`[5]` 参与 `dev_init` 比对，
  不一致位图 `1u << slot` 含 bit4 / bit5；不一致时该台 `MOTOR_CONTROL_DEV_INIT_MISMATCH`

- [ ] **Step 1: 写失败测试**

**不要新造夹具**——该文件已有一整套可用助手。按它的既有 device-param 用例改一个变体：
把 `bring_cerb_to_ready(ctx, id)` 换成 `bring_cerb_to_bound(ctx, id)`（停在已绑定、
待回读的状态），在 `feed_device_params()` 之前，把**记录**里的安全槽设为 0，
而让**设备侧第 4 阶段回读**返回 1：

```cpp
// 安全开关不一致 → MISMATCH（该台不可使能）；位图须指出是 bit4。
// 夹具复用既有的：bring_cerb_to_bound / feed_device_params / view_of / kR58Raw。
void test_dev_init_flags_safety_mismatch() {
  // 1) 与既有 device-param 用例同法建上下文并发现绑定该台，然后：
  bring_cerb_to_bound(&ctx, id);
  // 2) 记录：安全槽 = 0（保护开）、方向槽 = 0（CW）——即 kR58Raw[4]/[5] 的值。
  motor_control_stored_params_t rec = make_device_record(id);
  // make_device_record 已按 kR58Raw 填满 6 槽（Task 4 Step 4）。
  motor_control_apply_stored_params(&ctx, &rec, 1U);
  // 3) 设备回读：把第 4 阶段（安全开关）的应答值改成 1 = 保护被关。
  const uint32_t saved = kR58Raw[4];
  const_cast<uint32_t *>(kR58Raw)[4] = 1U;
  feed_device_params(&ctx, id);
  const_cast<uint32_t *>(kR58Raw)[4] = saved;
  // 4) 断言：不一致 → MISMATCH，且位图指出 bit4。
  const motor_control_param_view_t *v = view_of(&ctx, id);
  assert(v != nullptr);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert((v->dev_raw_present & (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY)) != 0U);
  // 5) 一致场景（记录与设备都是 0）→ OK。另起一个上下文同法跑到 ready 之后：
  //    bring_cerb_to_ready(&ctx2, id) 内部就走完了"记录与 kR58Raw[4]=0 一致"的路径，
  //    故直接断言既有断言即可——若既有用例已覆盖 OK 分支，本步不必重复。
}
```

在 `main()` 里加 `test_dev_init_flags_safety_mismatch();`。

> `kR58Raw` 目前是 `static const uint32_t kR58Raw[4]`。Task 4 Step 4 已把它改成 `[6]`；
> 若该文件里它是 `const` 而非可变，就改成 `static uint32_t kR58Raw[6]`（本步要临时改值）。
> `ctx` 的构造方式沿用该文件既有用例（本文件用静态上下文 + `bind_real_adapters()`）。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -20
```
Expected: 若 Task 2 已让比对循环自动覆盖，此测试可能**直接通过**——那也是有效结论
（说明 D3 的复用成立）。若失败，看 `motor_control_config.c:86-92` 是否有硬编码的
槽位范围。

- [ ] **Step 3: 修消息注释**

`micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg` 中 `EVENT_DEV_INIT` 那段注释，
把位图说明改为：

```
#   request_id 携带"设备参数不一致槽位位图"
#   （1u<<slot：1=位置下限 2=位置上限 4=最大转速 8=峰值电流
#     16=保护开关(0x2650:01) 32=方向(0x2541)；0=无此项信息）。
```

- [ ] **Step 4: 跑全部 host 测试**

```bash
cmake --build .host-build -j2 2>&1 | tail -5 && ctest --test-dir .host-build --output-on-failure
```
Expected: 全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add test/host/test_motor_control.cpp micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg
git commit -m "feat(fw): 安全开关/方向纳入上电比对位图，同步 MotorEvent 注释"
```

---

### Task 6: storage_task 双扇区槽区与单缓冲 boot

**这一步依赖 HAL/OSPI，host 测不了——先保证 `pio run` 通过，台架验证在 Task 8。**

**Files:**
- Modify: `src/storage_task.cpp`

**Interfaces:**
- Consumes: Task 1/3 的 `PARAM_STORE_BLOB_SIZE`（7712）与 `param_store_blob_migrate_v2`
- Produces: `slot_base(slot)`、`g_active_slot`（0/1）、`read_slot()`、`load_boot_params()`、
  `perform_save()`；对外接口 `storage_boot_params()` / `storage_task_request_save()` 不变

- [ ] **Step 1: 改槽区常量与命名**

`src/storage_task.cpp:26-27,37,52-54` 替换为：

```c
// 双槽 A/B，每槽 2 个 4KB 扇区（W25Q64 最小擦除单位），各放一个 blob：
// v3 为 7712B = 32B 头 + 64×120B 记录。save 总写非活动槽，boot 选 CRC 有效且
// seq 最大者——erase/write 中掉电至少保留一个好槽（擦 A 的扇区不碰 B）。
static const uint32_t kSlotBase[2] = {0x000000U, 0x00002000U};
static const uint32_t kSlotSectors = 2U;

static uint32_t slot_base(uint8_t slot) { return kSlotBase[slot & 1U]; }
```

`g_active_sector` 全部改名为 `g_active_slot`（注释改「0=A 1=B（无有效 blob 默认 A）」）。
`(uint8_t)(g_active_sector ^ 1U)` 改为 `(uint8_t)(g_active_slot ^ 1U)`。

- [ ] **Step 2: 单缓冲 scratch 与断言**

替换 `:48-49` 的两个大缓冲与 `:78-79` 的两个槽缓冲，改为一 **块** scratch：

```c
// 唯一一块整 blob 暂存：boot 阶段（任务创建前）读入胜者，save 阶段打包写出。
// 两者无时间重叠——load_boot_params() 只在 storage_task_start() 内调用一次，
// 且发生在 xTaskCreate 之前，其入口有 configASSERT(g_task == nullptr) 把关。
// 必须 8 字节对齐：上层按 param_store_blob_t 结构体访问，-Os 会用 STRD，
// 非对齐在 M7 上直接 UNDEFINSTR。
alignas(8) static uint8_t g_blob_scratch[PARAM_STORE_BLOB_SIZE];
// 读回核验按页做：不需要 7712B 的第二份全量缓冲。
alignas(8) static uint8_t g_verify_page[W25Q64_PAGE_SIZE];
```

- [ ] **Step 3: boot 顺序整读 + 必要时重读赢家**

`read_sector()` 改名 `read_slot()`，签名不变（`uint8_t slot, uint8_t *buf`），
内部 `sector_address(sector)` 改为 `slot_base(slot)`，并按 v3 → v2 → v1 顺序尝试：

```c
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
```

> `configASSERT` 需要 `<STM32FreeRTOS.h>`（本文件已 include）。

- [ ] **Step 4: save 双扇区擦写 + 页校验**

`perform_save()` 的 `:126-148` 改为：

```c
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
```

- [ ] **Step 5: 编译并跑全部 host 测试**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run 2>&1 | tail -12
cmake --build .host-build -j2 2>&1 | tail -3 && ctest --test-dir .host-build --output-on-failure
```
Expected: `pio run` SUCCESS（记录 RAM/Flash 数字，RAM_D1 口径比 v2 约 +1.2 KB）；
host 测试全绿。**不要跑 `-t clean`**（未改 `platformio.ini` 宏与链接脚本）。

- [ ] **Step 6: 提交**

```bash
git add src/storage_task.cpp
git commit -m "feat(fw): 参数槽区改每槽 2 扇区，boot 单缓冲顺序整读，save 双扇区擦写+页校验"
```

---

### Task 7: 台架模拟器补三个对象

**Files:**
- Modify: `micro_ros_agent/tools/fdcan_motor_sim.py`

**Interfaces:**
- Consumes: 无（独立主机脚本）
- Produces: 模拟器可应答 SDO 读 `0x2650:01`（默认 0 = 保护开）、`0x2541`（默认 0 = CW）、
  `0x6064`（负载端位置当前值）；并提供 `--safety-off <id>` 与 `--dir-ccw <id>` 开关

- [ ] **Step 1: 加对象表**

在 `fdcan_motor_sim.py` 的 CERB 节点 SDO 读应答逻辑附近补上：

```python
# CERB 可读对象（SDO 快速上传，40 idx_lo idx_hi sub …）。
# 0x2650:01 保护开关：0 = 看门狗 + 限位保护开（出厂默认），1 = 关。
# 0x2541 方向：0 = CW，1 = CCW。
# 0x6064 负载端位置当前值：设零后应为 0（设零生效核验用）。
CERB_READABLE = {
    (0x607D, 0x01): "pos_lower",
    (0x607D, 0x02): "pos_upper",
    (0x6080, 0x00): "max_speed_rpm",
    (0x2538, 0x00): "max_current_ma",
    (0x2650, 0x01): "safety_protect",
    (0x2541, 0x00): "motion_dir",
    (0x6064, 0x00): "load_position_cnt",
}
```

节点状态里初始化 `safety_protect = 0`、`motion_dir = 0`、`load_position_cnt = 0`。

- [ ] **Step 2: 加命令行开关**

在 argparse 段加：

```python
parser.add_argument("--safety-off", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2650:01 置 1（关闭看门狗+限位保护），"
                         "用于制造与固件记录不一致的场景")
parser.add_argument("--dir-ccw", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2541 置 1（CCW），用于制造方向不一致")
```

启动时按这两个列表设置对应节点的字段。

- [ ] **Step 3: 自检**

```bash
cd /home/ly/robot_ws/micro_ros_agent
python3 -c "import ast; ast.parse(open('tools/fdcan_motor_sim.py').read()); print('syntax ok')"
python3 tools/fdcan_motor_sim.py --help | head -20
```
Expected: `syntax ok`，`--help` 列出两个新开关。

- [ ] **Step 4: 提交**

```bash
git add micro_ros_agent/tools/fdcan_motor_sim.py
git commit -m "test(bench): 模拟器补 0x2650:01/0x2541/0x6064 与不一致场景开关"
```

---

### Task 8: 台架验证与文档更新

**前置**（顺序不能反，见 `docs/guides/fdcan1-bench-guide.md`）：CANable **先成为 ACK 节点**
（`C`→`S8`→`Y5`→`O`，间隔 ≥150ms）再复位/烧录控制板；**烧录前先停模拟器**。
CANable 与控制板 CDC 的 `ttyACM` 编号会交换，**一律按 `/dev/serial/by-id/` 区分**。

**Files:**
- Modify: `micro_ros_firmware/README.md`
- Modify: `micro_ros_firmware/AGENTS.md`
- Modify: `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md`
- Modify: `micro_ros_firmware/docs/plans/active/motor-runtime-validation.md`
- Create: `micro_ros_firmware/docs/plans/active/cerb-vendor-questions.md`
- Create: `micro_ros_agent/tools/cerb_fullscale_probe.py`

- [ ] **Step 1: 烧录并建链**

```bash
# 1) 确认模拟器已停
pkill -f fdcan_motor_sim.py || true
# 2) 烧录（ST-Link swd，已免 sudo）
cd /home/ly/robot_ws/micro_ros_firmware && ~/.platformio/penv/bin/pio run -t upload
# 3) 起模拟器（CANable 必须先成为 ACK 节点，脚本自动做 C→S8→Y5→O）
cd /home/ly/robot_ws/micro_ros_agent
python3 tools/fdcan_motor_sim.py --dev /dev/serial/by-id/<CANable>
# 4) 起 agent（板子 CDC，同样按 by-id 确认）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/<板子 CDC> -v
```

- [ ] **Step 2: 验证 v3 落盘与重载**

```bash
ros2 param set /micro_ros_node m5.vel_limit 20.0
ros2 param set /micro_ros_node m5.torque_limit 5.0
ros2 param set /micro_ros_node system.save 1
# 观察落盘耗时（v3 擦 2 个扇区）——用作 README 里"~1s"的替换值
ros2 param get /micro_ros_node system.heap_free
# 断电重启控制板，再读回：值必须还在
ros2 param get /micro_ros_node m5.vel_limit
```
Expected: 重启后 `m5.vel_limit == 20.0`。

- [ ] **Step 3: 验证安全开关不一致 → 拒绝使能**

```bash
# 停模拟器，用 --safety-off 5 重启，使设备侧 0x2650:01 = 1
pkill -f fdcan_motor_sim.py
python3 tools/fdcan_motor_sim.py --dev /dev/serial/by-id/<CANable> --safety-off 5
# 复位控制板后观察
ros2 topic echo /micro_ros/motor_events | grep -A2 DEV_INIT
ros2 param get /micro_ros_node system.last_error       # 期望 40
ros2 param get /micro_ros_node system.last_error_motor # 期望 5
```
Expected: 该台 `CONFIG_CONFLICT`、事件 `EVENT_DEV_INIT` 的 `reason` 位图含 bit4（16）；
经 `/motor_policy` 发 `enabled=true` 被拒，该台在 `motion_states` 无新值。

- [ ] **Step 4: 验证擦写中掉电**

在 `system.save 1` 之后立刻断电（拔电源或复位），重复 3 次，每次上电后：

```bash
ros2 param get /micro_ros_node system.last_error   # 期望 0
ros2 param get /micro_ros_node m5.vel_limit        # 期望 20.0 或回退到上一次成功值
```
Expected: 每次上电都能加载出一份**有效** blob（新旧任一槽），不出现两槽皆无效。

- [ ] **Step 5: 更新文档**

- `README.md`：Flash 布局段改为「v3 7712 B / 每槽 2 扇区」；RAM 数字按设计的 §5.5
  **净增约 +1.2 KB** 修正；把"擦写阻塞可达 ~1s"替换为 Step 2 的实测值；
  参数面章节补一段「设备侧保护开关（`0x2650:01`）与方向（`0x2541`）纳入上电比对，
  不一致则该台不可使能」，并**必须同时写明恢复路径缺口**（设计 §8.1）：
  `0x2650` / `0x2541` 不在参数面，所以「重跑参数写事务」改不了它们——
  恢复只能靠设备侧被改回保护态、或重连重读，否则需要 cangaroo / 厂商上位机。
  不写这一条，运维会以为能一键修复。
- `AGENTS.md` W25 段：补「blob v3 = 32 B 头 + 64×120 B；槽 A `0x0000–0x1FFF`、
  槽 B `0x2000–0x3FFF`；save 擦 2 个扇区；**回滚到旧固件会丢参数**（旧固件判两槽皆无效）」。
- `docs/architecture/motor-runtime-decision-log.md`：新增决策条（v3 布局；为什么
  `device_raw` 扩到 8 槽而不是新造 flags；为什么砍掉 header-only 捷径；为什么槽区取
  2 扇区；**零点代为什么被否**）。
- `docs/plans/active/motor-runtime-validation.md`：加开放项——`0x2650:01` 读子索引
  待实测、双扇区擦写阻塞实测、`pio run` 后的 RAM_D1 实测值。
- 新建 `docs/plans/active/cerb-vendor-questions.md`：按下面「附：厂商问题清单」逐条写入。
- 新建 `micro_ros_agent/tools/cerb_fullscale_probe.py`：按下面「附：台架探针脚本」。

- [ ] **Step 6: 提交**

```bash
git add micro_ros_firmware/README.md micro_ros_firmware/AGENTS.md \
  micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md \
  micro_ros_firmware/docs/plans/active/motor-runtime-validation.md \
  micro_ros_firmware/docs/plans/active/cerb-vendor-questions.md \
  micro_ros_agent/tools/cerb_fullscale_probe.py
git commit -m "docs(fw): blob v3 落地——布局/回滚代价/擦写实测/新增开放台架项与厂商问题"
```

---

## 附：厂商问题清单（Task 8 Step 5 新建）

1. **MIT 帧（`0x110|id`）的 `Pos Max` / `Vel Max` / `T_Max` 分别取自哪个对象或常量？**
   是否随 `0x607D` / `0x6080` / `0x2538` 的写入而改变？量纲与满量程值是多少？
   （四份文档中这三个量仅出现在 MIT 字节表内，均无定义。）
2. `0x2531` 的**读回**语义是什么？字典范围 0~2 中"2"代表什么？
   （手册"标定复位指令"与"标定指令"给了同一串 hex，疑为抄漏。）
3. `0x2650:01` 的**读**子索引是不是 `0x01`？出厂默认值是 0（保护开）还是 1？
4. 手册"读取正限位 / 读取负限位"两行的子索引与设置行相反，是否为笔误？
   （字典表明确 `:01` = 下限、`:02` = 上限。）
5. 规格书"位置环编码器 21 bit"与说明书"负载端 16 bit（65,536/圈）"如何对应？
   线上位置是否为 ±32768 cnt ↔ ±180°（182 cnt/度）？

## 附：台架探针脚本（Task 8 Step 5 新建）

只发满量程 MIT 单轴帧并读回，**不烧板子、不改设备**，用于在真电机到场后一次性
反推上表第 1 条：目标力矩置 `4095` 读 `0x6077`；目标速度置 `4095` 读 `0x606C`；
目标位置置 `65535` / `0` 读 `0x6064`。反算出的物理满量程与当时的 `0x6080` / `0x2538`
一比即见分晓。**注意**：本仓库的 `fdcan_motor_sim.py` 用 `PI/PI/45.0` 解码，
是照固件假设写的，**不能**用来回答这个问题。
