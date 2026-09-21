# W25 参数 blob v3 设计（每槽 2 扇区 + 设备侧事实）

状态：设计已定并已实现；当前资源与台架事实以
`docs/plans/active/motor-runtime-validation.md` 为准。日期：2026-09-19，实施记录
持续更新至 2026-09-22。

## 1. 背景与序列

「固件静态参数」扩容序列的第 1 项（本次）。序列全景（决策于 2026-09-19）：

| # | 项 | 依赖 |
|---|---|---|
| **8** | **blob v3 布局 + A/B 槽区扩容（本文档）** | 无 |
| 3 | 型号身份 + `identity_string` 落盘 | v3 |
| 2 | 零点：设零生效核验 | 无（已从 v3 字段中移出，见 §8.1） |
| 4 | `0x2650:00` / `0x2541` 回读落盘 | 本次已并入 |
| 5 | 跨零 / 行程拓扑 | v3 |
| 6 | 累计运动时长 | v3 |
| 7 | 温度阈值、按台额定/峰值对 | 多半可从型号目录派生 |
| 1 | MIT 全量程标度核验 | **阻塞**：需真机或厂家确认，见 §11 |

本次把第 4 项提前并入，是因为 `0x2650:00` / `0x2541` 与 §4.2 的 `device_raw` 槽位机制天然同构，
一次做完比拆两次便宜。

## 2. 现状

- 记录 `param_store_record_t` 48 B（`lib/param_store/include/param_store_blob.h:36-50`）。
- blob = 32 B 头 + 64×48 B = 3104 B，A/B 各占**一个** 4 KB 扇区（`0x0` / `0x1000`），
  见 `src/storage_task.cpp:26-27`。
- 单个 4 KB 扇区对 64 条记录的上限是 **63 B/条**，现余量仅 15 B。
- 静态缓冲（`storage_task.cpp`）：`slot_a` + `slot_b` + `g_write_buf` + `g_verify_buf`
  = 4×3104 = 12416 B，另有 `legacy_buf` 2080 B 与 `g_boot_records` 5120 B，
  合计 **19616 B**（19.16 KiB）。
- 上电比对是**逐槽位比设备原值**：`motor_control_config.c:78-92` 用
  `1u << MOTOR_PROTOCOL_PARAM_SLOT_*` 生成不一致位图。

## 3. 决策摘要

1. **D1** 单 blob v3，每槽 **2 个扇区**（A = `0x0000–0x1FFF`，B = `0x2000–0x3FFF`，共 16 KB）。
2. **D2** 记录 48 B → **120 B**（blob 7712 B），尾部留 23 B `reserved`，
   让后续项（零点 / 跨零 / 运行时长 / MIT 实测标度）**不再需要格式迁移**。
3. **D3** 设备安全开关做成**两个新 `device_raw` 槽位**，而不是新造 flags 字段——
   自动复用现有的落袋 / 位图 / 比对 / 事件 reason 通路，比对循环一字不改。
4. **D4** 不一致 → 该台 `CONFIG_CONFLICT`、拒绝使能。
5. **D5** 砍掉「只读头选胜者」的捷径（它拿未经校验的 `seq` 做裁决）。
6. **D6** `g_active_sector` 改名 `g_active_slot`（一个槽不再是单个扇区）。

## 4. 数据模型

### 4.1 v3 记录（120 B）

```c
typedef struct {
  // —— v2 已有字段，偏移 0..31 不变 ——
  uint32_t present;                 //  0  位 = 1u << param_field_t
  uint16_t motor_id;                //  4
  uint8_t  device_raw_present;      //  6  位 = 1u << param_device_slot_t
  uint8_t  reserved0;               //  7  （保持不动，偏移稳定）
  float    kp, kd;                  //  8, 12
  float    pos_min, pos_max;        // 16, 20
  float    vel_limit, torque_limit; // 24, 28
  // —— v3：设备原值槽 4 → 8（实际用 6，留 2 给后续设备对象如 0x6081 / 0x6060）——
  uint32_t device_raw[8];           // 32..63
  // —— v3 新增 ——
  char     model_name[16];          // 64..79  型号目录名，NUL 结尾；空 = 未绑定
  uint32_t identity_hash;           // 80..83  设备 0x1008 自报串的 CRC32；0 = 未核验
  float    mit_pos_limit_rad;       // 84      写记录时固件假定的 MIT 全量程
  float    mit_vel_limit_rad_s;     // 88
  float    mit_torque_limit_nm;     // 92
  uint8_t  profile_version;         // 96      型号目录版本
  uint8_t  reserved[23];            // 97..119
} param_store_record_t;             // 120 B
```

`device_raw_present` 是 `uint8_t`，**上限 8 槽**（现用 6，正好留 2）。

`model_name` 与 `identity_hash` 的分工：**匹配用哈希**（精确，不受长度影响），
`model_name` 仅作可读与调试。两者都不参与裁决——型号裁决仍走现有的
「设备自报串 → 型号目录」匹配（`cerb_adapter.c` 的 `CERB_SDO_IDENTITY_INDEX = 0x1008`）。

`mit_*` 三个标度记录的是**写入该记录时固件假定的全量程**（来源为型号目录
`motor_control_profile.c`）。它的用途是让「固件升级改了 profile → 本记录是在旧值下
写的」可被发现。值 0 表示未知（迁移来的记录）。

### 4.2 设备槽位枚举扩展

`lib/param_store/include/param_field.h:17-40` 与
`lib/motor_protocol/include/motor_protocol_adapter.h` 的 `MOTOR_PROTOCOL_PARAM_SLOT_*`
**两侧同步**：

```c
PARAM_DEVICE_SLOT_POS_MIN = 0, POS_MAX = 1, VEL_LIMIT = 2, TORQUE_LIMIT = 3,
PARAM_DEVICE_SLOT_SAFETY = 4,      // 0x2650:00（0=看门狗+限位保护开，1=关）
PARAM_DEVICE_SLOT_DIRECTION = 5,   // 0x2541（0=CW，1=CCW）
PARAM_DEVICE_SLOT_COUNT = 6
```

`src/storage_projection.c:11-17` 已有的跨库 `static_assert` 需增补两条。

## 5. Flash 布局与读写路径

```
槽 A = sector 0,1  →  0x0000..0x1FFF      blob = 32 B 头 + 64×120 B = 7712 B
槽 B = sector 2,3  →  0x2000..0x3FFF      落在 8192 B 槽内，余 480 B
                                          共 16 KB；W25Q64 其余约 8 MB 仍未用
```

### 5.1 命名（D6）

```c
static const uint32_t kSlotBase[2] = {0x000000U, 0x00002000U}; // A, B
static uint8_t g_active_slot = 0U;                             // 0=A 1=B
static uint32_t slot_base(uint8_t slot) { return kSlotBase[slot & 1U]; }
```

`target_slot = g_active_slot ^ 1U` 保持不变。（现状 `g_active_sector` 存的本就是索引
0/1，`^1` 正确；改名是因为一个槽从此跨两个扇区，"sector"已名不副实，会诱发
"把它当地址用"的错误。）

### 5.2 boot 裁决（D5）

**不做**「只读 32 B 头选胜者」的捷径——那会拿未校验的 `seq` 做裁决，头损坏时可能选错槽。
改为顺序整读、逐个校验、必要时重读赢家，**单缓冲**：

```
读 A 整块 7712 B → param_store_blob_validate()（CRC 覆盖头+记录）→ 记 ok_a / seq_a
读 B 整块 7712 B → param_store_blob_validate()                   → 记 ok_b / seq_b
胜者若是 B（最后读的那个）→ 缓冲里就是它，直接用
胜者若是 A               → 重读 A → 再 validate → 用
                           若重读失败或校验不过 → 回退用 B（B 已校验通过）
两槽皆失败               → 按现有顺序退 v2 迁移 → v1 迁移
```

`param_store_blob_validate()` 先查 magic / version / header_size / record_count，
再查 CRC，而 `blob_crc()` 覆盖 header（crc 字段置零）+ 全部记录
（`param_store_blob.c:22-30`）。因此**头损坏天然被捕获，无需新增 header CRC**。

代价：最坏 3 次整读（≈23 KB SPI）。若台架实测 boot 时间不可接受，再考虑加
header CRC 走捷径——那是优化，不是正确性。

### 5.3 save

```
target_slot = g_active_slot ^ 1;  addr = slot_base(target_slot)
erase_sector(addr); erase_sector(addr + 4096)
w25q64_write(addr, blob, 7712)   // 驱动内部已按 256 B 页切分（w25q64.c:269-298），
                                 // page_remaining 已处理末尾 32 B 残页，无需自行切分
按 256 B 页读回 memcmp            // g_verify_buf 从 3104 B 降到 256 B
g_active_slot = target_slot
```

失败语义与现状一致：**任一 erase / write / 页校验失败 → `STORAGE_SAVE_FAIL`
且不翻转 `g_active_slot`**，旧槽仍是当前槽，下次启动仍可加载。
注意两个扇区必须都擦成功后才能开始写；第一个擦成功、第二个失败时，
目标槽处于半擦状态——它本来就是非活动槽，故不影响启动，但**下次 save 会重擦**。

### 5.4 scratch 复用与断言

boot 的「胜者 blob」缓冲与 `g_write_buf` **共用一块 7712 B scratch**。成立前提：
`load_boot_params()` 只在 `storage_task_start()` 内跑一次、发生在 `xTaskCreate` 之前，
与 `perform_save()` 无时间重叠；且 `storage_projection_blob_to_boot()` 是逐字段**拷贝**
进 `g_boot_records`（`storage_projection.c:111-140`），不保留指向 scratch 的指针。

两道闸：

- `load_boot_params()` 入口 `configASSERT(g_task == nullptr);`
- scratch 变量处注明「仅 start 阶段（任务创建前）与 save 路径复用，二者无时间重叠」

### 5.5 静态占用（含槽位扩展的连带代价）

`storage_task` 自身（`src/storage_task.cpp`）**实测 `.bss` = 23168 B**。初稿的表格
只数了 blob 缓冲、漏了新增的 `v2_buf` 与 `views[64]`，故它的 15168 B 偏低：

| | 初稿 | 实测 |
|---|---|---|
| boot 缓冲 | 2×3104 | 共用 scratch |
| write / verify | 3104 + 3104 | 7712 + 256 |
| `legacy_buf` | 2080 | 2080 |
| `g_boot_records` | 5120 | 5120 |
| `v2_buf`（v2 迁移用） | 漏记 | 3104 |
| `views[64]`（perform_save） | 漏记 | 计入 `.bss` 实测 |
| 小计 | ~~15168 B~~ | **23168 B（实测 `.bss`）** |

**但 `MOTOR_PROTOCOL_PARAM_SLOT_COUNT` 4→6 会让所有按槽位数量的数组同时变大**，
每电机表项 +48 B：

| 数组 | 位置 | 每项增量 | ×64 表项 |
|---|---|---|---|
| `dev_raw[COUNT]` | `motor_control_internal.h:105` | +8 B | +512 B |
| `dev_si[COUNT]` | `:106` | +8 B | +512 B |
| `refresh_stages[COUNT]` | `:110` | +16 B | +1024 B |
| `record_raw[COUNT]` | `:117` | +8 B | +512 B |
| `tx_write_raw[COUNT]` | `:145` | +8 B | +512 B |
| `dev_raw[COUNT]` + `record_raw[COUNT]` | `motor_control.h:470,475` | +16 B | +2048 B |
| `device_raw[COUNT]` | `motor_control.h:504` | +8 B | +512 B |
| **合计** | | | **+6144 B（实测）** |

（后两行各存在两份 64 元数组：`param_bridge` 的 `g_views[64]` 与
`storage_task` `perform_save()` 的 `views[64]`；`g_boot_records[64]` 一份。
上表逐行是**设计估算**，**实测连带代价以 +6144 B 为准**。）

**历史链接基线（实测）：+4800 B RAM / +360 B Flash**（RAM_D1 从 220,308 B 到
225,108 B，即 68.7%）。该数字不代表当前分段轨迹、保存请求跟踪表和消息配置的最终
链接结果；当前 RAM_D1 余量以 validation plan 的 TBD-STORAGE-03 为准。

⚠️ 早期草案写的"扩容反而省 RAM（−4448 B）"**只算了 storage 缓冲、漏了槽位扩展
的连带代价**；其后草案的"+1184 B"也只补了一部分、仍**偏低**——**两个数都错**，
以实测 +4800 B RAM / +360 B Flash 为准。

仍选方案 A / D3 的理由**不是省 RAM**，而是：D3 让两个新事实**零改动**复用现有的
落袋 / 位图 / 比对 / 事件位图通路，省掉的是一整套并行比对逻辑与它的测试面。
几 KB RAM 换这个**复用**划算（注意：它**不是** RAM 节省）。若 RAM 后续吃紧，
退路是不扩 `PARAM_SLOT_COUNT`、给两个新事实各加一个专用字段（约 +2048 B），
代价是失去 D3 的复用。

### 5.6 已知代价

1. save 从擦 1 个扇区变擦 2 个；v2 单扇区约 1s 只是旧值，v3 双扇区阻塞时间仍需
   台架实测，列入 validation plan。
2. 静态 **实测 +4800 B RAM / +360 B Flash**（见 §5.5）。

## 6. 迁移链

- 常量：`PARAM_STORE_VERSION` 2→3；新增 `PARAM_STORE_VERSION_V2 = 2`、
  `PARAM_STORE_RECORD_SIZE_V2 = 48`、`PARAM_STORE_BLOB_SIZE_V2 = 3104`。
- 新增 `param_store_blob_migrate_v2(src, src_size, out, out_size)`：
  校验 v2 头 + v2 CRC → `reset(out)` → 逐记录搬 `present` / `motor_id` /
  `device_raw_present` / 6 个 float / `device_raw[0..3]`；
  `device_raw[4..7]` 清零（未核验，与 v1→v2 时 `device_raw_present = 0` 同口径）；
  `model_name` 清零、`identity_hash = 0`、`mit_*` 三标度写 0 → `finalize(seq 沿用)`。
- 保留现有 `param_store_blob_migrate_v1`（`param_store_blob.c:239`）。**两条路径是
  fan-in，不是 v1 → v2 → v3 链**：`migrate_v1` 自 Task 1 起就直接写 v3 blob（它写
  `param_store_blob_t`，`param_store_blob_reset()` 打 `version = 3`），新增的
  `migrate_v2` 也直接写 v3。`read_slot` 依次尝试：v3 `validate` → v2 迁移（结果再
  `validate`）→ v1 迁移（结果再 `validate`）。

**不可逆性（须写入 AGENTS.md）**：新固件 save 过一次后两个槽都是 v3，
旧固件会判两槽皆无效并从头开始——**回滚会丢参数**。v1→v2 当时同性质，属既有先例。

## 7. 设备侧新增读取

- `lib/motor_protocol/src/cerb_adapter.c` 的静态阶段描述符表加两个**只读**阶段：
  `0x2650:00`、`0x2541`，走现有总线级一问一答串行与 30 ms 超时。
- 纳入现有 `REFRESH_PARAMS` 回读事务（与 `0x607D` / `0x6080` / `0x2538` 同一轮），
  落 `PARAM_DEVICE_SLOT_SAFETY` / `PARAM_DEVICE_SLOT_DIRECTION`。
- **`0x2650` 的读子索引 = `0x00`**（曾按 `0x01` 实现，2026-09-19 真机台架纠正，见 §11）。
  手册给的写命令是 `23 50 26 00 01 …`——**子索引本来就是 0x00**，固件早期把"写命令里
  那个 `01`"当成了子索引。四台真机上读 `40 50 26 01` / `40 50 26 02` 一律回
  ABORT `0x06090011`（子索引不存在），只有 `40 50 26 00` 应答。**读错子索引的后果**：
  REFRESH 恒挂在安全槽上 → UNCONFIRMED → 三次退避后 UNVERIFIED，全部 CERB 都到不了
  `DEV_INIT_OK`、永远无法使能（`last_error = 41`）。
- Vaxor 无这些对象 → 对应 `device_raw_present` 位保持 0，不参与比对
  （现有 `device_side` 分支已如此处理）。

## 8. 安全语义

- `0x2650:00` 或 `0x2541` 与记录不一致 → 该台 `CONFIG_CONFLICT`、拒绝 `SET_ENABLE`、
  不出运动帧；复用现有 `apply_dev_init_result`（`motor_control_config.c:105`）。
  依据：两个 CERB 型号的 `capability_flags` 都是 `CAP_LIMIT_DEVICE_SIDE`
  （vaxor 才是 `CAP_LIMIT_SOFTWARE`），**CERB 的关节限位完全靠设备侧 `0x607D` 强制，
  固件没有软限位兜底**；`0x2650:00 = 1` 关的是"看门狗 + 限位保护"两条，
  对 CERB 而言等于该关节**没有任何限位在执行**。
- 位图自动扩展：`1u << slot` 新增 bit4 / bit5。**必须同步改
  `micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg` 的 `EVENT_DEV_INIT` 注释**
  （现为"1=位置下限 2=位置上限 4=最大转速 8=峰值电流"），否则上位机按旧注释解析。

### 8.1 恢复路径缺口（已知，本次不解决）

`0x2650` / `0x2541` **不在参数面**，所以"重跑参数写事务"这条既有恢复路径**改不了它们**。
若真有设备把限位保护关了，固件会拒绝使能，而参数面没有任何入口能打开它——
只能拿 cangaroo 或厂商上位机去改。

**处置**：本次只实现"检测 + 拒绝"，缺口写入 README 与 validation plan 当运维前置条件。
后续可加一个动作脉冲参数（如 `system.reset_safety = <电机 id>`）主动写回保护态，
与 `system.clear_fault` 同形——但那是一次独立的小改动，不并进布局变更。

**零点项（序列第 2 项）已从 v3 字段中移出**：设零会平移设备里 `0x607D` 限位的物理位置，
但**不改变任何可读对象的数值**（`0x6064` 是实时位置、`0x607D` 是 cnt 常量），
所以固件无法检测"绕过固件的设零"。原提案的"零点代"只能覆盖经固件的路径，
不是可靠的检测机制。改为做一个**真收益且不需要新字段**的改动：设零事务后读回
`0x6064`，应在容差内为 0（手册零位标定 5 步流程的第 3 步本就要求），
用于确认"设零真的生效了"——现在固件是单阶段只等写应答，写被拒也会报成功。

### 8.2 实际语义：**变更检测**，不是"保护必须在位"

比对基准是**上次成功落盘时设备里的值**，不是"保护=0"这个常量。因此这套机制
答的是"这两项自上次落盘以来变过没有"，**不是**"现在保护是不是开着"。两条必须
写明的后果（现均**已知并接受**，见下文加固方向）：

1. **先关保护、再触发任何一次保存 ⇒ 记录被重新基线化成"关"。** 保存投影
   （`storage_projection_views_to_blob`，`src/storage_projection.c:40-43`）在回读
   成功时**优先取设备值**：设备当前 `device_raw[4] = 1`（保护关）就被当成新基准
   落盘。下次上电设备与记录一致 → 该台正常使能，而按本文档自己的话，这个关节
   **没有任何限位在执行**。触发保存**不需要写参数**：`device_limit_write_allowed()`
   只查 `lifecycle != ACTIVE && != SAFE_STOP`（`motor_control_config.c:672-675`），
   `CONFIG_CONFLICT` 的台仍受理限位写事务；写成功即 `record_dirty`
   （`motor_control_config.c:198`）→ `motor_rt_task.cpp:584` 去抖 ~500 ms 落盘。
   即：这段比对是变更检测，不是"限位在位"的验证。
2. **刚由 v2 迁移上来的记录不参与比对，直到第一次保存。**
   `param_store_blob_migrate_v2`（`lib/param_store/src/param_store_blob.c:345`）只搬
   `device_raw[0..3]` 并 `device_raw_present & 0x0F`，bit4/bit5 为 0——比对循环
   （`dev_params_mismatch_mask`，`motor_control_config.c:80-96`）见到"记录没写过"
   就 `continue`。所以每块刚升级的板子都处在"这两槽没比"的状态里，直到它第一次
   保存。（台架实测正是如此：`reason 3` 且 bit4/bit5 未置位。）

**这是交给 owner 的边界，不是本次修的东西**——语义要不要收紧是 owner 的决定。
> **2026-09-21 补注**：下面两条已不再是「可选、未实现」——2026-09-20 的**上电归一**
> 在比对之前把保护恒写为「生效」，缺口由上电时序关闭，见
> [boot-normalize-design](boot-normalize-design.md)。原文保留。

可选加固方向（**仅列出，未实现**，留给 owner 拍板）：
- 「`0x2650:00 == 1` 恒判 MISMATCH」：把"保护必须在位"变成硬前置；代价是外部
  工具一次误操作就把该台永久锁死（参数面改不了，见 §8.1）。
- 「拒绝把 `SAFETY = 1` 当基线落盘」：保留变更检测语义，只阻断"关保护后再保存
  即洗白"这条路径。

## 9. 测试

`test/host/test_param_store.cpp`（host，纯 C 可直测）：

- v3 尺寸 `static_assert`（record 120 B / blob 7712 B）；
- `record_occupied` 语义、`present` 位图、`device_raw[8]` 越界拒绝；
- CRC 覆盖新增字段、`reserved` 尾置零；
- **v2 → v3 迁移**：含"v2 只有 4 槽设备原值 → 迁移后 [4..5] 为未核验"；
- **v1 → v3 迁移**（v1 直接升 v3，非链式 v1→v2→v3）。

`test/host/test_motor_control.cpp`：

- 新增两个槽位的比对、`CONFIG_CONFLICT` 冻结、位图上报。

保持不变：`PARAM_STORE_RECORD_COUNT == MOTOR_CONTROL_MAX_MOTORS` 的 `static_assert`
（`storage_projection.c:18`）。

`storage_task.cpp` 的 boot / save 路径依赖 HAL / OSPI，**无 host 覆盖**，靠台架：
双扇区擦写、擦写过程中掉电、回滚兼容性。

## 10. 文档更新

- **README**：Flash 布局段（3104 B → 7712 B、每槽 2 扇区）、RAM 数字、
  擦写阻塞时间改实测、参数面章节补"设备侧安全开关纳入上电比对"、
  补 §8.1 的恢复路径前置条件。
- **AGENTS.md**：W25 / 存储段更新（blob v3、双扇区、scratch 复用与断言、
  **回滚不可逆**）。
- **`docs/architecture/motor-runtime-decision-log.md`**：新增决策条——
  v3 布局；为什么 `device_raw` 扩到 8 槽而不是新造 flags；为什么砍掉 header-only 捷径；
  为什么槽区取 2 扇区；零点代为什么被否。
- ~~**`docs/README.md`**：开发文档索引表增补本文档；厂商参考资料段补
  R48 / R58 规格书与 CANopen 字典表~~ ——**已随本文档一并完成**。
- **厂商问题清单（新文件，见 §11）** + **台架探针脚本**（MIT 满量程反推 +
  `0x2650:00` / `0x2541` 读回，不烧板子）。

## 11. 开放问题（需外部信息）

1. **MIT 的 `Pos Max` / `Vel Max` / `T_Max` 取自哪个对象或常量？**（序列第 1 项的阻塞项）
   ——四份文档中仅出现 3 次，全在 MIT 帧字节表内，**均无定义**；
   本仓库的台架模拟器 `fdcan_motor_sim.py:147-151` 把 `PI/PI/45.0` 写死，是照固件假设写的，
   **无法用作证据**（自证循环）。
2. `0x2531` 的读回语义、以及字典范围 0~2 中"2"的含义（手册的"标定复位指令"给了同一串 hex）。
3. ~~`0x2650:01` 的读子索引与出厂默认值。~~ **2026-09-19 真机结案**：读子索引 =
   `0x00`（`:01`/`:02` 回 ABORT `0x06090011`）；四台真机回读值均为 **1（保护关）**
   ——但这是台架设备**当前**的状态，不等于出厂默认值，出厂值仍需厂商确认。
4. 手册"读取正限位 / 读取负限位"两行的子索引与设置行相反，疑为笔误
   （字典表明确 `:01` = 下限、`:02` = 上限）。
5. 规格书"位置环编码器 21 bit" vs 说明书"负载端 16 bit（65,536/圈）"
   ——前者是内部编码器，线上是 16 bit 量化（182 cnt/度）。比对基准必须用 cnt 原值。

## 12. 明确不在本次范围

- 序列第 3 / 5 / 6 / 7 项（型号身份已由本文档预留 `model_name` + `identity_hash`，
  但写入路径与目录版本的维护属第 3 项）。
- 序列第 1 项（阻塞于 §11.1）。
- 序列第 2 项的设零生效核验（独立小改动，见 §8.1）。
- 参数面覆盖台数与内核允许的 64 台之间的口子（那是 rclc 参数槽的 DTCM 预算问题，
  与 blob 无关，另议）：具体数值以 `include/param_bridge.h` 的容量注释为准——
  该值已多次变更，**不要在本文件复述数字**。
- 模拟器对 CERB 1–6 一律用 R58 的 `45.0`（对 R48 应为 11 Nm）的解码错误——另记待办。
