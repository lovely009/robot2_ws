# 上电归一（保护/方向）与方向参数设计

> **状态：设计已定（2026-09-20 owner 逐条拍板），已实现。**
>
> **⚠️ 勘误（2026-09-21，参数面退役）**：本文所有 `m<id>.*` 参数名与
> `PARAM_BRIDGE_MAX_PARAMS` 槽位算式（§7、§9 等）**已随 rclc 参数服务器整体退役失效**
> ——参数面改为 `micro_ros_do/param_set` / `system_command` + `micro_ros_up/system_status`
> 三条话题。**本文的设计结论一条未变**（上电归一时序、方向无记录不猜、`ZERO_STALE` /
> `UNPROTECTED_UNCONFIRMED` 成因、保护/方向双出口），变的只是承载通道：参数名
> `m<id>.limit_protect` → `param_set` 的 `field=4`，`m<id>.direction` → `field=5`，
> `system.save` → `system_command` 的 `COMMAND_SAVE`。**下文保留当时原文**，
> 不再逐行改写。退役决定见 `../../architecture/motor-runtime-decision-log.md` §6.9 结案块。
>
> 本文取代 [交接文档](../../../../docs/reports/2026-09-19-firmware-param-surface-handoff.md) §二
> 的两条决定（"保护只接受写 0"、"保护/方向不进参数面"），并承接其余部分。
> 涉及：固件启动状态机、设备写事务、参数面、状态话题、web、模拟器、文档。
>
> ## ⚠️ 勘误（2026-09-20 真机实测，已按实测改代码）
>
> 本文 §4 的**前提被证伪**：`0x2541`（方向）**不改位置坐标系**。真机对照实验：
> 同一物理方向转动 node1 关节两次（各约 10°），Δ 分别为 **+2394** 与 **+2573** cnt
> ——方向位从 0 改成 1 之后，计数方向**不变**；`0x607D:01/:02`（±32768 cnt）
> 全程未变；`0x6064` 在写方向前后各读两次都是同一个值。
>
> 所以：**零点与 cnt 限位都不镜像**，"翻转 → 冻结到重新设零"（`ZERO_STALE`）
> 解决的是一个不存在的问题，**已整套删除**（故障码 8 空置不复用）。翻转真正的
> 风险是驱动器自持的**位置环变成正反馈** → 使能后朝远离目标的方向冲到设备限位；
> 方向的门禁只剩"**仅未运动时受理**"。
>
> 保留正确的部分：方向是**易失**对象（上电回 0），所以上电归一按记录写回仍然必要。
> 下面 §4 的原文保留作为决策留痕，请以本勘误为准。

> 标源：**[实测]** 台架实测 / **[源码]** 源码可复算 / **[推导]** 由前两者推算 /
> **[未验]** 必须台架。所有路径相对本仓库根。

---

## 0. 结论摘要

| # | 结论 | 依据 |
|---|---|---|
| 0 | 设备两个安全对象**都易失**：`0x2650:00` 上电回 `1`（保护关），`0x2541` 上电回 `0`（CW）。固件今天不写它们 = 上电后该关节**没有任何限位在执行**，而页面/固件都不拦 | [实测] 09-19 四台；方向易失由 owner 确认 |
| 1 | 新增**上电归一阶段（0a）**，在设备参数回读比对（0b）**之前**：保护恒写 `0`、方向写 W25 记录值。归一不成功 → 走不到比对 → 走不到运动授权 | 本文 §3 |
| 2 | 方向进参数面（`m<id>.direction`，0=CW/1=CCW）**并**进状态话题（只读遥测）。参数面是控制入口，话题是遥测，两者同源（内核影子） | §4 / §6 |
| 3 | 方向**无记录不猜**：0a 不写设备，回读照跑，由比对判"基线缺失"→ `MISMATCH(bit5)` → `CONFIG_CONFLICT`；恢复 = 页面点一次方向 | §3.3 / §4 |
| 4 | **翻转即冻结**：写事务成功时若写入值 ≠ **写前**记录值 → 新成因 `ZERO_STALE(8)`、冻结到 `SET_ZERO` 成功（设备零点与 cnt 限位的物理含义已被镜像）。上电归一写的就是记录值 → 不触发 | §4 |
| 5 | 手动关保护后**仍允许运动**，但 `MotorPolicyTarget` 新增 `confirm_unprotected`，未带确认位一律拒绝；页面持续标红。**取代** 09-19 的"使能时检查必须为 0" | §5 |
| 6 | 参数面每电机 5 → 6 项，`PARAM_BRIDGE_MAX_PARAMS` 64 → 72，保住 10 台契约（堆 72×328 = 23.6 KB） | [推导] §7 |

---

## 1. 现状与问题

`motor_control_config_step` 今天的启动链是两段不变量式复检（`lib/motor_control/src/motor_control_config.c:374`）：

- **0) 身份识别** → `READ_IDENTITY` → 绑定型号
- **0b) 设备参数回读** → `REFRESH_PARAMS` 回读六槽 → 与 W25 记录逐槽比对 → `DEV_INIT` 定论

比对口径是**变更检测**：基准 = 上次成功落盘时设备里的值，不是"保护必须在位"。
于是上电后的常态是：设备把保护复位成"关"，固件只如实回读，记录里若本来没有安全槽
就什么都比不出来——**该关节在"看起来一切正常"的状态下没有任何限位**。
[实测] 台架四台上电 `0x2650:00` 全为 `1`。

两个对象都必须**每次上电重写**：

| 对象 | 上电初值 | 语义 | 保存码 |
|---|---|---|---|
| `0x2650:00` | `1` = 关闭 | `0` = 看门狗 + 限位保护生效 | `0x2539` 对它无效（官方 §4.11 码 1 只存 PI 与最大电流）[实测] |
| `0x2541:00` | `0` = CW | `0` = CW，`1` = CCW | 同形，易失 [owner 确认] |

写命令（原值直通，不做方向语义反转）：

```
读   0x2650:00  40 50 26 00 00 00 00 00  →  43 50 26 00 0x 00 00 00
开   0x2650:00  23 50 26 00 00 00 00 00
关   0x2650:00  23 50 26 00 01 00 00 00
读   0x2541:00  40 41 25 00 00 00 00 00
写   0x2541:00  23 41 25 00 0x 00 00 00     （0 = CW，1 = CCW）[owner 手工核实，未见于手册，[未验] 待台架]
```

---

## 2. 决策记录（owner 拍板，2026-09-20）

| # | 决定 |
|---|---|
| D1 | 驱动板上电默认**开**保护；关才需要手动 |
| D2 | 归一写发在**比对之前**（保护、方向都是） |
| D3 | 方向易失已确认 → 上电必须恢复方向 |
| D4 | 方向**无记录不猜**（09-19 已拍板：判 `CONFIG_CONFLICT`），沿用 |
| D5 | 翻转方向后**卡住使能直到重新设零**成功 |
| D6 | 手动关保护后**仍给运动授权**，但每次使能需二次确认，且该台在状态里持续标红 |
| D7 | 保护/方向状态**进状态话题**（只读遥测），同时保留参数面读写 |
| D8 | 参数面容量按 6 项/台重算，保住 10 台 |

**取代关系**（09-19 交接文档 §二，此前从未写进决策日志）：

| 09-19 决定 | 状态 |
|---|---|
| 上电自动写：保护写 0；方向写 W25 记录值（无记录 = 未初始化，判 `CONFIG_CONFLICT`） | **承接**，落为 §3 |
| 方向语义：驱动器自己处理限位镜像，固件记录的 cnt 限位值不用改 | **承接**（但"翻转要重设零"是本设计的补充，见 §4） |
| 保护开关只接受写 0（写 1 = 整块拒绝）；使能时检查必须为 0 | **取代**（D1/D6：允许手动关，改为二次确认 + 标红） |
| 保护/方向进 `motor_status_array`，不占参数槽；`m<id>.limit_protect` 撤掉 | **部分取代**（D7：话题要，但参数面也保留，作为写入口） |

---

## 3. 上电归一阶段（0a）

### 3.1 位置

`motor_control_config_step` 新增 **0a**，夹在"0) 身份识别"与"0b) 设备参数回读"之间，
与 0b 同形：不变量式复检、每拍至多发起一台、事务走现有总线一问一答串行。

### 3.2 触发不变量

```
used && online && adapter != NULL && profile != NULL
&& profile 具备 CAP_LIMIT_DEVICE_SIDE          // VAXOR 无设备对象，直接排除
&& !tx_active && boot_record_offered            // 记录先落袋（与 0b 同一理由）
&& !boot_normalize_done && !motion_enabled      // 防御：归一期间不可能有运动
&& lifecycle ∈ {DISCOVERED, IDENTIFYING, READY_DISABLED}
```

### 3.3 子步骤

按序推进，每台一次一个事务，各自写 + 读回核验：

| 步 | 事务 | 值 | 条件 |
|---|---|---|---|
| 1 | `SET_SAFETY_PROTECT`（已存在） | `raw = 0`（保护生效） | 每台必做（设备易失，必写） |
| 2 | `SET_DIRECTION`（新增，见 §4） | `raw = record_raw[SLOT_DIRECTION]` | 仅当 `record_raw_present` 有槽 5 **且值 ≤ 1** |

**记录无方向槽（或值非法）→ 跳过第 2 步，不写设备、不猜方向。** 该台照常进入 0b 回读，
由比对层判"方向基线缺失"（见 §3.5），页面能立刻看到设备真值 + 明确的恢复指引。
两步都核验成功 → `boot_normalize_done = true` → 0b 才对该台放行。

### 3.4 失败与重试

与回读同口径（`MOTOR_CONTROL_DEV_READ_*` 那套）：退避重试 ≤3 次、间隔 500ms，
用尽 → `DEV_INIT_UNVERIFIED` → `apply_dev_init_result` → `CONFIG_CONFLICT` + `last_error = 41`。

于是**"归一没成功"必然等于"该台不可使能"**，不需要额外门禁。

归一失败时 `refresh_slots` 仍为 0，`UNVERIFIED` 的位图因此是"六槽全未核对"——语义正确
（确实一个槽都没核验到），页面"回读不成功"的文案无需改。

新增表项字段（`lib/motor_control/src/motor_control_internal.h`）：
`boot_norm_stage`（0=待写保护 / 1=待写方向 / 2=已完成）、`boot_norm_attempts`、
`boot_norm_retry_tick_ms`、`boot_norm_request_id`。

**完成判定必须按 request_id**（`entry->tx_request_id == entry->boot_norm_request_id`），
不能只看 `tx_kind`——手动参数写用的是同一个 kind，靠 kind 判会把人工写当成归一步进。
（同 `param_bridge` 里 `g_refresh_id` / `g_set_zero_id` 的既有做法。）

`boot_norm_*` 与 `dev_read_done` 在**同一处**复位（重连 / 记录复位），
从而"故障台恢复 → 回 `READY_DISABLED` → 不变量复检自动补做归一"。

### 3.5 为什么必须在比对之前（顺序是契约）

| 顺序 | 后果 |
|---|---|
| **0a → 0b（选定）** | 回读读到的是归一后的真值；记录与它一致 = 归一成功，不一致 = 归一失败 → 语义闭环 |
| 0b → 0a | 比对先看到设备上电初值（保护关 / 方向 CW）与记录不符 → 该台被**误判** `MISMATCH` → 每次上电白报一次 `40`，中间的窗口里页面状态是错的 |
| 0a 在 0b 前但把两槽排除出比对 | 写失败无法被比对发现，必须另加"使能时检查保护为开"的门禁 → 双机制、两处真值 |

因此 **两个槽都留在比对里，不加例外**：归一成功它们自然一致，归一失败根本走不到比对。

**方向基线缺失**是唯一需要新增的比对判据，落在 `dev_params_match_record()` 一侧：
设备侧型号 + `record_raw_present` 缺槽 5（或值 > 1）→ 置 `MISMATCH` 位图 bit5。
放在比对层而非 0a 层，是为了保留回读（页面能看到设备真值）并复用现成恢复路径：
写回方向 → 写事务成功 → 重判 → `DEV_INIT_OK`。

### 3.6 顺带修掉的 Flash 白擦

`update_record_after_write()`（`motor_control_config.c:179`）现在无条件置 `ctx->record_dirty = true`。
上电归一每次都会写保护，于是**每次上电都白擦两个扇区**（阻塞时间：v2 单扇区 [实测] ~1s；
v3 双扇区 [未验]——ATmega 那条注记还挂在 `AGENTS.md` 里）[实测]/[未验]。
改为幂等：只有写入值与既有记录值不同（或 `record_raw_present` 无该位）才置脏。
改完的落盘时机只剩三种：真的改了值、首次建立基线、手动 `system.save`。

---

## 4. 方向参数与"翻转即冻结"

### 4.1 参数与协议

- 参数名 `m<id>.direction`（DOUBLE，`0 = CW`，`1 = CCW`），**原值直通、不做语义反转**。
  ⚠️ 不要因为保护开关有"1 = 关"的反向陷阱就给方向也加对称性处理——方向没有陷阱，
  多加一层反转只会在两个面上制造第二处反转点。
- `param_value_ok()` 增 DIRECTION 分支：只接受 0/1（布尔量，不做钳制），与 `limit_protect` 同口径。
- 新增 kind `SET_DIRECTION`，贯穿四处：
  - `lib/motor_protocol/include/motor_protocol_adapter.h`（`MOTOR_PROTOCOL_CONFIG_SET_DIRECTION = 13U` + 请求字段 `uint32_t direction_raw`）
  - `lib/control_mailbox/include/control_mailbox.h`（`CONTROL_MAILBOX_CONFIG_SET_DIRECTION = 13U` + 值校验 `<= 1`）
  - `src/motor_rt_task.cpp`（两个 `static_assert` 契约 + 字段转发）
  - `lib/motor_control/src/motor_control_config.c`（提交门禁与阶段启动）
- CERB 阶段表：`kStagesDirection[2] = { WRITE(0x2541, 0x00, SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
  READ(0x2541, 0x00, SLOT_DIRECTION, ...) }`（写 + 读回核验，与 `kStagesSafetyProtect` 同形）。
  需新增 `CERB_VALUE_DIRECTION_RAW` 取值来源——**不能复用 `CERB_VALUE_PASSTHROUGH`**：
  后者在 `cerb_stage_write_value()` 的写分支里落 `default: return 0`（恒写 CW）。
- 门禁复用 `device_limit_write_allowed()`（仅未运动受理）+ `tx_active` → `BUSY`，与限位写同口径。

### 4.2 翻转判据（内核，且必须在落记录之前）

```
tx_kind == SET_DIRECTION && 事务核验成功
&& (record_raw_present & bit5)               // 有基线才谈得上"翻转"
&& tx_write_raw[SLOT_DIRECTION] != record_raw[SLOT_DIRECTION]
    → entry->zero_stale = true
```

比较的是**写前的记录值**，不是设备值：上电归一把设备从 CW 改回 CCW 时，记录值没变，
因此**不触发**；只有操作员真的改参数才触发。这个判据不依赖调用方（人工/归一），
所以 `update_record_after_write()` 覆盖 `record_raw` 之前就必须完成比较。

### 4.3 冻结与解除

- **冻结**（`zero_stale` 置位时）：`lifecycle = CONFIG_CONFLICT`、
  `unified_fault_code = last_failure_reason = MOTOR_CONTROL_FAULT_ZERO_STALE(8)`、
  清 `policy_enable` / `motion_authorized` / `has_target` / `enable_pending`
  （与 `apply_dev_init_result` 的冻结分支同形；写门禁已保证仅未运动，故正常不会走到 `soft_stop`）。
- **解除**：`SET_ZERO` 事务成功即清 `zero_stale` 并重判 `DEV_INIT`，结论 OK 则回
  `READY_DISABLED` + `READY` 事件。
  ⚠️ 需要在 `motor_control_finish_tx()` 里为 `SET_ZERO` 补一条显式分支——它今天走
  `update_record_after_write()` 的 `default: return`，既不置 `dev_init_settled` 也不重判。

### 4.4 事件可见性

不新增事件类型：冻结/解除通过状态话题的 `lifecycle = 7` +
`unified_fault_code = 8` 就能看到，写事务成败走既有 `CONFIG_RESULT`。

---

## 5. 保护关闭 + 二次确认

- **手动关** = 写 `m<id>.limit_protect = 0`（现有路径不变，`submit_safety_protect` 反转为 `raw = 1`）。
- **仍可武装**，但门禁（`motor_control_apply_policy_entry()`，`motor_control_config.c:743` 一带）：

```
request->enable && 该台保护确知为关 && !request->confirm_unprotected  → 拒绝
其中"保护确知为关" = device_side && (dev_raw_present & bit4) && dev_raw[SLOT_SAFETY] == 1
```

- `MotorPolicyTarget.msg` 新增 `bool confirm_unprotected`。**必须显式**：不能用
  `policy_generation` 冒充，它已经承担"未武装 → 武装需新意图"的代数门，两者语义不同。
- 拒绝原因新增 `MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED = 9U`，**只写
  `last_failure_reason`、不进 `unified_fault_code`、不改 lifecycle**——它不是设备故障，
  而是本机拒绝了一条缺确认的授权请求（与 `STALE_GENERATION(7)` 同待遇）。
- **页面**：点"武装"时若该台 `m<id>.limit_protect == 0` → 弹二次确认
  （文案点明"该关节没有任何限位"），确认后 policy 带 `confirm_unprotected: true`。
  状态列对保护已关的台持续标红。
- **下次上电**：0a 把它写回"开"。若记录曾被基线化成 `1`，这一次归一会把它改回 `0` 并落盘一次
  ——这是"关不跨上电保留"的落点，也是唯一一种上电写 Flash 的情形。

---

## 6. 状态话题

`MotorStatusTarget.msg` 新增（只读遥测）：

```
bool  device_protect_known      # false = 本会话尚未核验过该台（值无意义，页面显示 --）
bool  device_protect_enabled    # true = 保护生效（正向语义）
bool  device_direction_known
uint8 device_direction          # 0 = CW，1 = CCW
```

⚠️ **不许把厂家原值放上这个话题。** `0x2650:00` 的 `1 = 关` 是反向陷阱，参数面已经
刻意只在**一处**做反转（`param_bridge.cpp` 的 `submit_safety_protect` / `view_field`）；
话题若放原值，页面就得做第二次反转，而"反转只在一处发生"正是当初立的规矩。
所以话题上只放正向布尔，原值止步于协议层。

- 填充：`motor_control_status_t`（`motor_control.h:229`）加这四个字段，由
  `entry->dev_raw` / `dev_raw_present` 取——与参数面同源，两个面不会打架。
  角色划分写进两处 msg 注释：**参数面 = 控制入口（读值 + 写设备），话题 = 遥测**。
- 尺寸：每目标约 +5 B。48 台方案 §3.0 记 `motor_status_array` 在 N ≥ 15 时超 BE 512 B 上限
  [推导]，加这些字段后阈值会略降。当前 4~10 台无影响；**本次不解决**，
  但台架要留一行"10 台仍能收发"的实测。

---

## 7. 参数面容量与 web

- `kParamMotorFields[]`（`src/param_bridge.cpp:76`）5 → 6 项（加 `PARAM_FIELD_DIRECTION`），
  `static_assert(PARAM_MOTOR_FIELD_COUNT == 6U)`、`param_field_t` 尾部追加
  `PARAM_FIELD_DIRECTION`（**追加在 `PARAM_FIELD_COUNT` 前，不重排既有字段**——
  present 位图与注册顺序是契约）。
- `PARAM_BRIDGE_MAX_PARAMS` 64 → **72** → `kMaxMotors = (72 − 10) / 6 = 10`（维持现有 10 台）。
  堆：72 × 328 B = 23,616 B [推导，斜率实测]，`heap_free ≈ 122,880 − 19,660 − 23,616 ≈ 79.6 KB`。
  `param_bridge_create()` 里按 `MAX_PARAMS × 328 + 4096` 的建服前拦截自动跟着走。
- `include/param_bridge.h` 的容量注释同步（"10 电机 × 5 项" → "× 6 项"、
  `m<id>.direction` 进参数清单、`PARAM_MAX_PARAMS 64 槽` 的算式更新）。
- **web**（`ros2_ws/src/web_host/web/app.js`）：
  - 参数表新增"方向"列：`CW` / `CCW` 按钮，点一下写 `m<id>.direction`（切换值）
  - 保护勾选框 tooltip 补："上电默认开；取消后该台需二次确认才能武装，且下次上电会自动恢复为开"
  - `DEV_SLOT_PARAM_MASK` `0x1F → 0x3F`；删 `DEV_SLOT_EXTERNAL_MASK`；
    `devRecoveryText()` 改为"方向无基线时点一次方向即可建立基线"
  - `FAULT_NAMES` 补齐到 8（现在只有 0..5，`DEV_PARAM(6)`/`STALE_GENERATION(7)` 会掉进数字回退）
  - 状态行：保护已关 → 持续标红 + 文案"保护已关：该关节无任何限位"
- **VAXOR**：方向列显示 `--` 且禁用（无设备对象 → `dev_raw_present == 0` → `view_field` 自然给 NaN），
  与今天 `limit_protect` 同待遇。

---

## 8. 失效路径总表

| 场景 | 现象 | 结论 |
|---|---|---|
| 归一写保护失败（重试用尽） | 走不到 0b 回读 | `UNVERIFIED(41)` → `CONFIG_CONFLICT`，不可使能 |
| 归一写方向失败 | 同上 | 同上 |
| 记录无方向槽 / 值非法 | 跳过第 2 步，回读照跑 | 比对判基线缺失 → `MISMATCH(bit5)` → `CONFIG_CONFLICT`；点一次方向即恢复 |
| 手动关保护 | 设备 `raw=1`，记录基线化为 `1` | 可武装，但需 `confirm_unprotected`；页面标红；下次上电被归一写回"开" |
| 翻转方向未设零 | `ZERO_STALE(8)` + `CONFIG_CONFLICT` | `SET_ZERO` 成功即解除 |
| 回读读不到槽 4/5 | `dev_raw_present` 无位 | 话题 `*_known = false`，参数面 NaN，页面 `--` |
| 上电时该台处于 FAULT | 0a 不在允许生命周期内 | 不归一、不回读；故障恢复回 `READY_DISABLED` 后不变量复检补做 |
| 归一中途掉电 | 下次上电重跑（易失设备本来就是每次必写） | 无需特殊处理 |

---

## 9. 测试

**host（CTest + cassert，`test/host/`）**

- `test_motor_control.cpp`：0a 严格先于 0b；保护写失败 → `UNVERIFIED` 且**不发起** `REFRESH`；
  记录无方向槽 → 跳过写方向 + 判 `MISMATCH(bit5)`；翻转 → `ZERO_STALE` 冻结；
  `SET_ZERO` 成功 → 解除并回 `READY_DISABLED`；**上电归一写方向不触发 `ZERO_STALE`**；
  手动参数写在归一中途到达 → 不推进 `boot_norm_stage`（request_id 判据）
- `test_motor_protocol_adapter.cpp`：`SET_DIRECTION` 写帧字节 = `23 41 25 00 0X 00 00 00` + 读回核验；
  写值 > 1 拒绝
- `test_storage_projection.cpp` / `test_param_store.cpp`：方向槽落盘 / 迁移 / 缺失
- 参数注册项数 = 6（防再犯"注册循环把枚举尾整段带走"那个坑，2026-09-20 的教训）

**模拟器（`micro_ros_agent/tools/fdcan_motor_sim.py`）**

必须加 `0x2541` 写分支 **+ 易失模型**（复位后 `0x2650:00` 回 `1`、`0x2541` 回 `0`）
——没有它，全通路台架根本复现不了"上电归一"这个场景。

**台架（新增进 `docs/plans/active/motor-runtime-validation.md`）**

1. 探针先验 `23 41 25 00 0X` 写 + 读回，并做掉电对照实测确认易失 [未验]
2. 冷启动：观察归一事件序（保护 → 方向 → 回读 → `DEV_INIT OK`）与上电耗时
3. 手动关保护 → 武装被拒（`last_failure_reason = 9`）→ 带确认位成功 → 页面标红
4. 翻方向 → `ZERO_STALE` 冻结 → 设零 → 恢复
5. 清记录后上电 → 判基线缺失 → 点一次方向恢复
6. 状态话题 10 台仍能收发（尺寸回归）

**web**：无自动化测试，靠台架验收清单覆盖。

---

## 10. 文档影响

- `docs/architecture/motor-runtime-decision-log.md`：新增条目，含 09-19 那张从未落盘的决定表
  + 本设计的取代关系（§2 表）
- `micro_ros_firmware/README.md`：上电归一、`m<id>.direction`、`0x2541` 写路径
- `docs/plans/active/cerb-vendor-questions.md`：**#5 结案**（写命令 + 易失）
- `docs/reports/2026-09-19-firmware-param-surface-handoff.md`：§一.3 / §二 标注被取代
- `micro_ros_firmware/AGENTS.md`：一条"归一顺序（0a 在 0b 之前）是契约，不许调回"
- `docs/plans/active/motor-runtime-validation.md`：§9 的台架项
- `MotorEvent.msg` / `MotorStatusTarget.msg` 注释：故障码表补全 `6/7/8`（现在只列到 5）
- ✅ **消息改动只需动一处**：`micro_ros_agent/src/can_bridge_msgs/msg` 是唯一来源，
  两处消费者都已经是符号链接——固件 `micro_ros_firmware/extra_packages/can_bridge_msgs`，
  以及 `ros2_ws/src/can_bridge_msgs`（[源码] `git ls-files -s` 显示 mode `120000`，
  由 `c3a970a` 引入，2026-09-20 复核）。**勘误**：本文与实现计划初稿曾断言 ros2_ws 那份是
  "真实副本"，那是由 `ls -la <path>/`（**带尾斜杠会跟随链接**，列出的是目标目录内容）
  推出的无效结论；判定链接必须看**父目录**条目或 `readlink`。之前那句"改消息要动两处"
  因此是错的，Task 1 也随之作废。
  改后跑 `micro_ros_agent/tools/refresh_microros_messages.py`，**勿跑 `-t clean_microros`**

---

## 11. 未验项与风险

1. `23 41 25 00 0X` 由 owner 手工核实，**未见于任何手册/字典表** → 第一件台架事就是验写 + 读回；
   我们的写事务本来就带读回核验，写不进去会以 `UNCONFIRMED` 收场而不是静默成功。
2. 方向易失此前是"同形"推断，由 owner 确认；台架留一次对照实测记录。
3. 状态话题尺寸阈值下降（§6）。
4. **开保护的自锁风险**：09-19 单机空载静态实测"停发全部帧静置 5 秒没有自锁"，
   带载运动中仍未验。归一把"每次上电都开保护"从偶发变成常态，等于把这个未验假设
   的暴露面放大——**这是本设计最大的安全未知项**，进台架首日清单。
5. 上电归一给每台 CERB 增加 2 个 SDO 阶段（保护 2 + 方向 2 = 4 阶段/台）。
   按 30ms 超时上限最坏 4N×30ms，实测应远小于；进台架记录。

---

## 12. 实现顺序（供 implementation plan 用）

1. 消息：把 `ros2_ws/src/can_bridge_msgs` 改成符号链接并验证 colcon 仍构建（§10），
   然后改 `MotorPolicyTarget.confirm_unprotected`、`MotorStatusTarget` 四字段、注释（含故障码表补 `6/7/8`）
2. 协议层：`SET_DIRECTION` kind + cerb 阶段表 + `CERB_VALUE_DIRECTION_RAW` + host 测试
3. mailbox / `motor_rt_task` 契约（`static_assert` + 字段转发）
4. 内核：0a 阶段 + 翻转冻结 + `ZERO_STALE` 解除分支 + 比对基线缺失判据 + 状态视图四字段 + 落盘幂等化
5. 参数面：`kParamMotorFields` + `PARAM_BRIDGE_MAX_PARAMS 72` + 注释
6. 模拟器易失模型 + 探针写模式
7. 台架：验写命令/易失 → 冷启动 → 关保护闭环 → 翻向闭环（**第 1 项不通就停下来重新设计**）
8. web：方向列、二次确认、标红、文案、`FAULT_NAMES`
9. 文档（§10）
