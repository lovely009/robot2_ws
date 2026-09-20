# 参数权威方向反转 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把设备参数初始化的权威从驱动板反转回固件记录——无记录不再静默放行、写与保存拆成两次动作、固件与驱动板两份值同时可见供人工核对。

**Architecture:** 内核侧三处改动：① `DEV_INIT` 增第五态 `NO_RECORD`（无记录 = 未初始化，不再落进放行的 `NA`）；② 写事务不再跟 `0x2539` 保存阶段，保存独立成一个配置 kind，按"哪些槽位脏"选保存码；③ 落盘从"写成功后 500ms 自动去抖"变成"只有 `system.save` 落"，未保存状态按台记录并经新话题暴露。上层新增只读低频话题 `micro_ros_up/device_param_report` 承载"固件值 / 驱动板值"双行快照。

**Tech Stack:** C17（STM32H723 固件，PlatformIO）、C++17（host CTest，`cassert`）、micro-ROS / rclc 消息面、ROS 2 消息定义（`can_bridge_msgs`）。

**Spec:** `micro_ros_firmware/docs/plans/active/param-authority-reversal-design.md`

## 实现期勘误（相对本计划原文）

实现过程中有若干处本计划自身的写法被推翻。它们是**计划的错**，不是实现的偏差；
活规格是 `param-authority-reversal-design.md`，逐条裁定见各任务的提交信息。

| 计划原文 | 实际落地 | 提交 |
|---|---|---|
| Task 2：`save_slots` 必须不含 `CONTROL_MAILBOX_SAVE_SLOTS_MASK` 之外的位，并保留跨层 static_assert | **推翻**：生产者传的是全量脏位图，必然含易失槽（保护 bit4 / 方向 bit5）。按原规则则凡写过保护/方向的台一律被 mailbox 拒收，而归一恒写保护 ⇒ 空板首次保存必败 ⇒ `system.save` 整条失效。改为只挡空位图；哪个槽真有保存码由内核按掩码决定。宏与 static_assert 已删 | 0edbd82 |
| Task 3：`NO_RECORD` 由 `record_raw_present == 0` 判定 | 该判据**不可用**：上电归一恒写保护开关，那一笔会把该位图置起来，空板在回读前就"有记录"。改用独立的 `record_established`（W25 本来就有设备参数记录，或**人**写过；内部伪 request_id 的写不算） | 385f7ed |
| Task 4：保存分支不设运动禁写门 | **补回**：改动前 `0x2539` 只作为写事务尾巴发出，而写事务本就被 `device_limit_write_allowed` 挡着，ACTIVE 的电机收不到它；保存独立成事务后必须自己带这道门（放在"无设备侧落盘对象即早返回"**之后**） | f38e856 |
| Task 5：写事务后 `dev_si[]` 自动同步 | **不成立**：`dev_si[]` 全仓库只有一个写入点（REFRESH 路径），写核验后"驱动板"那一行会显示陈旧值。改为在 `update_record_after_write` 里把同一笔事务的 SI 同时写进 `record_si[]` 与 `dev_si[]` | 79fe9cf |
| Task 6：新 `.msg` 自动可见，不需改 CMakeLists | **错**：`can_bridge_msgs/CMakeLists.txt` 的 `rosidl_generate_interfaces()` 逐条列举每个 `.msg`，漏列时刷新脚本报成功却什么都不生成。另：计划的填充片段漏了 `uint16 count`，消费方会读到 0 条 | 8829a60 |
| Task 7：保存循环一次 poll 内全量下发 | **改为按拍分批**：FIFO 只有 16 深、消费侧每 5ms 抽 4 条。溢出被拒的台设备侧没落盘，而 `storage_task_request_save()` 是无条件整份快照 ⇒ W25 新、设备旧 ⇒ 下次上电判 MISMATCH。改为游标分批、扫完一轮才落 W25 | d4abb37 |

另：本计划 Task 1–7 的测试用例里有几处计划自身的写法不可通过或不可证伪（事件断言对都是消费性 drain；断言两行相等却写的是同一个值；绝对 tick 超出 CERB 离线门），实现期按真实意图改正 —— 明细见各任务提交信息。


## Global Constraints

- 构建/测试命令（均在 `micro_ros_firmware/` 内执行）：
  - 编译：`~/.platformio/penv/bin/pio run`（`pio` 不在 PATH）
  - host 测试：`cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2 && ctest --test-dir .host-build --output-on-failure`
- **改 `platformio.ini` 编译宏或 `ldscript/*.ld` 后必须 `pio run -t clean`**；本计划不改这两处。
- **改任何 `.msg` 后必须跑** `../micro_ros_agent/tools/refresh_microros_messages.py` 增量刷新（PlatformIO 只检查 `libmicroros.a` 是否存在，普通 `pio run` 不会重生成类型）；**禁用 `-t clean_microros`**（联网重下环境）。
- 代码规范：2 空格缩进、`snake_case` 文件/函数/变量、`PascalCase` 类、`k` 前缀常量；中文注释写意图（数据流/单位/安全边界），协议名与 ROS 2 API 保留英文。
- 提交信息用 conventional 前缀（`feat:` / `fix:` / `refactor:` / `test:` / `docs:`）。
- 单位口径：位置 rad、速度 rad/s、力矩 **A**（设备对象 `0x2538` 存的就是电流）。`MOTOR_PROTOCOL_PARAM_SLOT_*` 顺序是契约，勿重排。
- 缺失的一律 `NaN`（不用 0 冒充）。
- 安全规则不得被本计划放松：`NO_RECORD` 必须与 `MISMATCH`/`UNVERIFIED` 同口径冻结（进 `CONFIG_CONFLICT`、拒使能、不出运动帧）。
- 术语：**记录** = 固件 W25 里的那份参数（本设计的权威）；**设备/驱动板** = 电机驱动器里的对象。

---

## 文件结构

| 文件 | 职责 | 改动 |
|---|---|---|
| `lib/motor_protocol/include/motor_protocol_adapter.h` | 协议契约（kind 枚举、请求结构、槽位定义） | 加 `SAVE_DEVICE_FLASH` kind、`save_slots` 字段、`PARAM_SAVE_SLOTS_MASK` |
| `lib/motor_protocol/src/cerb_adapter.c` | CERB 阶段表与解释器 | 写事务删保存阶段；新增保存阶段表 + 选择器 |
| `lib/control_mailbox/include/control_mailbox.h` | 任务间配置 FIFO 契约 | 加 kind 与 `save_slots` 载荷 |
| `lib/control_mailbox/src/control_mailbox.c` | 配置请求校验 | 新 kind 的载荷校验 |
| `lib/motor_control/include/motor_control.h` | 内核公共 API | `DEV_INIT_NO_RECORD`、视图新字段、删 `take_record_dirty` |
| `lib/motor_control/src/motor_control_internal.h` | 表项布局 | `record_dirty_slots`、`record_si` |
| `lib/motor_control/src/motor_control_config.c` | 比对与事务 | `NO_RECORD` 判据、写/保存分离、保存受理 |
| `lib/motor_control/src/motor_control_params.c` | 参数视图 | 新字段填充 |
| `lib/motor_control/src/motor_control.c` | 上下文与生命周期 | 删 `record_dirty` |
| `src/motor_rt_task.cpp` | 5ms 控制边界 | 删自动落盘去抖；`convert_config` 透传新 kind |
| `src/param_bridge.cpp` + `include/param_bridge.h` | 参数面 | `system.save` 走保存事务；`NO_RECORD` 错误码；暴露视图快照 |
| `src/motor_bridge.cpp` | ROS 消息面 | 新话题 `device_param_report` 发布 |
| `micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamEntry.msg` | 新增 | 单台双行快照 |
| `micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamReport.msg` | 新增 | 报告容器 |
| `test/host/test_motor_protocol_adapter.cpp` | 适配器用例 | 阶段数 6→4 / 3→2；新增保存事务用例 |
| `test/host/test_control_mailbox.cpp` | mailbox 契约用例 | 新 kind 双链 static_assert + 校验用例 |
| `test/host/test_motor_control.cpp` | 内核用例 | 写事务阶段断言；`NO_RECORD`；脏位；保存事务 |

---

### Task 1: 适配器 —— 写事务摘掉 `0x2539`，保存独立成事务

**Files:**
- Modify: `lib/motor_protocol/include/motor_protocol_adapter.h`（kind 枚举尾部、`motor_protocol_config_request_t`、槽位宏区）
- Modify: `lib/motor_protocol/src/cerb_adapter.c`（`kStagesPositionLimits` / `kStagesVelocityLimit` / `kStagesTorqueLimit`、`cerb_stage_table`）
- Test: `test/host/test_motor_protocol_adapter.cpp`

**Interfaces:**
- Produces: `MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH = 14U`；`motor_protocol_config_request_t::save_slots`（`uint8_t`，位图 `1u << MOTOR_PROTOCOL_PARAM_SLOT_*`）；`MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK`。
- Consumes: 既有 `CERB_DESC_SAVE(code)`、`CERB_STAGE_SAVE` 解释分支（不动）。

- [ ] **Step 1: 写失败用例**

在 `test/host/test_motor_protocol_adapter.cpp` 末尾（`main()` 之前）新增：

```cpp
// 保存事务（SAVE_DEVICE_FLASH）：0x2539 保存码按"哪些槽位脏"选表，全部无应答、
// 发完即推进。**写事务里不再跟保存阶段**——"写"与"保存"是两次独立动作
// （见 docs/plans/active/param-authority-reversal-design.md §3.2）。
static void test_cerb_save_flash_transaction() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  // 保存码位图 = 1u << MOTOR_PROTOCOL_PARAM_SLOT_*
  // （POS_MIN=0, POS_MAX=1, VEL_MAX=2, TORQUE_MAX=3, SAFETY=4, DIRECTION=5）
  // 地址即语义，这里直接写位图比引枚举更贴测试意图。
  const uint8_t kPosBoth = 0x03U;       // 位置下限 | 位置上限
  const uint8_t kTorque = 0x08U;        // 力矩限值
  const uint8_t kParamsAndPos = 0x07U;  // 位置上下限 | 速度限值
  const uint8_t kVolatileOnly = 0x30U;  // 保护开关 | 方向

  // 0x2539 帧逐字节：23 39 25 00 <码> 00 00 00
  // （快速 SDO 写 4 字节到 0x2539:00；码 1=PI 参数+最大电流 / 2=位置上限 / 4=位置下限）
  auto expect_save = [&](const motor_protocol_config_request_t *req, uint8_t stage,
                         uint8_t code) {
    motor_protocol_tx_frame_t frame = {};
    assert(motor_protocol_cerb_adapter.config_encode_stage(stage, profile, 1U, 0U,
                                                          req, &frame));
    assert(frame.size == 8U && frame.is_fd && frame.is_brs);
    assert(frame.can_id == 0x600U + 1U);
    assert(frame.data[0] == 0x23U && frame.data[1] == 0x39U &&
           frame.data[2] == 0x25U && frame.data[3] == 0x00U);
    assert(frame.data[4] == code);
    motor_protocol_config_stage_info_t info = {};
    assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, req, &info));
    assert(!info.waits_response);  // 无应答：驱动器写 FLASH 期间不拖总线
    assert(!info.param_raw_valid); // 保存码不是设备参数，不进记录/比对
    assert(info.timeout_ms == 0U);
  };

  motor_protocol_config_request_t pos = {};
  pos.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  pos.save_slots = kPosBoth;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &pos) == 2U);
  expect_save(&pos, 0U, 4U); // 下限
  expect_save(&pos, 1U, 2U); // 上限

  motor_protocol_config_request_t trq = {};
  trq.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  trq.save_slots = kTorque;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &trq) == 1U);
  expect_save(&trq, 0U, 1U);

  // 两组都脏 → 三条：先参数组，再位置两码（顺序只是确定性，不是协议要求）。
  motor_protocol_config_request_t both = {};
  both.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  both.save_slots = kParamsAndPos;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &both) == 3U);
  expect_save(&both, 0U, 1U);
  expect_save(&both, 1U, 4U);
  expect_save(&both, 2U, 2U);

  // 只脏易失槽（保护开关 / 方向）：厂家没给这两个对象保存码 → **0 阶段**。
  // 内核据此只清脏位 + 落 W25，不发任何 0x2539；两者靠上电归一从记录重建。
  motor_protocol_config_request_t volatile_only = {};
  volatile_only.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  volatile_only.save_slots = kVolatileOnly;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &volatile_only) == 0U);

  // Vaxor 无设备对象 → 0 阶段（内核回 UNSUPPORTED；其限值是本地软限位，
  // 记录落 W25 即持久，没有设备侧 flash 要写）。
  const motor_protocol_model_profile_t *vp = nullptr;
  assert(find_profile("vaxor_2550", &vp));
  assert(motor_protocol_vaxor_adapter.config_stage_count(vp, &both) == 0U);
}
```

并在 `main()` 里紧随 `test_cerb_sdo_position_limit_frames();` 之后追加 `test_cerb_save_flash_transaction();`。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd micro_ros_firmware
cmake --build .host-build -j2 --target test_motor_protocol_adapter
```
Expected: 编译失败，`MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH` 未定义 / `save_slots` 无此成员。

- [ ] **Step 3: 契约与字段**

`lib/motor_protocol/include/motor_protocol_adapter.h`：

(a) kind 枚举尾部（`MOTOR_PROTOCOL_CONFIG_SET_DIRECTION = 13U` 之后，注意把 `= 13U` 那一行的尾部逗号保留）：

```c
  MOTOR_PROTOCOL_CONFIG_SET_DIRECTION = 13U,
  // 设备侧保存（仅 CERB，写 0x2539 保存码）：把设备 RAM 里的值落到驱动器 flash。
  // **与写事务分开**——"写"只改设备 RAM + 更新固件记录，"保存"才落盘；两次动作
  // 之间该台处于「未保存」，由 DeviceParamReport 暴露、靠 system.save 收尾。
  // 载荷 save_slots：待保存的槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*），
  // adapter 据此选保存码、不写无关对象（驱动器写 flash 期间会阻塞自身控制循环，
  // 实测 ~1s，多存一个对象就是白等一秒）。
  MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH = 14U
} motor_protocol_config_kind_t;
```

(b) `motor_protocol_config_request_t` 内，`confirm_unprotected` 之后追加：

```c
  // SAVE_DEVICE_FLASH：待保存的槽位位图。只含**有厂家保存码**的槽位才有意义
  // （位置上下限 + 速度/力矩限值）；保护开关与方向是易失对象，无保存码。
  uint8_t save_slots;
```

(c) 槽位宏区（`MOTOR_PROTOCOL_PARAM_SLOT_COUNT` 之后）：

```c
// 有厂家保存码（0x2539）的槽位：位置上下限（码 4/2）+ 速度、力矩限值（码 1）。
// 保护开关与方向**不在内**——易失对象，厂家没给保存码，靠上电归一从记录重建
// （见 docs/plans/active/boot-normalize-design.md）。
#define MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK \
  ((uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN) |  \
             (1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX) |  \
             (1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX) |  \
             (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX)))
```

- [ ] **Step 4: CERB 阶段表**

`lib/motor_protocol/src/cerb_adapter.c`：把 `kStagesPositionLimits` 缩成 4 条、`kStagesVelocityLimit` / `kStagesTorqueLimit` 各缩成 2 条——

```c
// 下限写+读回、上限写+读回。**保存不在本表**：0x2539 是独立的保存事务
// （MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH），写到这里就落盘会让"只改没存"
// 变成不可观测的状态（见 param-authority-reversal-design.md §3.2）。
static const cerb_stage_desc_t kStagesPositionLimits[4] = {
  CERB_DESC_WRITE(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
                  MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_LOWER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, CERB_VALUE_POS_LOWER),
  CERB_DESC_WRITE(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
                  MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
  CERB_DESC_READ(CERB_OBJ_POS_LIMIT, CERB_SUB_POS_UPPER,
                 MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX, CERB_VALUE_POS_UPPER),
};

static const cerb_stage_desc_t kStagesVelocityLimit[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
                  MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
  CERB_DESC_READ(CERB_OBJ_MAX_SPEED, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, CERB_VALUE_SPEED),
};

static const cerb_stage_desc_t kStagesTorqueLimit[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
                  MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
  CERB_DESC_READ(CERB_OBJ_MAX_CURRENT, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX, CERB_VALUE_CURRENT),
};
```

紧接 `kStagesDirection` 之后（`cerb_stage_table` 之前）加保存阶段表与选择器：

```c
// 保存阶段表集合（SAVE_DEVICE_FLASH）。落盘粒度只有两组：参数组（码 1，
// 覆盖 PI 参数 + 最大电流 0x2538 + 最大转速 0x6080）与位置限位组（码 4/2）。
// 位置上下限在写事务里是同一笔（SET_POSITION_LIMITS 一次写两槽），所以脏掩码
// 实际上只出现"两组各自脏"或"两组都脏"；万一将来出现只脏一个位置槽的形状，
// 按"多存一个对象"兜底——多写一次 FLASH 只是慢，少写一次就是掉电丢参数。
static const cerb_stage_desc_t kStagesSavePosBoth[2] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_LOWER),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_UPPER),
};

static const cerb_stage_desc_t kStagesSaveParams[1] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_PARAMS),
};

static const cerb_stage_desc_t kStagesSaveParamsAndPos[3] = {
  CERB_DESC_SAVE(CERB_SAVE_CODE_PARAMS),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_LOWER),
  CERB_DESC_SAVE(CERB_SAVE_CODE_POS_UPPER),
};

static const cerb_stage_desc_t *cerb_save_table(
    const motor_protocol_config_request_t *request, uint8_t *count) {
  const bool params =
      (request->save_slots &
       ((1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX) |
        (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX))) != 0U;
  const bool pos =
      (request->save_slots &
       ((1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN) |
        (1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX))) != 0U;
  const cerb_stage_desc_t *table;
  if (params && pos) {
    table = kStagesSaveParamsAndPos;
    *count = 3U;
  } else if (params) {
    table = kStagesSaveParams;
    *count = 1U;
  } else if (pos) {
    table = kStagesSavePosBoth;
    *count = 2U;
  } else {
    // 只脏易失槽（保护开关 / 方向）：没有保存码可发。count=0 让内核把这一步
    // 收敛成"清脏位 + 落 W25"，而不是每拍重开一个空事务。
    *count = 0U;
    return NULL;
  }
  return table;
}
```

`cerb_stage_table` 的 `switch` 里，`SET_DIRECTION` 分支之后、`default` 之前插入：

```c
    case MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH:
      return cerb_save_table(request, count);
```

- [ ] **Step 5: 改既有阶段数断言**

同文件（`test/host/test_motor_protocol_adapter.cpp`）：

1. `test_cerb_sdo_position_limit_frames`（约 483-535 行）：
   - 头部注释改为：`// 位置限位：4 阶段（写下限/读下限/写上限/读上限）。保存（0x2539）不在这里——它是独立的 SAVE_DEVICE_FLASH 事务（见 test_cerb_save_flash_transaction）。`
   - `assert(... == 6U)` → `== 4U`
   - 删掉 `kSaveLower` / `kSaveUpper` 两个常量，`expect[6]` → `expect[4] = {kWriteLower, kReadLower, kWriteUpper, kReadUpper}`
   - 循环上界 `stage < 6U` → `stage < 4U`
   - `if (stage < 4U) { assert(info.waits_response); ... } else { ...写 FLASH... }` 整段简化为无条件断言：
     ```cpp
     assert(info.waits_response); // 四个 SDO 阶段都等应答（写等 ack、读等回读）
     assert(info.timeout_ms > 0U);
     ```
   - 越界断言 `config_encode_stage(6U, ...)` → `config_encode_stage(4U, ...)`，注释改为 `// 越界阶段无效（4 阶段：0..3）。`

2. `test_cerb_sdo_velocity_torque_frames`（约 715-760 行）：两处 `config_stage_count(profile, &vel) == 3U` / `&trq) == 3U` → `== 2U`。

3. `test_cerb_safety_protect_write_frames`（约 1056 行）注释里"哪天有人顺手补一条保存就会在这里被拦住"保持有效（阶段数仍是 2），不动。

- [ ] **Step 6: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
```
Expected: 适配器与 mailbox 用例全绿；`test_motor_control` 里两条按写事务阶段写死的用例此时会红——见 Step 7。

- [ ] **Step 7: 修内核用例里按写事务阶段写死的两处**

`test/host/test_motor_control.cpp`：

(a) `test_cerb_limits_sdo_transaction`（约 1753 行起）：
- 注释 `六阶段（…／**写 FLASH 0x2539 两次…**）全通才 OK` 改为
  `四阶段（写下限/读下限/写上限/读上限）全通才 OK；保存是独立的 SAVE_DEVICE_FLASH 事务，不在这里。`
- 删掉"阶段 4/5：写 FLASH"那一整段循环（`const uint32_t kSaveCodes[2] = {4U, 2U};` 到该循环结束），改为读回上限应答之后直接断言收尾：
  ```cpp
  // 阶段 3 的读回应答已由上面喂入；下一拍收尾（写事务不再跟保存阶段）。
  t += 10;
  motor_control_service(&ctx, t);
  assert(has_result_event(&ctx, 9U, MOTOR_PROTOCOL_CONFIG_OK));
  ```
  （若原用例在循环后还有 `has_result_event(9U, ...)` 断言，合并成一条，不要重复。）

(b) 速度限值那条（约 3130-3145 行）：把 `motor_control_service(&ctx, t); // 末阶段：写 FLASH（0x2539，无应答）后收尾` 的注释改成 `// 末阶段：读回核验之后收尾（写事务不含保存）`，`collect_tx` 与断言不动。

- [ ] **Step 8: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
```
Expected: 全绿。

- [ ] **Step 9: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/motor_protocol micro_ros_firmware/test/host
git commit -m "refactor(motor_protocol): 写事务摘掉 0x2539，保存独立成 SAVE_DEVICE_FLASH 事务"
```

---

### Task 2: mailbox 透传保存事务

**Files:**
- Modify: `lib/control_mailbox/include/control_mailbox.h`
- Modify: `lib/control_mailbox/src/control_mailbox.c`
- Modify: `src/motor_rt_task.cpp`（`convert_config` + `static_assert`）
- Test: `test/host/test_control_mailbox.cpp`

**Interfaces:**
- Consumes: Task 1 的 `MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH = 14U`、`save_slots`。
- Produces: `CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH = 14U`；`control_mailbox_config_request_t::value.save_slots`（`uint8_t`）。

- [ ] **Step 1: 写失败用例**

`test/host/test_control_mailbox.cpp`：

(a) 在既有 kind 契约 `static_assert` 之后（`SET_SAFETY_PROTECT` 那条下方）追加：

```cpp
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_DIRECTION) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_DIRECTION),
    "mailbox/protocol config kind 契约不一致：SET_DIRECTION");
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH),
    "mailbox/protocol config kind 契约不一致：SAVE_DEVICE_FLASH");
// mailbox 是协议无关层，不引 motor_protocol 头；保存槽位位图这条事实在两处
// 各写一遍，靠本断言双向锁死。
static_assert(CONTROL_MAILBOX_SAVE_SLOTS_MASK ==
                  (int)MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK,
              "mailbox/protocol 保存槽位位图不一致");
```

(b) 在 `test_safety_protect_request_validation` 之后新增：

```cpp
// 保存请求：载荷是槽位位图，只允许"有厂家保存码"的槽位——空位图没有要保存的
// 对象；含易失槽（保护开关/方向）的位图是概念错误（厂家没给保存码）。
static void test_save_request_validation() {
  control_mailbox_t mailbox;
  control_mailbox_init(&mailbox);

  control_mailbox_config_request_t req = make_mode_request(7U, 1U, 0U);
  req.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;

  // 位置上下限 | 力矩限值：合法。
  req.value.save_slots =
      (uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN) |
                (1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX) |
                (1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX));
  assert(control_mailbox_submit_config(&mailbox, &req));

  req.value.save_slots = 0U;
  assert(!control_mailbox_submit_config(&mailbox, &req));

  req.value.save_slots = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  assert(!control_mailbox_submit_config(&mailbox, &req));
}
```

并在 `main()` 里紧随 `test_safety_protect_request_validation();` 之后追加 `test_save_request_validation();`。

> `make_mode_request` 与"会话已连接"的引导手法照抄同文件既有用例（既有的 `SET_SAFETY_PROTECT` / `SET_DIRECTION` 用例就是这么进来的）。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 --target test_control_mailbox
```
Expected: 编译失败，`CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH` 未定义。

- [ ] **Step 3: 契约**

`lib/control_mailbox/include/control_mailbox.h`：

(a) kind 枚举尾部：

```c
  // 设备侧电机方向定义（仅 CERB，写 0x2541:00）：载荷是**原值** 0 = CW /
  // 1 = CCW。仅未运动时受理，写后读回核验，不跟保存帧（易失）。
  CONTROL_MAILBOX_CONFIG_SET_DIRECTION = 13U,
  // 设备侧保存（仅 CERB，写 0x2539 保存码）：把设备 RAM 里的值落到驱动器
  // flash。**与写分开**——写只改设备 RAM + 固件记录，保存才落盘；两次动作
  // 之间该台处于「未保存」（DeviceParamReport.record_dirty）。
  // 载荷 = 待保存槽位位图，只允许 CONTROL_MAILBOX_SAVE_SLOTS_MASK 内的位。
  CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH = 14U
} control_mailbox_config_kind_t;

// 有设备侧 flash 保存码的槽位位图（bit0..3 = 位置下限/位置上限/速度限值/力矩限值）。
// 与 motor_protocol_adapter.h 的 MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK 是同一条
// 事实——mailbox 是协议无关层，不引 motor_protocol 头；一致性由
// test_control_mailbox.cpp 的 static_assert 双向锁死。
#define CONTROL_MAILBOX_SAVE_SLOTS_MASK 0x0FU
```

(b) `union value` 内、`direction_raw` 之后：

```c
    // SAVE_DEVICE_FLASH：待保存槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*）。
    uint8_t save_slots;
```

(c) 校验（`control_mailbox.c` 的 `config_request_is_valid`），`SET_DIRECTION` 分支之后：

```c
    case CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH:
      // 只收"有厂家保存码"的槽位：空位图没有要保存的对象；含易失槽（保护开关
      // 0x2650 / 方向 0x2541）的位图是概念错误——厂家没给这两个对象保存码。
      return request->value.save_slots != 0U &&
             (request->value.save_slots &
              (uint8_t)~CONTROL_MAILBOX_SAVE_SLOTS_MASK) == 0U;
```

- [ ] **Step 4: 转换层**

`src/motor_rt_task.cpp`：

(a) `static_assert` 清单尾部（`SET_DIRECTION` 那条之后）追加：

```cpp
static_assert(
    static_cast<int>(CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH) ==
        static_cast<int>(MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH),
    "mailbox/protocol config kind 契约不一致：SAVE_DEVICE_FLASH");
```

(b) `convert_config` 的 `switch` 里补分支（跟 `SET_DIRECTION` 分支同形）：

```cpp
    case CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH:
      out->save_slots = in->value.save_slots;
      break;
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
```
Expected: host 全绿、固件编译通过。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/control_mailbox micro_ros_firmware/src/motor_rt_task.cpp micro_ros_firmware/test/host/test_control_mailbox.cpp
git commit -m "feat(control_mailbox): 透传设备侧保存事务（SAVE_DEVICE_FLASH）"
```

---

### Task 3: 内核 —— `NO_RECORD` 第五态

**Files:**
- Modify: `lib/motor_control/include/motor_control.h`（`motor_control_dev_init_t`）
- Modify: `lib/motor_control/src/motor_control_config.c`
- Test: `test/host/test_motor_control.cpp`

**Interfaces:**
- Produces: `MOTOR_CONTROL_DEV_INIT_NO_RECORD = 5U`；`motor_control_entry_t::record_established`；`motor_control_config.c` 内 static 助手 `entry_has_record()`、`settle_dev_init()`。
- Consumes: 既有 `dev_params_mismatch_mask()`、`apply_dev_init_result()`、`motor_control_dev_params_allow_motion()`、`MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE`。

**语义**：`NO_RECORD` 专指**该台没有设备参数的基线记录**（`record_established == false`，见 Step 3b）。部分槽位有记录时维持既有规则（没写过的槽位不参与比对）——刚烧录、Flash 半空的电机不该整体卡住。

- [ ] **Step 1: 写失败用例**

`test/host/test_motor_control.cpp`，紧跟 `test_boot_normalize_skips_direction_without_record()` 之后新增：

```cpp
// 首次上电（W25 对本机无记录）：**不再静默放行**。owner 2026-09-21：首次也当
// 不一致，必须人工核对后才可动。此前"无记录"落进 DEV_INIT_NA，而 NA 是放行的。
static void test_dev_init_no_record_blocks_enable() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);

  // Flash 全空：count=0 也必须调用——但 records 指针本身**不能**是 NULL：那是
  // "本次没有数据可传"，函数会直接返回，连"记录已就位（哪怕是没有记录）"都不
  // 声明，0b 回读永远不会发起（见 motor_control_apply_stored_params 的注释）。
  motor_control_stored_params_t empty = {};
  motor_control_apply_stored_params(&ctx, &empty, 0U, 110);
  assert(entry_of(&ctx, 7)->boot_record_offered);
  assert(!entry_of(&ctx, 7)->record_established);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);

  (void)feed_device_params(&ctx, 7, kR58Raw, 120U, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(!motor_control_dev_params_allow_motion(e));
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM);

  // 事件带的是本态本身（detail=0：没有"哪一项对不上"可报，整台没基线）。
  uint16_t state = 0U;
  uint32_t mask = 0xFFFFFFFFU;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_NO_RECORD);
  assert(mask == 0U);

  // 参数视图也带本态（页面据此把固件列整行显示「无记录」）。
  assert(view_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);

  // 使能请求被明确拒绝（不是静默吞掉）。
  motor_protocol_config_request_t en = {};
  en.request_id = 90U;
  en.motor_id = 7;
  en.kind = MOTOR_PROTOCOL_CONFIG_SET_ENABLE;
  en.enable = true;
  assert(!motor_control_submit_config(&ctx, &en, 200));
  assert(e->last_failure_reason == MOTOR_CONTROL_FAULT_DEV_PARAM);
}

// 写一项就建立记录 → 立刻重判；记录写齐（含方向基线）即转 OK、解冻。
// 这正是 owner 第 6 条"下发修改后校验应通过、不再是未初始化"。
static void test_dev_init_no_record_clears_after_write() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t empty = {};
  motor_control_apply_stored_params(&ctx, &empty, 0U, 110);
  uint32_t t = feed_device_params(&ctx, 7, kR58Raw, 120U, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD);

  // 只写速度限值一项：记录（内存里）建立了，但方向槽仍无基线 → 判 MISMATCH
  // 且位图点名方向。**不伪造通过**。
  motor_protocol_config_request_t vel = {};
  vel.request_id = 30U;
  vel.motor_id = 7;
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  assert(motor_control_submit_config(&ctx, &vel, t + 1U));
  t = feed_velocity_write(&ctx, 7, kR58Raw[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX],
                          t + 2U);
  assert(has_result_event(&ctx, 30U, MOTOR_PROTOCOL_CONFIG_OK));
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)); // 只差方向基线

  // 再写方向：记录写齐 → OK、解冻回 READY_DISABLED（可授权）。
  motor_protocol_config_request_t dir = {};
  dir.request_id = 31U;
  dir.motor_id = 7;
  dir.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  dir.direction_raw = 0U;
  assert(motor_control_submit_config(&ctx, &dir, t + 10U));
  (void)feed_direction_write(&ctx, 7, 0U, t + 11U);
  assert(has_result_event(&ctx, 31U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(motor_control_dev_params_allow_motion(entry_of(&ctx, 7)));
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}
```

需在助手区新增 `feed_velocity_write(ctx, node, rpm, tick)`：照抄既有 `feed_torque_write`，把索引 `0x2538U` → `0x6080U`、`sdo_read_ack(node, 3U, ...)` → `sdo_read_ack(node, 2U, ...)`，其余（写 ack → 读回核验 → 收尾）逐行同形。

并在 `main()` 里挂上两个新用例。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 --target test_motor_control
```
Expected: 编译失败，`MOTOR_CONTROL_DEV_INIT_NO_RECORD` / `feed_velocity_write` 未定义。

- [ ] **Step 3: 枚举**

`lib/motor_control/include/motor_control.h`：

```c
// 设备参数初始化状态（上电/重连后回读设备 → 与固件记录比对）。
// **固件内部维护的这份记录是权威**，设备里的参数要与它对齐（2026-09-21 owner
// 反转，此前口径是"以驱动板为准"）：不一致即"未初始化"，该台不可控。
typedef enum {
  MOTOR_CONTROL_DEV_INIT_NA = 0U,        // 非设备侧型号（VAXOR）：无从比对
  MOTOR_CONTROL_DEV_INIT_PENDING = 1U,   // 尚未回读设备（上电/重连后）
  MOTOR_CONTROL_DEV_INIT_OK = 2U,        // 有记录且逐槽一致
  MOTOR_CONTROL_DEV_INIT_MISMATCH = 3U,  // 有记录、某些槽不一致 → 未初始化
  MOTOR_CONTROL_DEV_INIT_UNVERIFIED = 4U, // 回读失败（超时/中止）→ 未核对
  // 本机对该台**一条设备参数记录都没有**（首次上电 / 新板 / Flash 空）。
  // 与 NA 必须分开：NA 是"本协议没有设备侧对象"（放行），无记录是"没核对过"
  // （拦住）。此前两者共用 NA，于是首次上电静默通过、无人核对就可用。
  // 只按"完全没有"判：部分槽位有记录时维持既有的"没写过的槽位不参与比对"。
  MOTOR_CONTROL_DEV_INIT_NO_RECORD = 5U
} motor_control_dev_init_t;
```

同步更新 `lib/motor_control/src/motor_control_internal.h:243` 附近 `motor_control_dev_params_allow_motion` 的注释：`PENDING/MISMATCH/UNVERIFIED/NO_RECORD 一律不放行`。

- [ ] **Step 3b: 表项加"基线记录"标志**

**为什么不能直接用 `record_raw_present` 判"有没有记录"**：上电归一恒写保护开关
（`0x2650:00`），那一笔走的是同一个写事务收尾，会把 `record_raw_present` 置上 bit4。
于是**每一块空 Flash 的板子在回读之前就已经"有记录"了**，`NO_RECORD` 永远判不出来。
必须把"W25 里本来就有这台设备参数的记录 / 人写过"与"固件自己的归一动作"分开。

`lib/motor_control/src/motor_control_internal.h`，`record_valid` 之后插入：

```c
  // 本机对该台有没有**设备参数的基线记录**：W25 里本来就有（device_raw_present
  // 非零），或人写过（写事务的 request_id 来自上层，bit31=0）。
  // 上电归一自己写保护/方向**不算**建立基线——否则空 Flash 的板子上电即"有记录"，
  // NO_RECORD 永远判不出来（归一恒写保护开关）。
  // 与 record_valid 的区别：后者只表示"记录结构里有内容"，归一的写也会置它。
  bool record_established;
```

- [ ] **Step 4: 判据**

`lib/motor_control/src/motor_control_config.c`：

(a) 在 `dev_params_match_record()` 之后加：

```c
// 本机对该台有没有基线记录。**不能用 record_raw_present 判**：上电归一恒写保护
// 开关，那一笔会把它的 bit4 置起来，于是空 Flash 的板子在回读前就"有记录"了，
// NO_RECORD 永远判不出来。判据是 record_established（见字段注释）。
static bool entry_has_record(const motor_control_entry_t *entry) {
  return entry->record_established;
}

// 统一的比对结论（自动回读收尾与写事务后的重判共用）：
//   无记录 → NO_RECORD；有记录 → 逐槽位比 → OK / MISMATCH。
// 写事务会就地建立记录，所以"写过一项"就不再是 NO_RECORD——但那不等于通过：
// 记录没写齐（典型是方向无基线）仍判 MISMATCH 并由位图点名，不伪造 OK。
static motor_control_dev_init_t settle_dev_init(
    const motor_control_entry_t *entry) {
  if (!entry_has_record(entry)) {
    return MOTOR_CONTROL_DEV_INIT_NO_RECORD;
  }
  return dev_params_match_record(entry) ? MOTOR_CONTROL_DEV_INIT_OK
                                        : MOTOR_CONTROL_DEV_INIT_MISMATCH;
}
```

(b) `motor_control_finish_tx` 的 REFRESH 成功分支：

```c
      entry->dev_raw_present = entry->refresh_slots;
      update_shadow_from_read(entry);
      dev_state = settle_dev_init(entry);
      entry->dev_read_done = true;
      dev_init_settled = true;
```

写事务成功后的重判：

```c
    if (entry->dev_read_done) {
      dev_state = settle_dev_init(entry);
      dev_init_settled = true;
    }
```

(c) `apply_dev_init_result`：把 `NO_RECORD` 并入冻结分支，并更新该分支注释——

```c
  } else if (state == MOTOR_CONTROL_DEV_INIT_MISMATCH ||
             state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED ||
             state == MOTOR_CONTROL_DEV_INIT_NO_RECORD) {
    // 未初始化（不一致 / 未能核对 / 本机无记录）：**运动中的走软停，不硬失能**
    // ——上电自检与写入都核验过，真出现不一致属于异常，此时按"停止运动但保留
    // 阻尼"处理（TBD-SAFETY-01 口径），不把力矩瞬间撤掉。
    // 静止态（READY_DISABLED 等）没有要停的东西，直接锁存结论。
    // 与冲突/故障同口径冻结（ARD §36：失败不自动恢复运动授权）：结论未转好
    // 之前不得使能，恢复需显式 SET_ENABLE(true) + 新快照。
    //
    // NO_RECORD 走同一条路（owner 2026-09-21："首次也当不一致，人工核对"）：
    // 此前它落进 NA 而 NA 是放行的，等于首次上电未经核对即可使能。
    freeze_entry_as_conflict(ctx, entry, MOTOR_CONTROL_FAULT_DEV_PARAM, now_ms);
  }
```

`ready` 判据与 `detail` 计算**不动**（`ready` 仍是 `NA || OK`；NO_RECORD 的 detail 保持 0）。

- [ ] **Step 4b: 基线标志的两个置位点**

(a) `lib/motor_control/src/motor_control_params.c` 的 `apply_stored_to_entry`，记录内容捕获那个分支（`memcpy(entry->record_raw, ...)` 之后）：

```c
    // 有设备参数的基线记录（device_raw_present 非零才算数：只存了 kp/kd 的
    // 记录对设备参数比对毫无用处，"无记录"是它诚实的结论）。
    if (rec->device_raw_present != 0U) {
      entry->record_established = true;
    }
```

(b) `lib/motor_control/src/motor_control_config.c` 的 `update_record_after_write`，`entry->record_valid = true;` 那一行之前：

```c
  // 只有**上层发起的写**才建立基线。上电归一（保护 / 方向）与它共用同一个收尾
  // 路径，但那是固件自己的动作：让它把 record_established 置起来，空 Flash 的
  // 板子上电即"有记录"，NO_RECORD 永远判不出来。
  // 判据 = 内部事务的 request_id 高位置位（MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE）；
  // 上层请求走 mailbox，id 最高位是 0（param_bridge 用 bit30 做标签）。
  if ((entry->tx_request_id & MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE) == 0U) {
    entry->record_established = true;
  }
```

> `update_record_after_write` 在 Task 4 里才改签名（去掉 `ctx`），本步先只加标志，不动签名。

- [ ] **Step 5: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
```
Expected: 两个新用例通过。

> 若出现失败：本步把"无记录 → 放行"改成了"无记录 → 拦"，任何**依赖旧放行洞**的既有用例都会红。逐条看它是不是在断言"无记录也 OK"——是就改断言（那正是 owner 要消掉的行为），是别的语义就修代码。**不要**为了让测试变绿而把 `NO_RECORD` 挪回放行侧。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/motor_control micro_ros_firmware/test/host/test_motor_control.cpp
git commit -m "feat(motor_control): 无记录判 NO_RECORD 未初始化（首次上电也要人工核对）"
```

---

### Task 4: 内核 —— 写/保存分离

**Files:**
- Modify: `lib/motor_control/include/motor_control.h`（删 `motor_control_take_record_dirty`）
- Modify: `lib/motor_control/src/motor_control_internal.h`
- Modify: `lib/motor_control/src/motor_control_config.c`
- Modify: `lib/motor_control/src/motor_control.c`
- Modify: `src/motor_rt_task.cpp`
- Test: `test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 1 的 `MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH` / `save_slots` / `MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK`。
- Produces: `motor_control_entry_t::record_dirty_slots`（`uint8_t`，全部改动过的槽位；含易失槽）。**删除** `motor_control_context_t::record_dirty` 与 `bool motor_control_take_record_dirty(...)`。

- [ ] **Step 1: 写失败用例**

`test/host/test_motor_control.cpp`：

(a) 新增：

```cpp
// 写不落盘：写事务成功只把记录改在内存里并置脏位；设备侧 0x2539 与 W25 都
// **不许发生**，直到 system.save。这是 owner 第 9 条"只点了写必须有未保存标记"
// 的机制面。
static void test_write_does_not_persist_until_save() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  assert(entry_of(&ctx, 7)->record_dirty_slots == 0U);

  motor_protocol_config_request_t vel = {};
  vel.request_id = 70U;
  vel.motor_id = 7;
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  assert(motor_control_submit_config(&ctx, &vel, 200));
  const uint32_t t = feed_velocity_write(&ctx, 7, kR58Raw[2], 201U);
  assert(has_result_event(&ctx, 70U, MOTOR_PROTOCOL_CONFIG_OK));

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->record_dirty_slots == (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX));
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK); // 写不影响可动性，只是"未保存"

  // 事务已收尾，此后任一拍都不得冒出 0x2539（写事务不含保存阶段）。
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  for (uint32_t k = 0U; k < 5U; ++k) {
    motor_control_service(&ctx, t + 10U + k);
    const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    for (size_t i = 0U; i < n; ++i) {
      assert(!(drain[i].size == 8U && drain[i].data[1] == 0x39U &&
               drain[i].data[2] == 0x25U));
    }
  }
}

// 保存事务：按脏位图发对应保存码，收尾清脏位。**只保存脏的那台**。
static void test_save_flash_clears_dirty_and_emits_codes() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  bring_cerb_to_ready(&ctx, 8, "cerb_r48_101", 100); // 不脏，保存时不许被碰

  motor_protocol_config_request_t limits = {};
  limits.request_id = 80U;
  limits.motor_id = 7;
  limits.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  limits.position_limits.min_position_rad = -1.0f;
  limits.position_limits.max_position_rad = 1.0f;
  assert(motor_control_submit_config(&ctx, &limits, 200));
  uint32_t t = feed_position_limits_write(&ctx, 7, 201U);
  assert(has_result_event(&ctx, 80U, MOTOR_PROTOCOL_CONFIG_OK));

  motor_protocol_config_request_t vel = {};
  vel.request_id = 81U;
  vel.motor_id = 7;
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  assert(motor_control_submit_config(&ctx, &vel, t + 10U));
  t = feed_velocity_write(&ctx, 7, kR58Raw[2], t + 11U);
  assert(has_result_event(&ctx, 81U, MOTOR_PROTOCOL_CONFIG_OK));

  const uint8_t want = (uint8_t)((1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN) |
                                 (1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX) |
                                 (1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX));
  assert(entry_of(&ctx, 7)->record_dirty_slots == want);

  motor_protocol_config_request_t save = {};
  save.request_id = 82U;
  save.motor_id = 7;
  save.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  save.save_slots = want;
  assert(motor_control_submit_config(&ctx, &save, t + 20U));

  // 三条保存帧（码 1 / 4 / 2），无应答、逐拍推进；不得出现在 8 号节点上。
  const uint8_t kCodes[3] = {1U, 4U, 2U};
  uint8_t seen = 0U;
  for (uint32_t k = 0U; k < 8U && seen < 3U; ++k) {
    motor_control_service(&ctx, t + 21U + k);
    motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
    const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    for (size_t i = 0U; i < n; ++i) {
      assert(drain[i].can_id != 0x600U + 8U); // 未脏的台一个字节都不发
      if (drain[i].size == 8U && drain[i].data[0] == 0x23U &&
          drain[i].data[1] == 0x39U && drain[i].data[2] == 0x25U) {
        assert(drain[i].data[4] == kCodes[seen]);
        seen++;
      }
    }
  }
  assert(seen == 3U);
  assert(has_result_event(&ctx, 82U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->record_dirty_slots == 0U);
  assert(entry_of(&ctx, 8)->record_dirty_slots == 0U);
}

// 只脏易失槽（方向）：没有 0x2539 可发 → 内核直接清脏位并回 OK，不发任何帧。
// 方向靠记录才能在下次上电重建（记录会落 W25），所以它**必须有**未保存标记。
static void test_save_of_volatile_only_slot_emits_nothing() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  motor_protocol_config_request_t dir = {};
  dir.request_id = 83U;
  dir.motor_id = 7;
  dir.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  dir.direction_raw = 1U; // 与记录基线(0)不同 → 脏
  assert(motor_control_submit_config(&ctx, &dir, 200));
  const uint32_t t = feed_direction_write(&ctx, 7, 1U, 201U);
  assert(has_result_event(&ctx, 83U, MOTOR_PROTOCOL_CONFIG_OK));
  const uint8_t dirty = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  assert(entry_of(&ctx, 7)->record_dirty_slots == dirty);

  motor_protocol_config_request_t save = {};
  save.request_id = 84U;
  save.motor_id = 7;
  save.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  save.save_slots = dirty;
  assert(motor_control_submit_config(&ctx, &save, t + 10U));
  motor_control_service(&ctx, t + 11U);
  assert(has_result_event(&ctx, 84U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(entry_of(&ctx, 7)->record_dirty_slots == 0U);

  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (size_t i = 0U; i < n; ++i) {
    assert(!(drain[i].size == 8U && drain[i].data[1] == 0x39U &&
             drain[i].data[2] == 0x25U));
  }
}
```

(b) 助手 `feed_position_limits_write(ctx, node, tick)`：照 `feed_torque_write` 同形，四阶段写下限/读下限/写上限/读上限（子索引 1/2、槽位 0/1，值 `-10430` = `0xFFFFD742`、`+10430` = `0x000028BE`——±1.0 rad 按 65536 cnt/360°），收尾补一次 `service` + `collect_tx` 让事务收尾。

并在 `main()` 里挂上三个新用例。

(c) 既有用例改形状：`test_boot_normalize_not_advanced_by_manual_write()` 里的 `assert(motor_control_take_record_dirty(&ctx));`（约 3480 行）换成

```cpp
  // 归一写回"生效"(0) 改了记录内容 ⇒ 该台进"未保存"，等 system.save 落盘。
  assert(entry_of(&ctx, 7)->record_dirty_slots ==
         (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY));
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 --target test_motor_control
```
Expected: 编译失败，`record_dirty_slots` 无此成员。

- [ ] **Step 3: 表项与上下文**

`lib/motor_control/src/motor_control_internal.h`：

(a) `motor_control_entry_t` 的 `record_raw_present` 之后插入：

```c
  // 已写进设备并核验、但**尚未保存**的槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*）。
  // "保存"= 写设备 0x2539（驱动器落 flash）+ W25 落盘；两者都由 system.save 触发。
  // 含易失槽（保护开关 / 方向）：它们没有保存码，但**记录**要落 W25 才能生效
  //（方向尤其——上电归一的方向值就是从记录读的），所以照样置脏。
  // 写事务成功时置位，保存事务成功时清零。
  uint8_t record_dirty_slots;
```

(b) 删掉 `motor_control_context` 的 `record_dirty` 字段及其注释块。

- [ ] **Step 4: 写事务置脏、不再自动落盘**

`lib/motor_control/src/motor_control_config.c` 的 `update_record_after_write`：签名去掉未用的 `ctx`（改成 `static void update_record_after_write(motor_control_entry_t *entry)`），并把尾部的落盘信号换成脏位：

```c
  if (changed) {
    // 只置脏位，**不落盘**：设备侧 0x2539 与 W25 都等到人工点「保存」。
    // 不保存就掉电 → 两边一起回到上一轮已保存的值（设计 §3.2）。
    entry->record_dirty_slots |= written_mask;
  }
  entry->record_valid = true;
```

调用点（`motor_control_finish_tx`）同步改成 `update_record_after_write(entry);`。

- [ ] **Step 5: 保存事务受理**

同文件 `motor_control_submit_config` 的 `switch` 里，`SET_DIRECTION` 分支之后、`SET_POLICY` 之前插入：

```c
    case MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH: {
      // 保存 = 设备落盘 + W25 落盘。这里只管设备那一半。
      // 没有设备侧落盘对象的两种台（VAXOR 的软限位只活在记录里；只脏了易失槽
      // 的保护/方向——厂家没给保存码）**不发任何帧**，直接清脏位回 OK：它们要靠
      // 上层落 W25 才算保存完成，走事务机器只会白开一个空事务。
      if (entry->profile == NULL) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      const bool device_side =
          (entry->profile->capability_flags &
           MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U;
      const uint8_t device_slots =
          (uint8_t)(request->save_slots & MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK);
      if (!device_side || device_slots == 0U) {
        entry->record_dirty_slots = 0U;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
                                 entry->motor_id, entry->bus_index,
                                 MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
        return true;
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      motor_protocol_config_request_t bounded = *request;
      bounded.save_slots = device_slots;
      return start_tx(ctx, entry, &bounded, now_ms);
    }
```

> `profile == NULL` 先判、**不能**并进下面那个分支：未绑定的台没有"设备侧对象"概念，把它当成功静默清脏是错的。

`motor_control_finish_tx` 里，`entry->tx_active = false; entry->tx_awaiting = false;` 之前加：

```c
  // 保存事务成功：脏位清除（设备侧落盘已发出；W25 由上层在同一动作里落）。
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH &&
      result == MOTOR_PROTOCOL_CONFIG_OK) {
    entry->record_dirty_slots = 0U;
  }
```

- [ ] **Step 6: 删自动落盘**

(a) `lib/motor_control/include/motor_control.h`：删掉 `motor_control_take_record_dirty` 的声明与注释。
(b) `lib/motor_control/src/motor_control.c`：删掉 `motor_control_take_record_dirty` 的定义（约 1459 行）。
(c) `src/motor_rt_task.cpp`：

- 删掉 `g_save_pending` / `g_save_deadline_ms` / `kRecordSaveDebounceMs` 三个静态定义（约 69-71 行）；
- 删掉控制边界里那段去抖落盘：

```cpp
  // 设备侧写成功 = 设备与固件记录已同步，记录必须落盘才算"一起写"（否则
  // 断电后上电比对必然不一致）。去抖 500ms：页面连续改多项只落一次。
  if (motor_control_take_record_dirty(g_motor_control)) {
    g_save_pending = true;
    g_save_deadline_ms = now_ms + kRecordSaveDebounceMs;
  }
  if (g_save_pending && (int32_t)(now_ms - g_save_deadline_ms) >= 0) {
    g_save_pending = false;
    storage_task_request_save();
  }
```

替换为：

```cpp
  // 落盘**不再自动**：写事务只改内存记录 + 置脏位，"保存"由人工经 system.save
  // 触发（param_bridge 发 SAVE_DEVICE_FLASH 事务 + 落 W25）。此前这里 500ms 去抖
  // 自动落盘，于是"只点了写"与"点了保存"无法区分——正是 owner 要求消掉的行为
  //（设计 §3.2）。
```

- `storage_task.h` 的 include 保留（`storage_task_set_param_snapshot` / `storage_boot_params` 仍在用）。

- [ ] **Step 7: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
```
Expected: host 全绿；固件编译通过且无 unused 警告。

- [ ] **Step 8: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/motor_control micro_ros_firmware/src/motor_rt_task.cpp micro_ros_firmware/test/host/test_motor_control.cpp
git commit -m "feat(motor_control): 写与保存分离——写只置脏位，保存经 system.save 落盘"
```

---

### Task 5: 内核 —— 参数视图补报告所需字段

**Files:**
- Modify: `lib/motor_control/include/motor_control.h`（`motor_control_param_view_t`）
- Modify: `lib/motor_control/src/motor_control_internal.h`（`record_si`）
- Modify: `lib/motor_control/src/motor_control_params.c`
- Modify: `lib/motor_control/src/motor_control_config.c`
- Test: `test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 4 的 `record_dirty_slots`；既有 `entry->dev_si[]`、`dev_params_mismatch_mask()`。
- Produces: 视图新字段 `float dev_si[6]`、`float record_si[6]`、`uint8_t record_dirty_slots`、`uint8_t dev_mismatch_mask`；`dev_params_mismatch_mask` 改名 `motor_control_dev_mismatch_mask` 并去掉 static（在 `motor_control_internal.h` 声明）。

**为什么两个 SI 数组都要**：报告的"两行"必须同一把尺子才可比。设备侧 SI（`dev_si`）由 adapter 在回读/写核验时算好，透出即可；记录侧 SI（`record_si`）由内核在**记录被改的两个点**填——从 W25 套用时取 blob 已存的 SI 值，写事务成功后取事务请求的 SI 值。**不要在视图层按 raw 重算**：那会把 `Kt × 减速比` 系数错误一起吞掉（比对层刻意比 raw 就是为了这个，显示值不能比比对值乐观）。

- [ ] **Step 1: 写失败用例**

`test/host/test_motor_control.cpp`，新增：

```cpp
// 参数视图要把"两行"和"脏位""不一致位图"都端出来：DeviceParamReport 靠它填。
static void test_param_view_carries_report_fields() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  // make_device_record 只填 raw；blob 里 SI 与 raw 是同一笔事务产生的两个投影，
  // 这里照同一套换算补上（否则 record_si 全是 0，两行对不上的断言就变假断言）。
  rec.pos_min_rad = (float)motor_protocol_cerb_limit_cnt_to_rad(-32768);
  rec.pos_max_rad = (float)motor_protocol_cerb_limit_cnt_to_rad(32768);
  rec.velocity_limit_rad_s =
      (float)motor_protocol_cerb_motor_rpm_to_joint_rad_s(101.0, kR58Raw[2]);
  rec.torque_limit_nm =
      (float)motor_protocol_limit_ma_to_joint_nm(kR58LimitNmPerAmp, kR58Raw[3]);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  const uint32_t t = feed_device_params(&ctx, 7, kR58Raw, 120U, false);

  const motor_control_param_view_t *v = view_of(&ctx, 7);
  assert(v->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(v->dev_mismatch_mask == 0U);
  assert(v->record_dirty_slots == 0U);

  // 设备侧 SI：回读值换算（−32768 cnt = −180° = −π rad；10800 mA = 45 N·m）。
  assert(fabs(v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] +
              (float)MOTOR_PROTOCOL_PI) < 1e-4);
  assert(fabs(v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] - 45.0f) < 0.1f);
  // 记录侧 SI：与设备一致（同一组值的镜像），两行能逐格对上。
  assert(fabs(v->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] -
              v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN]) < 1e-4);
  assert(fabs(v->record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] -
              v->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX]) < 0.1f);

  // 写一项 → 脏位亮、两行仍然同值（写就是用记录的值写设备）。
  motor_protocol_config_request_t vel = {};
  vel.request_id = 60U;
  vel.motor_id = 7;
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  assert(motor_control_submit_config(&ctx, &vel, t + 10U));
  (void)feed_velocity_write(&ctx, 7, kR58Raw[2], t + 11U);
  assert(view_of(&ctx, 7)->record_dirty_slots ==
         (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX));
  assert(fabs(view_of(&ctx, 7)->record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX] -
              view_of(&ctx, 7)->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX]) < 1e-4);

  // 不一致时位图点名是哪一项（运行中外部改动设备的形状）。
  uint32_t bad[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  memcpy(bad, kR58Raw, sizeof(bad));
  bad[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] = 2000U;
  motor_protocol_config_request_t rf = {};
  rf.request_id = 61U;
  rf.motor_id = 7;
  rf.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  assert(motor_control_submit_config(&ctx, &rf, 300));
  (void)feed_device_params(&ctx, 7, bad, 301U, false);
  assert(view_of(&ctx, 7)->dev_mismatch_mask ==
         (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX));
}
```

并在 `main()` 里挂上。

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 --target test_motor_control
```
Expected: 编译失败，`dev_si` / `record_si` / `dev_mismatch_mask` 无此成员。

- [ ] **Step 3: 表项加 `record_si`**

`lib/motor_control/src/motor_control_internal.h`，`record_raw_present` 之后（`record_dirty_slots` 之前）插入：

```c
  // 记录的关节侧换算值（与 record_raw 同槽位索引）。比对一律比 raw（系数配错时
  // 两边一起错、比对恒通过），这份 SI 只供页面显示"固件这一行"——两行必须同
  // 单位才可比。来源两处：W25 套用（blob 已存 SI）与写事务收尾（事务请求本身
  // 就是 SI）。**不按 raw 重算**，否则显示值会比比对值乐观。
  float record_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
```

- [ ] **Step 4: 视图字段**

`lib/motor_control/include/motor_control.h`，`record_raw[6]` 之后追加：

```c
  // 设备端原值对应的关节侧换算值（与 dev_raw 同槽位索引）：DeviceParamReport
  // 的"驱动板"那一行。缺失槽位为 NaN。
  float dev_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  // 记录的关节侧换算值："固件"那一行。缺失槽位为 NaN。
  float record_si[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  // 已写、未保存的槽位位图（1u << MOTOR_PROTOCOL_PARAM_SLOT_*；0 = 已保存）。
  // 含易失槽（保护开关 / 方向）——它们没有设备保存码，但记录要落 W25 才生效。
  uint8_t record_dirty_slots;
  // 记录与设备**对不上**的槽位位图（口径同 DEV_INIT 事件的 detail）。
  // dev_init == MISMATCH 时它点名是哪几项；其余态为 0。
  uint8_t dev_mismatch_mask;
```

- [ ] **Step 5: 填充**

(a) `motor_control_config.c`：`static uint8_t dev_params_mismatch_mask(...)` → `uint8_t motor_control_dev_mismatch_mask(...)`（去 static、改名），两处调用点（`dev_params_match_record`、`apply_dev_init_result` 的 detail 计算）同步改名。`motor_control_internal.h` 声明：

```c
// 记录与设备回读逐槽位比对（设备端原值）。返回值 = 不一致槽位位图
// （1u << MOTOR_PROTOCOL_PARAM_SLOT_*；0 = 一致）。参数视图与 DEV_INIT 事件共用。
uint8_t motor_control_dev_mismatch_mask(const motor_control_entry_t *entry);
```

(b) `motor_control_params.c` 的 `fill_view`：把既有 `dev_raw` / `record_raw` 填充循环**合并扩成**一个循环（不要写两遍）：

```c
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    view->dev_raw[slot] =
        (view->dev_raw_present & bit) != 0U ? entry->dev_raw[slot] : 0U;
    view->record_raw[slot] =
        (view->record_raw_present & bit) != 0U ? entry->record_raw[slot] : 0U;
    // 缺失量一律 NaN：0 会在页面上冒充"限位设成了 0"，那是个能让人按下去的值。
    view->dev_si[slot] =
        (view->dev_raw_present & bit) != 0U ? entry->dev_si[slot] : NAN;
    view->record_si[slot] =
        (view->record_raw_present & bit) != 0U ? entry->record_si[slot] : NAN;
  }
```

并在 `view->dev_raw_present = ...` 之前插入：

```c
  view->record_dirty_slots = device_side ? entry->record_dirty_slots : 0U;
  view->dev_mismatch_mask =
      (device_side && entry->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH)
          ? motor_control_dev_mismatch_mask(entry)
          : 0U;
```

(c) `entry->record_si` 的两个填充点：

1. `apply_stored_to_entry`（记录内容捕获那个分支，`memcpy(record_raw)` 之后）：

```c
    // 记录侧的 SI 值：blob 里本来就存了（写设备时同一笔事务产生的），直接搬。
    // 缺字段的槽位保持 NaN。
    for (uint8_t i = 0U; i < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++i) {
      entry->record_si[i] = NAN;
    }
    if (rec->has_position_limits) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN] = rec->pos_min_rad;
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX] = rec->pos_max_rad;
    }
    if (rec->has_velocity_limit) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX] =
          rec->velocity_limit_rad_s;
    }
    if (rec->has_torque_limit) {
      entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] =
          rec->torque_limit_nm;
    }
    // 两个安全槽是枚举/标志位，原值即 SI（与 adapter 的 passthrough 同口径）。
    entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] =
        (float)rec->device_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    entry->record_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] =
        (float)rec->device_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
```

2. `update_record_after_write`（`motor_control_config.c`）的记录同步循环里，与 `record_raw[slot] = tx_write_raw[slot]` 并排补 SI——写事务的请求本身就是关节侧 SI，按槽位取；位置/速度/力矩走的是钳制后的 `tx_request`（`SET_POSITION_LIMITS` 落事务的是 `bounded`、速度/力矩落的是 `clamped`），故这里天然是钳制后的值：

```c
    // 同一笔事务产生的关节侧值（"固件"那一行）；两个安全槽是原值直通。
    switch (slot) {
      case MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN:
        entry->record_si[slot] =
            entry->tx_request.position_limits.min_position_rad;
        break;
      case MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX:
        entry->record_si[slot] =
            entry->tx_request.position_limits.max_position_rad;
        break;
      case MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX:
        entry->record_si[slot] = entry->tx_request.max_velocity_rad_s;
        break;
      case MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX:
        entry->record_si[slot] = entry->tx_request.max_torque_nm;
        break;
      case MOTOR_PROTOCOL_PARAM_SLOT_SAFETY:
      case MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION:
        entry->record_si[slot] = (float)entry->tx_write_raw[slot];
        break;
      default:
        break;
    }
```

- [ ] **Step 6: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
```
Expected: 全绿。

- [ ] **Step 7: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/motor_control micro_ros_firmware/test/host/test_motor_control.cpp
git commit -m "feat(motor_control): 参数视图补双行 SI、未保存位图与不一致位图"
```

---

### Task 6: 设备参数报告话题

**Files:**
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamEntry.msg`
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamReport.msg`
- Modify: `micro_ros_firmware/src/motor_bridge.cpp`
- Modify: `micro_ros_firmware/include/param_bridge.h` + `micro_ros_firmware/src/param_bridge.cpp`（暴露视图快照）

**Interfaces:**
- Consumes: Task 5 的视图字段（`dev_si` / `record_si` / `record_dirty_slots` / `dev_mismatch_mask` / `dev_init`）；`param_bridge_views()`。
- Produces: 话题 `micro_ros_up/device_param_report`；`can_bridge_msgs__msg__DeviceParamReport`。

- [ ] **Step 1: 消息定义**

`micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamEntry.msg`：

```
# 单台设备的参数核对双行快照（MCU → 上层，低频）。
#
# 两行必须同单位、同槽位顺序才可比，故都走关节侧 SI：位置 rad、速度 rad/s、
# 力矩 A（设备对象 0x2538 存的就是电流，力矩列按 A 呈现与参数面一致）。换算在
# 固件侧完成——页面不碰 cnt / mA / RPM。
#
# 缺失量一律 NaN（不填 0：0 在页面上是个能被人按下去的值）。
uint16 motor_id
# motor_control_dev_init_t：0=NA（本协议无设备对象）/1=PENDING/2=OK/
# 3=MISMATCH/4=UNVERIFIED/5=NO_RECORD（本机对该台无记录 → 固件列整行「无记录」）
uint8 dev_init_state
# 已写、未保存：设备侧 0x2539 与 W25 都没落盘。含保护开关/方向两槽——它们没有
# 设备保存码，但记录要落 W25 才生效（方向尤其靠记录在下次上电重建）。
bool record_dirty
# 记录与设备对不上的槽位位图（口径同 MotorEvent 的 DEV_INIT detail）：
# bit0=位置下限 bit1=位置上限 bit2=最大转速 bit3=峰值电流
# bit4=保护开关 bit5=方向。仅在 dev_init_state==MISMATCH 时非零。
uint8 mismatch_mask
# 固件记录侧（本机权威值）
float32 rec_pos_min
float32 rec_pos_max
float32 rec_vel_limit
float32 rec_torque_limit
float32 rec_safety
float32 rec_direction
# 驱动板回读侧（影子）
float32 dev_pos_min
float32 dev_pos_max
float32 dev_vel_limit
float32 dev_torque_limit
float32 dev_safety
float32 dev_direction
```

`micro_ros_agent/src/can_bridge_msgs/msg/DeviceParamReport.msg`：

```
# 参数核对用的双行快照全集（MCU → 上层，低频）。
#
# 为什么不塞进 motor_status_array：那是每台 20 Hz 的流，加 12 个 float 会让每条
# 状态消息涨约 50 B/台（48 台约 2.4 KB/条），而参数值几乎不变；"USB CDC 上行
# 带宽余量"本就是验证计划的开放项。也不走参数面：PARAM_BRIDGE_MAX_PARAMS=64、
# 每台 5 项，再加一列设备侧影子会把支持台数从 ~12 压到 ~7。低频专用话题也更贴
# 现有先例（MotorTrajectoryState 同为「MCU → 上层、低频」）。
#
# 发布时机：状态变化时（内核 CONFIG_RESULT / DEV_INIT 事件触发）即时一帧 + 1 Hz
# 心跳。**可靠 QoS**——48 台时本消息约 3 KB，超过一个 MTU(512 B)，best-effort
# 流不分片会被静默丢弃（与 motor_topology 同一处理）。
uint64 sync_time_ns
uint16 count
DeviceParamEntry[<=64] entries
```

`micro_ros_firmware/extra_packages/can_bridge_msgs` 是指向 `micro_ros_agent/src/can_bridge_msgs` 的符号链接，新 `.msg` 自动可见，**不需要**改 `platformio.ini` 或 CMakeLists。

- [ ] **Step 2: 刷新 micro-ROS 消息类型**

```bash
cd /home/ly/robot_ws/micro_ros_agent
python3 tools/refresh_microros_messages.py
```
Expected: 输出里列出新增的 `DeviceParamEntry` / `DeviceParamReport`；**不要**改用 `-t clean_microros`（联网重下整个环境）。

- [ ] **Step 3: 暴露视图快照**

`include/param_bridge.h` 追加（注意头文件在 `include/`，实现才在 `src/`）：

```c
// 最近一次 param_bridge_poll 的参数视图快照（只读，条数写入 *out_count）。
// 设备参数报告（motor_bridge）复用这一份，而不是自己再拷一份：视图数组
// 64 台约 6 KB，拷两份就是白占一份 RAM_D1。两者同在本任务里顺序调用，无并发。
const motor_control_param_view_t *param_bridge_views(size_t *out_count);
```

`src/param_bridge.cpp`：

```cpp
const motor_control_param_view_t *param_bridge_views(size_t *out_count) {
  if (out_count != nullptr) {
    *out_count = g_view_count;
  }
  return g_views;
}
```

**不要为了这个去调换两个 poll 的顺序**：`param_bridge.h` 的 `param_bridge_poll`
注释写明它排在 `motor_bridge_poll` **之后**，那是既有契约（动态注册、影子同步、
脉冲动作都在那一段之后才对，见该头文件）。本报告读的是**上一个 10ms 周期**的
快照，对 1 Hz 的心跳与"写完一项看结论变绿"都无影响。首个周期 `g_view_count == 0`
会发一帧空报告，无害。

- [ ] **Step 4: 发布者**

`src/motor_bridge.cpp`：

(a) 加 include：`#include <can_bridge_msgs/msg/device_param_report.h>`（按字母序放 `motor_command_array.h` 之前）。

(b) 话题名常量区：

```cpp
static const char *kDeviceParamReportTopic = "micro_ros_up/device_param_report";
```

(c) 发布者状态区（`g_trajectory_state_up` 之后）：

```cpp
static rcl_publisher_t g_device_param_pub;
static can_bridge_msgs__msg__DeviceParamReport g_device_param_msg;
// 每台 12 个 float + 若干小字段 ≈ 60 B；48 台约 3 KB。**静态而非栈上**：
// 发布在 micro_ros 任务栈上跑，栈放不下。
static can_bridge_msgs__msg__DeviceParamEntry
    g_device_param_entries[MOTOR_CONTROL_MAX_MOTORS];
static bool g_device_param_up = false;
// 事件触发的"马上发一帧"标志（见 publish_events）。
static bool g_device_param_pending = false;
```

(d) 文件头与 `motor_bridge_create` 的段注释里 `发布（4）` → `发布（5）`，并补一行：

```
//   micro_ros_up/device_param_report（1 Hz + 变化即发，参数核对双行快照）
```

(e) 填充函数（放在 `publish_trajectory_state` 之后）：

```cpp
// 设备参数核对快照：1 Hz 心跳 + 状态变化即时一帧。变化由内核事件触发
// （CONFIG_RESULT / DEV_INIT）——写一项参数之后页面要立刻看到结论变绿，
// 不必自己维护一份影子做比对。不在这里拷一份视图：复用 param_bridge 那一份。
static void publish_device_param_report(uint32_t now_ms) {
  static uint32_t s_phase = 0U;
  const bool heartbeat = (uint32_t)(now_ms - s_phase) >= 1000U;
  if (!heartbeat && !g_device_param_pending) {
    return;
  }
  if (heartbeat) {
    s_phase = now_ms;
  }
  g_device_param_pending = false;
  size_t view_count = 0U;
  const motor_control_param_view_t *views = param_bridge_views(&view_count);
  if (views == nullptr) {
    return;
  }
  const uint16_t count = (uint16_t)(view_count < kRosArrayCapacity
                                        ? view_count
                                        : kRosArrayCapacity);
  g_device_param_msg.entries.size = count;
  for (uint16_t i = 0U; i < count; ++i) {
    auto &out = g_device_param_msg.entries.data[i];
    const motor_control_param_view_t &v = views[i];
    out.motor_id = v.motor_id;
    out.dev_init_state = v.dev_init;
    out.record_dirty = v.record_dirty_slots != 0U;
    out.mismatch_mask = v.dev_mismatch_mask;
    out.rec_pos_min = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.rec_pos_max = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.rec_vel_limit = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    out.rec_torque_limit = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX];
    out.rec_safety = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.rec_direction = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
    out.dev_pos_min = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.dev_pos_max = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.dev_vel_limit = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    out.dev_torque_limit = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX];
    out.dev_safety = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.dev_direction = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  }
  g_device_param_msg.sync_time_ns = rmw_uros_epoch_nanos();
  note_publish(rcl_publish(&g_device_param_pub, &g_device_param_msg, nullptr));
}
```

(f) `publish_events()` 里登记变化（在 `param_bridge_on_config_result(&events[i]);` 之后）：

```cpp
      // 参数面结论会变的两类事件：请求落地、比对定论。报告是低频电平视图，
      // 只在有必要时才不等到下一个心跳。
      if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT ||
          events[i].type == MOTOR_CONTROL_EVENT_DEV_INIT) {
        g_device_param_pending = true;
      }
```

(g) `motor_bridge_create`：静态序列接线（放在 `g_topology_msg.entries` 那两行之后）：

```cpp
  g_device_param_msg.entries.capacity = kRosArrayCapacity;
  g_device_param_msg.entries.data = g_device_param_entries;
```

并在发布者初始化段（`g_trajectory_state_pub` 那块之后）加：

```cpp
  pub_opts = rcl_publisher_get_default_options();
  // **可靠**：48 台时本消息约 3 KB，超过一个 MTU(512 B)；best-effort 流不分片，
  // 会被静默丢弃（与 motor_topology 同一处理，那里也是 48 台量级）。
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_device_param_pub, node,
                         ROSIDL_GET_MSG_TYPE_SUPPORT(
                             can_bridge_msgs, msg, DeviceParamReport),
                         kDeviceParamReportTopic, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor);
    return false;
  }
  g_device_param_up = true;
```

(h) `motor_bridge_destroy` 尾部加：

```cpp
  if (g_device_param_up) {
    rcl_ret_t rc = rcl_publisher_fini(&g_device_param_pub, node);
    (void)rc;
    g_device_param_up = false;
  }
```

(i) `motor_bridge_poll` 尾部加 `publish_device_param_report(now_ms);`。

> `motor_bridge_executor_handle_count()` **不变**：只加了发布者，订阅数仍是 3。

- [ ] **Step 5: 编译并上板核对**

```bash
~/.platformio/penv/bin/pio run
```
Expected: 编译通过。

上板前置（顺序不可颠倒）：`fdcan_motor_sim.py --tty /dev/ttyACM1 --gazebo` 先在线当 ACK 节点 → 再烧录/复位控制板 → `micro_ros_agent serial --dev /dev/ttyACM0`。然后：

```bash
ros2 topic hz  /micro_ros_up/device_param_report      # 期望 ~1 Hz（有变化时更快）
ros2 topic echo --once /micro_ros_up/device_param_report
```
Expected: 四台各一条 entry；`rec_*` 与 `dev_*` 逐格相同或差值在量化误差内。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs micro_ros_agent/tools micro_ros_firmware/src/motor_bridge.cpp micro_ros_firmware/include/param_bridge.h micro_ros_firmware/src/param_bridge.cpp
git commit -m "feat(firmware): 新增 device_param_report 话题（参数核对双行快照）"
```

---

### Task 7: `system.save` 走保存事务 + `NO_RECORD` 错误码

**Files:**
- Modify: `src/param_bridge.cpp`

**Interfaces:**
- Consumes: Task 4 的 `MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH`、视图 `record_dirty_slots`（本文件内的 `g_views` 直接读，不经 Task 6 的 `param_bridge_views()`——那个访问器是给 motor_bridge 用的）。

**无 host 用例**：`param_bridge.cpp` 依赖 rclc / FreeRTOS，不在 host CTest 覆盖内（既有的 `system.save` / `system.refresh` 也没有）。验证靠编译 + 台架，判据写在 Step 4。

- [ ] **Step 1: 错误码**

`src/param_bridge.cpp` 顶部错误码区：

```cpp
// 错误码（system.last_error）：0=无；1..6 对齐 CONFIG 结果七码（reason 1..6）；
// 10=本地校验拒绝；11=mailbox 提交失败（FIFO 满/断链）；12=未绑定/未知电机；
// 20=save 失败；30=refresh 失败；40=设备参数与固件记录不一致（未初始化，该台
// 不可运动）；41=设备参数回读不成功（未核对，同样不可运动）；42=本机对该台
// **无记录**（首次上电/新板，同样不可运动，须人工核对后写参数）。
// 40/41/42 由内核结论驱动（dev_init），与请求回告无关：固件那份记录说了算。
#define ERR_NONE 0
#define ERR_LOCAL_VALIDATION 10
#define ERR_SUBMIT_FAILED 11
#define ERR_UNKNOWN_MOTOR 12
#define ERR_SAVE_FAIL 20
#define ERR_REFRESH_FAIL 30
#define ERR_DEV_INIT_MISMATCH 40
#define ERR_DEV_INIT_UNVERIFIED 41
#define ERR_DEV_INIT_NO_RECORD 42
```

"设备参数结论"上报块补一支：

```cpp
    if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH) {
      report_error(ERR_DEV_INIT_MISMATCH, g_views[i].motor_id);
    } else if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED) {
      report_error(ERR_DEV_INIT_UNVERIFIED, g_views[i].motor_id);
    } else if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_NO_RECORD) {
      report_error(ERR_DEV_INIT_NO_RECORD, g_views[i].motor_id);
    } else if (g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_OK ||
               g_views[i].dev_init == MOTOR_CONTROL_DEV_INIT_NA) {
      // 结论转好也上报（last_error=0 + motor_id）：上位机据此确认"这次回读
      // 成功"，否则一个陈旧的 40 会永远挂在那里，分不清新旧。
      report_error(ERR_NONE, g_views[i].motor_id);
    }
```

- [ ] **Step 2: 保存走事务**

`param_bridge_poll()` 里的 `g_save_pending` 分支：

```cpp
  if (g_save_pending) {
    g_save_pending = false;
    // 保存 = 设备侧落盘（0x2539，只对**有保存码的脏槽**）+ W25 落盘。
    // 只处理脏的台：没写过的台一个字节都不发（驱动器写 flash 期间会阻塞自身
    // 控制循环，实测 ~1s，白存一次就是白等一秒）。
    // 只脏易失槽（保护开关/方向）的台由内核在受理时直接清脏位回 OK——那种台
    // 只欠 W25 那一半，下面的 storage_task_request_save() 就是它的全部。
    for (size_t i = 0U; i < g_view_count; ++i) {
      if (g_views[i].record_dirty_slots == 0U) {
        continue;
      }
      control_mailbox_config_request_t request = {};
      request.request_id = next_request_id();
      request.received_tick_ms = millis();
      request.motor_id = g_views[i].motor_id;
      request.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;
      request.value.save_slots = g_views[i].record_dirty_slots;
      if (!motor_rt_submit_config(&request)) {
        report_error(ERR_SUBMIT_FAILED, g_views[i].motor_id);
      }
    }
    storage_task_request_save();
    set_bool_internal(kSystemSave, false);
  }
```

> `next_request_id()`（本文件约 156 行）就是 `do_refresh_pulse` / `do_set_zero_pulse` / `do_clear_fault_pulse` 用的那个取号函数。**必须沿用同一个**——内核按 `request_id` 认领回告，另起一套计数器会让 `_seq` 完成判据错乱。保存不需要 `_id` 回执（脏位由 `device_param_report.record_dirty` 反映），故不登记 `g_*_id`。

- [ ] **Step 3: 编译**

```bash
~/.platformio/penv/bin/pio run
```
Expected: 编译通过。

- [ ] **Step 4: 台架闭环**

前置：模拟器带 `--gazebo` 在线（ACK 节点）→ 烧录/复位控制板 → `micro_ros_agent`。四条判据：

```bash
# 1) 首次上电：四台判 NO_RECORD（42），不可使能
ros2 param get  /param_bridge system.last_error            # 42
ros2 topic echo --once /micro_ros_up/device_param_report   # dev_init_state==5

# 2) 写一项（例：m1.vel_limit）→ record_dirty=true，总线上**没有** 0x2539
ros2 param set /param_bridge m1.vel_limit 3.0
ros2 topic echo --once /micro_ros_up/device_param_report   # record_dirty == true

# 3) 保存 → record_dirty 翻 false，模拟器/嗅探日志里出现 0x2539（码 1）
ros2 param set /param_bridge system.save true

# 4) 写后不保存即复位 → 下次上电仍不通过（42 / 40）
```
Expected: 四步逐条对上。`0x2539` 的判据：SDO 请求 `0x600+node`、`data[1..2] == 39 25`、`data[4] == 保存码`（1 / 2 / 4）。嗅探用 `micro_ros_agent/tools/fdcan_sniffer.py`，或直接看模拟器日志。

- [ ] **Step 5: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/src/param_bridge.cpp
git commit -m "feat(param_bridge): system.save 下发保存事务；新增 NO_RECORD(42) 错误码"
```

---

### Task 8: 文档纠偏

**Files:**
- Modify: `micro_ros_firmware/docs/plans/active/param-authority-reversal-design.md`
- Modify: `micro_ros_firmware/README.md`
- Modify: `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md`
- Modify: `ros2_ws/src/web_host/README.md`
- Modify: `micro_ros_firmware/docs/plans/active/motor-runtime-validation.md`

- [ ] **Step 1: 规格状态**

`param-authority-reversal-design.md` 头部：

```markdown
**状态：Accepted（2026-09-21 实现）。** 反转 2026-09-17 决策 §6.3「设备参数以驱动板为准」。
实现计划：`param-authority-reversal-plan.md`。
```

- [ ] **Step 2: 固件 README**

`micro_ros_firmware/README.md`（约 342 行起）：小标题「**设备参数以驱动板为准（2026-09-17）**」改成

```markdown
**固件记录为权威（2026-09-21 反转 09-17 口径）**：上电/重连绑定完成后，内核自动对
CERB 发一次 `REFRESH_PARAMS` 回读六个管理对象，与 W25 里的固件记录**逐槽位比设备端
原值**（cnt/RPM/mA；比关节侧 SI 会在换算系数配错时两边一起错）。**要动的是驱动板，
不是记录**——初始化语义的来源是固件那份表。
```

紧随的三条判据改成：

```markdown
- 一致 → 初始化通过，正常可控；
- **初始化未完成一律不可使能**（`PENDING` 回读未收尾 / `MISMATCH` 不一致 /
  `UNVERIFIED` 回读失败 / `NO_RECORD` 本机无该台记录）：进 `CONFIG_CONFLICT`、
  拒绝 `SET_ENABLE`、不出运动帧，`system.last_error=40`（不一致）/`41`（未核对）/
  `42`（无记录）+ `last_error_motor` 指出是哪一台；
- **写 ≠ 保存**：从参数页写回该台参数只改设备 RAM 与固件记录（写后立即重判，
  写齐即转好），要**持久化**必须再点「保存」（写 `0x2539` + 落 W25）。未保存的台在
  `micro_ros_up/device_param_report` 上 `record_dirty=true`，页面标黄。
```

同段落里若有「不做"开机以设备为准静默覆写记录"」这类句子，一并按新方向改写。

- [ ] **Step 3: 决策日志**

`micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md`：

(a) §6.3 标题与首段：

```markdown
### 6.3 固件记录为设备参数的权威（2026-09-21 反转 09-17 口径）

CERB 的位置限位（`0x607D:01/:02`）、最大转速（`0x6080`）、峰值电流（`0x2538`）
存在驱动器里，但**固件内部维护的那份记录是权威**：驱动板参数要与固件对齐，初始化
语义的来源是固件而不是设备。本节 09-17 的原文（"设备参数以驱动板为准、固件记录只作
比对基准"）方向相反，**已由 owner 于 2026-09-21 明确反转**——那个方向会让"设备里被
外部工具改过的脏值"成为通过标准。
```

(b) 该节里 `不做"开机以设备为准静默覆写记录"——那会把设备里的脏值无声吞掉。` 改为

```markdown
  不引入"固件侧出厂默认值"：首次上电（W25 无该台记录）固件侧显示「无记录」、判
  未初始化，走人工核对；写下去之后记录即成立（owner 2026-09-21 定）。
```

(c) 该节末尾"写 = 设备与记录一起写"那条：改成写/保存分离的表述，并写明 `0x2539` 已挪进独立保存事务、500ms 去抖自动落盘已删。

(d) 决策表尾部追加一行：

```markdown
| 2026-09-21 | **参数权威方向反转（owner）**：固件记录为权威、驱动板向固件对齐（取代 09-17 §6.3 的反向口径）。配套三改：① `DEV_INIT` 增第五态 `NO_RECORD`——本机无该台记录不再落进放行的 `NA`，判未初始化、拒使能、`last_error=42`（"首次也当不一致，人工核对"）；② **写与保存分离**——写事务摘掉 `0x2539` 保存阶段，保存独立成 `SAVE_DEVICE_FLASH` 事务（按脏槽位选保存码 1/2/4，不写无关对象：驱动器写 flash 期间阻塞自身控制循环实测 ~1s），`system.save` 同时触发设备落盘与 W25 落盘，控制边界里 500ms 去抖自动落盘删除；③ 新增只读低频话题 `micro_ros_up/device_param_report` 承载「固件记录 / 驱动板回读」双行 SI 快照 + 未保存标记 + 不一致槽位位图。设计 `docs/plans/active/param-authority-reversal-design.md`，实现 `param-authority-reversal-plan.md`。 |
```

- [ ] **Step 4: Web README**

`ros2_ws/src/web_host/README.md`（约 23 行）「设备参数（下方左栏）」一行的开头 **以驱动板为准** 改为：

```
| 设备参数（下方左栏） | **以固件记录为准**，驱动板向记录对齐：位置 **rad**、速度 **rad/s**、力矩 **A**（设备对象 `0x2538` 存的就是电流；位置/速度在固件参数边界是直通，只有力矩过一次 A↔N·m 换算，见决策日志 §6.4）。**写 ≠ 保存**——写只改设备 RAM 与固件记录（写后立即重判），要持久化必须再点「保存」（发 `0x2539` + 落 W25），未保存的台标黄。两行对照取自只读话题 `micro_ros_up/device_param_report`（固件记录列 / 驱动板回读列，同槽位同单位），`dev_init_state` 5（无记录）时固件列整行显示「无记录」 |
```

同文件按 P2（web 大修）会重写整段；本次只改**方向性错误**（"以驱动板为准"与"来源列"两处），不提前做布局工作。

- [ ] **Step 5: 验证计划**

`micro_ros_firmware/docs/plans/active/motor-runtime-validation.md` 的 TBD-CONFIG-01 条目（约 97 行）：

- 给 `**对齐已按 owner 决定结案**（同日）：以设备为准重新基线（refresh ×4 → system.save → 复位）…` 那段加订正：

```markdown
  **【2026-09-21 订正】该结案按新口径是错结论**：当时只解决了"那一台设备与记录
  对不上"，恢复方向取的是"以设备为准重新基线"——而 owner 已于 2026-09-21 反转，
  权威在固件记录、驱动板向记录对齐。结案的操作（refresh ×4 → save → 复位）结果
  仍然成立（四台恢复可用），但**不构成"以后遇到不一致就这么办"的先例**。新口径见
  `param-authority-reversal-design.md`。
```

- "剩下"那行补上新增台架项：

```markdown
  **剩下（2026-09-21 追加）**：① 首次上电（记录为空）判 `NO_RECORD` / `last_error=42`
  且拒使能；② 写一项后总线上**无** `0x2539`、`device_param_report.record_dirty=true`；
  ③ 点 `system.save` 后 `0x2539`（码 1/2/4 按脏槽位）出现、`record_dirty` 翻 false、
  复位后不因记录陈旧而重报不一致；④ 写后不保存即复位 → 下次上电仍不通过。
```

- [ ] **Step 6: 交叉检查**

```bash
cd /home/ly/robot_ws
grep -rn "以驱动板为准\|设备参数以驱动板" --include=*.md micro_ros_firmware/ ros2_ws/src/web_host/ docs/
```
Expected: 只剩**历史快照**（`docs/reports/2026-09-19-*.md` 这类记录"当时是怎么做的"）里的出现；现行文档（README / 决策日志 / 验证计划 / 设计文档）里不得再有未标注的反向表述。

- [ ] **Step 7: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/README.md micro_ros_firmware/docs ros2_ws/src/web_host/README.md
git commit -m "docs: 参数权威方向反转（固件记录为权威）+ 写/保存分离 + 新话题"
```

---

## 自检

**Spec 覆盖**

| Spec 章节 | 落点 |
|---|---|
| §2 目标语义 1–4（记录权威、上电读、一致通过、不一致拦） | Task 3（`NO_RECORD` 判据 + 冻结）、Task 4（保存分离） |
| §2 第 5 条（web 同时显示两行） | Task 5（视图双行 SI）+ Task 6（话题）——**页面本身属 P2** |
| §2 第 6 条（下发修改后校验应通过） | Task 3 `settle_dev_init` 写事务收尾重判 + 用例 `test_dev_init_no_record_clears_after_write` |
| §2 第 7 条（要持久化得点保存） | Task 4（写不落盘）+ Task 7（`system.save`） |
| §2 第 8 条（首次显示"无记录"） | Task 3 Step 3 的 `NO_RECORD` + Task 6 的话题字段 |
| §2 第 9 条（未保存标记） | Task 4 脏位 + Task 5 视图字段 + Task 6 `record_dirty` |
| §3.1 状态机拆四态 | Task 3 |
| §3.2 写/保存分离、`0x2539` 挪位、删 500ms 去抖、保存码按槽位选 | Task 1（挪位）+ Task 4（分离与删去抖） |
| §3.3 新只读话题 | Task 6 |
| §3.4 给 P2 的接口契约 | Task 6 的字段即契约；P2 另立 spec |
| §4 四处文档纠偏 | Task 8 |
| §5 host CTest 五项 | Task 3（无记录拦；VAXOR 仍落 `NA` 放行由既有用例守）、Task 4（写不落盘 / 保存清脏 / 只保存脏台） |
| §6 不在本次范围 | 未越界 |

> §5 里「写后不保存即复位 → 下次上电仍不通过」**不适合 host 用例**（host 无 W25 与掉电语义）：由"写不落盘"（host 断言 W25 与 `0x2539` 都没发生）+ 台架第 4 条共同覆盖，不在 host 里造一个假复位。

**未决（留到 P2 / 真机）**

- 页面实现（布局重做、拆模块、真机安全操作面）——P2 另立 spec，本计划只把接口（`device_param_report` 的字段）钉死。
- `0x2539` 在真机上阻塞驱动器自身控制循环的影响——真机台架项，已挂在 `cerb-vendor-questions.md`。
- 48 台规模下 `device_param_report` 的带宽（约 3 KB/s，可接受）与 `motor_status_array` 既有的超 MTU 问题——不属本次范围。
