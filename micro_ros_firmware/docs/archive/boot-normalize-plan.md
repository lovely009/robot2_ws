# 上电归一（保护/方向）与方向参数 实现计划

> **已归档（2026-09-21）**：本计划已执行完毕。设计结论见
> [boot-normalize-design](../plans/active/boot-normalize-design.md)（仍为活规格），
> 残留真机项见 [运行时验证计划](../plans/active/motor-runtime-validation.md)。
> 正文保留作历史追溯，不再维护。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 固件上电时在设备参数回读比对**之前**把 CERB 的保护开关写成"生效"、方向写成 W25 记录值，并新增可读写的 `m<id>.direction` 参数（页面点按切换 CW/CCW），翻向后冻结使能直到重新设零。

**Architecture:** 在 `motor_control_config_step` 的启动链里插入 **0a 上电归一** 阶段（不变量式复检，每拍至多一台），复用现有配置事务机制发两个独立写事务（保护用已有的 `SET_SAFETY_PROTECT`，方向新增 `SET_DIRECTION`）；比对层新增"方向基线缺失"判据；冻结成因新增 `ZERO_STALE`；状态话题与参数面同时暴露保护/方向（话题只读遥测、参数面为控制入口）。

**Tech Stack:** STM32H723 固件（C17 库 + C++17 任务层）、PlatformIO、micro-ROS + rclc_parameter、CTest + cassert host 测试、FDCAN SLCAN 台架、rosbridge web 上位机（原生 JS）。

**Spec:** [`docs/plans/active/boot-normalize-design.md`](../plans/active/boot-normalize-design.md)

## Global Constraints

- 语言/规范：固件库是 **C17**，`src/*.cpp` 是 **C++17**；`-Wall -Wextra -Wpedantic` 零警告；2 空格缩进；snake_case（文件/函数/变量）、PascalCase（类型）、`k` 前缀常量；中文注释写**意图**（数据流/单位/安全边界），协议名与 ROS API 保留英文。
- host 测试（无 gtest，CTest + `assert`）：全部命令在 `micro_ros_firmware/` 内执行
  `cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2 && ctest --test-dir .host-build --output-on-failure`
  （当前基线 **19 项 CTest 全绿**；单测可直接跑二进制，如 `./.host-build/test_motor_control`）
- 固件构建：`~/.platformio/penv/bin/pio run`（**pio 不在 PATH**）。只有改 `platformio.ini` 编译宏或 `ldscript/*.ld` 才需要 `pio run -t clean`。
- **禁止** `~/.platformio/penv/bin/pio run -t clean_microros`（会联网重下环境）。
- 消息定义唯一来源：`micro_ros_agent/src/can_bridge_msgs/msg`。改完必须跑
  `micro_ros_agent/tools/refresh_microros_messages.py`（勿删 `libmicroros.a`）。
- 不写死拓扑：电机数量/型号/ID 只来自配置与设备自报身份；安全边界（限位/保护/方向）**不得在运动中变动**。
- 提交信息：conventional 前缀 + 中文描述（`feat(fw):` / `fix(fw):` / `test(fw):` / `docs:` / `feat(web):`），照仓库现有风格，**不加** 署名行。

---

## Task 1: ~~`ros2_ws/src/can_bridge_msgs` 改为符号链接~~ —— **已存在，无需实现**

**状态：作废（2026-09-20 执行期核实）。** 该符号链接**早已在仓库里**：
`ros2_ws/src/can_bridge_msgs -> ../../micro_ros_agent/src/can_bridge_msgs`，
git mode `120000`，由 `c3a970a`（2026-09-20 02:41）引入，是 HEAD 的祖先。
本任务初稿的"ros2_ws 那份是真实副本"来自一个**无效推理**：`ls -la <path>/` 带尾斜杠
会跟随链接、列出的是**目标目录**的内容，据此判不出链接。判定必须看父目录条目或 `readlink`。

保留的验证配方（一次性，别每次重跑）：

```bash
cd /home/ly/robot_ws
ls -la ros2_ws/src/ | grep can_bridge      # 期望首字符 l，目标 ../../micro_ros_agent/...
stat -c '%F' ros2_ws/src/can_bridge_msgs   # 期望：符号链接
git ls-files -s ros2_ws/src/can_bridge_msgs  # 期望 mode 120000
source /opt/ros/humble/setup.bash && (cd ros2_ws && colcon list | grep can_bridge_msgs)
```

两条推论随之更新：**改消息只需改 `micro_ros_agent/src/can_bridge_msgs/msg` 一处**；
Task 2 的提交范围里不再有"同步 ros2_ws 副本"这件事。

---

## Task 2: 消息字段——`confirm_unprotected` 与设备安全事实四字段

**Files:**
- Modify: `micro_ros_agent/src/can_bridge_msgs/msg/MotorPolicyTarget.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/msg/MotorStatusTarget.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg`（故障码表补 `6/7/8`）

**Interfaces:**
- Consumes: Task 1 的链接
- Produces: `MotorPolicyTarget.confirm_unprotected`(bool)、`MotorStatusTarget.{device_protect_known,device_protect_enabled,device_direction_known,device_direction}`；micro-ROS 生成类型 `can_bridge_msgs__msg__MotorStatusTarget` 的对应字段

- [ ] **Step 1: 改 `MotorPolicyTarget.msg`**

在 `float32 kd` 之后追加：

```
# 未带本位的武装请求，在该台设备侧保护**确知为关**时一律拒绝（2026-09-20 owner
# 决定）：关保护 = 该关节没有任何限位在执行（CERB 是设备侧限位，固件无软限位
# 兜底），所以每次武装都要一次显式确认。它不是代数门——`policy_generation`
# 承担"未武装 → 武装需新意图"，两者语义不同，不能互相顶替。
bool confirm_unprotected
```

- [ ] **Step 2: 改 `MotorStatusTarget.msg`**

在 `uint32 limit_flags` 之后、`bool online` 之前插入：

```
# 设备侧安全事实（只读遥测；与参数面 m<id>.limit_protect / m<id>.direction 同源
# ——都来自内核的设备影子，参数面是**控制入口**，本话题是**遥测**）。
# ⚠️ 这里放的是**正向语义**布尔，不是厂家原值：0x2650:00 的 `1 = 关闭` 是个
# 反向陷阱，反转只允许发生在 param_bridge 一处（见 param_name.h 的注释）。
# 本话题若放原值，页面就会被迫做第二次反转——那正是当初立规矩要避免的。
bool device_protect_known       # false = 本会话尚未核验过该台（值无意义，页面显示 --）
bool device_protect_enabled     # true = 保护生效（看门狗 + 限位保护在位）
bool device_direction_known
uint8 device_direction          # 0 = CW，1 = CCW（原值直通，无反转陷阱）
```

- [ ] **Step 3: 在 `MotorEvent.msg` 的 DEV_INIT 注释块里补全故障码表**

把 `MOTOR_CONTROL_FAULT_*` 的枚举（现在只列到 `5=离线`）补成：

```
#    0=NONE 1=协议 2=厂家 3=Profile 4=冲突 5=离线 6=设备参数未初始化(DEV_PARAM)
#    7=策略代数未前进(STALE_GENERATION) 8=方向已翻需重新设零(ZERO_STALE)
```

- [ ] **Step 4: 刷新 micro-ROS 消息类型**

```bash
cd /home/ly/robot_ws
python3 micro_ros_agent/tools/refresh_microros_messages.py
```

Expected: 脚本报告已增量刷新（**不要**改用 `-t clean_microros`）。

- [ ] **Step 5: 验证固件仍编译（新字段尚无消费者，应零改动通过）**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run
```

Expected: 编译成功。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs/msg micro_ros_firmware
git commit -m "feat(msgs): 策略加 confirm_unprotected，状态加设备保护/方向遥测四字段"
```

---

## Task 3: 协议层 `SET_DIRECTION`（kind + CERB 阶段表）

**Files:**
- Modify: `micro_ros_firmware/lib/motor_protocol/include/motor_protocol_adapter.h`（kind 枚举 + 请求字段）
- Modify: `micro_ros_firmware/lib/motor_protocol/src/cerb_adapter.c`（值来源枚举、阶段表、`config_stage_count`/`config_stage_at` 分发、写值分发）
- Test: `micro_ros_firmware/test/host/test_motor_protocol_adapter.cpp`

**Interfaces:**
- Consumes: 无
- Produces: `MOTOR_PROTOCOL_CONFIG_SET_DIRECTION = 13U`；`motor_protocol_config_request_t.direction_raw`（`uint32_t`，0=CW/1=CCW）；CERB 阶段表 `{WRITE 0x2541:00, READ 0x2541:00}`

- [ ] **Step 1: 写失败的测试**

在 `test/host/test_motor_protocol_adapter.cpp` 追加（形态照抄同文件 `test_cerb_safety_protect_write_frames`——它已经把 0x2650 的驱动方式、`config_encode_stage`、`config_stage_info` 的用法演示全了）：

```cpp
// 方向写事务（0x2541:00）：写 + 读回核验，**没有保存阶段**（易失，同 0x2650）。
// 写命令 `23 41 25 00 0X 00 00 00` 由 owner 手工核实（手册未给这个对象的写命令），
// 逐字节锁死；阶段数恰好 2——哪天有人"顺手补一条保存"会在这里被拦住。
static void test_cerb_direction_write_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  req.motor_id = 3U;
  req.direction_raw = 1U; // CCW
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 2U);

  motor_protocol_tx_frame_t frame = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
                                                         &req, &frame));
  const uint8_t kCcwCmd[8] = {0x23, 0x41, 0x25, 0x00, 0x01, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, kCcwCmd, 8U) == 0);

  req.direction_raw = 0U; // CW
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
                                                         &req, &frame));
  const uint8_t kCwCmd[8] = {0x23, 0x41, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, kCwCmd, 8U) == 0);

  // 阶段 1 是读回核验（0x40 = 加速上传请求），不是写。
  assert(motor_protocol_cerb_adapter.config_encode_stage(1U, profile, 3U, 0U,
                                                         &req, &frame));
  assert(frame.data[0] == 0x40U && frame.data[1] == 0x41U &&
         frame.data[2] == 0x25U && frame.data[3] == 0x00U);

  // 写阶段登记槽 5（内核据此把记录一起挪到新值）。
  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &req, &info));
  assert(info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION &&
         info.param_raw_valid && info.param_raw == 0U);
}
```

把该用例加进 `main()` 的调用列表。值域（0/1）不在这里测——那是 mailbox（Task 4）与内核提交门禁（Task 9）的职责。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake -S test/host -B .host-build && cmake --build .host-build -j2 2>&1 | tail -5
```

Expected: 编译失败——`MOTOR_PROTOCOL_CONFIG_SET_DIRECTION` 与 `direction_raw` 未定义。

- [ ] **Step 3: 实现**

`motor_protocol_adapter.h`：在 `MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT = 12U` 之后加逗号并追加

```c
  // 设备侧电机方向定义（属 mailbox 契约，与 control_mailbox 侧同值 13 锁死）：
  // CERB 写 0x2541:00 = 0（CW）/ 1（CCW），写后读回核验，**不跟 0x2539 保存
  // 帧**（同 0x2650：易失，掉电回上电初值）。值为**原值直通**，本层不做任何
  // 方向语义反转（方向没有 0x2650 那种"1 = 关"的反向陷阱）。
  // 仅 CERB；Vaxor 阶段数 0 → 内核回 UNSUPPORTED。
  MOTOR_PROTOCOL_CONFIG_SET_DIRECTION = 13U
```

同文件 `motor_protocol_config_request_t` 里，紧跟 `uint32_t safety_protect_raw;` 之后加：

```c
  // SET_DIRECTION：设备端原值（0 = CW，1 = CCW）。
  uint32_t direction_raw;
```

`cerb_adapter.c`：`CERB_VALUE_*` 枚举里 `CERB_VALUE_SAFETY_RAW` 之后加

```c
  CERB_VALUE_DIRECTION_RAW, // 写/读：0x2541:00 原值（0=CW，1=CCW），不换算
```

在 `kStagesSafetyProtect` 之后加阶段表：

```c
// 0x2541:00 方向：写 + 读回核验，**不跟 0x2539 保存阶段**（易失，见上）。
// 翻转方向会让设备里零点与 cnt 限位的物理含义整体镜像——内核据此冻结该台
// 直到重新设零（见 motor_control_config.c 的 ZERO_STALE）。
static const cerb_stage_desc_t kStagesDirection[2] = {
  CERB_DESC_WRITE(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                  MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
  // ⚠️ 读阶段也必须用 DIRECTION_RAW，**不能**用 PASSTHROUGH：读回核验的期望值
  // 由本阶段的 value_kind 反算（cerb_config_on_response → cerb_stage_write_value），
  // PASSTHROUGH 落 default: return 0 → 期望恒为 CW → 写 CCW 永远 RETRY/UNCONFIRMED。
  // （执行期实测复现；此处已按实现修正，与 kStagesSafetyProtect 同形。）
  CERB_DESC_READ(CERB_OBJ_MOTION_DIR, CERB_SUB_DEFAULT,
                 MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION, CERB_VALUE_DIRECTION_RAW),
};
```

`cerb_stage_table()` 的 `switch` 里加：

```c
    case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION:
      *count = (uint8_t)(sizeof(kStagesDirection) / sizeof(kStagesDirection[0]));
      return kStagesDirection;
```

`cerb_stage_write_value()` 的 `switch` 里，`CERB_VALUE_SAFETY_RAW` 分支之后加：

```c
    case CERB_VALUE_DIRECTION_RAW:
      return request->direction_raw;
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ./.host-build/test_motor_protocol_adapter && echo PASS
```

Expected: `PASS`。

- [ ] **Step 5: 提交**

```bash
git add lib/motor_protocol test/host/test_motor_protocol_adapter.cpp
git commit -m "feat(fw): 协议层新增 SET_DIRECTION（0x2541:00 写 + 读回核验）"
```

---

## Task 4: mailbox / motor_rt 契约与转发

**Files:**
- Modify: `micro_ros_firmware/lib/control_mailbox/include/control_mailbox.h`（枚举 + 载荷字段）
- Modify: `micro_ros_firmware/lib/control_mailbox/src/control_mailbox.c`（值校验）
- Modify: `micro_ros_firmware/src/motor_rt_task.cpp`（`static_assert` + `convert_config` 分支）
- Test: `micro_ros_firmware/test/host/test_control_mailbox.cpp`

**Interfaces:**
- Consumes: Task 3 的 `MOTOR_PROTOCOL_CONFIG_SET_DIRECTION` / `direction_raw`
- Produces: `CONTROL_MAILBOX_CONFIG_SET_DIRECTION = 13U`；`control_mailbox_config_request_t.value.direction_raw`

- [ ] **Step 1: 写失败的测试**

在 `test/host/test_control_mailbox.cpp` 追加（形态照该文件的 `make_mode_request` + `test_config_is_neutral_bounded_fifo`：先 `control_mailbox_t mailbox{}; control_mailbox_init(&mailbox);`，再逐条 `control_mailbox_submit_config`）：

```cpp
// 方向载荷只有 0/1 两态：越界值在 mailbox 就挡掉，别让它进总线。
static control_mailbox_config_request_t make_direction_request(uint32_t request_id,
                                                               uint16_t motor_id,
                                                               uint32_t raw) {
  control_mailbox_config_request_t request{};
  request.request_id = request_id;
  request.received_tick_ms = request_id * 10U;
  request.motor_id = motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_DIRECTION;
  request.value.direction_raw = raw;
  return request;
}

static void test_config_set_direction_value_range() {
  control_mailbox_t mailbox{};
  control_mailbox_init(&mailbox);

  const control_mailbox_config_request_t ccw = make_direction_request(1U, 3U, 1U);
  assert(control_mailbox_submit_config(&mailbox, &ccw));

  const control_mailbox_config_request_t cw = make_direction_request(2U, 3U, 0U);
  assert(control_mailbox_submit_config(&mailbox, &cw));

  const control_mailbox_config_request_t bad = make_direction_request(3U, 3U, 2U);
  assert(!control_mailbox_submit_config(&mailbox, &bad));

  const control_mailbox_config_request_t neg =
      make_direction_request(4U, 3U, 0xFFFFFFFFU);
  assert(!control_mailbox_submit_config(&mailbox, &neg));
}
```

把该用例加进 `main()` 的调用列表。

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 2>&1 | tail -5
```

Expected: 编译失败——`CONTROL_MAILBOX_CONFIG_SET_DIRECTION` 未定义。

- [ ] **Step 3: 实现**

`control_mailbox.h`：枚举尾部 `CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT = 12U` 改成带逗号并追加

```c
  // 设备侧电机方向定义（仅 CERB，写 0x2541:00）：载荷是**原值** 0 = CW /
  // 1 = CCW。仅未运动时受理，写后读回核验，不跟保存帧（易失）。
  CONTROL_MAILBOX_CONFIG_SET_DIRECTION = 13U
```

载荷结构体里，紧跟 `uint32_t safety_protect_raw;` 加：

```c
    // SET_DIRECTION：厂家/设备原值 0/1（0 = CW，1 = CCW），收原值不收布尔，
    // 与 SET_SAFETY_PROTECT 同口径——语义反转只允许发生在参数面一处。
    uint32_t direction_raw;
```

`control_mailbox.c` 的值校验 `switch` 里，`CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT` 分支之后加：

```c
    case CONTROL_MAILBOX_CONFIG_SET_DIRECTION:
      // 设备对象只有 0/1 两态；越界值在 mailbox 就挡掉，别让它进总线。
      return request->value.direction_raw <= 1U;
```

`motor_rt_task.cpp`：`SET_SAFETY_PROTECT` 的 `static_assert` 之后加

```cpp
static_assert(static_cast<int>(CONTROL_MAILBOX_CONFIG_SET_DIRECTION) ==
                  static_cast<int>(MOTOR_PROTOCOL_CONFIG_SET_DIRECTION),
              "mailbox/protocol config kind 契约不一致：SET_DIRECTION");
```

同文件 `convert_config()` 的 `switch` 里，`SET_SAFETY_PROTECT` 分支之后加

```cpp
    case CONTROL_MAILBOX_CONFIG_SET_DIRECTION:
      out->direction_raw = in->value.direction_raw;
      break;
```

并把该文件里那句 kind 契约注释（`// kind 数值契约一致（1..6、8..12；…）`）更新为 `1..6、8..13`。

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ./.host-build/test_control_mailbox && echo PASS
```

Expected: `PASS`。

- [ ] **Step 5: 提交**

```bash
git add lib/control_mailbox src/motor_rt_task.cpp test/host/test_control_mailbox.cpp
git commit -m "feat(fw): mailbox/motor_rt 打通 SET_DIRECTION 契约与载荷校验"
```

---

## Task 5: 内核状态视图——设备保护/方向四字段

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/include/motor_control.h`（`motor_control_status_t`）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control.c`（填充；`motor_control_status` 一带）
- Modify: `micro_ros_firmware/src/motor_bridge.cpp`（状态话题填 4 个新字段）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 2 的 `MotorStatusTarget` 生成类型字段名
- Produces: `motor_control_status_t.device_protect_known` / `.device_protect_enabled` / `.device_direction_known` / `.device_direction`

- [ ] **Step 1: 写失败的测试**

在 `test/host/test_motor_control.cpp` 追加：

```cpp
// 状态视图暴露设备侧安全事实（正向语义！）：出厂的 kR58Raw[4]=0（保护开）→
// enabled=true；未回读时两个 known 都是 false，值不得被当成真值。
static void test_status_exposes_device_safety_facts() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  motor_control_status_t st[MOTOR_CONTROL_MAX_MOTORS];
  assert(motor_control_copy_statuses(&ctx, st, MOTOR_CONTROL_MAX_MOTORS) == 1U);
  assert(st[0].device_protect_known);
  assert(st[0].device_protect_enabled); // 设备 raw 0 = 保护生效
  assert(st[0].device_direction_known);
  assert(st[0].device_direction == 0U); // CW

  // 手动关保护后：known 仍为真，enabled 翻假（正向语义取自设备 raw）。
  motor_protocol_config_request_t req = {};
  req.request_id = 91U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
  req.safety_protect_raw = 1U;
  motor_control_service(&ctx, 200);
  assert(motor_control_submit_config(&ctx, &req, 201));
  (void)feed_safety_write(&ctx, 7, 1U, 202);
  assert(motor_control_copy_statuses(&ctx, st, MOTOR_CONTROL_MAX_MOTORS) == 1U);
  assert(!st[0].device_protect_enabled);
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 2>&1 | tail -5
```

Expected: 编译失败——`device_protect_known` 等字段不存在。

- [ ] **Step 3: 实现**

`motor_control.h` 的 `motor_control_status_t` 尾部（`uint16_t last_failure_reason;` 之后）加：

```c
  // 设备侧安全事实（只读遥测；与参数面 m<id>.limit_protect / m<id>.direction
  // 同源，都是内核设备影子）。**正向语义**：enabled=true 表示保护生效，厂家
  // 原值（1 = 关）止步于协议层，不许泄漏到这一层。
  // known=false = 本会话尚未核验过该槽（回读/写后核验都没有），此时值无意义。
  bool device_protect_known;
  bool device_protect_enabled;
  bool device_direction_known;
  uint8_t device_direction; // 0 = CW，1 = CCW
```

`motor_control.c` 里填 `motor_control_status_t` 的那处（`out.unified_fault_code`/`protection_flags` 同一段）加：

```c
  const uint8_t safety_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  const uint8_t dir_bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  out.device_protect_known = (statuses[i].dev_raw_present & safety_bit) != 0U;
  // 设备原值 0 = 保护生效，向外一律给正向布尔。
  out.device_protect_enabled =
      out.device_protect_known &&
      statuses[i].dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 0U;
  out.device_direction_known = (statuses[i].dev_raw_present & dir_bit) != 0U;
  out.device_direction =
      (uint8_t)statuses[i].dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
```

并确认 `motor_control_param_view_t` → `motor_control_status_t` 那条投影路径里 `dev_raw`/`dev_raw_present` 确实带得过来（`motor_control_params.c` 的 `view->dev_raw_present = device_side ? entry->dev_raw_present : 0U;` 已在）；若 `motor_control_copy_statuses` 直接读表项，则按表项字段填。

`src/motor_bridge.cpp` 的 `out.unified_fault_code = ...` 同段加：

```cpp
    out.device_protect_known = statuses[i].device_protect_known;
    out.device_protect_enabled = statuses[i].device_protect_enabled;
    out.device_direction_known = statuses[i].device_direction_known;
    out.device_direction = statuses[i].device_direction;
```

- [ ] **Step 4: 跑 host 测试 + 固件编译**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
```

Expected: host 全绿；固件编译成功。

- [ ] **Step 5: 提交**

```bash
git add lib/motor_control include src/motor_bridge.cpp test/host/test_motor_control.cpp
git commit -m "feat(fw): 状态话题暴露设备侧保护/方向（正向语义只读遥测）"
```

---

## Task 6: 比对层——方向基线缺失判 `MISMATCH(bit5)`

**为什么**：方向是**易失**对象，设备每次上电都回 `0`（CW）。"设备当前读到 0"因此毫无信息量，唯一权威来源是 W25 记录。记录里没有这一槽（新烧板 / v2 迁移上来的旧 blob）时不许猜，判未初始化。

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_config.c`（`dev_params_mismatch_mask`）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`（新增 1 个用例 + 改写 `test_device_param_migrated_record_skips_safety_slots`）

**Interfaces:**
- Consumes: Task 5 无依赖；依赖既有 `entry->record_raw_present` / `record_raw`
- Produces: 方向槽缺失 → `MISMATCH` 位图含 `1u << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION`

- [ ] **Step 1: 改写既有用例 + 写新用例**

`test_device_param_migrated_record_skips_safety_slots` 现在断言"槽 4/5 未核验 → OK"。新口径下方向槽缺失必须判不一致。把它改成：

```cpp
// v2 迁移上来的记录只搬低 4 位（槽 4/5 = 未核验）。保护槽缺 → 不比对（上电
// 归一会把设备写成"开"）；**方向槽缺 → 判未初始化**：设备每次上电都回 0，
// "读到 0"不能证明任何事，方向只能由记录说了算。
static void test_device_param_missing_direction_baseline_blocks() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU, false);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(state == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)); // 只有方向
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
}

// 本机完全无记录（新烧板）同样判未初始化——不是"无记录可比就放过"。
// ⚠️ 这里**必须**显式声明"无记录"（apply_stored_params 传空数组），不能用
// complete_device_param_init：Task 8 之后那个助手会注入一份**含方向基线**的
// 记录（正常板子的路径），本用例要测的恰恰是"没有基线"。
static void test_device_param_no_record_blocks_on_direction() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t none = {};
  motor_control_apply_stored_params(&ctx, &none, 0U, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
}

// 在页面上点一次方向（写事务建立基线）→ 重判转好。
static void test_device_param_direction_write_establishes_baseline() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t none = {}; // 同上：显式无记录
  motor_control_apply_stored_params(&ctx, &none, 0U, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);

  motor_protocol_config_request_t req = {};
  req.request_id = 77U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  req.direction_raw = 0U; // 认下 CW
  motor_control_service(&ctx, 300);
  assert(motor_control_submit_config(&ctx, &req, 301));
  (void)feed_direction_write(&ctx, 7, 0U, 302);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}
```

同时新增测试侧助手（放在 `feed_safety_write` 之后）：

```cpp
// 方向写事务（写 0x2541:00 + 读回核验），与 feed_safety_write 同形。
static uint32_t feed_direction_write(motor_control_context_t *ctx, uint16_t node,
                                     uint32_t raw, uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack = sdo_write_ack(node, 0x2541U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t rd = sdo_read_ack(node, 5U, raw, tick + 3U);
  motor_control_on_raw_frame(ctx, &rd);
  motor_control_service(ctx, tick + 4U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 4U;
}
```

把这三个新用例加进 `main()` 的调用列表。

> 注意：`complete_device_param_init` 目前用"无记录"启动，本任务后它会判 MISMATCH。**Task 8 会把它改成带方向的记录**（其余依赖 `bring_cerb_to_ready` 的用例在 Task 8 一并恢复）——本任务先让 `ctest` 里的失败如实暴露出来，别在这里顺手改助手。

- [ ] **Step 2: 跑测试确认新用例失败、且失败面符合预期**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ctest --test-dir .host-build -R test_motor_control --output-on-failure 2>&1 | tail -20
```

Expected: 三个新用例断言失败；另外一批用 `bring_cerb_to_ready` 的用例也开始失败（预期内，Task 8 收口）。

- [ ] **Step 3: 实现**

`motor_control_config.c` 的 `dev_params_mismatch_mask` 改成：

```c
static uint8_t dev_params_mismatch_mask(const motor_control_entry_t *entry) {
  uint8_t mask = 0U;
  // 方向基线**独立于"本机有无记录"判定**：方向是易失对象，设备每次上电都回
  // 0（CW），"设备当前读到 0"零信息量——记录是它唯一的权威来源。无记录 /
  // 记录无此槽 / 槽值非法，一律等于"方向未知"，判未初始化（不猜 CW）。
  // 这是 owner 2026-09-20 的决定，也是 09-19 交接文档"无记录 = 未初始化"的落地。
  const uint8_t dir_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  if ((entry->record_raw_present & dir_bit) == 0U ||
      entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] > 1U) {
    mask |= dir_bit;
  }
  if (!entry->record_valid) {
    return mask; // 无记录：除方向基线外没有别的可比项
  }
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    ... 既有循环原样保留 ...
  }
  return mask;
}
```

- [ ] **Step 4: 跑测试**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ./.host-build/test_motor_control 2>&1 | tail -3
```

Expected: 三个新用例通过；`bring_cerb_to_ready` 相关用例仍失败（Task 8 收口）。此步**不要求**全绿。

- [ ] **Step 5: 提交**

```bash
git add lib/motor_control/src/motor_control_config.c test/host/test_motor_control.cpp
git commit -m "feat(fw): 方向基线缺失判未初始化（无记录/旧 blob 都不猜方向）"
```

---

## Task 7: 落盘幂等化（消除上电白擦）

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_config.c`（`update_record_after_write`）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: 无
- Produces: 值未变时不置 `ctx->record_dirty`（Task 8 依赖它，否则每次上电都擦两个 Flash 扇区）

- [ ] **Step 1: 写失败的测试**

```cpp
// 写下去的值与记录里已有的一致 → 不排落盘。上电归一会每次写保护"开"，
// 不做幂等化就等于每次上电白擦两个扇区（v2 单扇区 [实测] ~1s）。
static void test_device_write_same_value_skips_flash() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  // 记录里已有槽 4（保护=开，0），设备也是 0。
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, kR58Raw, 120, false);
  (void)motor_control_take_record_dirty(&ctx); // 清掉上电期的脏位

  motor_protocol_config_request_t req = {};
  req.request_id = 95U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
  req.safety_protect_raw = 0U; // 与记录同值
  motor_control_service(&ctx, 200);
  assert(motor_control_submit_config(&ctx, &req, 201));
  (void)feed_safety_write(&ctx, 7, 0U, 202);
  assert(has_result_event(&ctx, 95U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(!motor_control_take_record_dirty(&ctx)); // 没变 → 不落盘
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 && ./.host-build/test_motor_control 2>&1 | tail -3
```
Expected: 该用例在最后一行断言失败。

- [ ] **Step 3: 实现**

`update_record_after_write()` 里，把 `entry->record_raw_present |= written_mask;` 到 `ctx->record_dirty = true;` 一段改成"先比较、后置脏"：

```c
  // 先算"是否真的变了"：值相同且记录已有该位 → 不排落盘。上电归一每次都会写
  // 保护"开"，不做这一步就是每次上电白擦两个扇区（阻塞 v2 单扇区 [实测] ~1s）。
  bool changed = false;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    if ((entry->record_raw_present & bit) == 0U ||
        entry->record_raw[slot] != entry->tx_write_raw[slot]) {
      changed = true; // 首次建立基线也算变化
    }
  }
  entry->record_raw_present |= written_mask;
  entry->dev_raw_present |= written_mask;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    // 刚写并读回核验过：设备与记录同值。
    entry->record_raw[slot] = entry->tx_write_raw[slot];
    entry->dev_raw[slot] = entry->tx_write_raw[slot];
  }
  if (changed) {
    // 设备已核验写入且记录内容有变 → 落盘（不落盘就断电 = 下次上电比对必然
    // 不一致 = 该台未初始化，属于自找的停机）。
    ctx->record_dirty = true;
  }
  entry->record_valid = true;
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ./.host-build/test_motor_control 2>&1 | tail -3
```
Expected: 新用例通过（`bring_cerb_to_ready` 那批仍等 Task 8）。

- [ ] **Step 5: 提交**

```bash
git add lib/motor_control/src/motor_control_config.c test/host/test_motor_control.cpp
git commit -m "fix(fw): 设备写落盘幂等化，消除上电白擦两个扇区"
```

---

## Task 8: 上电归一阶段（0a）+ 测试基建适配

**这是本计划的核心任务。** 归一必须在 0b 回读**之前**完成：归一没成功 → 走不到回读 → 走不到运动授权。

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_internal.h`（步进常量 + 4 个表项字段）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_config.c`（0a 循环 + `finish_tx` 收尾分支）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_discovery.c:147-153`（重连复位）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`（新助手 + 新用例 + 全量适配）

**Interfaces:**
- Consumes: Task 3/4 的 `SET_DIRECTION`、Task 6 的方向基线判据、Task 7 的幂等落盘
- Produces: 表项字段 `boot_norm_stage`（`MOTOR_CONTROL_BOOT_NORM_{PROTECT,DIRECTION,DONE}`）、`boot_norm_request_id`、`boot_norm_attempts`、`boot_norm_retry_tick_ms`

- [ ] **Step 1: 加测试助手（先做基建，再写用例）**

在 `feed_safety_write` / `feed_direction_write` 之后加：

```cpp
// 走完一次上电归一（0a）：保护恒写 0；方向**仅在记录有槽 5 且值合法时**写，
// 否则整个方向步跳过（不猜方向，交给比对判基线缺失）。
// 必须在 feed_device_params 之前调用——0b 回读被 0a 阻塞。
// 读表项的 record_* 是白盒（本文件本就 -I 内核 src）。
static uint32_t feed_boot_normalize(motor_control_context_t *ctx, uint16_t node,
                                    uint32_t tick) {
  const motor_control_entry_t *e = entry_of(ctx, node);
  const uint8_t dir_bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  const bool has_dir =
      (e->record_raw_present & dir_bit) != 0U &&
      e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] <= 1U;
  const uint32_t dir_raw = e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];

  uint32_t t = feed_safety_write(ctx, node, 0U, tick); // 保护：恒写"生效"
  if (!has_dir) {
    motor_control_service(ctx, t);
    return t;
  }
  return feed_direction_write(ctx, node, dir_raw, t + 1U);
}
```

把 `feed_device_params` 的**开头**改成先跑归一：

```cpp
static uint32_t feed_device_params(motor_control_context_t *ctx, uint16_t node,
                                   const uint32_t raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT],
                                   uint32_t tick, bool keepalive,
                                   bool *out_saw_p0 = nullptr) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  // 上电归一（0a）必须先走完，0b 的回读才有发起时机。测试侧统一在这里前置，
  // 于是绝大多数既有用例无需改动——只有"故意不回读"的失败注入用例要显式处理。
  uint32_t t = feed_boot_normalize(ctx, node, tick);
  ... 其余原样，把原来的 `uint32_t t = tick;` 删掉 ...
}
```

把 `complete_device_param_init` 改成给一份**带方向基线**的记录：

```cpp
// 设备参数初始化收尾：先声明一份含方向基线的记录（真实板子第一次写过方向后
// 就是这个形状），再驱动自动回读与六槽比对。
// 不能再用"无记录"：方向基线缺失会判未初始化（见 dev_params_mismatch_mask），
// 那是产品行为，不是测试可以绕过的。
static void complete_device_param_init(motor_control_context_t *ctx, uint16_t node,
                                       uint32_t tick) {
  motor_control_stored_params_t rec = make_device_record(node, kR58Raw, 0x00U);
  motor_control_apply_stored_params(ctx, &rec, 1U, tick);
  (void)feed_device_params(ctx, node, kR58Raw, tick + 1U, false);
}
```

> `make_device_record(node, kR58Raw, 0x00U)` 的 `raw_present = 0` ⇒ 四个限值槽不参与比对，只有 `include_safety_slots` 默认带上的槽 4/5 有基线（都等于 `kR58Raw[4]=0` / `kR58Raw[5]=0`），与归一回读值一致。

- [ ] **Step 2: 写新用例**

```cpp
// 归一的顺序是契约：保护/方向写事务必须先于六槽回读发生。
static void test_boot_normalize_runs_before_refresh() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 逐拍看请求帧：前两条 SDO 写必须是 0x2650:00（保护）与 0x2541:00（方向），
  // 之后才允许出现 0x607D 的读。
  static const uint16_t kExpectIdx[2] = {0x2650U, 0x2541U};
  for (size_t k = 0U; k < 2U; ++k) {
    motor_control_service(&ctx, 120U + (uint32_t)k * 10U);
    motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
    const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
    bool seen = false;
    for (size_t i = 0U; i < n; ++i) {
      if (drain[i].dlc == 8U && drain[i].data[0] == 0x23U &&
          drain[i].data[1] == (uint8_t)(kExpectIdx[k] & 0xFFU) &&
          drain[i].data[2] == (uint8_t)(kExpectIdx[k] >> 8U)) {
        seen = true;
      }
      assert(!(drain[i].data[0] == 0x40U && drain[i].data[1] == 0x7DU)); // 还没到回读
    }
    assert(seen);
  }
  // 归一未完成期间，回读不得发起：此时表项仍停在 0a。
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_PENDING);
  assert(!entry_of(&ctx, 7)->dev_read_done);
}

// 归一失败（设备不应答写）→ 用尽重试判 UNVERIFIED(41)、不可使能，且**绝不**
// 发起回读——"归一没成功就绝不放行"这条是设计的安全内核。
static void test_boot_normalize_failure_blocks_before_refresh() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 只喂心跳、一个 SDO 应答都不给：归一写超时重试用尽。
  (void)run_heartbeat_only(&ctx, 7, 120U, 3000U);
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_UNVERIFIED);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(!motor_control_dev_params_allow_motion(e));
}

// 记录无方向槽：归一**跳过**方向步、不写 0x2541，回读照跑，比对判基线缺失。
static void test_boot_normalize_skips_direction_without_record() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  motor_control_stored_params_t rec = make_device_record(7, kR58Raw, 0x0FU, false);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);

  // 保护写 + 读回核验，然后**直接**是六槽回读（中间不许出现 0x2541 写）。
  uint32_t t = feed_safety_write(&ctx, 7, 0U, 120U);
  motor_control_service(&ctx, t);
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  const size_t n = motor_control_collect_tx(&ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  for (size_t i = 0U; i < n; ++i) {
    assert(!(drain[i].dlc == 8U && drain[i].data[0] == 0x23U &&
             drain[i].data[1] == 0x41U && drain[i].data[2] == 0x25U));
  }
  (void)feed_device_params(&ctx, 7, kR58Raw, t + 1U, false);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
}

// 运行中回读发现保护被外部工具关掉（归一之后的变化）→ MISMATCH 且位图点名 bit4。
// 原 test_dev_init_flags_safety_mismatch 在新设计下的正确形态：上电那一次不可能
// 再出现这个现象（归一先写开了），只有**运行中**的外部改动才走这条路。
static void test_dev_init_flags_safety_mismatch_at_runtime() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  uint32_t bad[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  memcpy(bad, kR58Raw, sizeof(bad));
  bad[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] = 1U; // 外部工具把保护关了
  (void)feed_device_params(&ctx, 7, bad, 400U, true);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_MISMATCH);
  uint16_t state = 0U;
  uint32_t mask = 0U;
  assert(take_dev_init_event(&ctx, 7, &state, &mask));
  assert(mask == (1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY));
}
```

把新用例加进 `main()`；**删除**旧的 `test_dev_init_flags_safety_mismatch`（被上面的运行中版本取代）。

- [ ] **Step 3: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ./.host-build/test_motor_control 2>&1 | tail -3
```
Expected: 新用例失败（`feed_boot_normalize` 已调用，但内核还没有 0a，于是没有写事务）。

- [ ] **Step 4: 实现内核**

`motor_control_internal.h`：加步进常量（放在 `MOTOR_CONTROL_P0_*` 一带）与表项字段（放在 `dev_read_*` 字段之后）：

```c
// 上电归一步骤（motor_control_entry_t::boot_norm_stage）。
#define MOTOR_CONTROL_BOOT_NORM_PROTECT 0U   // 待写保护 0x2650:00 = 0（生效）
#define MOTOR_CONTROL_BOOT_NORM_DIRECTION 1U // 待写方向 0x2541:00 = 记录值
#define MOTOR_CONTROL_BOOT_NORM_DONE 2U      // 归一完成（或该协议无设备对象）
```

```c
  // 上电归一（config_step 的 0a 阶段）：把设备里**易失**的两个安全对象写成
  // 定态——保护"生效"、方向取 W25 记录值。必须在 0b 回读比对**之前**完成，
  // 否则比对比的是设备上电初值，每次上电都会白报一次不一致。
  // 归一失败（重试用尽）判 DEV_INIT_UNVERIFIED：该台走不到回读，也就走不到
  // 运动授权。boot_norm_request_id 是"在飞的是不是归一步骤"的唯一判据——
  // 人工写保护用的是同一个 kind，按 kind 判会把人工写当成归一步进。
  uint8_t boot_norm_stage;
  uint32_t boot_norm_request_id;
  uint8_t boot_norm_attempts;
  uint32_t boot_norm_retry_tick_ms;
```

`motor_control_discovery.c` 的重连复位块（`entry->dev_read_retry_tick_ms = 0U;` 之后）加：

```c
    // 归一结论与设备影子同批作废：重连后要重新把易失对象写回定态。
    entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_PROTECT;
    entry->boot_norm_request_id = 0U;
    entry->boot_norm_attempts = 0U;
    entry->boot_norm_retry_tick_ms = 0U;
```

`motor_control_config.c` 的 `config_step`：在身份识别循环与 0b 循环之间插入 0a：

```c
  // 0a) 上电归一：保护写"生效"、方向写 W25 记录值，**必须在 0b 回读之前**。
  //     与 0b 同为不变量式复检、每拍至多一台、走总线一问一答串行。
  //     记录里没有方向槽（新烧板 / v2 迁移的旧 blob）时跳过方向步：方向只能由
  //     记录说了算，猜一个 CW 会让反装的关节朝反方向冲（比对层会判基线缺失）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->tx_active ||
        entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE) {
      continue;
    }
    if (entry->profile == NULL ||
        (entry->profile->capability_flags &
         MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
      continue; // 未绑定 / 非设备侧型号（VAXOR）：无设备对象可写
    }
    if (!entry->boot_record_offered || entry->motion_enabled) {
      continue; // 记录先落袋；运动中不动安全边界（防御，理论上到不了）
    }
    if (entry->boot_norm_attempts > 0U &&
        (int32_t)(now_ms - entry->boot_norm_retry_tick_ms) < 0) {
      continue; // 失败退避窗内
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // 运动中/异常态不归一（故障恢复回 READY_DISABLED 后会复检补做）
    }

    motor_protocol_config_request_t request = {0};
    request.motor_id = entry->motor_id;
    if (entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT) {
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 2U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
      request.safety_protect_raw = 0U; // 厂家原值 0 = 保护生效
    } else {
      const uint8_t dir_bit =
          (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      const uint32_t dir_raw = entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
      if ((entry->record_raw_present & dir_bit) == 0U || dir_raw > 1U) {
        entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_DONE; // 不猜方向
        continue;
      }
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 3U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
      request.direction_raw = dir_raw;
    }
    entry->boot_norm_request_id = request.request_id;
    (void)start_tx(ctx, entry, &request, now_ms);
    break;
  }
```

`motor_control_finish_tx()` 里，紧跟 `finish_identity_read` 的早返回之后、`bool dev_init_settled = false;` 之前插入归一收尾：

```c
  // 上电归一收尾：按 request_id 认领（人工写用的是同一 kind，不能按 kind 判）。
  // 失败沿用回读那套退避重试，用尽才定论 UNVERIFIED——一次总线抖动不该让该台
  // 在"初始化未完成不可使能"的口径下白白卡住。
  bool boot_norm_failed_offline = false;
  if (entry->boot_norm_request_id != 0U &&
      entry->tx_request_id == entry->boot_norm_request_id) {
    entry->boot_norm_request_id = 0U;
    if (result == MOTOR_PROTOCOL_CONFIG_OK) {
      entry->boot_norm_stage =
          entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT
              ? MOTOR_CONTROL_BOOT_NORM_DIRECTION
              : MOTOR_CONTROL_BOOT_NORM_DONE;
    } else {
      entry->boot_norm_attempts++;
      if (entry->boot_norm_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS) {
        boot_norm_failed_offline = true;
      } else {
        entry->boot_norm_retry_tick_ms = now_ms + MOTOR_CONTROL_DEV_READ_RETRY_MS;
      }
    }
  }
```

并在同函数下方 `dev_init_settled` 的判定链里，最前面加一条：

```c
  if (boot_norm_failed_offline) {
    // 归一写不进去 = 保护/方向没到位。该台按"未核对"收口（41），不发起回读、
    // 不与记录比对——比对的是"归一的成果"，归一没成就没有成果可比。
    dev_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
    entry->dev_read_done = true;
    dev_init_settled = true;
  } else if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    ... 既有分支 ...
```

> 注意 `if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS)` 那一支要改成 `else if`，其余不变。

- [ ] **Step 5: 跑 host 全量测试，修剩余适配**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure 2>&1 | tail -30
```

按失败清单逐条处理，已知的三类：

1. **"不回读"的失败注入用例**（`test_device_param_unverified_blocks_motion`、`test_device_param_read_retry_recovers` 等）：它们期望超时发生在**回读**阶段，现在归一阶段先超时。改法：在注入前先 `feed_boot_normalize(&ctx, node, tick)` 把归一走完，再进入原有的 `run_heartbeat_only` 流程。
2. **帧计数用例**（`test_device_param_four_reads_per_motor`、`test_..._no_further_sdo` 一类）：上电多出 1~2 条 SDO 写请求，断言要相应更新（并保留"回读结束后不再产生 SDO 请求"的原意——用 `feed_boot_normalize` 走完归一后再开始计数）。
3. **`test_device_param_partial_record_compare`**：第二台（ctx2/9）的期望结论可能因方向基线缺失而变成 MISMATCH——按新口径修正期望，并在用例注释里写明原因。

不要为了让测试变绿而放宽产品行为（例如把方向基线判据改成"有记录才判"）——那是设计决定，不是测试细节。

- [ ] **Step 6: 跑固件编译 + 确认全绿**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
ctest --test-dir .host-build --output-on-failure 2>&1 | tail -5
~/.platformio/penv/bin/pio run
```
Expected: 19 项 + 新增用例全绿；固件编译成功。

- [ ] **Step 7: 提交**

```bash
git add lib/motor_control test/host/test_motor_control.cpp
git commit -m "feat(fw): 上电归一——回读比对之前先写保护/方向，失败即不可使能"
```

---

## Task 9: 方向写事务的提交门禁、翻转冻结 `ZERO_STALE` 与设零解除

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/include/motor_control.h`（故障码 `8`；`internal.h` 用同一枚举）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_internal.h`（`zero_stale` 字段 + `allow_motion` 收紧）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_config.c`（提交分支、`update_record_after_write` 加 direction、翻转判据、`SET_ZERO` 解除收尾）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 3 的 `SET_DIRECTION`、Task 6/8 的比对与归一
- Produces: `MOTOR_CONTROL_FAULT_ZERO_STALE = 8U`；表项字段 `zero_stale`

- [ ] **Step 1: 写失败的测试**

```cpp
// 翻转方向 = 设备零点与 cnt 限位的物理含义整体镜像（原零点 P0 现在对应 -P0）。
// 设备侧限位仍会拦住"跑出范围"，拦不住"基于旧零点朝反方向猛冲"，所以冻结到
// 重新设零成功为止。
static void test_direction_flip_freezes_until_zero() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);
  assert(entry_of(&ctx, 7)->dev_init == MOTOR_CONTROL_DEV_INIT_OK);

  // 翻向 CCW（记录里原为 CW=0）。
  motor_protocol_config_request_t req = {};
  req.request_id = 71U;
  req.motor_id = 7;
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  req.direction_raw = 1U;
  motor_control_service(&ctx, 300);
  assert(motor_control_submit_config(&ctx, &req, 301));
  (void)feed_direction_write(&ctx, 7, 1U, 302);

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(has_result_event(&ctx, 71U, MOTOR_PROTOCOL_CONFIG_OK));
  assert(e->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] == 1U); // 基线与设备同值
  assert(e->zero_stale);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_ZERO_STALE);
  assert(!motor_control_dev_params_allow_motion(e)); // 武装会被拒

  // 设零成功 → 解除冻结、回到 READY_DISABLED。
  motor_protocol_config_request_t z = {};
  z.request_id = 72U;
  z.motor_id = 7;
  z.kind = MOTOR_PROTOCOL_CONFIG_SET_ZERO;
  assert(motor_control_submit_config(&ctx, &z, 400));
  (void)feed_set_zero(&ctx, 7, 401); // 测试助手：走完 0x2531 写事务
  assert(!entry_of(&ctx, 7)->zero_stale);
  assert(entry_of(&ctx, 7)->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
  assert(motor_control_dev_params_allow_motion(entry_of(&ctx, 7)));
}

// 上电归一把设备从 CW 写回 CCW（记录值）**不是**翻转：比较的是"写前记录值"，
// 记录没变就不该触发重设零——否则每次上电都要重新标定。
static void test_boot_normalize_direction_write_is_not_a_flip() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_bound(&ctx, 7, "cerb_r58_101", 100);
  uint32_t raw[MOTOR_PROTOCOL_PARAM_SLOT_COUNT];
  memcpy(raw, kR58Raw, sizeof(raw));
  raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] = 1U; // 记录：CCW
  motor_control_stored_params_t rec = make_device_record(7, raw, 0x0FU);
  motor_control_apply_stored_params(&ctx, &rec, 1, 110);
  (void)feed_device_params(&ctx, 7, raw, 120, false); // 内含归一步

  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
  assert(!e->zero_stale);
  assert(e->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED);
}
```

新增测试助手 `feed_set_zero`（0x2531 写 1，无读回核验阶段，命令式事务）：

```cpp
static uint32_t feed_set_zero(motor_control_context_t *ctx, uint16_t node,
                              uint32_t tick) {
  motor_protocol_tx_frame_t drain[MOTOR_CONTROL_MAX_TX_PENDING];
  motor_control_service(ctx, tick);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  motor_protocol_raw_frame_t ack = sdo_write_ack(node, 0x2531U, 0U, tick + 1U);
  motor_control_on_raw_frame(ctx, &ack);
  motor_control_service(ctx, tick + 2U);
  (void)motor_control_collect_tx(ctx, drain, MOTOR_CONTROL_MAX_TX_PENDING);
  return tick + 2U;
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 && ./.host-build/test_motor_control 2>&1 | tail -3
```
Expected: 编译失败或新用例断言失败（`zero_stale`/`feed_set_zero` 不存在）。

- [ ] **Step 3: 实现**

`motor_control.h` 故障码枚举尾部加：

```c
  // 方向被翻转后必须重新设零：设备里零点与 cnt 限位的物理含义已整体镜像。
  // 与 STALE_GENERATION 不同，它**是**安全状态（该台此刻不能动），因此进
  // unified_fault_code 并冻结 lifecycle；解除靠 SET_ZERO 成功，不靠"清错"。
  MOTOR_CONTROL_FAULT_ZERO_STALE = 8U
```

（把 `MOTOR_CONTROL_FAULT_STALE_GENERATION = 7U` 改成带逗号。）

`motor_control_internal.h`：表项加 `bool zero_stale;`（挨着 `dev_init`），并把放行判据收紧：

```c
static inline bool motor_control_dev_params_allow_motion(
    const motor_control_entry_t *entry) {
  // zero_stale：方向翻过但还没重新设零。零点基准已被镜像，此刻放行就是让
  // 电机基于旧零点朝反方向走一大段（设备限位拦得住"跑出范围"，拦不住这个）。
  return entry != NULL && !entry->zero_stale &&
         (entry->dev_init == MOTOR_CONTROL_DEV_INIT_NA ||
          entry->dev_init == MOTOR_CONTROL_DEV_INIT_OK);
}
```

`motor_control_config.c`：

1. `motor_control_submit_config()` 的 `switch` 里加提交分支（照 `SET_SAFETY_PROTECT` 写）：

```c
    case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION: {
      // 方向也是安全边界：与限位/保护同口径，仅未运动时受理（运动中翻向 =
      // 零点语义当场改变）。
      if (entry->profile == NULL || request->direction_raw > 1U ||
          !device_limit_write_allowed(entry)) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (entry->tx_active) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      return start_tx(ctx, entry, request, now_ms);
    }
```

2. `update_record_after_write()` 的 `switch` 加：

```c
    case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION:
      written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      break;
```

3. **翻转判据**放在 `motor_control_finish_tx()` 里、`update_record_after_write()` 调用**之前**（`else if (result == MOTOR_PROTOCOL_CONFIG_OK)` 分支开头）：

```c
  } else if (result == MOTOR_PROTOCOL_CONFIG_OK) {
    // 翻转判据必须在记录被覆盖之前算：比的是"写前的记录值"，不是设备值。
    // 上电归一写的正是记录值 → 不触发；只有操作员真的改参数才触发。
    if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_DIRECTION) {
      const uint8_t dir_bit =
          (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      if ((entry->record_raw_present & dir_bit) != 0U &&
          entry->tx_write_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] !=
              entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION]) {
        entry->zero_stale = true;
      }
    }
    update_shadow_after_write(entry);
    update_record_after_write(ctx, entry);
    ...
```

4. 冻结与解除：在 `motor_control_finish_tx()` 收尾处（`apply_dev_init_result` 调用之后）加一段：

```c
  if (entry->zero_stale &&
      entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
    motor_control_freeze_zero_stale(ctx, entry, now_ms);
  } else if (!entry->zero_stale &&
             entry->unified_fault_code == MOTOR_CONTROL_FAULT_ZERO_STALE &&
             entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
    // 设零成功解除：清成因、回 READY_DISABLED，重新走一遍常规授权（ARD §36：
    // 恢复运动仍要显式 SET_ENABLE）。
    entry->unified_fault_code = MOTOR_CONTROL_FAULT_NONE;
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_NONE;
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY,
                             entry->motor_id, entry->bus_index,
                             MOTOR_CONTROL_FAULT_NONE, 0U);
  }
```

并在同文件加冻结助手（放在 `apply_dev_init_result` 之后）：

```c
// 方向翻转后的冻结：与"设备参数未初始化"同形（清授权、清目标、拒绝使能），
// 但成因不同、解除方式也不同（SET_ZERO 成功即解，不需要点"清错"）。
static void motor_control_freeze_zero_stale(motor_control_context_t *ctx,
                                            motor_control_entry_t *entry,
                                            uint32_t now_ms) {
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
    motor_control_soft_stop_entry(ctx, entry, now_ms); // 防御：写门禁本应已挡住
  }
  entry->motion_enabled = false;
  entry->enable_pending = false;
  entry->has_target = false;
  entry->motion_authorized = false;
  entry->policy_enable = false;
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
  entry->unified_fault_code = MOTOR_CONTROL_FAULT_ZERO_STALE;
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_ZERO_STALE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DEV_INIT,
                           entry->motor_id, entry->bus_index,
                           (uint16_t)MOTOR_CONTROL_FAULT_ZERO_STALE, 0U);
}
```

5. `SET_ZERO` 成功清位：在 `motor_control_finish_tx()` 的 `else if (result == MOTOR_PROTOCOL_CONFIG_OK)` 里补一条（`update_record_after_write` 对 `SET_ZERO` 是 `default: return`，不会自己走到）：

```c
    if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_ZERO) {
      // 零点已重设 → 方向翻转的后果被消解，冻结在下方的收尾里解除。
      entry->zero_stale = false;
    }
```

> ⚠️ `SET_ZERO` 事务的 `result` 可能是 `UNCONFIRMED`（协议无读回对象时 `finish_tx_success` 的判据）——确认 CERB 的 `kStagesSetZero` 是否只写不读；若只写，`confirmed` 判据会让结果停在 `UNCONFIRMED`，那就必须按 `MOTOR_PROTOCOL_CONFIG_UNCONFIRMED` 也算成功来清位。**先跑一次 `test_direction_flip_freezes_until_zero` 看实际结果码**，再决定这里收哪些码——不要凭猜写。

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure 2>&1 | tail -5
~/.platformio/penv/bin/pio run
```
Expected: 全绿 + 编译成功。

- [ ] **Step 5: 提交**

```bash
git add lib/motor_control test/host/test_motor_control.cpp
git commit -m "feat(fw): 方向翻转冻结到重新设零（ZERO_STALE），设零成功即解除"
```

---

## Task 10: 保护关闭时的武装二次确认门禁

**Files:**
- Modify: `micro_ros_firmware/lib/motor_control/include/motor_control.h`（新成因码 `9`、`motor_control_policy_target_t` 加字段）
- Modify: `micro_ros_firmware/lib/motor_control/src/motor_control_config.c`（`motor_control_apply_policy_entry` 门禁 + `SET_ENABLE` 分支同口径）
- Modify: `micro_ros_firmware/src/motor_rt_task.cpp`（策略话题 → 内核请求的字段搬运）
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp` + `test/host/test_policy_mailbox.cpp`

**Interfaces:**
- Consumes: Task 5 的设备影子（`dev_raw`/`dev_raw_present`）
- Produces: `MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED = 9U`；`motor_control_policy_target_t.confirm_unprotected`

- [ ] **Step 1: 写失败的测试**

```cpp
// 手动关保护后仍允许武装，但每次都要带显式确认位：关保护 = 该关节没有任何
// 限位在跑（CERB 是设备侧限位，固件无软限位兜底）。
static void test_enable_without_protection_needs_confirm() {
  motor_control_context_t ctx;
  motor_control_init(&ctx, g_adapters, 2);
  bring_cerb_to_ready(&ctx, 7, "cerb_r58_101", 100);

  // 手动关保护（设备 raw = 1）。
  motor_protocol_config_request_t off = {};
  off.request_id = 61U;
  off.motor_id = 7;
  off.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
  off.safety_protect_raw = 1U;
  motor_control_service(&ctx, 300);
  assert(motor_control_submit_config(&ctx, &off, 301));
  (void)feed_safety_write(&ctx, 7, 1U, 302);

  // 不带确认位武装 → 拒绝，且只是"本机拒绝"，不进统一故障码。
  motor_control_policy_target_t t = {7U, true, 1.0f, 0.1f, false};
  t.confirm_unprotected = false;
  motor_control_policy_snapshot_t snap = {};
  snap.received_tick_ms = 400U;
  snap.request_id = 500U;
  snap.policy_generation = 1U;
  snap.target_count = 1U;
  snap.targets[0] = t;
  assert(motor_control_apply_policy(&ctx, &snap, 401U));
  const motor_control_entry_t *e = entry_of(&ctx, 7);
  assert(e->last_failure_reason == MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED);
  assert(e->unified_fault_code == MOTOR_CONTROL_FAULT_NONE); // 不是设备故障
  assert(!e->policy_enable);

  // 带确认位（代数继续前进）→ 受理。
  snap.request_id = 501U;
  snap.policy_generation = 2U;
  snap.targets[0].confirm_unprotected = true;
  assert(motor_control_apply_policy(&ctx, &snap, 402U));
  assert(entry_of(&ctx, 7)->policy_enable);
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -5
```
Expected: 编译失败（字段/常量不存在）。

- [ ] **Step 3: 实现**

`motor_control.h`：故障码枚举加 `MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED = 9U`（注释写明"只进 last_failure_reason，与 STALE_GENERATION 同待遇"）；`motor_control_policy_target_t` 加 `bool confirm_unprotected;`。

`motor_control_config.c` 的两处使能门禁（`motor_control_apply_policy_entry` 与 `SET_ENABLE` 分支）各加：

```c
  if (request->enable && !motor_control_dev_params_allow_motion(entry)) { ... 既有 ... }
  // 保护被手动关掉的台仍允许武装，但每次都要显式确认：关保护 = 该关节没有
  // 任何限位在跑（CERB 是设备侧限位，固件无软限位兜底）。判据是**设备真值**
  // （影子），不是参数面读数——后者可能还没同步。
  if (request->enable && motor_control_unprotected_needs_confirm(entry) &&
      !request->confirm_unprotected) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED;
    return reject_invalid(ctx, entry, request, now_ms);
  }
```

同文件加判据助手：

```c
// 该台保护是否**确知为关**（设备原值 1 = 关）。未核验过（无位）不算"确知"，
// 那种情况本来就到不了使能路径（设备参数未初始化会先拦）。
static bool motor_control_unprotected_needs_confirm(
    const motor_control_entry_t *entry) {
  const uint8_t bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  return (entry->dev_raw_present & bit) != 0U &&
         entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U;
}
```

`motor_rt_task.cpp`：策略话题解码处把 `confirm_unprotected` 搬进内核目标（`MotorPolicyTarget` → `motor_control_policy_target_t`），并确认 `SET_POLICY` 的 config 请求结构里也带得下来（`motor_protocol_config_request_t` 加 `bool confirm_unprotected;`，`convert_config` 搬运）。

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure 2>&1 | tail -5
~/.platformio/penv/bin/pio run
```

- [ ] **Step 5: 提交**

```bash
git add lib/motor_control src/motor_rt_task.cpp lib/motor_protocol test/host
git commit -m "feat(fw): 保护关闭时武装需显式确认（每次都要），拒因不进统一故障码"
```

---

## Task 11: 参数面——`m<id>.direction` 与 6 项/台

**Files:**
- Modify: `micro_ros_firmware/lib/param_store/include/param_name.h`（枚举追加 + 校验）
- Modify: `micro_ros_firmware/lib/param_store/src/param_name.c`（名字表 + `param_value_ok`）
- Modify: `micro_ros_firmware/src/param_bridge.cpp`（注册表、`view_field`、`field_config_kind`、提交分支）
- Modify: `micro_ros_firmware/include/param_bridge.h`（容量注释 + `PARAM_BRIDGE_MAX_PARAMS`）
- Test: `micro_ros_firmware/test/host/test_param_store.cpp`

**Interfaces:**
- Consumes: Task 3/4 的 `SET_DIRECTION`
- Produces: 参数名 `m<id>.direction`；`PARAM_FIELD_DIRECTION`；`PARAM_BRIDGE_MAX_PARAMS = 72U`，`kMaxMotors = 10`

- [ ] **Step 1: 写失败的测试**

在 `test/host/test_param_store.cpp` 追加：

```cpp
// 方向是布尔量（0/1），不做钳制；越界值一律拒。
static void test_param_value_ok_direction() {
  assert(param_value_ok(PARAM_FIELD_DIRECTION, 0.0f, 0.0f, nullptr));
  assert(param_value_ok(PARAM_FIELD_DIRECTION, 1.0f, 0.0f, nullptr));
  assert(!param_value_ok(PARAM_FIELD_DIRECTION, 2.0f, 0.0f, nullptr));
  assert(!param_value_ok(PARAM_FIELD_DIRECTION, -1.0f, 0.0f, nullptr));
  assert(!param_value_ok(PARAM_FIELD_DIRECTION, NAN, 0.0f, nullptr));

  // 名字生成/解析往返。
  char buf[PARAM_NAME_MAX_LEN];
  assert(param_name_build_motor(buf, sizeof(buf), 12U, PARAM_FIELD_DIRECTION));
  assert(strcmp(buf, "m12.direction") == 0);
  uint16_t id = 0U;
  param_field_t f = PARAM_FIELD_ENABLE;
  assert(param_name_parse(buf, &id, &f) == PARAM_NAME_MOTOR);
  assert(id == 12U && f == PARAM_FIELD_DIRECTION);
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cmake --build .host-build -j2 2>&1 | tail -5
```
Expected: 编译失败（`PARAM_FIELD_DIRECTION` 不存在）。

- [ ] **Step 3: 实现**

`param_name.h`：`PARAM_FIELD_LIMIT_PROTECT` 之后（**`PARAM_FIELD_COUNT` 之前**）追加：

```c
  // 设备侧电机方向（0x2541:00）：0 = CW，1 = CCW。**原值直通、不做语义反转**
  // ——它没有 0x2650 那种"1 = 关"的反向陷阱，别为了对称多加一层。
  // 追加在 COUNT 前：present 位图与注册顺序是契约，不重排既有字段。
  // 值持久化在 device_raw[槽 5]（见 param_store_blob_set_device_raw）。
  PARAM_FIELD_DIRECTION,
```

`param_name.c`：字段名表加 `"direction"`（与枚举同序，第 8 项）、`param_value_ok` 加分支：

```c
    case PARAM_FIELD_LIMIT_PROTECT:
      return value == 0.0f || value == 1.0f; // 布尔量，不钳制
    case PARAM_FIELD_DIRECTION:
      return value == 0.0f || value == 1.0f; // 同上：0=CW / 1=CCW
```

`src/param_bridge.cpp`：
- `kParamMotorFields[]` 加 `PARAM_FIELD_DIRECTION`，`static_assert(PARAM_MOTOR_FIELD_COUNT == 6U, ...)`
- `field_config_kind()` 加 `case PARAM_FIELD_DIRECTION: return CONTROL_MAILBOX_CONFIG_SET_DIRECTION;`
- `submit_*` 里加方向提交（照 `submit_safety_protect`，但**不反转**）：

```cpp
// 方向提交（原值直通；越界由 param_value_ok 挡在前面）。
static bool submit_direction(MotorSlot *slot, double value) {
  if (!param_value_ok(PARAM_FIELD_DIRECTION, (float)value, 0.0f, nullptr)) {
    report_error(ERR_LOCAL_VALIDATION, slot->motor_id);
    return false;
  }
  control_mailbox_config_request_t request = {};
  request.request_id = next_request_id();
  request.received_tick_ms = millis();
  request.motor_id = slot->motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SET_DIRECTION;
  request.value.direction_raw = (value == 1.0) ? 1U : 0U;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, slot->motor_id);
    return false;
  }
  param_name_buf_t name;
  (void)param_name_build_motor(name, sizeof(name), slot->motor_id,
                               PARAM_FIELD_DIRECTION);
  get_double_or_nan(name, &slot->old_value[PARAM_FIELD_DIRECTION]);
  const param_field_t fields[1] = {PARAM_FIELD_DIRECTION};
  mark_outstanding(slot, request.request_id, fields, 1);
  return true;
}
```
并在 `param_on_modification` 的 `switch (field_config_kind(field))` 里加 `case CONTROL_MAILBOX_CONFIG_SET_DIRECTION: slot->old_value[field] = old_value; return submit_direction(slot, new_value);`

- `view_field()` 加（**正向/原值都直通，未知给 NaN**）：

```cpp
    case PARAM_FIELD_DIRECTION: {
      // 与 limit_protect 同口径：设备还没回读到该槽位时给 NaN（页面显示 -- 并
      // 禁用），**不给默认值**——猜一个 CW 会让操作员以为方向已确认。
      const uint8_t bit = (uint8_t)(1U << PARAM_DEVICE_SLOT_DIRECTION);
      if ((v->dev_raw_present & bit) == 0U) {
        return NAN;
      }
      return (double)v->dev_raw[PARAM_DEVICE_SLOT_DIRECTION];
    }
```

`include/param_bridge.h`：`PARAM_BRIDGE_MAX_PARAMS` 改 `72U`，并把容量注释整段更新为「72 槽 = 10 system + 10 电机 × 6 项」、把算式与 `heap_free ≈ 79.6 KB` 写进去；`kMaxMotors` 的说明同步（`(72-10)/6 = 10`）。

- [ ] **Step 4: 跑测试 + 编译（参数面容量断言是编译期，必须过 `pio run`）**

```bash
cmake --build .host-build -j2 && ctest --test-dir .host-build --output-on-failure 2>&1 | tail -5
~/.platformio/penv/bin/pio run
```
Expected: 全绿 + 编译成功（`static_assert(PARAM_MOTOR_FIELD_COUNT == 6U)` 与 `kMaxMotors` 推导均通过）。

- [ ] **Step 5: 台架留一行堆账**

上板后读一次 `system.heap_free`（`ros2 param get /micro_ros_node system.heap_free`），把实测值记进 `docs/plans/active/motor-runtime-validation.md`（预期 ~79 KB，与 64 槽时的 ~82 KB 相差约 2.6 KB）。

- [ ] **Step 6: 提交**

```bash
git add lib/param_store src/param_bridge.cpp include/param_bridge.h test/host/test_param_store.cpp
git commit -m "feat(fw): 参数面加 m<id>.direction，容量 64→72 保住 10 台"
```

---

## Task 12: 模拟器易失模型 + 探针写模式

**Files:**
- Modify: `micro_ros_agent/tools/fdcan_motor_sim.py`
- Modify: `micro_ros_agent/tools/cerb_sdo_probe.py`
- Test: `micro_ros_firmware/test/host/test_fdcan_motor_sim.py`

**Interfaces:**
- Consumes: 无
- Produces: 模拟器上电初值 `0x2650:00 = 1`（照真机）；探针 `--write IDX:SUB=VAL`

- [ ] **Step 1: 写失败的测试**

在 `test/host/test_fdcan_motor_sim.py` 追加：

```python
def test_cerb_power_on_defaults_match_real_device():
    """模拟器必须照**真机**建模，不是照固件假设建模（2026-09-19 的教训）。

    真机台架四台上电实测：0x2650:00 = 1（保护**关**），0x2541:00 = 0（CW）。
    保护开关是易失的，固件靠上电归一把它写成 0 —— 所以模拟器初值必须是 1，
    否则台架根本复现不了"上电后保护是关的"这个前提。
    """
    sim = CerbModel(1)  # 用该文件既有的构造方式
    assert sim.sdo_params[(0x2650, 0x00)] == 1
    assert sim.sdo_params[(0x2541, 0x00)] == 0
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
python3 test/host/test_fdcan_motor_sim.py 2>&1 | tail -5
```
Expected: `0x2650` 断言失败（当前初值是 `0`）。

- [ ] **Step 3: 实现**

`fdcan_motor_sim.py` 的 `sdo_params` 初值改成：

```python
                # 0x2650:00 保护开关：**上电初值 = 1（关）**，与真机台架四台
                # 2026-09-19 实测一致。它易失（0x2539 保存码对它无效），固件靠
                # 上电归一在回读之前写成 0（生效）——模拟器照对端建模，初值给 0
                # 会让台架永远复现不出这个场景。
                (0x2650, 0x00): 1,
                # 0x2541:00 方向：0 = CW，1 = CCW。同样易失，上电回 0；固件在
                # 对比之前写记录值。
                (0x2541, 0x00): 0,
```

并在文件头部 `sdo_params` 的注释块里补一句"进程重启 = 掉电复现（初值即上电初值）"。

`cerb_sdo_probe.py`：
1. **模块 docstring 第 1 行**从"只发加速上传请求，**永不写设备**"改成"默认只读；`--write` 显式写设备"——**这条不改就是撒谎**，探针现在能写设备了。
2. 加写方法（照 `read_once` 的形态）：

```python
    def write_once(self, index: int, sub: int, value: int,
                   timeout: float = 0.3):
        """发一条 4 字节快速 SDO 写（0x23）+ 读回核验。

        返回 (状态, 说明)。状态 ∈ {"ok", "abort", "timeout", "mismatch"}。
        **会改设备对象**：只给明确知道后果的对象用（0x2650:00 保护、0x2541:00 方向）。
        """
        want = SDO_RESPONSE_ID_BASE + self.node
        payload = int(value).to_bytes(4, "little", signed=True)
        req = bytes([0x23, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload
        os.write(self.fd, slcan_line(self.node + SDO_REQUEST_ID_BASE, req, True, True))
        for can_id, data in self.drain(timeout):
            if can_id != want or len(data) < 8:
                continue
            ccs = data[0] & 0xE0
            got_idx = data[1] | (data[2] << 8)
            if got_idx != index or data[3] != sub:
                continue
            if ccs == 0x80:
                return "abort", int.from_bytes(data[4:8], "little")
            if ccs == 0x60:
                status, _, raw, _ = self.read(index, sub)
                if status == "ok" and raw != value:
                    return "mismatch", raw
                return "ok", raw
        return "timeout", None
```

3. CLI 加 `--write`（`IDX:SUB=VAL`，十六进制索引 + 十进制或 `0x` 值）与执行分支，且在写之前**仍然**跑 `foreign_master_seen` 守卫（写比读更不能撞帧）。

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
python3 test/host/test_fdcan_motor_sim.py 2>&1 | tail -3
python3 -c "import ast,sys; ast.parse(open('/home/ly/robot_ws/micro_ros_agent/tools/cerb_sdo_probe.py').read())" && echo "probe 语法 OK"
```

- [ ] **Step 5: 提交**

```bash
git add micro_ros_agent/tools micro_ros_firmware/test/host/test_fdcan_motor_sim.py
git commit -m "test(tools): 模拟器保护初值改 1（照真机）+ 探针加显式写模式"
```

---

## Task 13: 台架验收（人工，含"第一项不通就停"的门）

**这一步只有 owner 能做**（真机 + CANable + 掉电操作）。前 12 项未完成前不要开始。

**台架前提**（照 `docs/guides/fdcan1-bench-guide.md`）：先让 CANable 成为 ACK 节点再复位控制板；**烧录前先停模拟器**（MCU 挂起期间模拟器仍在发帧而无人 ACK，会把 CANable 打进错误态）。

- [ ] **Step 1: 门：先验 `0x2541` 写命令与易失性（不通就停下重新设计）**

```bash
cd /home/ly/robot_ws/micro_ros_agent
python3 tools/cerb_sdo_probe.py --dev /dev/ttyACM1 --node 2 \
    --read 0x2541:00 --write 0x2541:00=1 --read 0x2541:00
```
Expected: 写返回 `ok`（读回核验 = 1）。
然后**断该台电机电、重新上电**，再读一次：

```bash
python3 tools/cerb_sdo_probe.py --dev /dev/ttyACM1 --node 2 --read 0x2541:00
```
Expected: 回到 `0`（易失确认）。
**任一步不符**：停下来找我，方向那半边的设计要重做（先改 spec，再改代码）。

- [ ] **Step 2: 冷启动观察归一事件序**

```bash
# 终端 1：固件 + agent（照 AGENTS.md 的台架顺序）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# 终端 2：观察
ros2 topic echo /micro_ros_up/motor_event        # 期望：DEV_INIT reason=2（已核对）
ros2 topic echo /micro_ros_up/motor_status_array # 期望：device_protect_enabled=true
ros2 param get /micro_ros_node m1.direction      # 期望：0 或 1（不是 NaN）
```
Expected: 上电后保护为**开**（不改任何东西），方向 = 记录值，`last_error = 0`。
记录本次上电耗时（归一给每台多 4 个 SDO 阶段）与 `system.heap_free`。

- [ ] **Step 3: 跑全通路**

```bash
cd /home/ly/robot_ws/micro_ros_agent
python3 tools/fdcan_motor_sim.py --help   # 确认易失初值改造已生效
ros2 launch motor_hardware_interface hardware.launch.py
```
Expected: `colcon test` 与真机 bring-up 均正常；`ros2 param get /micro_ros_node system.heap_free` ≈ 79 KB 上下。

- [ ] **Step 4: 手动关保护 → 武装被拒 → 带确认成功**

页面：取消某台"限位保护"→ 点"武装" → 期望提示二次确认；绕过页面用命令行复现：

```bash
ros2 param set /micro_ros_node m1.limit_protect 0.0   # 关
ros2 topic pub --once /micro_ros_do/motor_policy can_bridge_msgs/msg/MotorPolicyArray \
  "{...}"   # 不带 confirm_unprotected
ros2 param get /micro_ros_node system.last_error      # 期望 0（本机拒绝不进统一故障码）
ros2 param get /micro_ros_node system.last_error_motor
# 事件里看 CONFIG_RESULT / 状态里看 last_failure_reason = 9
```
Expected: 不带确认位被拒；带上 `confirm_unprotected: true` 后受理。

- [ ] **Step 5: 翻方向 → 冻结 → 设零 → 恢复**

```bash
ros2 param set /micro_ros_node m1.direction 1.0   # 翻到 CCW
ros2 topic echo /micro_ros_up/motor_status_array  # 期望 lifecycle=7、unified_fault_code=8
ros2 param set /micro_ros_node system.set_zero 1  # 设零
ros2 topic echo /micro_ros_up/motor_status_array  # 期望 lifecycle=3、fault=0
```
Expected: 冻结与解除均如预期；**再上电一次**确认方向保持为记录值（CCW）、且没有再报 `ZERO_STALE`。

- [ ] **Step 6: 清空记录后上电（基线缺失路径）**

清 W25 记录（或换一块没写过的板）→ 上电 → 期望 `EVENT_DEV_INIT` reason=3、位图 = 32（bit5）、`last_error=40`、拒绝使能；在页面上点一次方向 → 恢复 `OK`。

- [ ] **Step 7: 把结果写进 validation plan**

把上面每步的实测值（上电耗时、`heap_free`、状态话题 10 台是否仍能收发）追加到
`micro_ros_firmware/docs/plans/active/motor-runtime-validation.md`，并勾掉对应条目。

- [ ] **Step 8: 提交**

```bash
git add micro_ros_firmware/docs/plans/active/motor-runtime-validation.md
git commit -m "docs(fw): 上电归一/方向/二次确认台架验收记录"
```

---

## Task 14: web 上位机（方向列、二次确认、标红）

**Files:**
- Modify: `ros2_ws/src/web_host/web/app.js`
- Modify: `ros2_ws/src/web_host/web/index.html`（表头加"方向"列）
- Modify: `ros2_ws/src/web_host/README.md`（参数列表更新）

**Interfaces:**
- Consumes: Task 11 的参数 `m<id>.direction`；Task 2 的 `MotorPolicyTarget.confirm_unprotected`
- Produces: 无（终端用户界面）

- [ ] **Step 1: 参数表加"方向"列**

`PARAM_NUM_FIELDS` 保持数值四项；方向单独渲染（它不是数字输入框）。在 `renderDevRows()` 的 `protectCell` 之后加：

```javascript
    // 方向：原值直通（0=CW / 1=CCW），点一下切换。切换会镜像设备零点与 cnt
    // 限位的物理含义，所以要点确认、且固件会冻结该台直到重新设零。
    const dir = Number(v['direction']);
    const dirKnown = Number.isFinite(dir);
    const dirCell = dirKnown
      ? `<button class="tiny-btn" data-dir="${id}" data-dir-cur="${dir}">` +
        `${dir === 1 ? 'CCW' : 'CW'}</button>`
      : '<button class="tiny-btn" disabled>--</button>';
```

在 `renderDevRows()` 末尾的事件绑定处加：

```javascript
  tbody.querySelectorAll('[data-dir]').forEach((btn) => {
    btn.onclick = () => flipDirection(parseInt(btn.dataset.dir, 10),
      parseInt(btn.dataset.dirCur, 10));
  });
```

并实现：

```javascript
// 方向切换：写 m<id>.direction 的反值。翻转会让设备里零点与 cnt 限位的物理
// 含义整体镜像（原零点 P0 对应 -P0），固件会冻结该台直到重新设零（ZERO_STALE）。
async function flipDirection(id, cur) {
  const next = cur === 1 ? 0 : 1;
  const name = next === 1 ? 'CCW' : 'CW';
  if (!window.confirm(
    `把 m${id} 的方向切成 ${name}？\n\n` +
    '方向一翻，设备里零点与位置限位的物理含义整体镜像：该台会被固件冻结' +
    '（拒绝武装），必须先重新设零才能运动。')) {
    return;
  }
  try {
    const r = await paramSetValue(`m${id}.direction`, 3, 'direction', next);
    if (!r.results[0].successful) {
      setParamResult(false, `m${id}.direction 被拒：${r.results[0].reason || '见 last_error'}`);
      return;
    }
    setParamResult(true, `m${id} 方向已下发 ${name}；受理后需重新设零才能运动`);
    setTimeout(loadParams, 400); // 等写事务收尾后刷新
  } catch (e) {
    setParamResult(false, `m${id}.direction 失败：${e.message}`);
  }
}
```

- [ ] **Step 2: 武装时的二次确认 + 状态标红**

`sendPolicyRow()` 里，在 `pub('policy', ...)` 之前插入：

```javascript
  // 保护被关掉的台：该关节没有任何限位在跑（设备侧限位，固件无软限位兜底），
  // 每次武装都要显式确认。固件侧也有同一道门禁（缺确认位直接拒），页面这道
  // 只是把"为什么按不下去"提前说清楚。
  const protect = Number((paramCache.get(id) || {})['limit_protect']);
  const unprotected = Number.isFinite(protect) && protect === 0;
  if (el.enable.checked && unprotected) {
    if (!window.confirm(
      `m${id} 的设备侧限位保护当前是**关闭**的：\n` +
      '该关节没有任何限位在执行（含看门狗自锁）。确认要武装它吗？')) {
      el.enable.checked = false;
      return;
    }
  }
```

并在 `pub('policy', ...)` 的 target 里带上确认位：

```javascript
    targets: [{motor_id: id, enabled: el.enable.checked, kp, kd,
               confirm_unprotected: el.enable.checked && unprotected}],
```

保护关闭的行标红：在 `renderDevRows()` 的 `src`/`protectCell` 一带加一个类名（例如给 `<tr>` 加 `class="unprotected"`），并在 `style.css` 里给它一个醒目的状态色（照既有 `pill off` 的配色变量），行内文案："保护已关：该关节无任何限位"。

- [ ] **Step 3: 槽位掩码与文案同步**

```javascript
const DEV_SLOT_PARAM_MASK = 0x3F;   // 槽 0..5 全部有参数项了（方向已进参数面）
// DEV_SLOT_EXTERNAL_MASK 删除：不再有"只能用 cangaroo 改"的槽位
```

`devRecoveryText()` 改成：

```javascript
// 恢复口径：六个槽位都在参数面里了。方向槽没有基线时（新烧板/旧 blob），
// 点一次方向即建立基线。
function devRecoveryText(mask) {
  if ((mask & (1 << 5)) !== 0) {
    return '方向(0x2541)无基线：在参数表点一次方向（CW/CCW）即建立基线并恢复';
  }
  return '写回对应项即可恢复';
}
```

`FAULT_NAMES` 补齐到 8：`['无','协议','厂家','Profile','冲突','离线','设备参数','策略代数','方向待设零']`。

- [ ] **Step 4: 人工验证（无自动化测试）**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch web_host web_host.launch.py    # 或按该包 README 的启动方式
```
要点：方向按钮显示 CW/CCW 并点击弹确认；保护关的行标红；武装弹确认；`--` 显示在未回读时生效。

- [ ] **Step 5: 提交**

```bash
git add ros2_ws/src/web_host
git commit -m "feat(web): 方向列可切换 CW/CCW、保护关闭标红与武装二次确认"
```

---

## Task 15: 文档收口

**Files:**
- Modify: `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md`
- Modify: `micro_ros_firmware/README.md`
- Modify: `micro_ros_firmware/AGENTS.md`
- Modify: `micro_ros_firmware/docs/plans/active/cerb-vendor-questions.md`
- Modify: `micro_ros_firmware/docs/plans/active/w25-blob-v3-design.md` §8.2（"未实现的加固方向"标注已实现）
- Modify: `docs/reports/2026-09-19-firmware-param-surface-handoff.md`（§一.3 / §二 标注被取代）
- Modify: `micro_ros_firmware/docs/plans/active/motor-runtime-validation.md`（指向新计划的台架项）

- [ ] **Step 1: 决策日志落条目（含 09-19 那张从未落盘的决定表）**

新增一段（照该文件既有的表格式与日期列）：

```markdown
| 2026-09-20 | **上电归一 + 方向进参数面（owner 逐条拍板）**。①驱动板上电默认**开**保护，关才需要手动；归一写发在**比对之前**（保护恒写 0、方向写 W25 记录值）。②方向易失（owner 确认；09-19 的"同形"此前只是推断）。③方向无记录**不猜**（承接 09-19：判 `CONFIG_CONFLICT`，位图 bit5）。④翻转方向 → 冻结到 `SET_ZERO` 成功（新成因 `ZERO_STALE=8`），因为设备零点与 cnt 限位的物理含义被整体镜像。⑤手动关保护后**仍允许武装**，但每次需 `confirm_unprotected` 显式确认、页面持续标红——**取代** 09-19 的"使能时检查必须为 0"。⑥保护/方向**同时**进状态话题（只读遥测，正向语义）与参数面（`m<id>.limit_protect` / `m<id>.direction`，控制入口）——**部分取代** 09-19 的"只进话题、撤掉参数项"。设计：`docs/plans/active/boot-normalize-design.md`。 |
```

- [ ] **Step 2: README 与 AGENTS.md**

- `README.md`：把"设备侧保护开关与方向纳入上电比对（2026-09-19）"那一节改写成"上电归一（2026-09-20）"，写明顺序契约、`m<id>.direction`、`0x2541` 写命令与易失、以及"关保护后武装需确认"。
- `AGENTS.md`：在坑位清单里加一条——**"0a 上电归一必须在 0b 回读之前，顺序是契约；调回比对之后会让每次上电白报一次不一致"**，并补一句落盘幂等化的原因（否则每次上电白擦两个扇区）。

- [ ] **Step 3: 厂商问题与历史文档**

- `cerb-vendor-questions.md` #5 **结案**：写命令 `23 41 25 00 0X 00 00 00`（owner 手工核实）、读 `40 41 25 00 00 …`、易失（与 `0x2650` 同形）。
- `w25-blob-v3-design.md` §8.2：把两条"仅列出、未实现"的加固方向标注为**已由上电归一落地**（`0x2650` 由归一强制为开 + 方向基线缺失判未初始化）。
- handoff 报告 §一.3 / §二：加一段"**2026-09-20 更新**：保护/方向已进参数面并上电归一，'只接受写 0'与'不进参数面'两条已被取代，见 design 文档"。

- [ ] **Step 4: 提交**

```bash
git add micro_ros_firmware/README.md micro_ros_firmware/AGENTS.md \
  micro_ros_firmware/docs docs/reports/2026-09-19-firmware-param-surface-handoff.md
git commit -m "docs(fw): 上电归一与方向参数——决策日志/README/AGENTS/交接文档收口"
```

---

## 自检记录（写完计划后跑的一遍）

**Spec 覆盖**：§3 上电归一 → Task 8；§4 方向参数与翻转冻结 → Task 3/9（参数面注册在 Task 11）；§5 保护关闭 + 二次确认 → Task 10（页面在 Task 14）；§6 状态话题 → Task 2/5；§7 参数面容量与 web → Task 11/14；§8 失效路径 → 分散在 Task 6/8/9/10 的用例里逐条断言；§9 测试 → 各任务的 Step；§10 文档 → Task 15（消息单源：链接已存在，Task 1 作废）；§11 未验项 → Task 13 Step 1 是硬门；§12 实现顺序 → 任务顺序即此。

**类型/命名一致性**：`boot_norm_stage`/`boot_norm_request_id`/`boot_norm_attempts`/`boot_norm_retry_tick_ms`（Task 8 定义，Task 8 使用）；`zero_stale`（Task 9）；`feed_boot_normalize`/`feed_direction_write`/`feed_set_zero`（Task 6/8/9 定义与使用一致）；`device_protect_known`/`device_protect_enabled`/`device_direction_known`/`device_direction`（Task 2 消息 = Task 5 内核 = Task 14 页面）；`MOTOR_CONTROL_FAULT_ZERO_STALE = 8U` / `MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED = 9U`。

**已知需要在执行中确认的点**（不是占位符，是必须实测的判据）：
1. Task 9 Step 3 的 `SET_ZERO` 结果码（`OK` 还是 `UNCONFIRMED`）必须实测后决定清位条件。
2. Task 1 Step 4 的符号链接 + `--symlink-install` 组合（读源码得出可行，但未实跑）。
3. Task 8 Step 5 的失败注入用例适配清单以 `ctest` 实际输出为准（已列出三类，不保证穷尽）。
