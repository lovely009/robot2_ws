# control_mailbox

固定容量的跨任务控制输入交换层：micro-ROS 执行上下文（写侧）与 `motor_rt_task`
（读侧）之间的高频命令、低频配置和会话/授权事件的唯一通道（ARD-0007 §27/§28）。
纯 C、静态内存、无动态分配；不依赖 HAL、RTOS、Arduino、ROS 或任何电机协议。

## 模块结构

| 文件 | 职责 |
|---|---|
| `include/control_command_mailbox.h` + `src/control_command_mailbox.c` | 底层：完整命令快照的双缓冲 latest-wins 交换 |
| `include/control_mailbox.h` + `src/control_mailbox.c` | 上层：在命令外再包授权/会话门、16 槽配置 FIFO、连接/授权边沿 |

两层并发契约相同（见下）。`control_command_mailbox_*` 虽是公开 API，生产代码
只应经 `control_mailbox_*` 入口访问——直接 publish 内嵌 `command` 成员会绕过
授权/会话门，且不计入 `command_rejected_count`，使拒绝诊断失真。

## 数据通道

### 高频命令（latest-wins，不排队）

- `control_command_snapshot_t`：`sequence` + `received_tick_ms`（MCU 本地接收
  tick，ARD §12.2）+ 最多 `CONTROL_COMMAND_MAILBOX_MAX_TARGETS`(64) 个
  `{motor_id, position, velocity, effort}`；motor_id 必须唯一，全字段必须 isfinite。
- `submit_command`：授权门 + 会话门 → 通过则 publish。未取走重发只覆盖后台缓冲
  并计 `overwrite_count`，已发布快照不会被撕碎，`take` 拿到的永远是完整副本。
- `take_command`：取走当前已发布快照并清 pending；无新快照返回 false，不改写输出。
- 新鲜度（30ms/100ms staleness）、缺席电机软停等裁决**不在本库**
  ——mailbox 刻意不校验 sequence/tick，由 motor_control 内核负责（sequence 序号门
  已于 2026-09-15 移除，sequence 仅作诊断）。

### 低频配置（有界 FIFO，不覆盖）

- `control_mailbox_config_request_t`：`request_id`(非 0) + `motor_id` + `kind`
  （SET_MODE / SET_GAINS / SET_POSITION_LIMITS / SET_ENABLE，ARD §19）+ 联合负载。
- 容量 `CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY`(16)；满时拒绝新请求，**绝不丢弃
  或覆盖已入队项**；消费侧每 10ms 控制边界至多转发 4 条（`motor_rt_task` 的
  `kConfigDrainPerBoundary`）。
- 配置不属于运动授权：禁能/未授权下仍可排队，但会话门仍生效（Agent 断开即拒）。

### 会话/授权（状态电平 + 一次性边沿）

- `request_authorization` / `request_agent_connected`：同电平重复调用幂等（无新边沿）。
- init 默认 `authorization_requested=true`（引导豁免）：首次收到 Agent 状态前允许
  启动期写入；注意豁免实际会延续到首个会话全程（首个会话无需显式 enable），
  设计意图见 `control_mailbox.c` 内注释。
- Agent「曾连接后断开」：清命令 + 清配置 + 撤权 + disconnect 边沿；**重连不自动
  恢复授权**，必须显式 `request_authorization(true)`（ARD §33）。
- `take_events`：破坏性读出 4 个一次性边沿，同时带货当前两个电平；返回 false 时
  仍带货——任务层首边界应先调用一次同步初始电平（`motor_rt_task` 的
  `session_synced` 机制）。
- 边沿是历史事实（「发生过撤销」），可能与当前电平并存：同一边界内先撤权再授权，
  `take_events` 会同时交付 `authorization_disable_edge=true` 与电平 `true`。
  消费方应以电平为裁决依据，边沿用于感知变化；当前 `authorization_disable_edge`
  与 `agent_disconnect_edge` 尚无生产读者。

## 并发契约

- 库内**无锁、无原子字段、无 volatile**：每次 API 调用都必须位于调用方提供的
  临界区内。
- 写侧与读侧必须**共用同一互斥域**——单核下 FreeRTOS `taskENTER_CRITICAL` 或
  等效关中断即满足；「两侧各自加锁」不构成互斥，会产生数据竞争并可能静默丢命令。
- 仅支持任务上下文：ISR 内 `taskENTER_CRITICAL` 非法，本层不做检测。
- 现有调用点：写侧 `src/motor_rt_task.cpp` 的 `motor_rt_submit_*` /
  `motor_rt_set_agent_connected` 包装（micro_ros 任务，prio 2，逐调用临界区）；
  读侧 `control_boundary`（motor_rt 任务，prio 3，10ms 边界逐调用临界区）。

## 拒绝诊断（生命周期累计，clear 不清零）

| 计数 | 含义 |
|---|---|
| `command_rejected_count` | 命令被授权/会话门拒绝 |
| `mailbox_rejected_command`（诊断话题） | 快照非法（超 64 / 非有限浮点 / motor_id 重复）——**注意**：README 此前写的是 `command.invalid_snapshot_count`，那个计数器在代码里从未存在过 |
| `command.overwrite_count` | 命令快照未取走即被覆盖 |
| `config_invalid_count` | 配置请求非法（request_id==0 / 未知 kind / 负载非法） |
| `config_session_count` | Agent 断开期的配置提交 |
| `config_overflow_count` | 配置 FIFO 满 |

两条提交路径均为**门先于数据校验**：「断链 + 非法载荷」统一归因到门，各计数
互斥不重复。已知限制：这些计数目前没有生产读者或上报通路，「拒绝必可见」
只兑现到结构体字段为止，现场排障需接诊断发布后才有观测点。

## 内存与容量

全部编译期定容、BSS/静态分配（ARD §29）：双快照缓冲（2 × ~1 KB）+ 16 槽配置
FIFO 合计约 2.5 KB。`CONTROL_COMMAND_MAILBOX_BUFFER_COUNT` 固定为 2——publish
的缓冲翻转按双缓冲硬编码，`.c` 内有 `_Static_assert` 守护，勿单独调整该宏。

## 构建与测试

固件侧：PlatformIO 自动发现 `lib/`（`library.json` 无依赖声明），经
`src/motor_rt_task.h` 的 include 链接入构建。

host 单测：`test/host/test_control_mailbox.cpp`（CTest 目标 `control_mailbox`，
C17 + cassert，直接编译本库两个 `.c`）：

```bash
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure -R control_mailbox
```

已覆盖：latest-wins 与覆盖计数、64 目标满容量与非法快照拒绝、FIFO 顺序与回绕、
三类拒绝互斥、授权×会话边沿矩阵、门拒绝计数、SET_ENABLE 往返、门先于校验的归因。
已知缺口（ARD §43.2 要求但未做）：并发写读压力用例、clear 系列直测、NULL 分支、
诊断计数「clear 不清零」断言。

## 职责边界（ARD §28）

MUST：高频命令、低频配置、连接/授权事件的固定容量跨任务交换。
MUST NOT：策略裁决、协议校验、限位/能力判断、设备等待、CAN 发送——这些属于
`motor_rt_task` / `motor_control` / `motor_protocol`。库不保存 joint 名、协议
类型或电机型号。
