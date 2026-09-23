# control_mailbox

> 库审查：**符合职责边界**。本文件同时是接口说明和审查记录。

| 项目 | 结论 |
|---|---|
| `library.json` | 名称、版本和 `include/src` 映射正确；无外部库依赖 |
| 依赖边界 | 仅使用 C 标准库和本库头文件，不依赖 HAL、RTOS、Arduino、ROS 或协议库 |
| 所有权 | 写侧由 micro-ROS/任务包装层提供临界区，读侧由 `motor_rt_task` 在控制边界消费 |
| host 覆盖 | `test_control_mailbox.cpp`、`test_trajectory_mailbox.cpp`、`test_policy_mailbox.cpp` |
| 待补强 | 目标板上的任务调度与临界区时序验证 |

固定容量的跨任务控制输入交换层：micro-ROS 执行上下文（写侧）与 `motor_rt_task`
（读侧）之间的命令、轨迹、策略、低频配置和会话事件的唯一通道（ARD-0007 §27/§28）。
纯 C、静态内存、无动态分配；不依赖 HAL、RTOS、Arduino、ROS 或任何电机协议。

## 模块结构

| 文件 | 职责 |
|---|---|
| `include/control_command_mailbox.h` + `src/control_command_mailbox.c` | 底层：完整命令快照的双缓冲 latest-wins 交换 |
| `include/control_trajectory_mailbox.h` + `src/control_trajectory_mailbox.c` | 分段轨迹的有界拼接、校验与双缓冲发布 |
| `include/control_policy_mailbox.h` + `src/control_policy_mailbox.c` | 最多 64 台的策略快照双缓冲 |
| `include/control_mailbox.h` + `src/control_mailbox.c` | 会话门、16 槽配置 FIFO 与连接/授权边沿 |

两层并发契约相同（见下）。`control_command_mailbox_*` 虽是公开 API，生产代码
只应经 `control_mailbox_*` 入口访问——直接 publish 内嵌通道会绕过
授权/会话门，使拒绝诊断失真。

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
  （SET_MODE / SET_GAINS / SET_POSITION_LIMITS / SET_ENABLE 等）+ 联合负载。
- 容量 `CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY`(16)；满时拒绝新请求，**绝不丢弃
  或覆盖已入队项**；消费侧每 2ms/500Hz 控制边界至多转发 4 条（`motor_rt_task` 的
  `kConfigDrainPerBoundary`）。
- 配置不属于运动授权：禁能/未授权下仍可排队，但会话门仍生效（Agent 断开即拒）。

### 轨迹与策略

- 轨迹通道按 `trajectory_id` 逐段拼接，只有完整且校验通过才替换待消费轨迹；
  空单段轨迹表示取消。拼接容量和 latest-wins 缓冲均固定。
- 策略通道最多容纳 64 台的 enable/kp/kd 快照，使用独立双缓冲；不占配置 FIFO。
- 两者与命令通道共用会话/授权门。断链会撤销未消费输入和轨迹拼接态。

### 会话/授权（状态电平 + 一次性边沿）

- `request_agent_connected`：同电平重复调用幂等；`request_authorization` 仅供测试直接置电平。
- init 默认 `authorization_requested=true`（引导豁免）：首次收到 Agent 状态前允许
  启动期写入；注意豁免实际会延续到首个会话全程（首个会话无需显式 enable），
  设计意图见 `control_mailbox.c` 内注释。
- Agent「曾连接后断开」：清命令、轨迹、策略、配置并撤权，产生 disconnect 边沿；
  **重连自动重立授权电平**。逐台 FAULT/OFFLINE/CONFLICT 仍由内核门禁裁决。
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
  读侧 `control_boundary`（motor_rt 任务，prio 3，2ms/500Hz 边界逐调用临界区）。

## 拒绝诊断（生命周期累计，clear 不清零）

| 计数 | 含义 |
|---|---|
| `command_rejected_count` | 命令被授权/会话门拒绝 |
| `command.invalid_snapshot_count` | 快照非法（超 64 / 非有限浮点 / motor_id 重复） |
| `mailbox_rejected_command`（诊断话题） | 前两项拒绝计数之和；轨迹和策略也分别导出门拒绝 + 载荷拒绝之和 |
| `command.overwrite_count` | 命令快照未取走即被覆盖 |
| `config_invalid_count` | 配置请求非法（request_id==0 / 未知 kind / 负载非法） |
| `config_session_count` | Agent 断开期的配置提交 |
| `config_overflow_count` | 配置 FIFO 满 |

各提交路径均为**门先于数据校验**：「断链 + 非法载荷」归因到门，各计数
互斥不重复。`motor_rt_copy_diagnostics()` 将命令、轨迹、策略拒绝汇总给诊断话题。

## 内存与容量

全部编译期定容、静态分配（ARD §29）：命令、轨迹、策略各自有独立双缓冲，
轨迹还有一份固定容量的分段拼接缓冲，配置 FIFO 有 16 槽。实际内存占用以目标
链接图为准。各双缓冲容量固定为 2，翻转逻辑由 `_Static_assert` 守护。

## 构建与测试

固件侧：PlatformIO 自动发现 `lib/`（`library.json` 无依赖声明），经
`include/motor_rt_task.h` 的 include 链接入构建。

host 单测：`test/host/test_control_mailbox.cpp`、`test_trajectory_mailbox.cpp`、
`test_policy_mailbox.cpp`（CTest 目标分别以 `test_` 开头，直接编译本库四个 `.c`）：

```bash
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure -R control_mailbox
```

已覆盖：latest-wins、轨迹分段拼接、策略快照、FIFO 顺序与回绕、会话边沿、
拒绝计数、clear 保留诊断、NULL 提交和在共享锁下的并发发布/消费完整性。
host 用例不能替代目标板上的 FreeRTOS 临界区时序验证。

## 职责边界（ARD §28）

MUST：命令、轨迹、策略、低频配置和连接/授权事件的固定容量跨任务交换。
MUST NOT：策略裁决、协议校验、限位/能力判断、设备等待、CAN 发送——这些属于
`motor_rt_task` / `motor_control` / `motor_protocol`。库不保存 joint 名、协议
类型或电机型号。
