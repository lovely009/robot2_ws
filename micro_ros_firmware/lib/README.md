# micro_ros_firmware 库分层

`lib/` 只放硬件驱动、协议逻辑、实时数据结构和有界数据交换。任务调度、
ROS/micro-ROS 消息转换和应用编排留在 `src/`。顶层库按所有权和变化原因划分，
库内部再用多个源文件保持高内聚，避免把每个概念都拆成独立 PlatformIO 库。

## 目标目录

| 库 | 只负责 | 不负责 |
|---|---|---|
| `fdcan` | FDCAN1/2/3 的 HAL、IRQ、RX ring、TX 调度 | 电机协议、拓扑、控制策略 |
| `dma` | DMA 控制器、通道配置和 IRQ 资源管理 | IMU/LED 采样策略和设备业务 |
| `motor_protocol` | Vaxor/CERB 发现、编解码、MIT、停止、SYNC、配置帧 | 注册表、ROS、HAL |
| `motor_control` | 动态注册表、型号 Profile、生命周期、安全、freshness、状态快照 | ROS、HAL、厂家字节布局 |
| `control_mailbox` | 完整命令、分段轨迹、批量策略、低频配置和会话事件的跨任务交换 | 协议校验、限位裁决、CAN 发送 |
| `session_sync` | 会话/授权电平与边沿到内核调用的纯逻辑派生 | 任务调度、ROS、CAN |
| `storage_projection` | 参数视图与 Flash blob 记录的纯投影 | Flash 读写、保存时序、任务通知 |
| `param_store` | 参数字段、单位、blob 布局、CRC 和版本迁移 | W25Q64 设备访问、控制策略 |
| `imu` | BMI088、SPI2 阻塞通信、融合和 snapshot（不用 DMA） | 电机控制、ROS 发布 |
| `w25q64` | OCTOSPI2 W25Q64 原始读写与 geometry | 参数策略、日志调度、控制闭环 |
| `ws2812` | SPI6/PA7 的 LED 物理发送 | 系统状态判定、电机安全 |
| `board` | 板级共享常量和中断优先级约束 | 设备业务、任务调度 |

### 模块内部边界

`fdcan` 内部至少区分 `can_frame`、`can_rx_ring`、`can_tx_scheduler`；`fdcan.h` 的 HAL 句柄不得泄漏到协议层。

`motor_control` 内部区分 `motor_registry`、`motor_profile`、`motor_lifecycle`、
`motor_safety` 和 `motor_snapshot`。Profile 是固件内置的型号目录，不是安装台账；
当前只收录已确认参数的 CERB R48/R58 和 Vaxor 25/30mm 型号。

`control_mailbox` 内部使用相互独立的 command/trajectory/policy/config/event 存储
和 API。命令、完整轨迹和批量策略分别使用双缓冲 latest-wins；轨迹分段使用独立
固定容量拼接缓冲；低频配置使用固定容量 FIFO；连接和授权使用状态电平加一次性
边沿。库不保存 joint 名、协议类型或电机型号，也不做厂家协议能力裁决。

`dma` 是共享的板级资源门卫，只管控制器级资源（DMA1 + BDMA/DMAMUX2 时钟、
通道 NVIC、占用登记、优先级合规断言），由 `main` 板级初始化统一使能。设备
配置归各设备库：`lib/imu` 自持阻塞 SPI2 且当前不用 DMA；`lib/ws2812` 自持
BDMA_Ch0 句柄与 handler，帧缓冲位于 `.bdma_noinit`/RAM_D3。设备库之间禁止
互相 include 或调用。

## 依赖方向

```text
src/ros_bridge     -> motor_bridge + imu_bridge（ROS 实体编排）
src/motor_bridge   -> motor_rt_task（经包装函数写 control_mailbox）
src/motor_rt_task  -> control_mailbox + fdcan + motor_control + session_sync
motor_control      -> motor_protocol（接口接缝，文档明载）
storage_projection -> motor_control 类型 + param_store（纯投影）
src/sensor_task    -> imu
src/led_task       -> ws2812
src/storage_task   -> w25q64 + param_store + storage_projection（Flash 唯一所有者，见 storage_task.h）
lib/w25q64         -> octospi 控制器层 + W25 命令层/geometry（仅 storage_task 使用）
lib/param_store    -> blob 布局/CRC/v1-v2 迁移 + 参数字段/单位纯逻辑（无硬件依赖）
```

库不能反向依赖 `src/`、ROS 或其他任务。实时路径不使用动态内存、阻塞等待或
无限队列；正常电机 TX 只有 `motor_rt_task` 一个拥有者。

## 合并与保留说明

以下职责已经合并回上面的顶层库，不再建立独立目录：

- `can_transport` → `fdcan` 内部源文件；
- `motor_profile` → `motor_control` 内部源文件；
- `command_mailbox`、`config_mailbox` → `control_mailbox` 内部 API；
- `dma` 保持独立；当前管理 WS2812 的 BDMA 控制器资源，并为后续设备统一登记资源。

### `control_mailbox` 当前接口

- `control_mailbox_submit_command()` / `take_command()`：最多 64 个唯一
  `uint16 motor_id`，位置/速度/力矩为 `float32`，携带 sequence 和 MCU 接收 tick；
- `control_mailbox_submit_trajectory()` / `take_trajectory()`：有界分段拼接，完成后
  发布一条轨迹；空轨迹用于取消；
- `control_mailbox_submit_policy()` / `take_policy()`：最多 64 台的独立批量策略快照；
- `control_mailbox_submit_config()` / `take_config()`：固定容量 16 的低频语义请求队列，
  不覆盖已入队请求；
- `control_mailbox_request_authorization()`、`request_agent_connected()` 和
  `take_events()`：只传递连接/授权状态及边沿；
- 所有 API 都是纯 C、静态内存；并发临界区由任务调用方提供，mailbox 不创建锁、任务
  或动态内存。

## 库级文档与审查

每个库的接口、所有权、测试覆盖与审查结论见下方**各库细则**（一份文件，一层一份）。
跨库分层与依赖方向以本文件开头的总表为准。

## 各库细则

> 以下为各库的契约细则；「只负责 / 不负责」总表见上。


### `board`

#### 审查结论

**符合。** 这是一个有意的纯头文件板级约束库，不需要 `src/`。PlatformIO 已在固件依赖图中发现并参与构建；`library.json` 显式声明了 `include/` 目录。它不应放设备驱动、任务或 FreeRTOS 头。

#### 职责与边界

- `include/board_irq.h` 提供 `BOARD_ISR_SYSCALL_THRESHOLD`，作为板级 ISR 优先级的唯一共享常量。
- `lib/dma`、`lib/fdcan` 等板级库引用它做编译期约束。
- 不拥有中断处理器、外设句柄、任务或业务状态。
- 不反向依赖 `src/`、HAL、FreeRTOS 或 ROS。

#### 文件与接口

| 文件 | 作用 |
|---|---|
| `library.json` | PlatformIO 库元数据 |
| `include/board_irq.h` | ISR syscall 阈值常量和板级约束注释 |

公共接口只有一个宏：`BOARD_ISR_SYSCALL_THRESHOLD = 5U`。它必须与 FreeRTOS 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 保持一致；`main.cpp` 负责跨侧反向断言。

#### 依赖与验证

该库没有 host 单测目标，因为没有可执行逻辑。验证方式是：

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run
```

当前固件构建成功，说明头文件被正确解析。新增板级常量时，应同时检查 `main.cpp` 的反向断言和所有使用者。

#### 后续建议

保持 header-only 形态；若将来出现函数或状态，不要把实现塞进本库，应先重新划分所有权并补 `src/` 与测试。


### `control_mailbox`

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

#### 模块结构

| 文件 | 职责 |
|---|---|
| `include/control_command_mailbox.h` + `src/control_command_mailbox.c` | 底层：完整命令快照的双缓冲 latest-wins 交换 |
| `include/control_trajectory_mailbox.h` + `src/control_trajectory_mailbox.c` | 分段轨迹的有界拼接、校验与双缓冲发布 |
| `include/control_policy_mailbox.h` + `src/control_policy_mailbox.c` | 最多 64 台的策略快照双缓冲 |
| `include/control_mailbox.h` + `src/control_mailbox.c` | 会话门、16 槽配置 FIFO 与连接/授权边沿 |

两层并发契约相同（见下）。`control_command_mailbox_*` 虽是公开 API，生产代码
只应经 `control_mailbox_*` 入口访问——直接 publish 内嵌通道会绕过
授权/会话门，使拒绝诊断失真。

#### 数据通道

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

#### 并发契约

- 库内**无锁、无原子字段、无 volatile**：每次 API 调用都必须位于调用方提供的
  临界区内。
- 写侧与读侧必须**共用同一互斥域**——单核下 FreeRTOS `taskENTER_CRITICAL` 或
  等效关中断即满足；「两侧各自加锁」不构成互斥，会产生数据竞争并可能静默丢命令。
- 仅支持任务上下文：ISR 内 `taskENTER_CRITICAL` 非法，本层不做检测。
- 现有调用点：写侧 `src/motor_rt_task.cpp` 的 `motor_rt_submit_*` /
  `motor_rt_set_agent_connected` 包装（micro_ros 任务，prio 2，逐调用临界区）；
  读侧 `control_boundary`（motor_rt 任务，prio 3，2ms/500Hz 边界逐调用临界区）。

#### 拒绝诊断（生命周期累计，clear 不清零）

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

#### 内存与容量

全部编译期定容、静态分配（ARD §29）：命令、轨迹、策略各自有独立双缓冲，
轨迹还有一份固定容量的分段拼接缓冲，配置 FIFO 有 16 槽。实际内存占用以目标
链接图为准。各双缓冲容量固定为 2，翻转逻辑由 `_Static_assert` 守护。

#### 构建与测试

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

#### 职责边界（ARD §28）

MUST：命令、轨迹、策略、低频配置和连接/授权事件的固定容量跨任务交换。
MUST NOT：策略裁决、协议校验、限位/能力判断、设备等待、CAN 发送——这些属于
`motor_rt_task` / `motor_control` / `motor_protocol`。库不保存 joint 名、协议
类型或电机型号。


### `dma`

#### 审查结论

**符合职责边界；硬件路径没有 host 覆盖。** 该库只管理 DMA 控制器级资源，设备传输配置仍由设备库持有，和 `lib/README.md` 的分层定义一致。

#### 职责与边界

- `MX_DMA_Init()` 开启 DMA1、BDMA/DMAMUX2 时钟并配置 BDMA Channel0 的 NVIC 优先级。
- `BOARD_ISR_SYSCALL_THRESHOLD` 是 ISR 优先级门槛的唯一来源。
- WS2812 自持 SPI6 TX 的 DMA 句柄、handler 和帧缓冲；BMI088 当前使用阻塞 SPI2，不占 DMA。
- 本库不配置 IMU/LED 的业务传输，不依赖任务、ROS 或电机协议。

#### 文件与接口

| 文件 | 作用 |
|---|---|
| `include/dma.h` | HAL 依赖和 `MX_DMA_Init()` 声明 |
| `src/dma.c` | DMA1/BDMA 时钟、IRQ 配置和资源登记注释 |
| `library.json` | PlatformIO 元数据 |

#### 依赖与所有权

`dma.h` 依赖 STM32 HAL，`dma.c` 额外依赖 `board_irq.h`。`main.cpp` 在设备初始化前调用 `MX_DMA_Init()`；设备库不得互相占用或调用彼此的句柄。

#### 验证与缺口

- PlatformIO 固件构建通过。
- 没有 host 单测，原因是 HAL/NVIC 行为只能在目标板验证。
- 资源表目前是注释和配置的组合契约；新增 DMA 使用者必须同步更新占用表、IRQ 优先级和链接脚本中的缓冲区约束。

#### 后续建议

`library.json` 已显式声明 `include/`、`src/` 目录与 `board` 依赖。新增 DMA 使用者仍需在资源表和台架验证中确认通道不冲突。


### `fdcan`

#### 审查结论

**结构和职责符合；测试覆盖集中在纯逻辑层，硬件入口仍需台架验证。** 库没有向协议层泄漏策略，也没有反向依赖任务层。

#### 职责与边界

- 三路 FDCAN1/2/3 的 HAL 初始化、GPIO/时钟、过滤器和 IRQ 回调。
- 每总线一个 SPSC RX ring：ISR 是唯一生产者，`motor_rt_task` 是唯一消费者。
- 每总线一个 P0/P1/P2 TX 调度器：P0 安全帧、P1 latest-wins 控制帧、P2 有界管理队列。
- 不做协议解码、型号发现、ROS 发布或安全策略裁决。

#### 文件与接口

| 文件 | 作用 |
|---|---|
| `include/fdcan.h` / `src/fdcan.c` | 三路 HAL 句柄、初始化、MSP 和 IRQ |
| `include/can_bsp.h` / `src/can_bsp.c` | 总线初始化、RX/TX 访问和硬件统计 |
| `include/can_rx_ring.h` / `src/can_rx_ring.c` | ISR→任务的固定容量 RX 环 |
| `include/can_tx_scheduler.h` / `src/can_tx_scheduler.c` | P0/P1/P2 软件调度和拒收记账 |
| `library.json` | PlatformIO 元数据 |

关键接口包括 `bsp_can_init()`、`can_bsp_rx_ring()`、`can_bsp_submit_tx()`、`can_rx_ring_push_from_isr()`、`can_rx_ring_pop()` 和 `can_tx_scheduler_pump()`。

#### 实时与所有权契约

- RX ring 满时丢新帧并计数，ISR 不回收消费者可能仍在读取的槽位。
- TX 调度器有界，不阻塞、不分配内存；P0 不应被 P2 积压挤占。
- FDCAN ISR 只取硬件帧、盖 MCU tick、写 ring；正常 CAN TX 只有 `motor_rt_task` 提交。
- 板级 ISR 不包含 FreeRTOS，优先级约束来自 `lib/board`。

#### 验证与缺口

- `test_can_rx_ring.cpp`：环容量、溢出和批量弹出。
- `test_can_tx_scheduler.cpp`：P0/P1/P2 优先级、latest-wins、配额和硬件拒收记账。
- `test_fdcan_motor_sim.py`：电机运行路径仿真。
- PlatformIO 固件构建通过。

host 未编译 `can_bsp.c`、`fdcan.c` 的 HAL/IRQ 路径；这属于硬件测试缺口，不是目录越界。当前还保留 `TBD-CAN-01` 的 P2 配额和 `TBD-CAN-02` 的过滤器收敛项，不能在文档中标成已冻结规格。

#### 后续建议

台架验证应记录 RX FIFO 丢失、TX FIFO 拒收、实际仲裁/数据段时序；若将来需要脱机验证硬件包装层，可注入 HAL seam，不要把 HAL stub 混进纯 ring/scheduler 测试。


### `imu`

#### 审查结论

**职责符合；纯逻辑可 host 测试，SPI/HAL 路径只能目标板验证。** 该库集中拥有 BMI088、SPI2 阻塞传输、Mahony 融合和 IMU 双缓冲，不承担 ROS 发布。

#### 职责与边界

- `bmi088_driver.*`：厂商寄存器驱动和原始量读取。
- `bmi088_middleware.*`、`imu_spi2_transport.*`：SPI2、片选、延时和 HAL 传输错误适配。
- `imu.c`、`MahonyAHRS.c`：采样更新、SI 单位转换和姿态融合。
- `imu_snapshot.*`：固定双缓冲快照。
- 不使用 DMA；不依赖电机、ROS 或消息桥接。

#### 文件与接口

公共入口包括 `imu_try_init()`、`imu_update()`、`imu_get_data()`、`imu_ready()`、`imu_snapshot_*()` 和 `BMI088_transport_error_*()`。厂商 API（如 `BMI088_init()`、`BMI088_read()`）保留原名，以便追溯数据手册和原始例程。

#### 所有权与实时约束

- `sensor_task` 是初始化和采样的唯一调用者；`imu_bridge` 只读取发布快照。
- SPI2 是阻塞传输，不能放进电机实时任务；传输失败锁存到当前操作并由任务重试。
- `imu_data_t` 对外使用 rad/s、m/s²、°C 和四元数；库内不创建 ROS 消息。

#### 验证与缺口

- `test_imu_snapshot.cpp`：双缓冲发布/复制。
- `test_imu_units.cpp`：SI 单位和 Mahony 聚合。
- `test_bmi088_transport_status.cpp`、`test_bmi088_register_config.cpp`：错误锁存和寄存器配置。
- PlatformIO 固件构建通过。

host 没有编译 `bmi088_middleware.c`、`imu_spi2_transport.c` 和完整 BMI088 读写；GPIO、FreeRTOS 延时、SPI 时序和 EXTI 唤醒仍需台架验证。`bmi088_driver` 的厂商拼写是兼容性约束，不应在文档审查中误判为新 API 设计。

#### 后续建议

保持传感器任务单一所有者；若修改 SPI 速率、ODR 或融合采样频率，应同时更新 `platformio.ini`/系统时钟说明和 host 单位测试。


### `motor_control`

#### 审查结论

**符合核心库定义；型号目录和实时安全语义已落在本库，仍有明确 TBD 项。** 它是纯 C、ID 驱动的运行时内核，不依赖 HAL、FreeRTOS、ROS 或 `control_mailbox`。

#### 职责与边界

- 固定容量的动态电机注册表和事件环。
- Profile 目录、协议绑定、生命周期（ABSENT 到 ACTIVE/FAULT/OFFLINE 等）和配置事务。
- 完整快照、轨迹、策略、freshness、软停/失能和 P0 重试。
- 只通过 `motor_protocol_adapter_t` 使用厂家协议；不保存 `joint_name`，不拼厂家字节。

#### 文件与接口

| 文件 | 作用 |
|---|---|
| `include/motor_control.h` | 不透明上下文、生命周期、实时路径、快照/状态/事件 API |
| `include/motor_control_profile.h` | 编译期型号目录查询 |
| `src/motor_control.c` | 主状态机、快照、轨迹、策略和输出 |
| `src/motor_control_discovery.c` | 反馈发现、建档和重连 |
| `src/motor_control_config.c` | 配置事务和设备参数回读 |
| `src/motor_control_params.c` | 参数影子、限值和持久化视图 |
| `src/motor_control_profile.c` | Profile 表实现 |
| `src/motor_control_internal.h` | 仅库内实现使用的布局和辅助接口 |

关键入口是 `motor_control_init()`、`motor_control_on_raw_frame()`、`motor_control_service()`、`motor_control_apply_command_snapshot()`、`motor_control_apply_trajectory()`、`motor_control_apply_policy()`、`motor_control_collect_tx()` 和各类只读快照复制函数。

#### 实时与依赖

- `motor_rt_task` 是唯一调用者；1ms 服务和 2ms/500Hz 控制边界都由调用方驱动。
- 所有容量在编译期固定；不等待设备、不分配动态内存、不读取 ROS 时间。
- 唯一库依赖是 `motor_protocol` 的 adapter 接口；`library.json` 已声明该依赖。

#### 验证与缺口

- `test_motor_control.cpp` 覆盖注册、生命周期、授权、freshness、快照、配置和事件。
- `test_session_sync.cpp` 通过真实内核验证会话同步。
- `test_motor_protocol_adapter.cpp` 覆盖 Profile/adapter 交互。
- PlatformIO 固件构建通过。

当前 `Vaxor 80` 不进入目录是有意安全策略；`TBD-PROFILE-01`、`TBD-CONFIG-01` 和部分安全语义仍未冻结。它们属于产品/协议待决项，不是库边界违规。修改容量宏时必须重新做链接 RAM 和实时性验证。


### `motor_protocol`

#### 审查结论

**符合协议库定义，host 覆盖完整度最高。** 公共类型、CERB/VAXOR 编解码和 adapter 接缝都集中在本库，未发现 HAL、ROS、任务层或注册表反向依赖。

#### 三块结构

| 区域 | 内容 |
|---|---|
| 公共 | SI 侧命令/状态、通用量化、adapter 公共类型和传输帧描述 |
| CERB | CAN-ID、MIT/PDO、CANopen SDO、SYNC、设备参数换算和 `motor_protocol_cerb_adapter` |
| VAXOR | 25/30 mm MIT、控制/反馈/状态帧和 `motor_protocol_vaxor_adapter` |

#### 文件与接口

| 文件 | 作用 |
|---|---|
| `include/motor_protocol.h` | 公共 codec、ID 推导、CERB/VAXOR 线协议结构 |
| `include/motor_protocol_adapter.h` | 内核注入的协议 adapter 接口、Profile 和配置阶段契约 |
| `src/motor_protocol.c` | 公共/CERB/VAXOR 基础 codec 与 ID 函数 |
| `src/cerb_adapter.c` | CERB 帧归属、解码、MIT、停机、SYNC、SDO 阶段 |
| `src/vaxor_adapter.c` | VAXOR 帧归属、解码、MIT、停机和配置阶段 |

内核只调用 `motor_protocol_adapter_t`；新增协议应增加 adapter 和测试，不改 `motor_control` 的收发循环。

#### 验证与已知待决项

- `test_motor_protocol.cpp`：基础 codec、ID、边界和单位。
- `test_motor_protocol_adapter.cpp`：两种 adapter 的解码、MIT、停机、SYNC 和配置事务。
- `test_motor_control.cpp`、`test_storage_projection.cpp`：跨库的槽序和 Profile 契约。
- PlatformIO 固件构建通过。

`TBD-CAN-03`（各帧 FD/BRS/DLC 的最终确认）、`TBD-CONFIG-01`（厂家确认对象）和 `TBD-SAFETY-01/03`（软停/离线阈值）仍在代码注释中明确标记；当前实现按占位语义运行，不能在接口文档中当作厂家最终规格。


### `motor_tx_router`

`motor_tx_router` owns the join between the protocol-agnostic per-bus queue and
physical CAN submission policy. It selects one bounded P0/P1/P2 pump, accounts
for protocol adapters that reduce several P1 commands to one physical frame,
and reserves the P2 pump quota when a pump starts with an empty P0 queue. If P0
is queued at pump start, the router dequeues only P0. A hardware-rejected P0
stops that pump; it and all unattempted P0 frames return to the FIFO front in
their original order.

The module is pure C. It depends on `fdcan` queue types and the
`motor_protocol` aggregation interface, but has no HAL, FreeRTOS, ROS, or heap
dependency. A submit callback is the only hardware seam. `motor_rt_task` owns one
router instance and calls it serially for each bus; the fixed scratch buffer is
reused between buses.

P1 remains latest-wins. A rejected aggregated P1 frame is dropped until the
next latest-wins update, while each P0 frame is retried. A lone CERB MIT command
passes through unchanged; compatible adjacent CERB commands are encoded by the
CERB adapter as one multi-axis frame. The router itself does not contain CAN
IDs or manufacturer payload layouts.


### `param_store`

#### 审查结论

**符合纯参数存储库定义；v3 有意保留未实现字段。** 本库不访问 W25Q64、不决定保存时序，也不参与运动控制策略。

#### 职责与边界

- `param_field.*`：参数字段枚举、设备原值槽序和值域校验。
- `param_units.h`：N·m/A 的边界换算纯函数。
- `param_store_blob.*`：v3 固定布局、present 位图、CRC32、记录读写和 v1/v2 迁移。
- Flash 擦写、A/B 槽选择和任务通知由 `src/storage_task.cpp` 与 `w25q64` 负责。

#### 文件与接口

公共接口包括 `param_value_ok()`、`param_position_limits_ok()`、`param_store_blob_reset()`、`param_store_blob_validate()`、`param_store_blob_set/get()`、设备原值读写和 `param_store_blob_migrate_v1/v2()`。

v3 当前布局是 32B 头 + 64×120B 记录；记录内的 `model_name`、`identity_hash`、MIT 标度和 `profile_version` 是预留字段，写入路径尚未实现，值为零不能解释为已核验身份。

#### 依赖与安全约束

- 仅依赖 C 标准库；C++ 通过 `extern "C"` 暴露函数。
- blob 缓冲区由调用方提供，生产路径使用满足对齐要求的静态存储。
- `enable` 不持久化；`limit_protect` 和方向使用 `device_raw` 槽，不能误当作普通 float 字段。
- `param_store_blob_finalize()` 是落盘前重算 CRC 的最后一步。

#### 验证与缺口

`test_param_store.cpp` 覆盖字段值域、v3 布局、CRC、容量、设备原值、版本迁移和单位换算；PlatformIO 固件构建通过。实际 Flash 读写由 `w25q64` 和 storage 任务承担，不在本库 host 测试中重复模拟。

#### 后续建议

新增字段必须同步更新枚举顺序、位图语义、记录大小/迁移和 `storage_projection` 的跨库静态断言；不要把预留字段写成“已保存”。


### `save_transaction`

`save_transaction` coordinates one whole-system save without owning any motor,
ROS, RTOS, or flash implementation. It snapshots dirty device slots, submits a
bounded number of device-save requests per poll, correlates their result IDs,
and requests W25 persistence only after every device save succeeds.

The module exposes narrow injected ports for request-ID allocation, device
submission, storage request/result, and error reporting. The two phases use a
wrap-safe 5-second watchdog. A failed device phase drains already accepted
requests but never starts W25; storage failure cannot roll back device flash.
Duplicate SAVE requests merge into one queued follow-up, and disconnect clears
active and queued work without changing the ROS-owned completion sequence.


### `session_sync`

#### 审查结论

**符合纯逻辑边界，host 覆盖有效。** 该库只把 mailbox 的会话边沿/电平按固定顺序转发给 `motor_control`，不创建任务、不持有锁、不访问 CAN。

#### 职责与接口

唯一公共入口是：

```c
void session_sync_apply(motor_control_context_t *ctx,
                        const control_mailbox_events_t *events,
                        bool session_authorized, uint32_t now_ms);
```

处理顺序固定为：先转发 `agent_state_pending` 连接边沿，再按会话授权电平调用内核。这样断连时能触发内核的 P0 撤权路径；每个控制边界都可重放当前电平。

#### 依赖与调用契约

- 依赖 `control_mailbox` 的事件类型和 `motor_control` 的状态接口。
- 依赖方向是 `motor_rt_task -> session_sync -> (control_mailbox, motor_control)`，没有反向 include。
- `take_events()` 与授权电平读取必须由调用方放在同一临界区；本库不提供锁。
- `ctx` 或 `events` 为 NULL 时安全返回，但调用方仍应保证正常路径数据有效。

#### 验证

`test_session_sync.cpp` 通过真实 mailbox、内核和 adapter 对象验证连接、断连、授权和事件顺序；PlatformIO 固件构建通过。`library.json` 已显式列出 `control_mailbox` 和 `motor_control` 依赖。


### `storage_projection`

#### 审查结论

**职责符合；投影规则和空指针边界均有 host 测试。** 这是持久化字段语义的唯一归属，不负责 Flash 时序或任务调度。

#### 职责与接口

- `storage_projection_views_to_blob()`：把内核参数视图写入 `param_store_blob_t`。
- `storage_projection_blob_to_boot()`：把 blob 记录恢复为内核启动参数。
- CERB 只落设备核验过的原值；位置上下限必须成对；未回读时退回记录原值。
- VAXOR 使用有限的本地软限位值；`enable` 不持久化。
- 文件中的 `_Static_assert` 锁住 `param_store` 设备槽序与 `motor_protocol` 参数对象序号。

#### 依赖边界

依赖 `motor_control` 的参数视图/启动记录类型和 `param_store` 的 blob API；不 include Arduino、FreeRTOS、HAL，也不拥有 W25Q64。调用链是 `storage_task -> storage_projection -> (motor_control, param_store)`。

#### 验证与发现

`test_storage_projection.cpp` 覆盖 CERB/VAXOR 投影、设备原值、安全/方向槽、成对限位、启动恢复、容量和 motor_id=0 拒绝；PlatformIO 固件构建通过。

两个公共函数在 `blob == NULL` 时安全返回；`views == NULL` 且 `count > 0` 时不写入，`out == NULL` 且 `capacity > 0` 时返回 0。空容量调用保持原有语义。`test_storage_projection.cpp` 已覆盖这些边界。

#### 后续建议

调用方仍应传入有效且足够大的缓冲区；本库只检查空指针和容量组合，不负责验证内存区间或 blob CRC，后者由存储任务在调用前完成。


### `w25q64`

#### 审查结论

**硬件驱动分层符合；host 目前只覆盖 geometry。** `storage_task` 是唯一所有者，库不理解参数字段、保存命令或双槽选择策略。

#### 内部结构

| 文件 | 作用 |
|---|---|
| `include/octospi.h` / `src/octospi.c` | OCTOSPI2 时钟、GPIO、句柄和间接传输控制层 |
| `include/w25q64.h` / `src/w25q64.c` | JEDEC、reset、读、页编程、跨页写和扇区擦除命令层 |
| `include/w25q64_geometry.h` / `src/w25q64_geometry.c` | Flash 范围、页边界和擦除地址纯逻辑 |

公共操作是 `w25q64_init/read/write/program_page/erase_sector()`；状态码区分参数、HAL、超时、ID 和状态错误。`octospi2_*` 只属于该库内部的控制器层。

#### 依赖与所有权

- 依赖 STM32 HAL 和 OCTOSPI2；geometry 可独立 host 编译。
- 地址、长度单位为字节；页编程不得跨页，`w25q64_write()` 负责拆分。
- `storage_task` 在 boot 初始化并独占所有读写；其他任务不得直接访问。
- 库不决定 A/B 槽、CRC、seq 或保存触发条件，这些归 `param_store`/storage 任务。

#### 验证与缺口

`test_w25q64_geometry.cpp` 覆盖范围、页和擦除边界；PlatformIO 固件构建通过。`octospi.c`、命令时序、JEDEC ID、擦写超时和编程后读回只能在目标板/Flash 上验证，当前没有 host HAL 仿真。

#### 后续建议

台架验收应记录 JEDEC ID、读写回读、跨页写、擦除耗时和 HAL 超时映射；不要在 storage 任务之外增加第二个调用者。


### `ws2812`

#### 审查结论

**职责符合；这是目标板物理发送库，暂无独立 host 测试。** 状态到颜色的策略留在 `src/led_task.cpp`，本库只编码并发送 RGB 波形。

#### 职责与接口

- `ws2812_init()` 初始化 SPI6、PA5/PA7 和 BDMA1 Channel0。
- `ws2812_submit_rgb(r, g, b)` 把 RGB 按 WS2812 要求编码为 GRB，非阻塞提交 DMA；上一帧未完成时返回 false。
- 帧缓冲位于 `.bdma_noinit`/RAM_D3，帧尾保留低电平复位时间。
- IRQ handler 由本库持有：`BDMA_Channel0_IRQHandler()` 和 `SPI6_IRQHandler()`。

#### 依赖与所有权

前置条件是 `main.cpp` 先调用 `MX_DMA_Init()`；DMA 控制器级时钟/NVIC 归 `lib/dma`，SPI6 句柄、BDMA 配置和缓冲区归本库。运行期唯一调用者是 `led_task`，库不读取链路状态、不判断安全状态。

#### 验证与缺口

PlatformIO 固件构建通过。当前没有 host 目标，因为 SPI6 EOT、BDMA 完成中断、缓存/链接段和 WS2812 电气波形无法由纯 C 单测证明。目标板验证应检查颜色顺序、忙返回、DMA 完成后再次提交和复位脉冲。

#### 后续建议

保持 `MX_DMA_Init()` 的初始化顺序契约；如果增加多灯珠或帧队列，需重新评估固定缓冲大小、BDMA 可访问内存和 `led_task` 所有权。
