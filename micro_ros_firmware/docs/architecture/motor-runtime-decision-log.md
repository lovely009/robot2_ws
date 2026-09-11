# micro-ROS 电机运行时架构开发基线

> 对应 ADR：[ADR 0007：micro-ROS 采用发现驱动的聚合式电机运行时](../adr/0007-micro-ros-discovery-driven-motor-runtime.md)。
> 本文是 `micro_ros_firmware` 重构的开发基线；旧台账中的逐条讨论已归并，不要求开发者逐条回看。
> 原始基线日期：2026-09-04。
>
> **当前适用性（2026-09-11）**：架构边界仍有效；`motor_rt_task`、ID 快照、Profile 绑定、消息、结果事件、P0/P1/P2 与 RX ring 已落地。第 10 节实施顺序和第 11 节开放项保留为历史基线；当前未闭环项统一见[运行时验证计划](../plans/active/motor-runtime-validation.md)。

## 1. 目标和原则

系统由 STM32H723 上的 micro-ROS 固件控制约 40 台 Vaxor/CERB 外置电机，使用
FDCAN1、FDCAN2、FDCAN3，三路均为 CAN-FD 仲裁段 1 Mbps、数据段 5 Mbps。电机 ID
在系统内唯一；电机可动态上线、离线和热插拔。

开发只遵守以下原则：

- MCU 不维护“每台电机一行”的固定安装表，只维护运行时注册表。
- 固件和上层各维护一份按型号组织的 `MotorModelProfile`；上层维护
  `motor_id → joint_name`、方向、零位和关节语义。
- 两类电机默认 MIT 模式。协议字节只存在于对应协议适配器中。
- 控制和状态目标频率均为 100 Hz；`motor_rt_task` 以 1 ms 服务节拍运行。
- FDCAN ISR 只接收原始帧入环形缓冲；`motor_rt_task` 是唯一解码和正常发送的业务所有者。
- ROS 聚合消息不能直接当 CAN 帧发送；实时任务必须经过协议适配器组帧和调度。
- 实时路径使用固定容量和静态内存，不在 ISR、控制和 CAN 发送路径动态分配。

## 2. 总体数据流

```text
ROS 2 controller
  └─ ros2_control position/velocity/effort interfaces
       └─ upper hardware interface: joint ↔ motor_id
            ├─ MotorCommandArray (100 Hz, complete snapshot)
            ├─ MotorPolicy / MotorConfig (low frequency)
            └─ micro-ROS Agent / USB CDC
                         │
                    micro_ros_task
                         │ 只校验消息、写 mailbox、发布状态副本
              command/config mailbox
                         │
                   motor_rt_task (1 ms)
              ┌──────────┼──────────┐
              │          │          │
        registry     safety       protocol adapters
              │          │       Vaxor / CERB
              └──────────┴──────────┘
                         │
                 FDCAN1 / FDCAN2 / FDCAN3
                         │
                    外置电机
```

上行反馈反向经过 `motor_rt_task`：原始 CAN 帧 → 适配器解码 → 运行时状态副本 →
micro-ROS 状态消息。拓扑变化和配置结果走低频消息，不塞进 100 Hz 运动状态。

## 3. 运行时流程

### 3.1 启动、发现和热插拔

1. 初始化三路 FDCAN、静态缓冲、协议注册表和运行时注册表。
2. 启动 `motor_rt_task` 后进入发现窗口；Vaxor 用自身 20 ms 状态识别型号，CERB
   用反馈/管理读取识别身份。CERB 的 `0x080 SYNC` 只用于发现、重连和热插拔探测。
3. 身份确定后选择固件 `MotorModelProfile`，读取或确认设备能力；失败则保持禁能并
   发布原因。
4. 发布一次全量 `MotorTopology`。设备再次上线、离线、身份变化或能力冲突时更新
   拓扑并发布事件。
5. 只有完成识别、能力校验、策略授权和新的完整命令快照后，电机才允许进入 `ACTIVE`。

### 3.2 1 ms 服务节拍

`motor_rt_task` 每 1 ms 做有界工作：

1. 从 FDCAN1/2/3 RX 环逐帧取出原始帧。
2. 按总线和 CAN ID 找到注册表项，交给协议适配器解码。
3. 更新在线时间、运动状态、诊断和协议事务状态。
4. 处理低频发现/配置事务的状态机。
5. 按 P0/P1/P2 优先级向三路硬件 TX FIFO 泵送帧。

该节拍是服务频率，不是 1 kHz 控制频率。任何单次工作都必须有上限，不能在其中
等待 ROS、Agent、设备响应或 W25Q64。

### 3.3 10 ms 控制边界

1. 复制 `command_snapshot_mailbox` 中最新的完整命令快照。
2. 校验 `sequence`、时间、ID、数值、重复项、策略和设备生命周期。
3. 快照中缺席的已受控电机执行软停，不沿用上一周期目标。
4. 超过 30 ms 没有新快照时进入软停；超过 100 ms 或 micro-ROS 会话断开时发送
   协议级停机/失能并清除运动授权。
5. 只有 `ACTIVE` 且通过安全检查的电机才由适配器生成 MIT 控制帧，写入该总线的
   P1 latest-wins 槽；实际发送由 1 ms 服务完成。

恢复必须重新收到有效策略、显式 Enable 和新的完整运动快照，不能自动复用旧命令。

## 4. 运行时注册表和生命周期

注册表是编译期有界的，容量至少覆盖 64 台设备。表项只保存运行时事实和引用：

| 数据组 | 内容 |
|---|---|
| 定位 | `motor_id`、`bus_index`、最近接收时间 |
| 身份 | 协议、厂家型号、版本/序列号（协议提供时） |
| 能力 | 支持模式、编码器/单位换算、设备限值和限位能力 |
| 控制 | 当前策略、Enable、命令序列、目标和 freshness |
| 状态 | 位置、速度、力矩、电流、电压、温度、模式、故障 |
| 诊断 | RX/TX 统计、协议错误、事务状态和最近失败原因 |

生命周期保持简单：

```text
ABSENT → DISCOVERED → IDENTIFYING → READY_DISABLED → ACTIVE
                                      │       │
                                      └───────┴→ SAFE_STOP / FAULT / CONFIG_CONFLICT
                                                        │
                                                        └→ OFFLINE → DISCOVERED
```

只有 `ACTIVE` 允许发送运动目标；所有其他状态保持禁能或发送安全动作。

## 5. 型号和协议适配器

### 5.1 型号 Profile

Profile 是编译期型号目录，不是每台电机的安装表。至少包含协议类型、编码器和单位
换算、减速比、额定/峰值参数、默认 MIT 参数及限位能力。运行时由设备身份选择；
上层保留对应副本用于关节配置和校验。

规格书已确认的型号基线：

| 型号 | 主要参数 | 运行方式 |
|---|---|---|
| CERB R48-101 | `101:1`、30 RPM、额定/峰值 `5/11 Nm`、48 V、21 bit 编码器 | CAN，支持驱动器侧限位配置 |
| CERB R58-101 | `101:1`、30 RPM、额定/峰值 `20/45 Nm`、48 V、21 bit 编码器 | CAN，支持驱动器侧限位配置 |
| Vaxor 25 mm 系列 | 12~18 V、绝对编码器；规格书明确给出 30/50 速比参数 | CAN-FD，上电默认 MIT，无硬件限位 |

Vaxor 系列名中的 80 速比当前没有完整参数，不能用 30/50 档外推；补齐规格前不
加入可用于控制的 Profile。

### 5.2 适配器边界

| 能力 | Vaxor | CERB |
|---|---|---|
| 身份 | 从每 20 ms 自主状态识别型号 | 通过反馈/管理读取识别 |
| 默认控制 | 标准 MIT 单电机帧：`0x501 + 4*N`，12 字节 | MIT 单电机帧：`0x110 \| Dev_ID`，9 字节 |
| 反馈 | 自主状态；MIT 命令也可有响应 | 默认 MIT 命令反馈；SYNC 用于发现/探测 |
| 多电机帧 | 不使用 | `0x200` 仅在明确的非 MIT 模式使用 |
| 限位 | 固件软件限位 | 驱动器侧配置，MCU 负责事务和核验 |

通用运行时只处理“身份、能力、状态、控制意图、原始 CAN 帧”。新增协议只增加
适配器，不修改 `motor_rt_task`、mailbox 或总线调度器。

## 6. ROS 2 和 micro-ROS 接口

### 6.1 高频接口

上层控制器只使用 `ros2_control` 官方 `position`、`velocity`、`effort` command
interfaces；`sensor_msgs/msg/JointState` 只用于上层状态发布，不作为控制命令。

| 接口 | 必要字段/语义 | 方向 |
|---|---|---|
| `MotorCommandArray` | 同步时间、`uint32 sequence`、`uint8 count`、最多 64 项；每项 `uint16 motor_id` 和 `float32 position/velocity/effort` | 上层 → MCU |
| `MotorMotionStateArray` | 同步时间、最多 64 项；每项 `motor_id`、位置 `rad`、速度 `rad/s`、力矩 `N·m`、`state_age_ms` | MCU → 上层 |
| `MotorStatusArray` | `motor_id`、温度、电流、电压、驱动模式、统一故障码、厂家故障码、保护/限位标志、`online`、生命周期、`state_age_ms` | MCU → 上层 |
| `MotorTopology` / `MotorEvent` | 总线、协议、型号、能力、上线/离线/冲突等低频信息 | MCU → 上层 |

高频自定义消息中的物理量使用 `float32`；上层转换为标准 `JointState` 的 `float64`。
状态缺失的浮点量使用 `NaN`，逻辑 `motor_id` 使用 `uint16`，原始 CAN ID 使用
独立的 `uint32`。状态消息不携带 `joint_name`。

`protection_flags` 和 `limit_flags` 只使用集中定义的位标志；位编号放在消息/代码
常量的单一位置，不在业务模块中散落。当前统一单位为：电流 `A`、电压 `V`、温度
`°C`、限位数值 `rad`。

### 6.2 低频接口

| 接口 | 内容 | 规则 |
|---|---|---|
| `MotorPolicy` | Enable、MIT `Kp/Kd`、前馈、软件限幅、安全授权 | 运行时策略，不进入 100 Hz 命令 |
| `MotorConfig` | 模式切换、CERB 限位及其他设备配置 | 请求/结果事务，不刷新运动 freshness |

`MotorConfig` 的最小规则：

- 每个请求带 `request_id`；同一 `motor_id` 同时只允许一个未完成事务。
- 有未完成事务时新请求立即返回 `BUSY`，不排队、不覆盖。
- MCU 先校验请求和型号能力，再由协议适配器执行设备操作。
- 需要设备确认的参数必须读回一致后才返回 `OK`。
- 超时按协议阶段处理，可重试步骤最多自动重试 2 次；写入结果不明确时先读回，
  不盲目重复写入。
- 最终不能确认时不恢复运动授权。

统一结果码保持最小集合：

| 码 | 含义 |
|---|---|
| `OK` | 操作完成且已验证 |
| `BUSY` | 同一电机已有未完成事务 |
| `INVALID` | 消息或参数非法 |
| `UNSUPPORTED` | 当前型号/协议不支持 |
| `DEVICE_ERROR` | 驱动器拒绝或协议执行错误，细节看厂家码 |
| `TIMEOUT` | 阶段超时且重试耗尽 |
| `UNCONFIRMED` | 写入/读回结果无法确认或不一致 |

## 7. FDCAN 缓冲和发送调度

### 7.1 接收

每条总线一个独立的静态 RX 环，初始深度为 64 帧；帧元素至少包含：

```text
can_id, is_fd, is_brs, dlc, data[64], rx_tick, bus_index
```

FDCAN ISR 只完成取硬件 FIFO、加时间戳、写环和计数；满时丢弃并累计溢出。协议
解码、拓扑更新和状态更新全部由 `motor_rt_task` 完成。CAN 调试桥删除，任何调试
信息只能读取状态副本、拓扑事件和统计计数。

### 7.2 发送

每条总线一个独立调度器：

| 优先级 | 内容 | 规则 |
|---|---|---|
| P0 | 停机、失能、急停、安全帧 | 最高优先级，按电机去重 |
| P1 | 100 Hz 默认 MIT 控制 | 每台电机一个 latest-wins 槽，不积压旧目标 |
| P2 | SYNC、身份/能力读取、配置事务 | 有界管理队列，不能挤占 P0/P1 |

默认 MIT 不使用聚合 CAN 帧：同一总线有多少台活动电机，就生成多少条单电机 MIT
帧；CERB 超过 8 台也不因此切换到 `0x200`。`0x200` 只在明确的非 MIT 模式下由
CERB 适配器启用，并按每组最多 8 台拆分。

CAN-FD 侧的正常性由三件事保证：协议适配器独立组帧、P1 只保留最新目标、1 ms
调度器持续泵送硬件 FIFO。最终仍需在最大电机数和最坏反馈流量下实测总线负载、RX
溢出、TX 延迟和控制抖动，测试不通过时只调整容量/配额，不改变 ROS 与 CAN 边界。

## 8. 代码模块和所有权

> 实现落位注（2026-09-11）：表中 `src/motor_ros_bridge` 落位为 `src/ros_bridge`
> （编排）+ `src/motor_bridge`（电机域桥）+ `src/micro_ros_task`（Agent 状态机）；
> `src/storage_task` 未创建（W25Q64 待接线）。目录实况以固件 README「代码组织」
> 为准，责任边界不变。

| 模块 | 责任 | 不负责 |
|---|---|---|
| `lib/fdcan` | 三路初始化、过滤器、IRQ、RX ring、TX 调度、HAL FIFO 和底层统计 | 协议解码、ROS、策略判断 |
| `lib/dma` | 共享 DMA 控制器、通道配置和 IRQ 资源管理 | IMU/LED 采样策略和设备业务 |
| `lib/motor_protocol` | Vaxor/CERB 发现、编解码、停止、SYNC、配置事务 | 注册表和 ROS 消息 |
| `lib/motor_control` | 注册表、型号 Profile、生命周期、安全、命令 freshness、状态快照 | ROS API、硬件 HAL 细节 |
| `lib/control_mailbox` | 最新完整运动命令、Policy/Config 请求和连接事件的固定容量交换 | 策略、协议校验、设备等待、CAN 发送 |
| `src/motor_rt_task` | 1 ms 服务、10 ms 控制边界、调用 runtime | 阻塞等待、动态分配 |
| `src/motor_ros_bridge` | 消息校验、mailbox 写入、状态/拓扑发布 | 访问主 RX 环、直接发 CAN |
| `src/storage_task` | W25Q64 低频日志和诊断读取 | 控制闭环、实时配置裁决 |
| `src/led_task` | WS2812 状态显示 | 电机安全和总线操作 |

核心 interface 应保持小而稳定：runtime 输入为“新命令/策略/配置请求”和“原始
CAN 帧”，输出为“状态快照/事件”和“待发送 CAN 帧”。

## 9. DMA、MPU、WS2812、BMI088 和 W25Q64

- FDCAN 正常收发不依赖 DMA；先用中断 + 静态环实现可测量的路径。
- WS2812 属于低优先级显示；若阻塞发送影响 FDCAN 延迟，改为 SPI TX DMA。DMA
  缓冲放在 MPU/cache 策略明确的非缓存区，或在收发前后做 cache maintenance。
- BMI088 不进入电机实时任务；按现有独立任务运行，只有测量证明需要时再引入 DMA。
- W25Q64 不保存电机实时控制状态、上层关节映射或型号 Profile；由单独
  `storage_task` 写入低频事件/故障日志，禁止阻塞 1 ms/10 ms 路径。
- MPU 配置服务于 DMA 缓冲、外设区和任务隔离，不把 MPU 规则扩散到协议业务代码。

## 10. 实施顺序

1. 固化消息包、枚举、Profile 数据结构和静态容量常量。
2. 重构 `lib/fdcan`：三路 RX 环、ISR 所有权、TX 调度器和统计。
3. 在 `motor_control` 内建立 runtime seam，并接入 `motor_protocol` 的 Vaxor/CERB 解析与 MIT 组帧。
4. 接入 1 ms/10 ms `motor_rt_task`，删除旧固定四电机路径和 CAN 调试桥。
5. 接入 micro-ROS mailbox、状态/拓扑发布和低频配置事务。
6. 先做主机单元测试，再做单总线、双总线、三总线和约 40 台电机的台架测试。

开发阶段只允许一个模块拥有正常电机 TX；旧路径迁移完成前不得与新运行时并行发送。

## 11. 原始实现阶段开放项（历史）

> 下列条目保留 2026-09-04 基线原文；消息、QoS、公共状态位、过滤器和事务框架此后已落地。当前开放项见[运行时验证计划](../plans/active/motor-runtime-validation.md)。

- 自定义消息包的 `.msg` 文件名、字段顺序和 QoS。
- Vaxor/CERB 管理事务的具体对象、帧过滤器和各阶段超时值。
- 三路总线在最坏反馈流量下的 TX 配额、RX 深度和实测负载。
- `protection_flags`、`limit_flags` 和低频结果码的公共头文件/消息常量。

这些是实现参数，不再改变本文的总体架构边界。

## 变更记录

| 日期 | 变更 |
|---|---|
| 2026-09-04 | 建立总体架构、运行流程、CAN 缓冲、代码模块和外设边界。 |
| 2026-09-04 | 将此前 D-001~D-070 的逐条讨论收敛为本开发基线；保留必要协议和安全约束，删除不利于实现的特殊化展开。 |
| 2026-09-04 | 固件实施：删除旧 CAN 调试桥、raw CAN snapshot 与 debug TX mailbox；固定四电机旧路径其余部分暂保留，待运行时重构替换。 |
| 2026-09-04 | 将 lib 顶层边界收敛为 `fdcan`、`dma`、`motor_protocol`、`motor_control`、`control_mailbox`、`imu`、`w25q64`、`ws2812`；过细的 CAN、Profile、mailbox 占位职责合并回对应库，DMA 保持独立以支持多个外设复用。 |
| 2026-09-04 | 直接重构 `control_mailbox`：移除 `motor_control` 依赖和 per-joint/per-index 存储；命令改为最多 64 项的 `uint16 motor_id` 完整快照，配置改为有界语义 FIFO，连接/授权边沿保持幂等。 |
