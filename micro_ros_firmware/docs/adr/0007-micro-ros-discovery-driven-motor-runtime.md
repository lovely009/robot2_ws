# ADR 0007：micro-ROS 采用发现驱动的聚合式电机运行时

**Status**: proposed（核心运行时已实现；待最大规模与实时台架证据后转 accepted）
**Scope**: `micro_ros_firmware`，STM32H723，FDCAN1/2/3，约 40 台外置电机

> **实现状态（2026-09-11）**
>
> | 已实现并有自动测试 | 仍需验证或确认 |
> |---|---|
> | 动态发现、全局唯一 `motor_id`、Profile 显式绑定 | 三总线约 40 台最大规模与 USB CDC 余量 |
> | 完整命令快照、30/100 ms 安全时基、授权门禁 | 厂家级软停与 CERB 管理对象 |
> | 单一 `motor_rt_task`、每总线 SPSC RX ring、P0/P1/P2 | RX/P2/TX 最坏预算、WCET/jitter、P0/P1 延迟 |
> | 消息、QoS 基线、结果事件、公共状态位 | BUS_OFF 恢复与 Profile 自动一致性机制 |
>
> 详细现状见 [ARD-0007](../archive/ARD-0007-micro-ros-motor-runtime.md)；开放项由[运行时验证计划](../plans/active/motor-runtime-validation.md)维护。

## 2026-09-18 勘误（实现现状优先）

- **控制边界**：实际为 1 ms 服务（RX drain / 内核 service / SYNC / TX）+ 20 ms 控制边界，非本文的 10 ms。
- **消息面**：**3 个订阅** `/micro_ros_do/motor_command_array` + `/micro_ros_do/motor_trajectory` + `/micro_ros_do/motor_policy`，**6 个发布**（`motion_states`、`motor_status_array`、`motor_topology`、`motor_events`、`trajectory_state`、`diagnostics`）。MotorBind/MotorConfig 话题与独立 `motor_config_result` 话题均不存在。**参数树只放静态量**（`m<id>.pos_min/pos_max/vel_limit/torque_limit` + `system.*`）；动态量 enable/kp/kd 走 `/micro_ros_do/motor_policy`（稀疏 + 防复权代数）。结果经内核事件 `motor_events` 与参数错误码给出。
- **型号绑定**：内核经 READ_IDENTITY 事务读设备 SDO 0x1008 自报身份、查内置型号目录自动完成，非上层 Profile 通道。
- **参数持久化**：storage 任务独占 W25Q64 双槽 blob（v3 = 32 B 头 + 64×120 B 记录，每槽 2 个 4 KB 扇区；v1/v2 旧 blob 加载时各自就地迁移到 v3）；设备写事务去抖 500 ms 自动落盘，另支持 `system.save` 手动保存。上电对**六对象**回读比对（位置限位×2 / 最大转速 / 峰值电流 + `0x2650:00` 保护开关 + `0x2541` 方向），不一致报 40、失败报 41。
- **授权**：会话电平每控制边界纯派生（`lib/session_sync/`）；未知或在线即授权，断连立即 P0 撤权；冻结态恢复需 `SET_ENABLE(true)` 后收到新快照。
- **轨迹通道**：`MotorTrajectory` 多点轨迹在 20ms 控制边界段内线性插值执行；声明的电机由轨迹独占（快照对其既不下发也不软停），30/100ms 快照级联收窄到快照通道——轨迹自带时长，"多久没收到快照"不是它的失效判据（对 ARD §13.2 的范围限定）。无活动轨迹时行为与改动前逐字节一致。
- **故障清除**：参数 `system.clear_fault`。按成因区分——身份冲突锁存拒绝（ARD §9.3），DEV_PARAM 改走一次回读重比对，厂家故障发 CERB MIT 控制字 bit5（仅非运动态受理）。Vaxor 无清错手段 → `UNSUPPORTED`。
- **参数面容量**：每电机参数由 7 项收缩到 **4 项**（enable/kp/kd 移到 `/micro_ros_do/motor_policy`），`PARAM_BRIDGE_MAX_PARAMS` 128 → **208**；系统参数 8 → 10。
  **2026-09-20 更新**：以上是当时的记录，此后该值又变更多次（176 → 224 → 256 → 416 → **64**，历次理由见 git 历史）。**当前值与推导以 `include/param_bridge.h` 的容量注释为唯一来源**，每电机项数由 `src/param_bridge.cpp` 的 `kParamMotorFields[]` 显式列出（当前 5 项）。「208 覆盖 48 台」已被台架推翻（208 槽实测只剩 1,776 B 堆余量）。
- **策略通道**：`/micro_ros_do/motor_policy` 走**独立的有界 mailbox**（不走 config FIFO——那队列深 16，装不下最多 64 台的策略消息）。内核 `SET_POLICY` 单台入口与批量入口 `motor_control_apply_policy` 共用同一套落地逻辑；代数门是「`policy_enable` 为 false 时 `enabled=true` 要求代数严格前进」一处判定，四条冻结路径无需改动。
- **BUS_OFF 恢复**：HAL 回调清 `CCCR.INIT` 自清恢复，非仅计数。
- **W25Q64**：已接线使用，非休眠占位。
- **ARD 正文归档**：完整 ARD-0007 正文已移至 [`docs/archive/ARD-0007-micro-ros-motor-runtime.md`](../archive/ARD-0007-micro-ros-motor-runtime.md)，现行设计以本勘误与 `docs/architecture/motor-runtime-decision-log.md` 为准。

## Context

现有固件以固定测试电机、按关节名路由和分散的 CAN 收发逻辑为中心，不能自然支持
跨三路总线的 Vaxor/CERB 混合设备、动态拓扑和热插拔。ROS 回调、协议字节、实时
控制和 CAN 缓冲彼此耦合，也无法稳定保证 100 Hz 控制。

系统约束如下：

- 三路 FDCAN 使用 CAN-FD，仲裁段 1 Mbps、数据段 5 Mbps。
- 电机 ID 全局唯一；Vaxor 自主每 20 ms 上报状态，CERB 可通过协议读取/反馈。
- 两类电机默认 MIT 模式；上层使用 ROS 2 标准控制接口。
- 控制和反馈目标为 100 Hz；固件需要保留本地安全时基。

## Decision

### 1. 拓扑和参数

- MCU 依据设备上报/读取的身份建立编译期有界运行时注册表，不维护每台电机的固定表。
- 固件和上层各维护按型号组织的 `MotorModelProfile`。Profile 保存协议、编码器、
  单位换算、减速比、额定/峰值参数、默认 MIT 参数和能力；运行时按设备身份选择。
- 上层负责 `motor_id → joint_name`、方向、零位和关节语义；MCU 不维护 `joint_name`。

### 2. 任务和时序

- 只保留一个 `motor_rt_task`：每 1 ms 服务三路 FDCAN，每 10 ms 在控制边界处理一次
  完整命令快照。
- 命令快照超过 30 ms 未更新时软停；超过 100 ms 或 micro-ROS 会话断开时协议级停机/
  失能并清除运动授权。恢复必须重新 Enable 并收到新的完整快照。
- ROS 时间用于消息时间戳；命令 freshness、安全超时和 CAN 调度使用 MCU 单调时基。

### 3. ROS 2 / micro-ROS 边界

- 上层控制器使用 `ros2_control` 官方 `position`、`velocity`、`effort` command interfaces。
- micro-ROS 使用固定容量的自定义 `MotorCommandArray`、`MotorMotionStateArray`、
  `MotorStatusArray` 和低频 `MotorTopology`/`MotorEvent`。
- `MotorCommandArray` 是 100 Hz 完整快照，最多 64 台，每项只携带 `motor_id`、位置、
  速度和力矩目标；不携带关节名、原始 CAN 字段、协议字节、MIT 参数或 Enable。
- 状态拆成运动和诊断两类；上层将运动状态按自身映射转换为标准
  `sensor_msgs/msg/JointState`。状态物理单位为 `rad`、`rad/s`、`N·m`，缺失浮点值为
  `NaN`，逻辑 ID 为 `uint16`。
- `MotorPolicy` 负责 Enable、MIT 参数、软件限幅和安全授权；`MotorConfig` 负责模式
  切换、CERB 限位和设备配置。两者不进入 100 Hz 命令，也不刷新命令 freshness。
- `MotorConfig` 使用 `request_id` 请求/结果事务。同一电机同时只允许一个未完成事务，
  新请求返回 `BUSY`。MCU 校验、协议执行并完成设备读回后才返回 `OK`；可重试步骤最多
  自动重试 2 次，最终无法确认返回 `TIMEOUT` 或 `UNCONFIRMED`。

### 4. FDCAN 所有权和调度

- FDCAN ISR 只从硬件 FIFO 取原始帧，写入对应总线的静态 RX 环；不解码、不调用 ROS、
  不发送正常控制帧。
- `motor_rt_task` 是 RX 环唯一消费者、协议解码者和正常 CAN TX 所有者。CAN 调试桥
  删除，调试只能读取状态副本、拓扑事件和统计。
- FDCAN1/2/3 各自拥有 RX 环和 TX 调度器；软件容量固定，RX 初始深度 64 帧。
- TX 使用三类优先级：P0 安全帧，P1 周期控制，P2 发现/身份/配置。P1 为每台电机
  一个 latest-wins 槽，不能积压旧目标；P2 不能挤占 P0/P1。
- ROS 消息永远不直接映射为 CAN 帧。协议适配器按电机生成厂家帧，默认 MIT 下 Vaxor
  和 CERB 都使用单电机帧；CERB `0x200` 多控只在明确的非 MIT 模式下使用，`0x080`
  SYNC 只用于发现和热插拔探测。

### 5. 协议适配器

`motor_rt_task` 只依赖统一的身份、能力、状态、控制意图和原始 CAN 帧接口。
Vaxor/CERB 适配器分别封装发现、编解码、MIT 控制、反馈、停止、SYNC 和管理事务。
新增协议只增加适配器，不修改实时循环、消息边界或 FDCAN 调度器。

### 6. 内存和其他外设

- 注册表、mailbox、状态、事务和 CAN 缓冲均使用静态有界内存；实时路径禁止动态分配。
- FDCAN 不依赖 DMA。WS2812 和 W25Q64 不得阻塞电机实时路径；必要时 WS2812 使用
  低优先级 SPI DMA，DMA 缓冲和 cache 规则由 MPU 明确管理。
- W25Q64 只由单独存储任务保存低频事件/故障日志，不保存实时控制状态、型号 Profile
  或上层关节映射。

## Consequences

### 正面

- 新电机上线由身份和 ID 驱动，不需要修改固件中的每台电机表。
- CAN-FD 的协议格式、实时调度和 ROS 接口解耦；过期控制目标不会在队列中排队。
- 单一实时所有权使 RX、TX、安全和跨总线一致性可测试，状态和拓扑也能独立演进。

### 代价

- 需要实现固定容量的自定义消息和运行时注册表。
- 上层必须维护 ID 到关节的映射，并处理拓扑事件和配置事务结果。
- 三路总线的容量、负载和时序必须通过最大电机数的台架测试确认。

## 不在本 ADR 中展开

具体消息包名、字段顺序、QoS、CAN 过滤器、厂家管理对象、阶段超时值和公共状态位常量
属于实现规格；统一记录在补充文档或代码头文件中，不改变本 ADR 的边界。

## 参考

- [micro-ROS 电机运行时架构开发基线](../architecture/motor-runtime-decision-log.md)
- [电机运行时架构验证计划](../plans/active/motor-runtime-validation.md)
