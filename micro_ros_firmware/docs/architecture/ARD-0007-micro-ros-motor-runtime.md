# ARD-0007：micro-ROS 发现驱动的聚合式电机运行时——架构需求与设计文档

> **ARD**：Architecture Requirements & Design Document（架构需求与设计文档）  
> **系统**：`micro_ros_firmware`  
> **目标 MCU**：STM32H723  
> **电机总线**：FDCAN1 / FDCAN2 / FDCAN3  
> **目标设备规模**：约 40 台，架构容量至少 64 台  
> **主要电机族**：Vaxor / CREB  
> **文档状态**：Proposed / Development Baseline  
> **日期**：2026-09-04  
> **上位决策**：ADR 0007《micro-ROS 采用发现驱动的聚合式电机运行时》

---

## 0. 文档目的

本文把 ADR 0007 与《micro-ROS 电机运行时架构开发基线》中已经确认的设计收敛为一份可直接用于：

- 固件模块划分；
- ROS 2 / micro-ROS 消息设计；
- Vaxor / CREB 协议适配；
- FDCAN 收发与调度实现；
- 安全状态机实现；
- 单元测试、集成测试和约 40 台电机台架验收；
- 后续代码评审和架构变更控制；

的完整 ARD。

本文不凭空补齐设备规格书没有确认的协议常量。凡原始材料仍未确定的 QoS、CAN 过滤器、厂家管理对象、离线阈值、阶段超时、TX 配额等，统一标记为 **TBD**，并作为实现/验收前必须闭环的项目。

---

## 1. 规范性语言

本文中的关键字含义如下：

- **MUST / 必须**：架构强制要求，不满足即视为不符合 ARD。
- **MUST NOT / 禁止**：架构明确禁止。
- **SHOULD / 应**：强烈建议，偏离时必须在代码评审或新 ADR 中说明理由。
- **MAY / 可**：允许的实现选择。
- **TBD**：当前材料未确定，不能由实现者自行默认为某个值；必须在实现规格、代码常量或补充文档中固化。
- **DERIVED**：由既有架构原则直接推导出的约束，用于消除实现歧义，不新增厂家协议事实。

---

## 2. 输入基线与优先级

本文依据以下两份材料形成：

1. `0007-micro-ros-discovery-driven-motor-runtime.md`
2. `motor-runtime-decision-log.md`

若三者未来产生冲突：

1. 已 accepted 的 ADR 具有最高架构决策优先级；
2. 本 ARD 负责描述可实施的需求和设计边界；
3. 厂家协议事实以经确认的规格书/协议头文件为准；
4. 未确认的参数不得以“经验值”进入安全或协议路径。

---

## 3. 审查结论

### 3.1 总体结论

两份输入材料在核心架构上保持一致，没有发现需要推翻 ADR 的直接冲突。已经形成的主线是：

- 发现驱动、运行时注册；
- MCU 不维护固定“关节安装表”；
- 三路 FDCAN 由一个实时业务所有者统一调度；
- 1 ms 服务节拍 + 10 ms 控制边界；
- ROS 聚合语义与厂家 CAN 协议彻底隔离；
- P0/P1/P2 分级调度，P1 latest-wins；
- Enable/安全策略与 100 Hz 运动命令分离；
- 静态有界内存；
- 配置采用可验证的事务模型；
- 最大规模必须通过真实台架验证。

### 3.2 审查后需要显式化的风险

| 编号 | 审查项 | 结论 |
|---|---|---|
| REV-01 | “状态目标 100 Hz”与 Vaxor 20 ms 自主上报 | 100 Hz 状态发布不等于 100 Hz 新测量。必须携带 `state_age_ms`，不能把重复发布的最近状态解释为新采样。 |
| REV-02 | `MotorPolicy` 的“前馈”与 `MotorCommandArray.effort` | 语义可能重叠。`effort` 已承担 100 Hz 力矩/前馈目标；Policy 中若仍保留前馈，必须定义为低频偏置/限制类参数，否则应删除。**TBD**。 |
| REV-03 | 30 ms “软停”的具体协议动作 | 架构已要求软停，但 Vaxor/CREB 各自如何实现尚未固化。必须由适配器给出可测试定义。**TBD**。 |
| REV-04 | OFFLINE 判定阈值 | 生命周期包含 `OFFLINE`，但不同协议的离线超时未定义。**TBD**。 |
| REV-05 | CREB “CAN”与系统“三路 CAN-FD” | 总线硬件运行在 CAN-FD 环境，但 CREB 各类帧实际 `is_fd/is_brs/dlc` 必须由协议适配器明确，不能根据总线类型猜测。**TBD**。 |
| REV-06 | QoS、消息包名、字段最终顺序 | 明确属于实现规格，必须在接入 micro-ROS 前固化。**TBD**。 |
| REV-07 | 每 1 ms 的最大 RX 处理量、P2 配额、TX pump 配额 | 原设计要求“有界工作”，但预算未量化。必须通过计算和台架测试确定。**TBD**。 |
| REV-08 | 全局唯一 `motor_id` 的冲突处理 | **DERIVED**：同 ID 出现在不同总线/身份不一致时必须进入冲突状态并禁止运动，不能“后到覆盖先到”。 |
| REV-09 | ROS 时间与安全时基 | 必须彻底分离。ROS 时间只用于时间戳；freshness、超时、调度全部由 MCU 单调时基驱动。 |
| REV-10 | 配置事务是否可阻塞实时循环 | 禁止。事务必须为分阶段状态机，任何设备等待都跨周期推进。 |

---

## 4. 系统目标

### 4.1 功能目标

系统 MUST：

1. 在 STM32H723 上同时管理 FDCAN1、FDCAN2、FDCAN3。
2. 支持约 40 台 Vaxor/CREB 混合电机。
3. 支持设备动态上线、离线和热插拔。
4. 根据设备身份选择 `MotorModelProfile`，而不是依赖固定安装表。
5. 为 ROS 2 上层提供统一的电机控制、运动状态、诊断状态、拓扑和配置接口。
6. 默认以 MIT 模式控制 Vaxor 与 CREB。
7. 在正常控制路径实现 100 Hz 命令边界。
8. 在 ROS/Agent 异常时由 MCU 独立执行安全降级。
9. 保证历史运动命令不会因队列积压而在未来被发送。
10. 在最大负载下仍保持可测量、可界定、可验证的实时行为。

### 4.2 非功能目标

系统 MUST 满足：

- **确定性**：实时路径工作量必须有界。
- **可扩展性**：新增协议不得修改实时循环和 ROS 消息边界。
- **可诊断性**：溢出、协议错误、事务失败、拓扑变化必须可观测。
- **静态内存**：ISR、控制、CAN 发送路径禁止动态分配。
- **单一所有权**：正常电机 CAN TX 只能有一个业务所有者。
- **故障安全**：通信异常不能导致继续沿用旧命令。
- **协议隔离**：ROS 语义不得泄漏厂家 CAN 字节级细节。

---

## 5. 范围

### 5.1 本 ARD 范围内

- motor runtime；
- FDCAN 软件接收环和发送调度器；
- Vaxor/CREB 协议适配器边界；
- runtime registry；
- 生命周期与安全状态；
- command/config mailbox；
- micro-ROS 电机桥；
- 100 Hz 控制快照；
- 状态、诊断和拓扑；
- 配置事务；
- W25Q64/WS2812/BMI088 与实时电机路径的边界；
- 主机测试与台架验证要求。

### 5.2 本 ARD 不负责

- 上层机器人运动学；
- `joint_name` 的业务语义；
- ros2_control 控制器算法；
- 未确认的厂家私有管理对象；
- 机械限位设计本身；
- 电机内部驱动算法；
- USB CDC / Agent 的完整传输实现；
- W25Q64 文件系统或日志格式细节；
- BMI088 算法。

---

## 6. 系统上下文

```text
+---------------------------------------------------------------+
|                         ROS 2 Host                            |
|                                                               |
|  ros2_control controller                                     |
|     position / velocity / effort command interfaces          |
|                         |                                     |
|                         v                                     |
|  Upper Hardware Interface                                    |
|     joint_name <-> motor_id                                   |
|     direction / zero / joint semantics                        |
|                         |                                     |
|       +-----------------+------------------+                  |
|       |                 |                  |                  |
| MotorCommandArray   MotorPolicy        MotorConfig            |
+-------|-----------------|------------------|------------------+
        |                 |                  |
        v                 v                  v
+---------------------------------------------------------------+
|                 STM32H723 / micro_ros_firmware                |
|                                                               |
|  motor_ros_bridge / micro_ros_task                            |
|       | validate + mailbox + publish snapshots                |
|       v                                                       |
|  command/config mailbox                                       |
|       |                                                       |
|       v                                                       |
|  motor_rt_task (1 ms service, 10 ms control boundary)         |
|       |                                                       |
|  +----+-------------+--------------+----------------------+   |
|  | registry         | safety       | protocol adapters    |   |
|  |                  |              | Vaxor / CREB         |   |
|  +----+-------------+--------------+----------------------+   |
|       |                                                       |
|       v                                                       |
|  FDCAN1 scheduler   FDCAN2 scheduler   FDCAN3 scheduler       |
+-------|------------------|------------------|------------------+
        |                  |                  |
        v                  v                  v
      Motors             Motors             Motors
```

上行路径：

```text
FDCAN raw frame
    -> RX ring
    -> motor_rt_task
    -> protocol adapter decode
    -> registry/state snapshot
    -> motor_ros_bridge
    -> MotorMotionStateArray / MotorStatusArray / Topology / Event
    -> upper hardware interface
    -> sensor_msgs/msg/JointState
```

---

## 7. 核心架构原则

### AR-001 发现驱动

MCU MUST 通过设备上报、反馈或管理读取建立运行时拓扑，不维护按设备实例静态写死的安装表。

### AR-002 型号驱动

每台设备的协议和换算能力 MUST 由设备身份选择 `MotorModelProfile`。

### AR-003 关节语义上移

`motor_id → joint_name`、方向、零位和关节语义 MUST 位于上层；MCU MUST NOT 保存 `joint_name`。

### AR-004 协议封装

厂家 CAN ID、DLC、字节布局、缩放系数、MIT 打包、停止、SYNC、管理对象等 MUST 只存在于协议/Profile 层。

### AR-005 单一实时业务所有者

`motor_rt_task` MUST 是：

- RX 环唯一消费者；
- 协议解码业务所有者；
- 正常电机 CAN TX 唯一业务所有者。

### AR-006 消息与帧分离

ROS 消息 MUST NOT 直接映射为 CAN 帧。

### AR-007 有界实时路径

ISR、1 ms 服务、10 ms 控制边界的 CPU 工作量和内存使用 MUST 有上限。

### AR-008 安全本地化

命令 freshness、安全超时和 CAN 调度 MUST 使用 MCU 单调时基，不依赖 ROS 时间。

---

## 8. 运行时数据模型

### 8.1 运行时注册表

注册表容量 MUST 至少为 64 个电机实例，并在编译期固定上限。

建议逻辑结构：

```c
MotorRuntimeEntry {
    // location
    motor_id
    bus_index
    last_rx_tick

    // identity
    protocol_type
    model_id
    version_or_serial_ref

    // capability/profile
    profile_ref
    capability_flags

    // lifecycle
    lifecycle_state
    online

    // policy/control
    enable
    safety_authorized
    last_command_sequence
    last_command_rx_tick
    target_position
    target_velocity
    target_effort

    // state
    position
    velocity
    effort
    current
    voltage
    temperature
    drive_mode

    // diagnostics
    unified_fault_code
    vendor_fault_code
    protection_flags
    limit_flags
    protocol_error_count
    transaction_state
    last_failure_reason
}
```

上面是逻辑字段集合，不要求按此顺序布局。

### 8.2 `MotorModelProfile`

Profile MUST 是“型号目录”，而不是“设备实例表”。

至少包含：

- 协议类型；
- 型号身份；
- 编码器信息；
- 原始量到 SI 单位的转换；
- 减速比；
- 额定参数；
- 峰值参数；
- 默认 MIT 参数；
- 支持模式；
- 限位能力；
- 必要的协议能力标志。

### 8.3 已确认型号基线

| 型号 | 已确认参数 | 架构行为 |
|---|---|---|
| CREB R48-101 | 101:1；30 RPM；额定/峰值 5/11 N·m；48 V；21 bit 编码器 | 支持驱动器侧限位配置 |
| CREB R58-101 | 101:1；30 RPM；额定/峰值 20/45 N·m；48 V；21 bit 编码器 | 支持驱动器侧限位配置 |
| Vaxor 25 mm 系列 | 12~18 V；绝对编码器；30/50 速比参数已确认 | CAN-FD；上电默认 MIT；无硬件限位 |

Vaxor 80 速比的完整参数未确认前 MUST NOT 进入可运动的 Profile。

---

## 9. 生命周期状态机

规范状态：

```text
ABSENT
  |
  v
DISCOVERED
  |
  v
IDENTIFYING
  |
  v
READY_DISABLED
  |
  +-------> ACTIVE
  |           |
  |           +------------------+
  |                              |
  +--------------------------+   |
                             v   v
                         SAFE_STOP
                         FAULT
                         CONFIG_CONFLICT
                             |
                             v
                           OFFLINE
                             |
                             v
                         DISCOVERED
```

### 9.1 状态语义

| 状态 | 允许运动帧 | 说明 |
|---|---:|---|
| `ABSENT` | 否 | 无已知设备 |
| `DISCOVERED` | 否 | 已观察到设备，但身份未确认 |
| `IDENTIFYING` | 否 | 正在识别/读取能力 |
| `READY_DISABLED` | 否 | 身份和能力已确认，但未获得运动授权 |
| `ACTIVE` | 是 | 满足身份、能力、策略、Enable、新鲜命令等全部条件 |
| `SAFE_STOP` | 否 | 已触发软停/安全停机路径 |
| `FAULT` | 否 | 设备或系统故障 |
| `CONFIG_CONFLICT` | 否 | ID/身份/能力/配置冲突 |
| `OFFLINE` | 否 | 设备超时离线 |

### 9.2 进入 `ACTIVE` 的必要条件

电机 MUST 同时满足：

1. 已完成身份确认；
2. 选择到可控制的 `MotorModelProfile`；
3. 能力校验通过；
4. 无配置冲突；
5. 策略有效；
6. 显式 Enable；
7. 安全授权有效；
8. 收到新的完整运动快照；
9. 当前命令未超时；
10. 无禁止运动的故障/保护状态。

### 9.3 冲突规则（DERIVED）

若发现：

- 相同 `motor_id` 同时出现在不同总线；
- 相同 `motor_id` 的协议/型号身份在未完成离线重发现前发生不一致；
- 设备能力与已选择 Profile 冲突；

系统 MUST：

1. 将相关实例标记为 `CONFIG_CONFLICT`；
2. 禁止运动；
3. 发布拓扑/事件；
4. 不以“后到数据覆盖旧表项”的方式自动解决。

---

## 10. 启动、发现与热插拔

### 10.1 启动流程

1. 初始化 FDCAN1/2/3。
2. 初始化静态 RX 环、TX 调度器、统计和 mailbox。
3. 初始化协议目录与 Profile 目录。
4. 初始化空运行时注册表。
5. 启动 `motor_rt_task`。
6. 进入发现窗口。
7. 完成设备识别与能力确认。
8. 发布全量 `MotorTopology`。
9. 等待策略、Enable 和新完整命令快照。
10. 满足条件后个别电机进入 `ACTIVE`。

### 10.2 Vaxor 发现

- Vaxor 使用自身约 20 ms 的自主状态参与发现和身份识别。
- 其自主上报不应在 ISR 中解码。
- MIT 命令响应是否用于补充状态更新由协议适配器负责。

### 10.3 CREB 发现

- CREB 使用反馈/管理读取识别身份。
- `0x080 SYNC` MUST 仅用于发现、重连和热插拔探测，不作为 100 Hz 默认 MIT 控制聚合机制。

### 10.4 热插拔

设备出现以下变化时 MUST 更新拓扑并产生事件：

- 新上线；
- 离线；
- 重新上线；
- 身份变化；
- 总线位置变化；
- 能力冲突；
- 配置冲突。

重新上线的电机 MUST NOT 自动复用离线前的旧运动授权和旧运动目标。

---

## 11. 实时任务与时序

### 11.1 唯一实时电机任务

系统只保留一个核心 `motor_rt_task`。

其两个时间尺度：

- **1 ms**：服务节拍；
- **10 ms**：运动控制边界，即 100 Hz。

### 11.2 1 ms 服务节拍

每次运行 MUST 做有界工作：

1. 从 FDCAN1/2/3 RX 环取原始帧；
2. 定位对应协议/运行时对象；
3. 调用适配器解码；
4. 更新在线时间、状态、诊断；
5. 推进发现/配置事务状态机；
6. 处理 10 ms 控制边界到期事件；
7. 按 P0/P1/P2 调度三路 TX；
8. 更新实时统计。

MUST NOT 在 1 ms 路径中：

- 等待 ROS；
- 等待 micro-ROS Agent；
- 轮询等待设备响应；
- 等待 W25Q64；
- 执行阻塞式 LED 刷新；
- 动态分配；
- 无限制地 drain RX 队列；
- 无限制地处理 P2 管理事务。

每周期具体最大 RX 帧数、P2 步数、TX pump 数量为 **TBD**，但 MUST 在实现中以常量或可审查配置体现。

### 11.3 10 ms 控制边界

每 10 ms MUST：

1. 读取/复制最新完整 `MotorCommandArray` 快照；
2. 校验 `sequence`；
3. 校验 `count`；
4. 校验 ID 合法性；
5. 检查重复 `motor_id`；
6. 检查浮点值合法性；
7. 检查 Profile/生命周期；
8. 检查策略与 Enable；
9. 检查安全授权；
10. 对本周期未出现但仍属于已受控集合的电机执行软停；
11. 对合格 `ACTIVE` 电机生成厂家 MIT P1 帧；
12. 将 P1 目标写入 latest-wins 槽。

### 11.4 100 Hz 的准确语义

- 100 Hz 是**控制快照处理频率**。
- `MotorMotionStateArray` 可目标 100 Hz 发布。
- 100 Hz 状态发布 MUST NOT 被解释为每个电机都有 100 Hz 新 CAN 测量。
- 若某电机最近状态未更新，发布端可继续输出最近有效值，但 MUST 正确更新 `state_age_ms`。
- 上层控制器可基于 `state_age_ms` 判断测量新鲜度。

---

## 12. 时间基准

系统存在两类时间：

### 12.1 ROS 同步时间

用于：

- ROS 消息时间戳；
- 跨系统日志/数据对齐。

MUST NOT 用于：

- 命令失效判断；
- 安全超时；
- OFFLINE 判定；
- CAN 调度 deadline。

### 12.2 MCU 单调时基

用于：

- `last_rx_tick`；
- 命令接收 freshness；
- 30 ms / 100 ms 安全超时；
- 状态年龄；
- 配置阶段超时；
- 发现/离线定时；
- 调度统计。

**DERIVED**：`MotorCommandArray` 到达 MCU 时，bridge/mailbox MUST 记录本地接收 tick。安全 freshness 以该 tick 为基础，而不是以 ROS Header 时间戳为基础。

---

## 13. 命令快照与安全超时

### 13.1 完整快照

`MotorCommandArray` MUST 是完整快照语义，而不是增量命令。

每个周期：

- 快照中出现的电机：使用当前新目标；
- 快照中缺失、但仍处于“已受控集合”的电机：MUST 软停；
- 禁止默默沿用上一周期该电机的运动目标。

**DERIVED**：“已受控集合”至少包含当前具有有效 Policy/Enable/运动授权、且上层此前纳入控制的设备；其精确实现必须保证“删除一项命令不会导致旧目标继续生效”。

### 13.2 freshness 阈值

- 新完整命令快照超过 **30 ms** 未更新：进入软停。
- 超过 **100 ms** 未更新：协议级停机/失能，并清除运动授权。
- micro-ROS 会话断开：不等待 100 ms 到期，进入协议级停机/失能并清除运动授权。

### 13.3 恢复条件

发生 100 ms 超时或会话断开后，恢复 MUST 同时要求：

1. micro-ROS 会话恢复；
2. 有效策略重新建立；
3. 显式 Enable；
4. 新的完整运动快照；
5. 生命周期和安全检查通过。

MUST NOT 自动复用：

- 断开前的 Enable；
- 断开前的运动授权；
- 断开前的运动目标。

### 13.4 软停语义

“软停”厂家级动作尚未由输入材料定义。

实现前 MUST 为 Vaxor/CREB 分别明确：

- 是保持当前位置、速度归零、力矩归零还是专用 stop；
- `Kp/Kd` 如何处理；
- 软停是否保持 Enable；
- 到 100 ms 硬停时如何升级；
- 软停动作发送频率；
- 设备反馈确认方式。

该项为 **TBD-SAFETY-01**，属于安全验收阻塞项。

---

## 14. ROS 2 / micro-ROS 边界

### 14.1 上层标准接口

上层控制器 MUST 使用 ros2_control 官方 command interfaces：

- `position`
- `velocity`
- `effort`

`sensor_msgs/msg/JointState`：

- MAY 由上层发布；
- MUST NOT 作为 MCU 运动控制命令。

### 14.2 MCU 自定义接口总体原则

自定义消息 MUST：

- 固定容量；
- 不使用无界动态数组进入实时路径；
- 使用 `motor_id`，不使用 `joint_name`；
- 使用 SI 物理单位；
- 不携带原始 CAN payload；
- 不携带厂家协议字节布局；
- 将高频运动状态与低频诊断/拓扑分开。

---

## 15. `MotorCommandArray`

### 15.1 方向

上层 → MCU

### 15.2 语义

100 Hz 完整运动命令快照。

### 15.3 最小字段语义

数组级：

- 同步时间戳；
- `uint32 sequence`；
- `uint8 count`；
- 固定容量最多 64 项。

元素级：

- `uint16 motor_id`
- `float32 position`，单位 rad；
- `float32 velocity`，单位 rad/s；
- `float32 effort`，单位 N·m。

### 15.4 禁止字段

MUST NOT 包含：

- `joint_name`；
- 原始 CAN ID；
- DLC；
- CAN data bytes；
- MIT 原始编码值；
- Kp/Kd；
- Enable；
- 厂家管理对象。

### 15.5 校验

bridge 或 10 ms 边界 MUST 拒绝/安全处理：

- `count > 64`；
- 重复 `motor_id`；
- 不存在/未识别/冲突设备；
- 非法数值；
- 不满足 Profile 限制的值；
- 不满足 Policy 限幅的值；
- 不允许运动的生命周期。

精确的 NaN/Inf 命令处理策略应固化为消息校验规则。建议命令输入中禁止 NaN/Inf；最终规则 **TBD-ROS-01**。

---

## 16. `MotorMotionStateArray`

### 16.1 方向

MCU → 上层

### 16.2 目标频率

目标 100 Hz。

### 16.3 最小元素语义

- `motor_id`
- position，rad
- velocity，rad/s
- effort，N·m
- `state_age_ms`

### 16.4 状态缺失

缺失的浮点量 MUST 使用 `NaN`。

**DERIVED**：`state_age_ms` 必须来源于 MCU 单调时基，并表示“最近一次对应有效运动状态测量距当前发布时间的年龄”，而不是 ROS 消息周期。

---

## 17. `MotorStatusArray`

### 17.1 方向

MCU → 上层

### 17.2 语义

诊断/保护/设备状态，不与高频运动目标耦合。

### 17.3 最小语义字段

- `motor_id`
- temperature，°C
- current，A
- voltage，V
- drive mode
- unified fault code
- vendor fault code
- `protection_flags`
- `limit_flags`
- `online`
- lifecycle state
- `state_age_ms`

未提供的浮点状态使用 `NaN`。

### 17.4 标志位

`protection_flags` 与 `limit_flags` MUST：

- 在单一公共头文件或消息常量位置定义；
- 不在业务模块中重复定义位号；
- 具有稳定的跨协议语义；
- 允许同时保留厂家原始故障码。

公共位定义为 **TBD-STATUS-01**。

---

## 18. `MotorTopology` / `MotorEvent`

### 18.1 用途

低频表达：

- 当前全量设备拓扑；
- 上线；
- 离线；
- 重新发现；
- 身份变化；
- 总线变化；
- 能力变化；
- 冲突；
- 配置相关事件。

### 18.2 最小语义信息

最终字段顺序未定，但拓扑语义至少需要能表达：

- `motor_id`
- `bus_index`
- protocol
- model identity
- capability summary
- lifecycle
- online/conflict state

事件至少需要：

- 事件类型；
- `motor_id`；
- 相关 bus/protocol/model；
- MCU 时间或可对应的时间信息；
- 原因/结果码。

具体 `.msg` 名和字段为 **TBD-ROS-02**。

---

## 19. `MotorPolicy`

### 19.1 方向

上层 → MCU

### 19.2 定位

运行时安全/控制策略，低频更新。

### 19.3 已确认内容

包括：

- Enable；
- MIT `Kp/Kd`；
- 软件限幅；
- 安全授权；
- 原基线中提到的“前馈”。

### 19.4 关键规则

- MUST NOT 放进 100 Hz `MotorCommandArray`。
- Policy 更新 MUST NOT 刷新运动命令 freshness。
- Enable 变化 MUST 经过 runtime 安全状态机。
- 失去运动授权 MUST 使电机离开 `ACTIVE` 或进入安全状态。

### 19.5 前馈语义冲突

100 Hz `MotorCommandArray.effort` 已是动态 N·m 目标。若 Policy 中的“前馈”也是动态 torque feed-forward，则与命令边界冲突。

实现前必须二选一并固化：

A. `MotorCommandArray.effort` 是唯一动态前馈/力矩目标，Policy 不保存动态前馈；  
或  
B. Policy 仅保存低频静态 bias/scale/limit，最终输出由 runtime 合成。

该项为 **TBD-ROS-03**，在消息冻结前必须完成。

---

## 20. `MotorConfig` 事务

### 20.1 方向

上层 → MCU，请求；MCU → 上层，结果。

### 20.2 用途

- 模式切换；
- CREB 限位；
- 其他设备配置。

### 20.3 基本事务规则

每个请求 MUST 携带 `request_id`。

同一 `motor_id`：

- 同时最多存在一个未完成事务；
- 新请求 MUST 立即返回 `BUSY`；
- MUST NOT 排队覆盖前一个请求。

### 20.4 执行顺序

1. MCU 校验消息；
2. 校验型号/Profile 能力；
3. 进入适配器事务状态机；
4. 分阶段发送/等待；
5. 需要确认的写操作执行读回；
6. 读回一致才返回 `OK`；
7. 可重试步骤最多自动重试 2 次；
8. 写结果不明确时先读回，不盲目重复写；
9. 最终无法确认时返回失败；
10. 失败时不自动恢复运动授权。

### 20.5 统一结果码

| 结果码 | 含义 |
|---|---|
| `OK` | 操作完成且已验证 |
| `BUSY` | 同一电机已有未完成事务 |
| `INVALID` | 消息或参数非法 |
| `UNSUPPORTED` | 当前型号/协议不支持 |
| `DEVICE_ERROR` | 驱动器拒绝或协议执行错误 |
| `TIMEOUT` | 阶段超时且重试耗尽 |
| `UNCONFIRMED` | 写入/读回无法确认或不一致 |

### 20.6 非阻塞要求

事务 MUST 作为跨 1 ms 周期推进的状态机实现。

MUST NOT：

- 在 `motor_rt_task` 中等待设备回复；
- busy-wait；
- sleep 等待；
- 阻塞 P0/P1；
- 因配置读取耗时而延迟 10 ms 控制边界。

配置结果消息的最终名称与字段为 **TBD-ROS-04**。

---

## 21. FDCAN 接收架构

### 21.1 每总线独立 RX 环

FDCAN1/2/3 MUST 各自拥有独立静态 RX ring。

初始深度：

- 64 帧/总线。

是否需要增加深度只能通过最坏流量测试决定。

### 21.2 原始帧逻辑结构

至少包含：

```text
can_id
is_fd
is_brs
dlc
data[64]
rx_tick
bus_index
```

### 21.3 ISR 责任

ISR MUST 只执行：

1. 从硬件 FIFO 取帧；
2. 记录 MCU 接收 tick；
3. 写入本总线 RX ring；
4. 更新 ISR/RX 计数；
5. 满环时丢弃并增加 overflow 计数。

ISR MUST NOT：

- 解码 Vaxor/CREB；
- 修改复杂 runtime 状态机；
- 调用 ROS/micro-ROS；
- 发正常控制帧；
- 操作 W25Q64；
- 打印阻塞日志。

### 21.4 RX 溢出

发生 RX ring 溢出时：

- MUST 丢弃而不是阻塞 ISR；
- MUST 计数；
- SHOULD 产生诊断；
- 若达到安全影响阈值，runtime SHOULD 进入安全降级。

“达到何种溢出程度需要触发安全动作”为 **TBD-SAFETY-02**。

---

## 22. FDCAN 发送调度

每条总线 MUST 有独立调度器。

### 22.1 P0：安全

内容：

- 停机；
- 失能；
- 急停；
- 协议安全帧。

规则：

- 最高优先级；
- 可按电机去重；
- MUST 能抢占 P1/P2 的软件调度机会；
- MUST NOT 因 P2 队列积压而延迟。

### 22.2 P1：周期控制

内容：

- 默认 100 Hz MIT 单电机控制。

每台电机 MUST 只有一个 latest-wins 槽：

```text
new target replaces unsent old target
```

MUST NOT 形成 FIFO 历史运动命令队列。

### 22.3 P2：管理

内容：

- SYNC；
- 身份读取；
- 能力读取；
- 配置事务；
- 热插拔探测。

P2 MUST：

- 使用有界队列/状态；
- 受每周期配额限制；
- 不挤占 P0/P1。

具体 P2 queue depth、每周期预算为 **TBD-CAN-01**。

---

## 23. 默认 MIT 发送规则

### 23.1 Vaxor

已确认默认 MIT 单电机帧：

```text
CAN ID: 0x501 + 4*N
Length: 12 bytes
```

具体 N 与 `motor_id` 的映射、FD/BRS 位及字节定义由 Vaxor 适配器/协议文件固化。

### 23.2 CREB

已确认默认 MIT 单电机帧：

```text
CAN ID: 0x110 | Dev_ID
Length: 9 bytes
```

具体 FD/BRS 位及字节定义由 CREB 适配器/协议文件固化。

### 23.3 CREB 多控

`0x200`：

- MUST NOT 因“总线上电机超过 8 台”而自动启用；
- 仅在明确的非 MIT 模式下启用；
- 由 CREB 适配器负责；
- 每组最多 8 台。

### 23.4 CREB SYNC

`0x080 SYNC`：

- 用于发现；
- 重连；
- 热插拔探测；
- 不作为默认 MIT 周期控制帧。

---

## 24. 协议适配器接口

`motor_rt_task` 不应知道厂家字节布局。

每个协议适配器至少应提供逻辑能力：

```text
probe / discover
identify
decode_frame
encode_mit_command
encode_soft_stop
encode_disable
handle_sync
start_config_transaction
step_config_transaction
decode_config_response
normalize_status
map_vendor_fault
```

具体 C/C++ API 可不同，但边界 MUST 保持。

### 24.1 适配器输入

- 原始 CAN frame；
- `MotorModelProfile`；
- 统一控制意图；
- 配置请求；
- MCU tick。

### 24.2 适配器输出

- 身份；
- 能力；
- 标准化状态；
- 标准化故障；
- 待发送原始 CAN frame；
- 事务阶段结果。

### 24.3 新协议扩展

新增第三种厂家协议时 MUST NOT 要求修改：

- `MotorCommandArray`；
- `MotorMotionStateArray` 基本语义；
- `motor_rt_task` 主循环；
- mailbox；
- P0/P1/P2 抽象。

---

## 25. Vaxor/CREB 能力边界

| 能力 | Vaxor | CREB |
|---|---|---|
| 发现/身份 | 20 ms 自主状态 | 反馈/管理读取 |
| 默认控制 | MIT 单电机 | MIT 单电机 |
| 默认反馈 | 自主状态；MIT 可能有响应 | MIT 命令反馈 |
| SYNC | 不使用 | 仅发现/探测 |
| 多控 | 不使用 | 非 MIT 模式可用 `0x200` |
| 限位 | MCU 软件限位 | 驱动器侧配置 + MCU 事务核验 |

---

## 26. 软件限位与设备限位

### 26.1 Vaxor

由于没有硬件限位能力，运行时 MUST 支持固件软件限位。

软件限位：

- 由上层通过 Policy 提供；
- MCU 本地执行；
- MUST 在协议打包前检查；
- 不得依赖 ROS 回调实时执行安全裁决。

### 26.2 CREB

CREB 驱动器侧限位配置：

- 通过 `MotorConfig` 事务；
- 写入后必须读回确认；
- 不能确认时不得返回 `OK`；
- 不能确认时不得自动恢复运动授权。

限位单位统一为 rad。

---

## 27. Mailbox 与跨任务通信

### 27.1 `command_snapshot_mailbox`

必须具备：

- 固定容量；
- 保存“最新完整快照”；
- 保存本地接收 tick；
- 快照替换而不是累积；
- 读取时得到一致副本。

### 27.2 policy/config mailbox

低频请求与高频命令 MUST 分开。实现上都落在 `lib/control_mailbox`，但必须使用
相互独立的存储和 API；mailbox 只负责固定容量的跨任务交换，不负责协议事务或
CAN 发送。

### 27.3 并发约束

具体实现可以是：

- 双缓冲 + 原子索引；
- 短临界区复制；
- 其他有界同步方案。

但 MUST 满足：

- ISR 不参与复杂锁；
- `motor_rt_task` 不执行无界等待；
- 不使用动态分配；
- 不因 micro-ROS callback 长时间持锁而阻塞实时任务。

最终同步机制为实现选择，但必须通过 WCET/压力测试验证。

---

## 28. 模块划分与所有权

| 模块 | MUST 负责 | MUST NOT 负责 |
|---|---|---|
| `lib/fdcan` | 初始化、过滤器、IRQ、RX ring、TX 调度、HAL FIFO 和底层统计 | 协议解码、ROS、策略 |
| `lib/dma` | 共享 DMA 控制器、通道配置和 IRQ 资源管理 | IMU/LED 采样策略和设备业务 |
| `lib/motor_protocol` | Vaxor/CREB 发现、编解码、MIT、停止、SYNC、配置事务 | 注册表所有权、ROS 消息 |
| `lib/motor_control` | 注册表、型号 Profile、生命周期、安全、freshness、状态快照 | ROS API、厂家 HAL 字节 |
| `lib/control_mailbox` | 高频命令、低频 Policy/Config 和连接事件的固定容量交换 | 策略、协议校验、设备等待、CAN 发送 |
| `src/motor_rt_task` | 1 ms 服务、10 ms 控制、统一调度 | 阻塞等待、动态分配 |
| `src/motor_ros_bridge` | ROS 消息校验、写 mailbox、发布状态副本 | 访问主 RX ring、直接发正常 CAN |
| `src/storage_task` | W25Q64 低频日志 | 实时控制、配置裁决 |
| `src/led_task` | WS2812 状态显示 | 电机安全与 FDCAN 业务 |
| BMI088 现有任务 | IMU 采集 | 电机实时调度 |

### 28.1 正常 TX 唯一所有权

在迁移期间也 MUST 保证：

> 旧固定电机路径与新 runtime 不得同时向同一正常电机发送运动控制。

CAN 调试桥必须删除或改为只读诊断。

---

## 29. 内存策略

以下对象 MUST 使用编译期有界、静态或预分配内存：

- runtime registry；
- RX rings；
- P1 slots；
- P2 管理队列；
- command mailbox；
- policy mailbox；
- config transaction；
- motion state snapshot；
- status snapshot；
- topology/event buffer。

实时路径 MUST NOT 使用：

- `malloc/free`；
- C++ heap allocation；
- 无界容器增长；
- 运行时字符串拼接；
- 依赖不确定堆分配的日志。

---

## 30. DMA / MPU / Cache

### 30.1 FDCAN

正常 FDCAN RX/TX MUST 先使用中断 + 静态 ring，不依赖 DMA。

### 30.2 WS2812

WS2812 是低优先级显示。

若测量证明阻塞式发送影响 FDCAN 延迟：

- SHOULD 改用 SPI TX DMA；
- DMA buffer 必须采用明确的 MPU/cache 策略。

### 30.3 MPU/cache

DMA 缓冲必须：

- 放入明确的 non-cacheable 区域；或
- 在收发边界执行正确 cache maintenance。

MPU 规则 SHOULD 封装在平台/驱动层，不扩散到电机协议业务代码。

### 30.4 BMI088

BMI088 不进入 `motor_rt_task`。

只有测量证明需要时才引入 DMA 优化。

---

## 31. W25Q64

W25Q64 MUST 由独立 `storage_task` 使用。

允许保存：

- 低频事件；
- 故障日志；
- 诊断日志。

MUST NOT 保存为运行时权威源的内容：

- 当前运动命令；
- 当前实时状态；
- `MotorModelProfile`；
- `motor_id → joint_name`；
- 当前运动授权。

任何 flash 写入 MUST NOT 阻塞 1 ms/10 ms 电机路径。

---

## 32. 错误与诊断

系统 SHOULD 至少维护以下统计：

### 32.1 每总线

- RX hardware count；
- RX ring overflow；
- unknown CAN ID；
- decode error；
- TX submitted；
- TX hardware reject/busy；
- P0 sent；
- P1 replaced；
- P1 sent；
- P2 queued；
- P2 dropped/rejected；
- bus error；
- bus-off/recovery 计数。

### 32.2 每电机

- last RX tick；
- state age；
- last command sequence；
- command age；
- protocol error；
- config transaction status；
- unified fault；
- vendor fault；
- lifecycle；
- last failure reason。

### 32.3 实时调度

建议记录：

- `motor_rt_task` 周期；
- 最大执行时间；
- 10 ms 边界延迟；
- P0 最大排队延迟；
- P1 目标替换计数；
- RX backlog high-water mark。

统计字段的具体布局可调整，但台架验收所需指标 MUST 可获取。

---

## 33. micro-ROS 会话状态

`motor_ros_bridge` / micro-ROS 层 MUST 向 runtime 提供可用的会话状态事实。

runtime MUST：

- 检测会话断开；
- 触发协议级停止/失能；
- 清除运动授权；
- 不依赖 ROS 主机继续发送 stop。

会话恢复只代表通信恢复，不代表运动授权恢复。

---

## 34. QoS 原则

具体 QoS 为 **TBD-ROS-05**。

在最终冻结时必须分别定义：

- `MotorCommandArray`
- `MotorMotionStateArray`
- `MotorStatusArray`
- `MotorTopology`
- `MotorEvent`
- `MotorPolicy`
- `MotorConfig`
- 配置结果接口

的：

- reliability；
- durability；
- history；
- depth；
- deadline/liveliness（若使用）。

QoS 选择必须符合“最新运动命令优先、不得积压历史命令”的架构目标。

---

## 35. CAN 过滤器

过滤器具体配置为 **TBD-CAN-02**。

要求：

1. 尽量在硬件层过滤已知相关 ID；
2. 不能因过滤器过窄而破坏发现/热插拔；
3. 发现窗口与稳定运行是否使用不同过滤策略，可在实现中确定；
4. 过滤器变更不能产生不可控长阻塞；
5. 任何未知帧路径也必须有界。

---

## 36. 离线判定

Vaxor/CREB 的 OFFLINE 阈值当前未定义，为 **TBD-SAFETY-03**。

阈值设计必须考虑：

- Vaxor 20 ms 自主状态周期；
- CREB 反馈机制；
- CAN 总线拥塞；
- 临时丢帧；
- 100 Hz 控制；
- 热插拔恢复。

达到 OFFLINE 后 MUST：

- 禁止运动；
- 清除或冻结不可复用的控制授权；
- 发布 topology/event；
- 重新出现时经过 DISCOVERED/IDENTIFYING 流程。

---

## 37. 安全优先级

安全动作优先级 MUST 高于：

- 正常运动；
- 发现；
- 身份读取；
- 配置；
- 日志；
- LED；
- 存储。

任何模块遇到错误时不得通过直接发送厂家 CAN 帧绕过 P0 调度器。

---

## 38. 数据单位与类型

### 38.1 统一单位

- position：rad
- velocity：rad/s
- effort/torque：N·m
- current：A
- voltage：V
- temperature：°C
- limit：rad

### 38.2 类型

- 高频物理量：`float32`
- 上层 `JointState`：转换为 `float64`
- `motor_id`：`uint16`
- raw CAN ID：独立 `uint32`
- command sequence：`uint32`
- count：`uint8`

### 38.3 缺失状态

状态侧缺失浮点值使用 `NaN`。

命令侧 NaN/Inf 规则必须在消息实现规格中明确，见 `TBD-ROS-01`。

---

## 39. 上层 Hardware Interface 职责

上层 MUST 维护：

- `motor_id → joint_name`
- joint direction
- zero offset
- joint semantic configuration
- 对应型号 Profile 副本/校验数据

上层 MUST 将：

```text
joint command
   -> motor_id + SI motor command
   -> MotorCommandArray
```

并将：

```text
MotorMotionStateArray
   -> motor_id lookup
   -> joint position/velocity/effort
   -> sensor_msgs/msg/JointState
```

MCU 不应因为 joint 名称变化而需要重新编译。

---

## 40. Profile 一致性

固件与上层各自保存 Profile 副本，因此需要版本/一致性管理。

输入材料未定义 Profile 版本字段。

**DERIVED/TBD-PROFILE-01**：至少应存在一种可检测“不一致”的机制，例如型号/版本标识、配置版本或构建时校验。具体机制待实现确认。

在一致性无法确认且可能导致限位/单位换算错误时，不应允许进入 `ACTIVE`。

---

## 41. 实时预算要求

输入材料尚未给出具体 WCET。

实现必须建立以下预算并实测：

- 1 ms 周期最大执行时间；
- 10 ms 控制边界最大执行时间；
- 每周期最大 RX 解码数；
- 每周期最大 P2 推进数；
- P0 从生成到提交硬件 FIFO 的最大延迟；
- P1 从 10 ms 边界到提交硬件 FIFO 的最大延迟；
- 三路同时高负载时的最坏抖动。

具体验收数值为 **TBD-RT-01**，但原则上 MUST 留出可证明的调度余量，不能以“平均不超时”作为通过标准。

---

## 42. 总线容量验证

最终 MUST 在最大规模和最坏反馈流量下测试：

- 单总线；
- 双总线；
- 三总线；
- 约 40 台设备；
- Vaxor/CREB 混合；
- 最大可预期状态反馈；
- P2 同时存在发现/配置；
- 命令 100 Hz。

必须观测：

- 总线占用；
- RX overflow；
- TX 延迟；
- 控制抖动；
- P1 替换；
- P0 延迟；
- 状态年龄；
- CPU 负载。

测试失败时，允许调整：

- RX ring 深度；
- P2 容量；
- 每周期配额；
- TX pump 策略；

但 MUST NOT 为了通过负载测试而破坏以下边界：

- ROS 消息直接发 CAN；
- 引入历史运动 FIFO；
- 多个模块并行拥有正常 TX；
- 在 ISR 解码协议；
- 在实时路径动态分配。

---

## 43. 单元测试要求

### 43.1 runtime

至少覆盖：

- DISCOVERED → IDENTIFYING → READY_DISABLED；
- READY_DISABLED → ACTIVE；
- 30 ms freshness；
- 100 ms freshness；
- session disconnect；
- 重新 Enable；
- 缺席电机软停；
- 重复 ID；
- ID collision；
- profile conflict；
- fault；
- offline/reconnect。

### 43.2 mailbox

至少覆盖：

- 最新快照覆盖；
- 一致复制；
- sequence；
- count 边界；
- 64 项满容量；
- 并发写/读压力；
- 不产生动态分配。

### 43.3 scheduler

至少覆盖：

- P0 > P1 > P2；
- P1 latest-wins；
- P0 去重；
- P2 有界；
- 多总线独立；
- TX busy/retry 行为；
- 无历史运动积压。

### 43.4 protocol

Vaxor/CREB 各自覆盖：

- identity；
- state decode；
- MIT encode；
- stop/disable；
- fault mapping；
- config transaction；
- timeout；
- retry；
- readback confirm。

---

## 44. 集成与台架测试矩阵

| 阶段 | 设备规模 | 重点 |
|---|---:|---|
| T1 | 无电机/模拟帧 | runtime、消息、调度、静态内存 |
| T2 | 单协议单电机 | 发现、MIT、状态、安全停 |
| T3 | 单总线多电机 | P1 latest-wins、RX/TX 压力 |
| T4 | 单总线 Vaxor+CREB | 协议共存 |
| T5 | 双总线 | 并行服务、公平性 |
| T6 | 三总线 | 调度和 CPU 峰值 |
| T7 | 约 40 台 | 最大规模稳定性 |
| T8 | 约 40 台 + P2 事务 | 最坏管理流量 |
| T9 | 热插拔 | topology/event、重发现 |
| T10 | Agent 断开/恢复 | 100 ms 安全和重新授权 |
| T11 | RX overflow 注入 | 诊断与安全降级 |
| T12 | bus error/bus-off | 故障处理与恢复 |

---

## 45. 验收准则

ADR 从 `proposed` 转为 `accepted` 前，至少要求：

### 45.1 架构

- [ ] MCU 无固定按电机实例安装表。
- [ ] 上层独占 joint 映射职责。
- [ ] 协议字节只在适配器/Profile 层。
- [ ] 只有一个正常电机 TX 业务所有者。
- [ ] ISR 不做协议解码/ROS。
- [ ] 实时路径无动态分配。
- [ ] P1 为 latest-wins，不是 FIFO。

### 45.2 安全

- [ ] 30 ms 软停已对两种协议定义并验证。
- [ ] 100 ms 停机/失能已验证。
- [ ] Agent 断开时本地安全动作已验证。
- [ ] 恢复必须重新 Enable + 新完整快照。
- [ ] ID/Profile 冲突禁止运动。
- [ ] OFFLINE 阈值已固化并验证。

### 45.3 接口

- [ ] `.msg` 名称、字段顺序、容量已冻结。
- [ ] QoS 已冻结。
- [ ] Policy 前馈与 command effort 语义冲突已解决。
- [ ] protection/limit 位常量已统一。
- [ ] Config result 接口已冻结。
- [ ] `request_id` 类型与回绕策略已定义。

### 45.4 总线

- [ ] 每协议 CAN/FD/BRS 属性已固化。
- [ ] 过滤器已固化。
- [ ] P2 配额已固化。
- [ ] 最大规模无不可接受 RX overflow。
- [ ] P0/P1 延迟满足实测预算。
- [ ] 三总线最坏负载已记录。

### 45.5 实时

- [ ] 1 ms 任务 WCET 已测。
- [ ] 10 ms 控制 jitter 已测。
- [ ] 无等待 W25Q64/Agent/设备响应。
- [ ] 约 40 台规模压力测试通过。

---

## 46. 实施顺序

### Phase 1：接口和静态数据冻结

1. 定义消息包；
2. 定义枚举；
3. 定义 lifecycle；
4. 定义 fault/flags；
5. 定义 `MotorModelProfile`；
6. 定义静态容量；
7. 解决 `MotorPolicy` 前馈语义。

### Phase 2：FDCAN seam

1. 三路 RX rings；
2. ISR ownership；
3. raw frame；
4. TX scheduler；
5. P0/P1/P2；
6. stats；
7. 删除直接业务 TX。

### Phase 3：协议层

1. Vaxor identity/state/MIT；
2. CREB identity/state/MIT；
3. stop/disable；
4. discovery/SYNC；
5. config transaction；
6. fault mapping。

### Phase 4：runtime

1. registry；
2. lifecycle；
3. safety；
4. 1 ms service；
5. 10 ms boundary；
6. latest-wins；
7. session safety。

### Phase 5：micro-ROS

1. command mailbox；
2. policy/config mailbox；
3. state snapshot；
4. topology/event；
5. QoS；
6. Agent session state。

### Phase 6：外围隔离

1. storage_task；
2. LED non-blocking；
3. cache/MPU；
4. BMI088 与 runtime 隔离确认。

### Phase 7：验证

1. 主机单元测试；
2. 单电机；
3. 单总线；
4. 双总线；
5. 三总线；
6. 约 40 台；
7. 热插拔；
8. 故障/断链；
9. 最坏反馈 + 配置流量。

---

## 47. 必须闭环的 TBD 清单

| ID | 项目 | 阻塞阶段 |
|---|---|---|
| TBD-SAFETY-01 | Vaxor/CREB 30 ms 软停厂家级语义 | 安全验收 |
| TBD-SAFETY-02 | RX overflow 触发安全降级的阈值/策略 | 台架验收 |
| TBD-SAFETY-03 | Vaxor/CREB OFFLINE 判定阈值 | 热插拔/安全 |
| TBD-ROS-01 | 命令 NaN/Inf 校验规则 | 消息冻结 |
| TBD-ROS-02 | Topology/Event 最终 `.msg` 和字段 | micro-ROS 接入 |
| TBD-ROS-03 | Policy“前馈”与 Command `effort` 语义 | 消息冻结 |
| TBD-ROS-04 | Config result 最终接口名/字段 | 配置事务 |
| TBD-ROS-05 | 各 Topic QoS | micro-ROS 接入 |
| TBD-STATUS-01 | protection/limit 公共位定义 | 状态接口 |
| TBD-CAN-01 | P2 队列深度、每周期 RX/P2/TX 配额 | 实时集成 |
| TBD-CAN-02 | FDCAN 过滤器 | 总线集成 |
| TBD-CAN-03 | CREB/Vaxor 各帧 FD/BRS/具体 DLC 属性确认 | 协议集成 |
| TBD-RT-01 | WCET、P0/P1 延迟、jitter 预算 | 台架验收 |
| TBD-PROFILE-01 | 固件/上层 Profile 一致性检测方式 | 系统集成 |
| TBD-CONFIG-01 | 厂家管理对象、阶段超时值 | 配置事务 |
| TBD-CONFIG-02 | `request_id` 类型、回绕和去重规则 | 配置事务 |

---

## 48. 禁止性清单

以下实现即使“能跑”，也视为违反本 ARD：

1. 在 MCU 固定表中写死 40 台电机与关节名。
2. ROS callback 直接调用 HAL 发送运动 CAN。
3. ISR 中解码 Vaxor/CREB。
4. 每个 motor task 各自拥有一个 FDCAN TX 路径。
5. 用 FIFO 堆积 100 Hz 运动命令。
6. 在 1 ms task 内等待配置响应。
7. 用 ROS 时间判断 30/100 ms 安全超时。
8. Agent 断开后继续沿用旧 Enable/旧目标。
9. 配置写入未读回确认就返回 `OK`。
10. 为了调试保留可直接注入正常运动 CAN 的 bridge。
11. 在实时路径 `malloc`。
12. W25Q64 写日志阻塞 `motor_rt_task`。
13. 因 CREB 数量 > 8 自动切换 `0x200` 多控。
14. 用 `0x080 SYNC` 替代默认 MIT 控制。
15. Vaxor 80 速比参数未确认就套用 30/50 Profile。
16. 同 ID 冲突时“最后发现者覆盖前者”。
17. 将 100 Hz 状态发布等同于 100 Hz 新传感数据。

---

## 49. 代码评审检查表

每个涉及电机 runtime 的 PR 至少检查：

- [ ] 是否改变 TX 所有权？
- [ ] 是否引入动态分配？
- [ ] 是否在 ISR 做了更多业务逻辑？
- [ ] 是否增加阻塞等待？
- [ ] 是否可能让 P2 影响 P0/P1？
- [ ] 是否破坏 latest-wins？
- [ ] 是否让旧命令继续生效？
- [ ] 是否使用 ROS 时间做安全判断？
- [ ] 是否把厂家字节泄漏进 ROS 消息？
- [ ] 是否把 joint 语义放进 MCU？
- [ ] 是否新增未有界队列？
- [ ] 是否需要新 Profile？
- [ ] 是否需要新协议 adapter？
- [ ] 是否增加新的 fault/flag 位？
- [ ] 是否有对应单测？
- [ ] 是否影响 1 ms/10 ms WCET？

若答案涉及架构边界变化，应新建 ADR，而不是只在代码中修改。

---

## 50. 建议的代码目录

以下是目标逻辑组织。现有旧库在迁移完成前保留，但不得继续增加职责：

```text
micro_ros_firmware/
├── lib/
│   ├── fdcan/
│   │   ├── can_frame.*
│   │   ├── can_rx_ring.*
│   │   ├── can_tx_scheduler.*
│   │   ├── can_stats.*
│   │   └── fdcan_bsp.*
│   ├── dma/
│   │   └── dma.*
│   ├── control_mailbox/
│   │   ├── command_snapshot.*
│   │   ├── config_request.*
│   │   └── control_events.*
│   ├── motor_control/
│   │   ├── motor_registry.*
│   │   ├── motor_lifecycle.*
│   │   ├── motor_safety.*
│   │   ├── motor_profile.*
│   │   └── motor_snapshot.*
│   ├── motor_protocol/
│       ├── motor_protocol.*
│       ├── vaxor_adapter.*
│       └── creb_adapter.*
│   ├── imu/
│   ├── w25q64/
│   └── ws2812/
├── src/
│   ├── motor_rt_task.*
│   ├── motor_ros_bridge.*
│   ├── storage_task.*
│   └── led_task.*
└── msg-or-interfaces/
    └── TBD
```

目录名不是规范性要求；模块责任边界是规范性要求。

---

## 51. 关键伪代码

### 51.1 FDCAN ISR

```c
void FDCANx_RxISR(void)
{
    while (hw_fifo_has_frame() && isr_budget_available()) {
        RawCanFrame f = hw_pop_frame();
        f.rx_tick = monotonic_tick();
        f.bus_index = BUS_X;

        if (!rx_ring_try_push(&bus_x_rx_ring, &f)) {
            stats.rx_ring_overflow++;
        }
    }
}
```

### 51.2 1 ms runtime

```c
void motor_rt_task_1ms(void)
{
    for_each_bus_with_budget() {
        while (rx_budget_available() && rx_ring_try_pop(&frame)) {
            motor_runtime_on_raw_frame(&frame);
        }
    }

    motor_runtime_step_transactions_with_budget();

    if (control_boundary_10ms_due()) {
        motor_runtime_apply_latest_command_snapshot();
    }

    for_each_bus() {
        tx_scheduler_pump_P0();
        tx_scheduler_pump_P1();
        tx_scheduler_pump_P2_with_budget();
    }

    publish_or_swap_state_snapshots_if_due();
}
```

### 51.3 10 ms 控制

```c
void motor_runtime_apply_latest_command_snapshot(void)
{
    Snapshot s = mailbox_copy_latest_consistent();
    tick_t now = monotonic_tick();

    if (!session_alive()) {
        disable_all_motion_authority();
        schedule_protocol_disable_P0();
        return;
    }

    if (age(now, s.local_rx_tick) > 100_ms) {
        disable_all_motion_authority();
        schedule_protocol_disable_P0();
        return;
    }

    if (age(now, s.local_rx_tick) > 30_ms) {
        schedule_soft_stop_P0_or_safety_path();
        return;
    }

    validate_snapshot(s);

    for_each_controlled_motor(m) {
        if (!snapshot_contains(s, m.motor_id)) {
            schedule_soft_stop(m);
            continue;
        }

        if (!motor_is_ACTIVE_and_safe(m)) {
            continue;
        }

        UnifiedMotorCommand cmd = get_command(s, m.motor_id);
        cmd = apply_policy_limits(m, cmd);
        RawCanFrame f = adapter_encode_mit(m, cmd);
        p1_latest_replace(m.bus_index, m.motor_id, f);
    }
}
```

伪代码中的具体函数名仅用于表达架构，不是 API 冻结。

---

## 52. 需求追踪矩阵

| 需求 | 设计位置 | 验证 |
|---|---|---|
| 动态拓扑 | registry + adapters + topology | 热插拔测试 |
| 100 Hz 控制 | 10 ms boundary | jitter/周期测试 |
| 1 ms 服务 | motor_rt_task | WCET |
| 30 ms 软停 | safety | 断命令测试 |
| 100 ms 失能 | safety + P0 | 断命令/Agent 测试 |
| 三路独立 CAN | per-bus ring/scheduler | 三总线压力 |
| 不积压旧目标 | P1 latest-wins | scheduler 单测 |
| 配置可确认 | config transaction + readback | 事务测试 |
| 无动态分配 | static capacity | 静态检查/运行监测 |
| ROS/协议解耦 | bridge + adapter seam | 代码评审 |
| joint 语义上移 | upper HW interface | 配置测试 |
| 约 40 台规模 | capacity 64 + bench | 最大规模台架 |

---

## 53. 变更控制

以下变化通常需要新 ADR 或修订 ADR 0007：

- 从“发现驱动”改回固定安装表；
- 改变正常 CAN TX 单一所有权；
- 使用每电机独立实时任务；
- ROS 消息直接变成厂家 CAN；
- 改变 1 ms/10 ms 双时基架构；
- 改变 30/100 ms 安全总原则；
- 用队列积压历史运动命令；
- 将 joint 语义下沉 MCU；
- 在实时路径允许动态分配；
- CREB/Vaxor 默认控制不再以单电机 MIT 为主；
- 将厂家协议细节上移到 ROS 接口。

以下通常只需实现规格变更：

- ring depth；
- P2 quota；
- hardware filters；
- QoS；
- 具体阶段 timeout；
- flags 位号；
- `.msg` 字段顺序；
- 统计字段；
- 经过验证的 Profile 参数补充。

---

## 54. 最终架构摘要

该系统的核心不是“给 40 台电机分别写驱动”，而是：

> 在 STM32H723 上建立一个发现驱动、静态有界、单一实时所有权的电机运行时。  
> 上层只表达统一的 joint/motor 控制语义；MCU 负责身份、生命周期、安全和调度；  
> Vaxor/CREB 适配器独占厂家协议字节；三路 FDCAN 独立缓冲和调度；  
> 100 Hz 命令以完整快照进入，P1 只保留最新目标；  
> 30 ms 无新命令软停，100 ms 或 Agent 断开协议级停机/失能；  
> 配置走低频、可验证、非阻塞事务；实时路径禁止动态分配；  
> 最终以约 40 台设备的三总线最坏流量台架测试作为架构 accepted 的依据。

---

## 55. 当前状态

本 ARD 可作为重构开发基线，但在以下关键 TBD 闭环前仍保持 **Proposed**：

1. 两协议软停的精确定义；
2. OFFLINE 判定；
3. Policy 前馈语义；
4. 消息 `.msg` / QoS；
5. 配置结果接口；
6. CAN FD/BRS/过滤器细节；
7. 管理事务对象和 timeout；
8. RX/P2/TX 预算；
9. WCET/jitter/P0/P1 延迟验收值；
10. 公共 fault/limit/protection 常量；
11. Profile 一致性检测；
12. 最大规模三总线台架数据。

完成上述项目并通过最大规模测试后，ADR 0007 才具备从 `proposed` 转为 `accepted` 的证据基础。
