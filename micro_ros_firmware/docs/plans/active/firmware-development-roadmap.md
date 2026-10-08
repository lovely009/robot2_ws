# 固件开发路线图（顺序与门槛）

**状态：活动（2026-10-08 立）。** 本文档回答三件事：**接着按什么顺序做**、
**什么条件下才允许动架构**、**什么明确不做**。

> **本文档不承载开放项。** 逐条验证项（`TBD-*` 编号、判定状态、台架记录要求）
> 的唯一清单是 [motor-runtime-validation](motor-runtime-validation.md)（下称 validation plan）。
> 本文**不新增 `TBD-*` 编号、不复制条目文本**，引用只给编号 + 链接。

**三条硬规则**（写在这里，免得半年后被绕过）：

1. 门禁必须写成**可执行断言**——「某话题字段出现 / 某计数为零 / 某 CTest 通过」，
   不写「完成 XXX 验证」。
2. 架构决策写 [ADR](../../adr/0007-micro-ros-discovery-driven-motor-runtime.md)，
   结构基线写 [module-boundaries](../../architecture/module-boundaries.md)，
   设计理由写 [decision-log](../../architecture/motor-runtime-decision-log.md)。
   本文只写「**什么时候**去动它们」。
3. 阶段完成时本文只改两类内容：门禁状态（+ 日期 + 提交号）与阶段覆盖计数；
   其余文字不动。

## 0. 定位与来源

### 0.1 与既有文档的分工（互不复制）

| 文档 | 唯一职责 | 判据 |
|---|---|---|
| [validation plan](motor-runtime-validation.md) | 开放项的**唯一清单**与逐条证据 | 「这条什么时候验、验出什么算过」 |
| [ADR 0007](../../adr/0007-micro-ros-discovery-driven-motor-runtime.md)（`proposed`） | 架构决策 | 「为什么这么分」 |
| [module-boundaries](../../architecture/module-boundaries.md) | 结构、所有权、依赖方向、测试边界 | 「现在是什么结构」 |
| [decision-log](../../architecture/motor-runtime-decision-log.md) | 设计理由与变更史 | 「当时为什么这么定」 |
| **本文档** | **顺序与门槛** | 「先还哪一笔、什么时候才轮到动架构」 |

> **反面教材（写下来是为了别再犯）**：如果本文的阶段 1 把
> 「TBD-RT-01：实测 `motor_rt_task` WCET、1 ms 周期 jitter、P0/P1 最坏发送延迟」
> 原样抄一遍，它立刻变成第二份开放项清单，两份从此各自漂移。
> 本仓库已经出过一次同类事故——`PARAM_BRIDGE_MAX_PARAMS` 在 ADR 与代码之间漂移，
> decision-log 2026-09-20 条专门记了「已经漂移过一次，不在活注释里复制第二份」。

### 0.2 来源：对标一份外部调研，而那份调研描述的不是本工程

2026-10-08 取得一份调研《GitHub 开源人形机器人运动控制板及核心系统架构调研》。
它提到的这些东西，在本仓库固件里**不存在**（已 grep 全仓库确认）：

| 调研提到 | 本工程实际 |
|---|---|
| `HostLink → CommandRouter → MotionRuntime → ActuatorManager` 主链路 | `ros_bridge` / `motor_bridge` / `param_bridge` → `control_mailbox` → `motor_control` → `motor_tx_router` → `lib/fdcan` |
| USART10 串口舵机链路、`ServoBusTask` | **无串口舵机**；只有 3 路 FDCAN + SPI(BMI088) + OCTOSPI(W25Q64) |
| `DebugShellTask` | 无；`src/syscalls.cpp` 用强符号空 `_write` **禁止** MCU 侧 printf |
| 仿生人头、颈部绳驱、音频外设、USB Composite 的 RPC/Telemetry/Audio 通道 | 无；USB 只承载 micro-ROS 的 CDC |
| RK3588 上位计算机 | 无；上位是 ROS 2 主机 + `micro_ros_agent` |
| 多块 STM32 分布式控制（Gateway + 左腿 + 右腿） | **单颗** STM32H723VGT6 |

吻合的只有：STM32H723VGT6 + FreeRTOS + micro-ROS + USB CDC + CAN FD 电机，
以及轨迹点形状 `{time_from_start_ms, motor_id, position, velocity, effort}`。

**所以调研的骨架（三级分层、micro-ROS 只当通信、MCU 不承担完整动力学）与本工程一致，
但它的模块清单与任务表是按「带舵机的多 MCU 人形」写的。必须逐条裁决，不能照抄**（§3）。

## 1. 现状基线

### 1.1 规模与时间

| 维度 | 数值 |
|---|---|
| 最后一次动代码 | `1d73734`（2026-09-23，TX 路由 / SAVE 事务 / 配置契约重构）。此后只有文档与上位层改动（`b50d678` 2026-10-08） |
| `lib/` | 14 个库，约 12.4k 行（`motor_control.c` 1579、`motor_control_config.c` 1390、`cerb_adapter.c` 1093） |
| `src/` | 14 个文件，约 4.3k 行（`param_bridge.cpp` 881、`motor_bridge.cpp` 857、`motor_rt_task.cpp` 687） |
| host 测试 | 22 个 CTest，`test/host/` **直接编译固件真实 `.c`、无桩无复制**；`test_motor_control.cpp` 单文件 5931 行 / 128 用例 |
| 已实测通过（2026-09-23） | host 22/22 绿、`pio run` 成功、`format_firmware.sh --check` 120 文件干净 |
| RAM_D1 | **310,220 / 327,680 B = 94.7%**，余 **17,460 B** |

> **`micro_ros` 的 10 ms 桥定时器不是独立任务**：`bridge_timer_cb` 由
> `rclc_executor_spin_some`（[ros_bridge.cpp:150](../../../src/ros_bridge.cpp#L150)）
> 在 `micro_ros` 任务（prio 2）的栈上同步调用，所有发布与参数逻辑都跑在那 32 KiB
> 栈上。这也是 validation plan §2 栈审计量到最坏 4,436 B 的原因。

### 1.2 开放项分布（实测，含引用块内条目）

| 节 | 未办 `[ ]` | 部分 `[~]` | 已办 `[x]` |
|---|---:|---:|---:|
| §1 实时性与总线容量 | **7** | 0 | **0** |
| §2 安全语义真机确认 | 12 | 1 | 5 |
| §3 参数链与 Profile | 13 | 1 | 3 |
| §4 上层同步 | 2 | 1 | 6 |
| 近期已结案（不再追踪） | 0 | 0 | 8 |
| **合计** | **34** | **3** | **22** |

**§1 是全仓库唯一整节零证据的段落**，也是唯一能**证伪** ADR 0007 的一组：
如果约 40 台在三总线上跑不动，「发现驱动 + 单任务集中所有权」这个架构本身要重做，
而不是补测试。§2/§3 缺的是「已实现能力的真机确认」，越界只会局部返工。

### 1.3 三条硬约束（每个阶段都要先看）

1. **RAM_D1 余量 17,460 B。** 固件自己点了唯一的直接省 RAM 手段：
   [motor_control.h:43-47](../../../lib/motor_control/include/motor_control.h#L43-L47)
   的注释写着「改小 `MOTOR_CONTROL_MAX_TRAJECTORY_POINTS` /
   `MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS` 这两个常量是唯一的直接省 RAM 手段」。
   §6.0 有具体账。
2. **机构无抱闸，而且软停帧本身就是零位置刚度**（`kp = 0`，见 §3.2）。
   30 ms 与 100 ms 两级都是「撒手」。软件停机不替代硬件急停与机械限位。
3. **`libmicroros.a` 的 meta 槽位是库编译期事实**（publishers 10/12、
   subscriptions 5/6、services 0/5），`-D` 改不动；改 meta 或 `.msg` 只能走
   `../../../../micro_ros_agent/tools/refresh_microros_messages.py`（增量、离线），
   **不是** `-t clean_microros`。

### 1.4 结论：瓶颈是验证债，不是能力债

> **34 条开放项里，约 31 条（91%）属于「能力已实现、只缺真机/HIL 证据」。**

旁证三条：

- **代码已冻结约 15 天**（§1.1）：项目事实上已经停在「往台架转」的门口。
- **能力远快于证据**：12.4k 行库代码 + 4.3k 行任务代码，对着 22 个 host 测试目标、
  128 个内核用例；而开放项是 34 条。已结案的 22 条里绝大多数是「host 覆盖」级别。
- **RAM 余量 17,460 B**（§1.3）意味着任何「真缺能力」的大改动在预算上就先被卡住——
  这件事本身在逼着先还证据债，而不是加功能。

**因此路线图的口径不是「还要做什么功能」，而是「先还哪一笔债、什么时候才允许动架构」。**

## 2. 瓶颈分类

| 类别 | 定义 | 条数 | 明细 |
|---|---|---:|---|
| **(a) 验证债** | 能力已实现，缺真机/HIL 证据 | ~31 | §1 全 7；§2 的 11；§3 的 11；§4 全 2 |
| **(b) 真缺能力，且不依赖新硬件** | 清单内 2 条 + **清单外 3 条** | 5 | 清单内：TBD-SAFETY-02（RX ring overflow 的安全降级**阈值与恢复条件未定义**，不只是未验）、上层型号库 ↔ 固件 Profile 一致性机制（零机制）。<br>**清单外**：following-error 监测、受控保持/第三档软停、软件急停通道——三条**根本没进唯一清单** |
| **(c) 离线可做** | 不需要板子 | — | 上表全部 (b) + 21 个 host 测试目标 + 一致性机制 |
| **(d) 等外部** | 硬件/厂商/规格不到就做不了 | 2 | Vaxor 80 速比型号规格；`cerb-vendor-questions` 问题 1/2/8 |

**(b) 里最该先处理的是「清单外 3 条」**：它们是真缺口，但**没有 `TBD-*` 编号**，
所以永远不会出现在「开放项清零」的视野里。**动作顺序是：先登进 validation plan，
再按 §4 排期**——不是直接写进本文（违反硬规则 1）。

## 3. 对标调研：逐条裁决

| # | 调研主张 | 裁决 | 依据 |
|---|---|---|---|
| 1 | 三级架构；MCU **不**承担完整动力学 | **结构已满足，但第三级是空壳** | 固件侧只有逐台 MIT + 本地插值 + 安全级联，无动力学；[ADR 0008](../../../../docs/adr/0008-robot-dynamics-dependency-boundary.md) 已把 Pinocchio 划为独立边界。**但 `ros2_ws/src/robot_dynamics/README.md:12` 明写「不实现正逆运动学、逆动力学、重力补偿或轨迹前馈」**——所以「已满足」是结构上的，不是能力上的（见 §3.2） |
| 2 | micro-ROS 只当通信接口，回调不得阻塞写电机 | **已满足** | 三条下行订阅回调只做「校验 + `motor_rt_submit_*` 入 mailbox」，不碰 CAN；`motor_rt` prio 3 > `micro_ros` prio 2。两处阻塞调用都在 prio 2 且不在实时路径：`rmw_uros_sync_session(1000)`（[ros_bridge.cpp:139](../../../src/ros_bridge.cpp#L139)，在 `ros_bridge_create()` 内，且排在 `motor_rt_set_agent_connected(true)` **之前**）、`rmw_uros_ping_agent(200,3)`（`LINK_RUN` 态每 10 ms 一次）。**唯一该做的是把这条写进文档，不是改架构** |
| 3 | Input Adapters → CommandRouter → SafetyManager → MotionRuntime → ActuatorManager | **等价物齐全，但两处该拒** | 等价：ingress = `ros_bridge`/`motor_bridge`/`param_bridge`；router = `control_mailbox`（**注意**：只有 `config_queue[16]` 是 FIFO，command / trajectory / policy 都是**深度 2 的 latest-wins 双缓冲**）；Safety = `motor_control` 内联；Motion = `motor_control` 内核；Actuator = adapter 注册表 + `motor_tx_router` + `can_tx_scheduler`。<br>**拒 1（DebugShell / HostLink 作为 Input Adapter）**：ADR §4 明写「CAN 调试桥删除，调试只能读取状态副本、拓扑事件和统计」；module-boundaries §2 明禁第二条 RX 路径。<br>**拒 2（把 SafetyManager 拆成独立层）**：安全裁决**就在内核里**（`motor_control_send_p0` 被撤权/离线/故障/冲突/软停五条路径共用），抽出来就是新增第二个所有者 |
| 4 | 新增 **JointRegistry**：CAN FD 电机与串口舵机统一关节对象 | **该拒（架构反转），缺的是执行器家族** | ADR 0007 §1 明写「上层负责 `motor_id → joint_name`…**MCU 不维护 `joint_name`**」。把关节语义搬进 MCU 是架构反转，且固件侧「统一对象」已由 `micro_ros_up/motor_topology` + `MOTOR_PROTOCOL_CAP_MODE_*` 能力位承担。真正的缺口是**第二个执行器家族**（§6.1） |
| 5 | 新增 **MechanicalConstraintManager**：多关节耦合约束 `f(q) ≤ 0` | **不适用（本机构无耦合关节）；但它指出的问题在本工程真实存在** | 硬件拓扑是 `ros2_ws/src/motor_hardware_interface/config/motor_driver.yaml` 的 **14 台双臂**：`left_arm`/`right_arm` 各 4×`cerb_r58_101` + 3×`vaxor_2550`，关节是肩(yaw/roll/pitch)/肘(wrist…) 的串联链，**无绳驱、无并联、无干涉耦合**。（Gazebo 模型 `robot_arm_gazebo/models/zhoubuguanjie_youbi_001` 只有 `joint_link1..4`，是**仿真资产**，不代表硬件拓扑。）凡需 FK/IK 的约束按 ADR 0007 §1 归上位。**但它担心的「断扭矩不是安全停止」成立**——见 §3.2 |
| 6 | 区分 Trajectory Mode（MCU 收关键帧本地插值）与 Streaming Mode | **不适用（两条都已存在，且更完整）** | Trajectory = `micro_ros_do/motor_trajectory` + 非均匀单调三次 Hermite 本地插值，起点取受理时刻、ROS 时间不参与裁决。Streaming = `micro_ros_do/motor_command_array` 完整快照 + `motor_control_hold_step` 在 2 ms / 500 Hz 边界保持重发——**命令率由本地边界决定，与上行速率解耦**（[motor_control.h:377-387](../../../lib/motor_control/include/motor_control.h#L377-L387) 写明上行送达上限实测 ~72/s）。两条通道互斥仲裁、归属纯派生 |
| 7 | 9 任务表（`SafetyTask` 1 ms 最高 / `MotionTask` / `CANFDTask` / `ServoBusTask` / `SensorTask` 5 ms / `CommandTask` / `HostLinkTask` / `TelemetryTask` / `DebugShellTask`） | **该拒（过度设计且反向）** | 现役 5 任务：`motor_rt`（prio **3**，1 ms 服务 + 2 ms/500 Hz 边界，**已承担安全级联**）、`micro_ros`（2）、`sensor`（2）、`storage`（1）、`led`（1）。<br>`SafetyTask`+`MotionTask`+`CANFDTask` 拆出去 = **新增所有者、新锁、新队列**，收益只是「检测延迟从 ≤1 ms 缩到 ≤0.5 ms」——相对 30 ms 软停阈值与 ~500 ms 驱动器看门狗毫无意义。<br>`SensorTask` 5 ms：现为 prio 2 + ACC DRDY 触发（~100 Hz）+ 失败 1 s 降级重试，**优于**轮询。<br>`CommandTask`：命令已经经订阅回调在 `micro_ros` 任务上投进 mailbox，再加一跳只加延迟。<br>`HostLink`/`Telemetry` 就是 `micro_ros` + `bridge_timer_cb`。<br>`ServoBusTask`：无此总线。`DebugShellTask`：ADR §4 已否决。<br>**额外代价**：每个新任务的 TCB + 栈都落在已满 94.7% 的 RAM_D1 或 120 KiB DTCM 堆上 |

### 3.1 一句话版

调研的**分层骨架**与本工程一致（主张 1/2/3），但它的**模块清单与任务表**（主张 4/5/7）
是按「带舵机的多 MCU 人形」写的：**三条该拒，三条不适用**。主张 6 说的两条模式
本工程两条都有。

### 3.2 调研没点准、但本工程真有的那个问题：软停是「撒手」

调研担心「对承重机构，断开所有扭矩不是安全停止」——**这个判断对本工程成立**，
但成因不是绳驱，而是**软停帧本身没有位置刚度**：

- `cerb_encode_soft_stop` → `cerb_encode_neutral(..., enable=true, kd=default_kd, P0)`
  （[cerb_adapter.c:311-342](../../../lib/motor_protocol/src/cerb_adapter.c#L311-L342)），
  而 `cerb_encode_neutral` 是 `motor_protocol_control_intent_t neutral = {0}` 只填 `kd` 与 `enable`
  —— **`kp = 0`、目标位置 = 0、前馈 = 0**。
- `vaxor_encode_soft_stop` 同形：`damping = {0}` 只填 `kd = default_kd`
  （[vaxor_adapter.c:190-202](../../../lib/motor_protocol/src/vaxor_adapter.c#L190-L202)）。
- 100 ms 档的 `encode_disable` 进一步把使能位也清掉（[motor_protocol.c:195-196](../../../lib/motor_protocol/src/motor_protocol.c#L195-L196)）。

**结论**：30 ms 与 100 ms 两级都是「零刚度 + 保持使能」→「撒手」。
对承重关节，这不是安全停止。

**补充事实：受控保持的能力其实已经存在，只是安全路径到不了它。**
轨迹通道的 `motor_control_trajectory_step` 会钳末点保持、`motor_control_hold_step`
每 2 ms 重发——那是真正的位置保持。缺的不是机制，是**安全级联没有第三档去调用它**。

**再补一条：真正的前馈来源在上位，不在固件。** 前馈字段链路是通的
（`MotorCommandArray.effort` → `motor_control_command_snapshot_t.effort_nm` →
`motor_protocol_control_intent_t.effort_nm`），但 `robot_dynamics` **不实现重力补偿**
（§3 主张 1）；`AGENTS.md` 也记着「示教轨迹只有时间与位置，velocity/effort 恒 0，
所以没有前馈可用，臂的保持完全靠 MIT 的 kp 项」。**所以「不掉臂」的真实瓶颈在
上位补重力前馈，固件侧只负责把第三档留出来。**

## 4. 阶段路线图

排序原则：**先补能证伪架构的证据，再补能保护台架的能力，然后才收语义与参数。**
阶段 0 是所有阶段共同的前置。

### 阶段 0 —— 测量底座（唯一共同前置，约 1 周，离线为主）

**目标**：让「WCET / jitter / 周期 miss」从**测不出来**变成**可测**。
这是阶段 1A 与 1B 的共用前置，也是 validation plan 多条长期空着的一个直接原因：
现有手段只有临时插桩的静态分析（`platformio.ini` 临时加 `-fstack-usage` +
`-fcallgraph-info`，跑 `tools/stack_audit.py`，用完撤掉），且 `AGENTS.md` 明说它
**不含函数指针回调**。

**工作项**：一个 GPIO 翻转探针（进出 `motor_rt_task` 服务拍各翻一次）+ 每拍
`DWT->CYCCNT` 差值的最小统计。不碰公开头、消息、所有权。

**判据**：示波器/逻辑分析仪能看到 1 ms 周期方波；`pio run` 绿；静态占用增幅可忽略。

**风险**：RAM_D1 余 17,460 B —— **必须复用现有结构，不新增大数组**。

### 阶段 1A —— 实时与总线容量取证（需台架）

**目标**：回答两个决定规模的问题——「1 ms 服务 + 2 ms/500 Hz 边界站得住吗」、
「单总线挂几台」。

**覆盖**：validation plan §1 全部 7 条，加 §2 的四条（500 Hz 多轴快控复测、
快照通道 500 Hz 命令率、策略通道真机与代数门、堆高水位）。

**前置**：阶段 0；`dev-real`；无负载；硬件急停就绪。抓包通道二选一——
换 candleLight/socketcan，或**只以固件侧计数为准**并显式标注仪器限制
（台架 SLCAN 在 ~880 帧/s 已证不可信）。

**判据（全部用固件侧计数，不依赖仪器）**：

| 判据 | 建议值 | 状态 |
|---|---|---|
| `motor_rt` WCET | ≤ 1 ms 周期的 40% | ⚠️ 待 owner 拍 |
| 服务拍 jitter | 给出具体上界（非「可接受」） | ⚠️ 待 owner 拍 |
| 4 台 CERB / 单总线 | `tx_frame_count` 差分 ≈ 550 帧/s（500×`0x210` + 50 SYNC），`rx_hw_lost == 0` | 待实测 |
| 30 min SOAK | `tx_dropped == 0`、`event_dropped == 0`、`ring_drop[*] == 0`、`p1_replaced` 稳态 0、`heap_free` 无下降趋势 | 待实测 |
| 单总线 VAXOR 上限 | **必须落成一个数**（从「每 1 ms 16 物理帧 ÷ 每台每 2 ms 1 帧」推得 32 台，需实测确认或推翻），并写进 `README.md` | 待实测 |
| v3 双扇区 save 阻塞 | `system.save` 到 `save_in_progress` 翻 false 的实测时长 + 一次「擦写中掉电」 | 待实测 |

**关键文件**：`src/motor_rt_task.cpp`、`lib/motor_tx_router/src/motor_tx_router.c`、
`lib/fdcan/src/can_bsp.c`、`lib/motor_control/src/motor_control.c`（轨迹/保持热路径）、
`../../../../micro_ros_agent/tools/fdcan_motor_sim.py`、`bench_up.sh`。

**风险**：**本阶段可能得出「500 Hz 在三总线约 40 台上不成立」**——那么 §6 的扩展讨论
要重开。这正是它必须排在语义与参数之前的原因。

### 阶段 1B —— 安全闭环补完（离线，可与 1A 并行）

**目标**：补上「真缺能力」里**零硬件依赖**的三条。全部落 `lib/motor_control`
（纯逻辑库，不依赖 ROS/HAL/FreeRTOS，host 可测）。

**1B.1 跟踪误差监测（following error）—— 本轮第一缺口**

现状：只有「命令**没来**」的判据（快照 30 ms 未更新 → 软停），**没有「命令来了但
位置没跟上」的判据**。推论：驱动器卡死、编码器失效、机械打滑，在固件侧**完全不可见**
——快照一直在刷，固件就一直认为一切正常。
（`AGENTS.md` 已记录过同类盲区：某台不动的唯一线索是 `target_source`，
「ROS 侧完全看不出是哪台被门住」。）

- 落点：内核 service 路径 + 新增 `motor_control_event_t` 枚举值
  （**追加在 `MOTOR_CONTROL_EVENT_TRAJECTORY_REJECTED = 16` 之后**，保持已发布编号不变）
  + 诊断计数 + 阈值来源（`motor_protocol_model_profile_t` 加字段**或**参数面，二选一需先定）。
- 判据：新增 host 用例（某台偏差超阈 N 拍 → 该台出事件 + 软停，**其余电机不受影响**）；
  `ctest` 全绿；`pio run` 绿；`format --check` 绿。
- ⚠️ 待定：阈值单位（关节侧 rad？电机侧 cnt？）、判定窗口拍数。
  **无标定数据时不得给「能用的默认值」**——只能「必须显式配置，未配置则功能关闭」。

**1B.2 受控保持（第三档软停）—— 见 §3.2**

`motor_control_send_p0` 的 kind 枚举加一档（以最近有效测量位置为 setpoint、
用 policy 的 `kp`、有界窗口），窗口到期仍无会话再落回现有 `MOTOR_CONTROL_P0_DISABLE`。

- **这一条改安全边界**（`motor_protocol_adapter.h:376-378` 明写「厂家级语义关闭前
  不得更改安全边界」）→ **必须新 ADR**。
- **设计约束（重要）**：必须做成「三档并列、原两档字节不变」，并把档位选择与帧编码
  解耦——这样 `TBD-SAFETY-01` 的结论回来时（阶段 2）只需改档位选择，不用改帧编码。
- ⚠️ **owner 开关**：是否接受在 `TBD-SAFETY-01` 有结论**之前**先动安全边界？
  不接受则本项顺延到阶段 2 之后（见 §8）。

**1B.3 CERB 过温保护接线**（`protection_flags` bit1 悬空）

温度已解码进 `MotorStatusArray.temperature`（12B 反馈帧 `data[8..9]`，大端 int16、
0.1 °C），但**没有任何阈值来源**（`config/motor.yaml` 型号库无温度项）。
落点：型号库加阈值 → `kProfiles[]` 加字段 → 内核判定 → **复用既有故障/软停级联**
（不改安全边界）。判据：host 用例（越阈 → 故障位 + 级联；未配置则永不触发）+ 台架升温。
**成本最低、收益最明确的一条**：机制全在，只差一个阈值字段。

**1B.4 登记录入**：把 1B.1/1B.2/1B.3 加上 RX ring overflow 的阈值定义（TBD-SAFETY-02）
**登记进 validation plan**——它们现在不在唯一清单里（§2）。

**不做**：不新增独立的 `SafetyTask`（理由见 §3 主张 7）。

### 阶段 2 —— 安全语义真机收口（需台架）

**目标**：把「安全动作在真设备上到底是什么」从猜测变事实。

**覆盖**：validation plan §2 的 TBD-SAFETY-01（两协议 30 ms 软停真机动作）、
CERB 清错帧（bit5 是否要求失能态 / 是否清全部位 / 瞬时还是锁存）、Vaxor 故障恢复、
快照拒绝真机注入（NaN/Inf/未知/重复/跨总线）+ 厂家故障锁存 + 100 ms OFFLINE、
CERB SYNC/20 ms 上报/100 ms OFFLINE 边界、CERB 线圈温度、BMI088 断线/阻塞 SPI。

**判据**：每条给「某计数 / 某事件 / 某段 SDO 字节序列」。例：
- 软停：抓包确认软停帧发出后关节的实际行为（保持/滑行/刹停），写成结案记录。
- 清错：注入故障 → `FAULT` → `system_command`（`command=3`）→ `fault_active=false`
  → `READY_DISABLED` → 显式使能 + 新快照恢复；`clear_fault_seq` 逐次递增。
- 温度：冷态读数接近室温且带载升温方向为正；`protection_flags` bit1 在越阈时置位
  （1B.3 的闭环）。

**关键：本阶段的结论会回头修正 1B.2**——这就是 1B.2 必须「换档不换编码」的原因。

### 阶段 3 —— 参数链与规模收口

**前置**：阶段 2 的设备侧语义（清错 / 保存 / 易失性）已定。

**覆盖**：validation plan §3 的 TBD-PARAM-01..06、REFRESH_PARAMS 真机、
TBD-STORAGE-02、TBD-PROFILE-02（Kt 三套口径）、Vaxor 80 速比规格；
§4 的两条（长示教串行播放、多段组装）。

**新增机制（本节唯一的新代码）**：**`motor.yaml` ↔ `kProfiles[]` 一致性测试**。
建议形态（零新依赖、零 RAM）：新增一个 host CTest，**直接编译固件真实的
`lib/motor_control/src/motor_control_profile.c`**（沿用现有「直接编译固件真实 .c、
无桩」的口径），在同一进程里读
`../../../../ros2_ws/src/motor_hardware_interface/config/motor.yaml`，逐字段比对
`model_name` / `protocol` / `gear_ratio` / `gear_efficiency` /
`torque_constant_nm_per_amp` / 额定与峰值扭矩 / 额定与峰值电流；
并显式断言「**未确认型号不得入目录**」（Vaxor 80 速比两侧都不得出现）。

> **不推荐**「由 `motor.yaml` 代码生成 `kProfiles[]`」：引入构建期依赖与生成物入库问题，
> 收益不值。

**判据**：① 故意把 `cerb_r58_101` 的 `torque_constant_nm_per_amp` 改一位小数 →
`ctest` **立刻红**；② Kt 判别实验结论写进 profile 并附台架数据（工具放
`../../../../micro_ros_agent/tools/`，`cerb_fullscale_probe.py` 是同类先例）；
③ TBD-PARAM-01 若确认「静默写错槽位」仍成立，要把接口升级为**拒绝 + `last_error`**
（那会改 wire 契约 → 需要 ADR + 上位同步）。

**风险**：TBD-PARAM-01 可能揭示 `param_set` 的 `field` 序需要 breaking change。

### 阶段 4 —— 架构收口

**前置**：阶段 1–3 出口达标，且三总线最大规模验收通过
（validation plan 尾行明写「单次九电机 FDCAN1 通过不代表三总线 40 台最大规模验收」）。

**工作项**：
1. **ADR 0007 `proposed` → `accepted`**。
2. **RAM_D1 减压**（可选、有条件）：只在阶段 1A 给出真容量结论**之后**才允许动
   §1.3 的两个常量；且会同步波及 `web_host` 的 `TRAJ_MAX_POINTS` 与
   `motor_hardware_interface` 的 `MAX_TRAJECTORY_SEGMENT_POINTS`（三处刚对齐过，
   见 validation plan §4）。**未获 owner 授权则不动，§6.1 永久锁死。**
3. 复核 §6 两条扩展的触发条件。

**风险**：这一步最容易被当成「架构重构许可」。**只做减压与状态收口，不做分层重排。**

### 排序论证

- **阶段 0 先于一切**：没有它，「WCET / jitter」这个判据测不出来，1A 无从开始。
- **1A 与 1B 并行**：两者互不依赖——1A 要板子，1B 要离线主机；1B 的 host 测试
  可以在台架排期之外推进。**这是本文档唯一推荐的并行点。**
- **阶段 2 先于阶段 3**：参数链的验证依赖设备侧的保存/清错/易失性语义，
  假设错了就是白跑；且阶段 2 的结论会回头修正 1B.2。
- **阶段 4 最后**：架构只在证据齐了才动；减压必须先知道真容量。

## 5. 门禁总表

本文档**唯一允许变红变绿的地方**。阶段出口即门禁全绿。

| Gate | 断言 | 当前 | 判定日期 | 证据 |
|---|---|---|---|---|
| G0 | 示波器可见 1 ms 周期方波；`pio run` 绿 | ⬜ | — | — |
| G1A | §1.2 表里 §1 的开放项 = 0；单总线 VAXOR 上限有具体数字 | ⬜ | — | 待台架报告 |
| G1B | 新增 following-error / 过温 host 用例全绿；1B 四条已登进 validation plan | ⬜ | — | — |
| G2 | §2 覆盖项有结案记录；软停真机行为已写成结论 | ⬜ | — | 待台架报告 |
| G3 | 一致性测试「改 `motor.yaml` 即红」可复现；Kt 结论入 profile | ⬜ | — | — |
| G4 | ADR 0007 = `accepted`；RAM 减压决策已记录（做或不做都要记） | ⬜ | — | — |

## 6. 架构级扩展：触发条件（不排期）

### 6.0 硬门槛：RAM_D1 预算（任何扩展之前）

RAM_D1 余 **17,460 B**。按现有结构，**每多一条总线**至少要：

| 对象 | 量级 | 依据 |
|---|---:|---|
| `can_tx_scheduler_t` | ≈ 8.2 KB | `can_tx_frame_t` = 4 + 4×1 + 64 = **72 B**；P0 `queue[32]` + P1 `slots[64]` + P2 `queue[16]` = 112×72 ≈ 8.1 KB（[can_tx_scheduler.h:29-75](../../../lib/fdcan/include/can_tx_scheduler.h#L29-L75)） |
| RX ring | ≈ 5 KB | `CAN_RX_RING_CAPACITY = 64` |
| 路由工作区（若每总线一份） | ≈ 3 KB | `logical_frames[40]` × 72 B（[motor_tx_router.h:16-24](../../../lib/motor_tx_router/include/motor_tx_router.h#L16-L24)） |

**合成一条总线就要 ~13 KB ≈ 余量的 75%**，再加任何 per-entry 结构必然越界。
→ **在 94.7% 的前提下，扩展 A 连第一块砖都放不下。**
现实的前置不是改代码，是**先做 RAM 减压**（阶段 4 第 2 项）。

**编译期闸门**（改这些常量会连带编译失败，是故意的）：

- `static_assert(MOTOR_CONTROL_MAX_MOTORS == CAN_TX_P1_SLOTS)`（[motor_rt_task.cpp:25](../../../src/motor_rt_task.cpp#L25)）
- `static_assert(MOTOR_CONTROL_MAX_BUSES == CAN_RX_BUS_COUNT)`（[motor_rt_task.cpp:199](../../../src/motor_rt_task.cpp#L199)）

### 6.1 接入新执行器家族（串口舵机）

**五个耦合点**（写下来是为了让评估有依据，不是为了现在动手）：

1. **协议层的帧形状是 CAN 形状**：`motor_protocol_raw_frame_t`
   （[motor_protocol_adapter.h:53-62](../../../lib/motor_protocol/include/motor_protocol_adapter.h#L53-L62)）
   与 `motor_protocol_tx_frame_t`（:72-85）都带 `can_id / is_extended / is_fd / is_brs / dlc`，
   `match_rx_frame` 的语义就是 CAN-ID 归属判定。头里那句「新增协议家族 = 新增一个实例，
   内核零改动」**只在 CAN 之内成立**；建议把这句措辞限定为「CAN 帧形状之内」。
2. **总线索引与 CAN 外设硬绑**：`motor_tx_router.c:135` 硬拒
   `bus_index >= CAN_TX_BUS_COUNT`；`can_bsp.c:14-16` 的
   `kCanHandles[3] = {&hfdcan1, &hfdcan2, &hfdcan3}` 是三个具体外设，`bus_index`
   靠指针比较还原。
3. **TX 路由是半抽象**：HAL 提交已注入（`motor_tx_router_submit_fn`），**不绑 HAL**；
   但工作区是 `can_tx_frame_t logical_frames[40]`、入参 `can_tx_scheduler_t*`、
   `PHYSICAL_BUDGET = 16/ms`（按 CAN-FD 5 Mbps 帧率算）——这些都不是传输无关的。
   串口半双工（逐台轮询、命令-应答延迟、无多轴广播聚合）与 CAN 的共享广播模型不同，
   P1 latest-wins 槽位语义可留，**但 16/ms 这个数无意义**。
4. **ID 空间冲突（最硬的一条）**：`motor_id` **当前就派生自厂家 node 号**
   （[motor_control_internal.h:17](../../../lib/motor_control/src/motor_control_internal.h#L17)：
   「全局唯一（当前派生：= 厂家 node 号；冲突即 CONFIG_CONFLICT）」），
   且撞号是**锁存、不可解除、只能整机重启**。第三个家族若与 vaxor 1..60 /
   CERB 1..127 重叠，会让一台正常电机在整个上电周期内不可用。
   **这是扩展前必须先解决的第一件事**，且它改的是 `motor_id` 派生契约 → 需要 ADR。
5. **ROS 面**：`MotorTopologyEntry.protocol` 是 `kAdapters[]` 的下标（追加安全）；
   `MotorStatusTarget.bus_index` 是 schema v3 刚加的，总线数从 3 扩到 4+ 要复核
   `.msg` 语义与 `web_host` 的假设。

**按 [module-boundaries](../../architecture/module-boundaries.md) §5 的正确走法**：

- §5.1 **先在纯逻辑库加可测试契约**：把 `motor_tx_router` 的帧形状从 `can_tx_frame_t`
  抽象出来（例如一个 POD `actuator_frame_t` + adapter 侧「物理预算查询」），
  **CAN 路径逐字节不变**，用现有 `test_motor_tx_router` 做回归。
  **这一步零风险，是唯一值得提前做的准备**（且不依赖 §6.0 门槛）。
- §5.2 任务层接入：**必然新增第二个 TX 所有者**，直接违反 module-boundaries §2
  「`motor_rt_task` 是正常 CAN TX 唯一所有者」→ **这一步之前必须先有新 ADR**。
- §5.3–§5.5：先跑 host target，再跑最窄 PlatformIO 构建；更新 module-boundaries；
  改公开头/消息/时序/所有权一律另建 ADR。

**触发条件（两条同时满足才启动 §5.1 之后的任何事）**：BOM 里出现只能走串口的
执行器；**且** RAM_D1 已降到 ≤ 80%（≈ 65 KB 余量）。
**现在可以零成本做的只有两件**：把上面 5 处耦合点写成清单（本条即为该清单）；
把 `motor_protocol_adapter.h` 的措辞限定为「CAN 帧形状之内」。

### 6.2 多关节耦合约束 / 受控保持

| 需求 | 能否由现有 `motor_control` 承载 | 理由 |
|---|---|---|
| **following-error** | **能，不需要新模块** | `entry->measured` 与目标位置已在同一结构；判定点放 `service()` 零新依赖（1B.1） |
| **受控保持** | **能，不需要新模块** | 在 `motor_control_send_p0` 的 kind 枚举加一档 + 在级联处选它 + 一个窗口计数。不改所有权、不改 mailbox；但**改安全边界语义 → 需要新 ADR**（1B.2） |
| **多关节耦合约束 `f(q1..qn) ≤ 0`** | **承载不了，且本机构不需要** | ①硬件拓扑是 14 台双臂串联链（§3 主张 5），无绳驱/并联/干涉耦合。②MCU 上唯一合理的耦合是「不需运动学即可表达的代数耦合」；凡需 FK/IK 的按 ADR 0007 §1 归上位——搬到 MCU 就是让 MCU 承担动力学，与调研自己的主张 1 矛盾。③内核对电机以 `motor_id` 为键、**无 `joint_name`**，要聚成「耦合组」就必须先在 MCU 建关节语义 = 架构反转 |

**`MechanicalConstraintManager` 本身不需要 ADR——因为不做**。但要留一条
「已评估未采纳」的记录（写进 ADR 0007 勘误或 module-boundaries），
免得每半年被重新提一次。

## 7. 不做清单

| 不做 | 理由 |
|---|---|
| 在 STM32 上做动力学 / MPC / WBC | 调研自己也这么说；ADR 0008 已把 Pinocchio 划为独立计算边界，不进实时路径 |
| 新增第二个实时任务（`SafetyTask`/`MotionTask`/`ServoBusTask` 分家） | 第二个所有者，违反 module-boundaries §2/§5.2；收益（检测延迟 ≤1 ms → ≤0.5 ms）相对 30 ms 软停阈值与 ~500 ms 看门狗无意义 |
| `DebugShellTask` / CAN 调试桥 | ADR §4 明写调试桥已删除；`src/syscalls.cpp` 强符号空 `_write` 禁止 MCU 侧 printf；调试走诊断话题 + SWD |
| 在 MCU 侧维护 `joint_name` / 建 JointRegistry 关节语义 | ADR 0007 §1 禁止项；映射只存在于主机侧 |
| 复活参数服务器（`rclc_parameter`） | 2026-09-21 整体退役；`services 0/5` 是有意保留的余量，不降 |
| 恢复 name-to-ID 兼容层 | ARD-0007 禁止项 |
| 在 §6.0 预算审计前加任何静态大对象 | RAM_D1 余 17,460 B |
| 在阶段 1A 拿到容量数据前动架构或减压 | 数据可能直接推翻扩展的必要性 |
| 给未标定型号（Vaxor 80 速比）开控制 | 型号目录刻意不收；无规格就没有正确的 Kt 与限值 |
| 由 `motor.yaml` 代码生成 `kProfiles[]` | 引入构建期依赖与生成物入库问题，收益不值（用一致性测试替代） |

## 8. 待 owner 裁定（在启动对应阶段前必须答复）

| # | 问题 | 影响 |
|---|---|---|
| 1 | **目标规模到底是 14 台还是约 40 台？** ADR 0007 的 `Scope` 写「约 40 台外置电机」，而 `motor_driver.yaml` 是 14 台 | 阶段 1A 的容量判据与阶段 4 的 ADR 转 accepted 判据全取决于它 |
| 2 | **R58/R48 实际带不带抱闸？** `README.md` 说「机构当前无抱闸」，但 CERB MIT 控制字 bit6 就是「抱闸释放」且固件**恒置**它 | 决定 1B.2 的正解是「重新夹紧抱闸」还是「kp 保持」——实现完全不同 |
| 3 | **硬件急停是否已有物理通道**（板上按钮 / EXTI 引脚），还是只有外部断电？ | 决定软件急停在 1B 里是纯逻辑还是要动板级 |
| 4 | **是否接受在 `TBD-SAFETY-01` 出结论前先动安全边界**（即 1B.2 是否进阶段 1B）？ | **阶段 1B 的范围开关**。不接受则 1B.2 顺延到阶段 2 之后 |
| 5 | **RAM_D1 减压是否获授权**（阶段 4 第 2 项）？ | 不授权则 §6.1 永久锁死 |
| 6 | **上位 `robot_dynamics` 的重力前馈要不要作为「外部依赖」写进本文**？ | 它直接决定受控保持能否真的不掉臂（§3.2） |
| 7 | 本文档的落点：现为 `docs/plans/active/`，是否改到 `docs/roadmap.md`（避免被当成第二份开放项清单）？ | 只影响发现路径 |

## 9. 横向：文档与契约债

成本低、可与任意阶段并行。**每条都是 2026-10-08 探索中逐条核实过的。**

| 项 | 现状 | 建议 |
|---|---|---|
| `micro_ros_firmware/README.md:44` | 关键决策表写「5 个订阅 + 8 个发布」；实为 **5 订 10 发**（`device_param_report`、`system_status` 加入后未同步） | 改为 5 订 10 发 |
| `docs/adr/0007-…:23` | 勘误块写「3 个订阅 / 6 个发布」。该块自带 2026-09-18 日期、属冻结历史，**不算错**；但 2026-09-23 摘要不复述计数，读者易把 09-18 的数字当现状 | 在 09-23 摘要块补一句当前计数 |
| `micro_ros_firmware/test/README` | 写「当前 CTest 应发现 **14** 个目标」并只列 11 个名字；实为 **22** 个 | 重写该段 |
| 根 `AGENTS.md:82` | 断链：指向 `micro_ros_firmware/docs/handoffs/HANDOFF-motor-runtime-refactor.md`；实际在 `docs/archive/`（`docs/handoffs/` 不存在） | 修路径 |
| validation plan §3 TBD-PARAM-06 的理由 | 写它是「`param_bridge_destroy` 漏调 `rclc_executor_remove_subscription` 的唯一暴露路径」——**代码里已经调了**（`param_bridge.cpp:857,860`、`motor_bridge.cpp:785,789,798`）。用例本身仍值得保留（重连回归），但理由过期 | 改写成「重连回归」，删掉过期的机制描述 |
| `micro_ros_firmware/.host-build/` | 有 3 个**源码已删**的陈旧目标目录：`test_static_topology` / `test_upstream_contract` / `test_motor_event_relay` | 清掉（该目录本身不该入库） |
| `lib/param_store/include/param_store_blob.h` | 三处 v3 字段**预留未实现**：`model_name` / `identity_hash` 写入路径（:60）、记录写的 MIT 三标度（:65）、`profile_version`（:71） | 与阶段 3 的 Profile 一致性一起评估：要么实现，要么在头文件写明「不计划实现」 |
| `MotorStatusTarget.msg` | `protection_flags` bit1/bit2/bit3 与 `limit_flags` bit2 标「预留」，但 validation plan 无承接条目（清单断链） | 补条目或写明「不计划实现」 |
| `motor_protocol_adapter.h` 的「新增协议家族 = 零内核改动」措辞 | 只在 CAN 帧形状内成立（§6.1 第 1 点） | 限定措辞 |

## 10. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-08 | 立。来源：对标外部调研《GitHub 开源人形机器人运动控制板及核心系统架构调研》。确立 §1.4「瓶颈是验证债」、§3 逐条裁决、§4 阶段 0–4、§6 条件性扩展（含 RAM 账与 ID 冲突阻塞项）、§7 不做清单、§8 待 owner 裁定 7 条 |
