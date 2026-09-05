# 电机运行时重构交接

> 写入日期：2026-09-04。本文记录的是继续重构所需的实施断点，不替代架构设计。
> 目标设计以 [ARD-0007](ARD-0007-micro-ros-motor-runtime.md)、
> [ADR 0007](adr/0007-micro-ros-discovery-driven-motor-runtime.md) 和
> [架构开发基线](motor-runtime-decision-log.md) 为准；三者与本文冲突时，先查明
> 是否是本文已过期，再以 ARD/ADR 中更明确的安全边界为准。

## 0. 进度更新（2026-09-05）

阶段 A、B、C（固件侧）**已完成**并通过验证：host 套件 10/10（约 45 用例）、
`pio run` SUCCESS（Flash ~16.8%）。§4 的整编译断点已解除，`control_task` 已由
`motor_rt_task` 整体替换。本节记录实施中确立/核实的事实，与下文旧断点描述冲突时
以本节为准。

**协议核实（2026-09-05，与实际设备行为核对）**——两协议反馈均为两部分：

- vaxor：12B 回显帧 @ reply-id（0x701+4n）= 控制帧的实时反馈（100Hz 跟随命令）；
  20B 状态帧 @ status-id（0x702+4n）= 20ms 周期自主上报，承担总线检测。
- CREB：反馈帧（0x300+node）= 命令实时状态（100Hz）；0x080 SYNC 触发的上报帧 =
  20ms 周期状态心跳（同样承担总线检测）。
- 推论：两协议均有与命令无关的 20ms 心跳 → adapter `autonomous_feedback=true`、
  OFFLINE = 心跳丢失 100ms（vaxor 与 CREB 相同）；SYNC 须以 10ms 周期发送维持
  上报（任务层 `sustain_discovery_sync`，P2），属管理性使用，仍禁止作控制触发。

**实施补全**：

- TX 桥：`motor_rt_task::route_kernel_tx()` 每 1ms 把内核待发帧路由到每总线调度器
  （P0 FIFO / P1 latest-wins 槽 / P2 FIFO）；`motor_protocol_tx_frame_t` 新增
  `tx_slot` = 内核表项索引（只增不减故稳定）。
- 逐电机 `motion_authorized` 是真实门禁：apply 快照前检查；全新设备建档默认完整，
  故障锁存/离线/撤权/SET_ENABLE(false) 清除后**不得自动恢复**，须显式 SET_ENABLE(true)。
- SET_MODE 事务收尾强制复位 `motion_enabled`（重走使能序列，防事务停在 stop 态后
  内核仍记"已使能"）。
- 软停帧编码失败与失能路径同语义：挂起 `p0_pending` 重试。

**剩余工作**（按序）：① 上层 `motor_hardware_interface` 迁移（阶段 C 上半，见 §5
阶段 C）——joint↔motor_id 映射、发 MotorCommandArray、收 MotionStateArray。
**2026-09-05 用户指示：`ros2_ws/` 暂不动，本项挂起**；废弃旧消息的删除依赖上层
迁移，一并挂起（5 个旧消息继续保留）。上层迁移启动时的顺序：**先更新
`../micro_ros_agent/`（消息包所在真实源目录），再动 `ros2_ws/` 上层**。
② §7 台架验证（T1–T12）后 ADR 转 Accepted。


## 1. 先做什么

继续工作前按下面顺序执行，不要从旧的四电机示例继续扩展：

1. 阅读本文件、ARD 的第 8、11、14～28、43、46、50～55 节，以及开发基线。
2. 检查 `git status --short`，保护用户未提交的工作。
3. 先把 `lib/motor_control` 从“静态电机表 + `joint_name`”改成 ID 驱动的运行时内核；
   不给已经删除的 mailbox API 补兼容包装。
4. 再迁移 `control_task`、消息包和 ROS 桥。每完成一个边界，补对应 host test，最后才
   追求整固件重新通过编译。

交接写入时的代码基线为提交 `67e6a8e 重构一9.4`；本文和 `AGENTS.md` 的入口提示是
在该基线上新增的文档改动。该提交还含用户的 ROS/CAD 文件，处理固件时不要触碰它们。

## 2. 已经确定、不应重新讨论的架构边界

| 主题 | 已确认决定 |
| --- | --- |
| 规模和总线 | 约 40 台设备，FDCAN1/2/3 均为 CAN-FD、仲裁段 1 Mbps、数据段 5 Mbps。 |
| 设备标识 | 系统范围内 `uint16_t motor_id` 唯一；MCU 不维护每台电机的安装表或 `joint_name`。上层维护 `motor_id ↔ joint_name`、方向、零位和关节语义。 |
| 发现 | Vaxor 每约 20 ms 自主上报状态；CREB 的 `0x080` SYNC 用于发现、重连和热插拔探测，不是常规 100 Hz 控制触发。2026-09-05 核实：两协议反馈分"命令实时回显（100Hz）"与"20ms 周期状态心跳（总线检测）"两部分；SYNC 须 10ms 周期发送维持上报（详见 §0）。 |
| 默认控制 | Vaxor 和 CREB 默认 MIT。CREB 在设备侧支持限位设置；Vaxor 不支持设备侧限位。限位单位为 `rad`。 |
| 实时所有权 | FDCAN ISR 只把原始帧写入每总线静态 RX 环；`motor_rt_task` 是唯一解码者、正常 CAN 发送者和控制状态机所有者。CAN 调试桥已删除，只能读状态副本/统计。 |
| 节拍 | `motor_rt_task` 每 1 ms 做有界服务；每 10 ms（100 Hz）消费一次完整命令快照并生成控制意图。控制和反馈目标频率均为 100 Hz。 |
| 发送调度 | 每总线 P0 安全、P1 MIT latest-wins、P2 发现/配置；不能让 P2 挤占 P0/P1。MIT 默认逐电机帧，CREB 超过 8 台也不因此切到 `0x200`。 |
| ROS 边界 | 上层控制器使用 ros2_control 标准 position/velocity/effort interfaces；MCU 高频接口使用固定容量、ID 化的 `MotorCommandArray`，上行使用 ID 化 motion/status/topology 消息。不要再用 `sensor_msgs/JointState` 作为 MCU 命令接口。 |
| 存储和外围 | W25Q64 仅供低频日志/诊断，单一 storage 所有者，不能阻塞 1 ms/10 ms 路径；DMA 是共享基础设施，当前主要服务 IMU，未来可服务 WS2812 等；MPU/cache 仅围绕 DMA 缓冲和任务隔离设计。 |

实时路径只使用静态容量、短临界区和副本；不得在 ISR、控制边界、协议发送路径中动态分配、等待 Agent、等待电机响应或访问 W25Q64。

## 3. 已完成的工作

### 3.1 已删除的旧边界

- CAN 调试桥已删除：`include/can_debug_bridge.h`、`src/can_debug_bridge.cpp`。
- 原始 CAN snapshot API 已删除：`lib/fdcan/include/can_snapshot.h`、
  `lib/fdcan/src/can_snapshot.c` 及其 host test。
- 调试或 ROS 层不得重新成为 FDCAN RX 主队列的第二消费者。

库的目标归属已收敛为：

```text
lib/
  fdcan             三路硬件、IRQ、RX ring、TX 调度、底层统计
  motor_protocol    Vaxor/CREB 厂家帧编解码与设备事务
  motor_control     注册表、Profile、生命周期、安全、状态副本
  control_mailbox   跨任务完整命令/配置/会话事件交换
  dma               共享 DMA 资源
  imu               BMI088 业务驱动
  w25q64            OSPI 原始驱动
  ws2812            板载状态灯
```

### 3.2 已完成的 `control_mailbox` 直接重构

以下新接口已经存在，且 host test 覆盖了关键行为：

- `lib/control_mailbox/include/control_command_mailbox.h`
- `lib/control_mailbox/include/control_mailbox.h`
- `lib/control_mailbox/src/control_command_mailbox.c`
- `lib/control_mailbox/src/control_mailbox.c`

其语义是：

- 高频 `control_command_snapshot_t` 是最多 64 项的完整 ID 快照：
  `{ sequence, received_tick_ms, target_count, { motor_id, position, velocity, effort }[] }`。
- 命令采用双缓冲、latest-wins；拒绝超过容量、非有限浮点数、重复 ID 的快照。
- 低频配置是容量 16 的 FIFO：`SET_MODE`、`SET_GAINS`、`SET_POSITION_LIMITS`，带
  `request_id`、`motor_id`、接收时间；不覆盖已入队请求。
- `authorization` 与 Agent 连接状态输出“状态电平 + 一次性边沿”。撤销授权会清命令；
  已连接后断开会清命令和配置、撤销授权，恢复后必须重新授权并给出新快照。
- 该库不依赖 HAL、FreeRTOS、Arduino、ROS、`motor_control` 或任何厂家协议。并发调用者
  必须自己提供短临界区。

`control_mailbox` 有意不知道协议、关节名、静态电机槽位和策略裁决。协议/能力校验属于
`motor_control` 与 `motor_protocol`；不要把这些依赖放回 mailbox。

## 4. 当前断点：固件整编译暂时会失败（已解除，见 §0）

> 2026-09-05：本断点已解除——`control_task` → `motor_rt_task` 迁移完成，
> `pio run` 通过。下表保留作为迁移语义的记录。

这是一次**不保留兼容层**的库重构。旧的 `src/control_task.cpp` 仍调用已删除的接口，故交接时
`pio run` 会在该文件失败；这不是应靠 wrapper 掩盖的问题。

| 旧实现仍在使用 | 已删除/改变的事实 | 正确迁移方向 |
| --- | --- | --- |
| `control_mailbox_init(..., kDefaultMotors, kDefaultMotorCount)` | `control_mailbox_init()` 不接收电机表。 | 初始化空 mailbox；运行时注册表由 `motor_control` 在发现后维护。 |
| `motor_control_command_t` 按 `joint_name` 单条提交 | mailbox 只接收 `control_command_snapshot_t`。 | ROS 桥构造完整 ID 快照，实时任务在 10 ms 边界整体消费。 |
| `submit_mode`、`submit_gains`、`take_mode`、`take_gains`、`has_configuration` | 这些 per-slot API 已删除。 | 用统一的 `submit_config` / `take_config` FIFO，并由 runtime 做能力和事务判断。 |
| `control_mailbox_*_slot_t` 静态数组 | 槽位类型已删除。 | 不再有“默认第 N 台电机”的数据模型。 |

交接前已执行的 host 验证为：

```bash
cd micro_ros_firmware
cmake -S test/host -B .host-build
cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure
```

结果为 8/8 通过。存在一个与本重构无关的 `led_task.cpp` C++ narrowing warning。上述 host
验证不能证明当前整个 PlatformIO 固件可编译；下一阶段迁移完成后才应重跑 `pio run`。

## 5. 推荐的续作顺序

### 阶段 A：先重构 `lib/motor_control`

当前 `motor_control.h` 仍有以下旧模型，必须整体替换，而不是把容量从 8 改成 64：

- `MOTOR_CONTROL_MAX_MOTORS 8U` 与调用方传入的静态 `motor_config_t[]`；
- `char joint_name[32]`、按名称查找命令/模式/增益；
- 固定的 `motor_control_joint_state_t` / `motor_control_motor_status_t`；
- 把原始 FDCAN 帧形状暴露为旧业务 API。

新内核至少应提供：

1. 静态容量不少于 64 的运行时注册表，键为 `(motor_id, bus_index)`，且全局拒绝重复
   `motor_id`；表项保存身份、协议、型号/Profile 引用、能力、生命周期、freshness、运动与
   诊断状态。
2. 编译期**型号目录** `MotorModelProfile`，而不是安装表。型号从 Vaxor 状态或 CREB
   读取/管理事务确认后选择。没有完整规格的 Vaxor 80 速比不可作为可控 Profile。
3. 不含 ROS 字符串、HAL、FreeRTOS 或 `control_mailbox` 依赖的纯 C 运行时接口。
   `motor_rt_task` 把 mailbox 快照转换为 runtime 输入；runtime 不反向依赖任务层。
4. 原始帧输入、命令/Policy/Config 输入、状态/事件输出、待发送帧输出均使用固定容量
   数据结构。厂家字节处理只调用 `motor_protocol`。
5. 生命周期至少覆盖 `ABSENT → DISCOVERED → IDENTIFYING → READY_DISABLED → ACTIVE`，
   以及 `SAFE_STOP`、`FAULT`、`CONFIG_CONFLICT`、`OFFLINE`；只有 `ACTIVE` 生成运动帧。
6. 实现完整快照缺席即软停、30 ms 无新快照软停、100 ms 或 Agent 断开协议级停机/失能；
   恢复要求新的授权和新的完整快照，不能复用旧目标。
7. CREB 限位配置必须是 `request_id` 事务：参数校验、写入、读回核验、统一结果码；设备
   不支持或结果不一致不能恢复运动授权。Vaxor 只走软件限位。

先为上述行为建立/迁移 host test；不要把四个默认测试电机扩展到 40 台。

### 阶段 B：将 `control_task` 收敛为 `motor_rt_task`

`src/control_task.cpp` 当前同时拥有静态四电机表、RX drain、直接 `fdcanx_send_frame`、
mailbox 消费、状态机和对外的关节数组 API，职责过宽。目标不是拆出多条实时任务，而是：

- 保留**一条** `motor_rt_task` 作为唯一实时业务所有者；每 1 ms 做有界 RX/事务/TX 服务，
  每 10 ms 做控制边界。
- 把三路硬件接收、RX ring 和 TX 调度继续下沉到 `lib/fdcan`；任务层不保留 raw CAN
  调试出口，也不应长期直接组帧/直接发送。
- 任务层消费一次完整命令快照、有限个配置 FIFO 请求和会话边沿，调用 `motor_control`，并
  提供只读的状态/拓扑/诊断副本给 ROS 桥。
- 重命名为 `motor_rt_task.cpp/.h` 时同步修改 `main.cpp`、`ros_bridge.cpp` 和 host build。
  不需要为旧 `control_task_*` 建兼容转发。

建议的最终平面 `src/` 结构是：

```text
main.cpp                 启动编排
system_clock.cpp         时钟
micro_ros_task.cpp       Agent 会话
ros_bridge.cpp           node/executor/各域桥生命周期
imu_task.cpp             现 sensor_task 的 BMI088/温控职责
imu_bridge.cpp           IMU ROS 发布
motor_rt_task.cpp        唯一电机实时任务
motor_ros_bridge.cpp     电机 ROS 消息边界
led_task.cpp             WS2812 任务
ros_time.cpp             可选：现 time_utils 的明确命名
```

无需按功能再过度切目录；把深模块接口做小即可。`sensor_task.cpp` 可以在电机重构之外单独
改名为 `imu_task.cpp`，但不要把未来所有传感器塞入其中。

### 阶段 C：冻结消息包，再迁移 ROS 桥和上层

固件中的 `extra_packages/can_bridge_msgs` 是指向
`../../micro_ros_agent/src/can_bridge_msgs` 的符号链接。因此改消息时要改真实源目录：
`micro_ros_agent/src/can_bridge_msgs`。

当前 `MotorCommand.msg`、`MotorGains.msg`、`MotorMode.msg`、`MotorStatus.msg` 全部仍以
`string joint_name` 和单条命令为核心，是旧接口，不能继续向 MCU 扩张。先新增/冻结以下
有界 ID 化接口（具体 `.msg` 排列与 QoS 仍需依据 ARD 完成）：

- 高频 `MotorCommandArray`：同步时间、`uint32 sequence`、`uint8 count`、最多 64 个
  `uint16 motor_id + float32 position/velocity/effort`；表示完整快照。
- 上行 `MotorMotionStateArray`、`MotorStatusArray`、低频 `MotorTopology` / `MotorEvent`；
  不携带 `joint_name`，缺失浮点状态使用 `NaN`。
- 低频 `MotorPolicy` 和 `MotorConfig`：后者带 `request_id`、ID 和明确结果消息；CREB
  限位为 rad，并支持读回确认。

之后将 `motor_bridge.cpp/.h` 改为 `motor_ros_bridge.cpp/.h`：它只做消息校验、mailbox
写入和状态副本发布。现有 `motor_bridge` 的 create/destroy/poll 外部生命周期形状可以保留，
但不可继续含有固定 4 台、字符串拷贝、协议/CAN 细节或 `JointState` 命令解析。

上层 `motor_hardware_interface` 完成关节名映射，向 MCU 发 ID 消息；不要在 MCU 增加
name-to-ID 适配层来“临时兼容”。

### 阶段 D：再更新文档和验收

迁移到可编译实现后，更新 `README.md` 与本文件所在的 `AGENTS.md` 中仍描述旧
“四台默认测试电机”、`/joint_commands`、旧 mode/gains 话题和 10 ms CREB 常规 SYNC 的段落。
ARD 和开发基线已经是目标架构的规范来源，不要复制出第四套长文档。

## 6. 当前源码审查结论

下列内容是下一位开发者最容易误用的旧边界：

| 位置 | 现状 | 处理原则 |
| --- | --- | --- |
| `include/control_task.h` | 泄漏 `can_rx_queue.h`、`motor_control.h`、静态索引和 `joint_name` 反馈。 | 随 `motor_rt_task` 重写为 mailbox 输入、会话事件和只读 ID 状态副本接口。 |
| `src/control_task.cpp` | 静态四电机表、RX drain、直接 TX 和状态机混在一起。 | 保留任务编排，逐步把硬件/调度下沉到 fdcan，把生命周期/安全下沉到 motor_control。 |
| `include/motor_bridge.h` / `src/motor_bridge.cpp` | ROS entity 生命周期与旧 `JointState`、字符串、4 台上限、状态发布混合。 | 保留一个深的 ROS 桥 façade；迁移消息边界，必要时只拆私有实现文件。 |
| `src/ros_bridge.cpp` | 合理地负责 node/executor 生命周期，但硬编码 executor handle 数，且直接转发 Agent 状态给 control task。 | 消息迁移时让各 bridge 暴露 handle 数；会话事件在 motor bridge/mailbox 边界进入。 |
| `lib/fdcan` | 已有每总线静态 RX 队列，但当前深度和任务 drain 配额仍是旧实现。 | 按 ARD 收敛到每总线独立 RX ring、计数和有界配额；ISR 绝不解码。 |

`main.cpp`、`system_clock.cpp`、`micro_ros_task.cpp`、`imu_bridge.cpp`、`led_task.cpp` 的职责
目前基本清晰；本轮不需要因电机重构而连带改动它们，除非改名/启动接口必须同步。

## 7. 验证和安全底线

每一层迁移后至少执行其 host test；合并前执行：

```bash
cd micro_ros_firmware
cmake -S test/host -B .host-build
cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
git diff --check
```

改 `platformio.ini` 的 HAL 宏后先执行 `pio run -t clean`，因为 SrcWrapper 不会因宏变化
自动完全重编译。不要在常规验证中运行会重新下载依赖的 `clean_microros`。

真机验证只能在硬件急停、机械限位和供电隔离可用时进行；软件 stop 不能替代这些保护。
在三总线、约 40 台、最坏反馈流量的台架测试中，必须实际记录：RX overflow/backlog、P0/P1
发送延迟、100 Hz jitter/WCET、总线负载，以及 30/100 ms 超时边界。未取得这些数据前，
ADR 0007 仍为 Proposed。

## 8. 仍待闭环、不可擅自假定的项目

优先按 ARD 第 47 节逐项关闭，尤其是：

- 两协议精确软停定义与 OFFLINE 判定；
- Policy 前馈语义、`.msg` 最终字段和 QoS；
- 配置结果的消息和阶段 timeout；
- FDCAN 过滤器、P2/TX/RX 配额与容量；
- 公共 fault/limit/protection 位标志、Profile 一致性检查；
- Vaxor 80 速比完整 Profile；
- 最大规模台架的总线容量与实时数据。

遇到这些未决项时，先记录为 TBD 或做可验证的最小实现；不要依据旧四电机路径、厂商协议
猜测或为让编译通过而把安全条件放宽。

## 9. 下一阶段完成的判定

下一位开发者至少应能证明以下事项后再宣布 runtime 主干迁移完成：

- 不存在 `joint_name`/固定安装表/四电机上限作为 MCU 控制键；
- `motor_control` 是可 host-test 的 ID 化静态运行时内核，且不依赖 ROS、HAL、RTOS、mailbox；
- ISR → RX ring → 单一 `motor_rt_task` → runtime/protocol → 每总线 TX 调度的所有权闭合；
- 命令为完整快照且缺席/超时/断链的安全动作满足 ARD；
- ROS 桥只通过 mailbox 与状态副本交互，CAN 调试桥没有复活；
- host tests、`pio run`、格式检查通过，并有不带负载的硬件安全验证记录。
