# UR5e 仿真抱闸与自碰撞设计

## 1. 目标与边界

让 `robot_arm_gazebo` 在保持现有 CAN 协议、显式 enable、150 ms 超时和纯 Gazebo
边界的前提下，更接近现实机械臂的静止与故障表现：

- 启动、未使能、等待首帧命令、disable、超时和可恢复故障时，六轴进入仿真抱闸，
  不因重力自由塌落；
- 有效控制期间释放抱闸，由现有 motor plant 通过 effort controller 驱动；
- 同一模型的非相邻 link 不能互相穿透；
- 调试时允许不重启 Gazebo 反复恢复控制。

本设计不声称复现 UR5e 厂家制动器的保持力矩、闭合时间或安全等级，不新增真机
bring-up、外部硬件插件、第三个 ROS package 或 MoveIt。仿真抱闸是运行语义，不用于
推导真机安全结论。

## 2. 设计依据

官方实践调研记录在
`docs/research/2026-08-27-robot-arm-simulation-debugging-practices.md`。

- `gz_ros2_control` 的 `hold_joints` 在 command interface 未被 controller 占用时提供
  仿真保持；
- Controller Manager 支持运行时严格切换 controller；
- 故障恢复不必等同于重启整个 launch，但必须有显式确认或重新使能；
- SDFormat 模型级 self-collision 会排除由 joint 直接连接的相邻 link。

因此，抱闸通过释放/重新占用 effort command interface 实现，不在故障时伪造电机
effort；自碰撞交给模型和物理引擎负责。

## 3. 分层架构

### 3.1 `motor_sim`：通用驱动状态

`motor_sim` 继续不知道 UR5e 拓扑，只从任意 `TopologyContract` 和电机集合推导整机驱动
状态，并发布相对 topic `drive_state`。使用 `std_msgs/msg/String`，值严格限定为：

```text
brake_engaged
awaiting_first_command
ready_for_control
motor_control_active
fault_latched
```

状态含义：

- `brake_engaged`：全部电机 disabled，或发生非锁存故障后等待重新使能；
- `awaiting_first_command`：enable 序列已经开始，但尚未满足全部轴首帧条件；
- `ready_for_control`：全部配置轴均 enabled，且每轴至少收到一帧当前 enable 周期内的
  有效运动命令，但 coordinator 尚未确认 effort controller 已激活；
- `motor_control_active`：上述条件成立，且 coordinator 已通过 `control_gate` 确认 effort
  controller 激活成功；
- `fault_latched`：锁存模式下发生安全故障，普通控制帧不能恢复。

`MotorSimNode` 仍在非 active 状态发布六元零 effort。仿真保持由下游 controller/interface
状态完成，因此“故障时电机输出清零”的现有安全语义不变。

### 3.2 `robot_arm_gazebo`：抱闸协调器

在现有 package 内新增机器人侧 `brake_coordinator` 节点，不新增 package。它只负责：

- 订阅 namespace 内 `drive_state`；
- 发布 namespace 内 `control_gate` 布尔确认；
- 调用 `/robot_arm/controller_manager/switch_controller`；
- `ready_for_control` 时以 `STRICT` 模式激活 `effort_controller`，成功后才发布
  `control_gate=true`；
- `brake_engaged`、`awaiting_first_command` 或 `fault_latched` 时以 `STRICT` 模式停用
  `effort_controller`，让 command interface 未被占用；
- 仅在 controller manager 明确返回成功后更新 diagnostics；
- controller manager 不可用、切换超时或切换失败时保持/返回抱闸状态并请求统一 launch
  fail-fast 关闭，不能假装已经切换成功。进入抱闸或故障状态时先发布
  `control_gate=false`，motor node 确认零 effort 后再停用 controller。

协调器不解析 CAN 帧、不计算 plant effort，也不持有六轴协议或 ID。

### 3.3 `gz_ros2_control`：仿真保持

URDF 保持 `<hold_joints>true</hold_joints>`。effort controller 未激活时六轴 command
interface 不被占用，由 `gz_ros2_control` 保持关节。该机制称为“仿真抱闸”，不描述为真实
机械制动器模型。

实施第一步必须做可行性验收：在重力开启时停用 effort controller，六轴 10 秒内各自漂移
不得超过 0.01 rad，且不得明显振荡。若该条件不满足，停止实施并回到设计阶段选择独立、
可参数化的保持 controller；不得用关闭重力或无条件高阻尼掩盖失败。

## 4. 状态转换

整臂抱闸按六轴整体管理，只有全部轴准备好才释放，避免部分轴在重力下无控制。

```text
启动
  -> brake_engaged

任一轴 enable、但并非全部轴都有本 enable 周期的新鲜命令
  -> awaiting_first_command

全部轴 enabled + 每轴至少一帧有效运动命令
  -> ready_for_control

effort controller 激活成功 + control_gate=true
  -> motor_control_active

disable（任一轴）
  -> brake_engaged

超时、越限、错总线、重复控制器
  -> fault_latched       [fault_latching_enabled=true]
  -> brake_engaged       [fault_latching_enabled=false]

transport 断开或进程内部错误
  -> brake_engaged
  -> 统一 launch 关闭
```

进入任何非 active 状态时，顺序必须是：`control_gate=false`，motor node 发布并确认一次
六轴零 effort，再停用 effort controller。进入 active 时，顺序必须是：motor node 保持
零 effort 并发布 `ready_for_control`，确认 effort controller 激活成功，coordinator 再置
`control_gate=true`，最后 motor node 进入 `motor_control_active` 并允许非零 effort。
切换期间一律保持零 effort，避免 controller 激活前产生待发送的非零命令。

## 5. 故障恢复策略

保留 launch 参数 `fault_latching_enabled`，但不再规定锁存后必须重启整个 launch。

### 5.1 锁存开启

- 普通 enable 和运动命令不能清除 `fault_latched`；
- `MotorSimNode` 提供 namespace 内 `reset_fault` 服务，类型为 `std_srvs/srv/Trigger`；
- 只有 transport 仍连接、全部轴已收到 disable 且当前没有越限条件时 reset 成功；
- reset 后进入 `brake_engaged`，必须重新完成 enable + 全轴首帧流程才能释放抱闸；
- transport 断开或内部不可恢复错误仍关闭 launch。

### 5.2 锁存关闭（调试模式）

- 故障发生时仍清零并进入 `brake_engaged`；
- 故障条件消失后，必须观察到明确的全轴 disable，再接受新的 enable 周期；
- 迟到的旧命令或单独一帧运动命令不能释放抱闸；
- 不需要重启节点或 Gazebo。

这会收紧当前 `fault_latching_enabled=false` 的行为：调试模式允许快速恢复，但不再允许
故障后继续沿用旧控制状态。

## 6. 启动编排

统一入口仍是 `gazebo.launch.py`，启动顺序调整为：

1. Gazebo、state publisher、clock bridge；
2. 创建 UR5e entity；
3. 激活 `joint_state_broadcaster`；
4. 加载并配置 `effort_controller`，但保持 inactive；
5. 启动 `robot_arm_motor_sim_node` 和 `brake_coordinator`；
6. coordinator 收到 `motor_control_active` 后才激活 effort controller。

motor node、coordinator、Gazebo 或 controller 切换发生不可恢复失败时，统一 launch
fail-fast 关闭。动作播放器仍不由 launch 启动，也不得与外部电机仿真并行控制。

## 7. 自碰撞

UR5e 模型启用模型级 `self_collide`。实施必须使用 `gz sdf -p` 或等价转换检查最终 SDF，
确认属性确实为 true，而不是只检查 xacro 源文本。

验收顺序：

1. 七个 link 均保留 collision geometry；
2. 默认初始姿态无非相邻 collision mesh 重叠；
3. 直接由 joint 相连的 link 按 SDFormat 规则不互撞；
4. 非相邻 link 接触时不能穿透；
5. 接触不能造成持续爆振、模型飞散或实时因子不可接受下降。

若个别非相邻 collision mesh 在正常工作空间误碰撞，只针对有证据的 link/collision 做
最小修正，不全局关闭 self-collision。

## 8. 参数与接口

保留现有 launch 参数，并增加：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `drive_state_topic` | `drive_state` | motor node 发布、协调器订阅的驱动状态 |
| `control_gate_topic` | `control_gate` | 协调器确认 controller 就绪的布尔接口 |
| `reset_fault_service` | `reset_fault` | 锁存故障显式 reset 服务 |
| `controller_switch_timeout` | `2.0` | controller 切换确认超时，单位 s |
| `self_collision_enabled` | `true` | 是否启用模型级自碰撞，调试覆盖用 |

每个参数继续在 launch 中集中默认值、中文注释和中文 `description`，支持 `--show-args`
查看和命令行覆盖。

diagnostics 至少包含：当前 drive state、抱闸是否闭合、effort controller 实际状态、最近
一次转换原因、故障类型、锁存状态和 reset 拒绝原因。

## 9. 测试与验收

### 9.1 自动测试

- `FrameRouter`/安全层状态转换：全轴 enable、逐轴首帧、ready/gate 握手、disable、150 ms 超时、锁存与
  非锁存 rearm；
- reset 前置条件和 reset 后必须重新 enable + 首帧；
- motor node 的 `drive_state` 和零 effort 顺序；
- coordinator 的幂等切换、STRICT 请求、成功确认、超时和失败；
- launch 中 effort controller 初始 inactive，motor/coordinator 只启动一次；
- 最终 SDF 的模型级 self-collision 为 true；
- 现有协议 golden vector、PTY、SocketCAN 和六轴 ABI 测试不得回退。

### 9.2 Gazebo 运行验收

- 启动后 10 秒内六轴漂移各不超过 0.01 rad；
- enable 但缺任一轴首帧时仍保持；
- 全轴首帧后 effort controller 激活，vcan 命令可驱动关节；
- 任一轴命令超过 150 ms 后，effort 清零并在故障位置保持；
- 锁存开启时 reset + rearm 可恢复且无需重启 Gazebo；
- 锁存关闭时 disable + enable 可重复恢复；
- 非相邻 link 不能互穿；
- 自动测试不要求 GUI、真实 tty 或真实 CAN 设备，运行验收使用现有 `vcan0`。

## 10. 文档与兼容性

修改运行入口、协议安全状态和恢复语义时同步更新根 README、`CONTEXT.md`、相关测试和
`docs/verification/`。现有 CAN identifier、payload、六轴顺序和 `motors.yaml` 配置格式
保持兼容。

用户当前工作区对 `gazebo.launch.py` 默认连接参数的未提交修改不属于本设计内容，实施时
必须逐行保留，不得覆盖或提交为本功能的一部分。
