# UR5e RP/Vaxor Gazebo 机械臂仿真设计

## 1. 目标与边界

在现有 ROS 2 Humble + Gazebo Fortress 纯仿真工程中新增一套独立的
UR5e 机械臂仿真，专门验证本项目 RP 和 Vaxor 协议、USB/USB-CANFD 传输、
电机 plant、控制器接线和安全状态机。

机械臂是与人形机器人分开启动的独立仿真对象，不挂载到人形上，不替换人形
双臂。仓库仍只提供 Gazebo 仿真，不增加真机 bring-up、实物驱动、MoveIt 2、
硬件插件或对 UR 原厂驱动的依赖。本设计不模拟夹爪、力/力矩传感器、CAN-FD
仲裁、总线负载、终端电阻、误码或 USB 适配器完整固件行为。

第一版的交付标准是“控制逻辑闭环可验证”，不将仿真参数描述为 UR5e、
RP 或 Vaxor 的真机性能，不执行 10 分钟、30 分钟或更长时间的持续运行验收。

## 2. 包结构与模块边界

工作空间由三个 ament package 组成：

```text
src/
├── motor_sim/            # 与机器人拓扑无关的通用引擎
├── humanoid_gazebo/      # 现有30轴人形模型和拓扑契约
└── robot_arm_gazebo/     # UR5e模型、6轴契约、launch和动作
```

`motor_sim` 对外导出三个深模块和一个通用 ROS 组件：

- `motor_protocol`：`CanFrame`、RP/Vaxor 命令与反馈编解码、ID 策略；
- `motor_transport`：帧记录、POSIX tty、`usb_serial`、`serial_slcan`和到达时间；
- `motor_sim_core`：通用配置数据模型、传动换算、plant、路由和锁存安全状态；
- `MotorSimNode`：将任意轴数和总线数的配置接入 ROS joint state、effort 命令和
  diagnostics。

共享核心不知道“人形”或“UR5e”，也不包含 30 轴/6 轴分支。每个模型包
保留一个小型拓扑契约层，向通用 loader 提供固定的有序 joint、bus、protocol
和 ID 期望值。人形和机械臂包各自编译一个薄的 model-specific executable：它只构造
`TopologyContract` 并实例化 `MotorSimNode`，不复制协议、transport、plant 或安全逻辑。
`motors.yaml` 是模型电机配置的权威来源，拓扑契约是独立的
运行时护栏；静态测试负责确保契约、YAML、URDF 和 controller 顺序完全一致。

抽取共享包允许调整 `humanoid_gazebo` 内部头文件、namespace 和 target 名称，
但最终必须同时满足：三个包全部构建、全部自动测试通过，人形的三个现有
launch 仍能启动，其三端口、30 轴 ABI、topic、协议和安全行为不回退。

## 3. UR5e 上游资产

模型固定到 Universal Robots 官方 `Universal_Robots_ROS2_Description` Humble 分支
提交：

```text
18e6f603b3ebc2ec479fecb62d6be544b15755e9
```

只 vendor UR5e 必需的资产：

```text
LICENSE
urdf/ur_macro.xacro
urdf/inc/ur_common.xacro
urdf/inc/ur_transmissions.xacro
config/ur5e/joint_limits.yaml
config/ur5e/default_kinematics.yaml
config/ur5e/physical_parameters.yaml
config/ur5e/visual_parameters.yaml
config/initial_positions.yaml
meshes/ur5e/visual/{base,shoulder,upperarm,forearm,wrist1,wrist2,wrist3}.dae
meshes/ur5e/collision/{base,shoulder,upperarm,forearm,wrist1,wrist2,wrist3}.stl
```

`robot_arm_gazebo` 通过本地 wrapper xacro 调用上游 macro，设置
`generate_ros2_control_tag:=false`，再附加本项目 Fortress/`ign_ros2_control` effort 接口。
不复制上游 `ur.ros2_control.xacro`、launch、controller、MoveIt 或真机驱动。对
`$(find ur_description)` 和 mesh package URI 的修改仅限于指向 `robot_arm_gazebo`。

vendored 目录保留上游 BSD-3-Clause `LICENSE`，新增 `UPSTREAM.md` 记录原仓库、
完整 commit、文件清单和本地修改。UR5e 不在上游 README 列出的附加
Graphical Documentation 限制型号中。上游物理参数自身带有准确性保留说明，
因此它们只作为仿真名义参数，不用于真机安全结论。

## 4. 模型、世界与坐标

机械臂保留官方 link、joint 和 frame 命名，包括 `base_link`、
`shoulder_pan_joint` 至 `wrist_3_joint` 和 `tool0`。基座竖直固定在世界原点。
独立 world 只包含地面、灯光和机械臂，物理步长 1 ms，controller 更新 100 Hz。
默认 `gui:=true`，自动化验收使用 `gui:=false`。

使用官方 visual meshes 和简化 collision meshes。机械臂与地面或未来添加的
外部物体正常碰撞，第一版不启用内部 self-collision。不添加夹爪、传感器或工作台。

默认初始位置采用上游 `initial_positions.yaml`：

```text
[0.0, -1.57, 0.0, -1.57, 0.0, 0.0] rad
```

`initial_positions` launch 参数允许覆盖，但必须是严格 6 元、全部有限且不越过
官方 hard limit 的弧度值。文档只将默认值称为“仿真默认姿态”，不声称它是
真机通用安全姿态。

## 5. 六轴电机 ABI

六个驱动关节固定顺序如下：

| 顺序 | joint | protocol | ID | bus |
|---:|---|---|---:|---|
| 1 | `shoulder_pan_joint` | RP | 1 | `arm` |
| 2 | `shoulder_lift_joint` | RP | 2 | `arm` |
| 3 | `elbow_joint` | RP | 3 | `arm` |
| 4 | `wrist_1_joint` | Vaxor (`motor_2530`) | 4 | `arm` |
| 5 | `wrist_2_joint` | Vaxor (`motor_2530`) | 5 | `arm` |
| 6 | `wrist_3_joint` | Vaxor (`motor_2530`) | 6 | `arm` |

所有电机共用一条逻辑总线 `arm` 和一个 tty。RP 与 Vaxor 使用不同协议 ID
规则；选用全局数字 1–6 是为了便于识别和调试，不表示两类协议共用相同
CAN identifier 编码。

外部协议的正位置和正速度与官方 URDF joint axis 正方向一致。各轴
transmission `direction` 只处理电机坐标与 joint 坐标的符号转换，不改变对外解剖
语义。启动姿态反馈实际 URDF 关节角；Vaxor `set-zero` 仅改变当前进程内
该电机的逻辑零点，不写磁盘，重启后恢复 YAML/URDF 坐标。

## 6. 限位、传动与 plant

关节 hard limit、velocity limit 和 URDF effort limit 来自固定版本的官方 UR5e 配置。
command limit 在 hard limit 两端各收缩 5°。电机 plant 使用与本项目现有参数
同数量级的仿名义 profile：

- RP 1–3：肩部级 profile，ratio 1、效率 1、joint effort limit 60 Nm；
- Vaxor 4–6：踝部级 profile，ratio 80、效率 0.8、`Kt=0.012 Nm/A`、
  current limit 11 A、joint effort limit 10 Nm。

最终关节 effort 不得超过 UR5e 模型上限和电机 profile 上限中更严格的一个。
Vaxor feedforward current 按 current limit 裁剪后，再通过 Kt、ratio 和效率转换到
joint effort。位置/速度反馈与 MIT 增益继续按电机侧与关节侧传动关系正反换算。

plant 和 ROS 动作的 PD 增益从 `kp=30`、`kd=1` 起步，通过无 GUI Gazebo 逐轴
测试调整并固定到 YAML。最终值必须让验收动作在 effort 不持续饱和的前提下
达到 0.05 rad 目标误差。配置注释记录最终值是为满足仿真验收调整，不是真机标定。

## 7. ROS 接口和 launch

`robot_arm_gazebo` 的 Gazebo entity 名为 `ur5e`，所有 ROS 资源置于
`/robot_arm` namespace：

```text
/robot_arm/joint_states
/robot_arm/effort_controller/commands
/robot_arm/controller_manager
/robot_arm/diagnostics
```

包提供三个互斥入口：

- `gazebo.launch.py`：Gazebo + 6 轴 effort controller，不启动串口 motor node；
- `usb_sim.launch.py arm_port:=...`：Gazebo + `usb_serial` motor node；
- `canfd_sim.launch.py arm_port:=... allow_external_canfd:=true`：Gazebo +
  `serial_slcan` motor node。

ROS 动作模式使用有界 joint-space PD 直接发布 6 元 effort 数组，不引入
`joint_trajectory_controller` 或 MoveIt。动作包括 `home`、逐轴 `joint_scan`、六轴小幅
组合动作和回零。动作播放器不得与两个 external-control launch 同时运行。

launch 默认串口速率 3,000,000 baud，允许显式选择 transport 已支持的
115200、460800、921600、1M、2M 或 3M，不实现自动波特率探测。`arm_port`
可以是任意存在且可打开的 tty，包括 `/dev/serial/by-id/...`、`ttyUSB*`、
`ttyACM*` 和 PTY。启动日志和 diagnostics 必须报告请求路径及解析到的实际设备，
文档提醒非稳定路径在重枚举后可能指向不同设备。

## 8. USB 与 USB-CANFD 传输

`usb_serial` 只读写本项目现有 `t/T/d/D/b/B` 行帧记录，不发送适配器命令。
`serial_slcan` 打开 tty 后按顺序发送：

```text
C → S8 → Y5 → O
```

对应关闭通道、仲裁段 1 Mbit/s、数据段 5 Mbit/s + BRS、打开通道。
`serial_slcan` 只有在 `allow_external_canfd:=true` 时启动。

两种模式都模拟帧编解码、ID 路由、BRS 标志、初始化命令和反馈时序，不模拟
CAN-FD 仲裁、ACK、总线占用率、终端电阻、线束、误码、丢帧或重传。

## 9. 命令、反馈与时序

外部模式保留完整的 RP/Vaxor 语义：显式 enable/disable、位置、速度、力矩相关
命令、Vaxor mode、MIT `kp/kd`、限流、`set-zero` 和状态反馈。未 enable 时电机
effort 始终为零，但连接成功后立即反馈实际关节位置，状态位明确为 disabled。

时序默认值：

- plant 和 effort command：100 Hz；
- RP 反馈：100 Hz；
- Vaxor 反馈：50 Hz；
- 已 enable 电机命令超时：150 ms；
- duplicate-controller 冲突窗口：2 ms。

命令超时按电机独立记录。任意一个已 enable 电机在 150 ms 内没有收到该电机的
有效控制命令，整臂锁存停机；全部 disabled 时可以长期空闲。串口读取线程在帧完整
到达时记录 steady-clock 时间戳，后续 ROS callback 排队不得改写它。

一条串口不能识别物理控制器身份，因此 external duplicate-controller 是明确的启发式：
同一 bus 和 CAN ID 在 2 ms 内到达不同 payload 时锁存冲突。该规则能发现明显竞争，
但不声称能证明总线上只有一个物理控制器。

## 10. 安全和生命周期

任一以下条件触发整臂锁存故障，并将 6 轴 effort 全部置零：

- 任一已 enable 电机命令超时；
- 命令超过 command limit；
- 帧从错误逻辑总线到达；
- duplicate-controller 启发式触发；
- tty 断开或读写生命周期结束；
- effort command topic 出现第二个非预期发布者；
- Gazebo、模型生成、controller spawner 或 motor node 任一关键进程异常退出。

故障锁存后忽略后续控制命令，enable/disable 不能清除。恢复必须重启对应
external-control launch。串口断开或关键进程失败时，节点先发布一次可被
controller 消费的 6 元零 effort，等待有界的 middleware ACK/传播宽限，再让整个
launch fail-fast 退出。不自动重连、自动清错或恢复旧命令。

ROS 与外部控制通过不同 launch 互斥。除 launch 隔离外，motor node 还监视
`/robot_arm/effort_controller/commands` 发布者数量，捕获用户从另一终端误启动
动作播放器的情况。

## 11. Diagnostics

`/robot_arm/diagnostics` 至少发布：

- 当前控制模式和 transport mode；
- 请求的 tty 路径和解析后的实际设备；
- 连接/断开状态；
- 6 个电机的 enable 状态、命令年龄和 timeout 状态；
- 整臂锁存标志和第一个 fault 原因；
- RP/Vaxor 反馈成功、失败和丢弃计数。

一般运行期间可以低频发布汇总，故障状态变化必须立即发布。diagnostics 是
定位控制逻辑的工具，不引入历史数据库或额外 dashboard。

## 12. 测试与验收

### 12.1 共享核心

- RP/Vaxor golden vector、mode、MIT 增益、限流、set-zero 和状态反馈；
- 行帧分片/粘连、错帧恢复、SLCAN 初始化、到达时间和 tty 断开；
- 传动正反换算、direction、zero offset、plant 饱和和 disabled 零 effort；
- 按已 enable 电机独立超时、越限、错总线、重复控制器和故障锁存。

### 12.2 模型契约

- `humanoid_gazebo`：继续校验 30 轴顺序、三总线、模型质量/尺寸和现有 launch；
- `robot_arm_gazebo`：校验严格 6 轴顺序、`arm` 总线、RP 1–3、Vaxor 4–6、
  URDF/controller/YAML 一致、固定基座、初始姿态、mesh 存在和上游版本记录。

### 12.3 PTY 闭环

不要求真实 USB 设备。PTY 至少覆盖：

- 单端口 3 RP + 3 Vaxor 命令、enable 和反馈；
- `usb_serial` 和 `serial_slcan` 两种 transport；
- disabled 时的持续反馈与零 effort；
- 六个已 enable 电机的独立 timeout；
- 越限、duplicate-controller、断线零 effort 和 launch 退出；
- diagnostics 内容和反馈计数。

机械臂只有一个物理端口，收到的帧不携带“来自哪个逻辑总线”的额外字段，
因此错总线无法通过单端口 PTY 构造。该故障由共享 `FrameRouter` 单元测试直接传入
未配置 bus 或其他模型的 bus 标识进行验证；PTY 只验证它在单一 `arm` 端口上能观测的故障。

### 12.4 Gazebo 验收

无 GUI 启动 UR5e，确认模型生成、`joint_state_broadcaster` 和 6 轴 effort controller
活跃，`/robot_arm/joint_states` 严格包含 6 个驱动关节。依次执行 `home`、
`joint_scan`、小幅组合动作和回零；每个目标在有界时间内达到且稳态误差
不大于 0.05 rad，无 NaN、越限、持续 effort 饱和、controller 失败或进程异常退出。

验收不要求 MoveIt、真实 USB/USB-CANFD 或持续运行时长测试。

## 13. 文档与 Git 交付

根 README 改为多包纯 Gazebo 仿真介绍，分别列出人形和 UR5e 入口。`AGENTS.md`
删除“不得新增第二个 package”和固定 `dev/smi` 的过时规则，改为“功能最终合入
`dev-sim`，是否使用任务分支由具体任务决定”。`CONTEXT.md`、包 README、接线说明和
验收记录同步更新。

本任务按用户指定直接在 `dev-sim` 实施。构建和全量测试通过后，只提交源码、
配置、必需的 UR5e 资产、上游许可证、测试和文档；不提交 `build/`、`install/`、
`log/`、IDE 配置或 Python 缓存。最终必要文件直接提交并同步到远程 `dev-sim`。
