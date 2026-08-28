# 领域术语与边界

## 核心对象

| 术语 | 含义 |
|---|---|
| UR5e 模型 | 六轴 UR5e 机械臂，基座经 `world` 固定链接固定于 Gazebo 场景 |
| 六轴 ABI | `robot_arm_gazebo/config/motors.yaml` 中 shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3 的稳定顺序 |
| 逻辑总线 | `arm`；不是 Linux 设备名，当前仅一条 |
| 稳定串口路径 | 仿真主机上的 `/dev/serial/by-id/...`，`arm` 总线独占一个 |
| CERB | 配置名 `cerb` 的巨蟹关节模组协议策略（MIT 模式子集），本模型共 3 个实例（id 1–3） |
| Vaxor | 配置名 `vaxor` 的小型电机协议策略，本模型共 3 个实例（id 4–6） |
| 电机侧状态 | 电机协议所见的位置、速度、电流或力矩；Vaxor 需经过传动换算 |
| 关节侧状态 | Gazebo 的 position、velocity、effort，按六轴 ABI 排列 |
| 帧记录 | 串口承载的 ASCII CAN/CAN-FD 记录，支持分片接收与逐记录容错 |

## 包边界

- `motor_sim`：与机器人无关的共享库。protocol、transport、core、ros 四层，
  不包含任何具体机器人的拓扑或模型知识；拓扑契约由使用方传入。
- `robot_arm_gazebo`：UR5e 模型、Gazebo 集成与拓扑契约的唯一持有方；
  `config/motors.yaml` 是电机仿真配置的单一来源。

## 三种外部连接模式

`robot_arm_gazebo/launch/gazebo.launch.py` 是完整仿真后端的唯一启动入口；它统一编排
Gazebo、UR5e、controller、时钟桥和电机协议节点。外部工程通过 launch 参数选择
transport、设备、namespace 和 ROS topic，不单独启动 `robot_arm_motor_sim_node`。
默认连接为 `socket_can` 和 `vcan0`，且 `fault_latching_enabled:=false`，仅用于本机联调；
正式仿真验收应使用 `usb_serial` 或 `serial_slcan` 并保留 `fault_latching_enabled:=true`。
Gazebo 启动时保持暂停，依次配置 joint state、外部 effort 与闭环抱闸 controller；
抱闸 controller 成功激活后才恢复物理仿真，消除控制接管前的重力下坠窗口。

- `usb_serial`：外部设备和仿真主机通过 USB CDC/UART 交换帧记录。这里只模拟协议端点，
  不存在物理 CAN 总线行为。
- `serial_slcan`：仿真主机通过 USB-CANFD 适配器进入物理 CAN-FD 链路，串口侧使用
  SLCAN-FD 记录。启动时配置仲裁 1 Mbit/s、数据 5 Mbit/s 和 BRS。
- `socket_can`：仿真主机直接绑定 Linux SocketCAN 接口（如 `vcan0` 虚拟接口），
  用于本机 vcan 人工联调；不经串口，也没有适配器初始化序列。

三种模式互斥，均由同一个 `robot_arm_motor_sim_node` 消费。外部控制器可以是独立
上位机，也可以是带 CAN-FD 的单片机；本仓库不实现这些控制器的软件。

## 控制与安全术语

- `FrameRouter`：按逻辑总线、协议和电机 ID 路由命令，并把关节侧状态编码为反馈。
- `Transmission`：Vaxor 的电机侧与关节侧位置、速度、力矩换算。
- `MotorPlantModel`：把已启用且有效的电机命令变为仿真关节 effort。
- 显式 enable：启动后默认不驱动，必须先收到协议定义的使能命令。
- 驱动生命周期：`brake_engaged`、`awaiting_first_command`、`ready_for_control`、
  `motor_control_active`、`fault_latched`。只有六轴全部使能且命令新鲜，才允许控制 gate。
- 仿真抱闸：`brake_controller` 使用 effort 闭环保持姿态，与外部
  `effort_controller` 通过 Controller Manager STRICT 原子互斥切换。
- 锁存故障：命令越限、150 ms 超时、错总线、2 ms 内重复控制器或串口断开后，
  全部轴输出零 effort，不能靠后续普通命令自动恢复。
- 调试开关：`fault_latching_enabled:=false` 时故障仍清零并抱闸，但不锁存；全轴
  disable 后可重新 enable。默认 `true` 保持锁存和显式复位/重启语义。
- 动作验收工具：`scripts/robot_arm_actions` 直接读取 `/robot_arm/joint_states` 并向
  effort 控制器发布有界 PD effort；它不代表外部电机链路。

所有配置中的电气、传动、增益、限幅和惯量值均为仿名义值。
