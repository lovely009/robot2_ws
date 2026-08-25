# 领域术语与边界

## 核心对象

| 术语 | 含义 |
|---|---|
| 人形模型 | 170 cm、45 kg、躯体经固定支架悬空，双臂 18 轴、双腿 12 轴 |
| 30 轴 ABI | `motors.yaml` 中左臂 9、右臂 9、左腿 6、右腿 6 的稳定顺序 |
| 逻辑总线 | `left_arm`、`right_arm`、`legs`；不是 Linux 设备名 |
| 稳定串口路径 | 仿真主机上的 `/dev/serial/by-id/...`，每条逻辑总线独占一个 |
| RP | 仿真内置的 RP 关节协议策略，共 20 个实例 |
| Vaxor | 配置名 `motor_2530` 的小型电机协议策略，共 10 个实例 |
| 电机侧状态 | 电机协议所见的位置、速度、电流或力矩；Vaxor 需经过传动换算 |
| 关节侧状态 | Gazebo 的 position、velocity、effort，按 30 轴 ABI 排列 |
| 帧记录 | 串口承载的 ASCII CAN/CAN-FD 记录，支持分片接收与逐记录容错 |

## 两种外部连接模式

- `usb_serial`：外部设备和仿真主机通过 USB CDC/UART 交换帧记录。这里只模拟协议端点，
  不存在物理 CAN 总线行为。
- `serial_slcan`：仿真主机通过 USB-CANFD 适配器进入物理 CAN-FD 链路，串口侧使用
  SLCAN-FD 记录。启动时配置仲裁 1 Mbit/s、数据 5 Mbit/s 和 BRS。

两种模式互斥，均由同一个 `humanoid_motor_sim_node` 消费。外部控制器可以是独立
3588，也可以是带 CAN-FD 的单片机；本仓库不实现这些控制器的软件。

## 控制与安全术语

- `FrameRouter`：按逻辑总线、协议和电机 ID 路由命令，并把关节侧状态编码为反馈。
- `Transmission`：Vaxor 的电机侧与关节侧位置、速度、力矩换算。
- `MotorPlantModel`：把已启用且有效的电机命令变为仿真关节 effort。
- 显式 enable：启动后默认不驱动，必须先收到协议定义的使能命令。
- 锁存故障：命令越限、150 ms 超时、错总线、2 ms 内重复控制器或串口断开后，
  全部轴输出零 effort，不能靠后续普通命令自动恢复。
- 动作验收工具：直接读取 `/joint_states` 并向 `/effort_controller/commands` 发布有界
  PD effort；它不代表外部电机链路。

所有配置中的电气、传动、增益、限幅和惯量值均为仿名义值。
