# 单包 30 轴人形机器人 Gazebo 仿真设计

## 1. 产品边界

本仓库只实现 Gazebo 仿真，并且最终只安装一个 ROS 2 包：`humanoid_gazebo`。

仿真对象是一台 170 cm、45 kg、30 个驱动关节的工业仿人机器人。躯干通过 fixed joint 悬空固定，初始姿态下脚底离地 10 cm；双腿只做自由空间运动，不承担站立、平衡或步态任务。外部 RK3588 或 STM32 通过 USB 串口或 USB 转 CAN-FD 串口发送 RP/Vaxor 帧，仿真端模拟电机响应并驱动 Gazebo。

仓库不再提供独立电机驱动包、独立电机模拟包、`motor_hardware_interface`、真机 bring-up、本机 driver/vcan 回环或外部 ros2_control driver。Gazebo 内部仍使用 `gz_ros2_control` effort controller，但它只属于仿真内部实现。

不修改、不导入外部 RK3588/STM32 固件工程，也不提交本地未跟踪的 `ros2_ws/`。

## 2. 单包内部结构

`humanoid_gazebo` 是唯一 ament package，但内部保持深模块边界：

```text
src/humanoid_gazebo/
├── include/humanoid_gazebo/
│   ├── protocol/          # RP/Vaxor字段与帧策略
│   ├── transport/         # 串口、帧记录和端点生命周期
│   └── motor_sim/         # 配置、传动换算、plant、安全、指标
├── src/
│   ├── protocol/
│   ├── transport/
│   ├── motor_sim/
│   └── motor_sim_node.cpp
├── config/                # 30轴电机与机械参数
├── urdf/                  # 人形模型与内部ros2_control
├── worlds/
├── launch/
├── humanoid_gazebo/       # Python launch helpers、动作和验收工具
├── docs/protocol/         # RP、25/30 mm协议PDF
└── test/
```

CMake 内部目标：

- `humanoid_protocol`：无 ROS，负责 `CanFrame`、RP/Vaxor协议编解码和ID规则；
- `humanoid_transport`：无 ROS，负责 POSIX tty、USB帧记录、CANable SLCAN-FD；
- `humanoid_motor_sim_core`：无 ROS，负责 YAML 配置数据模型、传动换算、plant、安全状态机和运行指标；
- `humanoid_motor_sim_node`：ROS 2 node，路由三条串口总线、订阅Gazebo joint states并发布effort命令/诊断。

前三个库只在包内使用，不作为独立 ament package 或兼容层导出。

## 3. 两种外部传输

两种模式都要求三条逻辑总线各有一个稳定 tty，launch 参数必须使用 `/dev/serial/by-id/...`，不得根据 `/dev/ttyACM*` 枚举顺序自动猜测。

### 3.1 `usb_serial`

纯 USB CDC/UART 点对点串口，不经过 CAN 电气层。每条 tty 使用与 SLCAN-FD 数据帧相同的行记录格式，以 `\r` 结束：

- `t`/`T`：经典 CAN 标准/扩展帧；
- `d`/`D`：CAN-FD 标准/扩展帧；
- `b`/`B`：CAN-FD+BRS 标准/扩展帧。

该 transport 只读写数据帧记录，不发送 `C/S8/Y5/O` 配置命令，也不模拟 CAN 仲裁、ACK、错误帧或 BRS 电气行为。外部设备必须实现同一串口帧记录协议。仓库只消费已经出现在 Linux 中的 tty，不负责把仿真主机配置成 USB Gadget。

### 3.2 `serial_slcan`

USB 转 CAN-FD（首选 CANable2 SLCAN-FD 固件）。transport 打开 tty 后依次发送：

```text
C → S8 → Y5 → O
```

含义为关闭通道、仲裁段 1 Mbit/s、数据段 5 Mbit/s、打开通道。数据帧仍使用 `t/T/d/D/b/B` 行记录格式。每条真实 CAN-FD 总线由外部控制端和仿真主机侧 USB-CANFD 构成，必须正确连接 CANH/CANL/GND 与终端电阻。

两种 transport 统一实现内部 `FrameTransport` 接口：

```cpp
class FrameTransport {
public:
  virtual ~FrameTransport() = default;
  virtual void open() = 0;
  virtual void close() noexcept = 0;
  virtual bool send(const CanFrame &frame) = 0;
  virtual ReceiveResult receive(CanFrame &frame) = 0;
};
```

`ReceiveResult` 必须区分 `Frame`、`NoData` 和 `Disconnected`，避免把拔线误当成暂时无帧。

## 4. 30 轴拓扑

坐标采用 REP-103：`x` 向前、`y` 向左、`z` 向上。零位为双臂自然下垂、掌面朝向大腿、双腿伸直、脚尖向前。左右镜像关节使用相同解剖语义，左右外展等动作的命令符号一致。

每侧手臂 9 轴：

| 顺序 | 关节 | 电机 |
|---:|---|---|
| 1 | scapula_protraction | RP |
| 2 | scapula_elevation | RP |
| 3 | shoulder_pitch | RP |
| 4 | shoulder_roll | RP |
| 5 | shoulder_yaw | RP |
| 6 | elbow_pitch | RP |
| 7 | forearm_roll | Vaxor (`motor_2530`) |
| 8 | wrist_flexion | Vaxor (`motor_2530`) |
| 9 | wrist_deviation | Vaxor (`motor_2530`) |

每侧腿 6 轴：

| 顺序 | 关节 | 电机 |
|---:|---|---|
| 1 | hip_yaw | RP |
| 2 | hip_roll | RP |
| 3 | hip_pitch | RP |
| 4 | knee_pitch | RP |
| 5 | ankle_pitch | Vaxor (`motor_2530`) |
| 6 | ankle_roll | Vaxor (`motor_2530`) |

整机共 RP 20 台、Vaxor 10 台。三条总线：

- `left_arm`：RP 1–6，Vaxor 1–3；
- `right_arm`：RP 1–6，Vaxor 1–3；
- `legs`：左腿 RP 1–4、右腿 RP 5–8；左踝 Vaxor 1–2、右踝 Vaxor 3–4。

## 5. 机械与执行器参数

固定支架不计入45 kg。质量预算：pelvis 6 kg、abdomen 4 kg、chest 8 kg、head 3 kg；每臂4.8 kg；每腿7.2 kg。脚底 `z=0.10 m`，髋中心约 `z=1.04 m`，头顶 `z=1.80 m`，本体从脚底到头顶为1.70 m。

关节 hard limit（度）：

| 关节 | 范围 |
|---|---:|
| scapula_protraction | -25..25 |
| scapula_elevation | -15..30 |
| shoulder_pitch | -60..150 |
| shoulder_roll | -30..150 |
| shoulder_yaw | -90..90 |
| elbow_pitch | 0..145 |
| forearm_roll | -90..90 |
| wrist_flexion | -70..70 |
| wrist_deviation | -25..25 |
| hip_yaw | -45..45 |
| hip_roll | -35..35 |
| hip_pitch | -30..120 |
| knee_pitch | 0..135 |
| ankle_pitch | -40..30 |
| ankle_roll | -25..25 |

command limit 从 hard limit 两端各收缩5°。机械 YAML 是 link质量/质心/惯量、joint origin/axis/limit/damping/friction 的单一来源；电机 YAML 是 joint顺序、总线、ID、传动和plant参数的单一来源，跨源测试确保30个joint一一对应。

RP按关节侧量处理，名义 ratio=1。Vaxor按手册的减速前电机侧位置/速度处理，forearm/wrist名义 ratio=36，ankle名义 ratio=80，效率0.80，Kt=0.012 Nm/A。这些值明确标记为仿名义值，不代表厂家规格，不用于预测温升、能耗或极限负载。

## 6. Gazebo 数据流

```text
外部RK3588或STM32
  → 三路 usb_serial 或 serial_slcan
  → humanoid_motor_sim_node
  → 协议解码 / 传动换算 / plant
  → /effort_controller/commands
  → Gazebo 30轴模型
  → /joint_states
  → 传动反算 / RP与Vaxor反馈编码
  → 三路串口返回外部控制器
```

仿真内部 effort controller 更新100 Hz，Gazebo physics目标1 kHz，RP反馈100 Hz，Vaxor状态50 Hz，command timeout 150 ms。外部控制器使用真实单调时钟；Gazebo `/clock` 只供只读监控。

包提供三个入口：

- `gazebo.launch.py`：只启动模型、world和Gazebo内部controller；
- `usb_sim.launch.py`：Gazebo + 三路纯USB串口虚拟电机；
- `canfd_sim.launch.py`：Gazebo + 三路CANable2 SLCAN-FD虚拟电机。

不提供 vcan、SocketCAN 或本机 driver loopback。

## 7. 安全语义

- 三个端点必须非空、稳定且互不重复；任一路打不开则整机不启动；
- 任一joint命令越过command limit，拒绝该控制批次并锁存整机卸力；
- 150 ms没有有效命令，全部effort归零；
- 任一必需tty断开，清空缓存、卸力并fail-fast退出；
- 重连后不得自动使能，必须重启或显式enable；
- 同一bus/CAN ID在2 ms内收到payload不同的重复控制帧，锁存duplicate-controller错误；
- `serial_slcan` 默认拒绝启动，必须显式 `allow_external_canfd:=true`，避免在连接真实电机的总线上冒充相同ID；
- `usb_serial` 与 `serial_slcan` 不能在同一进程混用，避免一个机器人出现两个控制源。

## 8. 动作、诊断与验收

30台电机始终独立。肩胛协调器只位于参考动作层、默认关闭：抬臂30°以内肩胛不动，超过后 `scapula_elevation=0.25*(raise_angle-30°)`；前举时 `scapula_protraction=0.12*(shoulder_pitch-30°)`，最后裁剪到command limit。它是调试用经验曲线，不是医学模型。

自动化测试覆盖：协议编解码、USB记录、CANable初始化、30轴/ID契约、传动正反换算、逐电机plant参数、command limit、超时、断线、重复控制源、45 kg/惯量/镜像轴、launch三端点和动作库。

运行验收包括30轴逐关节扫描、肩胛协调抬臂、手臂组合动作、双腿悬空动作、左右镜像、非法命令、超时和拔线。无GUI目标：RTF≥0.95、100 Hz连续30分钟无反馈超时、99%控制周期抖动≤±2 ms、三路无跨总线路由错误。

最终交付代码、全量构建/测试证据、USB与USB-CANFD接线说明、运行报告和Gazebo截图。硬件不在当前环境时必须明确标记对应验收未执行，不能宣称通过。
