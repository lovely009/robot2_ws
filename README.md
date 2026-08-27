# UR5e 机械臂 Gazebo 电机仿真

本仓库包含两个 ROS 2 包，在 Gazebo Fortress 中模拟一台基座固定于 world 的
UR5e 六轴机械臂，并通过 USB 串口接收外部控制器的 RP / Vaxor 电机协议帧。
仓库不包含真机驱动、真机 bring-up 或本机 CAN 回环。

## 包结构

| 包 | 职责 |
|---|---|
| `motor_sim` | 与具体机器人无关的共享仿真库：RP/Vaxor/CERB 协议、串口与 SocketCAN 传输、传动换算、plant 模型、帧路由与锁存安全状态、ROS 连接节点 |
| `robot_arm_gazebo` | UR5e 模型与 Gazebo 集成：URDF、world、controller 配置、`arm` 单总线拓扑契约和 `robot_arm_motor_sim_node` 运行入口 |

## 机器人与数据流

- 六轴顺序以 `robot_arm_gazebo/config/motors.yaml` 为准：shoulder_pan、
  shoulder_lift、elbow 为 RP（id 1–3），wrist_1、wrist_2、wrist_3 为
  Vaxor（`motor_2530`，id 4–6）。
- 逻辑总线只有 `arm` 一条，绑定一个唯一的 `/dev/serial/by-id/...` 设备。
- 外部帧经 `motor_sim` 的协议、串口、传动与电机模型转换为 Gazebo effort；
  关节状态再编码成电机反馈帧返回外部控制器。
- `config/motors.yaml` 是电机仿真配置的唯一来源；所有电气、传动、增益和
  限幅值均为仿名义值，不代表 UR、RP 或 Vaxor 真机规格。

## 环境与构建

目标环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

`colcon list --names-only` 应只输出 `motor_sim` 和 `robot_arm_gazebo`。

## 启动

`gazebo.launch.py` 是完整仿真后端的唯一启动入口。默认使用
`usb_serial`、`/dev/ttyUSB0` 和 3,000,000 baud，一条命令同时启动 Gazebo、
UR5e、控制器、时钟桥和电机协议仿真节点：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```

设备不存在、不可打开或参数不合法时，整套后端会快速失败并关闭，避免留下无法接入的
Gazebo 场景。查看全部参数、默认值和中文用途说明：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
```

所有参数都可在 launch 命令中临时覆盖。例如使用常见的 ACM 串口并修改初始姿态：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py \
  arm_port:=/dev/ttyACM0 \
  initial_positions:=0,-1.57,0,-1.57,0,0
```

- `transport_mode` 只有 `usb_serial`、`serial_slcan` 和 `socket_can` 三种且互斥；
  选择 `serial_slcan` 时必须显式设置 `allow_external_canfd:=true`；
  `socket_can` 的端口是 SocketCAN 接口名（如 `vcan0`），不是设备路径。
- 启动后电机默认不使能，必须先收到协议定义的使能命令。
- 命令超时 150 ms、越限、错总线、2 ms 内重复控制器或传输断开都会锁存故障，
  并把全部关节输出置零，不能靠后续普通命令自动恢复。
- 调试时可显式设置 `fault_latching_enabled:=false` 关闭故障锁存（只记录
  diagnostics 不停机、不清零输出）；默认 `true` 保持完整安全语义，仅建议
  本机 vcan/PTY 联调时使用。
- `scripts/robot_arm_actions` 动作播放器只用于模型验收，直接读取
  `/robot_arm/joint_states` 并发布有界 PD effort，不代表外部电机链路。

### 用 vcan 本机人工联调

不需要 Gazebo 和真实串口，用 Linux 虚拟 CAN 接口即可手动测试协议链路：

```bash
# 一次性创建虚拟 CAN 口（需要 sudo）
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 终端 1：启动完整仿真后端并切换到 vcan0
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=socket_can \
  arm_port:=vcan0

# 终端 2：观察反馈帧（wrist_1 回复 0x711，状态帧 0x712 每 20 ms 一帧）
candump vcan0
```

以 wrist_1（N=4）为例，用 `can-utils` 注入命令：

```bash
# 使能电机（控制帧 0x502+4N）
cansend vcan0 512#0100000000000000

# 持续下发 wrist_1 的 MIT 目标：关节侧 1.5 rad、速度 0；
# payload 已按 motors.yaml 的 80:1 传动比编码，kp=0.5、kd=0.02。
# 否则使能后 150 ms 即触发超时锁存，需重启节点。
while true; do cansend vcan0 511##0AD276382008050C302000080; sleep 0.05; done
```

上述固定 payload 只对应当前 `motors.yaml` 中 `wrist_1` 的 80:1 传动配置。修改传动比、
方向或零点后，外部控制器必须重新执行关节侧与电机侧换算，不能继续复用该字节串。

也可以伪造关节运动观察反馈位置变化：

```bash
ros2 topic pub -r 50 /robot_arm/joint_states sensor_msgs/msg/JointState \
  "{name: [wrist_1_joint], position: [0.3], velocity: [0.0]}"
```

统一 launch 的默认接口位于 `/robot_arm` namespace，故障与状态可通过
`ros2 topic echo /robot_arm/diagnostics` 观察。

使用外部 SLCAN-FD 适配器时必须显式授权：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=serial_slcan \
  arm_port:=/dev/ttyUSB0 \
  allow_external_canfd:=true
```

本机 PTY/vcan 调试时可临时关闭锁存；不要将该选项用于正式仿真验收：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=socket_can \
  arm_port:=vcan0 \
  fault_latching_enabled:=false
```
