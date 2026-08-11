# robot_ws

ROS 2 Humble 工作空间，面向 14 自由度上半身双臂 CAN-FD 控制。当前仓库包含 4 个 ament 包，提供 Gazebo Fortress 仿真与双 SocketCAN 真机控制链路。

## 架构概览

```text
ROS 2 position controller
  -> motor_hardware_interface
  -> motor_canfd_drive
  -> can / vcan
  -> motor_canfd_sim
  -> Gazebo effort controller（仅仿真）
```

| 包 | 语言 | 职责 |
|---|---|---|
| [motor_canfd_drive](src/motor_canfd_drive/README.md) | C++ | 与 ROS 解耦的 SocketCAN/CAN-FD 驱动、协议编解码与协议策略 |
| [motor_hardware_interface](src/motor_hardware_interface/README.md) | C++ | ros2_control 适配层、YAML 解析与 joint 映射 |
| [motor_canfd_sim](src/motor_canfd_sim/README.md) | C++ | 唯一支持的仿真后端，通过 vcan 模拟电机命令与反馈 |
| [upper_body_canfd_gazebo](src/upper_body_canfd_gazebo/README.md) | Python | 机器人模型、Gazebo 集成、launch 编排与参考动作播放器 |

`motor_canfd_sim` 是当前唯一支持的仿真后端；仿真使用 [src/motor_canfd_sim/config/motor_canfd.yaml](src/motor_canfd_sim/config/motor_canfd.yaml)（拓扑来自仿真模型），真机使用 [src/motor_hardware_interface/config/motor_canfd.yaml](src/motor_hardware_interface/config/motor_canfd.yaml)，两侧只共用解析 API，不共用配置文件。

架构决策记录见 [docs/adr/0001-motor-canfd-layering-and-config-single-source.md](docs/adr/0001-motor-canfd-layering-and-config-single-source.md) 与 [docs/adr/0002-sim-topology-config-decoupled-from-hardware.md](docs/adr/0002-sim-topology-config-decoupled-from-hardware.md)。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress（`ros_gz`）
- 支持 CAN-FD 的 Linux SocketCAN；仿真需要 `vcan`

安装依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## 运行方式

### 全链路仿真

在当前机器上，已验证如下命令可启动仿真主流程：

```bash
bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py gui:=false
```

实际启动日志显示：
- Gazebo、`motor_canfd_sim`、`robot_state_publisher` 和基础桥接节点都能启动；
- 驱动侧 `ros2_control_node` 成功加载 `UpperBodyCanFdSystem`，`/driver/controller_manager` 与 `position_controller` 正常就绪；
- 完整回环可用：向 `/driver/position_controller/commands` 发布位置指令后，`/joint_states`（Gazebo）与 `/driver/joint_states`（CAN 反馈）同步跟随。

若只是验证模型与模拟器本身，可继续使用 `model_sim.launch.py` 或 `gazebo.launch.py`。

另开一个终端播放参考动作：

```bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 1
```

### 真机 bring-up

真机启动前必须将两条总线配置为仲裁段 1 Mbit/s、数据段 5 Mbit/s，并启用 CAN-FD BRS。

```bash
ros2 launch motor_hardware_interface hardware.launch.py \
  model_xacro:=src/upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro \
  left_interface:=can0 right_interface:=can1
```

真机 bring-up 入口由 `motor_hardware_interface` 提供（不依赖仿真栈），模型描述来自模型包；
真机部署只需底层驱动 + 适配层 + 模型描述。首次接入时应逐关节确认零位、方向、限位、增益、
反馈超时和硬件急停。当前仿真参数不能直接替代真机标定。

## 配置原则

- [src/motor_hardware_interface/config/motor_canfd.yaml](src/motor_hardware_interface/config/motor_canfd.yaml) 是真机侧总线、电机类型与 joint 映射的单一配置源；仿真侧拓扑在 [src/motor_canfd_sim/config/motor_canfd.yaml](src/motor_canfd_sim/config/motor_canfd.yaml)，来源于仿真模型，不与真机配置耦合。
- 仿真默认使用 `vcan0/vcan1`，真机默认使用 `can0/can1`；`bus_override.<逻辑总线名>` 可在启动时覆盖接口。
- 新增已有类型的电机只修改 YAML；新增电机家族时实现新的协议策略。
- 底层驱动不得写死关节数量、左右臂拓扑或 ROS 2 接口。

## 仓库内容

- `src/`：4 个 ROS 2 包。
- `docs/adr/`：已接受的架构决策。
- `docs/research/`：调研记录，不作为运行规范。
- 协议 PDF：[25–30 mm 驱动与 RP 系列关节 CAN-FD 协议手册](src/motor_canfd_sim/25-30mm驱控一体使用说明手册_20260204.pdf)（位于 `src/motor_canfd_sim/`）。
- `build/`、`install/`、`log/`：colcon 生成目录，不提交 Git。

## 文档入口

- 仿真与 launch 说明：[src/upper_body_canfd_gazebo/README.md](src/upper_body_canfd_gazebo/README.md)
- 底层驱动说明：[src/motor_canfd_drive/README.md](src/motor_canfd_drive/README.md)
- ROS 适配层说明：[src/motor_hardware_interface/README.md](src/motor_hardware_interface/README.md)
- 电机侧模拟器说明：[src/motor_canfd_sim/README.md](src/motor_canfd_sim/README.md)
