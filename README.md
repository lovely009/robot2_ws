# robot_ws

面向 ROS 2 Humble 的 14 自由度上半身双臂 CAN-FD 控制工作空间，支持 Gazebo Fortress 全链路仿真与双 SocketCAN 真机控制。

## 架构

```text
ROS 2 position controller
  -> motor_hardware_interface
  -> motor_canfd_drive
  -> can / vcan
  -> 真实电机 / motor_canfd_sim
  -> Gazebo effort controller（仅仿真）
```

| 包 | 构建类型 | 职责 |
|---|---|---|
| [`motor_canfd_drive`](src/motor_canfd_drive/README.md) | `ament_cmake` | ROS 无关的 SocketCAN、协议编解码和电机协议策略 |
| [`motor_hardware_interface`](src/motor_hardware_interface/README.md) | `ament_cmake` | `ros2_control::SystemInterface`、YAML 解析和关节映射 |
| [`motor_canfd_sim`](src/motor_canfd_sim/README.md) | `ament_cmake` | 通过 vcan 模拟 RP 与 25/30 mm CAN-FD 电机 |
| [`upper_body_canfd_gazebo`](src/upper_body_canfd_gazebo/README.md) | `ament_python` | 机器人模型、Gazebo 集成、launch 编排和参考动作 |

`motor_canfd_sim` 是唯一支持的电机仿真后端。仿真和真机共用
`motor_hardware_interface/config/motor_canfd.yaml`，不维护绕过底层驱动的 CAN 命令路径。

架构决策见 [`docs/adr/0001-motor-canfd-layering-and-config-single-source.md`](docs/adr/0001-motor-canfd-layering-and-config-single-source.md)。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress（`ros_gz`）
- 支持 CAN-FD 的 Linux SocketCAN；仿真需要 `vcan`

安装工作空间依赖：

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

## 运行

### 全链路仿真

```bash
bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py
```

另开终端播放参考动作：

```bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 1
```

### 真机

真机启动前必须将两条总线配置为仲裁段 1 Mbit/s、数据段 5 Mbit/s，并启用 CAN-FD BRS。

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

首次接入必须逐关节验证零位、方向、限位、增益、反馈超时和硬件急停。当前
`motor_2530` 的扭矩常数是仿真参数，不能直接用于真机标定。

## 配置原则

- `motor_canfd.yaml` 是总线、电机类型及 joint 映射的单一配置源。
- 仿真通过 `bus_override.<逻辑总线名>` 将 `can0/can1` 重定向到 `vcan0/vcan1`。
- 新增已有类型的电机只修改 YAML；新增电机家族时实现新的协议策略。
- 底层驱动不得写死关节数量、左右臂拓扑或 ROS 2 接口。

## 仓库内容

- `src/`：4 个 ROS 2 包。
- `docs/adr/`：已接受的架构决策。
- `docs/research/`：调研记录，不作为运行规范。
- 根目录 PDF：电机与关节 CAN-FD 协议手册。
- `build/`、`install/`、`log/`：colcon 生成目录，不提交 Git。

## 分支

- `develop`：日常开发与功能集成。
- `main`：经过验证的发布版本。
