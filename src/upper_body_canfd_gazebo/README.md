# 半身双臂 Gazebo 模型与 ROS 动作工具

本包提供 14 自由度半身双臂机器人的 URDF/Xacro、Gazebo Fortress 世界、Gazebo 侧 controller 配置、真机 bring-up 和 ROS 参考动作播放器。

## 架构边界

本包不实现电机仿真，也不直接向 vcan 发送电机命令。`motor_canfd_sim` 是工作空间唯一的电机仿真后端；仿真使用与真机一致的控制路径：

```text
ROS 2 position command
  -> /driver/position_controller
  -> motor_hardware_interface
  -> motor_canfd_drive
  -> vcan0 / vcan1
  -> motor_canfd_sim
  -> Gazebo effort controller
  -> /joint_states
  -> motor_canfd_sim CAN-FD feedback
  -> motor_canfd_drive
```

`launch/sim.launch.py` 只负责 Gazebo 世界、机器人模型、`joint_state_broadcaster`、`effort_controller` 和 `/clock`，由 `motor_canfd_sim/launch/driver_loopback.launch.py` 作为基础 launch 引入。

## 机器人结构

每条手臂 7 DoF：

| 本地编号 | 关节 | 电机类型 |
|---|---|---|
| RP ID 1 | shoulder_yaw | 意优 RP |
| RP ID 2 | shoulder_roll | 意优 RP |
| RP ID 3 | shoulder_pitch | 意优 RP |
| RP ID 4 | elbow_pitch | 意优 RP |
| N=1 | wrist_yaw | 25/30 mm 驱控一体 |
| N=2 | wrist_pitch | 25/30 mm 驱控一体 |
| N=3 | wrist_roll | 25/30 mm 驱控一体 |

左右臂分别使用独立总线；仿真为 `vcan0/vcan1`，真机默认为 `can0/can1`。

## 环境与构建

目标环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions \
  can-utils

cd ~/robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 启动仿真

先创建双 vcan：

```bash
cd ~/robot_ws/src/upper_body_canfd_gazebo
bash scripts/setup_vcan.sh
```

启动完整回环：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_canfd_sim driver_loopback.launch.py
```

仿真包含两个 controller manager：Gazebo 的 `/controller_manager` 接收 effort；驱动侧 `/driver/controller_manager` 加载真实 `motor_hardware_interface` 并接收 position command。

## 通过 ROS 播放参考动作

另开终端：

```bash
source ~/robot_ws/install/setup.bash
ros2 run upper_body_canfd_gazebo ros_actions
```

动作播放器从 `/driver/joint_states` 获取当前姿态，并向 `/driver/position_controller/commands` 发布 14 关节位置数组。它不打开 SocketCAN，也不编码 CAN-FD 帧。

常用参数：

```bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 8
ros2 run upper_body_canfd_gazebo ros_actions --speed-scale 0.5
```

关键帧之间使用三次 smoothstep 插值：

```text
s(u) = 3u² - 2u³
```

默认 100 Hz 发布目标，每段起止速度为零。

## 真机启动

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

真机总线目标配置为 1 Mbit/s 仲裁、5 Mbit/s 数据段并启用 CAN-FD BRS。接入真机前必须逐关节确认零位、方向、限位、增益和硬件急停；腕部扭矩常数也必须按实际型号标定。

## 测试

```bash
colcon test
colcon test-result --verbose
```

当前 Python 测试覆盖动作编号、关键帧限位、smoothstep 插值和 ROS position controller 命令顺序；CAN-FD 协议测试由 `motor_canfd_drive` 的 C++ GTest 负责。

## 主要文件

```text
launch/sim.launch.py                 Gazebo 基础仿真
launch/hardware.launch.py            真机 ros2_control bring-up
urdf/upper_body.urdf.xacro           14-DoF 模型与硬件后端选择
worlds/upper_body_world.sdf          Gazebo Fortress 世界
config/controllers.yaml              Gazebo effort controller
config/hardware_controllers.yaml     真机 position controller
upper_body_canfd_gazebo/motion_library.py
upper_body_canfd_gazebo/ros_action_player.py
scripts/setup_vcan.sh
```
