# robot_ws

ROS 2 Humble 工作空间：半身双臂机器人的 CAN-FD 电机仿真与真机硬件层。

## 包一览

| 包 | 语言 | 作用 |
|---|---|---|
| `upper_body_canfd_gazebo` | Python | 14-DOF 半身双臂 Gazebo Fortress 仿真：URDF、世界、CAN-FD/vcan 双向桥、键盘遥控、动作播放器 |
| `can_motor_sim` | Python | CAN-FD 电机假从站（RP 广播应答 + 25/30 mm 腕部 MIT 应答）与交互式调试 master |
| `upper_body_canfd_hardware` | C++ | 真机 ros2_control 硬件层：RP MIT / 2530 腕部协议移植 + `CanFdMotorSystem` SystemInterface 插件 |

## 常用命令

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

colcon test && colcon test-result --verbose
```

仿真（Gazebo + 双 vcan）：

```bash
ros2 launch upper_body_canfd_gazebo sim.launch.py
```

无 Gazebo 的全链路验证（真硬件插件 + vcan 假从站 + ros2_control）：

```bash
ros2 launch upper_body_canfd_gazebo hardware_sim.launch.py
```

真机 bring-up（显式放行 can0/can1）：

```bash
ros2 launch upper_body_canfd_gazebo real_hardware.launch.py \
  left_interface:=can0 right_interface:=can1 allow_hardware:=true
```

## 文档

- 仿真与协议细节：[`upper_body_canfd_gazebo/README.md`](src/upper_body_canfd_gazebo/README.md)
- 硬件层架构与参数：[`upper_body_canfd_hardware/README.md`](src/upper_body_canfd_hardware/README.md)
- 电机假从站：[`can_motor_sim/README.md`](src/can_motor_sim/README.md)
