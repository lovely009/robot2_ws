# robot_ws

ROS 2 Humble 工作空间：半身双臂机器人的 CAN-FD 电机仿真与真机硬件层。

## 包一览

| 包 | 语言 | 作用 |
|---|---|---|
| `upper_body_canfd_gazebo` | Python | 14-DOF 半身双臂 Gazebo Fortress 仿真：URDF、世界、CAN-FD/vcan 双向桥、键盘遥控、动作播放器，以及真机 ros2_control 的 launch 与控制器配置 |
| `can_motor_sim` | Python | CAN-FD 电机假从站（RP 广播应答 + 25/30 mm 腕部 MIT 应答）与交互式调试 master |
| `upper_body_canfd_hardware` | C++ | 真机 ros2_control 硬件层：`CanFdSystem` SystemInterface 插件，单实例管理左右两条 CAN-FD 总线，导出 14 路 position 命令接口 |

## 分支策略

- `main`：发布分支，只合并经过验证的功能；
- `develop`：开发分支，功能按提交拆分，验证通过后合并到 `main`。

## 常用命令

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

colcon test && colcon test-result --verbose
```

仿真（Gazebo + 双 vcan，xacro 默认 `hardware_backend:=gazebo`）：

```bash
ros2 launch upper_body_canfd_gazebo sim.launch.py
```

真机 bring-up（position 控制器 + CAN-FD 硬件插件）：

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

真机前先把两条物理总线配置为 1 Mbit/s 仲裁、5 Mbit/s 数据（CAN-FD BRS），并逐关节验证零位、方向、限位与急停。

## 文档

- 仿真、协议与动作库：[`upper_body_canfd_gazebo/README.md`](src/upper_body_canfd_gazebo/README.md)
- 真机硬件层架构与参数：[`upper_body_canfd_hardware/README.md`](src/upper_body_canfd_hardware/README.md)
- 电机假从站与调试 master：[`can_motor_sim/README.md`](src/can_motor_sim/README.md)
