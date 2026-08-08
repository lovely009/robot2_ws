# robot_ws

ROS 2 Humble 工作空间：半身双臂机器人的 CAN-FD 电机仿真与真机硬件层。

## 包一览

| 包 | 语言 | 作用 |
|---|---|---|
| `upper_body_canfd_gazebo` | Python | 原有 14-DOF Gazebo Fortress 仿真、Python CAN-FD bridge、动作与模型；继续保留 |
| `can_motor_sim` | Python | 原有 CAN-FD 假从站与交互式调试 master；继续保留 |
| `motor_canfd_drive` | C++ | 无 ROS 运行时依赖的通用底层驱动：动态总线/电机映射、协议策略注册、SocketCAN |
| `motor_hardware_interface` | C++ | 标准 `ros2_control::SystemInterface` 适配层；参数/映射集中到 YAML |
| `motor_canfd_sim` | C++ | 新的 CAN-FD/Gazebo 电机仿真 bridge，可跑 driver→vcan→sim 全链路 |
| `upper_body_canfd_hardware` | C++ | 旧版一体式硬件层，暂时保留作迁移参考；新的真机入口不再依赖它 |

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

新 C++ bridge（保留同一 Gazebo 模型）：

```bash
ros2 launch motor_canfd_sim gazebo_cpp_sim.launch.py
```

新 C++ driver + vcan + C++ motor sim + Gazebo 全回环：

```bash
ros2 launch motor_canfd_sim driver_loopback.launch.py
```

真机 bring-up（position 控制器 + CAN-FD 硬件插件）：

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

真机前先把两条物理总线配置为 1 Mbit/s 仲裁、5 Mbit/s 数据（CAN-FD BRS），并逐关节验证零位、方向、限位与急停。

## 文档

- 仿真、协议与动作库：[`upper_body_canfd_gazebo/README.md`](src/upper_body_canfd_gazebo/README.md)
- C++ 底层 CAN-FD 驱动：[`motor_canfd_drive/README.md`](src/motor_canfd_drive/README.md)
- `ros2_control` 硬件接口：[`motor_hardware_interface/README.md`](src/motor_hardware_interface/README.md)
- 新 C++ 仿真：[`motor_canfd_sim/README.md`](src/motor_canfd_sim/README.md)
- 旧 Python 电机假从站：[`can_motor_sim/README.md`](src/can_motor_sim/README.md)
- 旧一体式硬件层（迁移参考）：[`upper_body_canfd_hardware/README.md`](src/upper_body_canfd_hardware/README.md)
