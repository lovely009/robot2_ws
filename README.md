# robot_ws

ROS 2 Humble 工作空间：半身双臂机器人的 CAN-FD 电机仿真与真机硬件层。

## 包一览

| 包 | 语言 | 作用 |
|---|---|---|
| `upper_body_canfd_gazebo` | Python | 14-DOF Gazebo Fortress 模型、基础仿真 launch 与 ROS 动作播放器 |
| `motor_canfd_drive` | C++ | 无 ROS 运行时依赖的通用底层驱动：动态总线/电机映射、协议策略注册、SocketCAN |
| `motor_hardware_interface` | C++ | 标准 `ros2_control::SystemInterface` 适配层；参数/映射集中到 YAML |
| `motor_canfd_sim` | C++ | 新的 CAN-FD/Gazebo 电机仿真桥接器，可跑 driver→vcan→sim 全链路 |

`motor_canfd_sim` 是唯一电机仿真后端。仿真命令从 ROS 2 position controller 进入，依次经过真实的 `motor_hardware_interface`、`motor_canfd_drive`、vcan 和 C++ 电机模拟器；仓库不再保留绕过驱动直接发送 CAN-FD 的 Python 仿真路径。

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

仿真（ROS controller + C++ driver + 双 vcan + C++ motor sim + Gazebo 全回环）：

```bash
ros2 launch motor_canfd_sim driver_loopback.launch.py
```

另开终端通过 ROS controller 播放参考动作：

```bash
ros2 run upper_body_canfd_gazebo ros_actions
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

## 开发约定

- README、设计说明和配置文件注释以中文为主，协议名、类名、函数名、ROS 2 API 等标准标识保持英文；
- C++ / Python 的关键路径使用中文注释，重点解释“为什么这样设计”、数据流、单位、协议限制和安全边界；
- 不为显而易见的赋值机械添加逐行注释，避免注释噪声；
- 新增电机类型时，应优先扩展协议策略和 YAML 配置，不要把具体机器人关节数量或左右臂拓扑重新写死到底层驱动中。
