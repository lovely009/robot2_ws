# humanoid_gazebo

仓库唯一的 ROS 2 Humble 包，集成 30 轴人形 URDF、Gazebo Fortress、RP/Vaxor
协议、三路串口传输、电机仿真、安全监控和模型动作验收工具。

## 包内模块

| 模块 | 职责 |
|---|---|
| `humanoid_protocol` | RP、Vaxor 命令/反馈帧编解码 |
| `humanoid_transport` | USB 串口、SLCAN-FD 帧记录与非阻塞 tty 生命周期 |
| `humanoid_motor_sim_core` | YAML 配置、传动换算、plant、路由和锁存安全状态 |
| `humanoid_motor_sim_node` | 三路串口、Gazebo joint state、effort controller 的 ROS 连接层 |
| Python `motion_library` | 30 轴静态动作与肩肱节律 |
| Python `ros_action_player` | 仅面向 Gazebo 的有界 joint-space PD 播放器 |

## 30 轴拓扑

稳定顺序定义在 [`config/motors.yaml`](config/motors.yaml)：

1. 左臂 9 轴；
2. 右臂 9 轴；
3. 左腿 6 轴；
4. 右腿 6 轴。

每条手臂由 6 个 RP 和 3 个 Vaxor 驱动；双腿合用 `legs` 总线，由 8 个 RP 和
4 个 Vaxor 驱动。URDF、`ros2_control`、controller YAML、动作库和电机配置必须保持
完全相同的 30 轴顺序，静态测试会检查这一契约。

## 配置

- `config/motors.yaml`：协议、ID、逻辑总线、关节映射、限位、传动与仿名义 plant 参数。
- `config/mechanics.yaml`：质量和几何参数；动态连杆总质量严格为 45 kg。
- `config/controllers.yaml`：100 Hz controller manager 与 30 轴 effort controller。
- `worlds/humanoid_world.sdf`：1 kHz 目标物理步长的空世界。

默认时序为：控制 100 Hz、RP 反馈 100 Hz、Vaxor 反馈 50 Hz、命令超时 150 ms。

## Launch

```bash
# 只运行模型
ros2 launch humanoid_gazebo gazebo.launch.py gui:=false

# 原始 USB 串口帧记录
ros2 launch humanoid_gazebo usb_sim.launch.py \
  left_arm_port:=/dev/serial/by-id/... \
  right_arm_port:=/dev/serial/by-id/... \
  legs_port:=/dev/serial/by-id/...

# USB-CANFD，必须显式授权
ros2 launch humanoid_gazebo canfd_sim.launch.py \
  allow_external_canfd:=true \
  left_arm_port:=/dev/serial/by-id/... \
  right_arm_port:=/dev/serial/by-id/... \
  legs_port:=/dev/serial/by-id/...
```

端口为空、不是稳定路径或出现重复时，launch 会在创建节点前拒绝启动。任一路运行中
断开时，节点输出 30 个零 effort、发布 diagnostics 并关闭整个 launch。

## 动作验收

先只启动 `gazebo.launch.py`，再在另一终端运行：

```bash
ros2 run humanoid_gazebo ros_actions --list
ros2 run humanoid_gazebo ros_actions --action mirror_check
```

不要在 `usb_sim.launch.py` 或 `canfd_sim.launch.py` 运行时启动动作工具，否则两个发布者
会竞争同一个 effort command topic。

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select humanoid_gazebo
source install/setup.bash
colcon test --packages-select humanoid_gazebo
colcon test-result --verbose
```

GoogleTest 覆盖协议 golden vector、帧分片、PTY 串口、配置、传动、plant、路由和安全；
pytest 覆盖模型质量/尺寸、30 轴拓扑、launch 契约和动作工具。测试不要求真实 USB 设备。

外部设备接法见 [`docs/usb-wiring.md`](docs/usb-wiring.md)，协议原始手册位于
`docs/protocol/`。

本次软件、PTY、Gazebo 与 GUI 验收证据见
[`docs/verification/2026-08-25-humanoid-gazebo.md`](../../docs/verification/2026-08-25-humanoid-gazebo.md)。
