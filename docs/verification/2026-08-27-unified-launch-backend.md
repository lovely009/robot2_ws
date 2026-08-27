# robot_arm_gazebo 统一 Launch 仿真后端验收

验收时间：2026-08-27 11:07 CST

## 包边界

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
```

退出码为 0，输出严格为：

```text
motor_sim
robot_arm_gazebo
```

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

退出码为 0；`motor_sim` 和 `robot_arm_gazebo` 两个包均完成构建，总耗时 10.8 s。

## 自动测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

退出码为 0，结果为：

```text
Summary: 103 tests, 0 errors, 0 failures, 0 skipped
```

测试包含统一 launch 的 13 个公开参数、默认连接值、非法 transport、动态 namespace、
参数类型、topic remapping、controller 成功链和 motor node 退出关停契约。自动测试未启动
Gazebo GUI，也不依赖真实串口或 CAN 设备。

## Launch 参数接口

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
```

退出码为 0。输出包含以下全部参数及中文用途说明：

```text
initial_positions
world_file
gz_verbosity
transport_mode
arm_port
baud_rate
allow_external_canfd
fault_latching_enabled
motor_config_file
robot_namespace
joint_states_topic
effort_commands_topic
diagnostics_topic
```

确认默认连接为 `transport_mode=usb_serial`、`arm_port=/dev/ttyUSB0`、
`baud_rate=3000000`，故障锁存默认启用。

## 硬件相关说明

验收环境未提供 `/dev/ttyUSB0`，因此没有执行依赖真实设备的完整运行启动。这符合仓库的
自动测试约束；设备存在性、tty 可打开性、SocketCAN 接口存在性及 SLCAN-FD 显式授权仍由
`robot_arm_motor_sim_node` 在运行时执行权威校验，失败会触发统一 launch 关闭。

