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

验收环境未提供 `/dev/ttyUSB0`，因此没有执行依赖真实串口设备的启动。设备存在性、tty
可打开性、SocketCAN 接口存在性及 SLCAN-FD 显式授权仍由
`robot_arm_motor_sim_node` 在运行时执行权威校验，失败会触发统一 launch 关闭。

## vcan0 真实运行验收

验收时间：2026-08-27 11:33–11:35 CST。主机存在已启用的 `vcan0`，执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=socket_can \
  arm_port:=vcan0
```

实际运行结果：

- Gazebo Fortress、`robot_state_publisher`、`/clock` bridge 和 UR5e entity 启动成功；
- `joint_state_broadcaster` 与 `effort_controller` 依次加载、配置并激活；
- controller 成功后才启动 `/robot_arm/motor_sim`，进程保持运行；
- `ros2 node list` 显示 7 个预期节点：controller manager、两个 controller、
  `gz_ros2_control`、motor sim、state publisher 和 clock bridge；
- `ros2 control list_controllers --controller-manager /robot_arm/controller_manager`
  显示两个 controller 均为 `active`；
- `/robot_arm/joint_states` 可读取完整六轴状态，实测发布频率约 100 Hz；
- `/robot_arm/effort_controller/commands` 实测发布频率约 100 Hz；
- `/robot_arm/diagnostics`、joint states、effort commands、robot description、TF 和 clock
  等预期 topic 均存在；
- 稳定运行约 90 秒未出现节点异常退出，Ctrl-C 后 motor node、bridge、state publisher、
  controller manager 和 Gazebo 均完成有序关闭，未残留相关进程。

Gazebo GUI 输出了 Fortress/QML 的 deprecated connection、binding loop 和组件序列化警告；
这些警告未导致 controller、topic 或数据频率异常。Ctrl-C 通过 SIGINT 停止 Gazebo 时，
launch 将 Gazebo 退出码记录为 `-2`，属于本次人工终止路径，不是运行期崩溃。

`/robot_arm/diagnostics` 当前在 motor node 初始化时发布连接状态；启动后才执行的
`ros2 topic echo --once` 不会收到此前的非持久化消息。本次验收以 motor node 存活、
controller 状态、topic 存在性和 100 Hz 数据流确认后端工作状态。

## vcan0 端到端控制验收

验收时间：2026-08-27 11:43–11:44 CST。重新启动统一 launch 并等待两个 controller
进入 `active` 后，通过 `vcan0` 控制 Vaxor `wrist_1`（motor id 4）：

```bash
# 显式使能，控制帧 ID = 0x502 + 4*4 = 0x512
cansend vcan0 512#0100000000000000

# MIT 命令 ID = 0x501 + 4*4 = 0x511；持续周期小于 150 ms 超时
while true; do
  cansend vcan0 511##0AD276382008050C302000080
  sleep 0.05
done
```

MIT payload 表示电机侧目标位置 120 rad；按 `motors.yaml` 的 80:1 传动比换算为
`wrist_1` 关节侧目标 1.5 rad，速度为 0，电机侧 `kp=0.5`、`kd=0.02`。

实测证据：

- 控制前 `wrist_1 = 2.0405502872 rad`；
- 持续控制约 2 秒后 `wrist_1 = 1.5540688412 rad`，向 1.5 rad 目标收敛；
- 同时 `/robot_arm/effort_controller/commands` 的 wrist_1 分量为
  `-1.6562909462 Nm`，其余未使能轴输出为 0；
- `candump vcan0,711:7FF` 连续收到 motor id 4 的 `0x711` Vaxor reply 帧；
- 控制期间 motor node 和两个 controller 保持运行，未触发故障退出。

这证明链路 `vcan0 命令帧 → Vaxor 解码 → 传动换算 → plant effort → Gazebo 关节 →
Vaxor 反馈帧` 已形成端到端闭环。原 README 中未按当前 Vaxor 位置偏移和 80:1 传动配置
编码的示例 payload 已同步更正。
