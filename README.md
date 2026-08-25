# 30 轴人形机器人 Gazebo 仿真

本仓库只包含一个 ROS 2 包：`humanoid_gazebo`。它在 Gazebo Fortress 中模拟一台
170 cm、45 kg、躯体悬空固定的人形机器人，并通过三路 USB 串口接收外部控制器的
电机协议帧。仓库不包含真机驱动、真机 bring-up 或本机 CAN 回环。

## 机器人与数据流

- 双臂各 9 轴：肩胛前伸、肩胛上提、肩 pitch/roll/yaw、肘 pitch、前臂 roll、
  腕 flexion/deviation。
- 双腿各 6 轴：髋 yaw/roll/pitch、膝 pitch、踝 pitch/roll。
- 共 30 轴、20 个 RP、10 个 Vaxor（`motor_2530`）。
- 逻辑总线固定为 `left_arm`、`right_arm`、`legs`，每条总线绑定一个唯一的
  `/dev/serial/by-id/...` 设备。
- 外部帧经包内协议、串口、传动与电机模型转换为 Gazebo effort；关节状态再编码成
  电机反馈帧返回外部控制器。

## 环境与构建

目标环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

`colcon list --names-only` 应只输出 `humanoid_gazebo`。

## 启动

只查看模型，不连接外部控制器：

```bash
ros2 launch humanoid_gazebo gazebo.launch.py
```

三路 USB CDC/UART 串口直连：

```bash
ros2 launch humanoid_gazebo usb_sim.launch.py \
  left_arm_port:=/dev/serial/by-id/usb-left-arm \
  right_arm_port:=/dev/serial/by-id/usb-right-arm \
  legs_port:=/dev/serial/by-id/usb-legs
```

三路 USB-CANFD 适配器：

```bash
ros2 launch humanoid_gazebo canfd_sim.launch.py \
  allow_external_canfd:=true \
  left_arm_port:=/dev/serial/by-id/usb-canfd-left-arm \
  right_arm_port:=/dev/serial/by-id/usb-canfd-right-arm \
  legs_port:=/dev/serial/by-id/usb-canfd-legs
```

`canfd_sim.launch.py` 会向每个适配器发送 `C`、`S8`、`Y5`、`O` 初始化序列，
对应仲裁段 1 Mbit/s、数据段 5 Mbit/s 和 BRS。只有显式设置
`allow_external_canfd:=true` 才会启动。

## Gazebo 动作验收

动作工具直接控制 Gazebo，不经过串口节点，不能与外部控制仿真同时运行：

```bash
ros2 run humanoid_gazebo ros_actions --list
ros2 run humanoid_gazebo ros_actions --action joint_scan
ros2 run humanoid_gazebo ros_actions --action suspended_legs --speed-scale 0.5
```

动作包括逐轴扫描、肩肱节律、双臂组合、悬空双腿和左右镜像检查。肩肱节律在通用
计算接口中默认关闭，只有对应验收动作显式启用。

## 安全边界

- 三路端口必须存在、使用稳定设备名且互不重复；任一路断开都会输出全零 effort 并
  终止整个仿真。
- 电机必须收到显式 enable 才响应；命令超时、越限、错总线或重复控制器都会锁存
  故障并将全部 effort 置零。
- `usb_serial` 只传输帧记录，不模拟仲裁、布线、终端电阻、误码或总线占用率。
- YAML 中的电机、电气、传动与增益参数都是仿名义值，不是厂家保证值或真机标定值。

物理接线和终端要求见 [USB 接线指南](src/humanoid_gazebo/docs/usb-wiring.md)，包内结构与
参数说明见 [`humanoid_gazebo` README](src/humanoid_gazebo/README.md)。
