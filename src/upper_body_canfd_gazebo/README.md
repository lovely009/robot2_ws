# upper_body_canfd_gazebo

14 自由度上半身双臂的机器人模型、Gazebo Fortress 集成、真机 bring-up 和 ROS 参考动作工具。

## 机器人拓扑

左右臂各 7 自由度、各使用一条独立 CAN-FD 总线：

| 关节 | 电机类型 | 总线内 ID |
|---|---|---:|
| `shoulder_yaw` | RP | 1 |
| `shoulder_roll` | RP | 2 |
| `shoulder_pitch` | RP | 3 |
| `elbow_pitch` | RP | 4 |
| `wrist_yaw` | 25/30 mm | 1 |
| `wrist_pitch` | 25/30 mm | 2 |
| `wrist_roll` | 25/30 mm | 3 |

仿真默认使用 `vcan0/vcan1`，真机默认使用 `can0/can1`。权威电机映射位于
`motor_hardware_interface/config/motor_canfd.yaml`。

## Launch 入口

| 文件 | 启动内容 | 用途 |
|---|---|---|
| `gazebo.launch.py` | Gazebo、模型、`/clock`、Gazebo controllers | 模型开发 |
| `model_sim.launch.py` | `gazebo.launch.py` + C++ 电机模拟器 | 电机模型调试 |
| `driver_loopback.launch.py` | 模型、模拟器、真实硬件插件、position controller | 完整仿真回环 |
| `hardware.launch.py` | 硬件插件、position controller、state publisher | 真机 bring-up |

完整仿真包含两个 controller manager：

- `/controller_manager`：Gazebo 侧 `effort_controller`；
- `/driver/controller_manager`：CAN-FD 驱动侧 `position_controller`。

启动顺序由 launch event handler 编排，不使用固定延时。

## 全链路仿真

```text
/driver/position_controller
  -> motor_hardware_interface
  -> motor_canfd_drive
  -> vcan0 / vcan1
  -> motor_canfd_sim
  -> Gazebo effort_controller
  -> /joint_states
  -> CAN-FD feedback
```

准备并启动：

```bash
bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py
```

无图形环境可添加 `gui:=false`；渲染兼容问题可通过 `render_engine:=ogre` 覆盖。

## 参考动作

动作播放器读取 `/driver/joint_states`，向
`/driver/position_controller/commands` 发布 14 关节位置目标，不直接访问 SocketCAN。

```bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 8
ros2 run upper_body_canfd_gazebo ros_actions --speed-scale 0.5
```

关键帧使用 `s(u) = 3u² - 2u³` smoothstep 插值，默认发布频率为 100 Hz，段首尾速度为零。

## 真机

```bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

启动前要求：

1. 两条总线配置为仲裁段 1 Mbit/s、数据段 5 Mbit/s，并启用 CAN-FD BRS；
2. 按单关节到单臂、再到双臂的顺序验证；
3. 确认零位、方向、软硬限位、增益、扭矩常数和急停；
4. 确认反馈超时与停用流程能使电机安全失能。

## 主要文件

```text
config/controllers.yaml             Gazebo effort controller
config/driver_controllers.yaml      驱动侧 position controller
launch/                             仿真与真机入口
urdf/upper_body.urdf.xacro          机器人模型和硬件后端选择
worlds/upper_body_world.sdf         Gazebo 世界
upper_body_canfd_gazebo/
  motion_library.py                 参考动作与关键帧
  ros_action_player.py              ROS position command 播放器
scripts/setup_vcan.sh               创建 vcan0/vcan1
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to upper_body_canfd_gazebo
colcon test --packages-select upper_body_canfd_gazebo
colcon test-result --verbose
```

测试覆盖动作库、命令顺序和 launch 静态结构；测试不得启动 Gazebo 或真实硬件。
