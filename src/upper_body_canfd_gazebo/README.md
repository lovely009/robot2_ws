# upper_body_canfd_gazebo

14 自由度上半身双臂的 Gazebo Fortress 集成包，负责机器人模型、launch 编排、参考动作播放与仿真/真机入口。

## 包的定位

当前仓库将功能拆分到 4 个包中：

- [../motor_canfd_drive/README.md](../motor_canfd_drive/README.md)：底层 SocketCAN/CAN-FD 驱动与协议策略。
- [../motor_hardware_interface/README.md](../motor_hardware_interface/README.md)：ROS 2 适配层与 YAML 配置解析。
- [../motor_canfd_sim/README.md](../motor_canfd_sim/README.md)：唯一支持的电机仿真后端。
- 本包：机器人模型、Gazebo 集成与上层 launch/动作工具。

## 机器人拓扑

左右臂各 7 自由度、各使用一条独立 CAN-FD 总线。真机权威映射位于 [../motor_hardware_interface/config/motor_canfd.yaml](../motor_hardware_interface/config/motor_canfd.yaml)；仿真拓扑使用 [../motor_canfd_sim/config/motor_canfd.yaml](../motor_canfd_sim/config/motor_canfd.yaml)，来源于仿真模型，与真机解耦。

| 关节 | 电机类型 | 总线内 ID |
|---|---|---:|
| `shoulder_yaw` | RP | 1 |
| `shoulder_roll` | RP | 2 |
| `shoulder_pitch` | RP | 3 |
| `elbow_pitch` | RP | 4 |
| `wrist_yaw` | 25/30 mm | 1 |
| `wrist_pitch` | 25/30 mm | 2 |
| `wrist_roll` | 25/30 mm | 3 |

仿真默认使用 `vcan0/vcan1`，真机默认使用 `can0/can1`。

## Launch 入口

| 文件 | 用途 |
|---|---|
| `gazebo.launch.py` | 启动 Gazebo、模型与 `/clock` |
| `model_sim.launch.py` | 启动 Gazebo 模型与电机仿真后端 |
| `driver_loopback.launch.py` | 完整仿真回环：驱动侧 controller + 模拟器 + Gazebo |

真机 bring-up 入口已下沉到 `motor_hardware_interface`（launch、控制器配置与 CAN-FD
ros2_control 硬件片段），本包只负责模型描述与仿真集成。

## 运行示例

### 全链路仿真

已在当前环境中实际验证过以下命令：

```bash
bash scripts/setup_vcan.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py gui:=false
```

实际结果是：
- Gazebo、`motor_canfd_sim`、`robot_state_publisher` 与桥接节点会正常启动；
- 驱动侧 `ros2_control_node` 成功加载 `UpperBodyCanFdSystem`，`/driver/controller_manager` 与 `position_controller` 正常就绪；
- 向 `/driver/position_controller/commands` 发布位置指令后，`/joint_states`（Gazebo）与 `/driver/joint_states`（CAN 反馈）同步跟随，完整回环可用。

因此，当前仓库的“完整回环”已实际验证可运行。模拟器退出时整个回环会随之终止
（fail-fast，ADR 0004），避免 spawner 无限重试。

### 参考动作

```bash
source install/setup.bash
ros2 run upper_body_canfd_gazebo ros_actions --list
ros2 run upper_body_canfd_gazebo ros_actions --action 8
ros2 run upper_body_canfd_gazebo ros_actions --speed-scale 0.5
```

### 真机

```bash
ros2 launch motor_hardware_interface hardware.launch.py \
  model_xacro:=src/upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro \
  left_interface:=can0 right_interface:=can1
```

启动前需要完成 CAN-FD 总线配置，并逐关节检查零位、方向、限位、增益与急停。

## 关键目录

- `config/`：Gazebo 侧 controllers 配置（`driver_controllers.yaml` 已随真机入口下沉到
  `motor_hardware_interface`）。
- `launch/`：仿真与真机 launch 文件。
- `urdf/`：机器人模型与 ros2_control 相关配置。
- `worlds/`：Gazebo 世界文件。
- `upper_body_canfd_gazebo/`：动作库与 ROS 播放器实现。
- `scripts/`：辅助脚本，如 `setup_vcan.sh`。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to upper_body_canfd_gazebo
colcon test --packages-select upper_body_canfd_gazebo
colcon test-result --verbose
```

测试应避免启动 Gazebo 或真实硬件。`test_config_topology.py` 静态断言真机/仿真
拓扑一致，且 URDF、effort controller、仿真电机拓扑三者顺序一致；
`test_driver_loopback_launch.py` 静态断言启动/终止时序契约。
