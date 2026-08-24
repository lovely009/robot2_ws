# robot_ws

14 自由度上半身双臂机器人的完整工作空间，覆盖 CAN-FD 电机仿真与真机控制：ROS 2 电机控制栈 + STM32 micro-ROS 固件 + micro-ROS Agent。

## 仓库结构（三区域）

| 区域 | 内容 | 技术栈 |
|---|---|---|
| `ros2_ws/` | ROS 2 Humble 电机栈：真机驱动、硬件接口、仿真、Gazebo | C++17 / Python，colcon |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件 | PlatformIO + STM32duino + FreeRTOS |
| `micro_ros_agent/` | micro-ROS Agent 桥（serial ↔ 固件） | 独立 colcon 工作区 |

各区域独立构建；权威文档优先读取各区域/各包的 `README.md`，架构取舍见 `docs/adr/`。

## ros2_ws（ROS 2 电机栈）

四个 ament 包，真机与仿真并存：

| 包 | 定位 |
|---|---|
| `motor_drive` | 无 ROS 底层驱动库（CAN 传输 `socketcan`/`serial_slcan`、协议 `rp`/`vaxor`、电机映射） |
| `motor_hardware_interface` | ros2_control 适配层 + 真机 bring-up 入口 |
| `motor_canfd_sim` | C++ 电机侧模拟器（vcan ↔ Gazebo），唯一仿真后端 |
| `upper_body_canfd_gazebo` | Gazebo 模型 / launch / 动作库 |

**电机拓扑唯一来源是 YAML**：数量/类型/ID/关节名/总线/传输由
`motor_hardware_interface/config/motor_driver.yaml` 决定，增删改不修改 C++、launch 或 controller 配置。

## micro_ros_firmware（STM32 固件）

STM32H723VGT6 micro-ROS 固件，serial over USB CDC 进 ROS 2（humble）。业务：BMI088 IMU、三路 FDCAN、电机协议编解码（RP / 25-30mm，来自 `ros2_ws/src/motor_drive`）。详细决策与踩坑见 `micro_ros_firmware/README.md`。

## micro_ros_agent（micro-ROS Agent）

独立 colcon 工作区，把固件（serial USB CDC `/dev/ttyACM0`）桥进 ROS 2 域：`micro_ros_msgs` + `can_bridge_msgs` + micro-ROS-Agent 快照。构建/联调详见 `micro_ros_agent/README.md`。

## 分支

| 分支 | 内容 |
|---|---|
| `dev/robot_ws` | 全量开发分支（三区域完整，真机 + 仿真 + 固件） |
| `dev/ros_ws` | 仅 ROS 2 工作空间（不含固件） |
| `dev-real` | 真机线 |
| `dev-sim` | 仿真线 |
| `main` | 发布 / 仓库介绍 |

## 快速开始

```bash
# ros2_ws（ROS 2 电机栈）
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
ros2 launch motor_hardware_interface hardware.launch.py   # 真机（默认 tty 双总线）
ros2 launch upper_body_canfd_gazebo gazebo.launch.py      # 仿真（vcan）

# micro_ros_firmware（固件，独立于 colcon）
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 构建
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录

# micro_ros_agent（Agent，独立 colcon 工作区）
source /opt/ros/humble/setup.bash
colcon build --symlink-install
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

## 文档入口

- Agent 指南：`AGENTS.md`（三区域权威指南）
- 术语与命名：`CONTEXT.md`
- 架构决策：`docs/adr/`
- 各包/各区域说明：对应 `README.md`
