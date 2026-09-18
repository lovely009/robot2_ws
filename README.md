# robot_ws（dev/robot_ws）

机器人的 ROS 2 Humble 工作空间（开发分支 `dev/robot_ws`）。六个 ament 包，真机、仿真、消息定义与 Web 上位机并存：

| 包 | 定位 |
|---|---|
| `can_bridge_msgs` | 固件与 ROS 2 共用的 ID 化有界消息定义 |
| `motor_drive` | 无 ROS 底层驱动库（传输 / 线协议 / 电机映射） |
| `motor_hardware_interface` | ros2_control 适配层 + 真机 bring-up |
| `motor_sim` | 与机器人无关的共享仿真库（协议 / 传输 / 传动 / plant / 帧路由 / 锁存安全） |
| `robot_arm_gazebo` | 机械臂 URDF 模型库（`models/<模型名>`）与 Ignition Gazebo 集成 |
| `web_host` | rosbridge Web 上位机（姿态、IMU、电机状态与命令快照） |

## 分支

- `dev/robot_ws`：开发分支（本分支，真机 + 仿真 6 包）
- `dev-sim`：仿真线（纯 `motor_sim` + `robot_arm_gazebo`）
- `main`：发布

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

## 真机 bring-up

默认 serial_slcan 双总线（`/dev/ttyACM0`、`/dev/ttyACM1`），无需传参：

```bash
ros2 launch motor_hardware_interface hardware.launch.py
```

覆盖设备路径：`interface_<逻辑总线名>:=<路径>`（如 `interface_left_arm:=can0`）。
传输类型由 YAML 决定：切换 socketcan ↔ serial_slcan 须改
`motor_hardware_interface/config/motor_driver.yaml`；`bus_override` 只覆盖设备路径。

## 仿真 bring-up

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```

`gazebo.launch.py` 走 Ignition + `gz_ros2_control` 位置控制器，不经过 `motor_sim` 的 vcan 回路
（`motor_sim` 的接口选择由 `transport_mode` 与显式的 bus→端口参数决定，无默认值）。

## 配置（两份文件）

- **`config/motor.yaml`（型号库）**：每型号**协议运行所需静态参数**（如
  `torque_constant_nm_per_amp`、`gear_ratio`）与软件侧**安全阈值**（额定/峰值
  扭矩等），与拓扑无关、可跨机器人复用。完整电机规格（电磁参数、背隙、
  电流/电压/温度范围等）见 `ros2_ws/src/motor_drive/docs/` 规格书。
- **`config/motor_driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 +
  `timing` + `buses` + `motors` 装配（每台电机 `type` 引用 motor.yaml 型号，
  单电机可覆盖参数；`command.kp/kd` 为每台电机的动态控制增益）。

## 核心契约

- **双文件配置**：拓扑 `motor_driver.yaml`（电机数量/类型/ID/关节名/总线/传输/
  动态增益）与型号参数 `motor.yaml`（协议运行参数/安全阈值）分离；增删改电机
  零代码改动（新增物理关节时同步 URDF，测试兜底）。
- **接口自适应**：`CanTransportRegistry` + YAML `transport` 字段，内置
  `socketcan` / `serial_slcan`。
- **协议自适应**：`MotorProtocolRegistry` + YAML `type`，内置 `vaxor` / `cerb`。
- **清单自动生成**：controller joints 与 URDF ros2_control 关节清单由 launch 按
  实际配置生成，不手抄；`motor_config_file:=` 覆盖后以实际配置为准。

## 扩展指南

| 场景 | 改动 |
|---|---|
| 增删电机 | 只改 `motor_driver.yaml` `motors`（新物理关节时同步 URDF） |
| 修改电机物理参数 | 改 `motor.yaml` 型号段（torque_constant / gear_ratio 等） |
| 新增电机类型 | 实现 `MotorProtocol` + 注册工厂 + `motor.yaml` 声明 |
| 新增传输 | 实现 `CanTransport` + 注册工厂 + YAML `transport` |

## 文档

- 包说明：`ros2_ws/src/<pkg>/README.md`
- Agent 指南：[AGENTS.md](AGENTS.md)
- 术语与命名：[CONTEXT.md](CONTEXT.md)
- 架构决策：[docs/adr/](docs/adr/)
- 协议手册：`ros2_ws/src/motor_drive/docs/`
