# robot_ws（dev）

机器人的 ROS 2 Humble 工作空间（开发分支 `dev`）。四个 ament 包，真机与仿真并存：

| 包 | 定位 |
|---|---|
| `motor_drive` | 无 ROS 底层驱动库（传输 / 线协议 / 电机映射） |
| `motor_hardware_interface` | ros2_control 适配层 + 真机 bring-up |
| `motor_canfd_sim` | C++ 电机侧模拟器（vcan ↔ Gazebo） |
| `upper_body_canfd_gazebo` | Gazebo 模型 / launch / 动作库 |

## 分支

- `dev`：开发分支（本分支，真机 + 仿真 4 包）
- `dev-real`：真机线（仅 `motor_drive` + `motor_hardware_interface` 两包）
- `dev-sim`：仿真线
- `main`：发布

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
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
ros2 launch upper_body_canfd_gazebo gazebo.launch.py   # 或 model_sim / driver_loopback
```

仿真默认使用 `vcan0/vcan1`；模拟器仅允许打开 `vcan*` 接口，防止与真实电机重复 CAN ID。

## 配置（两份文件）

- **`config/motor.yaml`（型号库）**：每型号**完整电机规格**（电磁：Kt/Kv/R/L；
  传动：减速比/效率/背隙；能力与保护阈值：额定/峰值电流与扭矩、电压范围、
  绕组温度；控制默认值 `command` kp/kd），与拓扑无关、可跨机器人复用。
  标注「运行消费」的参数由协议策略读取（如 `torque_constant`、`gear_ratio`），
  其余为机器可读的规格档案，供校验与未来保护/故障逻辑消费。
- **`config/motor_driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 +
  `timing` + `buses` + `motors` 装配（每台电机 `type` 引用 motor.yaml 型号，
  单电机可覆盖参数）。

## 核心契约

- **双文件配置**：拓扑 `motor_driver.yaml`（电机数量/类型/ID/关节名/总线/传输）
  与型号参数 `motor.yaml`（物理规格/控制默认值）分离；增删改电机零代码改动
  （新增物理关节时同步 URDF，测试兜底）。
- **接口自适应**：`CanTransportRegistry` + YAML `transport` 字段，内置
  `socketcan` / `serial_slcan`。
- **协议自适应**：`MotorProtocolRegistry` + YAML `type`，内置 `rp` / `vaxor`。
- **清单自动生成**：controller joints 与 URDF ros2_control 关节清单由 launch 按
  实际配置生成，不手抄；`motor_config_file:=` 覆盖后以实际配置为准。

## 扩展指南

| 场景 | 改动 |
|---|---|
| 增删电机 | 只改 `motor_driver.yaml` `motors`（新物理关节时同步 URDF） |
| 修改电机物理参数 | 改 `motor.yaml` 型号段（Kt / gear_ratio / 型号） |
| 新增电机类型 | 实现 `MotorProtocol` + 注册工厂 + `motor.yaml` 声明 |
| 新增传输 | 实现 `CanTransport` + 注册工厂 + YAML `transport` |

## 文档

- 包说明：`ros2_ws/src/<pkg>/README.md`
- Agent 指南：[AGENTS.md](AGENTS.md)
- 术语与命名：[CONTEXT.md](CONTEXT.md)
- 架构决策：[docs/adr/](docs/adr/)
- 协议手册：`ros2_ws/src/motor_drive/docs/`
