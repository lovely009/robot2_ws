# robot_ws（dev-real）

机器人的 ROS 2 Humble 工作空间，真机线。仅两个 ament 包：
`motor_drive`（无 ROS 底层驱动）+ `motor_hardware_interface`（ros2_control 适配层）。
仿真在 `dev-sim` 分支，本分支不依赖仿真栈。

## 分支

- `dev-real`：真机线
- `dev-sim`：仿真线
- `main`：发布

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose   # 基线 81 项全绿
```

## 真机 bring-up

默认 serial_slcan 双总线（`/dev/ttyACM0`、`/dev/ttyACM1`），无需传参：

```bash
ros2 launch motor_hardware_interface hardware.launch.py
```

覆盖设备路径：`interface_<逻辑总线名>:=<路径>`（如 `interface_left_arm:=can0`）。
传输类型由 YAML 决定：切换 socketcan ↔ serial_slcan 须改
`motor_hardware_interface/config/motor.driver.yaml`；`bus_override` 只覆盖设备路径。

## 配置（两份文件）

- **`config/motor.yaml`（型号库）**：每型号**完整电机规格**（电磁：Kt/Kv/R/L；
  传动：减速比/效率/背隙；能力与保护阈值：额定/峰值电流与扭矩、电压范围、
  绕组温度；控制默认值 `command` kp/kd），与拓扑无关、可跨机器人复用。
  标注「运行消费」的参数由协议策略读取（如 `torque_constant`、`gear_ratio`），
  其余为机器可读的规格档案，供校验与未来保护/故障逻辑消费。
- **`config/motor.driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 +
  `timing` + `buses` + `motors` 装配（每台电机 `type` 引用 motor.yaml 型号，
  单电机可覆盖参数）。

## 核心契约

- **双文件配置**：拓扑 `motor.driver.yaml`（电机数量/类型/ID/关节名/总线/传输）
  与型号参数 `motor.yaml`（物理规格/控制默认值）分离；增删改电机零代码改动
  （新增物理关节时同步 URDF，测试兜底）。
- **接口自适应**：`CanTransportRegistry` + YAML `transport` 字段，内置
  `socketcan` / `serial_slcan`。
- **协议自适应**：`MotorProtocolRegistry` + YAML `type`，内置 `rp` / `motor_2530`；
  `motor_2530` 支持 `gear_ratio` 电机↔关节换算（默认 1，配置即启用）。
- **清单自动生成**：controller joints 与 URDF ros2_control 关节清单由 launch 按
  实际配置生成，不手抄；`motor_config_file:=` 覆盖后以实际配置为准。

## 扩展指南

| 场景 | 改动 |
|---|---|
| 增删电机 | 只改 `motor.driver.yaml` `motors`（新物理关节时同步 URDF） |
| 修改电机物理参数 | 改 `motor.yaml` 型号段（Kt / gear_ratio / 型号） |
| 新增电机类型 | 实现 `MotorProtocol` + 注册工厂 + `motor.yaml` 声明 |
| 新增传输 | 实现 `CanTransport` + 注册工厂 + YAML `transport` |

## 文档

- 包说明：[motor_drive](src/motor_drive/README.md) /
  [motor_hardware_interface](src/motor_hardware_interface/README.md)
- Agent 指南：[AGENTS.md](AGENTS.md)
- 术语与命名：[CONTEXT.md](CONTEXT.md)
- 架构决策：[docs/adr/](docs/adr/)
- 协议手册：`src/motor_drive/docs/`
