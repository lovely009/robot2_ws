# motor_hardware_interface

将 `motor_drive` 接入 ROS 2 Humble `ros2_control` 的适配包（`dev-real` 真机线），
提供真机 bring-up 入口，不依赖仿真栈。

## 职责

- `hardware_interface::SystemInterface` lifecycle；导出 position command 与
  position/velocity/effort state interface。
- 解析校验拓扑 `config/motor_driver.yaml`（唯一入口，经 `motor_specs_file`
  引用型号库 `config/motor.yaml`），转成 `motor_drive::DriverConfig`；按
  joint 名建立 ROS 关节 ↔ 电机映射。
- 首次反馈等待、运行期反馈超时与安全失能（fail-fast）。
- bring-up：launch、controller 配置、`urdf/canfd_hardware.xacro` 硬件片段与内置
  `urdf/upper_body.urdf.xacro` 模型；controller joints 与 ros2_control 关节清单
  由 launch 按配置生成（单一来源），不手抄。

## 配置（两份文件）

`config/motor.yaml`（型号库，仅协议运行所需的最小静态参数，与拓扑无关；
规格声明见 `src/motor_drive/docs/` 各型号规格书）。每个型号的 `protocol`
字段声明所属协议类型；`kp/kd` 是动态参数，在 `motor_driver.yaml` 配置：

```yaml
motor_types:
  creb_r58_101:
    protocol: creb                     # 巨蟹 CREB Lite 关节模组
    parameters:
      torque_constant_nm_per_amp: 0.086 # Iq A -> 电机轴 Nm
      gear_ratio: 101                   # 电机↔关节换算
  vaxor_2550:
    protocol: vaxor
    parameters:
      torque_constant_nm_per_amp: 0.042 # 电流 A -> 电机轴 Nm
      gear_ratio: 50                     # 电机↔关节换算
      gear_efficiency: 0.65              # effort × 效率
```

`config/motor_driver.yaml`（拓扑 + 动态参数，唯一入口）：

```yaml
motor_hardware:
  motor_specs_file: motor.yaml   # 型号库，路径相对本文件目录
  timing: {feedback_timeout_s: 0.10, initial_feedback_wait_s: 1.0}
  buses:
    left_arm:  {transport: serial_slcan, tty: /dev/ttyACM0}
    right_arm: {transport: serial_slcan, tty: /dev/ttyACM1}
  motors:
    left_shoulder_yaw: {joint: left_shoulder_yaw_joint, bus: left_arm,
                        type: creb_r58_101, id: 1,
                        command: {kp: 30.0, kd: 1.0}}
```

- `type` 填型号名（如 `creb_r58_101`、`vaxor_2550`），解析层经 `protocol` 字段映射为
  协议类型（`creb`/`vaxor`）交给底层驱动；参数默认值按型号名合并。
- `command.kp/kd` 是每台电机的动态调参增益，在拓扑里显式配置。
- `transport` 缺省 `socketcan`；`socketcan` 要求 `interface`（位率由 `ip link`
  配置），`serial_slcan` 要求 `tty`（驱动内固定 S8/Y5）。`motors` 数量不限，
  单电机参数/command 覆盖型号库默认值。
- `bus_override.<逻辑总线名>`（launch `interface_<总线名>`）只覆盖设备路径，
  传输类型切换须改 YAML。
- 公开解析 API：`load_driver_config(path)` / `load_system_config(path)`。

## 真机 bring-up

```bash
ros2 launch motor_hardware_interface hardware.launch.py          # 默认 tty 双总线
ros2 launch motor_hardware_interface hardware.launch.py \
  interface_left_arm:=can0                                       # 覆盖设备路径
```

`interface_<总线名>` 参数按默认配置声明；`model_xacro` / `motor_config_file`
可覆盖，后者生效后 joints 与 URDF 清单以实际配置为准（launch 用 `OpaqueFunction`
运行时求值），但覆盖配置的总线名集合应包含在默认配置中。

## 安全行为

- 激活：打开传输 → 失能 + 零增益轮询反馈（1 s 窗口，最多 3 次）→ 实测位置作为
  初始目标 → 统一使能；未收齐反馈即 fail-fast。
- 运行期 `read()` 反馈超时返回错误，`controller_manager` 立即终止。
- 软件兜底不能替代硬件急停、机械限位与真机标定；serial_slcan 需 tty 写权限。

## 模型与 launch

- `canfd_hardware.xacro`：与拓扑无关的硬件片段（`canfd_hardware` /
  `canfd_control_joint` / `emit_control_joints` 宏），参数
  `motor_config_file` / `bus_overrides` / `control_joints`。
- `upper_body.urdf.xacro`：物理模型人工维护；ros2_control 关节清单由
  `control_joints` 注入，不手写；一致性由测试兜底。

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to motor_hardware_interface
colcon test --packages-select motor_hardware_interface && colcon test-result --verbose
```

`test_motor_config`（YAML 解析含 transport/tty）、`test_canfd_logic`（激活/超时/
映射纯逻辑）、`test_hardware_launch`（launch 静态检查）、`test_xacro_consistency`
（渲染一致性：ros2_control 清单 == YAML == 物理关节）。
