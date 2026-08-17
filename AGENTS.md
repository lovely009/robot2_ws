# Repository Guidelines

ROS 2 Humble 工作空间（`dev` 开发分支）：真机 `motor_drive`（无 ROS 底层驱动，
双传输 `socketcan`/`serial_slcan`，协议 `rp`/`vaxor`）+ `motor_hardware_interface`
（ros2_control 适配 + 真机入口）+ 仿真 `motor_canfd_sim` + `upper_body_canfd_gazebo`。
**电机拓扑唯一来源是 YAML**：数量/类型/ID/关节名/总线/传输由
`motor_hardware_interface/config/motor_driver.yaml` 决定，增删改不修改 C++、launch
或 controller 配置。

## 结构与入口

- `ros2_ws/src/<package>` 标准 ament 布局（`src/`/`include/`/`config/`/`launch/`/`test/`，
  按需 `urdf/`、`scripts/`）；各包 `README.md` 是权威文档，修改前先读。
- launch 与测试共用 `motor_hardware_interface/motor_hardware_interface/helpers.py`
  （buses 读取、joints 生成、参数文件写入）。
- 协议 PDF 在 `ros2_ws/src/motor_drive/docs/`。

## 命令

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
ros2 launch motor_hardware_interface hardware.launch.py  # 真机，默认 tty 双总线
ros2 launch upper_body_canfd_gazebo gazebo.launch.py     # 仿真（vcan）
```

- 覆盖设备路径：`interface_<逻辑总线名>:=<路径>`；传输类型切换改 YAML `transport`。
- serial_slcan 需要 tty 写权限（dialout/sudo）；socketcan 启动前
  `ip link ... fd on` 配仲裁 1M/数据 5M；首次接入逐关节验证零位/方向/限位/急停。

## 代码规范

- C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类）、`k` 前缀常量；Python snake_case 且测试带类型注解。
- 中文注释写意图（数据流/单位/安全边界）；协议名、ROS 2 API 保持英文。

## 测试规范

- C++ GoogleTest（`test/test_*.cpp`）；Python pytest（`test/test_*.py`，
  函数名 `test_<behavior>()`）。
- 测试不得启动 Gazebo 或真实硬件；统一 `colcon test && colcon test-result --verbose`。

## 提交与 PR

- conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因。
- 功能经 PR 合入 `dev`；`main` 仅发布；真机/仿真线分别同步 `dev-real` / `dev-sim`；
  PR 附验证方式（如 `colcon test` 结果）。

## Agent 注意事项

- 不写死电机数量/类型/总线：拓扑只来自 YAML；扩展协议走 `MotorProtocolRegistry`、
  扩展传输走 `CanTransportRegistry`，不改通用收发循环。
- `bus_override` 只覆盖设备路径，传输类型切换须改 YAML。
- URDF 物理模型（几何/惯性）人工维护；ros2_control 关节清单由 launch 自动生成，
  不手写 `canfd_control_joint`。
- 软件安全逻辑不能替代硬件急停、机械限位与真机标定。
