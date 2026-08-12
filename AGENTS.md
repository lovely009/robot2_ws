# Repository Guidelines

ROS 2 Humble 工作空间（`dev-real` 分支）：14 自由度上半身双臂机器人的 CAN-FD 真机控制。
`motor_drive` 是与 ROS 无关的 SocketCAN/CAN-FD 驱动（内置 RP 与 25/30 mm 协议），
`motor_hardware_interface` 是 `ros2_control` 适配层与真机 bring-up 入口。仿真回环与
CANable2 串口台架能力保留在 `dev-sim` 分支，本分支不维护它们。

## 项目结构 (Project Structure & Module Organization)

- `src/<package>` — 2 个 ament 包，采用标准布局：`src/`（实现）、
  `include/<package>/`（C++ 头文件）、`config/`（YAML 配置）、`launch/`、`test/`，
  按需含 `urdf/`。
- `build/`、`install/`、`log/` — colcon 生成物，不提交到 Git。
- 协议 PDF — 25–30 mm 驱动与 RP 系列关节 CAN-FD 协议硬件手册，位于
  `src/motor_drive/docs/`。
- 各包内 `README.md` 是权威文档，先读它。
- launch 与静态测试共用辅助函数：`motor_hardware_interface/motor_hardware_interface/helpers.py`
  （buses 读取、joints 生成、参数文件写入）。

## 构建、测试与开发命令 (Build, Test, and Development Commands)

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

真机 bring-up（适配层入口，模型默认来自本包 `urdf/upper_body.urdf.xacro`）：

```bash
ros2 launch motor_hardware_interface hardware.launch.py \
  interface_left_arm:=can0 interface_right_arm:=can1
```

启动前把总线配置为仲裁 1 Mbit/s、数据 5 Mbit/s（`ip link ... fd on`），并逐关节
验证零位、方向、限位与急停。

## 代码风格与命名 (Coding Style & Naming Conventions)

- C++17，编译选项 `-Wall -Wextra -Wpedantic`，缩进 2 空格。
- C++：文件、函数、变量用 `snake_case`，类用 `PascalCase`，常量以 `k` 开头（如 `kRpMotorType`）。
- Python：`snake_case`，测试代码带类型注解。
- 注释：中文说明设计意图、数据流、单位与安全边界；协议名、ROS 2 API 保持英文。
- 未配置 linter，与现有代码风格保持一致。

## 测试规范 (Testing Guidelines)

- C++：GoogleTest（`ament_cmake_gtest`），测试文件位于 `src/<package>/test/test_*.cpp`。
- Python：pytest（`ament_pytest`），测试文件位于 `src/<package>/test/test_*.py`，
  函数命名 `test_<behavior>()`。
- 测试不得启动 Gazebo 或真实硬件。
- 统一运行：`colcon test && colcon test-result --verbose`。

## 提交与 PR 规范 (Commit & Pull Request Guidelines)

- 历史中英混用；优先简洁的 conventional 前缀（`feat:`、`fix:`、`docs:`）或祈使句摘要，
  正文写清原因。
- 功能经 PR 合入 `dev-real`；`main` 仅用于发布；`dev-sim` 保留仿真与串口能力，本分支
  不反向合入。
- PR 说明改动内容与验证方式（如 `colcon test` 结果），必要时附 launch/行为截图。

## Agent 专用说明 (Agent-Specific Instructions)

- 修改前先读根目录 `README.md` 与受影响包的 `README.md`。
- 新增电机类型优先扩展协议策略与 YAML 配置，不要在驱动里写死关节数量或左右臂拓扑。
- 本分支只保留 SocketCAN 传输；不要在 `motor_drive` 重新引入串口/slcan 传输。
- 真机运行前配置 CAN-FD 总线（仲裁 1 Mbit/s、数据 5 Mbit/s），并逐关节验证零位、
  方向、限位与急停。
