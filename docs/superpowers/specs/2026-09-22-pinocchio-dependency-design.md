# Pinocchio 依赖接入设计

- 日期：2026-09-22
- 状态：已实施
- 范围：仅接入依赖，不改变现有控制与仿真行为

## 背景

当前工作区包含底层电机驱动、ros2_control 真机适配、独立电机仿真和机械臂 Gazebo 模型，但没有通用的机器人运动学/动力学计算包。Pinocchio 后续可用于正逆运动学、动力学和轨迹前馈；本次只建立独立依赖边界，不让它进入 `motor_drive`、`motor_hardware_interface` 或 `robot_arm_gazebo` 的现有运行路径。

## 决策

新增 ROS 2 ament C++ 包 `robot_dynamics`，作为 Pinocchio 的依赖承载包和后续机器人计算入口。

- 使用系统/ROS 发行版提供的 `ros-humble-pinocchio`，不在仓库内复制或编译 Pinocchio 源码。
- `robot_dynamics` 直接声明 `pinocchio` 依赖，并在 CMake 中执行 `find_package(pinocchio REQUIRED)`。
- 包提供最小的可链接目标，并通过一个编译/运行 smoke test 验证 Pinocchio 头文件、CMake 导出和基础模型对象可用。
- 本次不添加 ROS 节点、topic、service、参数、控制器、launch 修改或 URDF 解析逻辑。
- 现有六个包的依赖关系和运行行为保持不变；后续真正使用 Pinocchio 时再在该包内增加明确的 API 和测试。

## 文件与边界

新增：

- `ros2_ws/src/robot_dynamics/package.xml`：ament、`pinocchio`、测试依赖。
- `ros2_ws/src/robot_dynamics/CMakeLists.txt`：Pinocchio 查找、接口库、安装导出和 smoke test。
- `ros2_ws/src/robot_dynamics/include/robot_dynamics/`：保留后续 API 的公开头文件目录，本次不放无用途的运行时抽象。
- `ros2_ws/src/robot_dynamics/test/test_pinocchio_dependency.cpp`：最小依赖验证。
- `ros2_ws/src/robot_dynamics/README.md`：安装前提、构建命令和本次范围。

不修改：

- `motor_drive`：继续保持 ROS 无关。
- `motor_hardware_interface`：继续只负责 ros2_control 与真机 bring-up。
- `motor_sim`：继续负责与机器人无关的电机仿真。
- `robot_arm_gazebo`：继续作为模型资源和 Gazebo 集成包。
- 现有 launch、YAML、URDF 和控制器配置。

## 构建与依赖策略

目标环境是 ROS 2 Humble / Ubuntu 22.04。构建前安装发行版包：

```bash
sudo apt install ros-humble-pinocchio
```

在 ROS 环境已 source 的前提下：

```bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_dynamics
source install/setup.bash
colcon test --packages-select robot_dynamics
colcon test-result --verbose
```

若 Pinocchio 的发行版包未安装，CMake 应明确失败，而不是静默降级或自动下载源码。

## 验证标准

1. `colcon build --symlink-install --packages-select robot_dynamics` 成功。
2. smoke test 能包含 Pinocchio 公开头文件并构造基础 `pinocchio::Model`。
3. `colcon test --packages-select robot_dynamics` 通过。
4. 构建或测试现有包的行为不因新增包发生变化；不启动 Gazebo、不连接真实硬件。
5. `git diff` 只包含新包和本设计文档，不包含生成的 `build/`、`install/`、`log/` 文件。

## 非目标

- 不实现 FK、IK、RNEA、重力补偿或轨迹前馈。
- 不从 URDF 加载模型。
- 不把 Pinocchio 加入底层电机驱动或硬件实时循环。
- 不自动修改系统 apt 源或绕过 ROS/系统包管理安装依赖。
