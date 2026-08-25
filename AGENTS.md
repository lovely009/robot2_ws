# Repository Guidelines

本仓库是 ROS 2 Humble 的纯 Gazebo 仿真工程，只包含 `humanoid_gazebo` 一个 ament
包。不要新增真机 bring-up、外部硬件插件或第二个 ROS package。

## 结构与边界

- `src/humanoid_gazebo/include`、`src`：包内 C++ 协议、串口、传动、电机模型和 ROS 节点。
- `config/motors.yaml`：30 轴顺序、三条逻辑总线、电机协议、ID、限位和仿名义参数的
  单一来源。
- `config/mechanics.yaml`、`urdf/humanoid.urdf.xacro`：170 cm、45 kg 悬空人形模型。
- `launch/`：只保留 `gazebo.launch.py`、`usb_sim.launch.py`、`canfd_sim.launch.py`。
- `docs/protocol/`：RP 与 Vaxor 协议手册；`docs/usb-wiring.md`：外部接线边界。
- `build/`、`install/`、`log/`：colcon 生成物，不提交。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

测试不得要求 Gazebo GUI 或真实串口。串口集成测试使用 PTY；launch 测试只做静态契约
验证。提交前还应确认 `colcon list --names-only` 只输出 `humanoid_gazebo`。

## 代码风格

- C++17，2 空格缩进，启用 `-Wall -Wextra -Wpedantic`。
- C++ 文件、函数、变量使用 `snake_case`，类使用 `PascalCase`，常量以 `k` 开头。
- Python 使用 `snake_case` 和类型注解。
- 中文注释说明设计意图、数据流、单位和安全边界；协议名与 ROS 2 API 保留英文。
- 新增电机或调整拓扑优先修改 YAML 和协议策略，不在节点中写死关节数量。

## 运行约束

- 传输模式只有 `usb_serial` 和 `serial_slcan`，一个进程只能选择一种。
- `left_arm`、`right_arm`、`legs` 必须绑定三个唯一的 `/dev/serial/by-id/...` 路径。
- `serial_slcan` 必须显式设置 `allow_external_canfd:=true`。
- 保持显式 enable、150 ms 命令超时、断线停机、越限/错总线/重复控制器锁存等安全语义。
- 动作播放器只用于模型验收，不应由两个串口 launch 启动，也不应与外部电机仿真节点并行控制。
- 仿真参数不得描述为厂家规格或用于推导真机安全结论。

## Git 与文档

功能在 `dev/smi` 开发，提交摘要优先使用 `feat:`、`fix:`、`refactor:`、`docs:`、
`test:`。修改运行入口、30 轴 ABI、协议或安全语义时同步更新根 README、包 README、
`CONTEXT.md` 和相关测试。
