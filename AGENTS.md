# Repository Guidelines

本仓库是 ROS 2 Humble 的纯 Gazebo 仿真工程，只包含 `motor_sim` 与
`robot_arm_gazebo` 两个 ament 包。不要新增真机 bring-up、外部硬件插件或
第三个 ROS package。

## 结构与边界

- `src/motor_sim`：与机器人无关的共享仿真库。`protocol`（RP/Vaxor/CERB 协议）、
  `transport`（串口帧记录与 SocketCAN）、`core`（配置、传动、plant、路由、锁存安全）、
  `ros`（节点连接层）。不得引入任何具体机器人的拓扑或模型知识，拓扑契约由
  使用方包传入。
- `src/robot_arm_gazebo`：UR5e 六轴模型、Gazebo 集成、`arm` 单总线拓扑契约和
  `robot_arm_motor_sim_node` 运行入口。
- `src/robot_arm_gazebo/config/motors.yaml`：六轴顺序、总线、电机协议、ID、
  限位和仿名义参数的单一来源。
- `launch/`：只保留 `gazebo.launch.py`。
- `docs/adr/`：架构决策记录（append-only）；`docs/superpowers/`：设计与计划
  存档；`docs/verification/`：验收记录。
- `build/`、`install/`、`log/`：colcon 生成物，不提交。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

测试不得要求 Gazebo GUI 或真实串口。串口集成测试使用 PTY；launch 测试只做静态契约
验证。提交前还应确认 `colcon list --names-only` 只输出 `motor_sim` 和
`robot_arm_gazebo`。

## 代码风格

- C++17，2 空格缩进，启用 `-Wall -Wextra -Wpedantic`。
- C++ 文件、函数、变量使用 `snake_case`，类使用 `PascalCase`，常量以 `k` 开头。
- Python 使用 `snake_case` 和类型注解。
- 中文注释说明设计意图、数据流、单位和安全边界；协议名与 ROS 2 API 保留英文。
- 新增电机或调整拓扑优先修改 YAML 和协议策略，不在节点中写死关节数量。

## 运行约束

- 传输模式只有 `usb_serial`、`serial_slcan` 和 `socket_can`，一个进程只能选择一种。
- `arm` 总线在串口模式下必须绑定一个唯一的 `/dev/serial/by-id/...` 路径；
  在 `socket_can` 模式下绑定一个已存在的 SocketCAN 接口名（如 `vcan0`）。
- `serial_slcan` 必须显式设置 `allow_external_canfd:=true`。
- 保持显式 enable、150 ms 命令超时、断线停机、越限/错总线/重复控制器锁存等安全语义。
- 动作播放器只用于模型验收，不应由 launch 启动，也不应与外部电机仿真节点并行控制。
- 仿真参数不得描述为厂家规格或用于推导真机安全结论。

## Git 与文档

功能在 `dev-sim` 开发，提交摘要优先使用 `feat:`、`fix:`、`refactor:`、`docs:`、
`test:`。修改运行入口、六轴 ABI、协议或安全语义时同步更新根 README、
`CONTEXT.md` 和相关测试。
