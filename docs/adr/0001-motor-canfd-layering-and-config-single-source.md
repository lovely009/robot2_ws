# 四包分层与配置单一来源（motor_canfd）

**Status**: accepted；其中“真机与仿真共享同一份 motor_canfd.yaml”部分已被
[ADR 0002](./0002-sim-topology-config-decoupled-from-hardware.md) 取代；
“四包单向依赖链”仅适用于 `develop` 仿真分支，`dev-real` 真机分支只保留
`motor_drive` 与 `motor_hardware_interface` 两包，见
[ADR 0005](./0005-dev-real-branch-and-motor-drive-rename.md)。

14 自由度上半身双臂 CAN-FD 工程按职责分三层、按包分四层单向依赖：`motor_canfd_drive ← motor_hardware_interface ← motor_canfd_sim ← upper_body_canfd_gazebo`。真机与仿真共享同一份 `motor_canfd.yaml`（电机拓扑单一配置源），由 `motor_hardware_interface` 持有并公开解析 API；`motor_canfd_drive` 是不解析 YAML、不依赖 ROS 的纯 C++ 协议库，只接收 `DriverConfig`。

## Considered Options

- **驱动是否解析 YAML**：下沉到 drive 在技术上可行（`yaml-cpp` 不是 ROS 依赖），但会推翻"底层只接收纯 `DriverConfig`"的边界，且偏离 ROS 2 Developer Guide 的 `config/` 归属惯例（参数文件放消费它的包），故否决。
- **sim 是否自行决定电机清单**：最初 sim 自带 YAML 解析并自己决定"挂哪些电机"；改为"虚拟 CAN 电机设备"模型——sim 只接收 `motor_config_file` 路径（由上层 `upper_body_canfd_gazebo` 决定挂多少、挂哪些），复用 `motor_hardware_interface` 的解析 API，消除两份重复解析。
- **4 层 vs 3 层职责**：职责是 3 层（底层驱动 / ROS 抽象层 / 仿真层），仿真层内部拆成模型集成包与电机侧模拟器两个包，形成 4 包依赖链；`motor_canfd_sim` 独立成包并保留自己的独立 launch。

## Consequences

- 新增电机类型只需改 `motor_canfd.yaml` 与 `motor_canfd_drive` 的协议策略，不需要改通用驱动循环或上层应用。
- 真机与仿真共用同一拓扑文件，不会各自漂移；`bus_override.*` 在启动时把物理总线重定向到 vcan。
- 依赖方向与职责一致：`upper_body_canfd_gazebo`（集成）→ `motor_canfd_sim`（虚拟设备）→ `motor_hardware_interface`（ROS 抽象）→ `motor_canfd_drive`（纯库）。
