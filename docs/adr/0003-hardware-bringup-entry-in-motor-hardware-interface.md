# 真机 bring-up 入口归属 motor_hardware_interface

> **当前适用性（2026-09-11）：有效，入口细节已演进。** 真机 bring-up 仍由 `motor_hardware_interface` 持有；当前包已内置上半身模型 xacro 和默认路径，正文“`model_xacro` 必填”是原始决策阶段的约束。

**Status**: accepted

真机启动入口（`hardware.launch.py`、`driver_controllers.yaml` 与 CAN-FD ros2_control
硬件 xacro 片段）从 `upper_body_canfd_gazebo` 下沉到 `motor_hardware_interface`：真机
部署只依赖底层驱动 + 适配层 + 模型描述，不再被模拟器/Gazebo 的安装依赖绑架。模型描述
（URDF）仍由模型包提供，`hardware.launch.py` 以 `model_xacro` 为必填参数，避免适配层
反向依赖模型包。

## Considered Options

- **保持现状（真机入口留在模型集成包）**：入口与模型放一起顺手，但安装真机链路必然
  拖入 `motor_canfd_sim` 与整套 Gazebo 依赖。
- **下沉到 motor_hardware_interface（已选）**：真机入口与 CAN-FD 适配职责同属一层，
  控制器配置与电机拓扑配置同归 `hw/config`；代价是入口不自带模型描述，调用方必须
  显式传 `model_xacro`。
- **新建轻量 bringup 包**：职责最清晰，但多一个包与一层依赖链，当前规模不值得。

## Consequences

- 真机命令变为 `ros2 launch motor_hardware_interface hardware.launch.py model_xacro:=...`。
- `driver_controllers.yaml` 成为 `hw/config` 的一部分，`driver_loopback` 与真机共用
  同一份控制器配置。
- `upper_body_canfd_gazebo` 回归纯仿真模型/launch 集成，仍负责模型描述。
- CAN-FD ros2_control 硬件片段（`urdf/canfd_hardware.xacro`）与拓扑无关，其他机器人
  模型可直接 include。

> 更新（dev-real 分支）：模型描述已随分支精简移入本包 `urdf/upper_body.urdf.xacro`，
> `model_xacro` 提供包内默认值，不再强制由调用方从模型包传入，见
> [ADR 0005](./0005-dev-real-branch-and-motor-drive-rename.md)。
