# dev-real 真机分支：motor_drive 更名与两包精简

**Status**: accepted（`dev-real` 分支专用决策；`develop` 分支不受影响）

## Context

工程此前按“四包单向依赖链”组织（`motor_canfd_drive ← motor_hardware_interface ←
motor_canfd_sim ← upper_body_canfd_gazebo`），仿真回环与真机控制共用一套代码。
随着向真机转移，需要一条干净的长期开发线：只保留真机部署真正需要的底层驱动与
ros2_control 适配层，并让包名反映“驱动不再只处理 CAN-FD 一种传输”的职责边界。

## Decision

- 从 `develop` 派生 `dev-real` 分支，旧 `develop` 保留仿真回环、CANable2 串口台架
  能力与四包结构，两条线各自演进。
- `dev-real` 只保留 `motor_drive` 与 `motor_hardware_interface` 两个 ament 包：
  `motor_canfd_sim`、`upper_body_canfd_gazebo` 从本分支移除。
- `motor_canfd_drive` 全量更名 `motor_drive`：包名、`include/motor_drive/`、
  `namespace motor_drive`、类 `MotorCanfdDriver → MotorDriver`、头文件
  `motor_canfd_driver.hpp → motor_driver.hpp`，消费方引用同步，不留兼容别名。
- 移除真机串口相关：`SerialSlcanTransport`、slcan 编解码、串口单测/fixture、
  台架验证脚本与 `BusConfig` 的 `transport`/`tty` 字段；`motor_drive` 只保留
  SocketCAN 传输（`CanTransport` 抽象保留，供未来扩展其他后端）。
- 机器人模型 `upper_body.urdf.xacro` 移入 `motor_hardware_interface/urdf/` 并去掉
  Gazebo 分支；`hardware.launch.py` 的 `model_xacro` 默认指向包内模型。
- 三份协议手册（25–30 mm 与 RP 系列）移入 `motor_drive/docs/`。

## Considered Options

- **在 develop 上直接更名并删除仿真包**：会破坏 develop 的仿真回环与串口验证能力，
  且未提交的串口工作无归属，故否决。
- **保留旧包名只改目录**：与“驱动支持两套协议”的职责不符，旧名字持续误导读者，
  故否决。
- **保留串口传输代码但不在真机配置中使用**：本分支目标是干净的真机链路，串口
  CANable2 只是台架工具，保留死代码违背精简意图；能力在 develop 有据可查，故移除。
- **模型保留在独立包**：两包结构下无承载包，且真机 bring-up 自足是既有 ADR 0003
  的方向，模型随适配层内置最简。

## Consequences

- `dev-real` 的真机部署链路：`motor_drive` + `motor_hardware_interface` + 内置模型，
  一条 launch 命令启动，无 Gazebo/模拟器依赖。
- `develop` 继续维护仿真回环与串口验证；`motor_canfd_drive` 旧名只在 `develop`
  分支存在，本分支文档不得引用。
- 新增电机家族仍走协议策略注册；`motor_drive` 的 `BusConfig` 只剩
  `name` + `interface_name`，配置/解析/launch 全部回到纯 SocketCAN 语义。
