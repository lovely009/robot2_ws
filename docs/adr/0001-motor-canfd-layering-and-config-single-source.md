# 历史：四包分层与共享配置

**Status:** superseded（2026-08-25）

本 ADR 记录早期将协议驱动、ROS 硬件适配、电机模拟器和 Gazebo 模型拆成四个包的
方案。该方案服务于同时支持真实机器人和本地 CAN 回环的目标。

当前分支的目标已经收敛为“仅 Gazebo 仿真，并接受外部 USB 或 USB-CANFD 控制”。
因此四包边界、真机配置和本地回环均被移除，协议、串口、传动和电机仿真先并入
`humanoid_gazebo`，后又抽取为独立的 `motor_sim` 包；`humanoid_gazebo`
及其设计文档已于 2026-08-26 移除，现行架构见 `motor_sim` 与
`robot_arm_gazebo` 的源码和根 README。

保留本记录仅用于解释 Git 历史，不应据此新增 package 或运行入口。
