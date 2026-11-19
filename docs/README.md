# 工程资料

构建与启动见[工程入口](../README.md)，服务和会话参数见[工作台启动](../ros2_ws/src/robot_bringup/README.md)。

## 架构与使用

- [主机控制与同步时间](architecture/host-synchronized-control.md)
- [micro-ROS 通信与执行时序](architecture/microros-timing.md)
- [关节目标跟随与重力补偿](architecture/joint-target-following.md)
- [电机驱动板方向设置与保存](architecture/motor-device-parameter-settings.md)
- [BVH 离线映射](architecture/generic-bvh-retarget.md)

## 测量与模型

- [2026-11-02 同步测量报告](outputs/2026-11-02-clock-sync/report.md)，原始记录位于同目录。
- [半身电机坐标系](../ros2_ws/src/robot_description/models/half_body_robot/cad/frames/README.md)，包含几何基准和待标定项。
- [公共模型目录](../ros2_ws/src/robot_description/README.md)，工程使用的模型和传动配置位于该包。

## 硬件资料

本目录保存巨蟹电机、指尖智擎、DM-MC02 和 CAN FD 适配器的手册、协议、图纸及机械模型。

厂商 SDK、例程、安装包和调试软件归档在仓库外的 `../robot2_ws-reference/vendor/`，按原 `docs/` 目录结构保存。
