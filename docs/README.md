# 工程资料

构建与日常使用见[工程入口](../README.md)，启动与控制会话见[工作台启动](../ros2_ws/src/robot_bringup/README.md)。

## 当前设计与验证

- [主机统一时间与 50 Hz 控制流](architecture/host-synchronized-control.md)：当前控制入口、约束和验收记录。
- [micro-ROS 通信与执行时序](architecture/microros-timing.md)：50/50/200 Hz、探测预算、schema 4 合帧与软件验收。
- [同步测量报告](outputs/2026-11-02-clock-sync/report.md)：测量条件和结果；原始记录保留在同目录。
- [半身电机坐标系](../ros2_ws/src/robot_description/models/half_body_robot/cad/frames/README.md)：几何基准与未标定项。

## 硬件参考

本目录保留巨蟹电机、指尖智擎、DM-MC02、CAN FD 适配器等厂商手册、协议、图纸与机械模型。工程使用的模型和传动配置以 `ros2_ws/src/robot_description/` 为准。

不参与工程构建的厂商 SDK、例程、安装包和调试软件已移至仓库外的 `../robot2_ws-reference/vendor/`（相对仓库根目录），保留原来在 `docs/` 下的目录结构。归档不属于工程依赖；需要厂商工具时从该处取用，勿直接运行在连接电机的总线上。

固件调试：PlatformIO 默认停在 `main`，按 F5 后 USB 与固件任务才继续运行；停核导致 USB 枚举超时后需复位并及时继续。状态灯首次等待 Agent 为红色，连接后绿闪，连接过后断开为蓝色。
