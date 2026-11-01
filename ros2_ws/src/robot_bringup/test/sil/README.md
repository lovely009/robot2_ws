# 本地半身机器人原型

虚拟串口版复用固件原生内核、CAN 编解码和简化电机模型，通过真实 Micro XRCE-DDS 串口协议连接 micro-ROS Agent。

在仓库根目录启动（当前已验证的本地安装）：

```bash
source /opt/ros/humble/setup.bash
source evidence/cleanup-20261102/install/setup.bash
python3 ros2_ws/src/robot_bringup/test/sil/serial_preview.py
```

需要本机已有的 `socat`、CMake、micro-ROS Agent 以及 PlatformIO 缓存中的 micro-CDR / Micro-XRCE-DDS-Client 源码；脚本离线编译客户端和固件内核，启动所有子进程，Ctrl+C 退出。

- 页面：http://127.0.0.1:8083/workbench/
- WebSocket：ws://127.0.0.1:9093（页面自动配置）
- Agent 串口：`/tmp/robot2-virtual-mcu`
- 仿真固件串口：`/tmp/robot2-virtual-firmware`
- ROS domain：221，仅本机。
- 日志和运行检查：`evidence/virtual-mcu-20261102/`。

链路：网页 ↔ rosbridge ↔ ROS 2 ↔ micro-ROS Agent ↔ PTY 虚拟串口 ↔ XRCE 客户端 ↔ 本机 CDR 数据报 ↔ 固件内核与简化电机模型。电机命令和反馈均经过串口；固件模拟节点不直接发布电机 DDS 话题。

选关节 → 使能所选组 → 输入关节目标 → 发送关节目标。全部软停后可重新使能并发送新目标。已在真实网页验证 `joint_link1` 到 `0.3 rad`、停止、重新使能和到 `-0.1 rad`。模型使用原半身机器人的 18 轴及差动映射，只对预览网格降面。

这是本机固件内核原型，使用 MotorPolicyArray / MotorCommandArray / JointTrajectory 等原型话题，不复刻生产 AgentWireV3 接口，也不运行完整 STM32 / FreeRTOS 镜像。原型保留目标至下次操作或停止，不验证实时调度、真实电机动力学、平衡或硬件时序。

不经过串口的独立版本为 `prototype_preview.py`，使用 HTTP 8082、WebSocket 9092 和 domain 219。两个版本可以同时运行；不要同时启动两个串口版本，它们使用相同端口和 PTY 链接名。
