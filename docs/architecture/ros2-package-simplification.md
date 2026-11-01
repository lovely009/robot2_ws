# ROS 2 功能包精简（2026-10-27）

后续变更：工作台控制已迁往 ros2_control，删除 motor_command；当前源码共 12 包。下文为 2026-10-27 的历史记录，当前控制接线见 [host-trajectory-usage.md](host-trajectory-usage.md)。

## 启动链路

`ros2 launch robot_bringup robot.launch.py` 默认启动五个独立节点：

| 节点 | 提供方 | 职责 |
| --- | --- | --- |
| rosbridge_websocket | web_host/rosbridge_shim | WebSocket，默认 9090 |
| web_static_server | web_host/static_server | 静态页面、文件域 API，默认 8080 |
| command_streamer | motor_command | 主机设定值与 100 Hz 命令快照 |
| micro_ros_bridge | micro_ros_bridge | 主机消息与固件线上协议转换 |
| bench_manager | robot_bringup | 生命周期管理器、/bench/* 服务、按需会话编排 |

管理器按 configure → activate 激活；Agent、仿真、电机后端、重力补偿不是默认常驻进程。
指定 firmware_model 时增加 firmware_topic_bridge 和 robot_state_publisher；
firmware_joint_control 默认 false，反馈链路默认只读。

## 职责评估

原 src 共 13 个可构建包（can_bridge_msgs 为 Agent 工作区的符号链接）。

| 包 | 决策与理由 |
| --- | --- |
| robot_bringup + bench_manager | 合并发行包：同属主机编排，后者还调用前者的 TF launch；保留独立管理器进程和 Python 模块 |
| web_host | 保留：Web 资产与文件域接口，允许单独启动页面 |
| motor_command | 保留：命令节拍与新鲜度逻辑，不应绑进页面或管理器进程 |
| micro_ros_bridge | 保留：固件协议适配，当前与新版固件协议尚未对齐，另行处理 |
| bench_msgs | 保留：生成的 ROS 服务接口，可供客户端独立依赖 |
| can_bridge_msgs | 保留：固件与主机共享的消息契约，不与台架管理服务混合 |
| motor_drive | 保留：不依赖 ROS 的硬件驱动库 |
| motor_hardware_interface | 保留：ros2_control 插件及反馈适配，依赖 motor_drive |
| motor_sim | 保留：可替代硬件的电机仿真后端，不与真实驱动合并 |
| robot_transmission | 保留：跨硬件、仿真和模型消费者共享的传动库及 Python 绑定 |
| robot_dynamics | 保留：Pinocchio 动力学，重依赖且按需使用 |
| robot_sim | 暂保留：模型资产与多仿真入口混合；拆出 description 会增加包数，暂不改模型路径 |

## 本次变更与约束

- 将 bench_manager Python 模块及其测试移入 robot_bringup，合并运行依赖和可执行入口。
- 删除独立 bench_manager ROS 包；新的直接入口为 `ros2 run robot_bringup bench_manager`。
- 保留 `ros2 launch robot_bringup robot.launch.py`、节点名、服务名、消息类型、systemd unit 名与生命周期行为。
- 更新 launch 调用方和 Web 源码契约测试；不改电机命令、保护、超时、固件协议或持久化行为。
- 先在独立安装目录构建，再更新常规安装目录；确认没有 bench 会话后重启入口并验证。

## 验收

1. 合并后的打包测试约束 launch 的包归属、Python 模块与 console_scripts。
2. 管理器全部测试及 bringup 测试通过；管理器测试禁止真实子进程。
3. Web Python 测试通过，确认 /bench 服务与前端契约保持一致。
4. 独立 build/install 中构建 robot_bringup，验证其可执行入口、launch 参数、无 bench_manager ROS 包注册。
5. 独立审查迁移完整性与依赖；不启动第二份管理器，不触发真机会话。

旧 build/bench_manager 与 install/bench_manager 已移至 `/tmp/robot2-bench-manager-premerge-20261027/`，避免旧包注册或导入路径掩盖迁移遗漏。当前常规安装使用 robot_bringup 提供管理器。

## 验证记录

- bringup + Web Python：236 项通过；常规安装环境复测同样通过。
- colcon test robot_bringup：自动发现 147 项，全部通过。
- 独立及常规安装目录构建成功；管理器实现及原测试的 16 个文件逐字节保持。
- 常规入口重启后 bench_manager 为 active，进程路径属于 install/robot_bringup。
- /workbench/ 返回 HTTP 200；/bench/options 返回可访问的 STM32 by-id 串口。
- ROS 包注册与 colcon 源码清单都不再包含独立 bench_manager 包。
- 未触发 Agent、仿真或电机后端会话，未发送电机动作。
- 独立审查：无阻塞，未发现迁移遗漏或打包依赖缺失。
