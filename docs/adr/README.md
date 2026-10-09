# 架构决策索引

这些 ADR 保留作出决策时的包名、分支和上下文。表中“当前适用性”才是当前工作区的导航；历史正文中的旧包名不是现行目录说明。

| ADR | 原始状态 | 当前适用性（2026-10-09） |
|---|---|---|
| [0001 四包分层与配置单一来源](0001-motor-canfd-layering-and-config-single-source.md) | accepted | **历史、部分被取代**：配置共享被 0002 取代；旧四包结构已演进为当前七包工作区。 |
| [0002 仿真拓扑与真机解耦](0002-sim-topology-config-decoupled-from-hardware.md) | accepted | **原则仍有效，实现已演进**：`motor_sim` 与机器人模型可独立运行；机器人专属联动配置由 `robot_arm_gazebo` 持有。 |
| [0003 真机 bring-up 入口](0003-hardware-bringup-entry-in-motor-hardware-interface.md) | accepted | **当前有效、细节已演进**：入口仍归 `motor_hardware_interface`，模型 xacro 已内置并有默认值。 |
| [0004 激活和回环 fail-fast](0004-fail-fast-on-activation-and-loopback-loss.md) | accepted | **当前有效**：真机激活与运行期反馈超时继续 fail-fast；仿真联动也应在关键进程退出时整体关闭。 |
| [0005 dev-real 分支与更名](0005-dev-real-branch-and-motor-drive-rename.md) | accepted | **历史分支决策、部分被 0006 取代**：`motor_drive` 更名保留；当前 `dev/robot_ws` 已重新包含七包，移除串口结论失效。 |
| [0006 三自适应与默认 tty](0006-dev-real-three-adaptation-and-default-tty.md) | accepted | **当前有效**：transport/protocol/数量配置驱动；生产 tty 仍须按现场设备校准。 |
| [固件 ADR 0007](../../micro_ros_firmware/docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md) | proposed | **固件专属提议**：大部分代码已实现，仍缺最大规模和实时性台架证据。 |
| [0008 robot_dynamics 独立依赖边界](0008-robot-dynamics-dependency-boundary.md) | accepted | **部分被 0009 取代**：Pinocchio 仍只由独立计算包承载，仍不进入 ros2_control 实时循环；「不提供节点/话题」「不进入 Gazebo 运行路径」两条由 0009 取代。 |
| [0009 仿真重力前馈的归属与边界](0009-simulation-gravity-feedforward-boundary.md) | accepted | **当前有效**：重力矩由 `robot_dynamics` 计算并发布话题，`robot_arm_gazebo` 在仿真侧叠加到关节力矩；真机 CAN 前馈路径待另立 ADR。 |
| [0010 web_host 前端的无构建链分层](0010-web-host-frontend-layering-without-build-chain.md) | accepted | **当前有效**：`web_host` 前端保持纯静态（原生 ES module + import map），按外壳/屏/面板三层组织，状态集中在 store，DOM 单一来源，可测性由 Python 侧静态契约测试承担。 |

## 维护规则

- 修改已 accepted ADR 的决策边界时，新建 ADR 或明确 `superseded-by`，不静默重写历史理由。
- 仅路径、拼写、后续状态和当前实现链接可作非语义修正。
- 当前操作步骤写入 README/指南；实验事实写入报告；未完成验收写入活动计划。
