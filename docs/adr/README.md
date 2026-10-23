# 架构决策索引

这些 ADR 保留作出决策时的包名、分支和上下文。表中“当前适用性”才是当前工作区的导航；历史正文中的旧包名不是现行目录说明。

> **更名记录（2026-10-13）**：`robot_arm_gazebo` 更名为 `robot_sim`（只改名，内部布局不变：`launch/`、`scripts/`、`models/`、`config/`、`dist/`、`test/`）。
> 历史 ADR 里的旧包名按上一条规则**原样保留**；现行导航文档（`CLAUDE.md`、`AGENTS.md`、各 `README.md`）已改为 `robot_sim`。

| ADR | 原始状态 | 当前适用性（2026-10-09） |
|---|---|---|
| [0001 四包分层与配置单一来源](0001-motor-canfd-layering-and-config-single-source.md) | accepted | **历史、部分被取代**：配置共享被 0002 取代；旧四包结构已演进为当前七包工作区。 |
| [0002 仿真拓扑与真机解耦](0002-sim-topology-config-decoupled-from-hardware.md) | accepted | **原则仍有效，实现已演进**：`motor_sim` 与机器人模型可独立运行；机器人专属联动配置由 `robot_sim` 持有。 （包已更名 `robot_sim`） |
| [0003 真机 bring-up 入口](0003-hardware-bringup-entry-in-motor-hardware-interface.md) | accepted | **当前有效、细节已演进**：入口仍归 `motor_hardware_interface`，模型 xacro 已内置并有默认值。 |
| [0004 激活和回环 fail-fast](0004-fail-fast-on-activation-and-loopback-loss.md) | accepted | **当前有效**：真机激活与运行期反馈超时继续 fail-fast；仿真联动也应在关键进程退出时整体关闭。 |
| [0005 dev-real 分支与更名](0005-dev-real-branch-and-motor-drive-rename.md) | accepted | **历史分支决策、部分被 0006 取代**：`motor_drive` 更名保留；当前 `dev/robot_ws` 已重新包含七包，移除串口结论失效。 |
| [0006 三自适应与默认 tty](0006-dev-real-three-adaptation-and-default-tty.md) | accepted | **当前有效**：transport/protocol/数量配置驱动；生产 tty 仍须按现场设备校准。 |
| [固件 ADR 0007](../../micro_ros_firmware/docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md) | proposed | **固件专属提议**：大部分代码已实现，仍缺最大规模和实时性台架证据。 |
| [0008 robot_dynamics 独立依赖边界](0008-robot-dynamics-dependency-boundary.md) | accepted | **部分被 0009 取代**：Pinocchio 仍只由独立计算包承载，仍不进入 ros2_control 实时循环；「不提供节点/话题」「不进入 Gazebo 运行路径」两条由 0009 取代。 （包已更名 `robot_sim`） |
| [0009 仿真重力前馈的归属与边界](0009-simulation-gravity-feedforward-boundary.md) | accepted | **决定 2、3、5 被 0013 取代**：τ_g 仍由 `robot_dynamics` 计算（决定 1 有效），但实际落地的叠加点是**发往固件的 CAN 命令帧**，不是 Gazebo 的 `/arm_effort_controller/commands`；决定 2 的 `gravity_feedforward_mixer.py` 从未落地，决定 5 的真机路径已由 `00065f7` 落地并被 0013 追认。纯 Gazebo / 纯 MuJoCo / `controller_mode:=host` 三路至今没有重力补偿。 （包已更名 `robot_sim`） |
| [0010 web_host 前端的无构建链分层](0010-web-host-frontend-layering-without-build-chain.md) | accepted | **被 0016 部分取代**：经典界面保持纯静态（原生 ES module + import map），按外壳/屏/面板三层组织，状态集中在 store，DOM 单一来源，可测性由 Python 侧静态契约测试承担。 |
| [0011 仿真后端可选与默认模型收敛](0011-simulation-backend-selection.md) | accepted | **当前有效、模型已更名**：`simulation.launch.py` 增 `backend:=gazebo|mujoco`（后端只替换被控对象入口，其余接线共用）；默认模型收敛到一个模型（`full_body`，2026-10-13 更名为 `half_body_robot` 且关节限位收窄）并删除 `models/robot_all/`；连带修 `fdcan_motor_sim.py` 的命令顺序与 `bench_up.sh` 默认档。与 0009 的关系见该 ADR 的 Consequences。 |

| [0012 电机模拟器收敛为单一后端](0012-single-motor-simulator-backend.md) | accepted | **论据已更正（2026-10-15）**：把 `motor_sim`（C++/ROS）与 `fdcan_motor_sim.py`（Python/HIL）收敛为一个 C++ 后端、一份配置、一个控制面；分两步交付，第二步补故障建模。 |
| [0013 重力前馈的落地边界与 web 端一键开关](0013-gravity-feedforward-ownership-and-web-toggle.md) | accepted | **当前有效（2026-10-16）**：`gravity_feedforward_overlay` 是 τ_g 的唯一落地叠加点，由 `bench_manager` 托管为第四个 unit，web 上位机提供一键开关（仅 STM32 在环时可用）；断链人工一键回退，不自动改接线。取代 0009 的决定 2、3、5。 |
| [0014 命令节拍迁到主机侧](0014-host-side-command-pacing.md) | accepted | **当前有效（2026-10-19）**：命令节拍搬出浏览器——页面只下设定值，100 Hz 快照流由 `command_streamer` 出，随 L1（`web_host.launch.py`）常驻、不做 unit。重力补偿的**命令改道**移入 `bench_manager` 的两个前馈服务（先起 unit 再改道；停止时先改道、确认成功才停 unit），页面只切轨迹并读 `/motor_command/status` 核对；`/client_count` 订阅改 `TRANSIENT_LOCAL`（原来那个"边沿触发"是 QoS 不匹配的产物）。 |
| [0015 固件消息流收敛为三平面](0015-firmware-message-planes-pdo-sdo.md) | accepted | **提议、实现未开始（2026-10-19）**：5 下行 + 10 上行收敛为循环面 / 低频状态面 / 配置事件面三平面，`WireContract.msg` 做话题名与定点量程的单一来源，循环面 480 B 永不分片，主机侧新增 `micro_ros_bridge` 作唯一边界节点（`/micro_ros/*` = 线上契约、`/motor/*` = 主机侧），旧消息保留为 facade 类型。只取代 0014 决定 3 的话题名，改道机制不变。 |
| [0016 Web 工作台采用 Vue 外壳](0016-web-workbench-vue-shell.md) | accepted | **当前有效（2026-10-23）**：A 均衡布局、静态构建部署、原生领域模块迁移适配与浏览器验证，部分取代 0010。 |

## 维护规则

- 修改已 accepted ADR 的决策边界时，新建 ADR 或明确 `superseded-by`，不静默重写历史理由。
- 仅路径、拼写、后续状态和当前实现链接可作非语义修正。
- 当前操作步骤写入 README/指南；实验事实写入报告；未完成验收写入活动计划。
