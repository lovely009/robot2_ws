# 工程文档

本目录保存跨区域、ROS 2 主机侧与工程治理资料。**固件资料统一在
[`micro_ros_firmware/docs`](../micro_ros_firmware/docs/)，不在两处重复维护。**

## 快速入口

| 需要了解 | 入口 | 状态 |
|---|---|---|
| 三个区域组成与构建 | [根 README](../README.md) | 当前 |
| ROS 2 区域（十一包、bring-up、配置契约） | [ros2_ws/README.md](../ros2_ws/README.md) | 当前 |
| 固件工程事实、话题契约、开放项 | [micro_ros_firmware/README.md](../micro_ros_firmware/README.md) | 当前 |
| 固件操作手册（构建 / 烧录 / 台架） | [micro_ros_firmware/AGENTS.md](../micro_ros_firmware/AGENTS.md) | 当前 |
| micro-ROS Agent 工作区 | [micro_ros_agent/README.md](../micro_ros_agent/README.md) | 当前 |
| 架构决策 | [ADR 索引](adr/README.md) | 当前索引 + 历史正文 |
| 台架操作指南 | [指南索引](guides/README.md) | 当前 |
| ROS 2 驱动约定研究 | [研究索引](research/README.md) | 参考 |
| Agent 即用指南 | [AGENTS.md](../AGENTS.md) | 当前 |

## 分类规则

- `adr/`：影响多个包或区域的架构决策。正文保留作出决策时的上下文；顶部与索引说明当前适用性。
- `research/`：仍有参考价值、但不直接定义当前实现的调研资料。
- `guides/`：当前可重复执行的操作流程。
- `micro_ros_firmware/docs/`：固件 ADR、架构基线（`module-boundaries.md`、
  `motor-runtime-decision-log.md`）与厂商参考资料原件（PDF/XLSX/PNG，不改内文）。
- `ros2_ws/src/<package>/README.md`：包的运行入口、接口、配置与维护边界。
- `micro_ros_firmware/lib/README.md`：库分层与各库契约细则（一层一份）。

## 事实的唯一宿主

每份事实只有一个宿主。这条规则比「多写一份更保险」重要——历史上出过同一数值在两处各自漂移、
以及两份 PDF 物理重复的事故。

| 事实 | 唯一宿主 |
|---|---|
| 区域组成、构建入口 | 各区域 `README.md` |
| 固件工程事实与话题契约 | `micro_ros_firmware/README.md` |
| 固件构建 / 烧录 / 台架顺序与坑 | `micro_ros_firmware/AGENTS.md` |
| 架构决策与当前适用性 | `docs/adr/`、`micro_ros_firmware/docs/adr/` |
| 固件设计理由与变更史 | `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` |
| 固件开放项与阶段门槛 | `micro_ros_firmware/README.md` 的「开放项与开发计划」 |
| 电机拓扑与型号参数 | `motor_hardware_interface/config/*.yaml`（代码不复制） |
| CERB 设备侧对象字典 / 帧 / 故障码 / 保护逻辑 | [`research/cerb-device-reference.md`](research/cerb-device-reference.md) |
| 消息定义 | `micro_ros_agent/src/can_bridge_msgs/`（两处为符号链接，无副本） |

**过程性产物（实施计划、任务台账、逐次运行报告）不进仓库**：结论落进上表的宿主，
过程留在 git 历史里。

## 文档状态

文档顶部或索引使用以下状态：

- **当前**：描述当前工作区，可作为开发入口；
- **提议**：方向已形成，但仍缺实现或验证证据；
- **历史/部分被取代**：保留原始事实，当前开发必须继续查看替代文档；
- **活动计划**：尚有未完成验收项；完成后把结论并入权威文档，过程文档不保留。

实体迁移时优先使用 `git mv`，并同步修复入口、相对链接和命令路径。
