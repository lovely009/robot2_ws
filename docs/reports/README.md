# 跨工程报告

本目录保存同时覆盖多个工作区的报告与活动计划。固件开发资料在
[micro_ros_firmware/docs](../../micro_ros_firmware/docs/README.md)。

- [2026-09-11 FDCAN1 九电机台架目标](2026-09-11-fdcan1-bench-9motors-goal.md) — 活动计划：CAN 上行/拓扑已通过，Profile 与下行闭环待验收。
- [2026-09-11 FDCAN1 九电机台架交接](2026-09-11-fdcan1-bench.md) — 历史快照；MotorBind 路线已废弃、BUS_OFF 自清恢复已实现，当前结论以本目录 2026-09-18 重构回归报告为准。
- [2026-09-11 第一轮 FDCAN1 台架记录](2026-09-11-fdcan1-bench-run1-findings.md) — 历史记录；Y7/物理层判断已被第二轮修正。
- [2026-09-11 第二轮复测结论](2026-09-11-fdcan1-bench-run2-findings.md) — 当前 CANable2 `b158aa7` 使用 Y5；九节点 CAN FD+BRS 30 秒 GREEN。
- [2026-09-18 固件高内聚重构九节点回归](2026-09-18-firmware-refactor-regression.md) — 重构行为保持回归（断连撤权/配置事务 golden/持久化/BUS_OFF），真机专项仍开放。
- [2026-09-19 真机 CERB 台架记录](2026-09-19-cerb-real-hardware-bench.md) — `0x2650` 读子索引纠错（`:01` → `:00`）：修前四台真机全部 `DEV_INIT` UNVERIFIED、无法使能；修后 `last_error` 41 → 40、位图点名真实不一致槽位。
- [2026-09-19 参数面容量重构交接](2026-09-19-firmware-param-surface-handoff.md) — **进行中**：`0x2650` 语义已定、`rclc` 参数树 48 台不可行（240 KB > 120 KB 池），决定「块话题配置 + `system.*` 保留 + 状态走话题」，代码半成品。
- [2026-09-20 Web 臂视图与发送侧仪表](2026-09-20-web-arm-view-and-tx-instrument.md) — 下行控制链端到端打通（以模拟器逐帧日志为证）；修掉 `rx_hw_lost`/`tx_frame_count` 未接线互相覆盖（"发送帧率"此前报错数，50 vs 真值 208）；模拟器两处会打死进程的缺陷；Web 上位机改以臂为中心 + URDF 三维臂视图。**未决**：MIT 帧实测每台 ~39.4 Hz，非 200 Hz。
- [2026-09-21 Web 示教下发轨迹无响应交接](2026-09-21-web-teach-trajectory-handoff.md) — **根因三层**：① 四台电机卡 `CONFIG_CONFLICT`（上电时模拟器不在线；§9.3 禁止重连自动解除，只能复位控制板）；② **页面真 bug**：`policyGeneration` 每次开页归零，而固件代数门跨页面持久 → 刷新后每条「武装」被静默拒，轨迹却照旧受理并 `completed`，表现即"播放完成但机构不动"（已修，改用启动时间垫高起点）；③ 模拟器漏 `--gazebo`，从未驱动 Gazebo。端到端已验收：1080 点自动分段 256×4+72，`cerb_mit` 26987 帧，机构跟到 0.05 rad 内。
- [2026-10-09 右臂模型更新与腕部连杆闭环交接](2026-10-09-wrist-linkage-closure-handoff.md) — **已被 2026-10-10 报告取代数值部分**：新 SolidWorks 导出已迁移进 `models/zhoubuguanjie_youbi_001`（12 关节）并通过全部测试与 Gazebo 冒烟（未提交）；腕部"曲柄-连杆（球铰）"机构已反演（闭环切口 + 球铰 + 曲柄自由度三层原因、关键数值、求解公式），唯一卡点是摇臂轴参数，待确认后实施"球铰 + 求解器"；分析工具在同名 `…-assets/` 目录。
- [2026-10-10 腕部连杆闭环实施与两处数值缺陷](2026-10-10-wrist-closure-implementation.md) — 方案 α 已实施（球铰 + 闭式求解器 + RViz 链路，`colcon test` 全绿，端到端误差在浮点噪声量级）。查实两处缺陷：`joint_link9` 原点偏离曲柄端球心约 10 mm（两种方法互证，求解器改用实测球心绕开）；摇臂轴方向与电机输出轴矛盾（band 判据把方向定在离竖直 20° 内，而电机 mesh 回转轴离竖直 75°）。**待用户核对 CAD**：输出法兰法线、摇臂回转中心、销孔中心到轴距离。**未做**：Gazebo 侧闭环驱动（增益是硬件级参数，会影响已验证的肩肘闭环）。
