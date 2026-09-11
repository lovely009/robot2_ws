# 2026-09-11 FDCAN1 九电机台架交接

> **状态：部分完成。** 九节点 CAN FD+BRS 上行和拓扑稳定性已通过；Profile 绑定后的 ROS 状态、事件和下行闭环仍待验收。
>
> 目标与报告：[活动计划](2026-09-11-fdcan1-bench-9motors-goal.md)、[第一轮记录](2026-09-11-fdcan1-bench-run1-findings.md)、[第二轮结论](2026-09-11-fdcan1-bench-run2-findings.md)。

## 已确认结论

- 控制板：STM32H723VGT6，FDCAN1 使用 PD0/PD1；仲裁 1 Mbps、数据 5 Mbps、CAN FD+BRS。
- 当前 CANable2 对应 normaldotcom/canable2 上游 `b158aa7`，数据位率必须用 `Y5`；`Y7` 会被静默拒绝并保留默认 2 Mbps。
- 主机模拟器已改为每节点 deadline 队列，跨 SYNC 窗口不再覆盖末节点待发送项；`--fb-period-ms` 已实际参与调度。
- 30 秒九节点结果：`ECR=0`，`EP=EW=BO=0`，motor_id 1–9 全部 online，发送计数为 `1546/1545×8`，判定 GREEN。
- 固件 host 回归为 12/12 通过；本轮工具改动没有修改 MCU 目标代码。

## 稳定复现入口

当前步骤见 [FDCAN1 台架指南](../guides/fdcan1-bench-guide.md)。关键时序为：先启动 Agent，再打开 CANable2 并写入 `C/S8/Y5/O`，确认其已成为 ACK 节点后复位控制板；模拟器收到首个 `0x080` SYNC 才发送反馈。

若 FDCAN1 因启动时无人 ACK 进入 BUS_OFF，当前固件不会自动恢复。应停止旧模拟器、重开 CANable2 通道，再复位控制板；只重启 Agent 不足以恢复。

## 尚未完成

1. 发布九条 `MotorBind`：CERB id 1–6 绑定 `cerb_r58_101`，VAXOR id 7–9 绑定 `vaxor_2550`；
2. 捕获九个 `PROFILE_BOUND` 事件；
3. 在最终调度修复后复测 `/micro_ros/motion_states`、`motor_status_array`、事件及 `state_age_ms`；
4. 验证 `motor_enable`、`motor_command_array`、CERB `0x110+id`、VAXOR `0x501/0x502+4*id` 及 VAXOR 逐帧应答；
5. 长时间与三总线最大规模验证。

发布 `MotorBind` 曾被运行环境以 `Modify Shared Resources` 拒绝，未绕过。后续发布或下行操作前必须重新确认总线上无真实电机并取得外部操作权限；不得把本次上行 GREEN 写成完整闭环已经通过。

## 安全边界

- 本交接不授权烧录、真实电机使能或运动；
- 软件停止不能替代急停、机械限位与供电隔离；
- 第一轮报告中的“Y7/物理层”判断仅是历史记录，以第二轮修正为准。
