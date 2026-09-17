# FDCAN1 九电机模拟台架指南

> **适用范围（2026-09-11）**：STM32H723 控制板 FDCAN1 + 上游 normaldotcom/canable2 `b158aa7` + 无真实电机的九节点模拟台架。本文只定义稳定启动和只读观察流程；Profile 绑定、使能和运动命令必须另行确认总线无真实电机并取得外部操作权限。

## 连接与固定参数

| 项目 | 值 |
|---|---|
| 控制板 micro-ROS CDC | `/dev/ttyACM0` |
| CANable2 SLCAN | `/dev/ttyACM1` |
| 控制板总线 | FDCAN1，PD0/PD1 |
| 位率 | 仲裁 1 Mbps / 数据 5 Mbps，CAN FD+BRS |
| SLCAN 配置 | `C` → `S8` → `Y5` → `O`，命令间隔至少 150 ms |
| 模拟节点 | CERB id 1–6；VAXOR id 7–9 |

当前 `b158aa7` 只接受 `Y2`/`Y5`；`Y5` 表示 5 Mbps。`Y7` 会静默拒绝并保留默认 2 Mbps。不能用“脚本写出了 Y7”推断设备已经接受配置。

## 启动顺序

顺序不可颠倒：CANable2 必须先成为 ACK 节点，控制板才复位；否则 FDCAN1 可能因无人 ACK 进入 BUS_OFF，而当前固件不会自动恢复。

1. 启动 micro-ROS Agent：

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ly/robot_ws/micro_ros_agent/install/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

2. 启动主机模拟器：

   ```bash
   python3 /home/ly/robot_ws/micro_ros_agent/tools/fdcan_motor_sim.py \
     --tty /dev/ttyACM1 --log /tmp/fdcan-motor-sim.jsonl
   ```

3. 看到 `S8/Y5/O opened` 后，再由人工复位键或已授权的 OpenOCD `reset run` 复位控制板。
4. 模拟器收到首个 `0x080` SYNC 后才开始错峰发送反馈。

## 只读验收

- 模拟器九节点反馈计数应均衡，长期相差不应持续扩大；
- `/micro_ros/motor_topology` 应恰好包含 id 1–9，全部 online、无 conflict；
- OpenOCD 只读寄存器应显示 `TEC=REC=0`、`EP=EW=BO=0`；
- FDCAN2/3 悬空导致的 BUS_OFF 不属于本指南的 FDCAN1 失败；
- 需要降低反馈负载时可用 `--fb-period-ms 50`，默认是 20 ms（50 Hz/节点）。

## 设备参数上电比对（2026-09-17 起）

固件上电会把每台 CERB 的驱动板参数（`0x607D:01/:02`、`0x6080`、`0x2538`）
回读一次，与 W25 里的固件记录**按设备端原值**比对：

- **初始化未完成的那台不可使能**（不一致 `40` / 回读不成功 `41`）：进
  `CONFIG_CONFLICT`、拒绝使能、不出运动帧，`system.last_error` +
  `last_error_motor` 指出是哪一台。台架上看到某台"使能按不下去"时先查这两个参数。
- 复位/换机/改接线后若记录与模拟器默认值不同（模拟器 CERB 默认
  `0x2538=1000 mA`、`0x6080=600 RPM`、位置限 `±32768 cnt`），走一次参数写
  （`ros2 param set /<node> m<id>.torque_limit <N·m>` 等）即可让设备与记录
  重新一致——写事务同时更新设备与记录并自动落盘。
- 运行中（ACTIVE）回读发现不一致走**软停**（阻尼帧、保留使能），不硬失能。
- **设零位**（页面「设零」按钮 / `ros2 param set /<node> system.set_zero <id>`）：写
  0x2531=1，**仅失能态受理**；总线上应看到一条 `23 31 25 00 01 00 00 00`。零位改变后
  设备内 0x607D 限位随之移动，须重认限位与机械行程。**持久化已实测**（2026-09-17）：设零 →
  驱动器断电重上电 → 零位保持，无需额外保存码。
- 设备侧写限值之后，总线上应能看到 `23 39 25 00 01/02/04` 的保存帧——看不到就说明
  落盘没做，驱动器一断电就回出厂值，下次上电会判"不一致 → 未初始化"。
- 回读失败会自动退避重试（≤3 次、间隔 500ms；SDO upload 是只读的，重试不
  改设备任何状态），用尽才判 `41`——偶发一次 `41` 通常重连或再点「回读」即可。
- 想核对设备真值：`ros2 param get /<node> system.last_error`，或看 web 参数页
  「来源」列（驱动板 / 记录）与「状态」列。

## 停止与恢复

Ctrl-C 停止模拟器时脚本发送 `C` 关闭 CANable 通道并输出每节点统计。FDCAN1 已进入 BUS_OFF 时，应先停止旧进程，重新启动模拟器并确认通道已开，再复位控制板；不要只重启 ROS Agent。

## 已验证基线与剩余操作

30 秒九节点基线：`ECR=0`、1–9 全 online、反馈计数 `1546/1545×8`，详见[第二轮报告](../reports/2026-09-11-fdcan1-bench-run2-findings.md)。

以下不属于只读指南：发布 `MotorBind`、`motor_enable`、`motor_command_array`，以及发送任何电机运动目标。继续前必须按[活动计划](../reports/2026-09-11-fdcan1-bench-9motors-goal.md)逐项记录环境和证据。
