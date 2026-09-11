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

## 停止与恢复

Ctrl-C 停止模拟器时脚本发送 `C` 关闭 CANable 通道并输出每节点统计。FDCAN1 已进入 BUS_OFF 时，应先停止旧进程，重新启动模拟器并确认通道已开，再复位控制板；不要只重启 ROS Agent。

## 已验证基线与剩余操作

30 秒九节点基线：`ECR=0`、1–9 全 online、反馈计数 `1546/1545×8`，详见[第二轮报告](../reports/2026-09-11-fdcan1-bench-run2-findings.md)。

以下不属于只读指南：发布 `MotorBind`、`motor_enable`、`motor_command_array`，以及发送任何电机运动目标。继续前必须按[活动计划](../reports/2026-09-11-fdcan1-bench-9motors-goal.md)逐项记录环境和证据。
