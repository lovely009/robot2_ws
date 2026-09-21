# FDCAN1 四电机模拟台架指南

> **适用范围（2026-09-21）**：STM32H723 控制板 FDCAN1 + 当前现场 CANable `16e7497-dirty` + 无真实电机的四节点模拟台架。默认节点为 CERB 1=R58、2–4=R48；若 CANable `V` 返回上游 `b158aa7`，自动配置会切换到 `Y5`。需要旧九节点台架时显式传入 `--cerb 1,2,3,4,5,6 --vaxor 7,8,9`。本文只定义稳定启动和只读观察流程；Profile 绑定、使能和运动命令必须另行确认总线无真实电机并取得外部操作权限。

## 连接与固定参数

| 项目 | 值 |
|---|---|
| 控制板 micro-ROS CDC | `/dev/ttyACM0` |
| CANable2 SLCAN | `/dev/ttyACM1` |
| 控制板总线 | FDCAN1，PD0/PD1 |
| 位率 | 仲裁 1 Mbps / 数据 5 Mbps，CAN FD+BRS |
| SLCAN 配置 | 当前 `V=16e7497-dirty`：`C` → `S8` → `Y7` → `M0` → `A1` → `O`；上游 `b158aa7` 用 `Y5`，命令间隔至少 150 ms |
| 模拟节点 | CERB id 1=R58；id 2–4=R48 |

先用 `V` 识别 CANable 固件：当前现场返回 `16e7497-dirty`，历史台架已验证该自定义版本的 5 Mbps 索引是 `Y7`；上游 `b158aa7` 只接受 `Y2`/`Y5`，其中 `Y5` 表示 5 Mbps，`Y7` 会静默拒绝并保留默认 2 Mbps。不能用“脚本写出了 Y7”推断设备已经接受配置。

## 启动顺序

顺序不可颠倒：CANable2 必须先成为 ACK 节点，控制板才复位；否则 FDCAN1 会因无人 ACK 进入 BUS_OFF。固件自清 INIT 恢复（2026-09-18 已实测），但仍须 ACK 节点在先才能重新同步。

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

3. 看到 `S8/Y7/M0/A1/O opened`（若 V 为 b158aa7 则是 Y5）后，再由人工复位键或已授权的 OpenOCD `reset run` 复位控制板。
4. 模拟器收到首个 `0x080` SYNC 后才开始错峰发送反馈。

## 只读验收

- 模拟器四节点反馈计数应均衡，长期相差不应持续扩大；
- `/micro_ros_up/motor_topology` 应恰好包含 id 1–4，全部 online、无 conflict；
- OpenOCD 只读寄存器应显示 `TEC=REC=0`、`EP=EW=BO=0`；
- FDCAN2/3 悬空导致的 BUS_OFF 不属于本指南的 FDCAN1 失败；
- 需要降低反馈负载时可用 `--fb-period-ms 50`，默认是 60 ms（约 16.7 Hz/节点，必须小于固件离线阈值）。

## 设备参数上电比对（2026-09-17 起）

固件上电会把每台 CERB 的驱动板参数（`0x607D:01/:02`、`0x6080`、`0x2538`）
回读一次，与 W25 里的固件记录**按设备端原值**比对：

- **初始化未完成的那台不可使能**（不一致 `40` / 回读不成功 `41`）：进
  `CONFIG_CONFLICT`、拒绝使能、不出运动帧，`system_status.last_error` +
  `last_error_motor` 指出是哪一台。台架上看到某台"使能按不下去"时先查这两个字段：
  `ros2 topic echo /micro_ros_up/system_status`。
- 复位/换机/改接线后若记录与模拟器默认值不同（模拟器 CERB 默认
  `0x2538=1000 mA`、`0x6080=600 RPM`、位置限 `±32768 cnt`），走一次参数写即可让
  设备与记录重新一致。

  **参数面自 2026-09-21 起是话题，没有 ROS 参数服务器**（`ros2 param set/get` 全部
  失效，web 参数页同期报废）。`field` 用**设备槽序**，与 `device_param_report` 的行序
  同一套：`0`=位置下限 `1`=位置上限 `2`=转速上限 `3`=力矩上限 `4`=保护开关
  `5`=方向。**力矩列的单位是安培**——设备侧权威量是 `0x2538` 的最大电流，折 N·m 要过
  Kt×减速比×效率，该系数本身有未决争议（见 `motor-runtime-decision-log.md` §6.4）。

  ```bash
  # 单字段：改 1 号机的力矩上限（安培）。count 必须显式给，且等于 targets 长度
  ros2 topic pub --once /micro_ros_do/param_set can_bridge_msgs/msg/ParamSetArray \
    "{count: 1, targets: [{motor_id: 1, field: 3, value: 2.5}]}"

  # 位置限位必须在同一条消息内成对（只有一边 → 本地拒绝，last_error=10）
  ros2 topic pub --once /micro_ros_do/param_set can_bridge_msgs/msg/ParamSetArray \
    "{count: 2, targets: [{motor_id: 1, field: 0, value: -3.14},
                          {motor_id: 1, field: 1, value: 3.14}]}"
  ```

  **写 ≠ 保存**：上面的写只改设备 RAM 与固件记录（写后立即重判，写齐即转好）。要
  持久化必须再发一条 SAVE，并等 `save_in_progress` 转 false、`save_seq` 前进才叫
  真落盘（W25 擦写有秒级耗时，此前报"完成"是错的）：

  ```bash
  ros2 topic pub --once /micro_ros_do/system_command can_bridge_msgs/msg/SystemCommand \
    "{command: 0, motor_id: 65535}"       # 0=SAVE；SAVE 只接受 MOTOR_ID_NONE
  ros2 topic echo /micro_ros_up/system_status
  ```

  没保存的台在 `/micro_ros_up/device_param_report` 上是 `record_dirty=true`。
- 运行中（ACTIVE）回读发现不一致走**软停**（阻尼帧、保留使能），不硬失能。
- **设零位**（`command: 2`）：写 0x2531=1，**仅失能态受理**；总线上应看到一条
  `23 31 25 00 01 00 00 00`。零位改变后设备内 0x607D 限位随之移动，须重认限位与机械
  行程。**持久化已实测**（2026-09-17）：设零 → 驱动器断电重上电 → 零位保持，无需额外
  保存码。

  ```bash
  ros2 topic pub --once /micro_ros_do/system_command can_bridge_msgs/msg/SystemCommand \
    "{command: 2, motor_id: 1}"           # 1=REFRESH 回读, 3=CLEAR_FAULT 清故障
  ```

  完成判据同样看 `system_status` 的 `set_zero_seq` / `refresh_seq` / `clear_fault_seq`。
- 设备侧写限值之后，总线上应能看到 `23 39 25 00 01/02/04` 的保存帧——看不到就说明
  落盘没做，驱动器一断电就回出厂值，下次上电会判"不一致 → 未初始化"。
- 回读失败会自动退避重试（≤3 次、间隔 500ms；SDO upload 是只读的，重试不
  改设备任何状态），用尽才判 `41`——偶发一次 `41` 通常重连或再发一条
  `{command: 1, motor_id: <id>}`（REFRESH）即可。
- 想核对设备真值：`ros2 topic echo /micro_ros_up/system_status` 看 `last_error` /
  `last_error_motor`；或看 `/micro_ros_up/device_param_report` 的 `dev_*`（驱动板回读）
  与 `rec_*`（固件记录）两行，以及 `record_dirty`（未落盘）。

## 停止与恢复

Ctrl-C 停止模拟器时脚本发送 `C` 关闭 CANable 通道并输出每节点统计。FDCAN1 已进入 BUS_OFF 时，应先停止旧进程，重新启动模拟器并确认通道已开，再复位控制板；不要只重启 ROS Agent。

## 已验证基线与剩余操作

30 秒九节点基线：`ECR=0`、1–9 全 online、反馈计数 `1546/1545×8`，详见[第二轮报告](../reports/2026-09-11-fdcan1-bench-run2-findings.md)。

以下不属于只读指南：发布 `motor_command_array` 或发送任何电机运动目标（`MotorBind`/`motor_enable` 话题已删除）。继续前必须按[活动计划](../reports/2026-09-11-fdcan1-bench-9motors-goal.md)逐项记录环境和证据。
