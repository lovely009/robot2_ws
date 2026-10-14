# FDCAN1 四电机模拟台架指南

> **适用范围（2026-10-14 修订）**：STM32H723 控制板 FDCAN1 + 当前现场 CANable `16e7497-dirty` + 无真实电机的模拟台架。默认节点为 CERB 1=R58、2–4=R48；需要旧九节点台架时显式传入 `--cerb 1,2,3,4,5,6 --vaxor 7,8,9`。**两只已知 CANable 版本的 5 Mbps 索引都是 `Y5`**（2026-10-12 实测修正了 `16e7497-dirty`=Y7 的旧说法，见下）。**固件位时序是冻结的标定值：仲裁 1 Mbps / 数据 5 Mbps，板子侧没有台架变体**——5 Mbps 在本台架收发器/接线余量不足、会周期性整链静默，修法在适配器侧且尚未实施（见下"数据相位"一节）。本文只定义稳定启动和只读观察流程；Profile 绑定、使能和运动命令必须另行确认总线无真实电机并取得外部操作权限。

## 连接与固定参数

| 项目 | 值 |
|---|---|
| 控制板 micro-ROS CDC | `/dev/ttyACM0` |
| CANable2 SLCAN | `/dev/ttyACM1` |
| 控制板总线 | FDCAN1，PD0/PD1 |
| 位率 | 仲裁 1 Mbps / 数据 5 Mbps（固件冻结标定值，板子侧无台架变体），CAN FD+BRS |
| SLCAN 配置 | `C` → `S8` → **`Y5`** → `M0` → `A1` → `O`（`16e7497-dirty` 与 `b158aa7` 均为 `Y5`），命令间隔至少 150 ms |
| 模拟节点 | CERB id 1=R58；id 2–4=R48 |

先用 `V` 识别 CANable 固件。**2026-10-12 实测修正**：现场这只 CANable（USB 描述符
`b158aa7`、`V` 响应 `16e7497-dirty`）的 5 Mbps 索引是 **`Y5`**，历史注释记的 `Y7`
是错的。判据（同硬件、同固件，只换索引做 A/B）：`Y7` → 九台全部 OFFLINE、fault=5，
固件 tx/rx 计数几十秒内冻结；`Y5` → 全部 READY_DISABLED、fault=0、online=true，
长期稳定。`Y7` 的失败指纹是 FDCAN1 `PSR.DLEC=3`（**数据相位** ACK error）——仲裁
相位对、数据相位位率/采样点不匹配。注意 `0x080` SYNC 是 DLC=0 帧、没有数据相位，
所以"能收到 SYNC"或"CAN 分析仪 0 错误"都**不能**证明数据位率正确，必须看带载荷的
FD 帧（DLC 8/12/20）是否畅通。索引与 CANable 健康自愈逻辑见
`micro_ros_agent/tools/fdcan_motor_sim.py` 的 `SLCAN_VERSION_DATA_BITRATE` 与
`_recover_latched_bus`。

## 数据相位：5 Mbps 在本台架余量不足（2026-10-14 定案，修法在适配器侧）

**长期"间歇静默、全台架 OFFLINE、重刷固件无效"的根因就是数据相位时序，不是适配器损坏、
不是模拟器、也不是带宽。** 判据（板子自身单调计数 + 空闲/负载三段对照）：

- 空闲时只发 `0x080`（DLC=0，**没有数据相位**）→ `tx_frame_count` 稳定 50/s、零抖动；
- **一旦板子发带数据相位的 FD 帧（运动命令），总线立刻崩**：`tx_frame_count` 掉到 0、
  `rx_frame_count=0`、18 台 OFFLINE↔ACTIVE 抖动、模拟器看到的 SYNC 最大间隔涨到 4~9 s；
- **与快照率无关**（33/50/100 Hz 表现相同）⇒ 是逐帧层面的失败，不是流量过载；
- 撤掉负载**下一拍就恢复** ⇒ 不是 BUS_OFF 残留、也不是适配器锁存（两者都会持续）；
- `tx_dropped`/`p0_retry`/`rx_hw_lost`/`ring_drop` 全 0，正是"每次提交都被硬件拒绝却看不见"
  的样子（`can_tx_scheduler` 的 `tx_hw_reject_count` 未导出到诊断）。

原理：5 Mbps 下 1 bit 只有 200 ns，与本台架收发器环路延迟同量级；两侧采样点本来就不同
（板子 16 tq → 81.25%，CANable2 上游固件 17 tq → 88.24%，见 `/tmp/canable2-fw/src/can.c`），
余量被吃光。`--no-brs`（只改模拟器方向的帧）无效，可反证问题在板子的发送侧。

**修法（2026-10-14 修订）**：改**适配器**的数据段采样点（`DataTimeSeg1/2` 14/2 → 13/3，
17 tq 不变 → 82.35%，贴近板子的 81.25%）再烧适配器——**不是改板子**：板子 81.25% 与真机
电机的 80% 是匹配的，改板子反而会把真机链路做坏。适配器固件源码见 `/tmp/canable2-fw/src/can.c`。

⚠️ 注意 DBTP 的 `DTSEG1` 只有 5 位、HAL 上界 **32**（仲裁段 `NTSEG1` 是 8 位，别照搬）。

**已删除的错误绕法**：2026-10-14 曾在固件加编译开关 `-DFDCAN_DATA_2M_BENCH` 把数据段降到
2 Mbps（N=2/SJW=2/Seg1=16/Seg2=3 → 20 tq，采样点 85%）绕开本问题，台架实测 100 Hz 快照下
`tx_frame_count` 稳定 4050/s、18/18 ACTIVE、**0 次离线事件**、SYNC 最大间隔 23 ms。当日已把该
开关从 `lib/fdcan/src/fdcan.c` 与 `platformio.ini` 删除：它是往真机标定上打补丁，且必须靠人
记着"接真实电机前关掉"，漏关就把真机链路做坏。**在适配器改好之前，5 Mbps 台架会复现上面的
间歇静默，这是已知未决项**——观察到静默时先按本节判据确认是数据相位，不要去动板子。

## 启动顺序

**顺序不可颠倒，而且"板子先于 ACK 节点启动"会永久卡死**：CANable2 必须先成为 ACK 节点，
控制板才复位。板子若在无 ACK 节点的总线上启动，SYNC 拿不到 ACK → `AutoRetransmission`
重传把三路 TX FIFO 全部填满（诊断显示 `tx_frame_count == 16`，恰好是 FIFO 深度）→ 此后
再也发不出任何帧（`rx_frame_count` 也停在 0），**再复位也好不了**（会立刻重演）。

1. 启动 micro-ROS Agent：

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ly/robot_ws/micro_ros_agent/install/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

2. 启动主机模拟器（板子数据段是 5 Mbps，索引走自动探测 `Y5`；自动探测不上再显式加
   `--data-bitrate-index 5`）：

   ```bash
   python3 /home/ly/robot_ws/micro_ros_agent/tools/fdcan_motor_sim.py \
     --tty /dev/ttyACM1
   ```

   `--log <jsonl>` 会逐帧落盘并把主循环拖慢，反馈窗口可能越过固件 100 ms 离线门而
   触发全体 OFFLINE——只在需要抓原始帧定位时开，抓完即关。

3. 看到 `S8/Y5/M0/A1/O opened`（若数据段配错，`V` 查询与 `PSR.DLEC=3` 会暴露）后，再由人工复位键或已授权的 OpenOCD `reset run` 复位控制板。
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

- ⚠ **2026-10-12：设备参数核对不再是安全门（owner 为台架效率作的取舍）**。
  重构前的语义（2026-09-17 起）：初始化未完成（不一致 `40` / 回读不成功 `41` /
  本机无记录）的那台进 `CONFIG_CONFLICT`、拒绝使能、不出运动帧。**现语义**：只把
  结论如实上报（`dev_init`、`DEV_INIT` 事件位图、`device_param_report`），
  **不再改 lifecycle、不再拒绝使能、不再拦运动帧**。代价：`dev_init ≠ OK` 时该台的
  限位/速度/力矩可能不是核对过的值，兜底只剩硬件急停与机械限位。
  恢复门禁（两处，缺一不可）：`lib/motor_control/src/motor_control_internal.h` 的
  `motor_control_dev_params_allow_motion` 改回 `(dev_init == NA || dev_init == OK)`；
  `lib/motor_control/src/motor_control_config.c` 放开注释掉的
  `freeze_entry_as_conflict(ctx, entry, MOTOR_CONTROL_FAULT_DEV_PARAM, now_ms);`。
  不恢复门禁时，"哪台没核对上"仍可从 `device_param_report` 与 `dev_init` 事件读出。
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
