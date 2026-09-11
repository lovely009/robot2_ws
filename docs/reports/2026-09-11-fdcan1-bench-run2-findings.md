# 2026-09-11 FDCAN1 九电机台架第二轮复测结论

> 本文是 `2026-09-11-fdcan1-bench-run1-findings.md` 的后续复测与结论修正。
> 第一轮原始数据仍保留，但其中“当前 CANable2 应使用 Y7、问题位于物理层”的判断不再适用。

## 1. 结论摘要

本轮已把两个相互独立的问题定位并修复：

1. **CAN FD+BRS 数据相位位率不匹配**
   - 当前台架 CANable2 已是 normaldotcom/canable2 上游提交 `b158aa7` 对应固件，不是第一轮报告记录的 `16e7497-dirty` 自定义固件。
   - 该上游版本只接受 `Y2` 和 `Y5`；`Y5` 才表示 5 Mbps。未识别的 `Y7` 被静默拒绝，设备不会返回 ACK/NACK，并保留默认 2 Mbps 数据位率。
   - 控制板 FDCAN1 数据相位实测为准确的 5 Mbps，因此原模拟器发送 `Y7` 时实际形成了 **CANable 2 Mbps 对控制板 5 Mbps** 的数据相位错配。
   - 改用 `C / S8 / Y5 / O` 后，单节点和完整九节点 CAN FD+BRS 场景立即由 RED 变 GREEN。
   - **无需修改 STM32 的 PLL2、DBTP 或 TDC。物理链路已由用户核实正确。**

2. **模拟器末节点反馈调度饥饿**
   - 原模拟器用每节点一个标量 deadline；新 SYNC 窗口会覆盖末节点尚未消费的上一窗口 deadline。
   - 该缺陷使 ID 9 长时间只收到极少反馈，并在 ROS topology 中反复 ONLINE/OFFLINE；此时 FDCAN 错误计数始终为 0，故不是总线或固件接收错误。
   - 最终改为每节点 deadline 队列，跨窗口同时保留旧 deadline 和新 deadline；九节点发送计数恢复均衡。

最终 30 秒九节点复测为 **GREEN**：FDCAN 无错误，motor_id 1–9 全部在线，各节点反馈计数相差不超过 1 帧。

---

## 2. 台架条件

- 控制板：STM32H723VGT6
- 控制板接口：FDCAN1，PD0/PD1
- 控制板位率：仲裁 1 Mbps，数据 5 Mbps，CAN FD + BRS
- micro-ROS CDC：`/dev/ttyACM0`
- CANable2 SLCAN：`/dev/ttyACM1`
- 当前 CANable2 固件来源：normaldotcom/canable2 上游 `b158aa7`
- 模拟节点：
  - CERB：motor_id 1–6
  - VAXOR：motor_id 7–9
- 总线上无真实电机
- 固定启动顺序：
  1. 启动 micro-ROS Agent；
  2. 模拟器打开 CANable2，并发送 `C`、`S8`、`Y5`、`O`；
  3. CANable2 已成为 ACK 节点后复位控制板；
  4. 模拟器收到控制板 `0x080` SYNC 后才开始发送反馈。

---

## 3. 根因一：Y 命令随 CANable2 固件版本变化

### 3.1 为什么第一轮的 Y7 结论失效

第一轮报告记录的设备是 `16e7497-dirty` 自定义固件，并据当时现象把 5 Mbps 映射为 `Y7`。本轮重新识别设备后，当前 USB 产品信息指向上游 `b158aa7`。

核对该提交源码可知：

- 支持的数据位率命令只有 `Y2` 和 `Y5`；
- `Y5` 配置 5 Mbps；
- 未识别的 `Y7` 返回失败，但 SLCAN 命令通道不向主机回送可见的 ACK/NACK；
- 设备初始化时默认数据位率为 2 Mbps。

因此，从主机日志看到“写出了 Y7”并不能证明设备接受了 Y7。实际因果链是：

```text
模拟器写 Y7
  -> 当前 b158aa7 不识别，静默拒绝
  -> CANable2 保持默认数据位率 2 Mbps
  -> 控制板仍按 5 Mbps 接收 CAN FD+BRS 数据段
  -> 数据相位错配，控制板累计发送错误并进入 error-passive
```

`motor_drive` 中已有的固定配置 `S8/Y5` 与当前上游 CANable2 固件一致，不应改成 Y7。

### 3.2 控制板位时序复核

OpenOCD 实测：

```text
FDCAN1 DBTP = 0x00000B21
FDCAN1 NBTP = 0x1E003E0F
FDCAN1 TDCR = 0x00000000
RCC D2CCIP1R = 0x20000000
```

RCC 配置选择 PLL2 作为 FDCAN kernel clock，PLL2Q 实际为 80 MHz。由寄存器反算：

- 仲裁段：80 tq/bit，1 Mbps，采样点 80%；
- 数据段：16 tq/bit，5 Mbps，采样点 81.25%，SJW 2 tq；
- TDC 未启用。

这些数值与固件目标一致。Y5 对照场景已在现有硬件链路上稳定通过，因此本次不调整 DBTP、PLL2 或 TDCR。

### 3.3 确定性红绿对照

| 场景 | 结果 | 关键证据 |
|---|---|---|
| 九节点，Y7，FD+BRS | RED | `TEC=128`，`EP=1`，`EW=1`，topology 为空；重复结果一致 |
| 仅 CERB ID 1，Y7，首个 FD+BRS 反馈 | RED | 单帧即可复现 `TEC=128`，排除九节点负载因素 |
| 仅 CERB ID 1，关闭 BRS、全程 1 Mbps | GREEN | `ECR=0`，`TEC=REC=0`，topology 出现 ID 1 |
| 仅 CERB ID 1，Y5，FD+BRS | GREEN | 数据相位改为 5 Mbps 后立即恢复 |
| 九节点，Y5，FD+BRS | GREEN | FDCAN 零错误，1–9 全在线 |

Y7 九节点的典型寄存器：

```text
CCCR=0x00000300
ECR =0x00100080
PSR =0x0000076B  # 也观测到 0x0000077B
TEC=128, EP=1, EW=1
```

无 BRS 对照说明仲裁段、接线和 ACK 路径可工作；Y5 的 FD+BRS 对照进一步把根因收敛到 CANable2 数据位率命令，而非物理链路或 STM32 位时序。

---

## 4. 根因二：跨 SYNC 窗口覆盖末节点 deadline

### 4.1 原始故障

九节点使用 20 ms 反馈周期、2.2 ms 节点错峰时，一个反馈窗口跨度约 17.6 ms。原实现给每个节点只保存一个 deadline：

```text
next_send[motor_id] = window_start + slot * stagger
```

主循环尚未消费 ID 9 的末槽 deadline 时，下一个 SYNC 窗口可能先到达并覆盖该值。约 153 秒实测计数为：

```text
ID 1–7: 各约 7595
ID 8:   7496
ID 9:     95
```

对应现象：ID 9 在 `/micro_ros/motor_topology` 中反复 ONLINE/OFFLINE，但同期 FDCAN `TEC/REC/CEL/EP/EW/BO` 全为 0。

### 4.2 中间修复为何仍不完整

第一版修复使用 `setdefault` 只保留尚未消费的旧 deadline。它避免了覆盖，却没有登记当前窗口的新 deadline，因而 ID 9 约为半速：

```text
ID 1–7: 各约 1545
ID 8:   1542
ID 9:    774
```

### 4.3 最终修复

每节点改为 deadline 列表：

- 每个符合反馈周期的 SYNC 都追加本窗口 deadline；
- 主循环只消费队首到期项；
- 队列为空后删除该节点键；
- 旧窗口未消费项和新窗口项都不会丢失。

同时让原先未生效的 `--fb-period-ms` 真正参与调度：

```text
feedback_sync_interval = round(fb_period_ms / period_ms)
```

默认 `period_ms=10`、`fb_period_ms=20`，即每两个 SYNC 建立一次反馈窗口。

---

## 5. 最终九节点硬件证据

30 秒复测标签：`nine-motors-y5-deadline-queue-30s`

```text
CCCR=0x00000300
ECR =0x00000000
PSR =0x00003008
TEC=0 REC=0 RP=0 CEL=0
EP=0 EW=0 BO=0

topology_ids=[1,2,3,4,5,6,7,8,9]
offline_ids=[]

simulator_feedback_counts={
  1:1546,
  2:1545,
  3:1545,
  4:1545,
  5:1545,
  6:1545,
  7:1545,
  8:1545,
  9:1545
}
balanced=True
VERDICT=GREEN
```

寄存器含义：

- `CCCR.INIT=0`：控制器处于正常运行态；
- `CCCR.FDOE=1`、`CCCR.BRSE=1`：CAN FD 与 BRS 已启用；
- `ECR=0`：TEC、REC、CEL 均为 0；
- `PSR.EP/EW/BO=0`：未进入 error-passive、error-warning 或 bus-off；
- `PSR.RFDF/RBRS=1`：最后接收活动包含 CAN FD+BRS 帧。

另以 `--fb-period-ms 50` 运行约 8 秒，九节点各发送约 176–177 帧，全部在线，FDCAN 错误计数仍为 0，`balanced=True`、`VERDICT=GREEN`。这同时验证了反馈周期参数已生效。

---

## 6. 仓库改动

### 6.1 `micro_ros_agent/tools/fdcan_motor_sim.py`

- 默认数据位率命令由 `Y7` 改为当前上游固件所需的 `Y5`；
- 新增 `--data-bitrate-index`，仅在对应自定义固件时显式覆盖；
- 把 SLCAN 配置序列提取为可回归测试的 helper；
- 把每节点单 deadline 改为 deadline 队列；
- 让 `--fb-period-ms` 实际控制反馈窗口间隔；
- 更新启动日志和使用说明。

### 6.2 host 回归

新增 `test/host/test_fdcan_motor_sim.py` 并接入 CTest，覆盖：

- 默认配置必须是 `C / S8 / Y5 / O`；
- 自定义索引仍可产生 `Y7`；
- 多位数非法索引被拒绝；
- 新 SYNC 窗口既不能覆盖旧 deadline，也不能丢弃本窗口的新 deadline。

完整 host 回归：

```text
12/12 tests passed
0 tests failed
```

构建仍有既有的 enum compare 与 narrowing conversion 警告，本次改动未新增失败。

本轮只修改主机模拟器、host 测试和文档，没有修改 MCU 目标代码，因此未为本轮工具改动重复执行 `pio run`；也未运行 `clean_microros`。

---

## 7. ROS 侧证据与未完成项

已确认以下话题存在：

```text
/micro_ros/imu/data
/micro_ros/imu/temp
/micro_ros/motion_states
/micro_ros/motor_bind
/micro_ros/motor_config
/micro_ros/motor_config_result
/micro_ros/motor_enable
/micro_ros/motor_events
/micro_ros/motor_policy
/micro_ros/motor_status_array
/micro_ros/motor_topology
/motor_command_array
```

调度修复前的样本曾验证：

- CERB ID 1–6 的位置正确解码为 10°、20°……60°对应弧度；
- 未绑定 Profile 时，velocity/effort 为 NaN；
- VAXOR 未绑定 Profile 时，motion state 为 NaN，`state_age_ms=0xFFFFFFFF`；
- ID 9 的离线/恢复与模拟器调度饥饿一致；
- `/micro_ros/motion_states` 当时约为 90.46 Hz。

这些样本采集于最终调度修复前，**不作为最终 ROS 语义验收证据**。最终修复后已经重新确认 FDCAN 寄存器、九节点 online 和反馈公平性，但以下项目尚未完成：

1. 发布 9 条 `MotorBind`：ID 1–6 绑定 `cerb_r58_101`，ID 7–9 绑定 `vaxor_2550`；
2. 捕获 9 个 `PROFILE_BOUND` 事件；
3. 重新测量 `/micro_ros/motion_states` 频率、九路数值和 `state_age_ms`；
4. 检查最终 `/micro_ros/motor_status_array` 与 `/micro_ros/motor_events`；
5. 验证下行 `motor_enable`、`motor_command_array`、CERB `0x110+id`、VAXOR `0x501/0x502+4*id` 及 VAXOR 逐帧应答；
6. 完成长时间稳定性运行。

尝试发布 `MotorBind` 时，运行环境以 `Modify Shared Resources` 权限拒绝，因此未绕过权限继续发送。下行使能或目标命令也未发送。后续执行前仍需确认总线上没有真实电机，并获得对应外部操作权限。

---

## 8. 最终判断

- **物理链路：已核实正确。**
- **STM32 FDCAN1 1M/5M 位时序：实测正确，无需调整。**
- **CANable2 配置：当前上游 `b158aa7` 必须用 Y5；Y7 会被静默拒绝并保留默认 2 Mbps。**
- **九节点 CAN FD+BRS 上行：30 秒全绿，已通过。**
- **模拟器反馈公平性：队列修复后已通过。**
- **完整 ROS Profile/状态语义与下行闭环：尚未验收。**
