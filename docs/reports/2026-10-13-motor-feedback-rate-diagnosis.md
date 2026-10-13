# 各电机反馈频率异常 —— 诊断与修复（2026-10-13）

台架：`simulation.launch.py backend:=mujoco transport:=slcan controller_mode:=firmware
model:=half_body_robot`，18 台电机（CERB 1-7/10-16、vaxor 8/9/17/18）由
`fdcan_motor_sim.py` 经一只 CANable2（SLCAN）模拟，STM32H723 跑固件真机路径。

## 1. 症状（基线，6 s / 275 帧）

| 分组 | ID | 协议 | 平均 `state_age_ms` | 最差 |
|---|---|---|---|---|
| A | 1-7, 10-16 | CERB | 82-97 ms | ~420 ms |
| B | 8, 9, 17, 18 | vaxor | 449-486 ms | 1586-1949 ms |

另三条：`motion_states` 45.8 Hz（标称 50）；`motor_status_array` 零消息（标称 20 Hz）；
`velocity`/`effort` 89.4% 为 NaN；`motor_events` 234 Hz。

## 2. 根因

### 2.1 主根因：模拟器反馈窗口被 SYNC 拉长，越过固件 100 ms 离线门

`fdcan_motor_sim.py` 的反馈窗口**只**由 SYNC 计数驱动：每
`feedback_sync_interval = round(fb_period_ms/period_ms) = round(60/20) = 3` 个 SYNC
登记一轮。窗口周期 = 3 × **实际** SYNC 周期，而非 `--fb-period-ms` 这个愿望值。
实测实际 SYNC 节拍远慢于假定的 20 ms：

- 窗口加体检后直接读到：`体检 SYNC=12.6~24.0Hz`，并持续打印
  `SYNC 间隔异常：0.10~0.20s`
- 模拟器写入 4931 B/s ÷ 604 B/窗口 ≈ 8.2 窗口/s ≈ **122 ms**；`tx_dropped=0`
  （证明缺口是窗口被拉长，**不是**发送队列丢帧）
- 固件侧 `rx_frame_count` 增量 160-170 帧/s ÷ 18 台 ≈ **106 ms/台**

而离线门是 `vaxor_adapter.c:295` / `cerb_adapter.c:1075` 的 **100 ms**。
106-122 > 100 → 全台架以 3.2-5.35 Hz 反复
`OFFLINE→RECONNECTED→PROFILE_BOUND→READY`。每次重连执行
`motor_control_discovery.c:143` 的 `entry->profile = NULL`，这是 NaN 的上游。

### 2.2 「vaxor 比 CERB 慢 5 倍」主要是**假象**

对抗性复核推翻了「vaxor 有专属慢路径」这一读法。真正机理是 **age 冻结**：

- `motor_control_discovery.c:196-197` 只在 `has_motion_state && isfinite(position_rad)`
  时才刷新 `last_motion_rx_tick_ms`，而 `state_age_ms = now - last_motion_rx_tick_ms`
- `vaxor_adapter.c:59-64` 在 `profile == NULL` 时把 position/velocity/effort **三者全置 NaN**；
  `cerb_adapter.c:250-259` 的 position 无条件写、只 NaN 速度与力矩
- 于是 vaxor 一旦掉 profile，position 变 NaN → age **不再刷新而单调爬升**，
  呈锯齿（max 1586-1949 ms 是"卡在 NULL 的时长"，不是反馈周期）；
  CERB 的 position 始终有效 → age 持续刷新 → 看起来"快"

即：**两族的帧到达间隔实测一致**（≈103 ms），vaxor 的 450 ms 是龄的假象。
真正需要修的是 2.1 的到达间隔。

### 2.3 `motor_status_array` 零消息：单条消息超 best-effort 流上限

`MotorStatusArray`(18 台) 序列化 **735 B** > `UXR_CONFIG_CUSTOM_TRANSPORT_MTU`
的 **512 B**，且分片只对 reliable 流有效（`48-motor-scale-design.md` §3.0）。
元素 40.83 B，**N ≥ 13 台即越线**——本 18 台台架从未成功发出过一条。
失败静默：话题在、发布者数 1、消息数恒 0，只有 `publish_fail` 以 **18.2/s**
（= 发布尝试率）增长。

### 2.4 `motion_states` 45.8 Hz：tick 实际 ~90 Hz

`micro_ros_task.cpp` 用 `vTaskDelay(pdMS_TO_TICKS(10))`，语义是「做完本轮事再睡
10 ms」，周期 = 工作耗时 + 10 ms ≈ 11 ms → 有效节拍 **~90 Hz**；`motion_states`
取 tick/2 → ~45 Hz。佐证：`imu/data`（tick/1）实测 91 Hz。

## 3. 已排除（均有实测反证）

- 链路带宽饱和（SLCAN 5 KB/s、CDC 上行 73 KB/s，远低于 USB FS 12 Mbps）
- FDCAN 硬件过滤（`can_bsp.c:87-88` 掩码过滤器全通）
- 固件 CAN 收发丢帧（`rx_hw_lost`/`ring_drop`/`decode_error_count` 全 0）
- QoS 误判、发布未实现、发布被门控
- 两发布者打架（`/joint_states` 唯一发布者是 `mujoco_effort_bridge`）

## 4. 修复

| # | 文件 | 改动 |
|---|---|---|
| 1 | `micro_ros_agent/tools/fdcan_motor_sim.py` | 反馈窗口加**墙钟兜底**（距上次窗口 ≥ `fb_period` 即补一轮），使 `--fb-period-ms` 成为周期**上界**而非「SYNC 快才成立」的愿望值；每 5 s 打印在线体检（SYNC 实测 Hz、窗口/s、SYNC 驱动占比、`tx_dropped`、队列深度、SYNC 最大间隔） |
| 2 | `micro_ros_firmware/src/motor_bridge.cpp:698` | `g_status_pub` 的 QoS 由 `qos_stream()` 改 `qos_reliable(1)`（与 `motor_topology` 同判据，见 `ros_qos.h`） |
| 3 | `micro_ros_firmware/src/micro_ros_task.cpp` | `vTaskDelay` → `vTaskDelayUntil` 固定 10 ms 节拍，并加超时保护（滞后达一个周期即重新对齐，避免追帧空转饿死低优先级任务） |
| 4 | `ros2_ws/src/robot_sim/launch/simulation.launch.py` | 默认传 `--fb-period-ms 20`（对齐真机 20 ms 上报契约，`sim_args` 仍可覆盖） |

**改动 3 的坑（首修无效）**：重新对齐分支最初在 `vTaskDelay` **之前**重置
`last_wake`，导致下一轮差值恒 ≥ 周期、永远走该分支，行为退化成原来的
`vTaskDelay(10ms)`——实测 `imu` 仍 90.5 Hz。改为**睡完之后**取时刻才生效。

## 5. 复测结果（修复后，复位板子后稳态）

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `imu/data` | 91.0 Hz | **100.14 Hz** |
| `motion_states` | 45.2 Hz | **50.07 Hz**（帧间隔 18.5-21.2 ms） |
| `motor_status_array` | **0 条** | **20.00 Hz** |
| `publish_fail` | 18.2/s 增长（累计 93416） | **0，不增长** |
| `motor_events` | 234 Hz | **0 Hz** |
| 逐电机均龄 | 82-620 ms | **5.8-16.2 ms** |
| 逐电机最差 | 1586-1949 ms | **23-54 ms** |
| `velocity`/`effort` NaN | 89.4% | 22.2%（见 §6） |
| 在线台数 | 抖动 | **18/18** |

`tx_dropped=0`、`decode_error_count=0`、`ring_drop`/`rx_hw_lost` 全 0。
固件 host 测试 **22/22 通过**；`pio run` RAM 54.6% / Flash 20.7%。

## 6. 遗留（未修，需单独决策）

### 6.1 profile 绑定在启动期非确定性失败

复测中始终有少数电机 `model_name` 为空、其 `velocity`/`effort` 为 NaN（本轮 22.2%，
上一次 33.3%）。**失败集合每次不同**（`[2,3,4,6,12,15]` vs `[2,4,15,16]`），
是启动期 SDO 身份读取（0x1008）握手的竞态，而非特定电机或特定协议。

值得注意：**抖动消失反而让它不再自愈**。`motor_control_config.c:580-589` 有重试，
但 `identity_read_done=true` 后即终局不再问；此前电机反复 OFFLINE→重连会
顺带重置该标志、触发重试，所以看起来"总能绑上"。现在无重连 → 一次失败就永久。

改动重试策略属安全相邻（既有注释明确「认不出的设备不会因为多问几次就认得」，
涉及设备身份与授权边界），**未擅自修改**，建议单独立项。

### 6.2 口径不一致

`MotorMotionStateArray.msg` 注释写「目标频率 100Hz」，与实现（50 Hz）不符。
该注释是 ARD-0007 §16 的目标值，实现有意减半；**未改 .msg**（避免触碰
micro-ROS 类型再生成链路），此处记录差异。

### 6.3 首次修复记录

本报告 §4 改动 3 的"首修无效"过程保留在案，说明**改完必须复测**：
同一次提交里 `motor_status_array` 与反馈窗口都生效了，唯独 tick 的改动
因为一个分支自锁而完全没起作用。
