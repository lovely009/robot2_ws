# 固件消息流收敛为三平面（PDO/SDO）

**Status**: accepted（2026-10-19 用户裁定：全链路可破坏，按此实现）

## Context

固件对外是 **5 个下行 + 10 个上行话题**（`micro_ros_firmware/README.md` 话题契约）。
四个缺陷，每一个都在台架上以具体故障出现过：

1. **同一拍多发 = best-effort 静默丢**。`ros_bridge` 的 10 ms 定时器里按
   `imu_bridge_poll → motor_bridge_poll → param_bridge_poll` 顺序调用，一次回调里可能有
   多个 publisher 各自 `rcl_publish`，而 best-effort 输出流是所有 publisher **共用**的
   一条。`imu/data` 现在是**每拍都发**（100 Hz，`src/imu_bridge.cpp`），所以同拍多发是
   常态不是偶发。`MotorDiagnostics.publish_fail` 这个计数器就是为此加的。
2. **大消息越过 best-effort 可用 MTU**。`UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512`，扣掉
   XRCE 头后可用约 490 B；48 台时 `motion_states`/`status` 就已中招（`MotorDiagnostics.msg`
   注释记着这次踩坑），64 台必然丢。改走 reliable 分片能送达，但吃 `STREAM_HISTORY`
   （16 × 512 B）的窗口——256 点轨迹已经撞过一次（decision-log 2026-09-22）。
3. **循环面与非循环面混在同一层**：高频电平、低频电平、事件、配置事务并列成 topic，
   消费者要自己拼多个流才能得到一致视图。
4. **版本只在 `MotorTopology` 上**，其余消息没有版本位——两端镜像不同步时是**静默按旧
   布局解析**。

设计前核对过的事实（这几条直接决定接线形状）：

| # | 事实 | 出处 |
|---|---|---|
| 1 | best-effort 输出流共用一块静态缓冲；reliable 是另一条 | `micro_ros_firmware/README.md` |
| 2 | `imu/data` 100 Hz 每拍发、`imu/temp` 2 Hz（50 拍一次） | `src/imu_bridge.cpp:97-100` |
| 3 | **`motor_id` ∈ [0,63] 是内核注册表下标**，不是自由 CAN ID | `src/param_bridge.cpp:351`（`motor_id >= MOTOR_CONTROL_MAX_MOTORS`）、`:150`（下标 = motor_id） |
| 4 | 消息族用 `uint64 sync_time_ns` 而非 `std_msgs/Header` | `can_bridge_msgs/msg/*.msg` |
| 5 | 配置面逐目标进深 16 FIFO，是**唯一**这样的下行路径 | `ParamSetArray.msg` 头注释 + `param_bridge.cpp` 的 `static_assert` |
| 6 | `policy` / `trajectory` 各有 latest-wins 邮箱，**不走**配置 FIFO | `control_mailbox.h` |
| 7 | 500 Hz 控制边界归 MCU 独占，主机只发意图（用户裁定） | 本次设计确认 |

## Decision

1. **收敛为三平面**（+ 传感器透传）：循环面 `low_cmd`(100 Hz) / `low_state`(50 Hz)、低频
   状态面 `motor_status`(10 Hz, reliable)、配置与事件面 `config`/`policy`/`trajectory` /
   `topology`/`events`/`device_param`。

2. **`WireContract.msg` 是线上契约的机器可读单一来源**：话题名、定点量程、哨兵、
   `schema_version`、QoS 类别码全部作为 `.msg` 常量，两端只引用生成常量，不写字符串字面量。
   话题命名 `/micro_ros_do/*`（主机→固件）与 `/micro_ros_up/*`（固件→主机）。

3. **循环面永不跨 MTU 分片**：`low_cmd`/`low_state` 用 SoA + int16 定点，64 台时
   序列化 **480 B**（可用约 490 B）。量程 ±12.5 rad / ±64 rad/s / ±64 N·m，−32768 留作
   "本拍无有效测量"哨兵；越界**饱和并置位**，不拒收。

4. **`state_age_ms` 与 `target_source` 让位给低频面**：它们进 `low_state` 会让消息涨到
   680 B（去 ids 改稠密槽位也是 528 B），越界即分片。循环面不分片是硬约束，所以两个
   遥测量降到 `motor_status`。

5. **发布调度：同类一拍一条**。100 拍为一个周期：偶数拍 `low_state`，奇数拍 `imu/data`
   （其中 2 拍让给 `imu/temp`），第 1 拍 `motor_status`，第 5/7/9 拍留给事件/拓扑/按需报告。
   `imu/data` 由 100 Hz 抽稀到 48 Hz 换取偶数拍腾空——`sensor_task` 仍 100 Hz 采样。

6. **主机侧新增 `micro_ros_bridge`**：唯一的编解码节点，也是唯一边界节点。`/micro_ros/*`
   从此只表示线上契约（固件私有），主机侧统一 `/motor/*`（IMU 为 `/imu/*` 透传）。

7. **旧消息一条都不删**：`MotorCommandArray`/`MotorMotionStateArray`/`MotorStatusArray`/
   `MotorTrajectoryState`/`MotorDiagnostics`/`SystemStatus`/`ParamSetArray`/`SystemCommand`
   从线上契约变成**主机 facade 的类型**，`web_host` 的 JS 只改话题前缀、不改字段。
   `can_bridge_msgs` 21 → 26 条。

8. **`schema_version` 进 `motor_status` 与 `topology`；bridge 发现两端不一致时拒绝发命令
   并报错**，不静默按旧布局解析。

9. **本阶段不做 ros2_control 接入**：`SystemInterface` 与 `command_mux` 另立 ADR。它会与
   web / 动作库同时成为命令源，而固件只认 latest-wins 单流，必须先有单一发布者与显式模式
   切换（复用 ADR 0013/0014 的"先切接线、再起停节点"顺序）。

**安全语义一律不变**：软停、30/100 ms 新鲜度级联、轨迹归属仲裁、策略代数门、会话撤权、
P0 安全帧语义全部照旧——本次只换编码与承载，不换裁决。软件兜底仍然**不能**替代硬件急停、
机械限位与真机标定。

**对 ADR 0014 的取代范围**：仅决定 3 里的两个话题名（`/micro_ros_do/motor_command_array`
→ `/motor/command_array`、`/robot_dynamics/command_array` → 主机侧 `/motor/*` 命名）；
改道机制、服务端执行顺序与不变式全部保留。

## Consequences

- 循环面丢帧从"常态"变成"可证伪"：`publish_fail` 归零是验收项，不是推断。
- RAM_D1 净减约 1.9 KB（删四条旧缓冲、加四条新的），94.7% → 约 94.1%。
- meta 槽位反而更宽松（publishers 7/12、subscriptions 4/6），**不需要重建 `libmicroros`**；
  但消息集合变了，必须走 `tools/refresh_microros_messages.py` 离线增量刷新。
- `motor_status` 约 2 KB / 4 分片 @10 Hz，与按需 3 KB `device_param_report` 共用可靠流。
  第一档回退是把 `motor_status` 降到 5 Hz（`SystemStatus` 自己的注释说 5 Hz 是"发命令→
  轮询 seq"的底线，代价是完成信号延迟 200 ms）。
- 新增一处需实测的前提：stream 与 reliable 是否真的各有一条输出流。若证伪，回退到严格
  每拍一条并把 `imu/data` 降到 25 Hz。
- 页面与脚本要改一遍话题名（前缀），但不改数据形状。
- 文档宿主调整：线上话题表的唯一宿主仍是固件 `README.md`；主机话题表的宿主是新的
  `micro_ros_bridge/README.md`；`ros2_ws/README.md` 只做指针。

## Considered Options

- **保留 15 个话题，只调 QoS / 加发布节流**：治不了根因——同拍多发是调度形状问题，大消息
  越 MTU 是编码问题。循环面与非循环面仍然混在一起。否决。
- **换掉 micro-ROS**（自定义二进制 / 以太网 / LCM）：改动面覆盖 agent + 固件 + 全部主机
  消费者，而收益是带宽——瓶颈是调度形状与 MTU 编码，不是带宽（USB CDC 约 1 MB/s，
  `480 B × 100 Hz` 只用 5%）。否决。
- **单一信封 + 全 float32**：64 台 × 4 量 × 4 B = 1024 B > MTU，必然分片。否决。
- **`low_state` 用稠密定长槽位（按 motor_id）带遥测量**：528 B，仍越界。否决。
- **诊断拆成独立按需话题**：话题数回升，且计数器在页面上是常显的。合并进 `motor_status`
  的代价（10 Hz 承载 1 Hz 数据）在 USB 带宽下可忽略。否决。
- **`policy` / `trajectory` 并进 `config`**：邮箱语义不同（latest-wins vs 深 16 FIFO），
  合并会让最多 64 台的策略去挤深 16 的队列。判据是队列语义不是频率。否决。
