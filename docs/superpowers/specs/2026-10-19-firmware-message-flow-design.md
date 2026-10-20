# 固件上下行消息流重新设计（PDO/SDO 收敛）

**状态**：提议（2026-10-19 设计经用户逐节确认；实现未开始）
**影响区域**：`micro_ros_firmware`、`micro_ros_agent`、`ros2_ws`
**变更性质**：全链路可破坏——话题名、消息集合、固件发布调度、主机接入层都可改，不背兼容包袱。
**不背包袱的部分**：CAN 侧线协议、`motor_drive`、`motor_sim`、`bench_msgs` 不动。

---

## 1. 背景：四个结构缺陷

当前固件对外是 **5 个下行话题 + 10 个上行话题**（见 `micro_ros_firmware/README.md` 话题契约）。
四个缺陷不是"不够优雅"，每一个都在台架上以具体故障出现过：

1. **同一拍多发，best-effort 静默丢**。`ros_bridge` 的 10 ms 定时器里按
   `imu_bridge_poll → motor_bridge_poll → param_bridge_poll` 顺序调用，一次回调里可能有
   多个 publisher 各自 `rcl_publish`。micro-ROS 的 best-effort 输出流是所有 publisher
   共用的一条（`micro_ros_firmware/README.md`），同拍第二条起就可能落不进去。
   `imu/data` 现在是 **每拍都发**（100 Hz，`src/imu_bridge.cpp`），所以"同拍多发"不是
   偶发，是常态。`MotorDiagnostics.publish_fail` 这个计数器就是为此加的。
2. **大消息越过 best-effort 可用 MTU**。`UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512`，扣掉
   XRCE 头后 best-effort 可用约 490 B。48 台时 `motion_states` / `status` 就已越界
   （`MotorDiagnostics.msg` 的注释记着这次踩坑），64 台必然丢。走 reliable 分片可以送达，
   但吃 `STREAM_HISTORY`（16 × 512 B）的窗口——256 点轨迹已经撞过一次
   （decision-log 2026-09-22）。
3. **循环面与非循环面混在同一层**。高频电平（运动状态、IMU）、低频电平（诊断、系统状态、
   轨迹状态）、事件、配置事务，全是并列的 topic，消费者得自己拼多个流才能得到一致视图。
4. **版本只在 `MotorTopology` 上**。其余消息没有版本位，固件与主机镜像不同步时是**静默按
   旧布局解析**，而不是报错。

---

## 2. 目标与非目标

**目标**

- 每个方向、每种性质，各一条通道；循环面消息**永不跨 MTU 分片**。
- 主机 100 Hz 只发意图；**500 Hz 控制边界与插值/保持/安全级联归 MCU 独占**，主机抖动
  不得影响驱动时序。
- 话题名、定点量程、哨兵、版本、QoS 类别码有**机器可读的单一来源**，两端不可能拼错。
- 主机侧既有消费者（`web_host` 页面、`motor_command`、`robot_dynamics` 叠加、
  `bench_manager` 改道）**不改数据形状**。

**非目标**

- 不改 CAN 侧线协议与 `motor_drive`（协议单位边界不动）。
- 不换掉 micro-ROS / XRCE-DDS（见 §12 被否方案）。
- **本阶段不做 ros2_control 接入**（§6.4 只留缝）。
- 不改安全裁决语义：软停、30/100 ms 新鲜度、轨迹归属、策略代数门、会话撤权一律照旧。

---

## 3. 通道地图（三平面）

| 平面 | 方向 | 消息 | 频率 | QoS | 分片 |
|---|---|---|---|---|---|
| **循环面** | 主机→MCU | `low_cmd` | 100 Hz | stream | 否（480 B） |
| | MCU→主机 | `low_state` | 50 Hz | stream | 否（480 B） |
| **低频状态面** | MCU→主机 | `motor_status` | 10 Hz | reliable | 是（约 4 片） |
| **配置/事件面** | 主机→MCU | `config` | 按需 | reliable | 否 |
| | 主机→MCU | `policy`、`trajectory` | 按需 | reliable | trajectory 是 |
| | MCU→主机 | `topology`、`events`、`device_param_report` | 事件驱动 | reliable | report 是 |
| **传感器面**（透传） | MCU→主机 | `imu/data`（`sensor_msgs/Imu`）、`imu/temp`（`std_msgs/Float32`） | 48 / 2 Hz | stream | 否 |

IMU 是**标准 ROS 类型、无需编解码**，bridge 只做话题改名的透传（§6.2）——它仍走 bridge，
是为了守住"bridge 是唯一边界节点"这条不变式。

**为什么 `policy` / `trajectory` 不并进 `config`**：它们在固件里各有 **latest-wins 邮箱**
（`control_mailbox`），而 `config` 的目标是逐条进深 16 的配置 FIFO
（`CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY`）。合并会让"最多 64 台的策略"去挤一条深 16 的
队列——那正是 `MotorPolicyArray` 当初绕开 FIFO 的理由（`control_mailbox.h` 有注释）。
判据是**队列语义不同**，不是频率不同。

**为什么循环面不带诊断**：`low_state` 要 50 Hz 且不许分片，字节预算算不过来；而
`state_age_ms` / `target_source` 是**遥测**不是控制量，10 Hz 足够。推导见 §4.3。

---

## 4. 消息契约

### 4.1 `WireContract.msg` —— 常量单一来源

新增 `can_bridge_msgs/msg/WireContract.msg`，**只有常量、没有字段**：

```
# 线上契约的机器可读唯一来源。固件与主机 bridge 都只引用这里的生成常量，
# 不各自写字符串字面量 —— 两端拼错话题名这类故障从此不可能发生。
string TOPIC_LOW_CMD      = "/micro_ros_do/low_cmd"
string TOPIC_POLICY       = "/micro_ros_do/policy"
string TOPIC_TRAJECTORY   = "/micro_ros_do/trajectory"
string TOPIC_CONFIG       = "/micro_ros_do/config"
string TOPIC_LOW_STATE    = "/micro_ros_up/low_state"
string TOPIC_MOTOR_STATUS = "/micro_ros_up/motor_status"
string TOPIC_TOPOLOGY     = "/micro_ros_up/topology"
string TOPIC_EVENTS       = "/micro_ros_up/events"
string TOPIC_DEVICE_PARAM = "/micro_ros_up/device_param"
string TOPIC_IMU_DATA     = "/micro_ros_up/imu/data"
string TOPIC_IMU_TEMP     = "/micro_ros_up/imu/temp"

uint16  SCHEMA_VERSION        = 1
uint8   MOTOR_COUNT_MAX       = 64
uint8   QOS_STREAM            = 0
uint8   QOS_RELIABLE          = 1
int16   SENTINEL_MISSING      = -32768
float32 SCALE_POSITION_RAD    = 12.5
float32 SCALE_VELOCITY_RAD_S  = 64.0
float32 SCALE_EFFORT_NM       = 64.0
```

- 话题名**带前导斜杠**：主机 bridge 若被放进 namespace，解析结果仍是同一个绝对名。
  固件现在用的是相对名（`micro_ros_up/imu/data`），在 namespace `/` 下解析结果相同；
  若 `rclc_*_init` 对绝对名有异议，退化为去掉前导斜杠（两端都在 `/` 下，等价）。
- 固件把常量直接用作 `rclc_publisher_init_*` / `rclc_subscription_init_*` 的入参；主机
  bridge 用同一份生成常量。
- QoS 不能作为消息常量类型，但**类别码同源**：两端各自把 `QOS_STREAM` / `QOS_RELIABLE`
  映射到本侧 QoS 对象，主机侧有一条用例断言映射是全的（§8）。
- 生成器风险：**2026-10-19 实测已销**——纯常量 `.msg` 被 rosidl 正常接受
  （`colcon build --packages-select can_bridge_msgs` 无拒绝，11 个话题名 + 6 个标量全部生成）。
  仍保留一个 `uint8 placeholder` 字段：不是为了绕过生成器，而是因为 C 不允许空结构体
  （`rosidl_generator_c` 对无字段消息会生成 `typedef struct { }`）。

### 4.2 `LowCommand.msg`（主机→MCU，100 Hz）

```
uint64 sync_time_ns        # 主机发送时刻，仅用于跨系统日志对齐（沿用现口径）
uint32 sequence            # 诊断/日志关联，不参与裁决（沿用现口径）
uint8  count               # 有效槽位数，必须等于各数组长度
uint8[<=64]  motor_ids     # 槽位顺序；motor_id ∈ [0,63] = 内核注册表下标
int16[<=64]  position_q    # rad
int16[<=64]  velocity_q    # rad/s
int16[<=64]  effort_q      # N·m，前馈 τ_ff
```

序列化尺寸（count = 64，CDR 对齐后）：`8 + 4 + 1 + 3(pad) + (4+64) + 3×(4+128)` =
**480 B** ≤ best-effort 可用 ~490 B。**永不跨 MTU**，这是循环面的硬要求。

### 4.3 `LowState.msg`（MCU→主机，50 Hz）

```
uint64 sync_time_ns
uint32 seq                 # 单调递增；best-effort 无重传，主机据此判丢帧
uint8  count
uint8[<=64]  motor_ids
int16[<=64]  position_q
int16[<=64]  velocity_q
int16[<=64]  effort_q
```

同样 **480 B**。`SENTINEL_MISSING`（−32768）= **本拍无有效测量**（含从未收到），
把"发布频率 ≠ 测量频率"这件事在 50 Hz 上就显式化；精确年龄在 `motor_status` 里给。

**为什么 `state_age_ms` / `target_source` 不放这里**——字节算术不给：

| 方案 | count=64 时 |
|---|---|
| `low_state` 带 ids + 3 量 + `uint16 age` + `uint8 source` | 8+4+1+3+68+396+(4+128)+(4+64) = **680 B** → 必然分片 |
| 去掉 ids、改成按 motor_id 稠密定长槽位（含 age + source） | 8+4+1+3+128×3+64+64 = **528 B** → 仍越界 |
| 只留 3 量（本设计） | **480 B** → 单帧 |

稠密槽位在 64 台时是 528 B，只比 MTU 大一点，但这"一点"就是分片与不分片的分界——
循环面不分片是本次设计的硬约束，所以两个遥测量让位给低频状态面。

### 4.4 `MotorStatus.msg`（MCU→主机，10 Hz，reliable）

合并现有 `MotorStatusArray` + `MotorTrajectoryState` + `SystemStatus` + `MotorDiagnostics`
四条的**全部字段**（字段级沿用现定义，只换承载与频率），并加 `uint16 schema_version`。
结构：

| 段 | 内容 | count=64 时 |
|---|---|---|
| 头 | `sync_time_ns` + `schema_version` + `count` + `motor_ids` | 约 84 B |
| 逐台健康 | `state_age_ms`(uint16，0xFFFF=从未收到)、`target_source`、`lifecycle`、`unified_fault_code`、`vendor_fault_code`、`protection_flags`、`limit_flags` | 约 1.1 KB |
| 逐台电气 | `temperature_dc`(0.1 ℃)、`current_ma`、`voltage_mv`、`drive_mode`、`status_flags`(online/protect_known/protect_enabled/direction_known/direction 五个位)、`bus_index` | 约 0.5 KB |
| 轨迹执行 | `trajectory_id`、`active`、`completed`、`elapsed_ms`、`total_ms`、`traj_target_ids` | 约 148 B |
| 系统 | `last_error`、`last_error_motor`、四个 `*_seq`、`save_in_progress` | 约 24 B |
| 诊断 | 内核计数器 + mailbox 拒绝三个 + `publish_fail` + 每总线 `ring_drop`/`rx_hw_lost`/`tx_frame_count` | 约 116 B |
| **合计** | | **约 2.0 KB → 4 个分片** |

- `state_age_ms` 由 uint32 收窄到 **uint16**：**真实年龄饱和在 `0xFFFE`，`0xFFFF` 独占
  "从未收到"**（不区分就会把 65535 ms 静默读成"从未收到"），省 128 B。
- **不含 `session_state` / `agent_connected`**（2026-10-19 写计划时核对源码后删除）：
  两者都不是"四条旧消息的字段"——`agent_connected` 是内核私有布尔
  （`lib/motor_control/src/motor_control_internal.h`），从未导出到 ROS；`session_state`
  全仓库不存在。facade 的 `SystemStatus` 也无落点，而页面顶栏的 Agent 状态本来就走
  `bench_manager` 的 `/bench/status`。要加是往 `SystemStatus.msg` 末尾追加两个字段的
  **新增遥测**，与本次"只换编码与承载"的收敛目标无关，另议。
- `protection_flags` / `limit_flags` 由 uint32 收窄到 **uint16**（现用 bit0..bit4 / bit0..bit3）。
- 逐台布尔（`device_protect_known` / `device_protect_enabled` / `device_direction_known` /
  `online` / `device_direction`）**打包成 `status_flags` 一个 uint8**：五个 bool 分开写是
  5 × 68 = 340 B，打包是 68 B。位定义写在 `.msg` 头注释里（沿用 `MotorStatusTarget.msg`
  现有的"正向语义布尔"约定，反转仍只允许发生在 `param_bridge` 一处）。
- **`publish_fail` 必须留在这里**：它是"调度是否真的做到一拍一条"的唯一证伪手段（§8）。
- 尺寸门禁：host 用例断言 count=64 时序列化长度 ≤ 2048 B（4 分片）。超了按
  `vendor_fault_code` → `drive_mode` → 逐台电气段的顺序砍，砍哪一段要记进 ADR。

### 4.5 `ConfigRequest.msg`（主机→MCU，reliable）

把 `ParamSetArray` 与 `SystemCommand` 合为一条，用 `kind` 区分、两组字段互斥：

```
uint64 sync_time_ns
uint32 request_id          # bit30/bit31 规则逐字不变
uint8  CONFIG_KIND_PARAM_SET=0
uint8  CONFIG_KIND_SYSTEM_COMMAND=1
uint8  kind
# kind = PARAM_SET：
uint16 count
ParamSetTarget[<=16] targets        # 深 16 的内核约束照旧，static_assert 保留
# kind = SYSTEM_COMMAND：
uint8  command                      # SAVE / REFRESH / SET_ZERO / CLEAR_FAULT / REPORT
uint16 motor_id                     # MOTOR_ID_NONE = 65535
```

**校验**：`kind` 决定哪组字段有效，另一组必须为 0/空，否则**整条拒绝**——沿用"整条受理
或整条拒绝"的现口径。位置限位成对、同一 `(motor_id, field)` 不重复等规则全部照旧。

### 4.6 定点量程与哨兵

| 量 | 量程 | 分辨率 | 说明 |
|---|---|---|---|
| 位置 | ±12.5 rad | 12.5/32767 ≈ **0.38 mrad**（0.022°） | 与 14 位编码器同量级 |
| 速度 | ±64 rad/s | ≈ 1.95 mrad/s | 关节侧 |
| 力矩 | ±64 N·m | ≈ 1.95 mN·m | 关节侧 |

- 编码：`q = clamp(round(x × 32767 / SCALE), -32767, 32767)`；**不使用 −32768**，把它
  留给哨兵。
- 越界**饱和并置饱和位**（`motor_status` 的对应位），不是拒收：超量程说明量程常量选错了，
  应当在台架上看得见，而不是让整条命令消失。
- 单位边界不变：换算只发生在固件的 wire codec 与主机的 codec 里，控制逻辑仍用 rad /
  rad/s / A / N·m。

### 4.7 保留的消息定义

**旧消息一条都不删**。`MotorCommandArray`、`MotorMotionStateArray`、`MotorStatusArray`、
`MotorTrajectoryState`、`MotorDiagnostics`、`SystemStatus`、`ParamSetArray`、`SystemCommand`
等从"线上契约"变成**主机 facade 的类型**（§6.2），`web_host` 的 JS 因此一个字段都不用改，
只改话题名前缀。`can_bridge_msgs` 从 21 条增至 **26 条**（+5）。

---

## 5. 固件侧改动

### 5.1 模块与所有权

| 模块 | 变化 |
|---|---|
| `motor_bridge` | 订阅 `low_cmd`（替 `motor_command_array`）；保留 `policy`、`trajectory`；发布 `low_state`（新）+ `topology`/`events`/`device_param_report`；**删除** `motion_states`、`motor_status_array`、`trajectory_state`、`diagnostics` 四个 publisher 及其静态缓冲 |
| `status_bridge`（新，`src/status_bridge.cpp` + `include/status_bridge.h`） | 唯一 `motor_status` 发布者：聚合 `motor_rt_copy_statuses` / `copy_diagnostics` / `copy_trajectory_state` + `param_bridge_status_snapshot()`。独立成模块而不是塞进 `motor_bridge`（已 857 行），理由是它跨域聚合、且与循环面的发布节拍不同 |
| `param_bridge` | 订阅 2→1（`config`）；**删除** `system_status` publisher；新增只读 `param_bridge_status_snapshot()`（`last_error` + 四个序号 + `save_in_progress` + 会话态） |
| `ros_bridge` | executor handle 6→**5**（1 timer + 3 订阅 + 1 订阅）；轮询顺序 **imu → motor → param → status**（status 最后，吃到本拍的 config 结果与序号） |
| `imu_bridge` | QoS 改 `qos_stream()`；`imu/data` 发布节拍 100 Hz → 落在奇数拍；`imu/temp` 2 Hz 保持 |
| `lib/publish_schedule`（新，纯逻辑） | "本拍该发哪条"的纯函数，无 ROS/RTOS 依赖，直接进 `test/host` |
| `lib/wire_codec`（新，纯逻辑） | 定点编解码 + 饱和 + 哨兵，固件侧唯一换算点，直接进 `test/host` |

### 5.2 发布调度：**每类每拍至多一条**

调度表以 **100 拍（1 s）** 为一个完整周期：

| 拍 | 类 | 消息 | 速率 |
|---|---|---|---|
| 偶数拍 | stream | `low_state` | 50 /s |
| 奇数拍 | stream | `imu/data` | 48 /s（其中 2 拍让给温度） |
| 奇数拍中每 50 拍取 1 | stream | `imu/temp` | 2 /s |
| 第 1 拍（每 10 拍） | reliable | `motor_status` | 10 /s |
| 第 5/7/9 拍（每 10 拍） | reliable | 事件 / 拓扑 / device_param | ≤30 /s（有待发才占用） |

- 不变式：**stream 类每拍至多一条，reliable 类每拍至多一条**。两者是 micro-ROS 里两条
  独立的输出流（`micro_ros_firmware/README.md` 记的是"best-effort 输出流共用一块缓冲"），
  所以同一拍各发一条不冲突。
- **假设与回退**：若台架实测证明两类共用缓冲（表现为 `publish_fail` 在第 1 拍增长），
  回退到严格"每拍一条"——把 `imu/data` 降到 25 Hz 让出奇数拍，其余不动。这条回退写进
  调度表常量旁边，改一个数即可。
- 可靠消息（事件、拓扑、按需参数报告）优先占 5/7/9 空档；当拍没有空档最多延后 10 ms。
- `low_state` 独占全部偶数拍，不会被可靠分片挤掉——这比"谁先调谁发"的隐式顺序可验证。
- 分片由 XRCE 会话在拍间自行排空；"一拍一条"约束的是**发布调用**，不是分片。
- `imu/data` 100 → 48 Hz 是**刻意的抽稀**：`sensor_task` 仍 100 Hz 采样，只在桥上抽稀。
  换取的是偶数拍腾空给 `low_state`。页面与现有消费者都不需要 100 Hz；将来若真需要，
  正确做法是给它独立的测量与调度，而不是恢复同拍多发（记入未决项 §11）。

### 5.3 内存与缓冲

| 动作 | 静态占用（约） |
|---|---|
| 删 `MotorMotionStateTarget[64]` | −1.3 KB |
| 删 `MotorStatusTarget[64]` | −2.8 KB |
| 删 `MotorCommandTarget[64]` | −0.9 KB |
| 删 `MotorTrajectoryState.target_ids[64]` | −0.13 KB |
| 加 `LowCommand` / `LowState` 缓冲 | +0.48 / +0.48 KB |
| 加 `MotorStatus` 缓冲 | +2.0 KB |
| 加 `ConfigRequest` 缓冲 | +0.15 KB |
| **净变化** | **约 −1.9 KB**（RAM_D1 94.7% → 约 94.1%） |

- 每个新消息的静态缓冲**必须在 init 里逐条接线**。`.data` 为 NULL 时写入落到地址 0，
  STM32H7 上不报错——`rx_hw_lost` 曾整整一版在报 `tx` 的数。新设计把这条变成契约：
  `status_bridge_init()` 之后跑一条 host 用例断言所有有界序列子数组 `.data != NULL`。
- 消息尺寸加 `static_assert` 上限（`low_cmd`/`low_state` ≤ 490，`motor_status` ≤ 2048）。
- RAM_D1 百分比以 `pio run` 输出为门禁；超预算按 §4.4 的顺序砍。

### 5.4 校验与安全语义（只换编码，不换裁决）

- `low_cmd`：`count` 必须等于各数组长度；`motor_id` 唯一、**< 64**（= 内核注册表下标，
  与 `param_bridge.cpp` 现有 `motor_id >= MOTOR_CONTROL_MAX_MOTORS` 同口径）、已注册；
  未列出的电机 = **软停**（现口径）；越界定点值饱和并置位（§4.6）。
- `config`：见 §4.5。位置限位成对、`request_id` 规则、逐目标进深 16 FIFO 全部不变。
- `policy` / `trajectory`：消息定义与语义逐字不变，只改话题名。
- **不变**：快照 30/100 ms 新鲜度、轨迹归属仲裁、策略代数门、会话撤权、P0 安全帧语义。
  软件兜底仍然**不能**替代硬件急停、机械限位与真机标定。

### 5.5 构建影响

- **meta 槽位不用改**：publishers 用 7（上限 12）、subscriptions 用 4（上限 6）——比现在
  更宽松，不需要重建 `libmicroros` 的槽位配置。
- 消息集合变了（+5 条类型、固件不再用其中 8 条），必须走**离线增量刷新**
  `micro_ros_agent/tools/refresh_microros_messages.py`；**禁止** `-t clean_microros`（联网重下）。
- 改 `platformio.ini` 宏或 `ldscript/*.ld` 后必须 `pio run -t clean`。

---

## 6. 主机侧接入

### 6.1 新包 `micro_ros_bridge`（ros2_ws）

| 组成 | 内容 |
|---|---|
| `micro_ros_codec` | header-only C++，**无 ROS 依赖**：`encode_low_command` / `decode_low_state` / `decode_motor_status` / `encode_config`，含量程校验、饱和位、哨兵处理。gtest 覆盖边界值 |
| `micro_ros_bridge` | rclcpp 节点，唯一的编解码节点，按 §3 接线 |

用 C++ 而不是 Python：将来的 ros2_control `SystemInterface` 要复用同一个 codec 库。

### 6.2 命名边界

`/micro_ros_do` 与 `/micro_ros_up` 从此**只表示线上契约（固件私有）**；主机侧统一 `/motor/*`。

| 侧 | 话题 |
|---|---|
| 线上（bridge ↔ 固件） | `/micro_ros_do/{low_cmd,policy,trajectory,config}`、`/micro_ros_up/{low_state,motor_status,topology,events,device_param,imu/data,imu/temp}` |
| 主机（页面 / 动作库 / ros2_control 之间） | `/motor/{command_array,motion_states,status_array,topology,events,trajectory_state,diagnostics,system_status,device_param_report,param_set,system_command,policy_array,trajectory}`、`/imu/{data,temp}` |

`micro_ros_bridge` 是**唯一边界节点**：5 条主机下行 → 4 条线上消息；4 条线上消息 → 9 条
主机上行（解码重编码），另加 IMU 两条**透传**（标准 ROS 类型，只改话题名）。
固件改名不再牵动页面与脚本。

### 6.3 受影响清单

| 组件 | 改动 |
|---|---|
| `motor_command` | 只改输出话题名 → `/motor/command_array`；节拍与语义不变。消费者从固件变成主机内的 bridge，QoS 由 stream 改 **reliable depth 1**（主机内不丢，且省掉一条潜在的静默丢路径） |
| `robot_dynamics` overlay | 输入/输出话题改 `/motor/*`；叠加逻辑与 `effort_gain` 不变 |
| `bench_manager` | 改道目标名更新：`command_streamer.output_topic` 在 `/motor/command_array` 与 overlay 输入之间切；改道机制与顺序不变式（先起 unit 再改道、停止时先改道再停 unit）不变 |
| `web_host` | **JS 只改 `core/format.js` 的 TOPICS 前缀**（含 `imu` → `/imu/data`、`temp` → `/imu/temp`）；消息类型与数据形状一个字段都不动 |
| `robot_bringup` | L1 增加 `micro_ros_bridge` 节点（与 `command_streamer` 同层；它是纯 ROS 节点，不碰设备） |
| `micro_ros_agent` | 只重建（`can_bridge_msgs` 变了），无代码改动 |
| `motor_sim` / `fdcan_motor_sim` / `bench_msgs` / `motor_drive` | 不动 |

ADR 0014 决定 3 里的两个话题名（`/micro_ros_do/motor_command_array`、
`/robot_dynamics/command_array`）由本设计取代，**机制不变**——ADR 0015 里写明
"supersede 范围仅限话题名"。

### 6.4 ros2_control：本阶段不做，但把缝留出来

- 500 Hz 归 MCU 已定，所以 ros2_control 只能做**主机侧 100 Hz 的意图层**：
  `SystemInterface` 读 bridge 发布的 `sensor_msgs/JointState`，写 `/motor/command_array`。
- 它会和 web / 动作库**同时成为命令源**，而固件只认 latest-wins 单流。所以必须先有
  `command_mux`（单一 `/motor/command_array` 发布者 + 显式模式切换），复用 ADR 0013/0014
  那套"先切接线、再起停节点"的顺序，不新发明一套。
- 结论：**本次只交付线上契约 + bridge**；`SystemInterface` 与 `command_mux` 另立 ADR，
  等新契约在台架上验过再动。

### 6.5 QoS 契约的防护

- 话题名共用 `WireContract.msg` 常量 → 编译期不可能拼错。
- QoS 类别码同源；固件 `ros_qos.h` 与 bridge 各自映射，主机侧用例断言映射完备。
- `publish_fail` 继续导出（在 `motor_status` 里）——调度是否真做到一拍一条由它证伪。

---

## 7. 迁移步骤

一步一个 commit（conventional 前缀）。S3、S4 各附台架验证记录进 PR 描述。

| 步 | 内容 | 验证 |
|---|---|---|
| **S1 契约层** | 加 `WireContract.msg` + `LowCommand`/`LowState`/`MotorStatus`/`ConfigRequest`；跑离线增量刷新；**旧消息不删** | 两个工作区编过；生成常量在 C/C++ 两侧都可用；纯常量 `.msg` 的生成器风险在此暴露 |
| **S2 主机 codec** | 新包 `micro_ros_bridge`（codec 库 + 节点 + gtest），launch 尚未接线 | `colcon test --packages-select micro_ros_bridge` 全绿 |
| **S3 下行切流** | 固件订 `low_cmd`、`config`；`motor_command` 输出改 `/motor/command_array`；bridge 编码下发；`robot_bringup` L1 加 bridge | **台架**：命令到达、参数写生效、撤权与 30/100 ms 超时行为与改前一致 |
| **S4 上行切流** | 固件发 `low_state` + `status_bridge`；bridge 解码发 `/motor/*`；`web_host` TOPICS 改前缀 | **台架**：页面臂视图/IMU/参数/诊断全有数；`publish_fail` 归零 |
| **S5 清场** | 删固件旧 publisher 与静态缓冲；更新 `ros_qos.h` 注释、固件 README、`micro_ros_agent/README.md` 的消息计数（21→26） | 全套测试 + RAM 门禁 |

S5 之前旧 publisher 仍在，排障时可临时双跑——但双跑会让 best-effort 流翻倍，**只在排障时用**。

---

## 8. 测试与门禁

| 层 | 内容 |
|---|---|
| 固件 host（CTest + cassert，无 gtest） | `lib/publish_schedule` 纯函数：任何拍至多一条 stream、至多一条 reliable；可靠消息不被饿死。`lib/wire_codec`：定点往返、饱和、哨兵。**所有有界序列 `.data != NULL` 接线断言**。消息尺寸 `static_assert`。配置 FIFO 深 16 断言不变 |
| 主机 gtest | codec 边界/哨兵/越界饱和；golden 向量（与固件 host 用例取同值，两边注释互指）；QoS 类别码映射完备；`MotorStatus` count=64 序列化 ≤ 2048 B |
| 仓库 | `colcon test`（ros2_ws）、`ctest --test-dir .host-build`、`pio run` 的 RAM_D1 百分比 |
| 台架（不进 CI） | S3/S4 验收清单 + `publish_fail` / `mailbox_rejected_*` / `rx_hw_lost` 全零 |
| 命名门禁 | 全仓库 grep：`/micro_ros_do/` 与 `/micro_ros_up/` 的**源码字面量**只应出现在 `WireContract.msg`（文档与注释不算）；`web_host` 的 `test_web_assets` 兜住 JS 侧前缀 |

**不启动 Gazebo、不启动 MuJoCo、不占 vcan / 串口 / 真实硬件**——沿用仓库测试规矩。

---

## 9. 风险与回退

| 风险 | 门禁 / 回退 |
|---|---|
| 可靠分片窗口（2026-09-22 教训）：`motor_status` 约 2 KB @10 Hz（4 片）与按需 3 KB `device_param_report` 共用可靠流 | `publish_fail` 与实测送达；第一档降 `motor_status` 到 **5 Hz**（`SystemStatus` 自己的注释说 5 Hz 是"发命令→轮询 seq"的底线，代价是完成信号延迟 200 ms），第二档再拆回独立话题 |
| stream / reliable 是否共用输出缓冲（§5.2 的假设） | `publish_fail` 在第 1 拍是否增长；回退到严格每拍一条 + `imu/data` 25 Hz |
| RAM 是按估算的 −1.9 KB | `pio run` 的 RAM_D1；按 §4.4 顺序砍 |
| 命名切换遗漏（页面 / 脚本 / 文档） | `web_host` 的 `test_web_assets` + 全仓库 grep（§8） |
| 两端镜像不同步（固件 symlink 的包 vs 主机构建的包） | `schema_version` 在 `motor_status`/`topology` 里；bridge 发现不一致时**拒绝发命令并报错**，不静默按旧布局解析 |

**回退**：S1–S4 各自独立 commit，哪步出问题回退哪步；S5 之前旧路径仍在。

---

## 10. 文档落点

| 文档 | 写什么 |
|---|---|
| `docs/adr/0015-*.md`（新，**提交**） | 决策：三平面收敛、契约常量单一来源、MCU 独占 500 Hz、主机 100 Hz 意图层；被否方案；对 ADR 0014 决定 3 的 supersede 范围（仅话题名） |
| `micro_ros_firmware/README.md` | **线上话题契约的唯一宿主**：新话题表、消息预算、调度表、RAM 数字、未决项 |
| `micro_ros_firmware/docs/architecture/module-boundaries.md` | `status_bridge`、`lib/publish_schedule`、`lib/wire_codec` 的层级与所有权、禁止依赖 |
| `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` | §6.x：为何收敛、为何一拍一条、为何 IMU 抽稀、为何 `state_age_ms` 降频 |
| `micro_ros_agent/README.md` | `can_bridge_msgs` 描述里的消息计数与构成（21→26） |
| `ros2_ws/README.md` | 包表加 `micro_ros_bridge`；「分层与命名边界」加线上/主机两级命名规则 |
| `ros2_ws/src/micro_ros_bridge/README.md`（新） | **主机话题表的唯一宿主** + codec 契约 |
| `motor_command` / `web_host` / `robot_bringup` 三份 README | 各自的话题名与节点清单改动 |
| `docs/README.md` | 若新增宿主文档，更新「事实的唯一宿主」表 |

单一来源三条：**机器可读契约 = `WireContract.msg`**；线上话题表 = 固件 README；主机话题表 =
`micro_ros_bridge/README.md`。`ros2_ws/README.md` 只做指针，不复制话题清单。

> 本设计稿与后续实现计划是**过程性产物**，按仓库规则不进仓库（`docs/README.md`
> 「过程性产物不进仓库」，2026-10-14 已清理过一轮 `docs/superpowers/`）。结论落进上表宿主。

---

## 11. 未决项

| 项 | 内容 |
|---|---|
| 未决-01 | 若将来有消费者真需要 IMU 100 Hz：给它独立的调度与测量，不恢复同拍多发 |
| 未决-02 | ros2_control `SystemInterface` + `command_mux`：另立 ADR（§6.4） |
| 未决-03 | ~~纯常量 `.msg` 的生成器兼容性~~ **已于 2026-10-19 实测销项**：rosidl 正常接受，C 侧生成 `static const char * const` / `enum` / `static const float`，C++ 侧生成 `std::string` / `static constexpr`。`placeholder` 字段保留（C 不允许空结构体） |
| 未决-04 | `MotorStatus` 尺寸是估算值（约 2.0 KB / 4 片），以 S1 的 host 尺寸断言为准 |
| 未决-05 | 48 台以上时 `low_cmd`/`low_state` 仍有 10 B 余量；64 台是硬上限，超上限需要新的压缩方案（不在本次范围） |

---

## 12. 被否方案

| 方案 | 否决理由 |
|---|---|
| **保留 15 个话题，只调 QoS / 加发布节流** | 治不了两个根因：同拍多发是调度形状问题，大消息越 MTU 是编码问题。且循环面与非循环面仍然混在一起 |
| **换掉 micro-ROS**（自定义二进制 / 以太网 / LCM） | 改动面覆盖 agent + 固件 + 全部主机消费者，而收益是带宽——当前瓶颈是调度形状与 MTU 编码，不是带宽。USB CDC 有约 1 MB/s，`480 B × 100 Hz` 只用到 5% |
| **单一信封 + 全 float32** | 64 台 × 4 量 × 4 B = 1024 B > MTU，必然分片；循环面不分片是硬约束 |
| **`low_state` 用稠密定长槽位 + 遥测量** | 528 B > MTU（§4.3 表），仍越界 |
| **诊断拆成独立按需话题** | 话题数回升，且计数器在页面上是常显的；合并进 `motor_status` 的代价（10 Hz 承载 1 Hz 数据）在 USB 带宽下可忽略 |
| **`policy` / `trajectory` 并进 `config`** | 邮箱语义不同：latest-wins vs 深 16 FIFO（§3） |
