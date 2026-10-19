# 固件消息流（一）：契约层与编解码器 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把新的线上契约（5 条消息 + 常量单一来源）与两侧的定点编解码器建起来，全程**加法式改动、不动任何现有行为**。

**Architecture:** 契约常量与消息定义落在唯一权威包 `can_bridge_msgs`；固件侧加一个纯 C 的 `lib/wire_codec`（无 ROS 依赖、host 可直编）与一个只放编译期断言的 `src/wire_contract_check.cpp`；主机侧新建 `micro_ros_bridge` 包，内含 header-only 的 C++ codec 与一个**尚未接线**的桥节点。两侧 codec 用同一组 golden 向量互相钉住。

**Tech Stack:** ROS 2 Humble（rosidl / ament_cmake / gtest）、PlatformIO + STM32duino（C17 / C++17）、CMake + CTest（固件 host 测试）。

**Spec:** `docs/superpowers/specs/2026-10-19-firmware-message-flow-design.md`（§4 消息契约、§6.1–6.2 主机侧、§7 的 S1–S2、§8 测试门禁）
**ADR:** `docs/adr/0015-firmware-message-planes-pdo-sdo.md`

## Global Constraints

- 消息定义**唯一权威**在 `micro_ros_agent/src/can_bridge_msgs/`；`ros2_ws/src/can_bridge_msgs` 是指向它的符号链接，**不要**在 ros2_ws 下编辑消息。
- 定点公式：`q = clamp(round(x * 32767 / SCALE), -32767, 32767)`；**不使用 −32768**，它留给 `SENTINEL_MISSING`。越界**饱和**，不拒收。
- 量程：`SCALE_POSITION_RAD=12.5`、`SCALE_VELOCITY_RAD_S=64.0`、`SCALE_EFFORT_NM=64.0`。
- `MOTOR_COUNT_MAX = 64`，且 `motor_id ∈ [0, 63]`（= 内核注册表下标，不是自由 CAN ID）。
- 循环面 `low_cmd`/`low_state` 在 count=64 时**序列化必须 ≤ 490 B**（best-effort 可用 MTU），永不跨 MTU 分片。
- 单位边界：codec 只做定点↔SI 换算，控制逻辑仍用 rad / rad/s / A / N·m。
- 改 `.msg` 后**必须**用 `python3 micro_ros_agent/tools/refresh_microros_messages.py` 增量刷新；**禁止** `pio run -t clean_microros`（联网重下）。
- 测试**不得**启动 Gazebo、不得开 MuJoCo 窗口、不得占 vcan / 串口 / 真实硬件。
- 提交用 conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因。
- 一份事实只写一处；过程性产物（本计划）不入库。
- 本计划**不接线任何 launch、不改任何现有固件桥的行为**。改动只有加法。

---

## File Structure

| 文件 | 职责 |
|---|---|
| `micro_ros_agent/src/can_bridge_msgs/msg/WireContract.msg` | 新增。话题名、量程、哨兵、schema 版本、QoS 类别码的常量单一来源 |
| `micro_ros_agent/src/can_bridge_msgs/msg/LowCommand.msg` | 新增。循环面下行（主机→MCU，100 Hz） |
| `micro_ros_agent/src/can_bridge_msgs/msg/LowState.msg` | 新增。循环面上行（MCU→主机，50 Hz） |
| `micro_ros_agent/src/can_bridge_msgs/msg/MotorStatus.msg` | 新增。低频状态面（MCU→主机，10 Hz reliable），合并四条旧消息的字段 |
| `micro_ros_agent/src/can_bridge_msgs/msg/ConfigRequest.msg` | 新增。配置/系统命令面（主机→MCU） |
| `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt` | 改。把 5 条新消息加进 `rosidl_generate_interfaces` |
| `micro_ros_firmware/lib/wire_codec/{include,src}/` | 新增。纯 C 定点编解码（无 ROS 依赖，host 可直编） |
| `micro_ros_firmware/src/wire_contract_check.cpp` | 新增。只放编译期断言：尺寸门禁 + 常量形态核对，无运行期代码 |
| `micro_ros_firmware/test/host/CMakeLists.txt` | 改。注册 `fw_wire_codec` OBJECT 库与 `test_wire_codec` |
| `ros2_ws/src/micro_ros_bridge/` | 新增包。codec 库 + 桥节点 + gtest + README |
| `ros2_ws/README.md` | 改。包表加一行 |

---

## Task 1: `WireContract.msg` —— 常量单一来源

**Files:**
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/WireContract.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt`（`rosidl_generate_interfaces` 清单）

**Interfaces:**
- Consumes: 无
- Produces: `can_bridge_msgs::msg::WireContract::TOPIC_LOW_CMD` 等 11 个 `std::string` 常量、
  `SCHEMA_VERSION` / `MOTOR_COUNT_MAX` / `QOS_STREAM` / `QOS_RELIABLE` / `SENTINEL_MISSING` /
  `SCALE_POSITION_RAD` / `SCALE_VELOCITY_RAD_S` / `SCALE_EFFORT_NM`；C 侧对应
  `can_bridge_msgs__msg__WireContract__TOPIC_LOW_CMD` 等。

- [ ] **Step 1: 写消息文件**

字符串常量的值**不写引号**（rosidl 的 `value_to_c` 会自己加；写引号会让引号进入值里）。
`placeholder` 字段不是装饰：C 不允许空结构体，而 `rosidl_generator_c` 对无字段消息照样生成
`typedef struct { }`。

```
# 线上契约的机器可读唯一来源：话题名、定点量程、哨兵、schema 版本、QoS 类别码。
# 固件与主机 bridge 都只引用这里生成出来的常量，不各自写字符串字面量。
#
# 方向：do = 主机→固件，up = 固件→主机。
#
# 本消息**从不发布**，只作常量载体；placeholder 的唯一作用是避免空结构体。
#
# 定点：q = clamp(round(x * 32767 / SCALE), -32767, 32767)；-32768 留给 SENTINEL_MISSING。
# 单位边界：换算只发生在两侧 codec 里，控制逻辑仍用 rad / rad/s / A / N·m。

string TOPIC_LOW_CMD=/micro_ros/do/low_cmd
string TOPIC_POLICY=/micro_ros/do/policy
string TOPIC_TRAJECTORY=/micro_ros/do/trajectory
string TOPIC_CONFIG=/micro_ros/do/config
string TOPIC_LOW_STATE=/micro_ros/up/low_state
string TOPIC_MOTOR_STATUS=/micro_ros/up/motor_status
string TOPIC_TOPOLOGY=/micro_ros/up/topology
string TOPIC_EVENTS=/micro_ros/up/events
string TOPIC_DEVICE_PARAM=/micro_ros/up/device_param
string TOPIC_IMU_DATA=/micro_ros/up/imu/data
string TOPIC_IMU_TEMP=/micro_ros/up/imu/temp

# 消息面契约版本：随字段**语义**变更递增（字段只增不改序）。
uint16 SCHEMA_VERSION=1
# 单条消息可携带的最大电机数（= 内核 MOTOR_CONTROL_MAX_MOTORS）。
uint8 MOTOR_COUNT_MAX=64
# QoS 类别码：0 = 高频流（best-effort depth 1）；1 = 低频状态声明（reliable）。
uint8 QOS_STREAM=0
uint8 QOS_RELIABLE=1
# 定点哨兵：本拍无有效测量。
int16 SENTINEL_MISSING=-32768
# 定点量程（full-scale，双极性）。
float32 SCALE_POSITION_RAD=12.5
float32 SCALE_VELOCITY_RAD_S=64.0
float32 SCALE_EFFORT_NM=64.0

uint8 placeholder
```

- [ ] **Step 2: 加进 rosidl 清单**

在 `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt` 的 `rosidl_generate_interfaces(${PROJECT_NAME}` 参数清单里，
按现有行的缩进格式加一行：

```cmake
  "msg/WireContract.msg"
```

- [ ] **Step 3: 构建并核对生成形态**

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs
```

预期：构建成功。然后核对两处生成物（路径里的 `can_bridge_msgs/can_bridge_msgs` 是 include 根 + 包名，不是笔误）：

```bash
H=/home/ly/robot_ws/ros2_ws/install/can_bridge_msgs/include/can_bridge_msgs/can_bridge_msgs/msg
grep -n "TOPIC_LOW_CMD\|SCALE_POSITION_RAD\|SENTINEL_MISSING" $H/detail/wire_contract__struct.h
grep -n "TOPIC_LOW_CMD\|SCALE_POSITION_RAD" $H/detail/wire_contract__struct.hpp
```

预期（C）：
```c
static const char * const can_bridge_msgs__msg__WireContract__TOPIC_LOW_CMD = "/micro_ros/do/low_cmd";
enum { can_bridge_msgs__msg__WireContract__SENTINEL_MISSING = -32768 };
static const float can_bridge_msgs__msg__WireContract__SCALE_POSITION_RAD = 12.5;
```
预期（C++）：`static const std::string TOPIC_LOW_CMD;` 与
`const std::string WireContract::TOPIC_LOW_CMD = "/micro_ros/do/low_cmd";`

**值里不能出现引号**。若出现（`""/micro_ros/do/low_cmd""`），说明 .msg 里误写了引号，去掉重来。

- [ ] **Step 4: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs/msg/WireContract.msg \
        micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt
git commit -m "feat(can_bridge_msgs): 新增 WireContract —— 线上契约常量的单一来源

话题名、定点量程、哨兵、schema 版本与 QoS 类别码集中成一份 .msg 常量，
两端只引用生成常量，消掉「拼错话题名」与「量程各写一份」两类故障。
本消息从不发布，只作常量载体。"
```

---

## Task 2: 循环面两条消息 `LowCommand` / `LowState`

**Files:**
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/LowCommand.msg`
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/LowState.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/msg/MotorMotionStateArray.msg`（追加 `seq`，见下）
- Modify: `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 的 `WireContract`（尺寸门禁在 Task 5 用它核对）
- Produces: `can_bridge_msgs::msg::LowCommand` / `LowState`，字段
  `sync_time_ns`(uint64) / `sequence`|`seq`(uint32) / `count`(uint8) /
  `motor_ids`(uint8[≤64]) / `position_q` / `velocity_q` / `effort_q`(int16[≤64])；
  `MotorMotionStateArray` 末尾追加 `uint32 seq`；
  C 侧 `can_bridge_msgs__msg__LowCommand__motor_ids__MAX_SIZE = 64`。

- [ ] **Step 1: 写 `LowCommand.msg`**

```
# 循环面下行：100 Hz 完整命令快照（ARD-0007 §15 的定点化重写）。
#
# 完整快照语义不变：受控集合中**未出现**的电机 = 软停；任何 NaN/Inf → 整条拒收。
# 定点与量程见 WireContract.msg（唯一来源）；越界饱和，不拒收。
#
# 尺寸：count=64 时序列化 480 B ≤ best-effort 可用约 490 B。本消息永不跨 MTU 分片，
# 这是循环面的硬约束 —— 改字段前先重算尺寸。
uint64 sync_time_ns
# 诊断/日志关联，不参与裁决。
uint32 sequence
# 有效槽位数，必须等于各数组长度。
uint8 count
# 槽位顺序的电机 ID；motor_id ∈ [0,63] = 内核注册表下标（不是自由 CAN ID）。
uint8[<=64] motor_ids
# rad
int16[<=64] position_q
# rad/s
int16[<=64] velocity_q
# N·m，前馈 τ_ff
int16[<=64] effort_q
```

- [ ] **Step 2: 写 `LowState.msg`**

```
# 循环面上行：50 Hz 运动状态（ARD-0007 §16 的定点化重写）。
#
# 只含运动量。state_age_ms 与 target_source 刻意不放这里：带上它们会越过 MTU 而被迫分片，
# 循环面不分片是硬约束（ADR 0015 决定 4）；两者在 MotorStatus 里给。
#
# SENTINEL_MISSING（-32768）= 本拍无有效测量（含从未收到），把「发布频率 ≠ 测量频率」
# 在 50 Hz 上显式化。
uint64 sync_time_ns
# 单调递增；best-effort 无重传，主机据此判丢帧。
uint32 seq
# 有效槽位数，必须等于各数组长度。
uint8 count
# 槽位顺序的电机 ID；motor_id ∈ [0,63] = 内核注册表下标。
uint8[<=64] motor_ids
# rad
int16[<=64] position_q
# rad/s
int16[<=64] velocity_q
# N·m
int16[<=64] effort_q
```

- [ ] **Step 3: 给 facade 的 `MotorMotionStateArray` 末尾追加 `seq`**

`LowState.seq` 是主机侧唯一的丢帧判据（best-effort 无重传），而 facade 的
`MotorMotionStateArray` 只有 `sync_time_ns` + `states`——不补一个字段，这个判据在主机侧
就**没有落点**，解码时只能丢掉。往末尾追加（加法，不改序，页面 JS 忽略未知字段）：

```
# 运动状态（ARD-0007 §16，MCU → 上层，目标频率 100Hz）。
# 只含运动量；诊断/故障走 MotorStatusArray（高频与低频分离，ARD §14.2）。
uint64 sync_time_ns
MotorMotionStateTarget[<=64] states
# 与线上 low_state.seq 同源：best-effort 无重传，上层据此判丢帧；追加于末尾以保留现有字段顺序。
uint32 seq
```

- [ ] **Step 4: 加进 rosidl 清单并构建**

在 `CMakeLists.txt` 里加：

```cmake
  "msg/LowCommand.msg"
  "msg/LowState.msg"
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs
H=install/can_bridge_msgs/include/can_bridge_msgs/can_bridge_msgs/msg
grep -n "MAX_SIZE" $H/detail/low_command__struct.h
grep -n "int16_t position_q\|uint8_t count\|uint32_t seq" $H/detail/low_state__struct.h
grep -n "uint32_t seq" $H/detail/motor_motion_state_array__struct.h
```

预期：`can_bridge_msgs__msg__LowCommand__motor_ids__MAX_SIZE = 64`、
`...__position_q__MAX_SIZE = 64`；`LowState` 的 `seq` 是 `uint32_t`（与 `LowCommand.sequence` 不同名，
故意的——一个是丢帧判据、一个是日志关联）；`MotorMotionStateArray` 末尾出现 `uint32_t seq;`。

- [ ] **Step 5: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs/msg/LowCommand.msg \
        micro_ros_agent/src/can_bridge_msgs/msg/LowState.msg \
        micro_ros_agent/src/can_bridge_msgs/msg/MotorMotionStateArray.msg \
        micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt
git commit -m "feat(can_bridge_msgs): 新增循环面 LowCommand / LowState

SoA + int16 定点，count=64 时各 480 B，落在 best-effort 可用 MTU 内。
带 state_age_ms / target_source 会涨到 680 B 而被迫分片，故两者让位给
低频状态面；low_state 用哨兵表达「本拍无有效测量」。
facade 的 MotorMotionStateArray 末尾追加 seq：low_state.seq 是主机侧唯一的
丢帧判据，不给它落点就等于解码时丢掉。"
```

---

## Task 3: 低频状态面 `MotorStatus`

**Files:**
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/MotorStatus.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 的 `WireContract.SCHEMA_VERSION`
- Produces: `can_bridge_msgs::msg::MotorStatus`，含逐台健康/电气段、轨迹执行段、系统段、诊断段；
  枚举取值沿用保留消息：`MotorMotionStateTarget::TARGET_SOURCE_*`、
  `MotorStatusTarget::LIFECYCLE_*`、`SystemCommand::MOTOR_ID_NONE`

- [ ] **Step 1: 写 `MotorStatus.msg`**

枚举**不重新定义**：取值沿用仍在包里的 `MotorMotionStateTarget.msg`（`TARGET_SOURCE_*`）与
`MotorStatusTarget.msg`（`LIFECYCLE_*`），两端代码直接引用那些生成常量——一份事实一处。

```
# 低频状态面：10 Hz 合并状态（reliable，允许分片）。
#
# 合并 MotorStatusArray + MotorTrajectoryState + SystemStatus + MotorDiagnostics
# 四条的全部字段；旧消息保留为主机 facade 的类型。
#
# 逐台布尔打包成 status_flags，位语义沿用 MotorStatusTarget.msg 的「正向语义布尔」
# 约定（0x2650 的反转只允许发生在固件 param_bridge 一处）。
#
# 枚举取值不在此定义：target_source 见 MotorMotionStateTarget.msg，lifecycle 见
# MotorStatusTarget.msg，motor_id 缺省见 SystemCommand.msg。
uint64 sync_time_ns
uint16 schema_version
uint8 count
uint8[<=64] motor_ids

# ---- 逐台：健康 ----
# 0xFFFF = 从未收到运动测量；真实年龄饱和在 0xFFFE（≈65.5 s），不与哨兵共用——
# 共用就会把 65535 ms 静默读成"从未收到"。解码端把 0xFFFF 还原成 0xFFFFFFFF。
uint16[<=64] state_age_ms
# 枚举取值见 MotorMotionStateTarget.msg。
uint8[<=64] target_source
uint8[<=64] lifecycle
uint16[<=64] unified_fault_code
uint32[<=64] vendor_fault_code
# bit0 厂家故障 bit1 过温 bit2 过流 bit3 欠压 bit4 帧解码错误
# （唯一定义点：motor_protocol_adapter.h）
uint16[<=64] protection_flags
# bit0 软限位下限钳制 bit1 软限位上限钳制 bit2 速度钳制 bit3 扭矩超峰钳制
uint16[<=64] limit_flags

# ---- 逐台：电气 ----
# 0.1 ℃；INT16_MIN = 无来源
int16[<=64] temperature_dc
# 电机侧 Iq，mA；INT16_MIN = 缺失
int16[<=64] current_ma
# mV；0xFFFF = 缺失
uint16[<=64] voltage_mv
# 厂家模式字节透传
uint8[<=64] drive_mode
# bit0 online bit1 device_protect_known bit2 device_protect_enabled
# bit3 device_direction_known bit4 device_direction(0=CW 1=CCW，原值直通)
uint8[<=64] status_flags
uint8[<=64] bus_index

# ---- 轨迹执行状态（全局一份） ----
uint32 trajectory_id
bool traj_active
bool traj_completed
uint32 traj_elapsed_ms
uint32 traj_total_ms
uint16 traj_target_count
uint16[<=64] traj_target_ids

# ---- 系统状态（字段与 SystemStatus.msg 一一对应，解码端直接搬） ----
uint16 last_error
uint16 last_error_motor
uint32 refresh_seq
uint32 set_zero_seq
uint32 clear_fault_seq
uint32 save_seq
bool save_in_progress

# ---- 诊断计数器（单调递增的生命周期累计，上层自己求差分） ----
uint32 rx_frame_count
uint32 unknown_frame_count
uint32 decode_error_count
uint32 snapshot_accepted
uint32 snapshot_rejected
uint32 config_rejected
uint32 event_dropped
uint32 tx_dropped
uint32 p0_retry
uint32 trajectory_accepted
uint32 trajectory_rejected
uint32 policy_accepted
uint32 policy_rejected
uint32 mailbox_rejected_command
uint32 mailbox_rejected_trajectory
uint32 mailbox_rejected_policy
# 出站发布失败总数。调度是否真做到「同类一拍至多一条」由它证伪，不靠推断。
uint32 publish_fail
uint32[<=3] ring_drop
uint32[<=3] rx_hw_lost
uint32[<=3] tx_frame_count
uint32 heap_free
```

**与 spec §4.4 的三处偏差（已核对源码，不是偷懒）**：

- `session_state` / `agent_connected` **不进这条消息**。spec 把它们列进"系统段"，但它们
  不属于"合并四条旧消息"的范围——`agent_connected` 是内核私有布尔
  （`lib/motor_control/src/motor_control_internal.h`），从未导出到 ROS；`session_state`
  全仓库不存在。facade 的 `SystemStatus` 也没有对应字段，解码端无处安放，而页面顶栏的
  Agent 状态本来就走 `bench_manager` 的 `/bench/status`。要加就是往 `SystemStatus.msg`
  **末尾追加**两个字段（加法，不改序），但那属于新增遥测，不是本次收敛的一部分。
- 逐台布尔打包成 `status_flags` 时，`online` / `device_protect_*` / `device_direction*`
  的**正向语义约定**照旧，位序见上面的注释，唯一定义点仍是 `motor_protocol_adapter.h`。
- `state_age_ms` / `target_source` 在**主机侧不落进 `MotorStatusTarget`**。spec §4.7 要求
  旧消息保留为 facade 类型且「字段级沿用现定义」，往 `MotorStatusTarget` 加字段会破坏
  这条；而页面读的是 **motion_states 元素**上的这两个字段
  （`web_host/web/js/panels/readout.js:34`「测量年龄」、`arm.js:270` 的 `target_source`），
  spec §4.7 又承诺「JS 一个字段都不用改」。两条约束一起只能得出：由 bridge 缓存
  （Task 9 的 `MotorFreshness` 出参 + Task 10 的 `freshness_`），在 50 Hz 填充
  `motion_states` 时盖章。**本任务只负责线上消息里带上它们**，主机侧落点见 Task 9/10。

- [ ] **Step 2: 加进 rosidl 清单并构建**

```cmake
  "msg/MotorStatus.msg"
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs
```

预期：构建成功（无循环依赖——`MotorStatus` 不引用其他消息类型，全部是内建类型）。

- [ ] **Step 3: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs/msg/MotorStatus.msg \
        micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt
git commit -m "feat(can_bridge_msgs): 新增低频状态面 MotorStatus

合并 MotorStatusArray / MotorTrajectoryState / SystemStatus / MotorDiagnostics
四条的字段，10 Hz reliable。逐台布尔打包成 status_flags、state_age_ms 与两个
flags 字段收窄，把 64 台时的体积压到约 2 KB（4 分片）。枚举取值沿用保留消息，
不重新定义。"
```

---

## Task 4: 配置面 `ConfigRequest`

**Files:**
- Create: `micro_ros_agent/src/can_bridge_msgs/msg/ConfigRequest.msg`
- Modify: `micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt`

**Interfaces:**
- Consumes: 既有 `ParamSetTarget`（复用其 `FIELD_*` 常量与单位口径）
- Produces: `can_bridge_msgs::msg::ConfigRequest`，字段
  `sync_time_ns` / `request_id` / `kind` / `count` / `targets`(ParamSetTarget[≤16]) /
  `command` / `motor_id`

- [ ] **Step 1: 写 `ConfigRequest.msg`**

```
# 配置/系统命令面：ParamSetArray 与 SystemCommand 合一，用 kind 区分。
# 两组字段互斥：kind 决定哪组有效，另一组必须为 0/空，否则整条拒绝（沿用
# 「整条受理或整条拒绝」的旧口径）。
# 取值不在此定义：command 见 SystemCommand.msg 的 COMMAND_*，motor_id 缺省见其 MOTOR_ID_NONE。
# targets 上界 16 是内核约束不是带宽约束（CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY = 16）。
uint64 sync_time_ns
# 关联 id 不是去重键：0 = 固件代分配；bit30 = 固件自发脉冲命名空间（置位被本地拒绝）；
# bit31 已被内核身份读取占用。
uint32 request_id
uint8 CONFIG_KIND_PARAM_SET=0
uint8 CONFIG_KIND_SYSTEM_COMMAND=1
uint8 kind
# kind = CONFIG_KIND_PARAM_SET 时有效
uint16 count
ParamSetTarget[<=16] targets
# kind = CONFIG_KIND_SYSTEM_COMMAND 时有效（取值 = SystemCommand.msg 的 COMMAND_*）
uint8 command
# MOTOR_ID_NONE(65535) = 不适用
uint16 motor_id
```

- [ ] **Step 2: 加进 rosidl 清单并构建**

```cmake
  "msg/ConfigRequest.msg"
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs
H=install/can_bridge_msgs/include/can_bridge_msgs/can_bridge_msgs/msg
grep -n "MAX_SIZE\|ParamSetTarget" $H/detail/config_request__struct.h | head
```

预期：出现 `can_bridge_msgs__msg__ConfigRequest__targets__MAX_SIZE = 16` 与
`can_bridge_msgs__msg__ParamSetTarget__Sequence targets;`。

- [ ] **Step 3: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_agent/src/can_bridge_msgs/msg/ConfigRequest.msg \
        micro_ros_agent/src/can_bridge_msgs/CMakeLists.txt
git commit -m "feat(can_bridge_msgs): 新增配置面 ConfigRequest

把 ParamSetArray 与 SystemCommand 合成一条，用 kind 区分、两组字段互斥。
targets 上界 16 沿用内核配置 FIFO 容量约束；command / motor_id 取值沿用
SystemCommand 的常量，不重新定义。"
```

---

## Task 5: 固件侧契约门禁（尺寸与常量形态）

**Files:**
- Create: `micro_ros_firmware/src/wire_contract_check.cpp`
- Modify: 无（`pio run` 自动编译 `src/*.cpp`）

**Interfaces:**
- Consumes: Task 1–4 生成的 C 头（`can_bridge_msgs/msg/detail/*__struct.h` 的 `MAX_SIZE` 枚举与常量）
- Produces: 编译期门禁。无运行期符号。

- [ ] **Step 1: 先刷新固件的 micro-ROS 类型**

固件的 `libmicroros` 里现在**没有**这 5 条新消息的头（`pio run` 不会自动重生成类型）：

```bash
cd /home/ly/robot_ws
python3 micro_ros_agent/tools/refresh_microros_messages.py
ls micro_ros_firmware/.pio/libdeps/stm32h723vgt6/micro_ros_platformio/libmicroros/include/can_bridge_msgs/msg/ | grep -E "wire_contract|low_command|low_state|motor_status|config_request"
```

预期：五条 `detail/` 头都在。若报"缺少 micro-ROS 基础环境"，说明 PlatformIO 缓存不完整——
**不要**退化成联网重建，先跑一次 `pio run` 把缓存补齐（那一步允许联网）。

- [ ] **Step 2: 写门禁 TU**

**本 TU 只断言整型常量表达式**。rosidl 给 C 侧生成的是 `static const float`（浮点量程）
与 `static const char * const`（话题名），**两者都进不了 `static_assert`**：C++ 的
[expr.const] 只放行 `constexpr` 对象与 const 整型/枚举对象，`static const float` 不在其中。
这一条已在真工具链（`arm-none-eabi-g++` 12.3.1、`-std=gnu++17`）上按五种候选形态实测确认，
没有可用形态——不要试图绕过。量程与话题名的值由 Task 7 的主机侧契约用例按**精确值**钉住
（那里的 C++ 头生成的是 `constexpr`）。本文件因此只留尺寸与枚举常量断言。

```cpp
// 线上契约的编译期门禁。本文件**没有运行期代码**，只有 static_assert：
// 尺寸越界在这里让 pio run 直接失败，而不是等台架上丢帧才发现。
//
// 尺寸算术用的是 CDR 序列化布局（每个有界序列 4 B 长度前缀 + 元素），不是
// sizeof()——后者含 data/size/capacity 三元组，比线上多 32 B，拿它当门禁没有余量。

#include <can_bridge_msgs/msg/detail/config_request__struct.h>
#include <can_bridge_msgs/msg/detail/low_command__struct.h>
#include <can_bridge_msgs/msg/detail/low_state__struct.h>
#include <can_bridge_msgs/msg/detail/motor_status__struct.h>
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>

namespace {

// CDR：uint64 + uint32 + uint8 + 3 B 对齐填充，然后 4 个有界序列。
constexpr size_t kLowCommandWire =
    8 + 4 + 1 + 3
    + (4 + can_bridge_msgs__msg__LowCommand__motor_ids__MAX_SIZE)
    + 3 * (4 + 2 * can_bridge_msgs__msg__LowCommand__position_q__MAX_SIZE);

constexpr size_t kLowStateWire =
    8 + 4 + 1 + 3
    + (4 + can_bridge_msgs__msg__LowState__motor_ids__MAX_SIZE)
    + 3 * (4 + 2 * can_bridge_msgs__msg__LowState__position_q__MAX_SIZE);

// best-effort 可用载荷：UXR_CONFIG_CUSTOM_TRANSPORT_MTU(512) 扣掉 XRCE 头。
constexpr size_t kBestEffortPayloadMax = 490;
// CDR 封装头（representation identifier + options）4 B。上面两个只算消息体，
// 线上实际占用要加上它——漏算这 4 B 会让门禁比真实余量宽。
constexpr size_t kCdrEncapsulation = 4;

static_assert(kLowCommandWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_cmd 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");
static_assert(kLowStateWire + kCdrEncapsulation <= kBestEffortPayloadMax,
    "low_state 越出 best-effort 可用 MTU：循环面不允许分片，请先砍字段");

// 枚举常量是整型常量表达式，可以直接断言。
static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == -32768,
    "哨兵值变了：定点编解码两端都会错");
static_assert(can_bridge_msgs__msg__WireContract__MOTOR_COUNT_MAX == 64,
    "电机数上界与内核 MOTOR_CONTROL_MAX_MOTORS 脱钩了");

// 量程与话题名在这里**故意不设断言**：rosidl 给 C 侧生成的是 `static const float` 与
// `static const char * const`，两者都不是 C++ 常量表达式（[expr.const] 只放行 constexpr
// 与 const 整型/枚举），进不了 static_assert。它们由主机侧契约用例按精确值钉住
// （micro_ros_bridge 的 Contract.ScalesAndSentinel / TopicNamesAreAbsoluteAndUnderMicroRos）。

// motor_status 不设尺寸门禁：它允许分片，而精确 CDR 长度要靠 40 多项字段算，
// 写成 static_assert 既冗长又容易与消息定义漂移。它的门禁是**运行期测量**
// （序列化一条 count=64 的 MotorStatus 看长度），等 status_bridge 存在之后。

}  // namespace
```

- [ ] **Step 3: 构建固件**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run 2>&1 | tail -25
```

预期：`SUCCESS`。本任务只加编译期断言、不加静态缓冲，所以构建汇总行里的 RAM 占用必须与
改动前**逐字节一致**（实测基线：RAM 315180 B / 577536 B = 54.6%，Flash 20.7%）。
`pio run` 的汇总行给的是**总体** RAM，不是 spec 里单列的 RAM_D1 分区数字——要分区数字得另测，
不属本任务。

若出现 `low_cmd 越出 best-effort 可用 MTU` 的断言失败，说明字段布局与 spec §4.2 不符——
先按 spec 核对，**不要**改大 `kBestEffortPayloadMax`。

**门禁不能是空门**：只含 static_assert 的 TU 若因构建过滤没被编进去，门禁形同不存在却看起来
存在。构建后确认 `.pio/build/*/src/wire_contract_check.cpp.o` 存在，把路径写进报告。

- [ ] **Step 4: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/src/wire_contract_check.cpp
git commit -m "test(firmware): 线上契约的编译期门禁

low_cmd / low_state 的 CDR 序列化尺寸（不是 sizeof）压在 490 B 以内，
哨兵与电机数上界一并钉死。越界在 pio run 就失败，而不是等台架上丢帧。
量程与话题名不在此断言：rosidl 的 C 侧常量不是 C++ 常量表达式，
它们由主机侧契约用例按精确值钉住。"
```

---

## Task 6: 固件侧定点编解码库 `lib/wire_codec`

**Files:**
- Create: `micro_ros_firmware/lib/wire_codec/include/wire_codec.h`
- Create: `micro_ros_firmware/lib/wire_codec/src/wire_codec.c`
- Create: `micro_ros_firmware/test/host/test_wire_codec.cpp`
- Modify: `micro_ros_firmware/test/host/CMakeLists.txt`
- Modify: `micro_ros_firmware/src/wire_contract_check.cpp`（加哨兵双份互钉，见 Step 6）

**Interfaces:**
- Consumes: 无（纯 C，无 ROS 依赖）
- Produces:
  - `typedef struct { int16_t q; bool saturated; } wire_quantize_result_t;`
  - `wire_quantize_result_t wire_quantize(double value, double scale);`
  - `double wire_dequantize(int16_t q, double scale);`（`q == WIRE_SENTINEL_MISSING` → NAN）
  - `#define WIRE_SENTINEL_MISSING ((int16_t)-32768)`

- [ ] **Step 1: 写失败测试**

```cpp
// 定点编解码的 host 测试。golden 向量与主机侧 C++ 用例取同一组值
// （ros2_ws/src/micro_ros_bridge/test/test_codec.cpp），两边注释互指；
// 量程常量本身仍是单一来源（WireContract.msg），这里只钉算术。
//
// 文件是 .cpp 而库是 .c：仓库里 22 个 host 测试全是 .cpp + <cassert>，
// wire_codec.h 有 extern "C" 保护，直编无碍。
#include <cassert>
#include <cmath>
#include <cstdio>

#include "wire_codec.h"

namespace {
constexpr double kScalePos = 12.5;
constexpr double kScaleVel = 64.0;

void test_golden_vectors()
{
  assert(wire_quantize(0.0, kScalePos).q == 0);
  assert(wire_quantize(kScalePos, kScalePos).q == 32767);
  assert(wire_quantize(-kScalePos, kScalePos).q == -32767);
  // 步长 = 12.5/32767 ≈ 0.000381469；0.00625 落在第 16 格
  assert(wire_quantize(0.00625, kScalePos).q == 16);
  assert(wire_quantize(64.0, kScaleVel).q == 32767);
}

void test_saturation()
{
  wire_quantize_result_t r = wire_quantize(1000.0, kScalePos);
  assert(r.q == 32767 && r.saturated);
  r = wire_quantize(-1000.0, kScalePos);
  assert(r.q == -32767 && r.saturated);
  // 边界内不算饱和
  r = wire_quantize(kScalePos, kScalePos);
  assert(!r.saturated);
}

void test_round_trip_within_half_step()
{
  const double step = kScalePos / 32767.0;
  for (int i = -1000; i <= 1000; ++i) {
    const double x = i * step * 3.7;
    const double back = wire_dequantize(wire_quantize(x, kScalePos).q, kScalePos);
    assert(std::fabs(back - x) <= step / 2.0 + 1e-12);
  }
}

void test_sentinel_is_not_a_value()
{
  assert(std::isnan(wire_dequantize(WIRE_SENTINEL_MISSING, kScalePos)));
  // 负满量程不会产生哨兵
  assert(wire_quantize(-kScalePos, kScalePos).q == -32767);
}

void test_non_finite_yields_sentinel()
{
  // 前置条件：调用方先校验有限性（命令路径 NaN = 整条拒收）。
  // 本函数对非有限输入给哨兵，而不是未定义行为。
  assert(wire_quantize(NAN, kScalePos).q == WIRE_SENTINEL_MISSING);
  assert(wire_quantize(INFINITY, kScalePos).q == WIRE_SENTINEL_MISSING);
}
}  // namespace

int main()
{
  test_golden_vectors();
  test_saturation();
  test_round_trip_within_half_step();
  test_sentinel_is_not_a_value();
  test_non_finite_yields_sentinel();
  std::printf("test_wire_codec: ok\n");
  return 0;
}
```

- [ ] **Step 2: 注册进 host 测试并确认失败**

在 `micro_ros_firmware/test/host/CMakeLists.txt` 里，**在 `fw_host_test()` 宏定义（约 113 行）
之前**、与其它 `add_library(fw_*)` 块并列处加：

```cmake
# 定点编解码纯逻辑（lib/wire_codec；无 ROS 依赖，host 可直编）
add_library(fw_wire_codec OBJECT
  ../../lib/wire_codec/src/wire_codec.c)
target_include_directories(fw_wire_codec PUBLIC
  ../../lib/wire_codec/include)
```

再在**其它 `fw_host_test()` 注册块之间**（即 `find_package(Python3 ...)` 之前，别落到文件末尾的
`fdcan_motor_sim` 之后）按宏的真实签名（`SOURCES` / `OBLIBS` / `INCLUDES` / `DEFINES`）注册用例。
`fw_wire_codec` 无间接依赖，只列它自己即可（宏的注释提醒：CMake 3.16 不传递 OBJECT 库的间接依赖）：

```cmake
fw_host_test(test_wire_codec
  SOURCES test_wire_codec.cpp
  OBLIBS fw_wire_codec)
```

宏自带 `target_link_libraries(${name} PRIVATE m)`，`lround` / `NAN` / `isfinite` 由此满足。

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2 2>&1 | tail -5
```

预期：**失败**——这一步就是确认测试真的被接进来了。此刻 `lib/wire_codec/src/wire_codec.c`
还不存在，所以失败点在 CMake 生成阶段：
`CMake Error: Cannot find source file: ../../lib/wire_codec/src/wire_codec.c`。
（若实现者先建了空的 `.c`，失败点才会前移到编译期的 `wire_codec.h: No such file or directory`。
两种都算 RED，照实记录实际看到的那条，不要照抄本条。）

- [ ] **Step 3: 写实现**

`lib/wire_codec/include/wire_codec.h`：

```c
// 定点编解码：物理量 ↔ 线协议 int16 字段。**固件侧唯一换算点**
// （主机侧对应 micro_ros_bridge 的 micro_ros_codec）。
//
// 单位边界：本库是协议单位与 SI 单位的边界，调用方传进来的必须是 rad / rad/s / N·m。
// 量程常量不在这里定义（唯一来源是 WireContract.msg 的 SCALE_* 常量）。
#ifndef WIRE_CODEC_H
#define WIRE_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 本拍无有效测量。**值必须与 WireContract.msg 的 SENTINEL_MISSING 一致**：
// 本库刻意不依赖 ROS（host 可直编），拿不到生成常量，所以这里是**受控的第二份**，
// 由 src/wire_contract_check.cpp 的 static_assert 把两份钉在一起。
#define WIRE_SENTINEL_MISSING ((int16_t)-32768)

typedef struct {
  int16_t q;
  bool saturated;  // 被量程钳位（上层据此判断量程常量是否选错）
} wire_quantize_result_t;

// 定点：q = clamp(round(value * 32767 / scale), -32767, 32767)。
// 前置条件：value 有限——非有限输入（NaN/Inf）返回哨兵值，不做未定义行为，
// 但**命令路径的 NaN 必须在调用前被拒**（整条拒收语义，不在这里兜）。
wire_quantize_result_t wire_quantize(double value, double scale);

// 反定点。q == WIRE_SENTINEL_MISSING → NAN。
double wire_dequantize(int16_t q, double scale);

#ifdef __cplusplus
}
#endif

#endif // WIRE_CODEC_H
```

`lib/wire_codec/src/wire_codec.c`：

```c
#include "wire_codec.h"

#include <math.h>

// 用 32767 而非 32768：满量程落在 int16 内，-32768 整个留给哨兵。
#define WIRE_Q_MAX 32767.0

wire_quantize_result_t wire_quantize(double value, double scale) {
  wire_quantize_result_t r = {0, false};
  if (!isfinite(value)) {
    r.q = WIRE_SENTINEL_MISSING;
    return r;
  }
  const double raw = value * WIRE_Q_MAX / scale;
  // 严格不等：**恰好**落在 ±full-scale 是可精确表示的，不算越界、不置饱和位。
  // 写成 >= 会让「满量程」这种正常命令在页面上显示成饱和（测试 test_saturation
  // 的 `!r.saturated` 正是钉这一点）。
  if (raw > WIRE_Q_MAX) {
    r.q = (int16_t)32767;
    r.saturated = true;
  } else if (raw < -WIRE_Q_MAX) {
    r.q = (int16_t)-32767;
    r.saturated = true;
  } else {
    r.q = (int16_t)lround(raw);
  }
  return r;
}

double wire_dequantize(int16_t q, double scale) {
  if (q == WIRE_SENTINEL_MISSING) {
    return NAN;
  }
  return (double)q * scale / WIRE_Q_MAX;
}
```

- [ ] **Step 4: 跑测试**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build --clean-first -j2 && ctest --test-dir .host-build --output-on-failure
```

预期：`test_wire_codec: ok`，且**原有全部 host 用例仍全绿**（本任务只加库，不动别的）。

- [ ] **Step 5: 把哨兵的两个副本钉在一起**

`wire_codec.h` 的注释承诺「本库是受控的第二份，由 `src/wire_contract_check.cpp` 的
`static_assert` 把两份钉在一起」——Task 5 的门禁当时没有这条断言，现在补上，否则那个承诺
是空的，`WIRE_SENTINEL_MISSING` 就只是一个重复的魔数（违反「一份事实只写一处」）。

**这条能成立**：`WireContract__SENTINEL_MISSING` 是枚举量，`WIRE_SENTINEL_MISSING` 是宏，
**两者都是整型常量表达式**，所以能进 `static_assert`（与 Task 5 删掉的那两条不同——
那两条是 `static const float` / `static const char * const`）。

在 `src/wire_contract_check.cpp` 顶部加 include，并在枚举断言旁加：

```cpp
#include "wire_codec.h"
```

```cpp
// wire_codec 的哨兵是**受控的第二份**（该库刻意不依赖 ROS，host 可直编，拿不到生成常量）。
// 宏与枚举都是整型常量表达式，所以这条能进 static_assert —— 两份漂移在这里就编不过。
static_assert(can_bridge_msgs__msg__WireContract__SENTINEL_MISSING == WIRE_SENTINEL_MISSING,
    "wire_codec 的哨兵与 WireContract 漂移了：两端定点解码会错位");
```

副作用（可接受）：`wire_contract_check.cpp` 从此依赖 `lib/wire_codec`，PlatformIO 的 LDF 会把
`wire_codec.c` 一并编进固件。Plan 1 里它还没有调用者，但 Plan 2 会用；提前编进来还能顺带
暴露只在固件工具链下才出现的编译错误。

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run 2>&1 | tail -25
```

预期：`SUCCESS`，RAM 仍与基线**逐字节一致**（本步只加编译期断言，不新增静态缓冲）。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/wire_codec micro_ros_firmware/test/host/CMakeLists.txt \
        micro_ros_firmware/test/host/test_wire_codec.cpp \
        micro_ros_firmware/src/wire_contract_check.cpp
git commit -m "feat(firmware): 新增 lib/wire_codec —— 定点编解码的固件侧唯一换算点

±full-scale 用 32767 而非 32768，把 -32768 整个留给哨兵；越界饱和并置位
（而不是拒收——量程选错应当看得见）；恰好落在满量程不算越界，不置位。
非有限输入给哨兵，命令路径的 NaN 拒收仍由调用方负责。
golden 向量与主机侧 C++ 用例同值互钉；哨兵的第二份副本由契约门禁
static_assert 钉住，两份漂移即编不过。"
```

---

## Task 7: `micro_ros_bridge` 包骨架与契约测试

**Files:**
- Create: `ros2_ws/src/micro_ros_bridge/package.xml`
- Create: `ros2_ws/src/micro_ros_bridge/CMakeLists.txt`
- Create: `ros2_ws/src/micro_ros_bridge/include/micro_ros_bridge/codec.hpp`（本任务只放契约常量别名）
- Create: `ros2_ws/src/micro_ros_bridge/test/test_contract.cpp`
- Modify: `ros2_ws/README.md`（包表加一行）

**Interfaces:**
- Consumes: Task 1–4 的消息包
- Produces: 包 `micro_ros_bridge`；`micro_ros_bridge::contract` 命名空间下的常量别名
  （`kSchemaVersion` / `kMotorCountMax` / `kSentinelMissing` / `kScalePosition` /
  `kScaleVelocity` / `kScaleEffort` / `topic::kLowCmd` …）

- [ ] **Step 1: 写失败测试**

```cpp
// 契约测试：常量值、枚举与旧 facade 消息的一致性。
// 话题名与量程的唯一来源是 WireContract.msg；这里只钉「取出来的值是什么」，
// 因为 bridge 与固件都靠这些值对齐。
#include <gtest/gtest.h>

#include <can_bridge_msgs/msg/motor_motion_state_target.hpp>
#include <can_bridge_msgs/msg/motor_status_target.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>

#include "micro_ros_bridge/codec.hpp"

TEST(Contract, TopicNamesAreAbsoluteAndUnderMicroRos) {
  using micro_ros_bridge::contract::topic::kLowCmd;
  EXPECT_EQ(kLowCmd, "/micro_ros/do/low_cmd");
  EXPECT_EQ(std::string(kLowCmd).rfind("/micro_ros/", 0), 0u);
}

TEST(Contract, ScalesAndSentinel) {
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScalePosition, 12.5);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleVelocity, 64.0);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleEffort, 64.0);
  EXPECT_EQ(micro_ros_bridge::contract::kSentinelMissing, -32768);
  EXPECT_EQ(micro_ros_bridge::contract::kMotorCountMax, 64);
}

// 旧 facade 消息保留着同一套枚举取值。两处若漂移，页面与固件对同一台电机的
// 解释就会不一致——钉死。
TEST(Contract, LegacyEnumsStillMatch) {
  EXPECT_EQ(can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY, 2);
  EXPECT_EQ(can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE, 4);
  EXPECT_EQ(can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE, 65535);
}
```

- [ ] **Step 2: 建包并确认失败**

`package.xml`（照 `robot_dynamics` 的格式）：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>micro_ros_bridge</name>
  <version>0.1.0</version>
  <description>micro-ROS wire contract codec and the single host-side boundary node.</description>
  <maintainer email="robot@example.com">Robot Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>can_bridge_msgs</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(micro_ros_bridge)

find_package(ament_cmake REQUIRED)
find_package(can_bridge_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# codec：header-only、纯函数、不依赖 rclcpp（只依赖消息类型）。
# 独立成 interface 目标，将来 ros2_control 的 SystemInterface 复用同一份换算。
add_library(${PROJECT_NAME}_codec INTERFACE)
target_compile_features(${PROJECT_NAME}_codec INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME}_codec INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_codec INTERFACE can_bridge_msgs)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(can_bridge_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_contract test/test_contract.cpp)
  target_compile_features(test_contract PRIVATE cxx_std_17)
  target_compile_options(test_contract PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_contract ${PROJECT_NAME}_codec)
endif()

ament_package()
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs micro_ros_bridge
```

预期：编译失败（`micro_ros_bridge/codec.hpp: No such file`）。

- [ ] **Step 3: 写 `codec.hpp` 的契约段**

```cpp
// 主机侧 codec：线上定点消息 ↔ 主机 facade 消息。
//
// 与固件 lib/wire_codec 是**同一条算术的两侧实现**：golden 向量在
// micro_ros_firmware/test/host/test_wire_codec.cpp 与 test/test_codec.cpp 里同值互钉。
// 本文件不含 rclcpp 依赖，将来 ros2_control 的 SystemInterface 直接复用。
#ifndef MICRO_ROS_BRIDGE__CODEC_HPP_
#define MICRO_ROS_BRIDGE__CODEC_HPP_

#include <cstdint>
#include <string>

#include <can_bridge_msgs/msg/wire_contract.hpp>

namespace micro_ros_bridge
{
namespace contract
{
// 常量别名：值全部来自 WireContract.msg，本文件不写第二份数值。
inline constexpr uint16_t kSchemaVersion =
  can_bridge_msgs::msg::WireContract::SCHEMA_VERSION;
inline constexpr uint8_t kMotorCountMax =
  can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX;
inline constexpr int16_t kSentinelMissing =
  can_bridge_msgs::msg::WireContract::SENTINEL_MISSING;
inline constexpr double kScalePosition =
  can_bridge_msgs::msg::WireContract::SCALE_POSITION_RAD;
inline constexpr double kScaleVelocity =
  can_bridge_msgs::msg::WireContract::SCALE_VELOCITY_RAD_S;
inline constexpr double kScaleEffort =
  can_bridge_msgs::msg::WireContract::SCALE_EFFORT_NM;

namespace topic
{
// 11 个线上话题名，值全部来自 WireContract.msg。缺一个就意味着某处得自己拼字符串。
inline const std::string & kLowCmd = can_bridge_msgs::msg::WireContract::TOPIC_LOW_CMD;
inline const std::string & kPolicy = can_bridge_msgs::msg::WireContract::TOPIC_POLICY;
inline const std::string & kTrajectory = can_bridge_msgs::msg::WireContract::TOPIC_TRAJECTORY;
inline const std::string & kConfig = can_bridge_msgs::msg::WireContract::TOPIC_CONFIG;
inline const std::string & kLowState = can_bridge_msgs::msg::WireContract::TOPIC_LOW_STATE;
inline const std::string & kMotorStatus = can_bridge_msgs::msg::WireContract::TOPIC_MOTOR_STATUS;
inline const std::string & kTopology = can_bridge_msgs::msg::WireContract::TOPIC_TOPOLOGY;
inline const std::string & kEvents = can_bridge_msgs::msg::WireContract::TOPIC_EVENTS;
inline const std::string & kDeviceParam = can_bridge_msgs::msg::WireContract::TOPIC_DEVICE_PARAM;
inline const std::string & kImuData = can_bridge_msgs::msg::WireContract::TOPIC_IMU_DATA;
inline const std::string & kImuTemp = can_bridge_msgs::msg::WireContract::TOPIC_IMU_TEMP;
}  // namespace topic
}  // namespace contract
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__CODEC_HPP_
```

- [ ] **Step 4: 跑测试**

```bash
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs micro_ros_bridge
colcon test --packages-select micro_ros_bridge && colcon test-result --verbose
```

预期：`test_contract` 三条用例全绿。

- [ ] **Step 5: 更新 `ros2_ws/README.md` 包表 —— **本计划内跳过，改到文档阶段做**

**不要在本任务里改 `ros2_ws/README.md`。** 该文件当前带着**未提交的既有改动**（包数
九包→十一包、`motor_command` 与 `robot_bringup` 两行），不是本计划的产物。`git add` 是按
文件粒度暂存的，会把那些改动一并卷进本任务的提交，把别人的在建工作混进本功能的分支历史。

该加的行（文档阶段补，届时按当时表格的实际列数对齐）：

```markdown
| `micro_ros_bridge` | 线上契约的编解码与**唯一边界节点**：`/micro_ros/*`（线上）与 `/motor/*`（主机侧）之间的翻译；codec 为 header-only 纯函数，供将来 ros2_control 复用 |
```

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/micro_ros_bridge
git commit -m "feat(micro_ros_bridge): 新包骨架与契约测试

常量别名全部取自 WireContract.msg，不写第二份数值；同时钉住旧 facade 消息
里那套枚举的取值，避免页面与固件对同一台电机的解释漂移。
本包尚未接线任何 launch。"
```

---

## Task 8: 主机侧标量编解码与 golden 向量

**Files:**
- Modify: `ros2_ws/src/micro_ros_bridge/include/micro_ros_bridge/codec.hpp`
- Create: `ros2_ws/src/micro_ros_bridge/test/test_codec.cpp`
- Modify: `ros2_ws/src/micro_ros_bridge/CMakeLists.txt`（注册 `test_codec`）

**Interfaces:**
- Consumes: Task 7 的 `contract::*`
- Produces:
  - `struct Quantized { int16_t q; bool saturated; };`
  - `Quantized quantize(double value, double scale);`
  - `double dequantize(int16_t q, double scale);`

- [ ] **Step 1: 写失败测试**

```cpp
// 与固件 lib/wire_codec 同一组 golden 向量（micro_ros_firmware/test/host/test_wire_codec.cpp）。
// 两边取值必须一致：不一致意味着固件与主机对同一个物理量的解释不同。
#include <gtest/gtest.h>

#include <cmath>

#include "micro_ros_bridge/codec.hpp"

using micro_ros_bridge::contract::kScalePosition;
using micro_ros_bridge::contract::kScaleVelocity;
using micro_ros_bridge::contract::kSentinelMissing;

TEST(Codec, GoldenVectors) {
  EXPECT_EQ(micro_ros_bridge::quantize(0.0, kScalePosition).q, 0);
  EXPECT_EQ(micro_ros_bridge::quantize(kScalePosition, kScalePosition).q, 32767);
  EXPECT_EQ(micro_ros_bridge::quantize(-kScalePosition, kScalePosition).q, -32767);
  EXPECT_EQ(micro_ros_bridge::quantize(0.00625, kScalePosition).q, 16);
  EXPECT_EQ(micro_ros_bridge::quantize(64.0, kScaleVelocity).q, 32767);
}

TEST(Codec, SaturatesInsteadOfRejecting) {
  const auto hi = micro_ros_bridge::quantize(1000.0, kScalePosition);
  EXPECT_EQ(hi.q, 32767);
  EXPECT_TRUE(hi.saturated);
  const auto lo = micro_ros_bridge::quantize(-1000.0, kScalePosition);
  EXPECT_EQ(lo.q, -32767);
  EXPECT_TRUE(lo.saturated);
  EXPECT_FALSE(micro_ros_bridge::quantize(kScalePosition, kScalePosition).saturated);
}

TEST(Codec, RoundTripWithinHalfStep) {
  const double step = kScalePosition / 32767.0;
  for (int i = -1000; i <= 1000; ++i) {
    const double x = i * step * 3.7;
    const double back = micro_ros_bridge::dequantize(
      micro_ros_bridge::quantize(x, kScalePosition).q, kScalePosition);
    EXPECT_LE(std::fabs(back - x), step / 2.0 + 1e-12);
  }
}

TEST(Codec, SentinelIsNotAValue) {
  EXPECT_TRUE(std::isnan(micro_ros_bridge::dequantize(kSentinelMissing, kScalePosition)));
  EXPECT_EQ(micro_ros_bridge::quantize(-kScalePosition, kScalePosition).q, -32767);
}

TEST(Codec, NonFiniteYieldsSentinel) {
  EXPECT_EQ(micro_ros_bridge::quantize(NAN, kScalePosition).q, kSentinelMissing);
  EXPECT_EQ(micro_ros_bridge::quantize(INFINITY, kScalePosition).q, kSentinelMissing);
}
```

在 `CMakeLists.txt` 的 `if(BUILD_TESTING)` 块里加：

```cmake
  ament_add_gtest(test_codec test/test_codec.cpp)
  target_compile_features(test_codec PRIVATE cxx_std_17)
  target_compile_options(test_codec PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_codec ${PROJECT_NAME}_codec)
```

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select micro_ros_bridge
```

预期：编译失败（`quantize` 未声明）。

- [ ] **Step 2: 写实现**

追加到 `codec.hpp`（放在 `contract` 命名空间之后）：

```cpp
namespace micro_ros_bridge
{
struct Quantized
{
  int16_t q;
  bool saturated;
};

// 定点：q = clamp(round(value * 32767 / scale), -32767, 32767)。
// 与固件 wire_quantize 同式；非有限输入返回哨兵（命令路径的 NaN 由消息级
// 编码函数拒收，不在这里兜）。
inline Quantized quantize(double value, double scale)
{
  constexpr double kQMax = 32767.0;
  if (!std::isfinite(value)) {
    return {contract::kSentinelMissing, false};
  }
  const double raw = value * kQMax / scale;
  // 严格不等，与固件 wire_quantize 逐字同式：**恰好**±full-scale 可精确表示，不算越界、
  // 不置饱和位（Codec.SaturatesInsteadOfRejecting 的 EXPECT_FALSE 钉这一点）。
  if (raw > kQMax) {
    return {static_cast<int16_t>(32767), true};
  }
  if (raw < -kQMax) {
    return {static_cast<int16_t>(-32767), true};
  }
  return {static_cast<int16_t>(std::lround(raw)), false};
}

// 反定点。q == SENTINEL_MISSING → NaN。
inline double dequantize(int16_t q, double scale)
{
  if (q == contract::kSentinelMissing) {
    return std::nan("");
  }
  return static_cast<double>(q) * scale / 32767.0;
}
}  // namespace micro_ros_bridge
```

同时在文件头补 `#include <cmath>`。

- [ ] **Step 3: 跑测试**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select micro_ros_bridge
colcon test --packages-select micro_ros_bridge && colcon test-result --verbose
```

预期：`test_codec` 五条与 `test_contract` 三条全绿。

- [ ] **Step 4: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/micro_ros_bridge
git commit -m "feat(micro_ros_bridge): 主机侧定点编解码与 golden 向量

与固件 lib/wire_codec 同式同向量（两边注释互指），越界饱和、非有限给哨兵。
换算集中在 codec 里，控制逻辑仍用 SI 单位。"
```

---

## Task 9: 消息级 encode / decode

**Files:**
- Create: `ros2_ws/src/micro_ros_bridge/include/micro_ros_bridge/messages.hpp`
- Create: `ros2_ws/src/micro_ros_bridge/src/messages.cpp`
- Create: `ros2_ws/src/micro_ros_bridge/test/test_messages.cpp`
- Modify: `ros2_ws/src/micro_ros_bridge/CMakeLists.txt`（加 `_messages` 库并注册 `test_messages`）

**Interfaces:**
- Consumes: Task 8 的 `quantize` / `dequantize`
- Produces:
  - `struct Result { bool ok = true; const char * error = ""; uint32_t saturated = 0; };`
  - `Result encode_low_command(const MotorCommandArray &, LowCommand &);`
  - `Result decode_low_state(const LowState &, MotorMotionStateArray &);`
  - `Result encode_config(const ParamSetArray &, ConfigRequest &);`（kind = PARAM_SET）
  - `Result encode_config(const SystemCommand &, ConfigRequest &);`（kind = SYSTEM_COMMAND）
  - `Result decode_motor_status(const MotorStatus &, MotorStatusArray &,
      MotorTrajectoryState &, SystemStatus &, MotorDiagnostics &,
      std::vector<MotorFreshness> &);`
  - `struct MotorFreshness { uint32_t state_age_ms = 0xFFFFFFFFu; uint8_t target_source = 0; };`

  **为什么 `decode_motor_status` 多一个出参**：`state_age_ms` / `target_source` 是循环面
  装不下的两个遥测量（§4.3），只在 10 Hz 的 `motor_status` 里。但页面读的是
  **motion_states 元素**上的这两个字段（`web_host/web/js/panels/readout.js:34`、
  `arm.js:270`），而 spec §4.7 承诺「JS 一个字段都不用改，只改话题名前缀」。所以它们
  不能落进 `MotorStatusTarget`——spec §4.7 同时要求旧消息「字段级沿用现定义」。两条约束
  一起只能得出：**由 bridge 缓存、在填充 `motion_states` 时盖章**（Task 10）。这个出参
  就是那块缓存，按 `motor_id` 索引、长度 `MOTOR_COUNT_MAX`。

  **为什么是两个重载而不是一个带"互斥"判定的函数**：主机侧有两条独立的订阅
  （`/motor/param_set`、`/motor/system_command`），要区分"这条是参数写"和"这条是系统
  命令"只能靠类型。合成一个函数就得用值来判，而默认构造的 `SystemCommand.command == 0`
  恰好等于 `COMMAND_SAVE`——"缺席"与"SAVE"不可分辨，必然写出一个靠不住的判定。两个
  重载让"两组字段互斥"由构造保证；线上消息里那两组字段的互斥校验归固件解码端。

- [ ] **Step 1: 写失败测试**

```cpp
// 消息级编解码：整条受理或整条拒绝的口径必须与固件旧契约逐字一致。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "micro_ros_bridge/messages.hpp"

using can_bridge_msgs::msg::LowCommand;
using can_bridge_msgs::msg::LowState;
using can_bridge_msgs::msg::MotorCommandArray;
using can_bridge_msgs::msg::MotorCommandTarget;
using can_bridge_msgs::msg::MotorMotionStateArray;

namespace {

MotorCommandTarget target(uint16_t id, double pos, double vel, double eff)
{
  MotorCommandTarget t;
  t.motor_id = id;
  t.position = pos;
  t.velocity = vel;
  t.effort = eff;
  return t;
}

}  // namespace

TEST(EncodeLowCommand, RoundTripThroughQuantization) {
  MotorCommandArray in;
  in.count = 2;
  in.targets.push_back(target(0, 0.5, -1.0, 2.0));
  in.targets.push_back(target(3, -0.25, 0.0, 0.0));

  LowCommand out;
  const auto r = micro_ros_bridge::encode_low_command(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(out.motor_ids.size(), 2u);
  EXPECT_EQ(out.motor_ids[0], 0);
  EXPECT_EQ(out.motor_ids[1], 3);
  EXPECT_NEAR(micro_ros_bridge::dequantize(out.position_q[0],
    micro_ros_bridge::contract::kScalePosition), 0.5, 1e-3);
  EXPECT_EQ(out.count, 2);
}

// 整条拒绝：任何一条不合法，一个字节都不出去（沿用旧口径）。
TEST(EncodeLowCommand, RejectsWholeMessage) {
  LowCommand out;

  MotorCommandArray bad_id;
  bad_id.count = 1;
  bad_id.targets.push_back(target(64, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(bad_id, out).ok);

  MotorCommandArray dup;
  dup.count = 2;
  dup.targets.push_back(target(1, 0.0, 0.0, 0.0));
  dup.targets.push_back(target(1, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(dup, out).ok);

  MotorCommandArray nan;
  nan.count = 1;
  nan.targets.push_back(target(0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(nan, out).ok);

  MotorCommandArray mismatch;
  mismatch.count = 2;
  mismatch.targets.push_back(target(0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(mismatch, out).ok);
}

TEST(EncodeLowCommand, CountsSaturatedChannels) {
  MotorCommandArray in;
  in.count = 1;
  in.targets.push_back(target(0, 1000.0, 0.0, 0.0));
  LowCommand out;
  const auto r = micro_ros_bridge::encode_low_command(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.saturated, 1u);
  EXPECT_EQ(out.position_q[0], 32767);
}

TEST(DecodeLowState, SentinelBecomesNaN) {
  LowState in;
  in.seq = 7;
  in.count = 1;
  in.motor_ids = {5};
  in.position_q = {100};
  in.velocity_q = {micro_ros_bridge::contract::kSentinelMissing};
  in.effort_q = {0};

  MotorMotionStateArray out;
  const auto r = micro_ros_bridge::decode_low_state(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(out.seq, 7u);  // 丢帧判据必须透到 facade，不能丢
  ASSERT_EQ(out.states.size(), 1u);
  EXPECT_EQ(out.states[0].motor_id, 5);
  EXPECT_TRUE(std::isnan(out.states[0].velocity));
  EXPECT_FALSE(std::isnan(out.states[0].position));
}

TEST(EncodeConfig, ParamSetOverloadFillsParamGroup) {
  can_bridge_msgs::msg::ParamSetArray in;
  in.request_id = 7;
  in.count = 2;
  can_bridge_msgs::msg::ParamSetTarget lo;
  lo.motor_id = 1;
  lo.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MIN;
  lo.value = -0.5;
  can_bridge_msgs::msg::ParamSetTarget hi;
  hi.motor_id = 1;
  hi.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MAX;
  hi.value = 0.5;
  in.targets = {lo, hi};

  can_bridge_msgs::msg::ConfigRequest out;
  const auto r = micro_ros_bridge::encode_config(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(out.kind, can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_PARAM_SET);
  EXPECT_EQ(out.count, 2);
  EXPECT_EQ(out.targets.size(), 2u);
  // 系统命令那组必须留成"不适用"，不是默认 0。
  EXPECT_EQ(out.motor_id, can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE);
}

TEST(EncodeConfig, ParamSetRejectsUnpairedLimit) {
  can_bridge_msgs::msg::ParamSetArray in;
  in.count = 1;
  can_bridge_msgs::msg::ParamSetTarget lo;
  lo.motor_id = 1;
  lo.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MIN;
  in.targets = {lo};

  can_bridge_msgs::msg::ConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_config(in, out).ok);
}

TEST(EncodeConfig, ParamSetRejectsDuplicateAndLengthMismatch) {
  can_bridge_msgs::msg::ConfigRequest out;

  can_bridge_msgs::msg::ParamSetArray dup;
  dup.count = 2;
  can_bridge_msgs::msg::ParamSetTarget a;
  a.motor_id = 2;
  a.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_VEL_LIMIT;
  dup.targets = {a, a};
  EXPECT_FALSE(micro_ros_bridge::encode_config(dup, out).ok);

  // count 与 targets 长度不一致 → 拒收。**这里测的就是这条守卫**：`count > 16` 的容量守卫
  // 经 ROS C++ API 不可达（targets 是 BoundedVector<...,16>，size 恒 ≤ 16，而本守卫先短路），
  // 所以用例名不能声称覆盖容量——那会看起来有覆盖、实际没有。
  can_bridge_msgs::msg::ParamSetArray many;
  many.count = 17;
  for (int i = 0; i < 16; ++i) {
    can_bridge_msgs::msg::ParamSetTarget t;
    t.motor_id = static_cast<uint16_t>(i);
    t.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_VEL_LIMIT;
    many.targets.push_back(t);
  }
  EXPECT_FALSE(micro_ros_bridge::encode_config(many, out).ok);
}

TEST(EncodeConfig, SystemCommandOverloadFillsCommandGroup) {
  can_bridge_msgs::msg::SystemCommand in;
  in.command = can_bridge_msgs::msg::SystemCommand::COMMAND_REFRESH;
  in.motor_id = 3;

  can_bridge_msgs::msg::ConfigRequest out;
  const auto r = micro_ros_bridge::encode_config(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(out.kind, can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND);
  EXPECT_EQ(out.command, can_bridge_msgs::msg::SystemCommand::COMMAND_REFRESH);
  EXPECT_EQ(out.motor_id, 3);
  EXPECT_TRUE(out.targets.empty());
  EXPECT_EQ(out.count, 0);
}

// SAVE 只接受整机（W25 那半必须整体落盘）。
TEST(EncodeConfig, SaveOnlyAcceptsWholeMachine) {
  can_bridge_msgs::msg::SystemCommand in;
  in.command = can_bridge_msgs::msg::SystemCommand::COMMAND_SAVE;
  in.motor_id = 3;
  can_bridge_msgs::msg::ConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_config(in, out).ok);
}

TEST(DecodeMotorStatus, RestoresFacadeFields) {
  can_bridge_msgs::msg::MotorStatus in;
  in.sync_time_ns = 42;
  in.count = 1;
  in.motor_ids = {9};
  in.state_age_ms = {0xFFFF};  // 从未收到
  in.target_source = {can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY};
  in.lifecycle = {can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE};
  in.unified_fault_code = {0};
  in.vendor_fault_code = {0};
  in.protection_flags = {0};
  in.limit_flags = {0};
  in.temperature_dc = {253};   // 25.3 °C
  in.current_ma = {1500};      // 1.5 A
  in.voltage_mv = {24000};     // 24 V
  in.drive_mode = {1};
  in.status_flags = {0x1F};    // 五个位全置
  in.bus_index = {0};

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  const auto r = micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(statuses.statuses.size(), 1u);
  const auto & s = statuses.statuses[0];
  EXPECT_EQ(s.motor_id, 9);
  EXPECT_EQ(s.lifecycle, can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE);
  // 新鲜度走缓存，不进 MotorStatusTarget（spec §4.7：旧消息字段级沿用现定义）。
  ASSERT_EQ(fresh.size(), can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX);
  EXPECT_EQ(fresh[9].state_age_ms, 0xFFFFFFFFu);  // 哨兵还原成"从未收到"
  EXPECT_EQ(fresh[9].target_source,
    can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY);
  EXPECT_EQ(fresh[0].state_age_ms, 0xFFFFFFFFu);  // 本帧未列出的电机读作从未收到
  EXPECT_NEAR(s.temperature, 25.3, 1e-6);
  EXPECT_NEAR(s.current, 1.5, 1e-9);
  EXPECT_NEAR(s.voltage, 24.0, 1e-9);
  EXPECT_TRUE(s.online);
  EXPECT_TRUE(s.device_protect_enabled);
  EXPECT_EQ(s.device_direction, 1u);
  EXPECT_EQ(statuses.sync_time_ns, 42u);
}

TEST(DecodeMotorStatus, MissingElectricalReadsAsNaN) {
  can_bridge_msgs::msg::MotorStatus in;
  in.count = 1;
  in.motor_ids = {0};
  in.state_age_ms = {100};  // 真实年龄，不是哨兵
  in.target_source = {0};
  in.lifecycle = {0};
  in.unified_fault_code = {0};
  in.vendor_fault_code = {0};
  in.protection_flags = {0};
  in.limit_flags = {0};
  in.temperature_dc = {micro_ros_bridge::contract::kSentinelMissing};
  in.current_ma = {micro_ros_bridge::contract::kSentinelMissing};
  in.voltage_mv = {0xFFFF};
  in.drive_mode = {0};
  in.status_flags = {0};
  in.bus_index = {0};

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  const auto r = micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(std::isnan(statuses.statuses[0].temperature));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].current));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].voltage));
  EXPECT_EQ(fresh[0].state_age_ms, 100u);  // 真实年龄，不是哨兵
}

TEST(DecodeMotorStatus, RejectsRaggedArrays) {
  can_bridge_msgs::msg::MotorStatus in;
  in.count = 2;
  in.motor_ids = {0, 1};
  in.state_age_ms = {0};  // 长度与 count 不符

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  EXPECT_FALSE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
}
```

- [ ] **Step 2: 注册测试并确认失败**

在 `CMakeLists.txt` 的 `if(BUILD_TESTING)` 块里加：

```cmake
  ament_add_gtest(test_messages test/test_messages.cpp)
  target_compile_features(test_messages PRIVATE cxx_std_17)
  target_compile_options(test_messages PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_messages ${PROJECT_NAME}_messages)
```

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs micro_ros_bridge
```

预期：编译失败（`messages.hpp: No such file`）。

- [ ] **Step 3: 写实现**

```cpp
// 消息级编解码：主机 facade 消息 ↔ 线上消息。
//
// 校验口径与固件旧契约逐字一致：**整条受理或整条拒绝**（任一目标不合法则一个
// 字节都不下发），命令路径的 NaN/Inf 同样整条拒收。
//
// 枚举取值沿用保留消息的常量（target_source / lifecycle / command），不在此重定义。
#ifndef MICRO_ROS_BRIDGE__MESSAGES_HPP_
#define MICRO_ROS_BRIDGE__MESSAGES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/low_command.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_array.hpp>
#include <can_bridge_msgs/msg/motor_status.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/param_set_array.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>

#include "micro_ros_bridge/codec.hpp"

namespace micro_ros_bridge
{
struct Result
{
  bool ok = true;
  const char * error = "";
  uint32_t saturated = 0;
};

// 每台最近一次低频面给出的新鲜度，按 motor_id 索引。循环面（low_state）不带这两个量
// ——带上会越过 MTU 而被迫分片——但页面读的是 motion_states 元素上的它们，所以由
// BridgeNode 缓存、在 50 Hz 填充时盖章。缺省值 = "从未收到"。
struct MotorFreshness
{
  uint32_t state_age_ms = 0xFFFFFFFFu;
  uint8_t target_source = 0;
};

Result encode_low_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::LowCommand & out);

Result decode_low_state(
  const can_bridge_msgs::msg::LowState & in,
  can_bridge_msgs::msg::MotorMotionStateArray & out);

Result encode_config(
  const can_bridge_msgs::msg::ParamSetArray & in,
  can_bridge_msgs::msg::ConfigRequest & out);

Result encode_config(
  const can_bridge_msgs::msg::SystemCommand & in,
  can_bridge_msgs::msg::ConfigRequest & out);

Result decode_motor_status(
  const can_bridge_msgs::msg::MotorStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorTrajectoryState & traj,
  can_bridge_msgs::msg::SystemStatus & sys,
  can_bridge_msgs::msg::MotorDiagnostics & diag,
  std::vector<MotorFreshness> & freshness);
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__MESSAGES_HPP_
```

实现放在新增的 `src/messages.cpp`。`codec.hpp` 是 header-only（纯函数、无 rclcpp），
消息级函数体量较大且有 `bool seen[64][6]`（第二维写的是 `Target::FIELD_DIRECTION + 1`）
这类栈上结构，独立成编译单元：

```cmake
# 消息级编解码：依赖 codec 的标量换算，不依赖 rclcpp。
add_library(${PROJECT_NAME}_messages src/messages.cpp)
target_compile_features(${PROJECT_NAME}_messages PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_messages PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_messages PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_messages PUBLIC ${PROJECT_NAME}_codec)
# 必须用 keyword 形式：同一 target 上 keyword 与 plain 两种签名不能混用，混了在配置期就报错。
# Humble 的 ament_target_dependencies 支持 PUBLIC（宏内 cmake_parse_arguments 收了它）。
ament_target_dependencies(${PROJECT_NAME}_messages PUBLIC can_bridge_msgs)
```

`src/messages.cpp` 内容：

```cpp
#include "micro_ros_bridge/messages.hpp"

#include <cmath>
#include <limits>

namespace micro_ros_bridge
{
namespace
{
constexpr uint16_t kIdLimit = contract::kMotorCountMax;  // motor_id ∈ [0,63]
}

Result encode_low_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::LowCommand & out)
{
  Result r;
  if (in.targets.size() != in.count) {
    r.ok = false;
    r.error = "count 与 targets 长度不一致";
    return r;
  }
  if (in.count > kIdLimit) {
    // 经 ROS C++ API 不可达：targets 是 BoundedVector<...,64>，size 恒 ≤ 64，而上面那条
    // 长度守卫先短路。保留作防御——将来若放宽 .msg 上界，这里是唯一的拦截点。
    r.ok = false;
    r.error = "目标数超过 MOTOR_COUNT_MAX";
    return r;
  }
  // 先全量校验再写输出：整条受理或整条拒绝，不留半条。
  bool seen[kIdLimit] = {false};
  for (const auto & t : in.targets) {
    if (t.motor_id >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（注册表下标 0..63）";
      return r;
    }
    if (seen[t.motor_id]) {
      r.ok = false;
      r.error = "motor_id 重复";
      return r;
    }
    seen[t.motor_id] = true;
    if (!std::isfinite(t.position) || !std::isfinite(t.velocity) || !std::isfinite(t.effort)) {
      r.ok = false;
      r.error = "命令含 NaN/Inf：整条拒收";
      return r;
    }
  }

  out.sync_time_ns = in.sync_time_ns;
  out.sequence = in.sequence;
  out.count = in.count;
  out.motor_ids.clear();
  out.position_q.clear();
  out.velocity_q.clear();
  out.effort_q.clear();
  for (const auto & t : in.targets) {
    const auto p = quantize(t.position, contract::kScalePosition);
    const auto v = quantize(t.velocity, contract::kScaleVelocity);
    const auto e = quantize(t.effort, contract::kScaleEffort);
    r.saturated += (p.saturated ? 1u : 0u) + (v.saturated ? 1u : 0u) + (e.saturated ? 1u : 0u);
    out.motor_ids.push_back(static_cast<uint8_t>(t.motor_id));
    out.position_q.push_back(p.q);
    out.velocity_q.push_back(v.q);
    out.effort_q.push_back(e.q);
  }
  return r;
}

Result decode_low_state(
  const can_bridge_msgs::msg::LowState & in,
  can_bridge_msgs::msg::MotorMotionStateArray & out)
{
  Result r;
  const size_t n = in.motor_ids.size();
  if (in.position_q.size() != n || in.velocity_q.size() != n || in.effort_q.size() != n ||
    in.count != n)
  {
    r.ok = false;
    r.error = "count 与各数组长度不一致";
    return r;
  }
  out.sync_time_ns = in.sync_time_ns;
  // seq 直通：主机侧唯一的丢帧判据（best-effort 无重传）。facade 的
  // MotorMotionStateArray 末尾为此追加了 seq 字段。
  out.seq = in.seq;
  out.states.clear();
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorMotionStateTarget s;
    s.motor_id = in.motor_ids[i];
    s.position = dequantize(in.position_q[i], contract::kScalePosition);
    s.velocity = dequantize(in.velocity_q[i], contract::kScaleVelocity);
    s.effort = dequantize(in.effort_q[i], contract::kScaleEffort);
    // state_age_ms 与 target_source 不在这条消息里（循环面不分片）。本函数保持
    // 纯函数：留成"未知"，由 BridgeNode::on_low_state 用低频面缓存盖章。
    s.state_age_ms = std::numeric_limits<uint32_t>::max();
    s.target_source = can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_NONE;
    out.states.push_back(s);
  }
  return r;
}
}  // namespace micro_ros_bridge
```

接着写余下两个编码函数与解码函数，**插在同一文件的 `namespace micro_ros_bridge` 内**
（即上面那段闭合大括号之前）。先把上面匿名命名空间里的 `kIdLimit` 补一条同级常量：

```cpp
// 配置面目标数上界 = 内核 CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY（不是带宽约束）。
constexpr size_t kConfigTargetsMax = 16;
// 缺省哨兵：与 WireContract.SENTINEL_MISSING 同值，这里给个语义化别名。
constexpr int16_t kMissingI16 = contract::kSentinelMissing;   // -32768
constexpr uint16_t kMissingU16 = 0xFFFFu;
```

```cpp
Result encode_config(
  const can_bridge_msgs::msg::ParamSetArray & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  using Target = can_bridge_msgs::msg::ParamSetTarget;
  Result r;
  if (in.targets.size() != in.count) {
    r.ok = false;
    r.error = "count 与 targets 长度不一致";
    return r;
  }
  if (in.count > kConfigTargetsMax) {
    // 经 ROS C++ API 不可达：targets 是 BoundedVector<...,16>，size 恒 ≤ 16，而上面那条
    // 长度守卫先短路。保留作防御——将来若放宽 .msg 上界，这里是唯一的拦截点。
    r.ok = false;
    r.error = "目标数超过配置 FIFO 容量 16";
    return r;
  }
  // bit30 是固件自发脉冲的命名空间，主机置位会被本地拒绝；bit31 已被内核身份读取占用。
  if ((in.request_id & 0x40000000u) != 0u) {
    r.ok = false;
    r.error = "request_id bit30 保留给固件自发脉冲，主机不得置位";
    return r;
  }
  // 第二维用 FIELD_DIRECTION + 1 而不是字面量 6：将来加槽位时，上面那条
  // `t.field > Target::FIELD_DIRECTION` 会跟着放宽，而写死的 6 不会——于是
  // `seen[motor_id][field]` 静默越界写。
  bool seen[kIdLimit][Target::FIELD_DIRECTION + 1] = {};
  bool has_min[kIdLimit] = {};
  bool has_max[kIdLimit] = {};
  for (const auto & t : in.targets) {
    if (t.motor_id >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（注册表下标 0..63）";
      return r;
    }
    if (t.field > Target::FIELD_DIRECTION) {
      r.ok = false;
      r.error = "未知参数槽位";
      return r;
    }
    if (seen[t.motor_id][t.field]) {
      r.ok = false;
      r.error = "同一 (motor_id, field) 重复";
      return r;
    }
    seen[t.motor_id][t.field] = true;
    if (t.field == Target::FIELD_POS_MIN) {
      has_min[t.motor_id] = true;
    }
    if (t.field == Target::FIELD_POS_MAX) {
      has_max[t.motor_id] = true;
    }
  }
  // 位置限位必须成对出现在同一条消息里（半边状态正是本设计要消掉的东西）。
  for (size_t id = 0; id < kIdLimit; ++id) {
    if (has_min[id] != has_max[id]) {
      r.ok = false;
      r.error = "位置限位必须成对：只给了一边";
      return r;
    }
  }

  out.sync_time_ns = in.sync_time_ns;
  out.request_id = in.request_id;
  out.kind = can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_PARAM_SET;
  out.count = in.count;
  out.targets = in.targets;  // 同类型序列，直接搬
  out.command = can_bridge_msgs::msg::SystemCommand::COMMAND_SAVE;  // 该组不适用
  out.motor_id = can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE;
  return r;
}

Result encode_config(
  const can_bridge_msgs::msg::SystemCommand & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  using Cmd = can_bridge_msgs::msg::SystemCommand;
  Result r;
  if (in.command > Cmd::COMMAND_REPORT) {
    r.ok = false;
    r.error = "未知系统命令码";
    return r;
  }
  if ((in.request_id & 0x40000000u) != 0u) {
    r.ok = false;
    r.error = "request_id bit30 保留给固件自发脉冲，主机不得置位";
    return r;
  }
  // 规则逐字沿用 SystemCommand.msg 的头注释：SAVE 只接受整机；REPORT 忽略 motor_id；
  // 其余三条必须给有效 id。
  if (in.command == Cmd::COMMAND_SAVE && in.motor_id != Cmd::MOTOR_ID_NONE) {
    r.ok = false;
    r.error = "SAVE 只接受整机保存（MOTOR_ID_NONE）";
    return r;
  }
  if (in.command != Cmd::COMMAND_SAVE && in.command != Cmd::COMMAND_REPORT &&
    in.motor_id >= kIdLimit)
  {
    r.ok = false;
    r.error = "该命令必须给有效 motor_id（0..63）";
    return r;
  }

  out.sync_time_ns = in.sync_time_ns;
  out.request_id = in.request_id;
  out.kind = can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  out.count = 0;
  out.targets.clear();  // 该组不适用：互斥由构造保证
  out.command = in.command;
  out.motor_id = in.motor_id;
  return r;
}

Result decode_motor_status(
  const can_bridge_msgs::msg::MotorStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorTrajectoryState & traj,
  can_bridge_msgs::msg::SystemStatus & sys,
  can_bridge_msgs::msg::MotorDiagnostics & diag,
  std::vector<MotorFreshness> & freshness)
{
  Result r;
  // 整帧重来：本帧未列出的电机读作"从未收到"，不保留上一帧的陈旧值。
  freshness.assign(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX, MotorFreshness{});
  const size_t n = in.motor_ids.size();
  const bool lengths_ok =
    in.count == n &&
    in.state_age_ms.size() == n && in.target_source.size() == n &&
    in.lifecycle.size() == n && in.unified_fault_code.size() == n &&
    in.vendor_fault_code.size() == n && in.protection_flags.size() == n &&
    in.limit_flags.size() == n && in.temperature_dc.size() == n &&
    in.current_ma.size() == n && in.voltage_mv.size() == n &&
    in.drive_mode.size() == n && in.status_flags.size() == n &&
    in.bus_index.size() == n;
  if (!lengths_ok) {
    r.ok = false;
    r.error = "逐台数组长度与 count 不一致";
    return r;
  }

  // ---- MotorStatusArray ----
  statuses.sync_time_ns = in.sync_time_ns;
  statuses.statuses.clear();
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorStatusTarget s;
    s.motor_id = in.motor_ids[i];
    // 这两个量不进 MotorStatusTarget（spec §4.7：旧消息字段级沿用现定义）；
    // 它们进 freshness 缓存，由 Task 10 在填充 motion_states 时盖章。
    // 0xFFFF = 从未收到 → 还原成 0xFFFFFFFF；其余毫秒直通（固件侧真实年龄
    // 饱和在 0xFFFE，不会与哨兵撞车）。
    freshness[in.motor_ids[i]].state_age_ms = (in.state_age_ms[i] == kMissingU16)
      ? 0xFFFFFFFFu : static_cast<uint32_t>(in.state_age_ms[i]);
    freshness[in.motor_ids[i]].target_source = in.target_source[i];
    s.lifecycle = in.lifecycle[i];
    s.unified_fault_code = in.unified_fault_code[i];
    s.vendor_fault_code = in.vendor_fault_code[i];
    s.protection_flags = in.protection_flags[i];
    s.limit_flags = in.limit_flags[i];
    // 电气量回 SI：facade 用 NaN 表示缺失（不是哨兵）。
    s.temperature = (in.temperature_dc[i] == kMissingI16)
      ? std::nan("") : static_cast<double>(in.temperature_dc[i]) * 0.1;
    s.current = (in.current_ma[i] == kMissingI16)
      ? std::nan("") : static_cast<double>(in.current_ma[i]) * 0.001;
    s.voltage = (in.voltage_mv[i] == kMissingU16)
      ? std::nan("") : static_cast<double>(in.voltage_mv[i]) * 0.001;
    s.drive_mode = in.drive_mode[i];
    // status_flags 位序见 MotorStatus.msg 头注释；还原成**正向语义**布尔，
    // 0x2650 的反转不在这里做（它只发生在固件 param_bridge 一处）。
    const uint8_t f = in.status_flags[i];
    s.online = (f & 0x01u) != 0u;
    s.device_protect_known = (f & 0x02u) != 0u;
    s.device_protect_enabled = (f & 0x04u) != 0u;
    s.device_direction_known = (f & 0x08u) != 0u;
    s.device_direction = (f & 0x10u) != 0u ? 1u : 0u;
    s.bus_index = in.bus_index[i];
    statuses.statuses.push_back(s);
  }

  // ---- MotorTrajectoryState ----
  traj.sync_time_ns = in.sync_time_ns;
  traj.trajectory_id = in.trajectory_id;
  traj.active = in.traj_active;
  traj.completed = in.traj_completed;
  traj.elapsed_ms = in.traj_elapsed_ms;
  traj.total_ms = in.traj_total_ms;
  traj.target_count = in.traj_target_count;
  traj.target_ids = in.traj_target_ids;  // 同类型序列

  // ---- SystemStatus ----
  sys.sync_time_ns = in.sync_time_ns;
  sys.last_error = in.last_error;
  sys.last_error_motor = in.last_error_motor;
  sys.refresh_seq = in.refresh_seq;
  sys.set_zero_seq = in.set_zero_seq;
  sys.clear_fault_seq = in.clear_fault_seq;
  sys.save_seq = in.save_seq;
  sys.save_in_progress = in.save_in_progress;

  // ---- MotorDiagnostics ----
  diag.sync_time_ns = in.sync_time_ns;
  diag.rx_frame_count = in.rx_frame_count;
  diag.unknown_frame_count = in.unknown_frame_count;
  diag.decode_error_count = in.decode_error_count;
  diag.snapshot_accepted = in.snapshot_accepted;
  diag.snapshot_rejected = in.snapshot_rejected;
  diag.config_rejected = in.config_rejected;
  diag.event_dropped = in.event_dropped;
  diag.tx_dropped = in.tx_dropped;
  diag.p0_retry = in.p0_retry;
  diag.trajectory_accepted = in.trajectory_accepted;
  diag.trajectory_rejected = in.trajectory_rejected;
  diag.policy_accepted = in.policy_accepted;
  diag.policy_rejected = in.policy_rejected;
  diag.mailbox_rejected_command = in.mailbox_rejected_command;
  diag.mailbox_rejected_trajectory = in.mailbox_rejected_trajectory;
  diag.mailbox_rejected_policy = in.mailbox_rejected_policy;
  diag.publish_fail = in.publish_fail;
  diag.ring_drop = in.ring_drop;            // uint32[<=3]，同类型
  diag.rx_hw_lost = in.rx_hw_lost;
  diag.tx_frame_count = in.tx_frame_count;
  diag.heap_free = in.heap_free;
  return r;
}
```

`std::nan("")` 需要 `<cmath>`（上面已经 include 了）。

- [ ] **Step 4: 跑测试**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select can_bridge_msgs micro_ros_bridge
colcon test --packages-select micro_ros_bridge && colcon test-result --verbose
```

预期：`test_messages` 全部用例与前面两个测试文件全绿（具体条数以实跑为准，别照抄一个数字）。

- [ ] **Step 5: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/micro_ros_bridge
git commit -m "feat(micro_ros_bridge): 消息级编解码

整条受理或整条拒绝的口径与固件旧契约逐字一致（先全量校验再写输出，
不留半条）；命令路径的 NaN/Inf 整条拒收；越界饱和计数回给调用方。
state_age_ms / target_source 出自 10 Hz 的 motor_status，经 freshness 出参交给
bridge 缓存，再由它盖章进 motion_states——页面读的是那里的这两个字段。"
```

---

## Task 10: 桥节点（已实现、未接线）

**Files:**
- Create: `ros2_ws/src/micro_ros_bridge/include/micro_ros_bridge/bridge_node.hpp`
- Create: `ros2_ws/src/micro_ros_bridge/src/bridge_node.cpp`
- Create: `ros2_ws/src/micro_ros_bridge/src/bridge_node_main.cpp`
- Create: `ros2_ws/src/micro_ros_bridge/test/test_bridge_node.cpp`
- Create: `ros2_ws/src/micro_ros_bridge/README.md`
- Modify: `ros2_ws/src/micro_ros_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 9 的四个编解码函数
- Produces: 可执行 `micro_ros_bridge_node`；类 `micro_ros_bridge::BridgeNode`

- [ ] **Step 1: 写失败测试（图接线）**

```cpp
// 节点图接线测试：话题名与 QoS 是**契约**，不是实现细节。
// 话题名取自 WireContract 常量，QoS 按消息性质选（高频流 vs 低频状态声明）。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/codec.hpp"

// 线上话题名是契约：少一个、拼错一个，症状都是"页面某块永远没数"。
TEST(BridgeNode, WireTopicsAreDeclared) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  namespace t = micro_ros_bridge::contract::topic;
  for (const auto * name : {&t::kLowCmd, &t::kPolicy, &t::kTrajectory, &t::kConfig,
      &t::kLowState, &t::kMotorStatus, &t::kTopology, &t::kEvents, &t::kDeviceParam,
      &t::kImuData, &t::kImuTemp})
  {
    EXPECT_EQ(topics.count(*name), 1u) << "线上话题缺失：" << *name;
  }
  rclcpp::shutdown();
}

// 主机侧话题名是页面与动作库的接口。
TEST(BridgeNode, HostTopicsAreDeclared) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  for (const char * name : {"/motor/command_array", "/motor/motion_states",
      "/motor/status_array", "/motor/trajectory_state", "/motor/system_status",
      "/motor/diagnostics", "/motor/param_set", "/motor/system_command",
      "/motor/policy_array", "/motor/trajectory", "/motor/topology", "/motor/events",
      "/motor/device_param_report", "/imu/data", "/imu/temp"})
  {
    EXPECT_EQ(topics.count(name), 1u) << "主机侧话题缺失：" << name;
  }
  rclcpp::shutdown();
}

// 端到端（同进程）：主机侧命令进 → 线上 low_cmd 出，且已被定点化。
TEST(BridgeNode, CommandArrayBecomesLowCommand) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  auto pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  can_bridge_msgs::msg::LowCommand got;
  std::atomic<bool> received{false};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](can_bridge_msgs::msg::LowCommand::SharedPtr m) { got = *m; received = true; });

  can_bridge_msgs::msg::MotorCommandArray in;
  in.count = 1;
  can_bridge_msgs::msg::MotorCommandTarget t;
  t.motor_id = 0;
  t.position = 0.5;
  in.targets.push_back(t);
  pub->publish(in);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
  }
  ASSERT_TRUE(received) << "2 s 内没有收到 low_cmd";
  EXPECT_EQ(got.count, 1);
  EXPECT_EQ(got.motor_ids[0], 0);
  rclcpp::shutdown();
}

// ADR 0015 决定 8：两端镜像不同步时拒绝发命令并报错，不静默按旧布局解析。
TEST(BridgeNode, RefusesCommandsOnSchemaMismatch) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  auto status_pub = node->create_publisher<can_bridge_msgs::msg::MotorStatus>(
    micro_ros_bridge::contract::topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto cmd_pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  std::atomic<int> low_cmd_count{0};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&low_cmd_count](can_bridge_msgs::msg::LowCommand::SharedPtr) {++low_cmd_count;});

  auto spin_for = [&exec](std::chrono::milliseconds d) {
      const auto deadline = std::chrono::steady_clock::now() + d;
      while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
      }
    };
  auto send_command = [&]() {
      can_bridge_msgs::msg::MotorCommandArray in;
      in.count = 1;
      can_bridge_msgs::msg::MotorCommandTarget t;
      t.motor_id = 0;
      in.targets.push_back(t);
      cmd_pub->publish(in);
      spin_for(std::chrono::milliseconds(200));
    };
  auto announce_schema = [&](uint16_t v) {
      can_bridge_msgs::msg::MotorStatus st;
      st.schema_version = v;
      st.count = 0;
      status_pub->publish(st);
      spin_for(std::chrono::milliseconds(200));
    };

  announce_schema(99);  // 对端版本不匹配
  const int before = low_cmd_count.load();
  send_command();
  EXPECT_EQ(low_cmd_count.load(), before) << "版本不一致时仍在发命令";

  announce_schema(1);  // 对端回到本机版本 → 恢复放行
  send_command();
  EXPECT_GT(low_cmd_count.load(), before) << "版本一致后仍未放行";
  rclcpp::shutdown();
}
```

在 `CMakeLists.txt` 里加：

```cmake
  ament_add_gtest(test_bridge_node test/test_bridge_node.cpp)
  target_compile_features(test_bridge_node PRIVATE cxx_std_17)
  target_compile_options(test_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_bridge_node ${PROJECT_NAME}_node ${PROJECT_NAME}_messages)
  ament_target_dependencies(test_bridge_node rclcpp)
```

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select micro_ros_bridge
```

预期：编译失败（`bridge_node.hpp: No such file`）。

- [ ] **Step 2: 写节点**

`include/micro_ros_bridge/bridge_node.hpp`：

```cpp
// 线上契约与主机侧之间的**唯一边界节点**。
//
// 订阅（主机侧 → 线上）：/motor/command_array、/motor/param_set、
//   /motor/system_command、/motor/policy_array、/motor/trajectory
// 发布（线上 → 主机侧）：/motor/motion_states、/motor/status_array、
//   /motor/trajectory_state、/motor/system_status、/motor/diagnostics、
//   /motor/topology、/motor/events、/motor/device_param_report，IMU 两条透传。
//
// QoS 按消息性质选，不看业务域：高频流 best-effort depth 1，低频状态声明 reliable。
// 线上话题名一律取自 WireContract 常量——两端不可能拼错。
#ifndef MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
#define MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_

#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <can_bridge_msgs/msg/config_request.hpp>
#include <can_bridge_msgs/msg/device_param_report.hpp>
#include <can_bridge_msgs/msg/low_command.hpp>
#include <can_bridge_msgs/msg/low_state.hpp>
#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_diagnostics.hpp>
#include <can_bridge_msgs/msg/motor_event.hpp>
#include <can_bridge_msgs/msg/motor_motion_state_array.hpp>
#include <can_bridge_msgs/msg/motor_policy_array.hpp>
#include <can_bridge_msgs/msg/motor_status.hpp>
#include <can_bridge_msgs/msg/motor_status_array.hpp>
#include <can_bridge_msgs/msg/motor_topology.hpp>
#include <can_bridge_msgs/msg/motor_trajectory.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_state.hpp>
#include <can_bridge_msgs/msg/param_set_array.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/system_status.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include "micro_ros_bridge/messages.hpp"  // MotorFreshness

namespace micro_ros_bridge
{
class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode();

private:
  // 纯改名透传：消息类型一个字段都不动，只把主机侧话题名与线上话题名对接。
  // 用类型擦除的容器持有，避免为每条透传写一对成员。
  template<typename T>
  void make_pass_through(
    const std::string & wire_topic, const std::string & host_topic, const rclcpp::QoS & qos);

  // 主机侧 → 线上（需要编码的两条）
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr sub_command_;
  rclcpp::Subscription<can_bridge_msgs::msg::ParamSetArray>::SharedPtr sub_param_set_;
  rclcpp::Subscription<can_bridge_msgs::msg::SystemCommand>::SharedPtr sub_system_command_;
  rclcpp::Publisher<can_bridge_msgs::msg::LowCommand>::SharedPtr pub_low_cmd_;
  rclcpp::Publisher<can_bridge_msgs::msg::ConfigRequest>::SharedPtr pub_config_;

  // 线上 → 主机侧（需要解码的两条）
  rclcpp::Subscription<can_bridge_msgs::msg::LowState>::SharedPtr sub_low_state_;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorStatus>::SharedPtr sub_motor_status_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorMotionStateArray>::SharedPtr pub_motion_states_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorStatusArray>::SharedPtr pub_status_array_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTrajectoryState>::SharedPtr pub_traj_state_;
  rclcpp::Publisher<can_bridge_msgs::msg::SystemStatus>::SharedPtr pub_system_status_;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorDiagnostics>::SharedPtr pub_diagnostics_;

  // 透传的生命周期持有者。订阅必须被持有，否则构造完就析构、话题静默消失。
  std::vector<rclcpp::PublisherBase::SharedPtr> pass_pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> pass_subs_;

  // 对端（固件）的 schema_version。0 = 尚未收到，**不据以拦截**（否则起不来）。
  // 非 0 且不等于本机版本时，拒绝一切下行命令（ADR 0015 决定 8）。
  std::atomic<uint16_t> peer_schema_{0};

  // 每台最近一次低频面给出的新鲜度，按 motor_id 索引。low_state 不带这两个量
  // （循环面不分片），但页面读的是 motion_states 元素上的它们（readout.js:34、
  // arm.js:270），所以在这里缓存、在 50 Hz 填充时盖章。构造期定长，之后只写不分配。
  std::vector<MotorFreshness> freshness_ =
    std::vector<MotorFreshness>(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX);

  bool peer_schema_mismatch() const;

  void on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg);
  void on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg);
  void on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg);
};
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__BRIDGE_NODE_HPP_
```

`src/bridge_node.cpp`：

```cpp
#include "micro_ros_bridge/bridge_node.hpp"

#include "micro_ros_bridge/codec.hpp"
#include "micro_ros_bridge/messages.hpp"

namespace micro_ros_bridge
{
namespace
{
// QoS 判据是**消息性质**不是业务域：
//   stream   高频流，丢了就丢了——下一拍就是新的（循环面）
//   reliable 低频状态声明，漏一拍会让页面停在旧值上
// 两端 QoS 必须**相容**：best-effort 发布者配 reliable 订阅者是静默失效
// （不报错、永远收不到）。所以下行链路上"订阅端要 reliable"就必须让上游也 reliable。
rclcpp::QoS stream_qos()
{
  return rclcpp::QoS(1).best_effort();
}
rclcpp::QoS reliable_qos()
{
  return rclcpp::QoS(10).reliable();
}
// 主机侧入口：语义是"最新意图"，depth 1 就够；reliable 因为主机内不该丢。
rclcpp::QoS host_intent_qos()
{
  return rclcpp::QoS(1).reliable();
}
}  // namespace

template<typename T>
void BridgeNode::make_pass_through(
  const std::string & wire_topic, const std::string & host_topic, const rclcpp::QoS & qos)
{
  auto pub = create_publisher<T>(host_topic, qos);
  auto sub = create_subscription<T>(
    wire_topic, qos, [pub](typename T::SharedPtr m) {pub->publish(*m);});
  pass_pubs_.push_back(pub);
  pass_subs_.push_back(sub);
}

BridgeNode::BridgeNode()
: rclcpp::Node("micro_ros_bridge")
{
  using namespace can_bridge_msgs::msg;  // NOLINT(build/namespaces)
  // 命名空间别名，不是对象引用：`contract::topic` 是命名空间，绑不到 `auto &`。
  // 块作用域的 namespace 别名是合法的（`namespace fs = std::filesystem;` 同理）。
  namespace T = contract::topic;

  // ---- 发布者先建齐，再建订阅者（回调里引用它们，虽在构造期不会触发） ----
  pub_low_cmd_ = create_publisher<LowCommand>(T::kLowCmd, stream_qos());
  pub_config_ = create_publisher<ConfigRequest>(T::kConfig, reliable_qos());
  pub_motion_states_ = create_publisher<MotorMotionStateArray>("/motor/motion_states", stream_qos());
  pub_status_array_ = create_publisher<MotorStatusArray>("/motor/status_array", reliable_qos());
  pub_traj_state_ = create_publisher<MotorTrajectoryState>("/motor/trajectory_state", reliable_qos());
  pub_system_status_ = create_publisher<SystemStatus>("/motor/system_status", reliable_qos());
  pub_diagnostics_ = create_publisher<MotorDiagnostics>("/motor/diagnostics", reliable_qos());

  // ---- 主机侧 → 线上 ----
  sub_command_ = create_subscription<MotorCommandArray>(
    "/motor/command_array", host_intent_qos(),
    [this](MotorCommandArray::SharedPtr m) {on_command(m);});
  sub_param_set_ = create_subscription<ParamSetArray>(
    "/motor/param_set", reliable_qos(),
    [this](ParamSetArray::SharedPtr m) {
      if (peer_schema_mismatch()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "schema_version 不一致：拒绝下发参数写");
        return;
      }
      ConfigRequest out;
      const auto r = encode_config(*m, out);
      if (!r.ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "param_set 整条拒收：%s", r.error);
        return;
      }
      pub_config_->publish(out);
    });
  sub_system_command_ = create_subscription<SystemCommand>(
    "/motor/system_command", reliable_qos(),
    [this](SystemCommand::SharedPtr m) {
      if (peer_schema_mismatch()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "schema_version 不一致：拒绝下发系统命令");
        return;
      }
      ConfigRequest out;
      const auto r = encode_config(*m, out);
      if (!r.ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "system_command 整条拒收：%s", r.error);
        return;
      }
      pub_config_->publish(out);
    });

  // ---- 线上 → 主机侧 ----
  sub_low_state_ = create_subscription<LowState>(
    T::kLowState, stream_qos(),
    [this](LowState::SharedPtr m) {on_low_state(m);});
  sub_motor_status_ = create_subscription<MotorStatus>(
    T::kMotorStatus, reliable_qos(),
    [this](MotorStatus::SharedPtr m) {on_motor_status(m);});

  // ---- 纯改名透传（消息类型不变） ----
  make_pass_through<MotorPolicyArray>(T::kPolicy, "/motor/policy_array", reliable_qos());
  make_pass_through<MotorTrajectory>(T::kTrajectory, "/motor/trajectory", reliable_qos());
  make_pass_through<MotorTopology>(T::kTopology, "/motor/topology", reliable_qos());
  make_pass_through<MotorEvent>(T::kEvents, "/motor/events", reliable_qos());
  make_pass_through<DeviceParamReport>(
    T::kDeviceParam, "/motor/device_param_report", reliable_qos());
  make_pass_through<sensor_msgs::msg::Imu>(T::kImuData, "/imu/data", stream_qos());
  make_pass_through<std_msgs::msg::Float32>(T::kImuTemp, "/imu/temp", stream_qos());
}

bool BridgeNode::peer_schema_mismatch() const
{
  const uint16_t known = peer_schema_.load();
  return known != 0 && known != contract::kSchemaVersion;
}

void BridgeNode::on_command(const MotorCommandArray::SharedPtr msg)
{
  // ADR 0015 决定 8：镜像不同步时**拒绝下发**，不静默按旧布局解析。
  if (peer_schema_mismatch()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "固件 schema_version=%u 与本机 %u 不一致：拒绝下发命令",
      peer_schema_.load(), contract::kSchemaVersion);
    return;
  }
  LowCommand out;
  const auto r = encode_low_command(*msg, out);
  if (!r.ok) {
    // 整条拒绝：一个字节都不下发（与固件侧同口径）。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "命令整条拒收：%s", r.error);
    return;
  }
  if (r.saturated != 0) {
    // 饱和说明量程常量选错了，必须在台架上看得见，而不是让命令静默变形。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "命令有 %u 个通道被量程钳位", r.saturated);
  }
  pub_low_cmd_->publish(out);
}

void BridgeNode::on_low_state(const LowState::SharedPtr msg)
{
  MotorMotionStateArray out;
  const auto r = decode_low_state(*msg, out);
  if (!r.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "low_state 丢弃：%s", r.error);
    return;
  }
  // 新鲜度不在这条消息里（循环面不分片），用低频面 10 Hz 缓存的值盖章。
  // 页面读的正是 motion_states 元素上的这两个字段（readout.js:34 的「测量年龄」、
  // arm.js:270 的 target_source），spec §4.7 承诺 JS 一个字段都不用改。
  // 代价：显示值最多滞后一个 motor_status 周期（100 ms）——年龄本身是陈旧度度量。
  for (auto & s : out.states) {
    s.state_age_ms = freshness_[s.motor_id].state_age_ms;
    s.target_source = freshness_[s.motor_id].target_source;
  }
  pub_motion_states_->publish(out);
}

void BridgeNode::on_motor_status(const MotorStatus::SharedPtr msg)
{
  // ADR 0015 决定 8：schema_version 由低频状态面带回，这里 latch 一次。
  // 只在**变化**时报，避免 10 Hz 刷屏。
  const uint16_t peer = msg->schema_version;
  if (peer != peer_schema_.load()) {
    peer_schema_.store(peer);
    if (peer == contract::kSchemaVersion) {
      RCLCPP_INFO(get_logger(), "固件 schema_version=%u 与本机一致，放行命令", peer);
    } else {
      RCLCPP_ERROR(
        get_logger(), "固件 schema_version=%u 与本机 %u 不一致：已停止下发命令",
        peer, contract::kSchemaVersion);
    }
  }

  MotorStatusArray statuses;
  MotorTrajectoryState traj;
  SystemStatus sys;
  MotorDiagnostics diag;
  const auto r = decode_motor_status(*msg, statuses, traj, sys, diag, freshness_);
  if (!r.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "motor_status 丢弃：%s", r.error);
    return;
  }
  pub_status_array_->publish(statuses);
  pub_traj_state_->publish(traj);
  pub_system_status_->publish(sys);
  pub_diagnostics_->publish(diag);
}
}  // namespace micro_ros_bridge
```

`make_pass_through` 是模板成员，只在同一 TU 的构造函数里实例化，因此定义放在 `.cpp` 里即可
（不必进头文件）。

`src/bridge_node_main.cpp`：

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "micro_ros_bridge/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<micro_ros_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
```

`CMakeLists.txt` 加：

```cmake
add_library(${PROJECT_NAME}_node src/bridge_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_messages)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs can_bridge_msgs)

add_executable(micro_ros_bridge_node src/bridge_node_main.cpp)
target_link_libraries(micro_ros_bridge_node ${PROJECT_NAME}_node)
install(TARGETS micro_ros_bridge_node RUNTIME DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: 跑测试**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select micro_ros_bridge
colcon test --packages-select micro_ros_bridge --pytest-args "-x" && colcon test-result --verbose
```

预期：`test_bridge_node` 四条全绿（两个话题表 + 端到端 + schema 门禁），且
`test_contract` / `test_codec` / `test_messages` 仍绿。

- [ ] **Step 4: 写包 README**

`ros2_ws/src/micro_ros_bridge/README.md` 要含：包的定位（唯一边界节点）、
**主机话题表**（唯一宿主）、线上话题表指向固件 README、codec 的定点公式与量程、
QoS 判据、以及"本包尚未接线任何 launch"的状态说明。

- [ ] **Step 5: 全仓库命名门禁**

```bash
cd /home/ly/robot_ws
grep -rn --include=*.cpp --include=*.hpp --include=*.c --include=*.h --include=*.py --include=*.js \
  -e '"micro_ros/do/' -e '"micro_ros/up/' -e '"/micro_ros/do/' -e '"/micro_ros/up/' . \
  --exclude-dir=build --exclude-dir=install --exclude-dir=.pio --exclude-dir=libmicroros \
  | grep -v WireContract \
  | grep -v '^ros2_ws/src/micro_ros_bridge/test/test_contract.cpp:'
```

预期：**无输出**（线上话题名的源码字面量只应出现在 `WireContract.msg` 里；
生成头与文档不算）。有输出说明某处自己拼了话题名，改回引用常量。

最后一行排除的是**唯一一处故意的例外**：`test_contract.cpp` 的
`EXPECT_EQ(kLowCmd, "/micro_ros/do/low_cmd")` 是"按精确值钉住"的钉子本身——它存在的意义
就是持有那份字面量，好让 `WireContract.msg` 被改动时测试变红。它**不是**第二份事实来源
（别名仍指向常量），所以不能算违反门禁。除它之外任何命中都是缺陷。
派发前已实测：加上这行排除后，全仓库该 grep 无输出。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/micro_ros_bridge
git commit -m "feat(micro_ros_bridge): 桥节点（已实现、未接线）

主机侧 /motor/* 与线上 /micro_ros/* 之间的唯一翻译点：5 条下行合 4 条线上
消息、4 条线上消息拆 9 条主机上行，IMU 两条透传。QoS 按消息性质选。
本 commit 不接线任何 launch——切流在下一份计划里做。"
```

---

## 完成本计划后的状态

- 5 条新消息定义可用，固件与主机两侧都能编过；
- 固件侧有编译期尺寸门禁（`pio run` 失败即越界）；
- 两侧定点编解码各有独立测试，用同一组 golden 向量互钉；
- `micro_ros_bridge` 包可构建、可测试，节点已实现但**没有任何 launch 起它**；
- 现有行为**零变化**：旧话题照旧发布，页面照旧工作。

## 与 spec 的偏差汇总（都已核对源码，不是省略）

| # | spec 处 | 本计划 | 理由 |
|---|---|---|---|
| 1 | §4.4 系统段含 `session_state` / `agent_connected` | 不进 `MotorStatus` | 两者都不是"四条旧消息的字段"：`agent_connected` 是内核私有布尔（`motor_control_internal.h`），从未导出；`session_state` 全仓库不存在。facade 的 `SystemStatus` 也无落点，页面顶栏 Agent 状态本就走 `bench_manager`。要加是往 `SystemStatus.msg` 末尾追加两个字段的**新增遥测**，另议 |
| 2 | §4.4 `state_age_ms` uint16 收窄 | 收窄，但真实年龄饱和在 `0xFFFE`，`0xFFFF` 独占"从未收到" | spec 只说"饱和"，没说哨兵与真实值如何区分；不区分就会把 65535 ms 静默读成"从未收到" |
| 3 | §6.1 `encode_config` 单函数 | 两个重载（ParamSet / SystemCommand） | 主机侧是两条独立订阅；单函数必须靠值判"哪组有效"，而默认构造的 `SystemCommand.command == 0` 恰好是 `COMMAND_SAVE`，"缺席"与"SAVE"不可分辨 |
| 4 | §5.3 "host 用例断言所有有界序列 `.data != NULL`" | 改为**固件 init 期断言**（Plan 2） | host 测试按仓库规矩不含 ROS 生成头（22 个 host 测试都是纯逻辑直编），拿不到消息结构体 |
| 5 | §4.4 / §8 尺寸门禁写"host 用例断言 ≤ 2048 B" | 移到固件编译期 `static_assert`（Task 5） | 主机侧 C++ 类型是 `std::vector`，拿不到 `__MAX_SIZE`；C 侧才有。门禁必须落在能拿到编译期上界的那一侧 |
| 6 | §5.1 `lib/wire_codec` 属 S3 | 提到 Plan 1（Task 6） | 它是纯逻辑、无 ROS 依赖，与主机 codec 用同一组 golden 向量互钉才有效；分两份计划写会失去互钉的时机 |
| 7 | §4.3 `low_state.seq` 是主机侧丢帧判据 | facade 的 `MotorMotionStateArray` 末尾**追加** `uint32 seq`（Task 2） | 不追加则该判据在主机侧没有落点，解码时只能丢掉——它是 best-effort 无重传下唯一的丢帧证据 |
| 8 | §4.4 / §8 `MotorStatus ≤ 2048 B` 门禁 | **推迟到 Plan 2**（`status_bridge_init()` 的运行期测量） | 精确 CDR 长度要按 40 多项字段算，写成 `static_assert` 冗长且会与消息定义漂移；主机侧 C++ 类型是 `std::vector` 拿不到上界。能拿到真实长度的时机是 Plan 2 |
| 9 | ADR 0015 决定 8（bridge 发现版本不一致时拒绝发命令） | 进 Plan 1 的 Task 10（含 gtest） | 它是决策记录里的一条，且实现只需要在节点里 latch 一个版本号；推迟意味着 Plan 1 交付一个无视版本门禁的 bridge |
| 10 | §5.3 "消息尺寸加 `static_assert` 上限" 隐含"常量值也一并断言" | Task 5 只断言尺寸与枚举常量 | rosidl 给 C 侧生成 `static const float` / `static const char * const`，**不是** C++ 常量表达式（[expr.const] 只放行 `constexpr` 与 const 整型/枚举），进不了 `static_assert`；真工具链上五种形态实测皆失败。量程与话题名的值改由 Task 7 的主机侧契约用例按**精确值**钉住，覆盖面反而更强（不是"大于零"而是"等于 12.5/64.0/64.0"）。话题名绝对性另见 spec §4.1：前导斜杠本就是待 `rclc_*_init` 验证的暂定项，在固件接线前断言它反而会把未决项钉死 |
| 11 | §4.6 "越界**饱和**并置饱和位" 未定义边界点算不算越界 | 恰好落在 ±full-scale **不算**越界、不置位（Task 6 实现用严格不等） | 满量程是可精确表示的（32767 就在 int16 内），没有任何精度损失；把它算成饱和会让一条正常命令在页面上显示饱和位，饱和位就失去了"量程常量选错"的信号意义。Task 6 的 `test_saturation`（`assert(!r.saturated)`）与 Task 8 的 `Codec.SaturatesInsteadOfRejecting`（`EXPECT_FALSE(...saturated)`）钉的就是这一点，两侧实现都必须用严格不等 |
| 12 | F4 裁定"`WIRE_SENTINEL_MISSING` 由 `wire_contract_check.cpp` 的 `static_assert` 钉住" | 该断言推迟到 Task 6（`wire_codec.h` 此时才存在） | 断言的两个操作数一个是 `wire_codec.h` 的宏，Task 5 时它还不存在。宏与枚举都是整型常量表达式，所以这条断言成立（与偏差 10 里删掉的两条不同） |

## 下一步（不在本计划内）

第二份计划覆盖 spec §7 的 **S3–S5**：固件 `motor_bridge`/`param_bridge`/`imu_bridge`/
`ros_bridge` 的接线改动与 `lib/publish_schedule`、新模块 `status_bridge`、
主机侧 `motor_command`/`web_host`/`robot_dynamics`/`bench_manager`/`robot_bringup` 的改名与
接线、以及清场。写第二份计划前必须先读：`src/motor_bridge.cpp`、`src/param_bridge.cpp`、
`src/ros_bridge.cpp`、`src/imu_bridge.cpp`、`src/motor_rt_task.cpp`、
`web_host/web/js/core/format.js`、`bench_manager` 的两个前馈服务、`robot_bringup` 的 launch。

**Plan 2 必须接住的两项挂账**（Plan 1 明确没做，别在切流时漏掉）：

1. **`MotorStatus ≤ 2048 B` 的尺寸门禁**（偏差 8）。spec §5.3 把它与 `low_cmd`/`low_state`
   并列，但 §4.4 又说归主机用例——两处口径不一致。Plan 1 的裁定是：主机侧 C++ 类型是
   `std::vector` 拿不到 `__MAX_SIZE`，固件侧算 40 多项字段的 CDR 长度写成 `static_assert`
   冗长且会与消息定义漂移，所以**推迟到 Plan 2 在 `status_bridge_init()` 里运行期测量**
   （序列化一条 count=64 的 `MotorStatus` 看长度）。若 Plan 2 不做，这条 spec 要求就无人认领。
2. **`lib/wire_codec` 在固件侧接线**。Plan 1 只交付库与 host 测试；`motor_bridge` 的实际调用
   发生在 Plan 2。库本身已由 Task 6 的契约门禁 include 进固件构建，所以固件工具链下的编译
   错误不会拖到那时才暴露。
