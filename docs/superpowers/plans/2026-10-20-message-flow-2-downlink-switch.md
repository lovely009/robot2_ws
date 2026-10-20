# 固件消息流（二）：下行切流 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把固件下行从旧的五条话题切到线上契约的 `low_cmd` / `config` / `policy` /
`trajectory`，并让主机侧所有命令源改发 `/motor/*`，使下行只剩 `micro_ros_bridge` 一个编码者。

**Architecture:** 固件 `motor_bridge` 改订 `LowCommand`（定点解码走 `lib/wire_codec`），
`param_bridge` 两条订阅合并为一条 `ConfigRequest`；主机侧命令源（页面、节拍器、叠加节点、
台架脚本）统一落到 bridge 的 `/motor/*` 入口。上行在本阶段**保持旧话题与旧节拍不变**，
页面照常有数，S4 再切。

**Tech Stack:** STM32H723 + PlatformIO + micro-ROS（rclc）、ROS 2 Humble（rclcpp / rclpy）、
CTest（固件 host，无 gtest）、colcon/gtest/pytest（主机）。

**Spec:** `docs/superpowers/specs/2026-10-19-firmware-message-flow-design.md`（§5.1、§6.2、
§6.3、§7 的 S3），决策见 `docs/adr/0015-firmware-message-planes-pdo-sdo.md`。
本计划只实现 **S3**；S4（上行切流）与 S5（清场）另立计划。

## Global Constraints

- 本阶段**只切下行**。上行八条旧话题（`motion_states`、`motor_status_array`、
  `system_status`、`trajectory_state`、`diagnostics`、`motor_topology`、`events`、
  `device_param_report`）与 IMU 两条**不改名、不改节拍**——页面在 S3 期间必须继续可用。
- 固件话题名一律**相对名或 `WireContract` 生成常量**（绝对名在根命名空间解析结果相同）；
  主机侧一律 `/motor/*`（IMU 为 `/imu/*`）。线上话题名与量程只来自
  `micro_ros_agent/src/can_bridge_msgs/msg/WireContract.msg`，不在别处写第二份。
- 单位边界：协议定点 ↔ SI 的换算只发生在 `lib/wire_codec`（固件）与
  `micro_ros_bridge/messages.cpp`（主机）。回调里不许出现裸 `32767`。
- 校验口径不变：下行仍是「整条受理或整条拒绝」，缺失电机 = 软停，NaN/Inf 整条拒收
  （编码端拒），`motor_id ∈ [0,63]`。
- QoS 必须两端相容：`low_cmd` = best-effort depth 1；`config` / `policy` / `trajectory`
  = reliable。主机侧 `/motor/command_array` 是**主机内**通路，改成 reliable depth 1
  （bridge 订阅端已经是 reliable，不改它）。
- 固件 host 测试直接编译真实 `.c`，无桩；新增内核 `.c` 只改
  `test/host/CMakeLists.txt` 一处。测试不得启动 Gazebo / MuJoCo、不得占 vcan / 串口 /
  真实硬件。
- 本计划**不改 `.msg`**，因此不需要 `refresh_microros_messages.py`；若实施中不得不改，
  走离线增量刷新，**禁止** `pio run -t clean_microros`。
- 文档：一份事实一处。线上话题表宿主是 `micro_ros_firmware/README.md`；主机话题表宿主是
  `ros2_ws/src/micro_ros_bridge/README.md`；其余 README 只写自己的改动。
- 台架步骤（Task 5）需要真机，不进 CI；固件 host 与 colcon 测试不得依赖它们。

## File Structure

| 文件 | 职责 / 本阶段改动 |
|---|---|
| `micro_ros_firmware/lib/wire_codec/{include/wire_codec.h,src/wire_codec.c}` | 新增两个纯函数：循环面槽位形状校验、配置面两组字段互斥校验 |
| `micro_ros_firmware/src/wire_contract_check.cpp` | 把 `wire_codec` 的 kind 常量钉到 `ConfigRequest.msg` 生成常量上 |
| `micro_ros_firmware/test/host/test_wire_codec.cpp` | 两个纯函数的 host 用例 |
| `micro_ros_firmware/src/motor_bridge.cpp` | 订阅 `LowCommand`；policy / trajectory 改用契约名 |
| `micro_ros_firmware/src/param_bridge.cpp` | 两条订阅合并为 `ConfigRequest` |
| `micro_ros_firmware/include/param_bridge.h` | executor 句柄数 2 → 1；话题注释更新 |
| `ros2_ws/src/motor_command/motor_command/command_streamer.py` | 输出话题 `/motor/command_array`，输出 QoS 改 reliable |
| `ros2_ws/src/bench_manager/bench_manager/gravity_feedforward_manager.py` | 直连落点改名 `/motor/command_array` |
| `ros2_ws/src/robot_dynamics/{include,src}/.../gravity_feedforward_overlay_node.*` | 输入/输出默认话题改 `/motor/*`；命令输出 QoS 改 reliable |
| `ros2_ws/src/web_host/web/js/core/{format.js,agent.js}` | 下行 TOPICS 前缀改 `/motor/*` |
| `ros2_ws/src/micro_ros_bridge/README.md` | 去掉「未接入 launch」 |
| `ros2_ws/src/robot_bringup/launch/robot.launch.py` | L1 增加 `micro_ros_bridge` 节点 |
| `ros2_ws/src/robot_bringup/test/test_robot_launch.py` | 静态断言 bridge 随入口起 |
| `micro_ros_agent/tools/{traj_play.py,snapshot_stream.py}` | 下行话题改 `/motor/*` |
| `ros2_ws/src/motor_hardware_interface/scripts/firmware_topic_bridge.py` | 轨迹输出改 `/motor/trajectory` |
| 各 README / `docs/guides/fdcan1-bench-guide.md` | 下行话题名同步 |

---

## Task 1: 固件 `motor_bridge` 改订 `LowCommand`

**Files:**
- Create: 无
- Modify: `micro_ros_firmware/lib/wire_codec/include/wire_codec.h`
- Modify: `micro_ros_firmware/lib/wire_codec/src/wire_codec.c`
- Modify: `micro_ros_firmware/test/host/test_wire_codec.cpp`
- Modify: `micro_ros_firmware/src/motor_bridge.cpp`

**Interfaces:**
- Produces: `bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
  size_t position_q_len, size_t velocity_q_len, size_t effort_q_len, size_t max_slots);`
  —— Task 1 在 `motor_bridge` 的 `low_command_cb` 里消费。
- Consumes: 生成常量 `can_bridge_msgs__msg__WireContract__TOPIC_LOW_CMD` /
  `__TOPIC_POLICY` / `__TOPIC_TRAJECTORY` / `__SCALE_POSITION_RAD` /
  `__SCALE_VELOCITY_RAD_S` / `__SCALE_EFFORT_NM`。

- [ ] **Step 1: 写失败的 host 测试**

在 `micro_ros_firmware/test/host/test_wire_codec.cpp` 的匿名命名空间里加：

```cpp
void test_frame_shape()
{
  // count 必须等于四个序列的声明长度，且不超过内核容量。
  // 0 = 空目标列表 = 全部软停，合法。
  assert(wire_frame_shape_ok(2, 2, 2, 2, 2, 64));
  assert(wire_frame_shape_ok(0, 0, 0, 0, 0, 64));
  assert(!wire_frame_shape_ok(2, 2, 2, 2, 1, 64));
  assert(!wire_frame_shape_ok(2, 1, 2, 2, 2, 64));
  assert(!wire_frame_shape_ok(2, 2, 2, 2, 2, 1));
  assert(!wire_frame_shape_ok(65, 65, 65, 65, 65, 64));
}
```

在 `main()` 里 `test_non_finite_yields_sentinel();` 之后加：

```cpp
  test_frame_shape();
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake -S test/host -B .host-build
cmake --build .host-build --target test_wire_codec -j2
```

预期：编译失败，`'wire_frame_shape_ok' was not declared in this scope`。

- [ ] **Step 3: 实现纯函数**

`wire_codec.h` 在 `#include <stdbool.h>` 上方补 `#include <stddef.h>`，并在
`wire_dequantize` 声明之后加：

```c
// 循环面槽位形状：count 必须等于四个序列的声明长度，且不超过内核容量。
// 只校验「同一帧内的自洽性」，不解释内容——值的合法性归 mailbox。
bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots);
```

`wire_codec.c` 末尾加：

```c
bool wire_frame_shape_ok(uint8_t count, size_t motor_ids_len,
    size_t position_q_len, size_t velocity_q_len, size_t effort_q_len,
    size_t max_slots) {
  return count <= max_slots &&
         motor_ids_len == count && position_q_len == count &&
         velocity_q_len == count && effort_q_len == count;
}
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build --target test_wire_codec -j2
ctest --test-dir .host-build -R test_wire_codec --output-on-failure
```

预期：`100% tests passed, 0 tests failed out of 1`。

- [ ] **Step 5: 改 `motor_bridge.cpp` 的订阅**

头部 include 区加：

```cpp
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>
#include <can_bridge_msgs/msg/low_command.h>
#include "wire_codec.h"
```

把话题名常量（`src/motor_bridge.cpp:59-69`）改成：

```cpp
// 下行三条改用契约常量（绝对名，根命名空间下与旧相对名解析结果相同）。
// 上行仍是旧话题：S4 切流之前页面靠它们取数。
static const char *kCommandTopic =
    can_bridge_msgs__msg__WireContract__TOPIC_LOW_CMD;
static const char *kTrajectoryTopic =
    can_bridge_msgs__msg__WireContract__TOPIC_TRAJECTORY;
static const char *kPolicyTopic =
    can_bridge_msgs__msg__WireContract__TOPIC_POLICY;
```

把静态命令缓冲（`src/motor_bridge.cpp:63` 附近）从

```cpp
static can_bridge_msgs__msg__MotorCommandTarget g_command_targets_buf[kRosArrayCapacity];
```

改成四个 SoA 缓冲：

```cpp
static uint8_t g_command_ids_buf[kRosArrayCapacity];
static int16_t g_command_position_buf[kRosArrayCapacity];
static int16_t g_command_velocity_buf[kRosArrayCapacity];
static int16_t g_command_effort_buf[kRosArrayCapacity];
```

把订阅状态块（`src/motor_bridge.cpp:76-79`）改成：

```cpp
static rcl_subscription_t g_command_sub;
static can_bridge_msgs__msg__LowCommand g_command_msg;
```

- [ ] **Step 6: 改回调**

把 `command_array_cb`（`src/motor_bridge.cpp:145-168`）整体替换为：

```cpp
static void low_command_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__LowCommand *)msgin;
  // 声明长度与四组数组必须自洽；超内核容量一律整条拒绝。NaN/Inf 在编码端已被
  // 拒收，这里解出来的 -32768 只会来自畸形帧——mailbox 的 isfinite 会整条拒。
  if (!wire_frame_shape_ok(msg->count, msg->motor_ids.size,
          msg->position_q.size, msg->velocity_q.size, msg->effort_q.size,
          CONTROL_COMMAND_MAILBOX_MAX_TARGETS)) {
    return;
  }
  control_command_snapshot_t snapshot = {0};
  snapshot.sequence = msg->sequence;
  snapshot.received_tick_ms = millis();
  snapshot.target_count = msg->count;
  for (uint8_t i = 0; i < msg->count; ++i) {
    snapshot.targets[i].motor_id = msg->motor_ids.data[i];
    snapshot.targets[i].position = (float)wire_dequantize(
        msg->position_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_POSITION_RAD);
    snapshot.targets[i].velocity = (float)wire_dequantize(
        msg->velocity_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_VELOCITY_RAD_S);
    snapshot.targets[i].effort = (float)wire_dequantize(
        msg->effort_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_EFFORT_NM);
  }
  (void)motor_rt_submit_command(&snapshot); // 非法快照由 mailbox 拒绝并计数
}
```

- [ ] **Step 7: 改 init 的缓冲接线与订阅类型**

把 `src/motor_bridge.cpp:580-581`

```cpp
  g_command_msg.targets.capacity = kRosArrayCapacity;
  g_command_msg.targets.data = g_command_targets_buf;
```

替换为：

```cpp
  g_command_msg.motor_ids.capacity = kRosArrayCapacity;
  g_command_msg.motor_ids.data = g_command_ids_buf;
  g_command_msg.position_q.capacity = kRosArrayCapacity;
  g_command_msg.position_q.data = g_command_position_buf;
  g_command_msg.velocity_q.capacity = kRosArrayCapacity;
  g_command_msg.velocity_q.data = g_command_velocity_buf;
  g_command_msg.effort_q.capacity = kRosArrayCapacity;
  g_command_msg.effort_q.data = g_command_effort_buf;
```

把订阅初始化（`src/motor_bridge.cpp:636-641`）的

```cpp
  if (rcl_subscription_init(&g_command_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(
      can_bridge_msgs, msg, MotorCommandArray),
      kCommandTopic, &sub_opts) != RCL_RET_OK) {
```

改成：

```cpp
  if (rcl_subscription_init(&g_command_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(
      can_bridge_msgs, msg, LowCommand),
      kCommandTopic, &sub_opts) != RCL_RET_OK) {
```

把 `rclc_executor_add_subscription`（`src/motor_bridge.cpp:759-761`）的回调
`command_array_cb` 改成 `low_command_cb`。

- [ ] **Step 8: 编译固件**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run
```

预期：`[SUCCESS]`。若报 `LowCommand` 未定义，检查 `libmicroros` 是否已含新消息
（`grep TOPIC_LOW_CMD .pio/libdeps/stm32h723vgt6/micro_ros_platformio/libmicroros/include/can_bridge_msgs/msg/detail/wire_contract__struct.h`）——
本计划前置状态里已刷新过。

- [ ] **Step 9: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/wire_codec micro_ros_firmware/test/host micro_ros_firmware/src/motor_bridge.cpp
git commit -m "feat(firmware): 下行命令改订低层契约 LowCommand

motor_bridge 的完整快照订阅从 MotorCommandArray 切到定点的 LowCommand，
解码只经 lib/wire_codec；形状校验（count 与四组数组自洽）抽成纯函数进
host 测试。policy / trajectory 同时改用 WireContract 常量名。上行未动。"
```

---

## Task 2: 固件 `param_bridge` 合并为 `ConfigRequest`

**Files:**
- Modify: `micro_ros_firmware/lib/wire_codec/include/wire_codec.h`
- Modify: `micro_ros_firmware/lib/wire_codec/src/wire_codec.c`
- Modify: `micro_ros_firmware/test/host/test_wire_codec.cpp`
- Modify: `micro_ros_firmware/src/wire_contract_check.cpp`
- Modify: `micro_ros_firmware/include/param_bridge.h`
- Modify: `micro_ros_firmware/src/param_bridge.cpp`

**Interfaces:**
- Produces: `bool wire_config_groups_exclusive(uint8_t kind, uint16_t count,
  size_t targets_len, uint8_t command, uint16_t motor_id);`
  以及宏 `WIRE_CONFIG_KIND_PARAM_SET` / `WIRE_CONFIG_KIND_SYSTEM_COMMAND`。
- Consumes: `can_bridge_msgs__msg__ConfigRequest` 的生成常量
  `CONFIG_KIND_PARAM_SET` / `CONFIG_KIND_SYSTEM_COMMAND`、`SystemCommand` 的
  `COMMAND_*` 与 `MOTOR_ID_NONE`、`ParamSetTarget` 的 `FIELD_*`。

- [ ] **Step 1: 写失败的 host 测试**

在 `test_wire_codec.cpp` 加：

```cpp
void test_config_groups_exclusive()
{
  // PARAM_SET：只带参数组。命令组不适用时，bridge 的编码端写的是
  // command = COMMAND_SAVE(0)、motor_id = MOTOR_ID_NONE(65535)——
  // 不是 0（0 是合法电机 id），照着 .msg 字面「0/空」写会拒掉每一条合法参数写。
  assert(wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 1, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 4, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 0, 0));
  // SYSTEM_COMMAND：只带命令组；目标必须为空。motor_id 可以是 MOTOR_ID_NONE。
  assert(wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_SYSTEM_COMMAND, 0, 0, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(WIRE_CONFIG_KIND_SYSTEM_COMMAND, 1, 1, 0, 0));
  // 未知 kind 整条拒。
  assert(!wire_config_groups_exclusive(7, 0, 0, 0, 0));
}
```

在 `main()` 的 `test_frame_shape();` 之后加：

```cpp
  test_config_groups_exclusive();
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build --target test_wire_codec -j2
```

预期：编译失败，`'WIRE_CONFIG_KIND_PARAM_SET' was not declared`。

- [ ] **Step 3: 实现纯函数**

`wire_codec.h` 在 `wire_frame_shape_ok` 之后加：

```c
// 配置面两组字段互斥（ConfigRequest.msg）。不适用组的哨兵值按**实际编码端**取：
// micro_ros_bridge 的 encode_config(ParamSetArray) 写 command = COMMAND_SAVE(0)、
// motor_id = MOTOR_ID_NONE(65535)。kind/motor_id 哨兵是本库的**受控第二份**，
// 由 src/wire_contract_check.cpp 的 static_assert 钉到生成常量上。
#define WIRE_CONFIG_KIND_PARAM_SET 0U
#define WIRE_CONFIG_KIND_SYSTEM_COMMAND 1U
#define WIRE_MOTOR_ID_NONE 65535U

bool wire_config_groups_exclusive(uint8_t kind, uint16_t count,
    size_t targets_len, uint8_t command, uint16_t motor_id);
```

`wire_codec.c` 末尾加：

```c
bool wire_config_groups_exclusive(uint8_t kind, uint16_t count,
    size_t targets_len, uint8_t command, uint16_t motor_id) {
  if (count != targets_len) {
    return false;
  }
  if (kind == WIRE_CONFIG_KIND_PARAM_SET) {
    return command == 0U && motor_id == WIRE_MOTOR_ID_NONE;
  }
  if (kind == WIRE_CONFIG_KIND_SYSTEM_COMMAND) {
    return count == 0U;
  }
  return false;
}
```

`src/wire_contract_check.cpp` 在现有 sentinel 断言之后加：

```cpp
static_assert(WIRE_CONFIG_KIND_PARAM_SET ==
    can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_PARAM_SET,
    "wire_codec 与 ConfigRequest 的 PARAM_SET 常量漂移了");
static_assert(WIRE_CONFIG_KIND_SYSTEM_COMMAND ==
    can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_SYSTEM_COMMAND,
    "wire_codec 与 ConfigRequest 的 SYSTEM_COMMAND 常量漂移了");
static_assert(WIRE_MOTOR_ID_NONE ==
    can_bridge_msgs__msg__SystemCommand__MOTOR_ID_NONE,
    "wire_codec 的 MOTOR_ID_NONE 与 SystemCommand 漂移了");
```

（该文件已经包含 `config_request__struct.h` 与 `wire_codec.h`；补 include
`<can_bridge_msgs/msg/detail/system_command__struct.h>`。）

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build --target test_wire_codec -j2
ctest --test-dir .host-build -R test_wire_codec --output-on-failure
```

预期：`100% tests passed`。

- [ ] **Step 5: 先把句柄数改成 1（红）**

`micro_ros_firmware/include/param_bridge.h`：

```c
#define PARAM_BRIDGE_EXECUTOR_HANDLES 1U
```

并改注释：「两条订阅」→「一条订阅（`ConfigRequest` 合并了参数写与系统命令）」。

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run
```

预期：编译失败，`static assertion failed: PARAM_BRIDGE_EXECUTOR_HANDLES 必须等于本文件的订阅数`。

- [ ] **Step 6: 改 `param_bridge.cpp` 的常量与状态**

话题常量（`src/param_bridge.cpp:30-32`）改成：

```cpp
static const char *kConfigTopic =
    can_bridge_msgs__msg__WireContract__TOPIC_CONFIG;
static const char *kSystemStatusTopic = "micro_ros_up/system_status"; // S4 前不变
```

include 区加：

```cpp
#include <can_bridge_msgs/msg/config_request.h>
#include <can_bridge_msgs/msg/detail/wire_contract__struct.h>
#include "wire_codec.h"
```

状态块（`src/param_bridge.cpp:102-108`）改成：

```cpp
static rcl_subscription_t g_config_sub;
static rcl_publisher_t g_system_status_pub;
static can_bridge_msgs__msg__ConfigRequest g_config_msg;
static can_bridge_msgs__msg__ParamSetTarget g_config_targets_buf[kParamSetMaxTargets];
static can_bridge_msgs__msg__SystemStatus g_system_status_msg;
static bool g_sub_up = false;
static bool g_pub_up = false;
static bool g_sub_added = false;
static bool g_created = false;
```

`static_assert(PARAM_BRIDGE_EXECUTOR_HANDLES == 2U, ...)` 改成 `== 1U`。

- [ ] **Step 7: 把整条校验 / 分发改成收数组指针**

把 `validate_param_set(const can_bridge_msgs__msg__ParamSetArray *msg)` 整体替换为
（`count != targets.size` 的一致性检查已经由 `wire_config_groups_exclusive` 承担）：

```cpp
static bool validate_param_set(
    const can_bridge_msgs__msg__ParamSetTarget *targets, uint16_t count) {
  if (count > kParamSetMaxTargets) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    return false;
  }
  for (uint16_t i = 0; i < count; ++i) {
    const can_bridge_msgs__msg__ParamSetTarget &t = targets[i];
    if (t.field >= PARAM_DEVICE_SLOT_COUNT ||
        t.motor_id >= MOTOR_CONTROL_MAX_MOTORS) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    const motor_control_param_view_t *v = find_view(t.motor_id);
    if (v == nullptr) {
      report_error(ERR_UNKNOWN_MOTOR, t.motor_id);
      return false;
    }
    // 上界参数传 0：内核侧还会按型号额定再钳一次，本层只做纯值域判定。
    if (!param_value_ok(kFields[t.field].param_field, t.value, 0.0f, nullptr)) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_TORQUE_LIMIT &&
        !isfinite(param_units_amp_to_nm(t.value, v->torque_per_amp))) {
      report_error(ERR_LOCAL_VALIDATION, t.motor_id);
      return false;
    }
    // 同一 (motor_id, field) 出现两次：静默取一个会掩盖调用方 bug。
    for (uint16_t j = 0; j < i; ++j) {
      if (targets[j].motor_id == t.motor_id && targets[j].field == t.field) {
        report_error(ERR_LOCAL_VALIDATION, t.motor_id);
        return false;
      }
    }
  }
  // 位置限位成对：同一条消息内该台必须两边都给，且 min < max。
  const uint8_t pos_bits = (uint8_t)((1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) |
      (1U << can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX));
  for (uint16_t i = 0; i < count; ++i) {
    const uint16_t id = targets[i].motor_id;
    uint8_t mask = 0U;
    float lo = NAN;
    float hi = NAN;
    for (uint16_t j = 0; j < count; ++j) {
      const can_bridge_msgs__msg__ParamSetTarget &p = targets[j];
      if (p.motor_id != id) {
        continue;
      }
      mask |= (uint8_t)(1U << p.field);
      if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) {
        lo = p.value;
      } else if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
        hi = p.value;
      }
    }
    if ((mask & pos_bits) == 0U) {
      continue; // 本台没写位置限位
    }
    if ((mask & pos_bits) != pos_bits) {
      report_error(ERR_LOCAL_VALIDATION, id);
      return false;
    }
    if (!param_position_limits_ok(lo, hi)) {
      report_error(ERR_LOCAL_VALIDATION, id);
      return false;
    }
  }
  return true;
}
```

把 `apply_param_set(const ParamSetArray *msg, uint32_t request_id)` 整体替换为：

```cpp
static void apply_param_set(
    const can_bridge_msgs__msg__ParamSetTarget *targets, uint16_t count,
    uint32_t request_id) {
  bool consumed[kParamSetMaxTargets] = {};
  for (uint16_t i = 0; i < count; ++i) {
    if (consumed[i]) {
      continue;
    }
    const can_bridge_msgs__msg__ParamSetTarget &t = targets[i];
    const motor_control_param_view_t *v = find_view(t.motor_id);
    if (v == nullptr) {
      continue; // 预校验已确认存在；防御
    }
    if (t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN ||
        t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
      float lo = NAN;
      float hi = NAN;
      for (uint16_t j = 0; j < count; ++j) {
        const can_bridge_msgs__msg__ParamSetTarget &p = targets[j];
        if (p.motor_id != t.motor_id) {
          continue;
        }
        if (p.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN) {
          lo = p.value;
          consumed[j] = true;
        } else if (p.field ==
            can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX) {
          hi = p.value;
          consumed[j] = true;
        }
      }
      if (!submit_position_limits(v, lo, hi, request_id)) {
        return; // last_error 已置；继续下发只会扩大不一致
      }
      continue;
    }
    consumed[i] = true;
    if (!dispatch_target(v, t.field, t.value, request_id)) {
      return;
    }
  }
}
```

- [ ] **Step 8: 把系统命令处理改成显式参数**

把 `system_command_cb(const void *msgin)` 整体替换为：

```cpp
static void handle_system_command(uint8_t command, uint16_t motor_id,
    uint32_t request_id) {
  if (command == can_bridge_msgs__msg__SystemCommand__COMMAND_SAVE) {
    // 只接受整机保存：W25 那一半必须整体落盘，按台保存只省设备侧一半。
    if (motor_id != can_bridge_msgs__msg__SystemCommand__MOTOR_ID_NONE) {
      report_error(ERR_LOCAL_VALIDATION, motor_id);
      return;
    }
    save_transaction_request(&g_save_transaction);
    return;
  }

  if (command == can_bridge_msgs__msg__SystemCommand__COMMAND_REPORT) {
    // 总是整机一帧，motor_id 忽略；重复 REPORT 合并为一次。
    g_report_pending = true;
    return;
  }

  control_mailbox_config_kind_t kind;
  switch (command) {
  case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH:
    kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
    break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO:
    kind = CONTROL_MAILBOX_CONFIG_SET_ZERO;
    break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT:
    kind = CONTROL_MAILBOX_CONFIG_CLEAR_FAULT;
    break;
  default:
    report_error(ERR_LOCAL_VALIDATION, motor_id);
    return;
  }

  const motor_control_param_view_t *v = (motor_id < MOTOR_CONTROL_MAX_MOTORS)
                                            ? find_view(motor_id)
                                            : nullptr;
  // 未绑定型号的台没有设备事务可做（内核会回 UNSUPPORTED），本地就拦掉。
  if (v == nullptr || !v->bound) {
    report_error(ERR_UNKNOWN_MOTOR, motor_id);
    return;
  }

  if ((request_id & PARAM_INTERNAL_ID_TAG) != 0U) {
    report_error(ERR_LOCAL_VALIDATION, motor_id);
    return;
  }
  if (request_id == 0U) {
    request_id = next_internal_id();
  }

  control_mailbox_config_request_t request = {};
  request.request_id = request_id;
  request.received_tick_ms = millis();
  request.motor_id = motor_id;
  request.kind = kind;
  if (!motor_rt_submit_config(&request)) {
    report_error(ERR_SUBMIT_FAILED, motor_id);
    return;
  }

  // 同一命令重复下发：后来的取代先前的（旧的在飞事件到达时 id 已不匹配）。
  switch (command) {
  case can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH:
    g_refresh_id = request_id;
    break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO:
    g_set_zero_id = request_id;
    break;
  case can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT:
    g_clear_fault_id = request_id;
    break;
  default:
    break;
  }
}
```

- [ ] **Step 9: 新的 `config_cb`**

在 `handle_system_command` 之后加：

```cpp
static void config_cb(const void *msgin) {
  const auto *msg = (const can_bridge_msgs__msg__ConfigRequest *)msgin;
  if (msg == nullptr) {
    return;
  }
  // kind 决定哪组有效，另一组必须为空——两组混装是发送方误解契约，整条拒。
  if (!wire_config_groups_exclusive(msg->kind, msg->count,
          msg->targets.size, msg->command, msg->motor_id)) {
    report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
    return;
  }
  if (msg->kind == WIRE_CONFIG_KIND_PARAM_SET) {
    if (!validate_param_set(msg->targets.data, msg->count)) {
      return;
    }
    uint32_t request_id = msg->request_id;
    if ((request_id & PARAM_INTERNAL_ID_TAG) != 0U) {
      report_error(ERR_LOCAL_VALIDATION, kMotorIdNone);
      return;
    }
    if (request_id == 0U) {
      request_id = next_internal_id();
    }
    apply_param_set(msg->targets.data, msg->count, request_id);
    return;
  }
  handle_system_command(msg->command, msg->motor_id, msg->request_id);
}
```

- [ ] **Step 10: 改 init / destroy**

`src/param_bridge.cpp:768-772` 的缓冲接线改成：

```cpp
  g_config_msg.targets.capacity =
      sizeof(g_config_targets_buf) / sizeof(g_config_targets_buf[0]);
  g_config_msg.targets.data = g_config_targets_buf;
  if (g_config_msg.targets.capacity < kParamSetMaxTargets) {
    return false;
  }
```

把两条订阅初始化（`:794-812`）替换为一条：

```cpp
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  // 配置/系统命令面是低频状态声明，reliable；< 512 B MTU，不分片。
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_config_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, ConfigRequest),
      kConfigTopic, &sub_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_up = true;
```

`rclc_executor_add_subscription`（`:814-826`）替换为：

```cpp
  if (rclc_executor_add_subscription(executor, &g_config_sub,
      &g_config_msg, config_cb, ON_NEW_DATA) != RCL_RET_OK) {
    param_bridge_destroy(node, executor);
    return false;
  }
  g_sub_added = true;
```

`param_bridge_destroy`（`:834-877`）删掉 `g_cmd_sub_*` 的全部分支，只留
`g_config_sub` 的移除与 fini。进入函数时的空检查改成
`if (!g_created && !g_sub_up && !g_pub_up)`。

- [ ] **Step 11: 编译固件**

```bash
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run
```

预期：`[SUCCESS]`。

- [ ] **Step 12: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/lib/wire_codec micro_ros_firmware/test/host micro_ros_firmware/src/wire_contract_check.cpp micro_ros_firmware/include/param_bridge.h micro_ros_firmware/src/param_bridge.cpp
git commit -m "feat(firmware): 参数写与系统命令合并到 ConfigRequest

param_bridge 的两条订阅收敛为一条 micro_ros_do/config：kind 决定参数组或
命令组，两组混装整条拒绝。整条校验/分发改成收数组指针，系统命令处理抽成
显式参数；kind 互斥校验进 host 测试，kind 常量由 static_assert 钉住。"
```

---

## Task 3: 主机侧命令源全部落到 `/motor/*`

**Files:**
- Modify: `ros2_ws/src/motor_command/motor_command/command_streamer.py`
- Modify: `ros2_ws/src/motor_command/test/test_command_streamer.py`
- Modify: `ros2_ws/src/bench_manager/bench_manager/gravity_feedforward_manager.py`
- Modify: `ros2_ws/src/robot_dynamics/include/robot_dynamics/ros/gravity_feedforward_overlay_node.hpp`
- Modify: `ros2_ws/src/robot_dynamics/src/ros/gravity_feedforward_overlay_node.cpp`
- Modify: `ros2_ws/src/robot_dynamics/test/test_gravity_feedforward_overlay.cpp`
- Modify: `ros2_ws/src/web_host/web/js/core/format.js`
- Modify: `ros2_ws/src/web_host/web/js/core/agent.js`
- Modify: `ros2_ws/src/web_host/test/test_web_assets.py`
- Modify: `micro_ros_agent/tools/traj_play.py`
- Modify: `micro_ros_agent/tools/snapshot_stream.py`
- Modify: `ros2_ws/src/motor_hardware_interface/scripts/firmware_topic_bridge.py`

**Interfaces:**
- Consumes: `micro_ros_bridge` 已存在的主机入口——`/motor/command_array`
  （`MotorCommandArray`，reliable depth 1）、`/motor/param_set`、`/motor/system_command`、
  `/motor/policy_array`、`/motor/trajectory`（后四条 reliable depth 10）。
- Produces: 所有命令源只发 `/motor/*`，不再直接发 `/micro_ros_do/*`。

- [ ] **Step 1: 写失败测试**

`ros2_ws/src/motor_command/test/test_command_streamer.py`：

1. 顶部 import 改：

```python
from motor_command.command_streamer import (CLIENT_COUNT_QOS, COMMAND_OUTPUT_QOS,
                                            SETPOINT_QOS, CommandStreamer)
```

2. QoS 用例改成：

```python
def test_command_output_qos_is_reliable():
    """命令流在主机内走：bridge 订阅端是 reliable，节拍器必须同一条判据。

    也顺带修掉叠加路线：overlay 的命令输入订阅是 reliable，发 best-effort
    在 ROS 2 里**不建连接**（静默失效）。
    """
    assert COMMAND_OUTPUT_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert COMMAND_OUTPUT_QOS.durability == DurabilityPolicy.VOLATILE
    assert COMMAND_OUTPUT_QOS.depth == 1
```

3. 所有 `/micro_ros_do/motor_command_array` 字面量改成 `/motor/command_array`。

`ros2_ws/src/robot_dynamics/test/test_gravity_feedforward_overlay.cpp`：

1. 在文件末尾（`Harness` 之前或独立用例区）加：

```cpp
TEST(GravityFeedforwardOverlayDefaults, OutputsGoThroughBridge) {
  // 叠加节点是主机侧命令源之一：输出必须落在 bridge 的主机入口，
  // 直发线上话题会让命令绕过 schema 门禁与定点编码。
  robot_dynamics::GravityFeedforwardOverlayParameters parameters;
  EXPECT_EQ(parameters.command_output_topic, "/motor/command_array");
  EXPECT_EQ(parameters.trajectory_output_topic, "/motor/trajectory");
}
```

2. `Harness` 里观察命令输出的订阅 QoS 从 `best_effort()` 改成 `reliable()`，
并把注释「命令输出是 best_effort」改成「命令输出是 reliable（bridge 订阅端）」：

```cpp
    command_subscription =
        observer->create_subscription<can_bridge_msgs::msg::MotorCommandArray>(
            topics.command_output_topic,
            rclcpp::QoS(1).reliable().durability_volatile(),
            [this](const can_bridge_msgs::msg::MotorCommandArray & message) {
              commands.push_back(message);
            });
```

`ros2_ws/src/web_host/test/test_web_assets.py` 加：

```python
def test_downlink_topics_are_host_facade():
    """页面的下行只发主机侧 /motor/*：线上编码与文件名是 bridge 的私有实现。

    这五条在 S3 之后不再由固件直接订阅；页面若漏改一条，症状是「按钮点了没反应」
    而 ROS 侧毫无报错。
    """
    js = FORMAT_JS.read_text(encoding='utf-8')
    expected = {
        'trajectory': '/motor/trajectory',
        'policy': '/motor/policy_array',
        'paramSet': '/motor/param_set',
        'sysCmd': '/motor/system_command',
    }
    for key, topic in expected.items():
        m = re.search(
            rf"^\s*{key}:\s*{{name:\s*'([^']+)'", js, re.M)
        assert m, f'format.js 里找不到 TOPICS.{key}'
        assert m.group(1) == topic, f'{key} 仍发线上话题：{m.group(1)}'

    agent_js = AGENT_JS.read_text(encoding='utf-8')
    m = re.search(r"DIRECT_COMMAND_TOPIC\s*=\s*'([^']+)'", agent_js)
    assert m and m.group(1) == '/motor/command_array', '直连落点仍是线上话题'
```

- [ ] **Step 2: 跑测试确认失败**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select motor_command robot_dynamics web_host bench_manager
source install/setup.bash
colcon test --packages-select motor_command robot_dynamics web_host
colcon test-result --verbose
```

预期：`test_command_streamer` 找不到 `COMMAND_OUTPUT_QOS`（ImportError）；
`test_gravity_feedforward_overlay` 默认话题仍旧、命令流收不到；`test_web_assets`
的 `test_downlink_topics_are_host_facade` 断言失败。

- [ ] **Step 3: 改 `command_streamer.py`**

顶部注释与 QoS 常量改成：

```python
# 出·流：落点是主机侧 /motor/command_array（micro_ros_bridge 的入口）。
# 这是**主机内**通路，用 reliable depth 1：bridge 的订阅端就是 reliable，
# 且叠加路线的命令输入也是 reliable——发 best-effort 在 ROS 2 里不建连接。
COMMAND_OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

把 `STREAM_QOS` 的全部引用改成 `COMMAND_OUTPUT_QOS`；默认参数：

```python
        self.output_topic = self.declare_parameter(
            "output_topic", "/motor/command_array").value
```

- [ ] **Step 4: 改 `gravity_feedforward_manager.py`**

```python
DIRECT_COMMAND_TOPIC = "/motor/command_array"
```

模块 docstring 改成：「叠加节点与 bridge 的入口是主机侧 `/motor/*`；线上 `/micro_ros_*`
只由 `micro_ros_bridge` 读写。」

- [ ] **Step 5: 改叠加节点的默认话题与输出 QoS**

`gravity_feedforward_overlay_node.hpp:34-38`：

```cpp
  std::string command_input_topic{"robot_dynamics/command_array"};
  std::string trajectory_input_topic{"robot_dynamics/trajectory"};
  std::string command_output_topic{"/motor/command_array"};
  std::string trajectory_output_topic{"/motor/trajectory"};
```

`gravity_feedforward_overlay_node.cpp:115-123` 的 `declare_parameter` 默认值同步；
命令发布者改成：

```cpp
  // 输出落到 bridge 的主机入口：那条订阅端是 RELIABLE，发 BEST_EFFORT
  // 在 ROS 2 里不建连接（静默失效）。轨迹仍是 reliable。
  command_publisher_ =
      create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
          parameters.command_output_topic,
          rclcpp::QoS(1).reliable().durability_volatile());
```

文件头与 `gravity_feedforward_overlay_main.cpp`、`launch/gravity_feedforward.launch.py`
里「转发到固件订阅的 `/micro_ros_do/*`」改成「转发到 `micro_ros_bridge` 的
`/motor/*` 主机入口」。

- [ ] **Step 6: 改页面话题**

`format.js`：

```js
  trajectory: {name: '/motor/trajectory',
    type: 'can_bridge_msgs/msg/MotorTrajectory'},
  policy: {name: '/motor/policy_array', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
  paramSet: {name: '/motor/param_set', type: 'can_bridge_msgs/msg/ParamSetArray'},
  sysCmd: {name: '/motor/system_command', type: 'can_bridge_msgs/msg/SystemCommand'},
```

`agent.js:35`：

```js
export const DIRECT_COMMAND_TOPIC = '/motor/command_array';
```

只改下行。`topology` / `motion` / `status` / `events` / `imu` / `temp` /
`trajectoryState` / `devParams` / `sysStatus` 保持旧名（S4 再切）。

- [ ] **Step 7: 改台架脚本与 ros2_control 适配桥**

`micro_ros_agent/tools/traj_play.py`：

```python
        self.pub_pol = self.create_publisher(MotorPolicyArray, "/motor/policy_array", 10)
...
    ap.add_argument("--trajectory-topic", default="/motor/trajectory",
                    help="轨迹下发话题（bridge 的主机入口）。改指 "
                         "/robot_dynamics/trajectory 可经重力前馈叠加节点补上逐点 τ_ff")
    ap.add_argument("--command-topic", default="/motor/command_array",
                    help="起点对齐阶段的快照话题（同上）")
```

`micro_ros_agent/tools/snapshot_stream.py`：

```python
        self.pub_pol = self.create_publisher(
            MotorPolicyArray, "/motor/policy_array", qos_tx)
...
    ap.add_argument("--command-topic", default="/motor/command_array",
                    help="快照下发话题（bridge 的主机入口）。改指 "
                         "/robot_dynamics/command_array 可经重力前馈叠加节点中转")
```

`firmware_topic_bridge.py`：参数默认值 `'micro_ros_do/motor_trajectory'` 改成
`'/motor/trajectory'`；文件头说明同步。

- [ ] **Step 8: 重新构建并跑测试**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select motor_command robot_dynamics web_host bench_manager motor_hardware_interface
source install/setup.bash
colcon test --packages-select motor_command robot_dynamics web_host bench_manager motor_hardware_interface
colcon test-result --verbose
```

预期：0 failures。

- [ ] **Step 9: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/motor_command ros2_ws/src/bench_manager ros2_ws/src/robot_dynamics \
        ros2_ws/src/web_host ros2_ws/src/motor_hardware_interface micro_ros_agent/tools
git commit -m "feat(host): 命令源统一落到 bridge 的 /motor/* 入口

页面、命令节拍器、重力前馈叠加、台架脚本与 ros2_control 适配桥不再直发线上
话题。命令输出 QoS 改 reliable：bridge 与叠加节点的订阅端都是 reliable，
best-effort 发布在 ROS 2 里不建连接。上行话题保持不变。"
```

---

## Task 4: `micro_ros_bridge` 接入台架入口

**Files:**
- Modify: `ros2_ws/src/robot_bringup/launch/robot.launch.py`
- Modify: `ros2_ws/src/robot_bringup/package.xml`
- Create: `ros2_ws/src/robot_bringup/test/test_robot_launch.py`
- Modify: `ros2_ws/src/micro_ros_bridge/README.md`

**Interfaces:**
- Consumes: 可执行产物 `micro_ros_bridge_node`（安装到 `lib/micro_ros_bridge/`）、节点名
  `micro_ros_bridge`。
- Produces: 起 `robot.launch.py` 即有下行编码者。

- [ ] **Step 1: 写失败的 launch 静态测试**

创建 `ros2_ws/src/robot_bringup/test/test_robot_launch.py`：

```python
"""静态检查台架入口：bridge 必须随 L1 起，否则下行没有编码者。"""

import importlib.util
from pathlib import Path

from launch_ros.actions import Node


def _load_launch_module():
    path = Path(__file__).parent.parent / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bridge_node_is_started():
    actions = _load_launch_module().generate_launch_description().entities
    found = [
        action for action in actions
        if isinstance(action, Node)
        and action.node_package == 'micro_ros_bridge'
        and action.node_executable == 'micro_ros_bridge_node'
    ]
    assert found, 'robot.launch.py 没有起 micro_ros_bridge：下行没有编码者'
```

`package.xml` 的依赖区加：

```xml
  <exec_depend>micro_ros_bridge</exec_depend>
  <test_depend>python3-pytest</test_depend>
```

- [ ] **Step 2: 跑测试确认失败**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select robot_bringup
source install/setup.bash
colcon test --packages-select robot_bringup
colcon test-result --verbose
```

预期：`assert found` 失败。

- [ ] **Step 3: 在 launch 里加节点**

`robot.launch.py` 顶部：

```python
from launch_ros.actions import LifecycleNode, Node
```

在 `bench_manager` 定义之前加：

```python
    # 下行唯一的编码者：主机 /motor/* ↔ 线上 /micro_ros_*。纯 ROS 节点，
    # 不碰设备，所以随 L1 常驻；没有固件时话题空转，不报错。
    micro_ros_bridge = Node(
        package='micro_ros_bridge',
        executable='micro_ros_bridge_node',
        name='micro_ros_bridge',
        output='screen',
    )
```

在 `LaunchDescription([...])` 里、`bench_manager` 之前加 `micro_ros_bridge,`。

- [ ] **Step 4: 跑测试确认通过**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install --packages-select robot_bringup
source install/setup.bash
colcon test --packages-select robot_bringup
colcon test-result --verbose
```

预期：0 failures。

- [ ] **Step 5: 更新 bridge README 的接线状态**

`ros2_ws/src/micro_ros_bridge/README.md` 第一段：

```markdown
本包的 `micro_ros_bridge_node` 是主机 facade 消息与 micro-ROS 线上契约之间的边界节点。
线上话题名由 [`WireContract.msg`](../can_bridge_msgs/msg/WireContract.msg) 定义；固件侧
接线与发布节奏见 [`micro_ros_firmware/README.md`](../../../micro_ros_firmware/README.md)。
节点由 [`robot_bringup/robot.launch.py`](../robot_bringup/launch/robot.launch.py) 随 L1 启动。
```

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add ros2_ws/src/robot_bringup ros2_ws/src/micro_ros_bridge/README.md
git commit -m "feat(robot_bringup): 台架入口常驻 micro_ros_bridge

下行编码者随 L1 起，与 command_streamer 同层。补一条静态 launch 测试钉住
节点与可执行名，避免改名后「页面有话题、命令没人编码」。"
```

---

## Task 5: 文档同步与台架验收

**Files:**
- Modify: `micro_ros_firmware/README.md`（下行话题表唯一宿主）
- Modify: `ros2_ws/src/micro_ros_bridge/README.md`（已在 Task 4 改接线状态）
- Modify: `ros2_ws/src/motor_command/README.md`
- Modify: `ros2_ws/src/web_host/README.md`
- Modify: `ros2_ws/src/robot_dynamics/README.md`
- Modify: `ros2_ws/src/bench_msgs/README.md`（若含落点名字）
- Modify: `docs/guides/fdcan1-bench-guide.md`
- Modify: `ros2_ws/README.md`（补 `micro_ros_bridge` 包行）

**Interfaces:**
- Produces: 文档与代码同一套下行名字；台架验收清单。

- [ ] **Step 1: 全仓库查残留（源码门禁）**

```bash
cd /home/ly/robot_ws
grep -rn --include='*.py' --include='*.js' --include='*.cpp' --include='*.hpp' \
  --include='*.c' --include='*.h' --include='*.yaml' --include='*.xml' \
  -e 'micro_ros_do/motor_command_array' -e 'micro_ros_do/motor_policy' \
  -e 'micro_ros_do/motor_trajectory' -e 'micro_ros_do/param_set' \
  -e 'micro_ros_do/system_command' \
  ros2_ws/src micro_ros_agent micro_ros_firmware/src micro_ros_firmware/include
```

预期：**无输出**（文档在下一步单独改；二进制与 `.pio` 不在搜索范围）。

- [ ] **Step 2: 改固件 README 的下行表**

`micro_ros_firmware/README.md` 的「订阅（下行，5 个话题）」整节替换为：

```markdown
订阅（下行，4 个话题；线上契约见 `WireContract.msg`）：

- `/micro_ros_do/low_cmd`（`LowCommand`，best-effort）：100 Hz 完整命令快照，
  位置/速度/力矩是 int16 定点（量程见 `WireContract`）；未列出的电机 = 软停。
- `/micro_ros_do/config`（`ConfigRequest`，reliable）：参数写与系统命令合一，
  `kind` 区分 `PARAM_SET` / `SYSTEM_COMMAND`，两组字段互斥。
- `/micro_ros_do/policy`（`MotorPolicyArray`，reliable）：逐电机 enable/kp/kd。
- `/micro_ros_do/trajectory`（`MotorTrajectory`，reliable）：多点轨迹。

上行仍是旧话题（名称与节拍见下一节）；S4 切流后本节与上行表一起重写。
```

其余提到旧下行话题名的行（如「重新武装」「enable/kp/kd 不在这里」「参数面三条话题」
表格）逐个改成新名字；保留历史叙述里的旧名并标注「2026-10-20 前」。

- [ ] **Step 3: 改各主机 README**

- `motor_command/README.md`：出·流话题 `/motor/command_array`、QoS reliable。
- `web_host/README.md`：下行四条的 `/motor/*` 名字；强调上行未变。
- `robot_dynamics/README.md`：overlay 的输入/输出话题表。
- `bench_msgs/README.md`：`StopGravityFeedforward` 注释里的直连落点。
- `ros2_ws/README.md`：包表补一行：

```markdown
| `micro_ros_bridge` | 主机 `/motor/*` 与线上 `/micro_ros_*` 之间唯一的编解码节点；随 `robot_bringup` 的 L1 常驻 |
```

- `docs/guides/fdcan1-bench-guide.md`：`/micro_ros_do/param_set` 与
  `/micro_ros_do/system_command` 的示例改成 `/motor/param_set`、
  `/motor/system_command`；上行示例保持旧名（S4 前）。

- [ ] **Step 4: 构建、跑全套自动测试**

```bash
source /opt/ros/humble/setup.bash && cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose

cd /home/ly/robot_ws/micro_ros_firmware
cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
```

预期：colcon 0 failures；CTest `100% tests passed`；`pio run` `[SUCCESS]`。

- [ ] **Step 5: 台架验收（需真机，不进 CI）**

前置：CANable 已配到台架位时序（`micro_ros_agent/tools/can_up.sh`）；固件已烧录本分支。

```bash
cd /home/ly/robot_ws
source /opt/ros/humble/setup.bash
source micro_ros_agent/install/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_bringup robot.launch.py
# 页面「启动」里起 Agent 与电机后端
```

逐项核对并记录在 PR 描述里：

1. `ros2 topic info -v /micro_ros_do/low_cmd`：发布者是 `micro_ros_bridge`（best-effort），
   订阅者是固件（best-effort）。
2. `ros2 topic list | grep micro_ros_do`：只有 `low_cmd` / `config` / `policy` /
   `trajectory` 四条，旧的四条下行名字（`motor_command_array` 等）**不存在**。
3. 页面「下发」→ 臂跟随、失焦 30 ms 软停 / 100 ms 失能行为与改前一致。
4. 参数屏写入/保存：`device_param_report` 与 `system_status` 序号推进、事件回执正常。
5. 使能与增益下发（`/motor/policy_array`）：代数门表现与改前一致。
6. 轨迹下发与动作库播放：`/motor/trajectory` → 线上 `trajectory` 生效。
7. 重力前馈开关：打开后命令流改到 `/robot_dynamics/command_array`，叠加节点输出仍在
   `/motor/command_array`；关闭时先改道再停 unit，无空窗。
8. 上行旧话题照常有数（页面臂视图 / IMU / 诊断），`diagnostics.publish_fail` 无异常增长。

- [ ] **Step 6: 提交**

```bash
cd /home/ly/robot_ws
git add micro_ros_firmware/README.md ros2_ws/README.md ros2_ws/src/motor_command/README.md \
        ros2_ws/src/web_host/README.md ros2_ws/src/robot_dynamics/README.md \
        ros2_ws/src/bench_msgs/README.md docs/guides/fdcan1-bench-guide.md
git commit -m "docs: 下行切流后的唯一话题表与验收口径

固件 README 写新的四条下行契约；主机 README 写各自的新入口；台架指南的
命令示例改到 /motor/*。上行表暂留旧名，S4 再改。"
```

---

## 后续计划（不在此文档）

- **S4 上行切流**：固件发 `low_state` + `status_bridge`（`motor_status`），bridge 解码回
  `/motor/*`，`web_host` 上行前缀切换；`lib/publish_schedule` 与它的 host 测试。
- **S5 清场**：删固件旧 publisher / 静态缓冲，更新 `ros_qos.h` 注释、消息计数与
  `docs/architecture/module-boundaries.md`。
