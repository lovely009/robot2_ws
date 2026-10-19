# motor_command

主机侧命令控制器：持有台架的**设定值**，按固定节拍出命令快照。GUI 只下设定值，
节拍由本节点持有。

## 为什么节拍不能留在网页里

`/micro_ros_do/motor_command_array` 的唯一发布者曾是网页 JS（经 rosbridge），节拍来自
`setInterval`。浏览器会把**后台标签页**的定时器钳到 1 Hz——2026-10-16 实测同一页面：
前台 88 Hz，后台整整 25 秒钉在 1.0 Hz（间隔中位 1000.5 ms）。而固件的快照新鲜度线是
`MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS = 30` / `..._DISABLE_MS = 100`（照真机 CANopen
命令超时）。两者天生冲突：**页面一失去焦点，台架就整臂掉力**，现场表现为"关节绕目标
上去又下来、停不到目标位置"。节拍搬到主机侧进程后，浏览器被不被节流都不再影响台架。

## 契约

| 面 | 默认名 | 类型 | QoS |
|---|---|---|---|
| 入·设定值 | `/motor_command/setpoint` | `can_bridge_msgs/msg/MotorCommandArray` | `qos_reliable(1)`：低频声明、不允许静默丢（丢一条会拿旧目标继续跑） |
| 出·流 | `/micro_ros_do/motor_command_array` | 同上 | `qos_stream()`：best-effort + VOLATILE + depth 1，与固件对该话题的订阅**逐字一致** |
| 入·活跃判据 | `/client_count` | `std_msgs/msg/Int32` | reliable |
| 出·状态 | `/motor_command/status` | `std_msgs/msg/String`（JSON） | reliable |

QoS 的判据只有一处：固件 `include/ros_qos.h`。**同一话题两边 QoS 不一致是静默失败**
——话题存在、发布也"成功"，固件就是收不到。

**网页侧必须改发设定值话题、并且不再自己定节拍**（`web_host/web/js/core/format.js` 的
`command_topic`）。两个发布者同时发同一话题，固件会收到混合流。

## 行为

- 没收到过设定值 → 什么都不发（台架保持未使能，而不是"发一条空的"）。
- 每拍重发**同一份完整快照**：固件按快照受理，重复发同一目标是"保持"，不是新证据。
  序号门 2026-09-15 已移除、`sync_time_ns` 只作日志对齐，所以纯转发成立；本节点每帧
  自盖当前时刻与自增序号，让日志仍分得清帧。
- `count == 0`（空目标列表）**照发**：完整快照语义下它是"全部软停"这条真消息，
  显式停止不靠断流实现。
- 固件会**整条拒收**的快照（`count` 与长度不符、含 NaN/Inf）先在这里拦下并记进
  `status.reject`，不往总线上白发。拒收**不覆盖**上一条有效设定值。
- `status.reason` 是**状态**（为什么现在没在发），`status.reject` 是**事件**（粘住直到
  下一条有效设定值）。分开记是因为每拍的 `reason` 会被覆写成 `ok`——混在一起的话，
  "设定值没生效"这件事只存在 10 ms，操作者永远看不到。

## 停流的两个条件

1. **显式停止**（主要路径）：发一条空目标列表 → 固件软停（阻尼帧）→ 100 ms 后失能。
2. **兜底**：`/client_count` 归零（网页关了）→ 停流 → 固件 100 ms 内失能。

**读得到当前值**（2026-10-19 修正）：rosbridge 的 `/client_count` publisher 是 depth 1 +
`TRANSIENT_LOCAL`（`rosbridge_server/client_manager.py` 的 `ClientManager`），即 latched。
订阅侧必须同为 `TRANSIENT_LOCAL` 才收得到迟到的锁存样本——本节点原来用 VOLATILE 订阅，
于是"只在连接数变化时才知道"，浏览器已连着时启动读不到当前值。那曾被记成 rosbridge 的
**边沿触发**性质（"实测 8 秒 0 条"），实为这里 QoS 不匹配。现在两边逐字一致，启动那一刻
就生效。

`status.clients` 仍可能为 `null`：那表示**连 rosbridge 都没有**，也就没有网页这条路，
操作者只可能从命令行驱动——此时**不拦**（`None != 0`），是有意的 fail-open。

本节点自身失效（退出/被杀）= 流停 = 固件 100 ms 内失能，**失效方向是安全的**。固件那
两条新鲜度线保留、不绕过——它照真机 CANopen，是最后一道。

## 命令从哪条路出去（运行期可改）

`output_topic` 是**运行期可改参数**，改的是命令流发往哪里：

| 落点 | 话题 | 谁切 |
|---|---|---|
| 直连固件 | `/micro_ros_do/motor_command_array` | — |
| 经重力前馈叠加 | `/robot_dynamics/command_array` | `bench_manager` 的 `/bench/{start,stop}_gravity_feedforward` |

改道**不重起本进程**（重起会断流，撞上固件的命令超时软停，而机构没有抱闸），也不是
页面切的——页面只发设定值，切不了命令话题，只能读 `/motor_command/status` 的
`output_topic` 核对（ADR 0014 决定 3）。顺序是安全边界，由 `bench_manager` 保证：
开启时先起叠加节点、确认 running 再改道；停止时先改回直连、**确认成功才**停节点。

```bash
ros2 param set /command_streamer output_topic /robot_dynamics/command_array
```

换发布者在下一拍生效（参数回调只校验，不在回调里做发布者手术——回调跑在参数值落定
之前，那时换掉会让"参数已改"与"实际发往哪"短暂不一致）。

## 运行

```bash
# 常态随 L1 起（web_host.launch.py）；单起用于排查
ros2 run motor_command command_streamer
ros2 topic echo /motor_command/status        # 常态观察看这条（reliable）
# 真正下发的帧是 best-effort，看它要显式指定：
ros2 topic echo /micro_ros_do/motor_command_array --qos-reliability best_effort
```

参数：`stream_hz`(100)、`setpoint_topic`、`output_topic`（运行期可改）、
`client_count_topic`、`status_topic`、`require_client`(true)。

`/motor_command/status` 里 `output_topic` 是命令**当前实际发往**哪里，`reason` 说明
为什么现在没在发（`no_setpoint` / `no_client` / `ok`），`reject` 是最近一次被拦下的
设定值——页面据此核对开关与接线是否一致。

## 测试

`colcon test --packages-select motor_command`。发布端用替身捕获、定时器不 spin：只测
本节点的裁决逻辑（节拍 / 拒收 / 停流 / 空快照 / 状态字段 / QoS 常量），不起进程、
不碰串口与 CAN。
