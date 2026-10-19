# 命令节拍迁到主机侧：web 只下设定值

**Status**: accepted（2026-10-19 用户裁定：全量实现）

## Context

`/micro_ros_do/motor_command_array` 的唯一发布者曾是网页 JS（经 rosbridge），节拍来自
`setInterval`。浏览器会把**后台标签页**的定时器钳到 1 Hz——2026-10-16 实测同一页面：
前台 88 Hz，后台整整 25 秒钉在 1.0 Hz（间隔中位 1000.5 ms）。而固件的快照新鲜度线是
`MOTOR_CONTROL_SNAPSHOT_SOFT_STOP_MS = 30` / `..._DISABLE_MS = 100`（照真机 CANopen
命令超时）。两者天生冲突：**页面一失去焦点，台架就整臂掉力**，现场表现为"关节绕目标
上去又下来、停不到目标位置"。

`motor_command` 包为此已建好并写全了契约（commit `ae4ca1d`），但**没有接线**：

- `/motor_command/setpoint` 全仓库除本包自身外**零引用**；
- `web_host/web/js/core/format.js` 的 `TOPICS.command` 仍指 `/micro_ros_do/motor_command_array`，
  页面照旧自己发；
- 没有任何 launch 或 systemd unit 起 `command_streamer`。

也就是说：**风险一直原样存在**，包只是摆在那里。本 ADR 定住接线形态，并已按此落地
（`motor_command` 的运行期 `output_topic`、`web_host.launch.py` 的第三个节点、web 侧改发
设定值、`bench_manager` 两个前馈服务里的改道）。

设计前核对过的事实（这几条直接决定接线形状）：

1. **快照与轨迹可以并存**：`target_ids` 声明的电机在轨迹在播期间由轨迹**独占**，同一
   时刻到达的 100 Hz 快照对它们**既不下发也不软停**（`micro_ros_firmware/README.md`
   「归属仲裁」）。集合外的电机仍由快照负责。⇒ 常驻快照流**不会打死轨迹**。
2. **快照不是解除归属的路径**：解除只有三条——新轨迹替换、空轨迹取消、会话断链。
   轨迹播完仍持有归属，期间发快照**被静默忽略且无报错**（`micro_ros_firmware/AGENTS.md`
   「轨迹播完不释放归属」）。⇒ **取消轨迹的那一刻，常驻快照会立刻接管。**
3. **`/client_count` 的"边沿触发"是 QoS 不匹配的产物**：rosbridge 的 publisher 是
   depth 1 + **`TRANSIENT_LOCAL`**（`rosbridge_server/client_manager.py:57-59`，即 latched），
   而 `command_streamer` 用 VOLATILE 的 `SETPOINT_QOS` 订阅
   （`command_streamer.py:78-79`）——VOLATILE 订阅者**收不到迟到的锁存样本**，所以只在
   计数变化时才知道。`motor_command/README.md` 记的"读不到当前值、`status.clients` 为
   `null` 且不拦"的 fail-open 洞，改订阅 QoS 即可关掉。
4. 三条命令源（web / `snapshot_stream.py` / `traj_play.py`）里只有 web 走浏览器定时器；
   后两者是自定拍的工具（`traj_play.py:211`、`snapshot_stream.py:301` 各自带
   `--command-topic`）。

## Decision

1. **web 只下设定值**。`/motor_command/setpoint` 成为页面下行的唯一入口；`panels/command.js`
   的 100 Hz 连发删除，`sendTargets` 改发设定值。页面不再持有命令发布者。

2. **常驻归属给 `web_host.launch.py`（L1 的第三个节点），不做成台架 unit**。
   它的存在意义就是服务页面：页面在，它就该在。三条启动路径因此都覆盖——`robot_bringup`
   的 `robot.launch.py`（include L1）、`simulation.launch.py start_web_host:=true`（include
   L1）、以及手起 HIL 会话后单起 L1。依赖是单向无环的（`motor_command` 只依赖
   `rclpy` / `can_bridge_msgs` / `std_msgs`），所以 `web_host` 可以**真声明**
   `exec_depend motor_command`——这正是 ADR 0001 以来那几处"运行期引用不敢声明"的反例。

3. **接线开关（重力补偿）挪到 streamer 的输出侧，并入既有的两个服务**。
   `command_streamer` 的 `output_topic` 变成运行期可改参数：`DIRECT` =
   `/micro_ros_do/motor_command_array`，`OVERLAY` = `/robot_dynamics/command_array`。
   切换**不新增控制面**：ADR 0013 的 `/bench/{start,stop}_gravity_feedforward` 已经持有
   「先起 unit、确认 `running`，再切接线」的顺序不变式（决定 7），把 streamer 的
   `output_topic` 一并交给它们翻——服务端执行顺序，比今天"web 自己按顺序切本地发布者"
   更硬。调用参数服务前 `wait_for_service` 探一次，只有回执确认成功才更新状态（沿用
   决定 6 的模式）；**不重起进程**（重起断流会撞固件的 100 ms 失能，而机构没有抱闸）。
   ⇒ ADR 0013 的"页面切自己的发布者"变成"服务端切 streamer 的输出"；轨迹发布者仍归
   web（轨迹不过 streamer，见决定 5），页面切它自己的那一条。

4. **`/client_count` 订阅改 `TRANSIENT_LOCAL` + depth 1**，与 rosbridge publisher 逐字一致，
   关掉事实 3 那个 fail-open 洞。

5. **轨迹通道不过 streamer**（轨迹是单条大消息，浏览器节流对它无影响）；动作库的
   "对齐起点"快照改发设定值。但**"停止"按钮必须先发一条"当前实测位姿"的设定值，再发空
   轨迹取消**——顺序反了，取消的瞬间常驻快照会接管，机构弹回最后一次设定值（可能是很早
   以前的目标）。这是本 ADR 引入的**新失效模式**，实现里必须守住，并写进面板文案。

6. **失效方向不变**：streamer 死 = 流停 = 固件 100 ms 内失能，是安全方向。固件那两条
   新鲜度线保留、不绕过——它照真机 CANopen，是最后一道。

## Consequences

- 页面失焦不再影响台架（本 ADR 的全部目的）。
- 新增失效模式一条（决定 5 的取消顺序），必须在实现与测试里覆盖。
- **切重力补偿接线从此必须经 `bench_manager`**（原本页面自己就能切）。代价是"只想看页面"
  那条路上没有重力补偿开关——但那条路上本来也没有 `gravity_feedforward_overlay` 的订阅者
  （ADR 0013 已定它只在 STM32 在环时可用）。换来的是"页面不再是命令发布者"这一条不变式。
- "单发"按钮语义改变：现在是真的只发一条（30 ms 后固件软停），改后是"改设定值并持续保持"。
  这是行为改进，但要改面板文案。
- `motor_command/README.md` 关于 `/client_count` 的那一段要按事实 3 重写。
- 不保留"streamer 不在时页面直连"的降级路径：两个发布者写同一话题 = 混合流。断链时页面
  报红，不静默换路——与 ADR 0013 决定 8 的"不自动回退"同一条取向。

## Considered Options

- **做成第五个案台 unit（`bench_manager` 托管）**：与 `gravity_feedforward` 同构、页面能在
  `/bench/status` 里看到它的状态。但"只想看页面"那条路（单起 L1，不带进程管理权限）就
  下不了命令，而那正是页面存在的场景；且它不需要按需起停。否决。
- **由 `robot_bringup` 起**：只在 `robot.launch.py` 那条路有。HIL 会话常常是操作者在终端
  手起 `simulation.launch.py`（`robot_dynamics/README.md` 明说），那条路上没有它 ⇒ 页面
  下发发到没人接的 setpoint 上。否决。
- **让 overlay 变成必经一跳**（`effort_gain=0` 时逐字段直通），streamer 输出固定：
  ADR 0013 已定 overlay 只在 STM32 在环时可用（纯 Gazebo / 纯 MuJoCo / `controller_mode:=host`
  三路没有订阅者），常驻会让那三路没人订阅。否决。
- **保留 web 侧发布，另加一个"心跳"进程重发**：两个发布者写同一话题 = 固件收到混合流。否决。
- **给 `MotorCommandArray` 加路由字段**：该消息也发给固件，会污染固件契约。否决。
