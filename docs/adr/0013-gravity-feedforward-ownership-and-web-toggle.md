# 重力前馈的落地边界与 web 端一键开关

**Status**: accepted

## Context

ADR 0009 决定 2 写的实现是 `robot_sim` 里的 `gravity_feedforward_mixer.py`——把 `motor_sim`
的 effort 与 τ_g 求和后发到 `/arm_effort_controller/commands`，只在 `simulation.launch.py` 的
`gravity_feedforward:=true` 下接线。**该文件从未存在**，`simulation.launch.py` 里也搜不到
`gravity_feedforward` 参数；0009 索引行自己标着「（至今未落地）」。

真正落地的相反是 `robot_dynamics` 的 `gravity_feedforward_overlay`（commit `00065f7`）：夹在命令源
与固件之间，把 τ_g 加进 `can_bridge_msgs/MotorCommandArray` 与 `MotorTrajectory` 的 `effort`
字段——那是 0009 **决定 5** 明说「本轮不做、真机 τ_ff 路径另立 ADR」的那条路。也就是说：已实现
的叠加路径在仓库里没有 ADR 授权，而 0009 授权的那条从未实现。`robot_dynamics/README.md` 把它
挂在「ADR 0009 决定 1」下，但决定 1 只覆盖纯计算节点 `gravity_torque_node`。

本次要做的需求（2026-10-16）暴露了这个缺口：web 上位机要一个「重力补偿」一键开关，而不是让
操作者记 URL 参数、再另开终端起节点。要定住这个按钮的接线与所有权，就得先把叠加路径的边界
追认下来。

设计前核对过的既有事实：

1. **全仓库唯一订阅** `/micro_ros_do/motor_command_array` 与 `/micro_ros_do/motor_trajectory`
   的是 **STM32 固件**（`micro_ros_firmware/src/motor_bridge.cpp:59-60`，订阅声明 `:80-89`），
   且这两个订阅只有在 micro-ROS Agent 把 MCU 桥进 DDS 之后才出现在 ROS 图上。
2. **不存在纯软件的固件模拟器**。`micro_ros_agent/tools/fdcan_motor_sim.py` 模拟的是 CAN 总线
   上的**电机**，不是固件：它订阅 `/joint_states`（`:779`）与
   `/micro_ros_up/motor_status_array`（`:788`），发布 `/arm_effort_controller/commands`（`:792`）。
   `controller_mode:=firmware` 必须有真 STM32 与物理 CAN：`simulation.launch.py:678-683` 直接
   拒掉 vcan（`"vcan cannot connect to an MCU"`），`robot_sim/README.md:124` 的模式表同样写明。
3. 因此 `/micro_ros_do/*` 在纯 Gazebo、纯 MuJoCo、`controller_mode:=host` 三路里是**死话题**
   （只发不收）。`host` 路即使想补也无处可补：`CanFdSystem::send_position_commands`
   （`motor_hardware_interface/src/canfd_system.cpp:287-300`）只写 position/kp/kd，
   `JointCommand::feedforward` 恒为默认 0——ADR 0009 点 2 至今成立。
4. ADR 0009 点 3 记的 τ_ff 编解码量程不一致（5.0 Nm 往返成 1.222 Nm）**已修**：`motor_sim`
   按型号读 `mit_torque_limit_nm`（`motor_sim/src/protocol/cerb_protocol.cpp:91`），launch 从拓扑
   注入同一量程（厂家 2026-10-21 修正为 300 N·m，R58/R48 同值，与机械峰值 `peak_torque_nm`
   45/11 分开），两侧同源。所以前馈不再被静默打折——
   这一点是 0009 决定 5 当初把真机路径压后的理由之一，现已消失。
5. `gravity_feedforward_overlay` 有运行期可改参数 `effort_gain`，但 web 前端**没有**任何改
   ROS 参数的通道（只用 rosbridge 的 publish / subscribe / call_service），且
   `web_host/README.md:59` 明确不依赖 `rosapi` 与参数服务器。
6. 前端的下行话题名是模块加载期常量（`web_host/web/js/core/format.js:7-21`），只能靠
   `?command_topic=…` 在开页前指定；发布者在连接建立时统一 advertise（`core/ros.js:74`）。

## Decision

1. **追认叠加路径**：τ_g 的注入点是**发往固件的 CAN 命令帧**（叠加节点的输出话题
   `/micro_ros_do/motor_command_array`、`/micro_ros_do/motor_trajectory`），不是
   `/arm_effort_controller/commands`。ADR 0009 的**决定 2、决定 3、决定 5 由本 ADR 取代**——
   决定 3 的核心断言「叠加点是 Gazebo 关节力矩，不是 CAN 帧」与已实现并在此追认的相反；
   决定 2 的 mixer 从未落地；决定 5「真机 τ_ff 路径本轮不做」已被 commit `00065f7` 越过
   （且当时列为前置条件的量程不一致已修，见 Context 第 4 条）。0009 的决定 1（τ_g 计算归
   `robot_dynamics`）与决定 4 的一般原则（`motor_drive` 不依赖 ROS、Pinocchio 不进
   ros2_control 实时循环、真机适配层不依赖 Pinocchio）**继续有效**。
2. **可用性判据 = STM32 在环**（Agent 在线）。纯 Gazebo / 纯 MuJoCo / `controller_mode:=host`
   三路下令不经 micro-ROS，叠加节点插不进去：界面置灰并写明原因，不提供静默降级。
3. **所有权归 `bench_manager`**：新增第四个 unit `bench-gravity-feedforward.service`，浮点
   与 `bench.slice` 下其余 unit 同构。**按需起停，不随会话自动起**（HIL 会话常常是操作者
   在终端手起的，随会话起的方案在那条路上不成立）。`stop_slice()` 用
   `systemctl --user stop bench.slice` 覆盖它，只需把它加进 `_reset_failed` 的清理元组
   （`bench_manager/bench_manager/systemd_units.py:236-243`），否则失败态残留会被下一次
   `on_activate` 认领成"上一轮的会话"。
4. **服务契约**（`bench_msgs/srv/`，响应沿用既有的 `ok` / `error` / `status_json` 三件套）：

   | 服务 | 请求 | 语义 |
   |---|---|---|
   | `StartGravityFeedforward` | `model`、`joint_map`、`effort_gain` | 已在跑且 `model` 不同 → **返回错误，不静默替换** |
   | `StopGravityFeedforward` | — | |
   | `SetGravityFeedforwardGain` | `effort_gain` | 运行期改，不重起 unit |

   `/bench/status` 的 JSON 增加一个 `gravity_feedforward` 键，形状与其余三个会话同构：
   `{state: stopped|starting|running|failed, config: {model, urdf_path, joint_map, effort_gain} | null,
   returncode, gain_error}`。`config` 从 unit 环境读回（所以 `bench_manager` 重启后仍说得清
   在跑什么），未起时为 `null`；`gain_error` 非空表示最近一次增益写入失败。
5. **分工，两边都不复制对方的表**：`joint_map` 由前端算好传来（权威是
   `web_host/web_host/models.py::simulation_model_joints()`，前端已通过 `/api/model-joints` 拿到）；
   `urdf_path` 由 `bench_manager` 用 ament share 目录按 `model` 解析为
   `<robot_sim share>/models/<model>/urdf/<model>.urdf`。选 `.urdf` 而非 `.standalone.urdf` 的理由：
   前者正是模型枚举的判据（`models.py:26-40`），这样"下拉里选得到"与"节点起得来"是同一个谓词；
   两者实测动力学等价（同 18 关节、同 origin/axis/limit、逐 link 同质量与惯量张量）。
6. **增益不新增 `robot_dynamics` 接口**：`bench_manager` 直接写运行中叠加节点的 `effort_gain`
   参数——该参数本就是设计上的运行期可改项。实现口径：Humble 的 rclpy **没有** parameters
   client 封装（`rclpy.parameter_client` 是 Iron+ 才有的），因此手工调节点的
   `/gravity_feedforward_overlay/set_parameters_atomically`。两条必须守住的约束——调用前
   `wait_for_service` 探一下（无对端时请求会被中间件丢掉、future 永不完成，界面就会显示
   "已改"而节点其实没收到），且**只有回执确认成功才更新状态里的增益**，失败落进
   `gain_error` 而非静默。
7. **接线顺序与开关状态一致，且前端运行期可切**：开 = 先起 unit、确认 `running`，再把
   `command` / `trajectory` 两个发布目标切到 `/robot_dynamics/*`；关 = 先切回
   `/micro_ros_do/*` 直连，再停 unit。反过来会出现"话题已切、节点没起"的空窗。
8. **断链不自动回退**：unit 变成 `failed` / `stopped` 而开关仍开着时，界面给大红告警与
   **一键回退**按钮（切回直连），并写事件日志；**不静默改接线**。代价必须写清：告警无人
   值守时命令会断，最终兜底是固件自己的命令超时软停。这里与 `robot_dynamics` 的
   「丢一帧比少加一点前馈危险得多」不变式不冲突——那条约束的是叠加节点不得丢帧，回退是
   人按下的一键动作，不是自动行为。
9. **按钮只覆盖 web 自己的下发通道**。`micro_ros_agent/tools/snapshot_stream.py`、
   `micro_ros_agent/tools/traj_play.py`、`motor_command/motor_command/command_streamer.py` 是
   独立命令源，按钮管不到；它们与前馈同时跑就是混流。前端不依赖 `rosapi`，无法检测，只能
   写进面板文案与 README。
10. **过程性产物不进仓库**：设计与实施计划留在会话里，结论落本 ADR 与各 `README.md`
    （沿用 `a63897a` 的收敛约定）。

## Considered Options

- **让 overlay 常驻、跟着 simulation / 电机后端 unit 一起起**：少一个 unit、少两个服务，但
  操作者手起 HIL 会话时它不在，"确保它在跑"落空，否决。
- **前端只切话题，节点由操作者在终端起**（把现有 URL 参数做法搬进按钮）：零后端改动，但
  按钮会亮着而实际不生效——正是本次要消除的失效模式，否决。
- **给 `CanFdSystem` 增加 effort 命令接口**，让 `host` 路也能补：触及 ADR 0008 的「Pinocchio
  不进入 ros2_control 实时循环」与 0009 决定 5(a)，是另一个量级的改动；且 `host` 路今天连
  命令入口都不在前端手上。列为后续项，本轮不做。
- **实现 0009 决定 2 的仿真侧 mixer**：与已落地的 CAN 路径重复，且需重开 0009 的边界问题，
  不做。
- **断链自动回退直连**：命令不断、更安全，但接线被自动改动后事后只能翻日志才知道哪一段没补
  重力；选择人工回退。若将来台架实践表明值守不可靠，另立 ADR 改这条。
- **把开关塞进「使能/增益」面板**：那是逐台电机策略（`/micro_ros_do/motor_policy`），这是会话级
  接线开关，语义不同，会破坏该面板的单一定义，否决。

## Consequences

- `bench_msgs` 服务从 10 个增至 13 个；`GetUnitLog.srv` 里「三个固定 unit 名之一」的注释要改成
  四个。
- `bench_manager` 增加一个 manager 模块、三个服务回调与一个状态键；`_UNITS` 白名单与
  `_sessions()` 同步扩充。
- **前端运行期切换发布话题是本次唯一有运行期风险的前端改动**，而 ADR 0010 定的边界是前端只做
  静态契约测试——切换逻辑本身无法自动化，只能手动验证；静态契约测试覆盖新面板、新 DOM id、
  新 JS 模块与 `SVC` 表里的三个新服务名，外加跨包对账（SVC 表 ↔ `node.py` 的服务注册、
  四个 unit 名 ↔ `systemd_units.py`、`/robot_dynamics/*` ↔ 叠加节点的参数默认值）与两处
  源码形状断言（切接线必须同时换发布者；起停顺序）。
- **删掉 `?command_topic=` / `?trajectory_topic=` 两个 URL 覆盖参数**：同一件事有两个宿主正是
  漂移的来源，而那条路还漏掉「起节点」这一步——正是本 ADR 否决的失效模式（按钮亮着但不生效）。
  非常规接线改为直接调用 `/bench/*` 服务或手工起 launch。
- 前端新增订阅 `/joint_states`（只取**新鲜度**，不用它的数据）：叠加节点在它超时/缺席时把帧
  原样转发、不留任何痕迹，所以「开关亮着但没有补偿」这一路只能由页面自己看得见。
  在 Gazebo 台架上这条流可能被 gz 自己的 broadcaster 抢同名话题——它只影响这个提示的
  可信度，不影响臂视图（臂视图读 `motion_states`，见 `web_host/README.md`）。
- 换模型必须由操作者显式停再起（服务拒绝静默替换），面向换模型的静默失效从此不可达。
- **与 ADR 0012 第二步并发**：两者共用 `systemd_units.py` 的 unit 常量表与 `node.py` 的服务注册；
  0012 第二步还要改 `MotorBackendCommand` 的语义（socket → ROS 服务）并删除
  `fdcan_motor_sim.py`。本 ADR 只新增、不修改既有 unit 与服务，冲突面限于同文件不同区域。
- 纯 Gazebo、纯 MuJoCo、`controller_mode:=host` 三路仍然没有重力补偿；要覆盖它们需另立 ADR
  （见 Considered Options 第三条）。
- 真机标定、硬件急停与机械限位仍是最终安全边界：重力前馈是软件补偿，**不能**替代它们。
