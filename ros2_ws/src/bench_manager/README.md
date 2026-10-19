# bench_manager

台架主机侧进程管理节点：把**仿真 launch / micro-ROS Agent / FDCAN 电机后端 / 重力前馈
叠加节点**的起停暴露为 `/bench/*` ROS 服务 + `/bench/status` 状态话题。web 前端
（web_host）经 rosbridge 调用；任何 ROS 工具同样可用。旧版由 web_host 进程内的私有
HTTP API 管理，2026-10-15 迁入本包（web 从此只依赖 rosbridge 与静态 HTTP 两个通用面）。

```bash
# 通常随 web 栈一起起（ros2 launch web_host web_host.launch.py，默认含本节点）
ros2 launch web_host web_host.launch.py start_bench_manager:=false   # 只要静态页
# 独立起
ros2 run bench_manager bench_manager --ros-args -p model_dir:=/path/to/robot_sim
# 查看
ros2 service call /bench/options bench_msgs/srv/GetBenchOptions '{}'
ros2 topic echo /bench/status
```

| 关注点 | 事实 |
|---|---|
| 服务 | `/bench/options`、`/bench/agent_status`、`/bench/{start,stop}_simulation`、`/bench/{start,stop}_agent`、`/bench/{start,stop}_motor_backend`、`/bench/motor_backend_command`、`/bench/get_unit_log`（白名单只认下面四个 unit 名）、`/bench/{start,stop}_gravity_feedforward`、`/bench/set_gravity_feedforward_gain`（定义见 bench_msgs） |
| 状态话题 | `/bench/status`（`std_msgs/String`，JSON：simulation / agent / motor_backend / gravity_feedforward 四段），1 Hz + 每个命令后立即发；无订阅者时不构造。**日志不在快照里**（四个 unit 各 80 行会把它撑成几十 KB，而页面只在抽屉展开时看），改按需拉 `/bench/get_unit_log` |
| 进程所有权 | 会话进程由 **systemd user transient unit** 承载（`bench.slice` 下的四个 unit，名字常量只在 `systemd_units.py` 定义一次），本节点**不持有任何进程句柄**。状态权威是 unit 状态，会话描述（模型 / 总线 / 设备）存在 unit 环境里——所以本节点重启后能原样认领上一轮的会话，而不是把它们变成孤儿 |
| 会话归宿 | **干净退出**（Ctrl-C / SIGTERM）停掉整个 `bench.slice`：本节点是页面唯一的会话出口，它没了页面就停不掉还在跑的 Gazebo；**异常死亡**（崩溃 / kill -9）把 unit 留给下一个 `bench_manager` 在 `on_activate` 里认领。`stop_slice()` 一并清掉失败态残留，否则下一次 `on_activate` 会把它认领成「上一轮的会话」 |
| lifecycle | `on_configure` 只做校验，其中 systemd 可达性（`systemctl --user show-environment`）是 fail-fast——没有 user manager 就承载不了任何会话，早说好过等页面点「启动」才报错；`on_activate` 认领已在跑的 unit（只报告，不动它们）；`on_deactivate` 停整个 slice |
| HIL 联动 | `StartSimulation` 在 firmware + 物理总线时先起电机后端（auto 标记），仿真启动失败回滚；`StopSimulation` 只联动停 auto 起的后端，手动起的保持。**重力前馈不随会话自动起**：HIL 会话常常是操作者在终端手起的，随会话起的方案在那条路上不成立（见 ADR 0013） |
| 控制 socket | 本节点起的模拟器在 `/tmp/bench_motor_backend_<pid>.sock`；launch 内嵌模拟器固定在 `/tmp/robot_sim_motor_backend.sock`（status 报 `external`：命令可转发，起停归那次 launch） |
| 命令改道 | 两个前馈服务顺带切 `command_streamer` 的 `output_topic`（直连固件 ↔ 经叠加节点）。改道**不重起**那个节点；页面切不了它，只能读 `/motor_command/status` 核对 |
| 参数 | `model_dir`：模型包根（含 models/），枚举根 = `<model_dir>/models`；空 = 回落 robot_sim 安装树 |

重力前馈那一路的三条非常规细节（其余见 [ADR 0013](../../../docs/adr/0013-gravity-feedforward-ownership-and-web-toggle.md)、[0014](../../../docs/adr/0014-host-side-command-pacing.md)）：

- **增益走参数服务，不重起 unit**。重起会断掉命令流，撞上固件的命令超时软停，而机构没有
  抱闸。Humble 的 rclpy 没有 parameters client 封装，节点直接调
  `/gravity_feedforward_overlay/set_parameters_atomically`；调用前 `wait_for_service` 探一次
  （无对端时请求会被中间件丢掉、future 永不完成，界面就会显示「已改」而节点没收到），
  且**只有回执确认成功才更新状态里的 `effort_gain`**，失败落进 `gain_error`。
- **已在跑另一个模型时拒绝启动**，不静默替换：换模型拿到旧模型的 τ_g 是这个功能最危险的
  失效模式。要换模型必须先停。
- **命令流的改道归这两个服务**（ADR 0014 决定 3）：`start` 在 unit 确认 running 之后把
  `command_streamer` 的 `output_topic` 改到 `/robot_dynamics/command_array`；`stop` 反过来
  **先**改回直连、**确认成功才**停 unit。停 unit 挂在改道的成功回执上，不是顺序写两句
  ——`Client.call()` 在回调里会死锁（rclpy 明文禁止），所以顺序只能靠异步链保证。
  两个方向的失败处置不同：开启时改道失败**不回滚** unit（命令继续走直连是有命令、只是
  没补偿，安全方向），停止时改道失败则**不许**停 unit（否则命令落进没人订阅的话题，
  固件命令超时软停，而机构没有抱闸）。两者都写进状态的 `route_error`，页面看得见。

依赖边界：本包运行期经 `ros2 launch` / `ros2 run` 引用 robot_sim 与 robot_dynamics，
但**都不声明** `exec_depend`——robot_sim 反向依赖 web_host、web_host 又依赖本包，声明会
成环（旧 web_host 同为运行期引用而未声明）。

测试（不真起进程、不碰真 systemd）：`colcon test --packages-select bench_manager`。
`conftest.py` 把 `subprocess.Popen`/`run` 换成「一调用就报错」的守卫，逼测试注入假体；
`FakeSystemd` 建模的是 unit 的**状态迁移**而不是只记 argv——认领、复用、失败态带回退码
这些分支正是把状态权威从 `poll()` 换成 systemd 的全部意义所在。不碰串口 / CAN / Gazebo /
MuJoCo。
