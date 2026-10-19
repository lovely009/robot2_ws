# bench_msgs

台架主机侧控制服务的接口定义（由 `bench_manager` 节点实现，web 前端经 rosbridge 调用）。

| 服务 | 作用 |
|---|---|
| `GetBenchOptions` | 环境枚举：CAN 接口 / 串口 / 可用模型（有拓扑 / 有 MJCF） |
| `GetAgentStatus` | micro-ROS Agent 进程即时状态 |
| `StartSimulation` / `StopSimulation` | robot_sim 的 simulation.launch.py 起停（含 HIL 联动） |
| `StartAgent` / `StopAgent` | micro-ROS Agent 起停（枚举串口参数） |
| `StartMotorBackend` / `StopMotorBackend` | FDCAN 电机模拟器独立进程起停 |
| `StartGravityFeedforward` / `StopGravityFeedforward` | 重力前馈叠加节点（第四个台架 unit）起停；`model` / `joint_map` / `effort_gain` 随启动给，已在跑另一个模型时**拒绝**而非静默替换 |
| `SetGravityFeedforwardGain` | 运行期改 `effort_gain`，**不重起** unit（重起会断命令流，撞上固件的命令超时软停） |
| `MotorBackendCommand` | 模拟器控制通道命令转发（JSON，一连接一命令） |
| `GetUnitLog` | 按需拉某个会话 unit 的尾部日志（`journalctl`，不进 1 Hz 快照） |

约定：

- 除 `MotorBackendCommand`（响应 `ok/error/response_json`）外，其余命令类服务的响应
  统一为 `ok/error/status_json`——`status_json` 与 `/bench/status` 话题同构，失败时
  供页面回显；
- 空串字段 = "未给该参数"（与 launch 参数的缺省语义一致）；
- 只有 `StartSimulation` 会编排其它子进程（HIL 时先起电机后端、失败回滚）。
- **接线顺序是安全边界**：`StopGravityFeedforward` 的调用方要**先**把命令源话题切回
  `/micro_ros_do/*` 直连再调本服务——反过来会出现「话题已切、节点没起」的空窗（见
  [ADR 0013](../../../docs/adr/0013-gravity-feedforward-ownership-and-web-toggle.md)）。

构建：`colcon build --packages-select bench_msgs`（rosidl 生成，无运行代码）。
