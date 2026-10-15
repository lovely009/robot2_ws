# bench_manager

台架主机侧进程管理节点：把**仿真 launch / micro-ROS Agent / FDCAN 电机后端**的起停
暴露为 `/bench/*` ROS 服务 + `/bench/status` 状态话题。web 前端（web_host）经
rosbridge 调用；任何 ROS 工具同样可用。旧版由 web_host 进程内的私有 HTTP API
管理，2026-10-15 迁入本包（web 从此只依赖 rosbridge 与静态 HTTP 两个通用面）。

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
| 服务 | `/bench/options`、`/bench/agent_status`、`/bench/{start,stop}_simulation`、`/bench/{start,stop}_agent`、`/bench/{start,stop}_motor_backend`、`/bench/motor_backend_command`（定义见 bench_msgs） |
| 状态话题 | `/bench/status`（`std_msgs/String`，JSON：simulation / agent / motor_backend 三段），1 Hz + 每个命令后立即发；无订阅者时不构造 |
| 进程所有权 | 三个 manager 实例全在本节点内存里；节点退出（launch 的 SIGINT 会走到 finally）时停掉全部子进程。仿真/电机后端是 `ros2 launch robot_sim simulation.launch.py …` 子进程，Agent 是 `ros2 run micro_ros_agent` |
| HIL 联动 | `StartSimulation` 在 firmware + 物理总线时先起电机后端（auto 标记），仿真启动失败回滚；`StopSimulation` 只联动停 auto 起的后端，手动起的保持 |
| 控制 socket | 本节点起的模拟器在 `/tmp/bench_motor_backend_<pid>.sock`；launch 内嵌模拟器固定在 `/tmp/robot_sim_motor_backend.sock`（status 报 `external`：命令可转发，起停归那次 launch） |
| 参数 | `model_dir`：模型包根（含 models/），枚举根 = `<model_dir>/models`；空 = 回落 robot_sim 安装树 |

依赖边界：本包运行期经 `ros2 launch` 引用 robot_sim，但**不声明** `exec_depend`——
robot_sim 反向依赖 web_host、web_host 又依赖本包，声明会成环（旧 web_host 同为
运行期引用而未声明）。

测试（不真起进程）：`colcon test --packages-select bench_manager`。三个 manager 的
单测在 `test/`，全部注入假 popen；不碰串口 / CAN / Gazebo / MuJoCo。
