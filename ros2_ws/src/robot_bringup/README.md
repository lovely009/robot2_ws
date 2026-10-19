# robot_bringup

**台架唯一入口**：一个 launch 起 L1 与 L2，L3 会话由 `bench_manager` 按需起停。

| 层 | 内容 | 由谁起 |
|---|---|---|
| L1 | rosbridge WebSocket + 静态页面 + 命令节拍（`command_streamer`）（include `web_host` 的 `web_host.launch.py`） | 本 launch |
| L2 | `bench_manager`（lifecycle 节点，`/bench/*` 服务 + `/bench/status`） | 本 launch |
| L3 | 会话 unit：仿真 / 电机后端 / micro-ROS Agent / 重力前馈叠加 | `bench_manager`，页面点「启动」时 |

L3 不在本 launch 里：那是页面上按钮的事。本包**只装一个 launch**，没有可执行、没有编译目标。

## 与 web_host.launch.py 的分工

`web_host.launch.py` 只做 L1——「只想看页面」不必带上进程管理权限；「起一套台架」才是本
入口。两者不是新旧关系，是能力范围不同：本 launch include 前者，不重复它的节点定义。

## lifecycle 推进

仓库里**没有 lifecycle_manager**，用 launch 事件把 configure → activate 串起来：

```
bench_manager 进程起来 ──OnProcessStart──► CONFIGURE
        进 inactive ──OnStateTransition──► ACTIVATE
```

于是「起 robot.launch」就等于「页面立刻能用」。`on_configure` 只做校验，其中 systemd
可达性（`systemctl --user show-environment`）是 **fail-fast**：没有 user manager 就承载不了
任何会话，节点停在 unconfigured、`/bench/*` 服务不注册——这是刻意的，日志里有原因，好过等
页面点「启动」才报错。

## 停止

| 退出方式 | 后果 |
|---|---|
| Ctrl-C / SIGTERM（干净退出） | `on_deactivate` → `stop_sessions()` → `stop_slice()`，停掉整个 `bench.slice` |
| 崩溃 / `kill -9`（异常死亡） | 走不到上面那步，unit 留给下一个 `bench_manager` 在 `on_activate` 里认领 |

本节点是页面唯一的会话出口——它没了，页面就停不掉还在跑的 Gazebo。所以干净退出必须停整个
slice，而不是把会话变成孤儿（细节见 [`bench_manager/README.md`](../bench_manager/README.md)）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_dir` | `robot_sim` 安装树的 share | 模型包根（含 `models/`）：臂视图几何与模型枚举的共同来源，**可指向源码目录**（改模型不必重构建）。取不到包时为空 = 降级（模型列表空、臂视图提示加载失败，静态页面照常） |
| `rosbridge_port` | `9090` | 透传给 `web_host.launch.py` |
| `web_port` | `8080` | 透传给 `web_host.launch.py` |

## 启动

```bash
source /opt/ros/humble/setup.bash
source "$(git rev-parse --show-toplevel)/micro_ros_agent/install/setup.bash"   # Agent 在本工作区
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"
ros2 launch robot_bringup robot.launch.py
```

浏览器打开 `http://<主机IP>:8080`。启动成功的标志与页面能力见
[`web_host/README.md`](../web_host/README.md)。

## 依赖边界

`exec_depend`：`launch`、`launch_ros`、`lifecycle_msgs`、`web_host`、`bench_manager`。

**`robot_sim` 刻意不声明**：模型枚举根与臂视图几何来自它，但缺包时本 launch 自己降级；而
`robot_sim` 的 `simulation.launch.py` 会 include `web_host`、`bench_manager` 又经 `ros2 launch`
调它，声明会成环（理由写在 `package.xml` 的注释里）。

## 测试

本包没有 `test/`（只有一个 launch，无逻辑可单测）。launch 的行为由
`bench_manager` 与 `web_host` 各自的测试覆盖；本包只保证 include 与 lifecycle 接线正确。
