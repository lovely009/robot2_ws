# micro_ros_agent

micro-ROS Agent 的独立 colcon 工作区（ROS 2 Humble / Ubuntu 22.04）。作为 ROS 2 侧的
桥接节点，与 MCU 固件（`../micro_ros_firmware/`，见其 README：STM32H723VGT6 + serial
传输 + BMI088/三路 FDCAN 电机运行时）通过 **serial（USB CDC）** 传输通信，并与
`../ros2_ws` 电机栈同域（domain 0）互通。

## 源码来源（humble 分支，快照提交）

| 包 | 仓库 | 分支 |
|---|---|---|
| `micro_ros_msgs` | https://github.com/micro-ROS/micro_ros_msgs.git | `humble` |
| `micro-ROS-Agent` | https://github.com/micro-ROS/micro-ROS-Agent.git | `humble` |
| `can_bridge_msgs` | 本地自定义（11 个 ID 化有界消息） | 固件 `extra_packages/` 以符号链接复用 |
| `motor_param_bridge` | 本地自定义（ROS 参数 → 固件下行配置话题） | 主机侧节点，不参与固件构建 |

> 说明：官方仓库已由 `micro_ros_agent` 更名为 `micro-ROS-Agent`；官方 apt 仓库
> **没有** `ros-humble-micro-ros-agent`，故采用源码构建。第三方 `.git` 已移除，
> 以普通目录快照提交（不含构建产物）。`can_bridge_msgs` 为本地新增包，定义
> ID 化有界电机消息（命令快照、运动/诊断状态、拓扑、事件、身份绑定与运行时
> 策略），供主机侧与上层电机栈使用。`motor_param_bridge` 把 ROS 参数
> `motor_<id>.model/kp/kd/pos_min/pos_max` 翻译成 `micro_ros/motor_bind` 与
> `micro_ros/motor_policy` 两条下行话题，写进固件现成的 mailbox 配置/绑定 FIFO。

## 构建（免 sudo）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

- 依赖自动排序：`micro_ros_msgs` → `can_bridge_msgs` → `micro-ROS-Agent`。
- Agent 的 superbuild 构建时会从 GitHub 拉取 Micro-XRCE-DDS-Agent（走代理），
  并使用系统 Fast DDS / Fast CDR（Humble 自带 `ros-humble-fastrtps`）。
- 若报缺测试/lint 依赖，可加 `--cmake-args -DBUILD_TESTING=OFF` 跳过，无需 sudo。

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# serial 传输（默认 115200）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# 查看全部传输与参数
ros2 run micro_ros_agent micro_ros_agent --help
```

可用传输：`udp4|udp6|tcp4|tcp6|canfd|serial|multiserial|pseudoterminal`。

## 与 ros2_ws 集成

- 两套工作区独立构建；运行时同时 source 两个 `install/setup.bash`（先 `/opt/ros`，
  再 `ros2_ws` 与 `micro_ros_agent`，顺序无强依赖）。
- 默认 DDS 域（domain 0）下，micro-ROS 客户端发布的 topic 与 `ros2_ws` 节点直接互通，
  无需额外桥接；上层 joint↔motor_id 映射的迁移进度见固件 HANDOFF
  （`../micro_ros_firmware/docs/handoffs/HANDOFF-motor-runtime-refactor.md`）。

## 冒烟验证（无固件）

```bash
# 1) 二进制与帮助
ros2 run micro_ros_agent micro_ros_agent --help
# 2) serial 传输初始化（用伪终端代替真实串口）
python3 /tmp/micro_ros_pty.py 60 &   # 打印 /dev/pts/N
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/pts/N -v
# 看到 "TermiosAgentLinux ... running... fd: N" 即初始化成功
```

> 无客户端时 Agent 节点不会出现在 `ros2 node list`（节点在客户端建立会话后才注册），
> 属正常现象；与固件的端到端联调见固件 README「与 micro_ros_agent 联调」与
> [FDCAN1 台架指南](../docs/guides/fdcan1-bench-guide.md)。
