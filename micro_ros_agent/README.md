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
| `can_bridge_msgs` | 本地自定义（21 个 ID 化有界消息 = 13 个顶层 + 8 个 Target/Entry 子消息） | 固件 `extra_packages/` 以符号链接复用 |

> 说明：官方仓库已由 `micro_ros_agent` 更名为 `micro-ROS-Agent`；官方 apt 仓库
> **没有** `ros-humble-micro-ros-agent`，故采用源码构建。第三方 `.git` 已移除，
> 以普通目录快照提交（不含构建产物）。`can_bridge_msgs` 为本地新增包，定义
> ID 化有界电机消息（命令快照、运动/诊断状态、拓扑、事件与配置事务），
> 供主机侧与上层电机栈使用。

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
  无需额外桥接；上层 joint↔motor_id 映射见固件 `README.md` 的话题契约与
  `docs/architecture/module-boundaries.md`。

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

## FDCAN 电机节点模拟器

`tools/fdcan_motor_sim.py` 在 CAN 总线上模拟 CERB/VAXOR 电机节点，处理固件发现用的
身份 SDO、周期反馈和 MIT 控制帧；可选连接 Gazebo effort 控制器。总线端支持 CANable
SLCAN 串口和 Linux CAN-FD SocketCAN：

```bash
# CANable SLCAN
python3 tools/fdcan_motor_sim.py --tty /dev/serial/by-id/CANABLE

# CAN-FD SocketCAN；can0 的位率须预先设成与 STM32 固件一致
python3 tools/fdcan_motor_sim.py --interface can0
```

运行期控制（可选）：`--control-socket <path>` 开一条 Unix socket（0600）做 JSON 行
协议，一连接一命令：

| 命令 | 语义 |
|---|---|
| `{"cmd":"status"}` | 列出当前模拟的电机（id/协议/型号/关节/链接/使能） |
| `{"cmd":"add","id","kind","model"?}` | 加一台；`model` 只对 CERB 有意义（`r48`/`r58`）。id 在 `--joint-map` 里则自动挂对应 Gazebo 关节，否则是只走线协议的裸假节点（反馈取内部假 plant） |
| `{"cmd":"remove","id"}` | 从模拟集合移除；再加回来是全新电机（SDO/位置回初值） |
| `{"cmd":"link","id","up":false\|true}` | 断开/恢复。断开 = **完全静默**（停发反馈、不回 SDO、忽略下行帧），对固件等价于设备掉电；恢复时清命令状态，固件走 RECONNECTED 重识别 |

`ros2_ws` 的 web 上位机把它当「电机后端」独立进程管理（可脱离仿真单独起停；
HIL 启动仿真时默认拉起、停止时联动收起）：经 `simulation.launch.py` 的
`simulator_only:=true` 启动并传 `sim_control_socket:=<path>`，页面经
`/api/motor-sim/*` 转发命令；手动启动时可自行指定路径。

反馈节拍默认**照真机**：每个 SYNC 回一帧（真机 TPDO 传输类型 = 1）。`--fb-period-ms`
给的不是 0 时切到台架降载模式（按周期出窗口 + `--stagger-ms` 错峰 + 墙钟兜底）。

**反馈里的电流与转速是状态导出，不是上电常量**（2026-10-16 修）。曾经两者写死在
`MotorModel.__init__` 里（`rpm=101`、`iq_ma=1000`），于是**不发任何指令、电机没使能，
页面上也恒有转速与力矩**——固件把那个假电流乘 Kt×速比，显示成 7.777/8.686 N·m，
与真机"未使能就是 0"相反。现在：

| 反馈量 | 来源 | 缺参数时 |
|---|---|---|
| 位置 | 被控对象真实关节角（`--gazebo`）/ 内部假 plant | — |
| 转速 | 关节速度 × 速比，编码到电机侧（CERB 为 RPM，VAXOR 为 rad/s） | 报 0 |
| 电流 | **本拍实际施加的关节力矩** ÷ (Kt×速比×效率)，电机侧 A | 报 0 |
| 温度 | 名义值 25.0 ℃（不建模热学；与 C++ `motor_sim` 的 `kCerbNominalTemperatureDeciDegC` 同值） | — |

"实际施加"只有一个产生路径：`GazeboBridge.publish_commands` 把力矩写进命令数组的
那一刻（含饱和钳位后的值）。未使能 / 无命令 / 固件撤权 / 位置模式因此都自动落到 0 A，
与真机一致。换算公式与 C++ `motor_sim` 的 `frame_router` 同式，两个后端在页面上给出
同一个数（ADR 0012 的收敛要求）。

两个系数由型号库（`motor_hardware_interface/config/motor.yaml`）逐台传入，
`simulation.launch.py` 按 `simulation_topology.yaml` 自动生成：

- `--joint-nm-per-amp`（`id:值`）＝ `torque_constant_nm_per_amp × gear_ratio ×
  gear_efficiency`。未列的电机上报 0 电流（不猜系数）；启动时会告警——**缺了它页面
  显示 0 A 而胳膊照样动**，比不模拟更误导。
- `--gear`（`id:值`）＝ `gear_ratio`。未列的 vaxor 退回内置 50，cerb 上报 0 转速。

取证工具 `tools/cerb_sdo_probe.py` 支持两种传输：`--interface can0`（gs_usb/SocketCAN，
现场当前配置）与 `--dev <串口>`（SLCAN）。**默认只读**，带 `--write` 才改设备对象。
设备侧对象字典、帧格式与故障码见
[`docs/research/cerb-device-reference.md`](../docs/research/cerb-device-reference.md)。

一键组合 Gazebo、该模拟器与 STM32 micro-ROS Agent 的入口和完整拓扑参数见
[`robot_sim` 仿真说明](../ros2_ws/src/robot_sim/README.md)。
