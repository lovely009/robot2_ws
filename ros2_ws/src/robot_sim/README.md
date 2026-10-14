# robot_sim

机械臂 URDF 模型库与 Ignition Gazebo 仿真集成。每个模型是一个**自包含目录**
（`models/<模型名>/`：几何 + 网格 + 控制器参数 + 自己的 README）。RViz/Gazebo 基础
launch 按模型名选择资产；完整电机仿真拓扑随模型存放在
`models/<模型名>/config/simulation_topology.yaml`，声明电机到关节的映射。

仿真后端是 Ignition（`ros_gz_sim` + `gz_ros2_control`）。

## 模型

| 模型 | 说明 | 自由度 |
| --- | --- | --- |
| `half_body_robot` | **默认模型**（四个 launch 的默认值都是它）。半身机器人 = 上身 + 双臂整体装配体（SolidWorks 导出，躯干 `base_link` 上挂**两条互不相连**的 9 轴链，link `Joint_link1..9` / `Joint_link1.1..9.9`，joint 小写，第二链非严格镜像）。两条链的 18 个关节都有电机、都接入端到端仿真；命令接口**只有 effort**（不支持 `control_mode:=position`，原因见模型 README），并自带 `mujoco/half_body_robot.xml`（MuJoCo 后端可用） | 18 revolute + 0 fixed |

> `models/` 下另有**两个原始资产目录**（`g1_description`、`pro_urdf_publish`），
> 是导入的厂商/CAD 导出件，不满足 `models/<模型名>/` 的自包含契约四件套
> （`urdf/<名>.urdf` + `urdf/<名>.urdf.xacro` + `config/controllers.yaml` + `README.md`）。
> 它们**不在** `test_model_consistency.py` 的参数化范围内（该文件只收「带
> `urdf/<目录名>.urdf.xacro` 的契约模型」），所以测试既不会为它们报红、也不会替它们兜底——
> 契约缺项是静默的，改这些目录时不要指望测试拦住你。要么补齐契约、要么移出 `models/`，
> 见该测试文件头部对契约的说明。它们不是可加载的模型，`model:=` 不要指向它们。
>
> 模型自己的 **CAD 溯源件放在 `models/<模型名>/cad/`**（如 `half_body_robot/cad/`），
> 不放在 `models/` 顶层——顶层多一个目录就会多一个「看起来像模型但不是」的东西。

模型细节、CAD 出处、关节表与占位参数告警见各目录自己的 `README.md`。

> 2026-10-12 清理：`chest_upper_00`、`chest_dual_00`、`robot-001`、`zhoubuguanjie_youbi_002`
> 及其 `dist/` 发布件已移除；同日新增 `full_body`（双臂 18 关节，力矩控制）。
> 2026-10-13：`robot_all`（原始导出薄壳，也是唯一带 position 命令接口的模型）一并删除——
> **现在唯一可加载模型是 `half_body_robot`**，四个 launch 的默认模型都是它。
> 同日稍后 CAD 重新导出（**10 个关节的 `lower`/`upper` 收窄**）后 `full_body` **更名为
> `half_body_robot`**，`models/full_body/` 与 `dist/full_body_urdf/` 已删除；网格是同一份几何
> 重新导出（顶点集合一致），因此碰撞凸包与简化网格沿用即可，`--check` 已复核。
> 被移除/更名的模型仍在 git 与备份里可回滚，例如
> `git show fa57f56:ros2_ws/src/robot_sim/models/robot_all/`。

## 目录结构

```
robot_sim/
├── config/
│   └── display.rviz                        # RViz 预览配置
├── launch/
│   ├── display.launch.py                   # RViz 预览（无 Gazebo、无控制器）
│   ├── gazebo.launch.py                    # Ignition 仿真 + effort/position 控制器
│   ├── mujoco.launch.py                    # MuJoCo 仿真（自写力矩直通桥，话题契约同 Gazebo）
│   └── simulation.launch.py                # Gazebo + 电机总线 + 驱动/固件 HIL 组合入口
├── scripts/
│   ├── simulation_gate.py                  # 电机生命周期到 Gazebo effort 门控
│   ├── agent_after_sim_ready.py             # 模拟总线 ready 后启动 micro-ROS Agent
│   └── mujoco_effort_bridge.py             # MuJoCo ↔ ROS 2 力矩/关节状态直通桥
├── models/
│   └── <模型名>/                            # 自包含：urdf / meshes / config / cad / README
├── dist/
│   └── <模型名>_urdf/                       # ROS 无关的对外发布件（相对路径网格），不安装
└── test/
    ├── test_model_consistency.py           # 模型目录三件套一致性
    └── test_launch_models.py               # launch 的模型选择链路
```

## 构建与使用

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_sim
source install/setup.bash

# RViz 预览（滑杆拖动关节）
ros2 launch robot_sim display.launch.py

# Ignition 仿真（空世界 + 生成模型 + 控制器）
ros2 launch robot_sim gazebo.launch.py
ros2 launch robot_sim gazebo.launch.py world_file:=/path/to/world.sdf

# FDCAN/串口仿真电机后端：默认用 effort 控制器接收 MIT 等效力矩
ros2 launch robot_sim gazebo.launch.py gravity:=false

# 若要测试 MIT 等效力矩动力学，显式切换两端为 effort
ros2 launch robot_sim gazebo.launch.py control_mode:=effort

# MuJoCo 替代后端（不依赖 ros2_control/Gazebo，力矩直通；话题契约与上面一致）
ros2 launch robot_sim mujoco.launch.py model:=half_body_robot             # 带 viewer
ros2 launch robot_sim mujoco.launch.py model:=half_body_robot headless:=true
```

## 完整电机—机械臂仿真

`simulation.launch.py` 一条命令启动 Gazebo、`motor_sim`、effort 门控和
`motor_hardware_interface`。默认用本机自动创建的 `robot_vcan0` 闭环：

```bash
ros2 launch robot_sim simulation.launch.py

# 发送目标（个数 = 该模型的电机数，half_body_robot 是 18）；数组顺序 = simulation_topology.yaml 里
# motors 的顺序（与 launch 重写后的控制器 joints 顺序一致）
ros2 topic pub --once /driver/position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states
```

默认模型 `half_body_robot`：18 台电机（`arm_joint1..9` 驱第一条链、`arm_joint1.1..9.9` 驱第二条链，
id 1–18；3/4/12/13 是 CERB R58，8/9/17/18 是 vaxor_2550，其余是 CERB R48）全部挂在同一条
`arm` 总线上。电机本体（总线、id、型号、增益）与
plant 参数取自模型目录自带的 `config/motor_driver.yaml` +
`config/simulation_topology.yaml` —— 这 18 台**不在**权威的
`motor_hardware_interface/config/motor_driver.yaml` 里（那些关节尚未并入
`upper_body.urdf.xacro`，并进去会让 `test_xacro_consistency` 变红、真机 bring-up 去驱动
不存在的关节）。`simulation.launch.py` 会在模型目录有 `config/motor_driver.yaml` 时优先取它，
否则回落权威配置。

启动时从这两处配置生成 Gazebo、`motor_sim` 与 CAN 驱动端的临时配置，并检查关节名
属于所选机械模型。Gazebo 的关节状态
进入 `motor_sim`；模拟器回出的 effort 接到 `/arm_effort_controller/commands`。
全轴就绪后门控节点才开放 effort；故障或失能时关门。

控制端和总线覆盖：

| 控制端 | vcan | SocketCAN | SLCAN |
| --- | --- | --- | --- |
| `host`：主机 ros2_control 驱动 | 支持，自动建虚拟总线 | 支持，驱动与模拟器各接一个接口 | 支持，两只适配器接同一物理总线 |
| `external`：外部 CAN 控制器 | 支持同机软件节点 | 支持单接口接入 | 支持单适配器接入 |
| `firmware`：STM32 FDCAN 固件 HIL | 不可连接物理 MCU | 支持单接口 + STM32 USB CDC | 支持单 CANable + STM32 USB CDC |

传输选择：

```bash
# 默认：纯软件 CAN-FD 闭环；启动时创建 robot_vcan0，退出时删除本次创建的接口
ros2 launch robot_sim simulation.launch.py transport:=vcan

# 物理 SocketCAN；接口和位率需要事先配置。纯软件仿真优先使用 vcan
ros2 launch robot_sim simulation.launch.py transport:=socketcan \
  driver_interface:=can0 sim_interface:=can1

# 主机 ROS 驱动闭环需要两只 SLCAN 适配器，并接在同一物理 CAN-FD 总线上
ros2 launch robot_sim simulation.launch.py transport:=slcan \
  driver_tty:=/dev/serial/by-id/DRIVER_CANABLE \
  sim_tty:=/dev/serial/by-id/SIM_CANABLE

# 单 CANable + 实体 STM32 固件 HIL；默认启动 WebHost，由页面连接 Agent
ros2 launch robot_sim simulation.launch.py transport:=slcan \
  controller_mode:=firmware \
  sim_tty:=/dev/serial/by-id/CANABLE

# 单个 SocketCAN 适配器 + STM32 固件 HIL；自动启动 Agent，不经过 WebHost
ros2 launch robot_sim simulation.launch.py transport:=socketcan \
  controller_mode:=firmware sim_interface:=can0 \
  start_web_host:=false start_micro_ros_agent:=true \
  agent_tty:=/dev/serial/by-id/STM32_CDC
```

`controller_mode:=external` 会启动 Gazebo 和电机模拟端，但不启动主机
`motor_hardware_interface`，用于接外部 CAN 控制器。SLCAN host 闭环要用两只适配器，
因为一个串口不能同时被驱动和模拟器占用。`controller_mode:=firmware` 支持 SocketCAN
raw CAN FD 或 SLCAN 两种主机总线接口，使用 `micro_ros_agent/tools/fdcan_motor_sim.py`，
保留固件发现、身份和 SDO 参数应答；
启动前需 source `micro_ros_agent` 工作区。该模式默认同时启动 WebHost
（`http://localhost:8080`）；在页面选择 STM32 CDC 端口并点「启动并连接」，WebHost 会在
模拟器就绪后启动 micro-ROS Agent。若需自动启动 Agent 并跳过页面操作，可设置
`start_web_host:=false start_micro_ros_agent:=true agent_tty:=/dev/serial/by-id/STM32_CDC`。
当前完整组合入口按该机械臂的单条 CAN 总线拓扑运行；`motor_sim` 库自身仍支持多总线。

firmware 模式还支持把**电机模拟器拆成独立进程**（web 上位机「电机后端」即此形态）：

```bash
# 只起 FDCAN 模拟器（不拉 Gazebo/控制器）：可单独用于固件发现/离线/重连验证
ros2 launch robot_sim simulation.launch.py simulator_only:=true \
  controller_mode:=firmware transport:=slcan \
  sim_tty:=/dev/serial/by-id/CANABLE \
  start_web_host:=false start_micro_ros_agent:=false \
  sim_control_socket:=/tmp/motor_sim.sock

# 仿真流程不再内嵌模拟器（由外部/页面负责，避免两个实例争同一条总线）
ros2 launch robot_sim simulation.launch.py controller_mode:=firmware \
  transport:=slcan sim_tty:=/dev/serial/by-id/CANABLE \
  start_fdcan_sim:=false
```

- `simulator_only:=true` 跳过被控对象/控制器/驱动端，只跑模拟器（要求
  `controller_mode:=firmware` + 非 vcan 传输）；参数推导（电机/关节/力矩）与完整仿真完全同源。
- `start_fdcan_sim:=false` 让 firmware 流程不内嵌模拟器；与
  `start_micro_ros_agent:=true` 冲突（Agent 排序依赖模拟器写出的 ready-file）。
- `sim_control_socket:=<path>` 开运行期控制通道（status/add/remove/link），协议见
  [`micro_ros_agent/README.md`](../../../micro_ros_agent/README.md)。

`vcan` 创建和启用需要当前用户具有 `CAP_NET_ADMIN`。若没有权限，可预先创建并启用接口：

```bash
sudo ip link add dev robot_vcan0 type vcan
sudo ip link set dev robot_vcan0 up
```

随后普通用户即可启动总仿真；launch 会复用已启用的接口，退出时不会删除或关闭它。
SocketCAN 物理总线须在启动前配置与固件一致的仲裁/数据位率并置为 UP。host 双接口闭环
要把 `driver_interface` 与 `sim_interface` 接在同一条总线上；固件 HIL 只需一个
SocketCAN 适配器与 STM32 FDCAN 接到同一物理总线，USB CDC 另接主机供 micro-ROS Agent
使用。`vcan` 只能用于主机软件闭环，不能连实体 MCU。SLCAN host 闭环也需两只适配器
接同一条总线。以上传输路径不替代真机机械限位、硬件急停或电机标定。

RViz 预览还需要 `joint_state_publisher_gui` / `rviz2`，Ignition 仿真还需要 `ros_gz_sim` +
`gz_ros2_control`。它们都写在 `package.xml` 的 `exec_depend` 里，但 **rosdep 不会自动装**；
缺包时 launch 在启动任何进程之前就抛 `package ... not found`（例如本机缺
`ros-humble-joint-state-publisher-gui`），补装即可，不是模型问题。

独立的 `gazebo.launch.py`、`display.launch.py` 与 `mujoco.launch.py` 接受 `model:=<模型名>`
选择 `models/` 下的模型，默认 `half_body_robot`；`mujoco.launch.py` 要求模型目录
带有 `mujoco/<模型名>.xml`（由模型自己的 `tools/build_mujoco_model.py` 生成）与
`urdf/<模型名>.standalone.urdf`（rsp 的 robot_description 用它，缺了会「起来了但模型是空的」）。完整
`simulation.launch.py` 同样默认 `half_body_robot`，
要求所选模型目录带有 `models/<模型名>/config/simulation_topology.yaml`，且拓扑中的模型名与
launch 参数一致；`motor_config_file` **留空**时优先用模型目录自带的
`config/motor_driver.yaml`，其次才用权威整机配置（选不带拓扑的模型会显式 `RuntimeError`，
不会静默退化）：

```bash
ros2 launch robot_sim gazebo.launch.py model:=<模型名>
```

`gazebo.launch.py` 的其余参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `world_file` | `empty.sdf` | Ignition 世界文件 |
| `spawn_z` | `0.0` | 模型生成高度（m），按模型几何实测值定（`half_body_robot` 零位最低点恰为 z = 0.000000） |
| `fixed_base` | `true` | 把基座焊到世界原点；`false` = 自由浮动基座 |
| `controllers_file` | `models/<模型>/config/controllers.yaml` | controller_manager 参数 |
| `control_mode` | `effort` | `effort` 接收 FDCAN/Gazebo 电机后端发布的 MIT 等效力矩；`position` 仅用于话题/单位对照 |

## 仿真后端（`backend:=gazebo|mujoco`）

`simulation.launch.py` 的**被控对象**可替换：两个后端对外契约相同（吃
`/arm_effort_controller/commands`、出 `/joint_states`），所以传输（vcan/SocketCAN/SLCAN）、
控制端（host/external/firmware）、被控电机模型（`motor_sim`）、effort 门控与驱动端接线完全共用。

| backend | 被控对象 | 说明 |
| --- | --- | --- |
| `gazebo`（默认） | `gz sim` + `gz_ros2_control` 的 effort 控制器 | 现状；`gravity:=false` 只有它有对应开关 |
| `mujoco` | `mujoco.launch.py`（自写力矩直通桥 + MuJoCo 物理） | 需要模型的 `mujoco/<名>.xml`；viewer 窗口开在**启动它的那台机器**上（从 web_host 面板启动时即 web_host 所在机器） |

```bash
# vcan + 主机驱动闭环，被控对象换成 MuJoCo
ros2 launch robot_sim simulation.launch.py backend:=mujoco

# 物理 SocketCAN + 外部控制器
ros2 launch robot_sim simulation.launch.py backend:=mujoco \
  transport:=socketcan sim_interface:=can1 controller_mode:=external
```

- **命令数组的顺序**：`motor_sim` 按模型拓扑的电机顺序发。Gazebo 侧由 launch 重写控制器的
  `joints` 保证同序；MuJoCo 侧把**同一份关节名**通过 `joint_names` 交给桥，桥按**名字**落到
  actuator（`scripts/mujoco_command_mapping.py`），因此不依赖「MJCF 关节顺序恰好等于拓扑顺序」
  这个巧合——顺序漂移在 Gazebo 侧踩过一次，且两边长度相同不会报错。
- **`/clock`**：两条路都发（Gazebo 走 `ros_gz_bridge`，MuJoCo 由桥按物理时间发），
  robot_state_publisher 的 `use_sim_time:=true` 因此都有意义。
- **`gravity:=false` 不支持 MuJoCo**：MJCF 的重力在生成期固定，运行期没有开关，launch 会显式
  拒绝而不是静默忽略。
- **Wayland 下的 viewer**：pyGLFW 在 Wayland 会话默认载入 wayland 变体，实测该后端窗口自旋
  （`sync()` 不返回、主线程 ~39% CPU、`/joint_states` 一帧不发）。`mujoco.launch.py` 在**进程外**
  给桥进程置 `PYGLFW_LIBRARY_VARIANT=x11` 并删掉 `WAYLAND_DISPLAY`（脚本内设置太晚：
  `import mujoco` 已经先把 glfw 载入了）。桥启动时会打印实际平台，出问题先看那行。
- **MuJoCo 的物理是简化近似**（接触形状、软限位、`frictionloss`），与 Gazebo 不逐位一致，
  见 `models/half_body_robot/README.md`。

### firmware（HIL）后端下被控对象的三个关键量（2026-10-14 实测修正）

HIL 时被控对象由 `fdcan_motor_sim` 驱动，它同时是假电机与被控对象的力矩源，因此：

- **关节必须有折算惯量（armature），否则整条臂乱动**：假电机的 MIT 律是当**外力**
  注入的（`mujoco_effort_bridge` 直接写 `data.ctrl`），显式积分条件 `dt < 2J/kd` 对
  URDF 的纯连杆惯量（`1e-5~1e-7 kg·m²`）要求 `dt < 0.4 µs`，而 MuJoCo 默认 2 ms
  ⇒ 数值发散（实测 `joint_link1 v=+9595 rad/s`、位置冲到 33 rad）。URDF 缺的是
  减速器折算的转子惯量（`J_rotor·N²`，N=30~101），而它是齿轮关节的主要惯量来源。
  `tools/build_mujoco_model.py` 现按 `armature = kp/ω²`（ω=5 Hz，kp 取模型
  `config/motor_driver.yaml` 的 `command.kp`）写每个 `<joint armature>` 并设
  `<option timestep="0.001" integrator="implicitfast"/>`；本模型为 0.0304 kg·m²。
  修后实测：目标 +0.658 / 实测 +0.705（误差 0.048 rad，0.5 s 内收敛）。
- **力矩饱和值逐台取型号库**：`simulation.launch.py` 从 `motor.yaml` 读每台的
  `peak_torque_nm`（`cerb_r58=45` / `cerb_r48=11` Nm）并逐台传给
  `fdcan_motor_sim --tau-limits`；没有该字段的型号（vaxor）退回 `--tau-limit`
  （默认 10 Nm）。**这是 HIL 下力矩上限的唯一定义点**——MJCF 的 `actuatorfrcrange`
  （照抄 URDF 的 `<limit effort>`，±10）只标注关节上的合力上限，MuJoCo **不**用它夹
  actuator 的 `ctrl`（2026-10-15 实测：给 30 N·m 就是 30 N·m）。
- **关节速度上限必须写进 MJCF，否则 MuJoCo 会自激**：URDF 每条 `<limit velocity>` 在
  MuJoCo 侧没有对应属性，靠执行器的 `user` 字段携带（即 `actuator_user`），由
  `mujoco_effort_bridge` 在**每个物理步**写 `data.ctrl` 前执行（只挡继续加速、不挡反向
  制动）。缺了它被控对象会跑到真机不可能达到的转速（2026-10-15 实测 40 rad/s），MIT 律的
  `kd·vel` 项自己就顶进 ±peak 饱和 ⇒ 阻尼退化成 bang-bang、整臂陷入饱和极限环
  （速度不衰减、力矩在两个峰值间反复打满）。**只有 MuJoCo 这条路发作**：Gazebo 侧由 Dart
  执行 URDF 的 velocity 限幅。补上后峰值速度 43 → 1.4 rad/s、饱和路数 10/18 → 0/18。
- **假电机 MIT 律的更新率**：`--effort-rate-hz`（默认 200）让发力矩与 CAN 主循环
  （`--period-ms`，20 ms）解耦。它是本回路的**死区**：跟随主循环时只有 ~50 Hz，加上被控对象
  100 Hz 的 `/joint_states` ≈ 30 ms 死区，配置里的 `(kp=30, kd=1)` 恰好越过稳定边界
  （判据 `atan(kd·ω/kp) > ω·T`，`ω≈√(kp/J)`），于是**静止目标下也有 ±0.3 rad 的极限环**。
- **被控对象没有抱闸**：力矩为 0 时机构按重力自由下落。所以任何未武装/撤权/冻结的时段它都会
  塌下去；`填入在线电机（原位）` 会把"塌下去的位姿"当成目标——重新武装前先看清当前位姿。
- **整链间歇静默 = 适配器是总线上一个采样点配错的节点（2026-10-15 寄存器定案）**：
  静默时板子 TX 与 RX **同时**冻结（`TXBRP=0x0000ffff` 十六个发送缓冲全挂起、
  `TFFL=0`/`TFQF=1` FIFO 满），而 USB CDC 侧 `snapshot_accepted` 一路照涨——只有 CAN 侧死。
  出错到静默的寄存器序列：前导 `DLEC=4`（**数据相位** Bit1Error）+ `BO=1`（bus-off），
  静默期定格在 `TEC=128`、`DLEC=3`（数据相位 ACK error）。**根因是 SLCAN CANable 以
  88.24% 的数据段采样点在总线上当第二个节点**：它把板子每个带数据相位的 FD 帧采样越界 ⇒
  发错误帧、不给 ACK ⇒ 板子 TEC 累加 ⇒ 负载下冲过 255 进 bus-off。
  **板子侧一个字节都不该动**——1 Mbps/5 Mbps、81.25% 是冻结标定，匹配真机电机的 80%。
  把那只 CANable 请出总线、改用**可在主机侧精确设位时序**的 gs_usb/SocketCAN 适配器后，
  负载下实测 `TEC=0`、`DLEC=0`、`BO=0`、`TFQF=0`（380 个采样全零），静默 0 次：
  ```bash
  ip link set can0 type can bitrate 1000000 sample-point 0.8 \
      dbitrate 5000000 dsample-point 0.8125 fd on
  ```
  `dsample-point` 取 **0.8125**（26/32）与板子逐位相同——160 MHz 时钟下 5 Mbps 取
  `dbrp=1` 共 32 tq，恰好 80% 需要 25.6 tq、整数不可达，而 81.25% 正是板子的值。
  若必须用那只 SLCAN CANable，则要改它的数据段采样点（`DataTimeSeg1/2` 14/2 → 13/3
  → 82.35%）后重烧，尚未实施。另有硬性顺序：**板子必须在 ACK 节点（模拟器）已在场时才
  复位**，否则无 ACK 会填满 TX FIFO（16 帧）后永久发不出帧。
- **冻结过的台要重新武装**：OFFLINE/FAULT/CONFIG_CONFLICT 的台按 ARD §36 不会自动复权，
  需要**新的策略代数 + 新鲜完整快照**——所以**先等 18/18 稳定几秒再武装**，武装后确认
  `运行 18`（全部 ACTIVE）再下发目标。
- **「运行 18/18」不等于每台都在被控制（2026-10-14 实测）**：抓原始 CAN 帧解码 MIT 命令后
  发现，稳态下可能只有个别台在收真命令，其余台停在武装前的中性帧（`kp=0`、目标=0）⇒ 假电机
  力矩恒 0 ⇒ 那些关节只在地球重力下甩动，看起来就像"整臂乱动"。排查手段：把
  `/arm_effort_controller/commands` 与 `kp×(目标−实测)` 对照，或把 `kp` 改小看行为是否变化
  （完全不变即为该信号）。另外 **vaxor 的 `command.kp=0.008` 是占位值，会被 MIT 12 位量化
  （[0,500] 步长 0.122）归零**，那几台腕关节只剩阻尼项、不可能被位置控制。
- **台架验证用 `micro_ros_agent/tools/snapshot_stream.py`，不要用浏览器当命令源**：
  浏览器面板的 100 Hz 连发靠 `setInterval`，页面渲染一卡就 >100 ms ⇒ 固件 `revoke_motion`
  撤权；该工具以固定节奏（`--hz`，默认 100）持续下发，并在发现电机掉出运行态时**自动
  重新武装**（新代数 + 新策略），且把保持目标改成**当前实测位姿**（不是抖前的旧目标——
  对着旧目标重新发力会把机构弹回去）。模式 `echo`（只看反馈）/`arm`/`hold`/`step`；
  注意 `arm` 单独用**不会**进运行态（>100 ms 无快照即撤权），必须持续下发。
- **`policy_generation` 必须跨进程单调递增**：内核按表项锁存上次成功武装的代数，
  `enable` 时要求严格大于它（ARD §36 防「周期重发自动复权」）；只从进程启动时间
  推导起点会在重启后变小 ⇒ 每次武装都被拒，而且内核把它**报成
  `CONFIG_RESULT reason=2`（INVALID）**，看不出真因。`snapshot_stream.py` 现已把
  代数落地到 `~/.ros/micro_ros_policy_generation`（起点 `0x80000000`、只增不减、
  被拒翻倍）。自查：订阅 `/micro_ros_up/motor_events`（`[事件] 配置结果 … reason=`），
  以及 `/micro_ros_up/motor_topology` 确认 18 台都在且 model 已绑定。

## 控制机械臂

`gazebo.launch.py` 默认自动激活 `joint_state_broadcaster` 与
`arm_effort_controller`（MIT 等效力矩）。FDCAN/Gazebo 电机后端的
`--gazebo-control-mode` 必须与这里的 `control_mode` 一致。effort 模式把 MIT 控制律送入
`arm_effort_controller`，适合验收真实电机后端到 Gazebo 的闭环；position 模式把
固件已解码的关节目标送入 position controller，适合做话题、单位、ID、顺序和
轨迹插补对照。模型未内置重力补偿时需要单独调增益/前馈。两种控制器不会同时占用同一组关节命令接口：

```bash
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 1.0, 0.2]}"

ros2 topic echo /joint_states
```

命令向量顺序 = 该模型 `config/controllers.yaml` 里 `joints` 的顺序（
`test_model_consistency.py` 保证它是 URDF 可控关节的有序子集）。`half_body_robot` 是 18 个关节
全接管，顺序为第一链 `joint_link1..9` → 第二链 `joint_link1.1..9.9`。

## 加一个新模型

1. 在 `models/` 下建 `<新模型名>/`，含 `urdf/`（`<名>.urdf`、`<名>.standalone.urdf`、
   `<名>.urdf.xacro`）、`meshes/`、`config/controllers.yaml`、`cad/`、`README.md`；
2. 网格路径写成 `package://robot_sim/models/<新模型名>/meshes/...`；
3. 薄壳 xacro 用相对文件名 include 同目录的 URDF（`<xacro:include filename="<名>.urdf"/>`），
   不要用 `$(dirname)`——它只在 `ros2 launch` 的文件上下文里可用，本包 launch 是以子进程
   方式调 `xacro` 的；
4. 跑 `colcon test --packages-select robot_sim`，测试会以新目录为参数自动覆盖它。

已知限制：

- **一次只生成一个模型**：控制器名（`joint_state_broadcaster` /
  `arm_position_controller`）是全局的，同一仿真里生成两个模型会撞名。
- `config/display.rviz` 的 Fixed Frame 写死 `base_link`，根连杆不叫 `base_link` 的模型
  需要自行覆盖 `rviz_config`。
- 模型是**仿真/可视化资产**：几何与惯性来自 CAD 未做真机标定，关节限位/力矩/速度是
  占位值，接真机前必须替换。真机链路（`motor_drive` + `motor_hardware_interface` +
  `motor_driver.yaml` 拓扑）与本包没有代码依赖。

## 单文件纯模型

每个模型的 `urdf/<名>.standalone.urdf` 是把薄壳展开、剥掉 `<ros2_control>` 与
`<gazebo>` 插件块后的纯模型，单个文件即可加载（RViz、第三方工具、拖到别处都行），
不需要 xacro 处理：

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat install/robot_sim/share/robot_sim/models/<模型名>/urdf/<模型名>.standalone.urdf)"
```

网格仍走 `package://robot_sim/models/<模型名>/meshes/`，因此加载前须 source
本工作空间；要脱离 ROS 使用，用 `dist/<模型名>_urdf/` 那份（网格是相对路径）。

## 对外发布件（`dist/`）

`dist/<模型名>_urdf/` 是不依赖 ROS 的单文件模型 + 网格，可以直接打包发给别人：

```bash
cd dist/half_body_robot_urdf && zip -r ../half_body_robot_urdf.zip .
```

zip 不入库（内容在目录里都有，随时可重打）；`dist/` 也不随包安装。
