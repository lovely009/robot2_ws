# robot_arm_gazebo

机械臂 URDF 模型库与 Ignition Gazebo 仿真集成。每个模型是一个**自包含目录**
（`models/<模型名>/`：几何 + 网格 + 控制器参数 + 自己的 README）。RViz/Gazebo 基础
launch 按模型名选择资产；完整电机仿真拓扑随模型存放在
`models/<模型名>/config/simulation_topology.yaml`，声明电机到关节的映射。

仿真后端是 Ignition（`ros_gz_sim` + `gz_ros2_control`）。

## 模型

| 模型 | 说明 | 自由度 |
| --- | --- | --- |
| `zhoubuguanjie_youbi_001` | 右臂（SolidWorks 导出，固定基座 + 4 个 revolute 关节） | 4 |

模型细节、CAD 出处、关节表与占位参数告警见各目录自己的 `README.md`。

## 目录结构

```
robot_arm_gazebo/
├── config/
│   └── display.rviz                        # RViz 预览配置
├── launch/
│   ├── display.launch.py                   # RViz 预览（无 Gazebo、无控制器）
│   ├── gazebo.launch.py                    # Ignition 仿真 + effort/position 控制器
│   └── simulation.launch.py                # Gazebo + 电机总线 + 驱动/固件 HIL 组合入口
├── scripts/
│   ├── simulation_gate.py                  # 电机生命周期到 Gazebo effort 门控
│   └── agent_after_sim_ready.py             # 模拟总线 ready 后启动 micro-ROS Agent
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
colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash

# RViz 预览（滑杆拖动关节）
ros2 launch robot_arm_gazebo display.launch.py

# Ignition 仿真（空世界 + 生成模型 + 控制器）
ros2 launch robot_arm_gazebo gazebo.launch.py
ros2 launch robot_arm_gazebo gazebo.launch.py world_file:=/path/to/world.sdf

# FDCAN/串口仿真电机后端：默认用 effort 控制器接收 MIT 等效力矩
ros2 launch robot_arm_gazebo gazebo.launch.py gravity:=false

# 若要测试 MIT 等效力矩动力学，显式切换两端为 effort
ros2 launch robot_arm_gazebo gazebo.launch.py control_mode:=effort
```

## 完整电机—机械臂仿真

`simulation.launch.py` 一条命令启动 Gazebo、`motor_sim`、effort 门控和
`motor_hardware_interface`。默认用本机自动创建的 `robot_vcan0` 闭环：

```bash
ros2 launch robot_arm_gazebo simulation.launch.py

# 发送四关节位置目标；数组顺序取自所选模型的 simulation_topology.yaml
ros2 topic pub --once /driver/position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 1.0, 0.2]}"
ros2 topic echo /joint_states
```

电机 ID、协议、总线、类型和控制增益取自
`motor_hardware_interface/config/motor_driver.yaml` 与 `motor.yaml`；
`models/<模型名>/config/simulation_topology.yaml` 只记录参与仿真的电机子集、Gazebo 关节
映射和 plant 参数。当前右臂模型接入肩部到肘部四台 CERB 电机；现有 14 电机拓扑中的
腕部和左臂尚未纳入这个 Gazebo 模型。
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
ros2 launch robot_arm_gazebo simulation.launch.py transport:=vcan

# 物理 SocketCAN；接口和位率需要事先配置。纯软件仿真优先使用 vcan
ros2 launch robot_arm_gazebo simulation.launch.py transport:=socketcan \
  driver_interface:=can0 sim_interface:=can1

# 主机 ROS 驱动闭环需要两只 SLCAN 适配器，并接在同一物理 CAN-FD 总线上
ros2 launch robot_arm_gazebo simulation.launch.py transport:=slcan \
  driver_tty:=/dev/serial/by-id/DRIVER_CANABLE \
  sim_tty:=/dev/serial/by-id/SIM_CANABLE

# 单 CANable + 实体 STM32 固件 HIL；默认启动 WebHost，由页面连接 Agent
ros2 launch robot_arm_gazebo simulation.launch.py transport:=slcan \
  controller_mode:=firmware \
  sim_tty:=/dev/serial/by-id/CANABLE

# 单个 SocketCAN 适配器 + STM32 固件 HIL；自动启动 Agent，不经过 WebHost
ros2 launch robot_arm_gazebo simulation.launch.py transport:=socketcan \
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

独立的 `gazebo.launch.py` 接受 `model:=<模型名>` 选择 `models/` 下的模型，默认
`zhoubuguanjie_youbi_001`。完整 `simulation.launch.py` 要求所选模型目录带有
`models/<模型名>/config/simulation_topology.yaml`，且拓扑中的模型名与 launch 参数一致：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py model:=<模型名>
```

`gazebo.launch.py` 的其余参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `world_file` | `empty.sdf` | Ignition 世界文件 |
| `spawn_z` | `0.0` | 模型生成高度（m），按模型几何实测值定 |
| `fixed_base` | `true` | 把基座焊到世界原点；`false` = 自由浮动基座 |
| `controllers_file` | `models/<模型>/config/controllers.yaml` | controller_manager 参数 |
| `control_mode` | `effort` | `effort` 接收 FDCAN/Gazebo 电机后端发布的 MIT 等效力矩；`position` 仅用于话题/单位对照 |

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

命令向量顺序 = 该模型 `config/controllers.yaml` 里 `joints` 的顺序 =
URDF 关节串联顺序（三者由 `test_model_consistency.py` 锁死）。当前模型顺序为
`joint_link1` → `joint_link4`。

## 加一个新模型

1. 在 `models/` 下建 `<新模型名>/`，含 `urdf/`（`<名>.urdf`、`<名>.standalone.urdf`、
   `<名>.urdf.xacro`）、`meshes/`、`config/controllers.yaml`、`cad/`、`README.md`；
2. 网格路径写成 `package://robot_arm_gazebo/models/<新模型名>/meshes/...`；
3. 薄壳 xacro 用相对文件名 include 同目录的 URDF（`<xacro:include filename="<名>.urdf"/>`），
   不要用 `$(dirname)`——它只在 `ros2 launch` 的文件上下文里可用，本包 launch 是以子进程
   方式调 `xacro` 的；
4. 跑 `colcon test --packages-select robot_arm_gazebo`，测试会以新目录为参数自动覆盖它。

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
  -p robot_description:="$(cat install/robot_arm_gazebo/share/robot_arm_gazebo/models/<模型名>/urdf/<模型名>.standalone.urdf)"
```

网格仍走 `package://robot_arm_gazebo/models/<模型名>/meshes/`，因此加载前须 source
本工作空间；要脱离 ROS 使用，用 `dist/<模型名>_urdf/` 那份（网格是相对路径）。

## 对外发布件（`dist/`）

`dist/<模型名>_urdf/` 是不依赖 ROS 的单文件模型 + 网格，可以直接打包发给别人：

```bash
cd dist/zhoubuguanjie_youbi_001_urdf && zip -r ../zhoubuguanjie_youbi_001_urdf.zip .
```

zip 不入库（内容在目录里都有，随时可重打）；`dist/` 也不随包安装。
