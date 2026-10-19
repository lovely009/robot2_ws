# ros2_ws —— ROS 2 Humble 十一包工作区

真机、仿真、消息定义与 Web 上位机并存。构建与测试、真机/仿真 bring-up、配置与扩展
入口都在这里；各包细节见 `src/<pkg>/README.md`。架构取舍见 [../docs/adr/](../docs/adr/)。

| 包 | 定位 |
|---|---|
| `can_bridge_msgs` | 固件与 ROS 2 共用的 ID 化有界消息定义（唯一来源是 `../micro_ros_agent/src/can_bridge_msgs`，本目录内是符号链接） |
| `motor_drive` | 无 ROS 底层驱动库（传输 / 线协议 / 电机映射） |
| `motor_hardware_interface` | ros2_control 适配层 + 真机 bring-up 入口 |
| `motor_sim` | 与机器人无关的共享仿真库（协议 / 传输 / 传动 / plant / 帧路由 / 锁存安全） |
| `robot_sim` | 机械臂模型库（`models/<模型名>/`）+ 两个可互换被控对象后端（Gazebo / MuJoCo）与端到端组合入口 |
| `robot_dynamics` | 独立 Pinocchio 运动学/动力学计算边界：重力矩库 `ArmModel` + `gravity_torque_node`（只算）与 `gravity_feedforward_overlay`（把 τ_g 叠进命令帧） |
| `web_host` | rosbridge Web 上位机（臂视图、IMU、设备参数、使能/增益、重力补偿开关、轨迹）＋静态文件与模型域 API（模型枚举 / 关节映射 / 动作库） |
| `bench_msgs` | 台架主机侧控制服务定义（`/bench/*`：仿真 / Agent / 电机后端 / 重力前馈） |
| `bench_manager` | 台架会话的唯一编排者：`/bench/*` 服务 + `/bench/status` 话题，会话进程由 systemd user unit 承载；web 的仿真屏、顶栏 Agent 控件与重力补偿开关走它 |
| `motor_command` | 主机侧命令节拍：持有台架设定值，按固定节拍出命令快照（GUI 只下设定值——浏览器定时器会被后台标签页节流） |
| `robot_bringup` | 台架唯一入口：一个 launch 起 L1（rosbridge + 静态页）与 L2（`bench_manager`）；L3 会话（仿真 / 电机后端 / Agent / 重力前馈）由 `bench_manager` 按需起停 |

## 分层与命名边界

| 术语 | 含义 |
|---|---|
| 底层驱动 | `motor_drive`：传输、线协议、通用电机映射（无 ROS） |
| ROS 适配层 | `motor_hardware_interface`：ros2_control lifecycle、joint 映射、YAML 解析、真机 bring-up |
| 仿真层 | `motor_sim`（共享仿真库）+ `robot_sim`（模型库 `models/<模型名>`、launch、Gazebo/MuJoCo 集成） |
| 运动学/动力学层 | `robot_dynamics`：独立 Pinocchio 计算边界，不属于电机驱动实时链路 |

区分**逻辑总线**、**传输类型**与**设备路径**，不混称（`left_arm` ≠ `socketcan` ≠ `/dev/ttyACM0`）。
协议名、类名、ROS 2 API、topic 保留英文原名。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
```

按包构建：`colcon build --symlink-install --packages-select motor_drive`。

## 台架入口

```bash
ros2 launch robot_bringup robot.launch.py
```

一条命令起 L1（rosbridge + 静态页 + 命令节拍 `command_streamer`）与 L2（`bench_manager`）；
L3 会话（仿真 / 电机后端 / micro-ROS Agent / 重力前馈）由 `bench_manager` 按需起停。只想看页面
（不带进程管理权限）用 `ros2 launch web_host web_host.launch.py`。详见
[`src/robot_bringup/README.md`](src/robot_bringup/README.md)。

## 控制链路

- 驱动侧 controller：`/driver/controller_manager` 下的 position controller。
- 真机 bring-up：`motor_hardware_interface` 提供，默认 serial_slcan 双总线。
- 仿真链路（`robot_sim` 各有一个入口）：
  - 纯 Gazebo：`gazebo.launch.py` 用 `gz_ros2_control` 位置控制器直接驱动模型，不经 vcan
    也不经 `motor_drive`。
  - 纯 MuJoCo：`mujoco.launch.py` 用自写力矩直通桥（`mujoco_effort_bridge.py`）自己步进物理，
    话题契约与 effort 回环一致（吃 `/arm_effort_controller/commands`、出 `/joint_states`），
    完全不经 ros2_control。
  - 端到端：`simulation.launch.py` 把上面两条串成闭环——position controller →
    `motor_hardware_interface` → CAN 传输（默认 vcan，也可 socketcan/slcan）→ `motor_sim` →
    **被控对象** → `/joint_states` → `motor_sim`；effort 由 `simulation_gate` 按全轴就绪门控。
    被控对象由 `backend:=gazebo|mujoco` 选，其余接线共用。`controller_mode:=external` 只起
    被控对象与模拟端，`firmware` 换成 STM32 HIL。
- fail-fast 停机：激活重试 3 次（每次 1 s 窗口）或运行期反馈超时即终止，人工诊断后重跑。

## 真机 bring-up

默认 serial_slcan 双总线（`/dev/ttyACM0`、`/dev/ttyACM1`），无需传参：

```bash
ros2 launch motor_hardware_interface hardware.launch.py
```

覆盖设备路径：`interface_<逻辑总线名>:=<路径>`（如 `interface_left_arm:=can0`）。传输类型由
YAML 决定：切换 socketcan ↔ serial_slcan 须改
`motor_hardware_interface/config/motor_driver.yaml`；`bus_override` 只覆盖设备路径。

## 仿真 bring-up

```bash
ros2 launch robot_sim gazebo.launch.py            # 纯 Gazebo（model:= 选模型）
ros2 launch robot_sim mujoco.launch.py            # 同一模型换 MuJoCo 后端
ros2 launch robot_sim simulation.launch.py [backend:=gazebo|mujoco] \
  [transport:=vcan|socketcan|slcan] [controller_mode:=host|external|firmware]
```

`simulation.launch.py` 默认用本机自动创建/清理的 `robot_vcan0` 把两条串成闭环，拓扑由
`models/<模型名>/config/simulation_topology.yaml` 叠加权威电机配置生成。MuJoCo 要求模型自带
`mujoco/<名>.xml` 与 `urdf/<名>.standalone.urdf`，不支持 `gravity:=false`。详见
`src/robot_sim/README.md`。

## 配置（两份文件，单一来源）

- **`config/motor.yaml`（型号库）**：每型号协议运行所需静态参数（如
  `torque_constant_nm_per_amp`、`gear_ratio`）与软件侧安全阈值（额定/峰值扭矩等），与拓扑无关、
  可跨机器人复用。完整电机规格见 `src/motor_drive/docs/` 规格书。
- **`config/motor_driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 + `timing` +
  `buses` + `motors` 装配（每台电机 `type` 引用型号，单电机可覆盖参数；`command.kp/kd` 为
  每台电机的动态控制增益）。

| 术语 | 含义 |
|---|---|
| 逻辑总线 | 与设备路径解耦的总线标识，如 `left_arm` |
| 传输类型 | `BusConfig.transport`：`socketcan`（内核接口）或 `serial_slcan`（CANable2 串口），缺省 `socketcan` |
| 设备路径 | 可覆盖目标：socketcan 为接口名（`can0`），serial_slcan 为 tty（`/dev/ttyACM0`） |
| 总线覆盖 | `bus_override.<逻辑总线名>`（launch 的 `interface_<总线名>`）：只覆盖设备路径，不切传输类型 |
| joints 单一来源 | controller joints 与 URDF ros2_control 清单由 launch 从配置生成，不手抄 |
| 覆盖路径一致性 | `motor_config_file:=` 覆盖后，controller joints 与 URDF 清单以实际配置为准 |

## 核心契约

- **双文件配置**：拓扑 `motor_driver.yaml` 与型号参数 `motor.yaml` 分离；增删改电机零代码改动
  （新增物理关节时同步 URDF，测试兜底）。
- **接口自适应**：`CanTransportRegistry` + YAML `transport`，内置 `socketcan` / `serial_slcan`。
- **协议自适应**：`MotorProtocolRegistry` + YAML `type`，内置 `vaxor` / `cerb`。
- **清单自动生成**：controller joints 与 URDF ros2_control 关节清单由 launch 按实际配置生成。

## 扩展指南

| 场景 | 改动 |
|---|---|
| 增删电机 | 只改 `motor_driver.yaml` 的 `motors`（新物理关节时同步 URDF） |
| 修改电机物理参数 | 改 `motor.yaml` 型号段（`torque_constant` / `gear_ratio` 等） |
| 新增电机类型 | 实现 `MotorProtocol` + 注册工厂 + `motor.yaml` 声明 |
| 新增传输 | 实现 `CanTransport` + 注册工厂 + YAML `transport` |
