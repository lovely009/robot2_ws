# robot_dynamics

独立的 Pinocchio 运动学/动力学计算包（边界见 [ADR 0008](../../../docs/adr/0008-robot-dynamics-dependency-boundary.md)）。
当前提供 URDF 重力矩计算库 `ArmModel` 与两个节点：只做计算的 `gravity_torque_node`
（[ADR 0009](../../../docs/adr/0009-simulation-gravity-feedforward-boundary.md) 决定 1）、
把 τ_g 叠进命令帧的 `gravity_feedforward_overlay`
（[ADR 0013](../../../docs/adr/0013-gravity-feedforward-ownership-and-web-toggle.md)：
叠加路径的授权与所有权；0009 的决定 2、3、5 已被它取代）。

## 当前范围

- `ArmModel`：URDF → `pinocchio::Model`，按**关节名**给出重力矩 τ_g。
  构造期把关节分成三类：`nq==1 && nv==1` 的可控关节、可动但非单自由度（URDF
  `continuous` 在 Pinocchio 里 `nq==2`）、以及被 Pinocchio 并入父连杆的 `fixed` 关节。
  不提供 FK/IK、轨迹积分与逆动力学接口。
- `trajectory_gravity_feedforward()`：轨迹逐路点 τ_g 的纯函数（`trajectory_feedforward.hpp`），
  不依赖 ROS 消息类型，节点测试与单测都打在它上面。
- `gravity_torque_node`：订阅 `/joint_states`，发布 τ_g。**不感知** Gazebo、ros2_control
  与 CAN，**无门控与超时逻辑**（超时/长度校验下的零输出是叠加侧的事）。
- `gravity_feedforward_overlay`：夹在命令源与固件之间，把 τ_g 加进命令帧的 `effort`
  字段（= MIT 律的 τ_ff）。见下节。
- 不依赖 `motor_drive`、`motor_hardware_interface`、`motor_sim`，也不进入 ros2_control
  实时循环；`can_bridge_msgs` 只在叠加节点用（编译期依赖）。

## 节点接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `joint_states` | `sensor_msgs/JointState` | 只用 `name[]` 与 `position[]`；可用子集、任意顺序，未知名字忽略并告警 |
| 发布 | `robot_dynamics/gravity_torque` | `sensor_msgs/JointState` | **只填 `name[]` 与 `effort[]`**；`position`/`velocity` 恒为空 |

- 输出的 `name[]` 固定为模型可控关节、按 Pinocchio 模型顺序，与 `effort[]` 逐位对齐；
  输入是子集或乱序时输出不变，缺失关节沿用上一次收到的位置（初始为 neutral 0）。
- `header.stamp` 取计算时刻（节点时间），不是输入帧的时间戳。
- 发布的 `effort[]` 是模型里全部可控关节；输入帧里模型不认识的名字（如夹爪）只记
  节流告警，不整帧拒绝。`name[]` 与 `position[]` 长度不一致的帧丢弃并告警。
- QoS 为默认值（reliable、深度 10），无门控、无超时、无服务与状态机。

参数（`ros2 run robot_dynamics gravity_torque_node --ros-args -p ...`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `urdf_path` | 无（必填） | URDF 路径；为空或解析失败即构造失败 |
| `joint_states_topic` | `joint_states` | 输入话题，相对名 |
| `gravity_torque_topic` | `robot_dynamics/gravity_torque` | 输出话题，相对名 |

相对名在默认命名空间下解析为 `/joint_states` 与 `/robot_dynamics/gravity_torque`，并随
命名空间/remap 正常组合；需要绝对话题时传前导 `/`。

## 重力前馈叠加节点 `gravity_feedforward_overlay`

### 解决什么

HIL（`controller_mode:=firmware`）下，主机命令经 micro-ROS Agent 到 STM32 固件，固件再发
CAN MIT 帧给电机：`τ = kp(p_des−p) + kd(v_des−v) + τ_ff`。**τ_ff 就是重力前馈的注入位**，
命令消息 `can_bridge_msgs/MotorCommandTarget.effort`（关节侧 N·m）与轨迹点
`MotorTrajectoryPoint.effort` 就是它。

三个命令源（web 上位机、`micro_ros_agent/tools/snapshot_stream.py`、`traj_play.py`）默认直接
发 `/motor/command_array`（轨迹走 `/motor/trajectory`），再由 `micro_ros_bridge` 编成线上
`low_cmd` / `trajectory`。micro-ROS Agent 的 CLI 显式丢弃 `--ros-args`、无 remap 能力，
固件侧话题名又是编译期常量——所以「夹在中间加前馈」只能靠**改命令源的话题名**，本节点
就是那个中间层：

    命令源 ──► /robot_dynamics/command_array ─┐
    命令源 ──► /robot_dynamics/trajectory    ─┤
                                               ▼
                             [ gravity_feedforward_overlay：加上 τ_g ]
                                               │
                             ┌─────────────────┴─────────────────┐
                             ▼                                   ▼
               /motor/command_array   /motor/trajectory ──► micro_ros_bridge ──► 固件

**按需旁路，默认不变**：命令源的默认话题保持原样，只有启用前馈时才被指到
`/robot_dynamics/*`。本节点不是必经一跳。

### 话题与 QoS

| 方向 | 话题（默认，相对名） | 类型 | QoS |
|---|---|---|---|
| 订阅 | `robot_dynamics/command_array` | `can_bridge_msgs/MotorCommandArray` | reliable + volatile，深度 1 |
| 订阅 | `robot_dynamics/trajectory` | `can_bridge_msgs/MotorTrajectory` | reliable + volatile，深度 10 |
| 订阅 | `joint_states` | `sensor_msgs/JointState` | reliable + volatile，深度 1 |
| 发布 | `/motor/command_array` | `can_bridge_msgs/MotorCommandArray` | **reliable** + volatile，深度 1 |
| 发布 | `/motor/trajectory` | `can_bridge_msgs/MotorTrajectory` | **reliable** + volatile，深度 1 |

输出 QoS 是**照 `micro_ros_bridge` 主机入口的订阅端定的**，不是本节点的偏好：bridge 对
`/motor/*` 的订阅是 RELIABLE（随后它按线上契约重新编码并选用线上 QoS）——发
best_effort 在 ROS 2 里不建连接、静默失效，别按"更可靠总是更好"顺手改。

### 参数

| 名字 | 类型 | 默认值 | 语义 |
|---|---|---|---|
| `urdf_path` | string | 无（必填） | URDF 路径；空或解析失败 → **构造期抛异常，节点起不来**（静默直通会让人以为前馈开着） |
| `joint_map` | string | `1:joint_link1,...,9:joint_link9,10:joint_link1.1,...,18:joint_link9.9` | `motor_id:关节名` 逗号分隔；空串 = 用内置兜底表。**兜底表只对默认模型 `half_body_robot` 正确**，换模型必须传参（该表此前因换模型静默失效过一次）。条目畸形、id 非 0..65535、id 重复、关节名为空 → 构造期抛异常 |
| `command_input_topic` | string | `robot_dynamics/command_array` |  |
| `trajectory_input_topic` | string | `robot_dynamics/trajectory` |  |
| `command_output_topic` | string | `/motor/command_array` | `micro_ros_bridge` 的主机入口 |
| `trajectory_output_topic` | string | `/motor/trajectory` | `micro_ros_bridge` 的主机入口 |
| `joint_states_topic` | string | `joint_states` | 快照通道的构型来源 |
| `joint_states_timeout_s` | double | `0.5` | 超过这么久没有新的 `/joint_states` 就视为 τ_g 不可用（原样转发）；`0` = 永远视为过期（A/B 直通用） |
| `effort_gain` | double | `1.0` | 乘在 τ_g 上；**运行期可改**（`ros2 param set /gravity_feedforward_overlay effort_gain 0.0`），0 时输出逐字段等于输入 |

### 两个通道的计算差异

- **快照通道（`MotorCommandArray`）**：用 `/joint_states` 的**实测**构型算 τ_g——重力矩是实际
  状态的函数。收到 `/joint_states` 后 `ArmModel::update_configuration(names, positions)`，
  帧到达时取 `gravity_torque()`，对每个 `targets[i]` 按其 `motor_id` 查关节取对应分量：
  `targets[i].effort += effort_gain * τ_g[joint]`。
  `/joint_states` 从未收到、超时、电机/关节没映射上 → **该帧不加任何东西，原样转发**。
- **轨迹通道（`MotorTrajectory`）**：轨迹是**未来**构型，实测话题给不了，所以用
  `ArmModel::gravity_torque_at()` 无副作用逐路点算：
  1. 按 `time_from_start_ms` 把 `points` 归组（同一时刻的多个点 = 同一构型）；
  2. 每个出现过的时刻 t 组装 q：`target_ids` 里每台电机——该时刻**有**路点就用它的
     `position`，**没有**就在它自己的相邻路点之间**线性插值**（早于首点/晚于末点这种
     只有单侧路点的情况保持该侧值）；缺拍**不跳过**该电机（重力矩多关节耦合，少算一个
     关节会让别的关节的前馈也错）；本分段完全没有路点的电机不参与（保留 `ArmModel`
     内部构型）；
  3. 用组装好的 (names, positions) 调 `gravity_torque_at`，把结果按 `effort_gain` 加到
     该时刻每个路点的 `effort` 上。
  **不跨分段边界**：处理单位是一条 `MotorTrajectory`（= 一个分段），相邻分段的时刻不互相
  插值——分段端点因此可能取到保持值而不是跨段插值，这是有意的取舍。

### 安全不变式：任何情况下都不丢帧

命令快照是**完整快照**语义（未出现的电机 = 固件立即软停），被控对象没有抱闸，丢一帧比少加
一点前馈危险得多。所以：

- τ_g 不可用（无 `/joint_states`、超时、`motor_id` 不在 `joint_map`、关节名不在 URDF、
  模型更新失败）→ **原样转发该帧**（τ_g 贡献为 0），不是丢弃、不是空帧、不是部分帧。
- 输入与输出长度/容量不符、字段异常（`count` 超界、路点 `motor_id` 不在 `target_ids` 里）
  → 同样原样转发。
- 转发逐字段完整：`sync_time_ns`/`sequence`/`count`/`target_count`/`segment_index`/
  `segment_count`/`target_ids` 与所有路点字段原样保留，**只加 `effort`**——改 `target_ids`
  或 `count` 会改变固件的归属语义。
- 只有 `urdf_path` 缺失或 URDF 解析失败 fail-fast（节点起不来）。
- 未知 `motor_id` / 模型外关节名 → 节流告警（5 s）但继续，不刷屏也不静默。

### 怎么启用

**常规路径是 web 上位机运行屏的「重力补偿」开关**
（[ADR 0013](../../../docs/adr/0013-gravity-feedforward-ownership-and-web-toggle.md)、
[0014](../../../docs/adr/0014-host-side-command-pacing.md)）：它把本节点当第四个案台 unit
托管（`bench-gravity-feedforward.service`），并把下行切到本节点的输入话题。**两条通道的
归属不同**：命令那条（`command_array`）由 `bench_manager` 改 `command_streamer` 的
`output_topic` 切——页面只下设定值，切不了它；轨迹那条仍由页面自己切。

「起节点」与「改接线」是两件事，服务按「先起节点、确认 running，再改道」的顺序做；
关的时候反过来（先改道回直连、**确认成功才**停节点）。顺序反过来会出现「话题已切、
节点没起」的空窗，或停止时「节点没了、接线还指着它」——两种都让命令落进没人订阅的话题。

手工路径（其它命令源 / 排查）：命令源不再直接发 `/motor/*`，改指到
`/robot_dynamics/*`（各命令源自己改话题名或 remap）；然后起节点：

```bash
ros2 launch robot_dynamics gravity_feedforward.launch.py \
  urdf_path:=/home/ly/robot_ws/ros2_ws/src/robot_sim/models/half_body_robot/urdf/<模型>.urdf \
  joint_map:=1:joint_link1,2:joint_link2    # 留空 = 内置兜底表（只对 half_body_robot 正确）
```

launch 只透传 `urdf_path` 与 `joint_map`，只起本节点，不 include 也不修改
`simulation.launch.py`——**接线始终是命令源自己的事**，本节点不旁观谁在发命令。所以页面
按钮管不到 `snapshot_stream.py` / `traj_play.py`，它们与前馈同时跑就是混流。
（`command_streamer` 不在此列：它是页面下行的**唯一**通道，本就该经本节点。）A/B 对照：运行期 `ros2 param set /gravity_feedforward_overlay
effort_gain 0.0` 时两个通道都逐字段等于输入。

## C++ 接口

权威定义在 [`include/robot_dynamics/arm_model.hpp`](include/robot_dynamics/arm_model.hpp)：

- `explicit ArmModel(const std::string & urdf_path)`：空路径/文件缺失/解析失败抛
  `std::invalid_argument`。
- `joint_names()` / `non_scalar_joint_names()` / `fixed_joint_names()` / `has_joint(name)`：
  构造期完成的关节分类，`joint_names()` 的顺序即 τ_g 位序。
- `update_configuration(names, positions)`：按名字更新内部构型，返回未能生效的名字
  （模型里不存在或非可控）；长度不一致抛 `std::invalid_argument`。
- `gravity_torque()`：当前内部构型的 τ_g [Nm]。
- `gravity_torque_at(names, positions)`：以内部构型为底、无副作用求 τ_g（轨迹前馈按
  路点求值用），同时返回本次忽略的名字。
- `trajectory_gravity_feedforward(model, joint_map, target_ids, points)`（权威定义在
  [`include/robot_dynamics/trajectory_feedforward.hpp`](include/robot_dynamics/trajectory_feedforward.hpp)）：
  轨迹逐路点的 τ_g 增量，与 `points` 逐位对齐，不含 `effort_gain`。

`ArmModel` 非线程安全（内部共享一份 Pinocchio scratch），只在单线程执行器里用。

## 依赖与构建

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src/robot_dynamics --ignore-src -r -y   # 装 ros-humble-pinocchio
colcon build --symlink-install --packages-select robot_dynamics
source install/setup.bash
colcon test --packages-select robot_dynamics && colcon test-result --verbose
```

## 测试

测试自带 fixture [`test/fixtures/planar_arm.urdf`](test/fixtures/planar_arm.urdf)：3R 平面臂
（重力矩有闭式解），另挂一个 massless 的 fixed 关节与一个 continuous 关节用于关节分类。
不依赖 `robot_arm_gazebo` 的模型资产，不启动 Gazebo / MuJoCo，不占 vcan 与串口。

- `test_arm_model.cpp`：闭式解对照、`computeGeneralizedGravity` 与势能梯度有限差分对照、
  子集/乱序/含未知名字输入、缺失关节沿用前值、无副作用求值。
- `test_gravity_torque_node.cpp`：节点只填 `name`/`effort`、输出顺序与位序、乱序子集不
  拒帧、畸形帧只告警不发布。
- `test_trajectory_feedforward.cpp`：同拍 = 逐点 g(q)、缺拍线性插值、单侧路点保持、
  未知 `motor_id`/模型外关节名只进诊断、单点与空轨迹。
- `test_gravity_feedforward_overlay.cpp`：参数契约与 fail-fast、快照按实测构型加 τ_g 且
  其余字段逐字段不变、超时/无 `/joint_states` 时逐字段直通、未知 `motor_id` 只不动自己、
  轨迹缺拍用插值构型、畸形与取消帧直通、`effort_gain` 运行期改 0 = 直通。
