# 仿真重力补偿前馈设计

- 日期：2026-10-08
- 状态：待批准
- 范围：`simulation.launch.py` 的 `host` / `external` 组合里，给 Gazebo effort 闭环叠加
  Pinocchio 计算的重力矩；真机 CAN 前馈路径不在本轮
- ADR：[0009 仿真重力前馈的归属与边界](../../adr/0009-simulation-gravity-feedforward-boundary.md)

## 背景

`gazebo.launch.py` 的 `gravity` 默认 `true`，模型自身不带重力补偿，`motor_sim` 输出的关节
力矩里没有重力项，因此位置闭环在重力下存在稳态落差 `Δq ≈ τ_g/kp`。本模型总质量 0.73 kg、
MIT `kp = 30`，实测最大重力矩 `τ_g ≈ 0.90 Nm`（`joint_link1` 在 q = 0.5 rad），对应落差
≈ 0.03 rad。

`robot_dynamics` 目前只验证 Pinocchio 依赖可用（ADR 0008），没有任何计算能力。本设计让
Pinocchio 真正接进仿真：计算 τ_g，并在仿真侧把它叠加到电机输出力矩上。真机侧要复用这条
前馈必须走 CAN MIT 帧，受 Humble 无节点句柄、实时路径边界与 MIT 力矩量程不一致三条限制
（见 ADR 0009 Context），本轮不做。

## 决策

### 数据流

```text
/joint_states ─┬─> motor_sim（不修改，仍按 MIT 律算力矩）
               └─> gravity_torque_node ─> /robot_dynamics/gravity_torque
                                              （sensor_msgs/JointState：name + effort）

motor_sim ─> /motor_sim/effort ─┐
                                 ├─> gravity_feedforward_mixer ─> /arm_effort_controller/commands
/motor_sim/drive_state ─────────┘
```

`gravity_feedforward:=false`（默认）时不启动这两个节点，`motor_sim` 的 `effort_topic` 仍是
`/arm_effort_controller/commands`，接线与今天完全一致。

### 为什么按关节名对齐而不是下标

两边的顺序来源不同：Pinocchio 的关节序是 URDF 顺序，effort 数组的顺序是
`simulation_topology.yaml` 的电机顺序；两者今天相同只是巧合。`gravity_torque` 因此携带
`name`，由叠加侧按下标映射，顺序差异不构成隐式耦合。

话题类型用 `sensor_msgs/JointState`（只填 `name` 与 `effort`），不新增消息包：这个量是仿真
内部中间量，`can_bridge_msgs` 是固件契约，不适合承载它；为一条消息新增消息包的成本高于收益。
代价是复用了关节状态类型表达力矩，已在此处显式记录。

### `robot_dynamics`：`ArmModel`（纯计算）

```cpp
namespace robot_dynamics {

class ArmModel {
public:
  // urdf_path 不存在、无法解析，或含自由浮动基座/非 1 自由度关节时抛 std::runtime_error。
  explicit ArmModel(const std::string &urdf_path);

  const std::vector<std::string> &joint_names() const;  // 与 gravity_torque 同序
  std::size_t nv() const;

  // 重力矩（关节侧 Nm）：τ = RNEA(q, 0, 0)，等价于 ∂U/∂q。
  // q 长度必须等于 nv，否则抛 std::invalid_argument。
  std::vector<double> gravity_torque(const std::vector<double> &q);

private:
  pinocchio::Model model_;
  pinocchio::Data data_;  // 非线程安全：一个实例只在一个线程里使用
};

}  // namespace robot_dynamics
```

- 关节名取 Pinocchio 模型里除 universe 之外的关节名（本模型实测为 `joint_link1..4`，与
  `models/<模型>/config/controllers.yaml` 一致）。
- 单位边界：输入 rad，输出 Nm，均为关节侧物理量；不做减速比或方向换算。
- 重力向量取 Pinocchio 默认 `(0, 0, −9.81)`；Gazebo 世界默认 `−9.8`，0.1% 偏差本轮不补偿。
- 不做静默降级：模型不符合假设就直接抛错，由节点或测试显式暴露。

### `robot_dynamics`：`gravity_torque_node`

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_file` | 空 | URDF 路径（必填）。空、缺失或解析失败时节点报错退出，launch 随之失败 |
| `joint_states_topic` | `/joint_states` | 关节状态输入 |
| `gravity_torque_topic` | `/robot_dynamics/gravity_torque` | 重力矩输出 |

行为：

1. 收到 `sensor_msgs/JointState` 后按名字取每个模型关节的位置；缺任一关节、或值非有限时
   记 WARN（限频）并**本周期不发布**，不用过期或错误位姿算出的力矩驱动仿真。
2. 发布 `sensor_msgs/JointState`：`header.stamp` 用输入时间戳，`name` 为模型关节名（固定），
   `effort` 为 τ_g，`position` / `velocity` 留空。
3. 没有输入就没有输出，不发布零、不做超时：降级策略只由叠加侧决定，避免两处各自兜底。
4. 节点不感知 Gazebo、ros2_control 与 CAN，不订阅 motor_sim 的任何话题。

### `robot_arm_gazebo`：`gravity_feedforward_mixer`

| 参数 | 默认 | 说明 |
|---|---|---|
| `joints` | 无（必填） | 有序关节名，长度必须等于 effort 数组长度；由 launch 按拓扑顺序传入 |
| `effort_input_topic` | `/motor_sim/effort` | motor_sim 的力矩输出 |
| `effort_output_topic` | `/arm_effort_controller/commands` | Gazebo effort 控制器输入 |
| `gravity_torque_topic` | `/robot_dynamics/gravity_torque` | τ_g 输入 |
| `drive_state_topic` | `/motor_sim/drive_state` | 控制门控状态 |
| `input_timeout_ms` | 100 | effort 或 τ_g 超过该时长未更新即视为失效 |
| `publish_rate_hz` | 100 | 输出定时器频率，与 `motor_sim` 的 control_hz 一致 |

叠加规则（纯函数 `mix_efforts()`，与 ROS 节点分离以便测试）：

| 条件 | 输出 |
|---|---|
| 门控关闭（`drive_state` 不是 `ready_for_control` / `motor_control_active`） | 全零 |
| effort 输入超时 | 全零 |
| effort 数组长度 ≠ `len(joints)` | 全零 + ERROR（状态变化时记一次） |
| τ_g 未覆盖某关节，或 τ_g 输入超时 | 该关节前馈记 0，effort 直通 |
| 正常 | `effort[i] + τ_g[joints[i]]` |

- 门控初值为关闭；没有输入、输入乱序、输入缺失都只会让输出变成零或直通，不会放大。
- 定时器按 `publish_rate_hz` 固定发布，保证「输入停了要归零」不依赖下一条输入到达。
- 节点退出时由 launch 整体关闭（与 `simulation_gate.py` 相同的 `OnProcessExit` 处理）。
- 不对叠加后的总和限幅：Gazebo 侧仍有 URDF `<limit effort="10.0">`；与真机「电机内部对含
  前馈的总力矩限幅」在饱和区的语义差异已记入 ADR 0009。

### launch 接线

`simulation.launch.py` 新增参数 `gravity_feedforward`（默认 `false`）。为 `true` 时，在任何
进程启动之前校验下列前置条件，不满足就抛 `RuntimeError`：

1. `gravity:=true`（否则前馈会把臂抬起来）；
2. `controller_mode ∈ {host, external}`（`firmware` 走 STM32 固件与台架模拟器，本轮不接）；
3. `models/<模型>/urdf/<模型>.standalone.urdf` 存在（Pinocchio 用展开后的纯模型，不含
   `<ros2_control>` 与 `<gazebo>` 块）。

通过后：`motor_sim` 的 `effort_topic` 改为 `/motor_sim/effort`；启动 mixer（`joints` 用
launch 已算出的拓扑关节顺序）与 `robot_dynamics` 的 `gravity_torque_node`。

## 文件与模块边界

新增：

- `ros2_ws/src/robot_dynamics/include/robot_dynamics/arm_model.hpp`、`src/arm_model.cpp`
- `ros2_ws/src/robot_dynamics/src/gravity_torque_node.cpp`
- `ros2_ws/src/robot_dynamics/test/test_arm_model.cpp`、`test/models/pendulum.urdf`（单摆夹具）
- `ros2_ws/src/robot_arm_gazebo/scripts/gravity_feedforward_mixer.py`
- `ros2_ws/src/robot_arm_gazebo/test/test_gravity_feedforward_mixer.py`

修改：

- `robot_dynamics`：`CMakeLists.txt`（`INTERFACE` 库改为普通库 + 可执行 + 测试）、
  `package.xml`（`rclcpp`、`sensor_msgs`）、`README.md`
- `robot_arm_gazebo`：`launch/simulation.launch.py`、`CMakeLists.txt`（安装脚本 + pytest）、
  `package.xml`（`exec_depend` `robot_dynamics`）、`README.md`
- 根 `README.md` 与 `CLAUDE.md` 中「`robot_dynamics` 当前仅验证依赖、无节点/话题」的描述

不修改：`motor_drive`、`motor_sim`、`motor_hardware_interface`、模型 URDF/xacro、
`gazebo.launch.py`、`web_host`、固件与 micro-ROS Agent。`motor_sim` 的 MIT 力矩量程不一致
（ADR 0009 Context 第 3 条）本轮只记录，不修。

## 验证标准

1. `ArmModel` 单测（不启 Gazebo、不依赖 `robot_arm_gazebo`）：
   - 单摆夹具 URDF：`τ_g(q) = m·g·l·cos(q)` 在若干 q 上与解析值一致；
   - 能量一致性：对夹具模型用中心差分算 `∂U/∂q`（U 由 Pinocchio 质心高度给出），与
     `gravity_torque` 一致；
   - 失败路径：q 长度不符抛 `std::invalid_argument`；URDF 不存在或含非 1 自由度关节抛
     `std::runtime_error`。
2. mixer 纯函数测试：门控关闭、effort 超时、长度不符、τ_g 缺关节、τ_g 超时、正常叠加。
3. `colcon build` 与 `colcon test` 对 `robot_dynamics`、`robot_arm_gazebo`（及相关包）通过，
   既有测试不回退；自动化测试不启动 Gazebo、不连接真实硬件。
4. 手动 A/B（需 Gazebo 与 vcan 权限，写入 `docs/reports/`）：命令位姿 `[0.5, 0, 0, 0]`，
   比较 `gravity_feedforward:=false` 与 `true` 的 `/joint_states` 稳态误差，预期 `joint_link1`
   的落差从 `τ_g/kp ≈ 0.03 rad` 降到接近 MIT 律残差；同时记录 `gravity_torque` 与实测 τ_g
   的对照值。
5. `gravity_feedforward` 默认 `false` 时，`simulation.launch.py` 生成的话题与节点集合与
   今天一致。

## 非目标

- 真机 CAN MIT 前馈通道（`CanFdSystem` 的 effort 命令接口或等效注入点）。
- 修正 `motor_drive` / `motor_sim` 之间 CERB MIT 力矩量程不一致。
- 惯量参数辨识或标定；CAD 惯性仍按占位值使用。
- 自由浮动基座、非 1 自由度关节、多模型或 6/7 轴通用化；本轮只服务现有一台 4 关节模型。
- 速度/加速度相关项（科氏力、轨迹前馈、逆动力学），以及 Gazebo 世界 `−9.8` 与 Pinocchio
  `−9.81` 的差异补偿。
- 在 `gazebo.launch.py`（不经电机链路）或 `web_host` 上暴露前馈开关与曲线。
