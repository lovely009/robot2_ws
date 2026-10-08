# 仿真重力前馈的归属与边界

**Status**: accepted

## Context

`robot_arm_gazebo` 的端到端仿真在 `control_mode:=effort` 下由 `motor_sim` 计算关节力矩并
直接发布到 `/arm_effort_controller/commands`；`gazebo.launch.py` 的 `gravity` 默认 `true`，
而模型不带重力补偿（模型 xacro 与 README 都写明「模型未内置重力补偿时需要单独调增益/前馈」）。
同时 `robot_dynamics` 只验证 Pinocchio 可被找到、编译与链接（ADR 0008）：不读 URDF、没有
节点/话题，不进入任何运行路径。

本次要让 Pinocchio 真正参与仿真：给 Gazebo 的 effort 闭环加**重力补偿/力矩前馈**，用于观察
重力下的稳态落差。设计前核对过的既有事实（2026-10-08）：

1. 真机 MIT 帧自带前馈力矩字段：`motor_drive::protocol::JointCommand::feedforward`，CERB 在
   电机内算 `τ = kp(p_des−p) + kd(v_des−v) + τ_ff`；`motor_sim` 的 plant 已按同一式子实现，
   并在 profile 的 `effort_nm` 内限幅。
2. 主机适配层只导出 position 命令接口（`motor_hardware_interface/urdf/canfd_hardware.xacro`
   只有 `<command_interface name="position"/>`）；`CanFdSystem::send_position_commands()`
   只写 position/kp/kd，MIT 帧里的 `feedforward` 恒为 0（默认值）。
3. **执行器的 MIT 前馈通道在仿真回环里目前不可用**：`motor_drive` 按配置
   `mit_torque_limit_nm: 45.0` 用 12 位编码 τ_ff（步长 0.022 Nm），`motor_sim` 的 CERB 解码
   常量是 `kCerbTorqueMaxNm = 11.0`（按 R48-101 峰值）。实测同一帧 5.0 Nm 前馈经
   encode→decode 变成 1.222 Nm（比例 11/45）；位置/kp/kd 往返正常。τ_ff 至今恒为 0，
   所以这个量程不一致此前不可见。
4. ROS 2 Humble 的 `hardware_interface::SystemInterface` 不提供节点句柄，硬件组件无法自行
   订阅话题；把 Pinocchio 链接进 `CanFdSystem` 等于把 Pinocchio 放进 ros2_control 实时循环。
5. Pinocchio 4.0.0 能直接解析 `models/<模型>/urdf/<模型>.standalone.urdf`：本模型解析结果
   nq=nv=4，关节名 `joint_link1..4` 与 `controllers.yaml` 一致；RNEA 的重力项与势能梯度
   `∂U/∂q` 数值一致（有限差分核对到 1e-9）。

## Decision

1. **计算归 `robot_dynamics`**：新增 `ArmModel`（URDF → `pinocchio::Model`，按关节名给出
   重力矩 τ_g）与节点 `gravity_torque_node`——订阅 `/joint_states`，发布
   `/robot_dynamics/gravity_torque`（`sensor_msgs/JointState`，只填 `name` 与 `effort`）。
   该节点不感知 Gazebo、ros2_control 与 CAN，没有门控与超时逻辑（那是叠加侧的事）。
2. **叠加归 `robot_arm_gazebo`**：新增 `scripts/gravity_feedforward_mixer.py`，把 `motor_sim`
   的 effort 与 τ_g 求和后发布到 `/arm_effort_controller/commands`；只在
   `simulation.launch.py` 的 `gravity_feedforward:=true`、且 `controller_mode ∈ {host, external}`
   与 `gravity:=true` 时接线。默认 `false`，现有接线不变。
3. **叠加点是 Gazebo 关节力矩，不是 CAN 帧**：τ_g 以关节侧 Nm 直接加在电机输出力矩上，
   叠加前后都不经过 CERB MIT 编码，因此不受第 3 条量程问题与 12 位量化影响。
4. **依赖方向**：没有包在编译期依赖 `robot_dynamics`。新增的是
   `robot_arm_gazebo → robot_dynamics` 的 `exec_depend`（launch 要启动该节点）与一条运行期
   话题契约。ADR 0008 中「`robot_dynamics` 当前不提供 ROS 节点、topic、参数」与「不进入
   Gazebo 运行路径」两条由本 ADR 取代；「`motor_drive` 不依赖 ROS」「Pinocchio 不进入
   ros2_control 实时循环」「真机适配层不依赖 Pinocchio」继续有效。
5. **真机 τ_ff 路径另立 ADR**：把 τ_g 写进 CAN MIT 前馈字段需要 (a) 给 `CanFdSystem` 增加
   effort 命令接口或等效注入点，(b) 解决 Humble 无节点句柄，(c) 先修掉第 3 条的编解码量程
   不一致。本轮不做。

## Considered Options

- **仿真侧关节力矩叠加（已选）**：不动 `motor_sim`、`motor_hardware_interface`、URDF/xacro 与
  实时路径；叠加发生在关节侧，语义清晰；代价是前馈不经 CAN 帧，**不能直接复用到真机**，
  只是仿真装置与 Pinocchio 的接入验证。
- **把前馈写进 CAN MIT 帧，与真机同路径**：语义最正，但 Humble 的 `SystemInterface` 无节点
  句柄；让硬件适配层内联 Pinocchio 会把 Pinocchio 带进实时循环，违反 ADR 0008 划出的边界；
  而且第 3 条的量程不一致会让仿真里的前馈被缩小 4.09 倍，今天也无法验证「真机保真」。推迟。
- **在 `motor_sim` 的 plant 内叠加外部前馈话题**：能复用 plant 的限幅语义，但 `motor_sim` 是
  「与机器人无关的电机模型」，外部前馈话题要把仿真装置塞进电机语义，还要在电机名与关节名
  之间建映射；同时会掩盖「电机输出」与「补偿项」的边界，验收时分不清差异来自哪一侧。否决。
- **把 position 命令接口改成 effort 命令接口，由控制器写力矩**：要改真机 RT 适配层与
  `canfd_hardware.xacro`，且仍要把 Pinocchio 放进控制器实时路径。否决。

## Consequences

- 现有仿真路径默认逐字节不变；`gravity_feedforward:=false` 时 `motor_sim` 仍直接发布 effort。
- 能观察到「有/无前馈」的稳态落差差：本模型总质量 0.73 kg、MIT kp=30，实测最大
  τ_g ≈ 0.90 Nm（`joint_link1` 在 q=0.5 rad），对应落差 ≈ τ_g/kp ≈ 0.03 rad；`joint_link2`、
  `joint_link4` 在自己的转角下 τ_g ≈ 0，落差不可见。因此本轮仿真的价值在**结构与管线验证**
  （Pinocchio 接入、接线、门控、超时），数值意义上的补偿效果在 14 电机真机拓扑上才有意义。
- 前馈只在控制门开启时叠加；effort 或 τ_g 超时、长度不符一律输出零，不新增第二条安全路径。
- 叠加后的总力矩可超出 `motor_sim` 的 profile `effort_nm`，Gazebo 侧仍受 URDF
  `<limit effort="10.0">` 约束；与「电机内部对含前馈的总力矩限幅」在饱和区存在语义差异，
  已记录，本轮不补偿。
- Pinocchio 取 (0,0,−9.81)，Gazebo 世界按 SDF 规范默认 −9.8
  （`/usr/share/sdformat12/1.9/world.sdf` 的 `<gravity>` 默认值），0.1% 偏差本轮不补偿，
  写入文档。
- `robot_dynamics` 从「只验证依赖」变为「参与仿真运行路径的计算包」；其自动化测试仍不得
  启动 Gazebo，也不得依赖 `robot_arm_gazebo` 的模型资产。
