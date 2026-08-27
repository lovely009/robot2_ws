# 机械臂仿真调试的官方实践调研

日期：2026-08-27  
范围：ROS 2 Humble、Gazebo Fortress、`gz_ros2_control`、SDFormat，以及 Universal Robots 官方运行语义。本文只依据官方文档、规范和官方源码；结论用于仿真设计，不代表真实 UR5e 的制动性能或安全认证。

## 结论摘要

主流做法不是把“断电”简单实现成零 effort，而是把机械臂分成可观察、可切换的运行状态：控制器未接管时保持关节；有效控制期间由运动控制器占用接口；超时或故障时停止运动控制并进入保持状态；恢复动作由显式操作触发。ROS 2 Control 本身支持运行时 controller lifecycle 与切换，并没有规定仿真故障后必须重启整个 launch。

对于本工程，更合适的调试语义是：

- 启动、未 enable、enable 后尚无首帧有效命令、disable、超时和断线时进入“仿真抱闸/保持”。
- 第一帧有效命令到达后释放保持，由现有 CAN 协议与 effort 控制链驱动。
- `fault_latching_enabled` 保持为 launch 可配置项。为贴近安全运行可设为 `true`；本机反复调试可设为 `false`，恢复时仍应要求明确的 disable/enable 或 reset/rearm 状态转换，不能由任意运动帧自动解锁。
- 不必强制重启整个 launch。锁存开启时可要求显式 reset/rearm；锁存关闭时可在故障条件消失后重新 enable。ROS 2 Control controller 可在运行时切换。
- 模型级启用 `self_collide`；SDFormat 已规定直接由 joint 相连的 link 不参与彼此碰撞，不必先手写一份相邻 link 排除表。
- 将接口/协议调试与动力学/接触调试分层，避免为了调 CAN 而每次都依赖完整物理场景。

## 1. 未使能、断线与“抱闸”

`gz_ros2_control` 官方文档提供 `<hold_joints>`：默认值为 `true`，当关节接口未被 controller claim（例如 controller 尚未激活）时保持关节位置。[gz_ros2_control 官方文档](https://github.com/ros-controls/gz_ros2_control/blob/rolling/doc/index.rst)

这意味着一种符合现有框架的实现路径是：抱闸时让 effort controller 释放六轴 command interface，由 `gazebo_ros2_control` 对未占用关节执行保持；释放抱闸时再激活 effort controller。Controller Manager 正是为 controller 的配置、激活、停用和切换提供生命周期管理。[Controller Manager 官方文档](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)

但必须注意，“`hold_joints=true`”不是经过参数化的真实机械制动器模型。它是接口未被占用时的仿真保持机制；应实测重力下的漂移、抖动和控制器切换瞬态，不能把结果描述成真实 UR5e 抱闸能力。[gz_ros2_control 官方文档与源码仓库](https://github.com/ros-controls/gz_ros2_control)

另一种标准保持机制是 `joint_trajectory_controller`：其 API 将“保持当前位置”定义为以当前位置、零速度和零加速度创建新命令；effort command interface 下还可通过 PID 闭环产生保持力矩。[JointTrajectoryController API](https://control.ros.org/humble/doc/api/classjoint__trajectory__controller_1_1JointTrajectoryController.html)、[Humble 参数文档](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)。不过这属于通电闭环保持，并非失电机械抱闸，而且会与当前直接占用 effort interface 的 CAN 控制器冲突，因此不宜和现有 effort controller 同时激活。

## 2. Controller 切换是正常运行机制

ROS 2 Control 的 `SwitchController` 服务可以在 controller manager 的同一个控制周期中停用一组 controller 并激活另一组；`STRICT` 模式会在名称无效或激活失败等情况下令整个切换失败。[SwitchController 服务定义](https://docs.ros.org/en/ros2_packages/humble/api/controller_manager_msgs/srv/SwitchController.html)

官方 RRBot 示例也演示了 controller 先加载为 `inactive`，再通过 `ros2 control switch_controllers --activate ... --deactivate ...` 在运行时切换。[ROS 2 Control RRBot 示例](https://control.ros.org/humble/doc/ros2_control_demos/example_1/doc/userdoc.html)

因此，仿真后端可以明确维护以下状态，而无需重启 Gazebo：

1. `brake_engaged`：effort controller 不占用接口，由保持路径接管。
2. `awaiting_first_command`：已经 enable，但继续保持。
3. `motor_control_active`：首帧有效命令到达，切换到 effort 控制。
4. `fault_or_timeout`：先清零电机输出，再切回保持。

切换必须做成有确认结果的状态机：只有 Controller Manager 返回成功后才更新对外状态；建议使用 `STRICT` 和有限超时，并测试 Gazebo 暂停、恢复及快速重复切换。

## 3. 故障锁存不必等同于“重启 launch”

真实 UR 的不同停机类型本来就有不同恢复流程。UR 官方资料显示，Protective Stop 可在排除原因后执行 unlock 并继续；Emergency Stop 释放后还要执行 brake release；Safety fault/violation 则需要 restart safety、power on、brake release，再重新运行程序。[Universal Robots Stop recovery](https://docs.universal-robots.com/tutorials/controlling-robot-externally/stop-recovery.html)

这说明更现实的抽象是“显式确认和重新使能”，而不是所有故障一律杀死进程。UR 官方 ROS 2 driver 也使用 `controller_stopper`：机器人运行状态变为 false 时停止运动 controllers，恢复为 true 时重启此前停止的 controllers。[ur_robot_driver 官方文档](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/index.html)

ROS 2 Control Humble 对硬件重启后的全自动恢复仍有限制：官方文档明确说明，硬件重启应重新走 lifecycle，但该版本自动停止/恢复相关 controllers 的路径尚未完成，暂时需要手工 stop/restart controllers。[Controller Manager：Restarting hardware](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)

据此，本工程可以提供两种 launch 策略：

- `fault_latching_enabled:=true`：故障保持锁存，普通运动命令无效，但通过专用 reset/rearm 操作恢复；仅对无法安全复位的内部错误要求重启节点或 launch。
- `fault_latching_enabled:=false`：用于本机仿真调试。故障条件消失后，必须重新完成 disable/enable（或 reset/rearm）才能释放抱闸；避免一帧迟到的旧命令直接恢复运动。

该策略既保留安全边界，也能满足快速重复调试。它只改变仿真调试体验，不应被描述成真实机器人的安全恢复流程。

## 4. 自碰撞应由模型和物理引擎负责

SDFormat 模型规范中，`<self_collide>` 默认是 `false`；设为 `true` 后，同一模型内 link 可以相互碰撞，但由 joint 直接连接的两个 link 被规范排除。link 级 `self_collide` 还可覆盖模型级设置。[SDFormat model 规范](https://sdformat.org/spec/?elem=model&ver=1.10)、[SDFormat link 规范](https://sdformat.org/spec/?elem=link&ver=1.9)

URDF 经 Gazebo 转换为 SDF 时，可通过 `<gazebo>` extension 写入 `selfCollide`；官方还建议用 `gz sdf -p <urdf>` 检查转换后的 SDF。[SDFormat 官方 URDF extensions](https://github.com/gazebosim/sdf_tutorials/blob/master/docs/urdf/sdf_extensions.md)

推荐调试顺序是：

1. 检查每个 link 是否有 collision geometry，尺寸、origin 与 visual 是否合理。
2. 转换并检查最终 SDF 中 `self_collide` 是否实际生效。
3. 验证初始姿态没有非相邻 collision mesh 重叠。
4. 逐步施加运动命令，观察 contact、抖动、求解器不稳定和实时因子。

自碰撞只能阻止实体互穿，不会自动提供姿态保持；抱闸也不能替代碰撞检测，两者必须分别实现和验证。

## 5. 别人的仿真调试为何通常分层

ROS 2 Control 官方的 Mock Components 文档把 `GenericSystem` 定位为理想化的 command-to-state 镜像，用于离线测试 controllers、broadcasters、launch 与 MoveIt 集成，从而减少在真实硬件上的调试时间。[Mock Components 官方文档](https://docs.ros.org/en/ros2_packages/humble/api/hardware_interface/doc/mock_components_userdoc.html)

这反映出常见且稳健的分层方式：

- 协议/接口层：测试 CAN 编解码、ID、enable、timeout、错误路由和 diagnostics，不依赖接触物理。
- 控制器层：测试 lifecycle、interface claim、controller switching、首帧与超时状态机。
- 动力学层：在 Gazebo 中验证重力、惯性、关节阻尼、限位、地面碰撞、自碰撞与保持漂移。
- 场景/规划层：再接 MoveIt 或上层工程验证轨迹和系统集成。

对于当前仓库，“一条 launch 启动完整后端”与分层调试并不矛盾：默认启动完整链路，同时通过 launch 参数提供可控的调试配置即可。

## 6. 对当前工程的建议决策

1. 保留现有 `fault_latching_enabled`，不要规定锁存后必须重启 launch。
2. 增加显式的仿真抱闸状态机；锁存关闭时也要求重新 enable 或 reset/rearm，避免旧帧自动释放。
3. 优先验证“停用 effort controller + `hold_joints`”是否满足保持和切换瞬态；若漂移或冲击不可接受，再实现独立且参数化的制动保持 controller。
4. 启用模型级自碰撞，并依靠 SDFormat 对直接相邻 link 的规范排除；只在实测发现特定 collision geometry 问题时做更细粒度调整。
5. diagnostics 至少发布 `brake_engaged`、`awaiting_first_command`、`motor_control_active`、`fault_latched` 和最近一次状态转换原因。
6. 自动测试覆盖状态机和 controller 切换；Gazebo 验收覆盖重力保持、首帧释放、150 ms 超时重新抱闸、自碰撞和反复恢复。

## 版本背景

本仓库使用 Gazebo Fortress 和 `gz_ros2_control`，不是 Gazebo Classic 的
`gazebo_ros2_control`。复杂的制动状态机仍应由本仓库显式设计并测试，不能只依赖
`hold_joints` 的默认行为。
