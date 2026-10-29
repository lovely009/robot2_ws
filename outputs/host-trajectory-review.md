# 上位机轨迹执行：独立静态审查

**最终状态（2026-10-29 针对性复核）：原四项问题均已闭合；在四项修复及直接影响范围内，未发现新增实质阻塞。** 本结论为当前源码的有界静态复核，不代表真机验收通过。复核记录见文末。

以下保留初审历史；其中行号与缺陷描述对应修复前源码，不代表当前状态。

初审日期：2026-10-29。初审结论：**存在软件阻塞问题：1 项 P1、3 项 P2。** 以下均依据当时源码推导，没有以测试通过代替审查，也没有声称真机通过。

## 发现

### 1. [P1] 尚未 armed 时的当前故障不会闭锁，恢复授权会继续已有目标

位置：[micro_ros_system.cpp:695](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:695)，相关分支为 457–471、639–650、726–737 行。

- **触发**：插件以未武装状态激活，JTC 已占用接口并持有待执行 goal；随后完整、新鲜的控制状态报告非零 `unified_fault_code` / `vendor_fault_code`。也可以在激活时就带有该故障。运动反馈仍有限且新鲜。
- **原因与影响**：`on_control()` 不闭锁当前故障；`controls_armed()` 把故障与普通“尚未授权”都折为 `false`，而 `read()` / `write()` 仅在原先 `armed_ == true` 时调用 `fail()`。因此上述故障期间 JTC 可以继续运行；故障清除并通过既有 policy 显式授权后，插件直接将 `armed_` 置真，发送 JTC 当前目标，无需重新激活插件或重新提交 goal。显式设备授权不等于批准恢复故障前的 host goal，违反故障后须重新激活的边界。
- **最小修复**：将当前故障判定与正常未授权等待分开；对配置电机的明确当前故障，无论是否曾 armed 都闭锁；激活时存在当前故障则拒绝成功。继续允许健康、未授权状态等待，保留不依赖 `motion_enabled` 的首目标和 `ENABLING` 许可；不要重新使用历史 `last_failure_reason` 作当前故障。
- **针对性验证**：未 armed、已有 goal 时注入当前故障，再恢复健康并授权，必须持续拒绝非空命令，直至显式重新激活。

### 2. [P2] 把 MCU 聚合状态要求成与配置完全相等，使合法配置子集无法工作

位置：[micro_ros_system.cpp:342](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:342)、[micro_ros_system.cpp:382](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:382)，以及 349、390 行。

- **触发**：host 配置控制电机 `{1}`，同一 MCU 还登记电机 `{2}`，或运行中发现一个非本控制组电机。配置的关节、传动组和 URDF 本身完整合法。
- **原因与影响**：桥按全部 `status_.motors` 组装运动反馈（`bridge_node.cpp:251`），并发布全部控制状态（`agent_messages.cpp:436–471`）；插件却同时检查数组长度相等、拒绝所有非配置 ID。因而 `{1,2}` 的正常聚合反馈会被整体拒收，启动失败，或使已经运行的配置组闭锁。现有插件夹具和单电机集成测试均使 MCU 集合恰好等于配置集合，未覆盖此情形。
- **最小修复**：保留聚合消息的 ID 合法性、唯一性检查，按配置 ID 提取本组；要求每个配置 ID 恰好出现一次，仅对本组执行数值、来源、时效与武装检查。不要因非本组设备存在而扩大命令控制范围。补充“配置子集 + 聚合状态”验证，并继续拒绝本组缺失/重复 ID。

### 3. [P2] 激活等待期间已置 faulted，仍可能返回 SUCCESS

位置：[micro_ros_system.cpp:635](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:635)，成功出口为 639–650 行。

- **触发**：一秒激活等待中，先收到一帧无效运动消息，例如旧固件轨迹尚未停止时 `target_source == TRAJECTORY`，或接管前残留的过期反馈；`on_motion()` 调用 `fail()`。随后同一次激活等待中收到完整、新鲜的合法控制和运动消息。
- **原因与影响**：`fail()` 将 `faulted_` 置真，后续好消息不清除它；`on_activate()` 却只检查最新反馈，没有检查该闭锁标志，仍将 `active_` 置真并返回 `SUCCESS`。紧接着第一次 `read()` 在 678 行必定返回 `ERROR`，接口启动也会在 525 行失败。出现“激活成功但无法启动/立即报错”的不一致，尤其影响需要 STOP_ALL 接管旧轨迹的启动过程。
- **最小修复**：激活过程中发生闭锁后，明确返回 `ERROR`，不得再从成功出口退出；若将某些启动期旧帧定义为正常等待，则应明确采用非闭锁拒收，而不能留下 `SUCCESS && faulted_`。不建议在成功出口无条件清除故障。
- **针对性验证**：同一次激活先坏帧、后好帧，检查生命周期返回值与下一次 read/接口启动一致。

### 4. [P2] 测量年龄漏算桥到插件的排队与传输延迟

位置：[micro_ros_system.cpp:363](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:363)、[micro_ros_system.cpp:435](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:435)。

- **触发**：MCU 快照产生时测量已经老化 60 ms；10 ms 后桥收到，组装器将发布的 `state_age_ms` 修正为 70 ms（`messages.cpp:194–200`）；该消息再经过 50 ms 的 DDS/回调排队才到插件，此时真实测量年龄是 120 ms。
- **原因与影响**：插件分别检查源快照年龄 60 ms 和消息携带的测量年龄 70 ms，两者均未超过 100 ms，于是接受；接收处的 `elapsed_ms(now, now)` 恒为零，后续仅从插件回调时刻继续累加。桥发布至插件接收的 50 ms 永远丢失，导致 JTC 使用超过规定 100 ms 的实测值进行误差/完成判断，插件还可能继续发布目标。MCU 的独立反馈门仍能阻止对应过期测量下的运动，不能据此宣称插件反馈判断正确。
- **最小修复**：在插件接收时建立包含传输时间的保守测量年龄/失效截止，再用单调钟累加；在现有消息下，可保守地把非负源时间差计入 `state_age_ms`（注意桥已累计部分延迟，会保守重复计算），或明确传递能消除重复计算的年龄基准。不能继续仅分别比较两种年龄。补充“测量已有年龄 + 消息排队延迟相加超过 100 ms”验证。

## 覆盖与边界

- 已读指定 brief、review.diff、需求与使用说明、实施计划；直接检查当前插件头/源、全部新插件测试、host launch 与配置测试、robot launch/测试、新真实 JTC 集成测试，以及桥/固件相关差异与必要调用方。历史 `last_failure_reason` 修复按当前源码评估，没有将已修问题重复列出。
- 已静态核对 session/seq、真实测量年龄与分组、模型双向映射、URDF 位置/速度边界、全组 P/V command-mode-switch、首目标/ENABLING、来源隔离、启动只读模式、接管 STOP_ALL 与后续状态确认、多桥检测、取消/停用和子进程清理。除上述发现外，在本次有界范围内未确认其它实质阻塞。
- 固件共同反馈门已接入 snapshot、trajectory、hold 和 `motor_control_target_valid()`；物理 TX 出队前调用后者。SDO 消费路径不刷新运动时间戳，过期服务清目标、逐电机授权和 policy。没有发现此次改动放宽现有 30/100 ms 命令门的证据。
- 新测试覆盖正常传动/控制器占用、首目标、ENABLING、历史失败原因、会话切换和过期反馈，但没有覆盖上述四个反例。集成模拟设备立即跟随命令、只有一个配置电机、测量年龄恒为零，不能证明这些边界或设备物理行为。
- 本次没有运行 ROS/固件测试或构建，没有派 agent，没有修改产品、硬件或运行服务。用户提供的 352 ROS、33 固件及真实 JTC 四阶段通过信息未在本次复跑。
- 未穷尽旧测试和手册；原有八文件 WIP、其它任务输出及延期的工作台迁移不纳入缺陷。真机使能、USB 最坏时延、CAN bus-off、重力关节停机和烧录均未验证，仍需专项授权验收。

## 2026-10-29 针对性复核（最终）

范围仅为当前 `micro_ros_system.hpp/.cpp` 中上述四项修复、直接影响的缓存/生命周期/读写路径，以及 `test_micro_ros_system.cpp` 的七项新增回归。没有重新开展全量审查，没有运行测试、构建、派 agent 或操作硬件；本轮只更新本报告。

1. **当前故障闭锁：已闭合。** [micro_ros_system.cpp:416](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:416) 对配置组的非零 unified/vendor 故障及 `LIFECYCLE_ERROR_PROCESSING` 直接 `fail()`，不再依赖曾经 armed。后续健康消息不能清除 `faulted_`，read 和已占用接口的 write 继续拒绝执行；重新激活才重置闭锁。正常未授权等待、首目标不依赖 `motion_enabled`、`ENABLING` 许可及忽略历史 `last_failure_reason` 的行为保留。对应回归：`UnarmedCurrentFaultCannotResumePendingCommand`、`ActivationRejectsPresentDeviceFault`。
2. **配置子集与聚合反馈：已闭合。** [micro_ros_system.cpp:347](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:347) 和 [micro_ros_system.cpp:409](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:409) 先对整帧检查 ID 范围及唯一性，再要求配置集合完整包含于聚合集合。数值、来源、测量年龄及当前故障检查仅作用于配置组；缓存前移除非本组状态，因此武装检查不会被无关设备阻塞，输出也未扩大到非配置电机。对应回归：`AggregatedFeedbackControlsOnlyConfiguredDevices`、`AggregatesStillRequireUniqueAndCompleteConfiguredIds`。控制帧的缺失/重复分支亦已直接核对，后一个测试本身主要覆盖运动帧。
3. **激活返回值与闭锁一致性：已闭合。** [micro_ros_system.cpp:669](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:669) 在 `spin_some()` 后立即检查 `faulted_` 并返回 `ERROR`，成功出口之前没有清除故障的操作；同轮回调中即使后来收到好帧也不能掩盖已发生的闭锁。对应回归：`ActivationCannotSucceedAfterRejectedStartupFrame`。旧轨迹等坏帧导致本次激活失败、需要显式重试，是此次采用的闭锁语义。
4. **测量年龄包含源延迟：已闭合。** [micro_ros_system.cpp:369](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:369) 先验证源戳，再把非负源时间差向上取整为 ms，与 `state_age_ms` 相加检查；[micro_ros_system.cpp:468](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp:468) 使用缓存的 `motion_source_delay_ms_` 加上接收后的单调钟间隔持续老化。无效消息不会成为有效缓存；新有效反馈覆盖对应延迟，旧缓存失效后不会参与状态更新。初审中的“70 ms 测量年龄 + 50 ms 排队”反例现在会被拒收。对应回归：`MeasuredAgeIncludesQueuedSourceDelay`、`QueuedMeasuredAgeContinuesAgingAfterReception`。

**复核结论：四项修复充分，原 1 项 P1、3 项 P2 均关闭；本次限定范围内无新增实质阻塞。** 已直接阅读新增测试的输入与断言，没有用通过数量代替源码判断。主代理提供的当前源 22 项插件测试、359 项 ROS 测试及真实 JTC 集成 1 项完成结果，本轮未独立复跑。

保守重复计入桥已累计的部分源延迟符合本轮明确约定，会收紧有效反馈窗口，可能提前闭锁；本轮未据此调整协议或放宽 100 ms 门。剩余台架边界仍是实际 USB/调度最坏延迟、设备物理使能、CAN bus-off 和重力关节停止效果；硬件未操作，这些结果仍不能由软件复核或模拟 JTC 集成替代。
