# 固件与上位机职责拆分评估

日期：2026-10-29。依据当前工作区源码和 ROS 2 Humble 官方文档。本文记录改造前的静态评估，当时未实施改造、未改变协议与运行配置。

后续已获授权开展软件实施；本文保留改造前的评估记录。当前方案和验收边界见 [实施设计](/home/ly/robot2_ws/docs/architecture/host-trajectory-execution.md) 与 [使用说明](/home/ly/robot2_ws/docs/architecture/host-trajectory-usage.md)。

## 判断

建议采用“上位机管理任务与轨迹，MCU 管理设备执行与本地保护”的分工，保留现有内核并分阶段迁移。当前已经具备独立于 ROS 的电机核心、命令邮箱、周期任务与命令有效期机制；不需要按文章重新搭建一套相同框架。

主要收益是统一 ROS 任务语义、减少 MCU 的轨迹事务和缓冲管理。主要代价是主机采样、调度与 USB 链路的连续供给变得更重要；MCU 以更高频率重发旧目标不能补偿主机停止计算。

## 上位机实际链路

- `robot.launch.py` 默认没有加载 JTC；固件模型与标准关节轨迹入口是条件启动项。[launch](/home/ly/robot2_ws/ros2_ws/src/robot_bringup/launch/robot.launch.py:81)
- 当前 Web 动作库生成带时间的电机轨迹，交给固件执行。[调用方](/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/panels/actionLibrary.js:205)
- 已有 `JointTrajectory` 入口仍将消息转换成 `MotorTrajectory` 上传固件，不是主机上的 JTC 执行器。[适配器](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/scripts/firmware_topic_bridge.py:111)
- 已有独立 ros2_control/CAN 硬件入口，其现行控制器是 100 Hz 的 ForwardCommandController，硬件声明 position command 与 position/velocity/effort state。这不能直接当成“JTC 已通过 micro-ROS 控制 MCU”的实现。[控制器配置](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/config/driver_controllers.yaml:6)、[硬件声明](/home/ly/robot2_ws/ros2_ws/src/motor_hardware_interface/urdf/canfd_hardware.xacro:18)

目标接线应为：主机 action/任务入口 → 主机轨迹控制器 → 保留原意图有效性的命令适配 → micro-ROS Agent → MCU 命令邮箱与电机核心 → CAN 电机。轨迹结果和误差判断回到主机；MCU 继续独立报告设备状态与故障。ros2_control 控制循环本身负责 read/update/write，这与该分工相符。[官方架构](https://control.ros.org/humble/doc/getting_started/getting_started.html)

### 当前重发流与控制器活性必须分开

`command_streamer` 默认 100 Hz，接收目标后保留最后设定值；每拍重发时生成新的时间戳与序号，`setpoint_at` 只用于诊断显示，没有参与停发条件。[接收](/home/ly/robot2_ws/ros2_ws/src/motor_command/motor_command/command_streamer.py:158)、[重发](/home/ly/robot2_ws/ros2_ws/src/motor_command/motor_command/command_streamer.py:186)

主机桥按该帧时间计算剩余寿命，再生成 MCU 执行截止。[源码](/home/ly/robot2_ws/ros2_ws/src/micro_ros_bridge/src/bridge_node.cpp:237)

因此，原始目标停止更新时，只要 streamer、相关客户端条件及下游门禁仍允许，旧设定值会继续以新帧身份到达 MCU。**当前 30 ms 租约只能证明该命令流仍在更新，不能证明产生运动目标的控制器仍在运行。**

持续保持一次手动位置设定可以是明确的产品语义，不能仅凭“位置值相同”就判定控制器停滞。但若将这条保持流直接用于上位机轨迹外环，转发器会掩盖控制器停止计算。迁移时应区分“操作者明确要求保持”与“控制器必须持续生成有效更新”，给后者建立不会被转发重发自动延长的有效期/活性约束；不必为了这个需求盲目增加多套 ID、状态或配置。

官方 Humble JTC 提供轨迹插值、action 抢占与误差检查，适合作为候选主机执行器；项目仍需打通命令与反馈接线，并明确 MIT 增益和停止行为。[JTC 官方文档](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

## 固件核查与改造建议

### 现有基础可以复用

- `motor_control` 的公开边界已明确不依赖 ROS/HAL/FreeRTOS，也不认识 joint_name；无需重新设计一个相同的设备核心。[源码](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/include/motor_control.h:4)
- 命令回调负责会话、帧形状、序号与截止时间等检查后提交实时任务，命令邮箱采用双缓冲覆盖待消费目标。[回调](/home/ly/robot2_ws/micro_ros_firmware/src/motor_bridge.cpp:132)、[邮箱](/home/ly/robot2_ws/micro_ros_firmware/lib/control_mailbox/src/control_command_mailbox.c:40)
- 实时任务按 1 ms 服务 RX、运行时保护和 TX，另按配置周期装载目标；控制默认值为 200 Hz，可设 50–1000 Hz 且要求整数毫秒周期。这里只确认调度实现与配置范围，不代表最大负载下达到该频率。[任务](/home/ly/robot2_ws/micro_ros_firmware/src/motor_rt_task.cpp:630)、[默认值](/home/ly/robot2_ws/micro_ros_firmware/lib/firmware_config/include/firmware_config.h:5)、[校验](/home/ly/robot2_ws/micro_ros_firmware/lib/firmware_config/src/firmware_settings.c:26)
- 通信租约超过 30 ms 触发软停，超过 100 ms 或断连/校时失效触发撤权；重复保持目标不改写原执行截止。[租约](/home/ly/robot2_ws/micro_ros_firmware/lib/agent_protocol/src/agent_protocol.c:63)、[目标保持](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:1058)
- 反馈年龄从真实运动帧接收 tick 派生，并独立于发布快照的时间；重新上传缓存不会刷新测量年龄。[年龄](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:1435)、[上传](/home/ly/robot2_ws/micro_ros_firmware/src/motor_bridge.cpp:249)
- Flash 擦写已有独立低优先级 storage 任务，无需为这次职责调整重写持久化。[源码](/home/ly/robot2_ws/micro_ros_firmware/src/storage_task.cpp:1)

### 真正应迁移的内容

当前 MCU 有整条轨迹锁存、单调 Hermite 插值、轨迹归属，以及 LOAD/COMMIT/CANCEL/ACK 事务和终态保持；它承担的是轨迹执行器职责，不是透明 CAN 转发。若最终选择主机流式目标，应迁移这些轨迹职责，保留设备内核中的命令校验、周期保持、模式与型号管理、授权、反馈、协议与安全调度。[插值](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:801)、[事务](/home/ly/robot2_ws/micro_ros_firmware/lib/trajectory_transaction/src/trajectory_transaction.cpp:291)

先保持当前协议可用，验证主机执行路径；同一电机在任一时刻只能有一个运动命令所有者。主机路径验收后，再同步删除主机桥、消息定义、固件中的轨迹协议与相应 UI 依赖，并处理协议版本兼容。文章的“三类消息”可以理解为职责分类，无需为了数量立即重写现有 V3 协议。

MCU 尚有原生 rclc 参数服务器。是否撤掉它是独立取舍；把业务 service 留在主机，不要求把设备参数调试接口一并删除。[参数服务器](/home/ly/robot2_ws/micro_ros_firmware/src/firmware_parameters.cpp:86)

### 迁移前必须处理的差异

1. **把真实运动反馈年龄保护放到共有设备路径。** 当前轨迹健康检查拒绝无真实测量、反馈超过 100 ms 等情况；普通快照的运动门禁没有相同的独立反馈年龄条件。通用离线检查使用任意归属帧的 last_rx_tick_ms，而 SDO 应答也会刷新该字段。因此应先补齐并测试共同门禁，再退出轨迹执行路径，避免持续收到参数回复时掩盖运动反馈停更。[轨迹检查](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:920)、[快照共有门禁](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:354)、[离线判断](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:1235)、[SDO接收](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control_discovery.c:90)
2. **使能需要区分软件意图与设备确认。** 当前目标路径在使能帧成功入队后设置 motion_enabled，再等待厂家整定延时。这个变量本身不是电机物理使能的反馈证据。若设备协议能报告实际使能，应据此确认；不能报告的情况应保留“未确认”语义。[实现](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:494)
3. **区分各层频率。** 运动上传支持最高 100 Hz，CERB SYNC 调度仍是 20 ms；不能据此宣称真实电机测量也达 100 Hz。MCU 周期重发是零阶保持，不会自动补出主机未计算的轨迹中间点。[上传配置](/home/ly/robot2_ws/micro_ros_firmware/lib/firmware_config/src/firmware_settings.c:14)、[SYNC](/home/ly/robot2_ws/micro_ros_firmware/src/motor_rt_task.cpp:391)、[保持](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:1058)
4. **保留原命令有效期与防重放。** 本地单调钟适合判断超时，但仅以接收时刻重新给每个包 30 ms 有效期，会重新放行链路中迟到的旧指令。当前绝对截止贯穿接收、邮箱和执行的机制应保留，不能通过放宽时钟门禁掩盖映射或传输问题。[有效期](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:619)、[门禁](/home/ly/robot2_ws/micro_ros_firmware/lib/agent_protocol/src/agent_control_gate.c:38)
5. **明确完成、取消、失联的机械行为。** 当前 CERB 软停为保持使能的阻尼目标，轨迹完成最终调用这条路径；官方 Humble JTC 在成功后继续最后目标，超差中止后保持当前位置。迁移可能改变重力关节受力与末点行为。[CERB软停](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_protocol/src/cerb_adapter.c:295)、[当前终点](/home/ly/robot2_ws/micro_ros_firmware/lib/trajectory_transaction/src/trajectory_transaction.cpp:336)、[JTC官方文档](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
6. **确认插值和控制模式。** 当前 MCU 有单调 Hermite 及端点零速；JTC 默认 spline 对仅位置路点采用线性插值，对位置和速度采用三次插值。相同位置路点不能保证迁移后曲线完全一致。MIT 的 position/velocity/effort/kp/kd 应明确对应驱动器已有控制项，不能把标准 JTC 的接口配置等同于任意 MIT 模式。[当前插值](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:801)、[MIT下发](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_control/src/motor_control.c:385)、[JTC插值文档](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
7. **CAN 故障后的停机不能只依赖再发一帧。** 当前 CERB 适配器注明看门狗/限位对象、关闭保护的含义，以及实际看门狗行为仍需台架验证。保留 MCU 有本地保护与调度价值；当前不建议因职责简化顺便取消 MCU 或改传输层。[适配器约束](/home/ly/robot2_ws/micro_ros_firmware/lib/motor_protocol/src/cerb_adapter.c:42)

### 建议验收顺序

- 先在宿主测试中覆盖源控制器停更但转发器仍运行、旧序号/旧会话/过期命令、真实反馈停更但 SDO 仍回复，以及完成/取消/停用后的命令所有权。
- 验证主机轨迹采样到 MCU 接受的端到端间隔、时延、抖动与时钟映射；100 Hz 名义频率不代表一定满足 30 ms 租约，不能以增大阈值替代根因验证。
- 经专项授权后在台架验证使能实际确认、短时断流、断连撤权、CAN异常、重连不自动复动及重力关节停机行为；通过后切换默认执行路径，再删除旧轨迹代码和协议。

本次仅静态核查源码与官方文档，未运行测试、改产品代码、重启服务、操作 CAN、烧录或驱动电机。上述差异是改造设计与验收的输入，不代表已实施修复或已获得真机安全结论。
