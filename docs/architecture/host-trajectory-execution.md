# 上位机轨迹执行与 MCU 设备执行边界（首阶段记录）

后续工作台迁移已由 [ros2-control-migration.md](ros2-control-migration.md) 接续；当前使用方式见 [host-trajectory-usage.md](host-trajectory-usage.md)。下文保留首阶段约束与验证记录，其中默认 firmware、独立 command_streamer 和工作台尚未迁移的描述已被后续迁移替代。

2026-10-29。用户已授权实施此前职责评估中的分阶段迁移。本次软件交付不修改运行中的服务、设备 RAM、Flash 或运动状态。

## 目标与约束

- 上位机使用现有 Humble joint_trajectory_controller（JTC）提供 FollowJointTrajectory action、插值、取消、误差检查；MCU 继续负责设备协议、周期调度、授权、状态和本地超时保护。
- 保留 V3 消息和现行 30 ms 软停/100 ms 撤权边界，不放宽时钟或命令时效门禁，不自动使能、不自动修改 policy 或保护参数。
- 新路线由 robot.launch.py 的 trajectory_execution=host 显式选择，默认 firmware。两条路线对同一 MCU 不同时拥有命令权限。
- 第一版提供标准关节 action 接口和可测试的完整 MCU Topic 链路；现有工作台 MotorTrajectory 入口继续使用默认 firmware 路线。host 模式明确拒绝固件轨迹上传，用户通过 JTC action 操作。工作台 action 客户端改造、默认切换和删除 MCU 轨迹模块留到台架验收之后，避免把未验证路径投入真机。
- 复用 robot_transmission::Model 进行执行器与关节双向换算。首版支持 position+velocity command，position/velocity/effort state；MIT kp/kd 仍由现有 policy 管理，不增加自研控制器，不宣称支持未接线的力矩前馈轨迹。
- 当前共享工作区已有八个文件的未提交修改，保持其内容，不批量覆盖、不提交用户 WIP。

## 方案

1. 在 motor_hardware_interface 增加 MicroRosSystem 硬件插件。它仅通过已有主机消息接入 MCU，不打开 CAN/串口。控制循环 read 取得真实反馈、write 直接发布到 /motor/host_command_array；没有独立的目标重发定时器。通过 command mode switch 跟踪全部 P/V 接口的控制器占用，控制器停用后即使 manager 仍循环 write 也只发空快照；重新占用从实测初始化，拒绝部分接口占用。JTC/控制循环停滞后即停止产生新帧，由桥与 MCU 既有截止处理。显式手动保持仍由原 command_streamer 提供，二者不混用。
2. 插件校验完整映射、唯一且合法 motor_id、有限数值、反馈实际年龄与控制状态。聚合状态可包含其它设备，但配置组必须完整；只对配置组映射、检查故障和输出目标。使用单调钟跟踪接收间隔，把非负源时间差保守加入消息中的 state_age_ms，再持续累加本地时间；桥已累计的部分延迟可能重复计入，无效消息不得刷新旧测量的有效期。激活从新鲜实测位置初始化命令，不继承旧命令；不自动武装 policy。未授权阶段只发空快照建立通信租约；当前故障无论是否曾授权都闭锁，授权撤销/会话切换/反馈过期同样要求重新激活，避免自动复动。激活等待中已经闭锁时返回失败。
3. 桥增加启动时只读 execution 模式选择：firmware 使用原命令源；host 只接受 /motor/host_command_array，拒绝 MotorTrajectory 上传。检测遗留固件轨迹或多个线上命令发布者时不发送新增运动。停止和撤权保持现有独立安全路径。
4. MCU 共同设备路径增加真实运动反馈年龄检查（100 ms），覆盖快照、保持重发与 TX 前复检。SDO 或其它回复不能替代真实运动反馈；触发后清除该电机目标与授权，保持不可自动复动，恢复需显式授权与新鲜目标。整条固件轨迹仍按原规则中止。
5. 新增 host_trajectory.launch.py，从显式提供的已展开真实 URDF 和映射配置生成关节接口与 JTC 参数，保留真实关节类型并传播位置/速度边界，拒绝缺失、mimic 或无效限位。复用系统安装的 controller manager/JTC/JSB；命名空间 /host_control，action 为 /host_control/trajectory_controller/follow_joint_trajectory。robot.launch.py 只启动一个桥，并按执行模式选择是否组合该 launch。

## 语义与验收

- JTC 成功后保持末点，取消/超差时遵循 JTC 保持行为；MCU 命令超时仍走本地软停/撤权。与原 CERB 阻尼软停的差异写入使用说明，不伪称等价。
- 验证边界：MCU 公共 command/service/TX 接口；插件 lifecycle、command/state interfaces 与真实 ROS Topic；主机桥到 AgentCommand/AgentTrajectoryRequest Topic；launch 配置与真实 JTC action。
- 核心 TDD：SDO 活跃但运动反馈过期；超过期限的目标不能再产生 P1；失效后新目标不自动复动；控制循环停更后没有独立续租；反馈/会话无效不得执行；传动组换算与接口顺序；停用发送空快照；host 模式不接受旧命令源和固件轨迹；真实 JTC action 在独立 ROS_DOMAIN_ID 下连接模拟 Topic 设备并执行/取消。
- 构建固件（不上传），运行固件宿主测试和受影响 ROS 包测试，独立审查一次，修复阻塞问题后只做有依据的复核。
- 软件测试不能证明 USB 最大负载时延、真实使能或重力关节停机效果。真机台架验收、切换线上服务、烧录与运动均按专项授权执行；在此之前保留旧协议和默认路线。
