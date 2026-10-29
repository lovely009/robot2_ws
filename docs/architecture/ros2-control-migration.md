# 工作台控制入口迁移到 ros2_control

2026-10-29。用户要求完成迁移并移除 motor_command。已有未提交的主机轨迹实现作为本次基础，原始工作区快照在 /tmp/robot2-control-migration-20261029/baseline。

## 约束与方案

- 周期运动命令只从 controller_manager → controller update → MicroRosSystem::write → micro_ros_bridge → MCU 产生。不增加独立重发目标的定时器，不修改固件、不操作运行服务、RAM、Flash 或真机。
- 工作台使用低频 ROS 客户端适配节点 workbench_control，安装在 motor_hardware_interface。它校验/转换现有动作文件和电机调试输入，调用标准 JTC action 或标准 MultiInterfaceForwardCommandController；它不发布 /motor/host_command_array。
- 保留位置/速度/力矩手动输入：每关节一个 manual_controller_<index>，一次严格切换共同占用全部 position/velocity/effort 接口。轨迹使用 trajectory_controller 占用 position/velocity；可选 gravity_controller 在同一 ros2_control 周期计算重力并占用 effort。手动与轨迹模式严格切换，禁止双写；停止后不自动恢复控制器。
- 前端改用 /control/setpoint、/control/trajectory、/control/status、/control/trajectory_state；标准关节入口 /arm_controller/joint_trajectory 由客户端提交 JTC action。沿用 MotorTrajectory 作为动作文件的低频兼容格式，不再向 MCU 上传轨迹。任意长度动作继续串行分段，不增加总点数上限。
- 控制客户端参数 config_file、urdf_file、model_name。/control/status JSON 提供 model、joint_names、mode、reason、reject、output_topic=/motor/host_command_array、trajectory_output_topic=/host_control/trajectory_controller/follow_joint_trajectory、gravity_enabled、effort_gain。同时发布 /robot_transmission/status，保留模型身份和反馈门禁。
- 取消轨迹采用 JTC cancel（保持位置）；显式“全部软停”和 /client_count 确认归零时停用运动控制器，硬件发送空快照，由 MCU 保留本地软停/撤权。客户端断开不恢复旧目标；节点重启不恢复旧输入。无 /client_count 的纯 ROS action 客户端仍可使用独立主机入口。
- 参数/使能/故障处理仍用 /motor/*；不自动使能设备，不放宽既有时效和保护。新入口只接受对应当前模型、完整传动组、新鲜状态和有限且符合限位的输入。
- robot.launch.py 使用主机桥；选定模型的台架会话启动对应控制栈和 TF，显式配置路径继续可用。模型映射错误须明确报告，不回退到旧命令流。

## 实施与验证接口

1. C++：MicroRosSystem 的 P/V/E 接口、控制器资源占用和空快照停止；robot_dynamics 重力控制器的 update、参数校验、有限输出和 effort 清零。
2. Python：低频输入 → 实际 JTC action / controller manager switch / manual commands；覆盖传动换算、拒绝无效/过期输入、action 拒绝/取消/失败、显式停止和客户端断开。
3. 工作台：关节滑块、原始命令、动作分段、取消/软停、真实控制器状态与重力开关；前端单测、构建与模拟浏览器检查。
4. 编排：模型会话/显式 launch 只启动一套控制器和一个桥；移除 motor_command 包、依赖、安装产物和旧重力接线。保留无关 WIP。
5. 隔离 ROS_DOMAIN_ID + ROS_LOCALHOST_ONLY 的真实 controller manager/JTC 与 Topic 假设备集成；不连接 Agent、不打开 CAN/串口。

独立审查一次；阻塞问题修复后做针对性复核。软件测试不能替代 USB 调度最大间隔、真实使能、重力负载停机等台架验收。

## 进度

- 已完成只读范围审计和原始工作区快照。后续检查以本文件约束为准。

- 干净环境构建 12 包通过；clean install 不注册 motor_command。原固件与 FirmwareParameters.vue WIP 逐项比对保持。
- 真实 CM/JTC + Topic 假设备集成 2/2：标准 action、手动 P/V/E、完成/取消、重力开关和力矩、停止/重连不复动、控制循环停止后断流均通过。
- 编排/启动/模型会话 236 项通过；硬件新增停止门禁后 194 项通过；重力与原硬件此前 225 项通过。
- Web Python 97 通过，1 个既有静态规范测试失败（deviceParams 的 a11y 标签 ID）；修改前快照同样失败，未删除可访问性关联。

- 六个受影响包的 colcon 检查已执行：四个控制核心包无失败；robot_sim 有 6 个原有模型 README/dist 缺失失败，Web 有 1 个原有静态 ID 规则失败，均用修改前测试快照复现。未跳过或删除这些检查。

- 独立审查发现的三项阻塞已修复：未受理 goal/取消终态进入停止屏障；手动与动作共用运行模型身份门禁；动作分窗不再发送重复 t=0，单点窗保持正时长。对应专项回归通过，前端最终 86/86；最终真实 CM/JTC 集成 2/2，通过前端生产构建。
- 完整浏览器检查执行 53 项，51 通过；另两项固件参数页面用例未通过，产品组件未由本迁移修改。按用户功能优先要求不扩展处理该页面。
- 软件修改与隔离安装已完成，普通运行服务和原安装环境未切换；未烧录、未写设备 RAM/Flash、未执行真机动作。

- 最终收尾：手动命令/限位/滑块/停止相关浏览器检查通过；模拟控制状态改为周期上报，保留产品新鲜度门禁。补齐硬件停止 Topic 的 std_msgs 直接构建依赖并成功增量构建。所有运行服务保持原状。
