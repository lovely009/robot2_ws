# MCU 反馈适配与离线控制器测试

实机电机统一由 micro-ROS MCU 控制。请使用 `robot_bringup/robot.launch.py`；主机不直接打开电机 CAN 或串口设备。

pluginlib 只导出 `MicroRosSystem`；实机通过 `robot_bringup/robot.launch.py` 启动。构建后需重新加载当前工作区；不要用旧安装目录启动设备。

本包继续提供 `firmware_topic_bridge.py`、控制状态跟踪和传动适配。实机动作播放由主机 JTC 插值，滑块由连续跟随控制器生成目标，MCU 执行周期目标；下面的模拟入口用于离线验证。

实机 `host_trajectory.launch.py` 的路径位置误差为 `0.1`，到位误差为 `0.05`（转动关节单位 rad，移动关节单位 m），终点额外等待 `1 s`。位置限位以关节面板提交并经主机回读确认的范围为准，URDF 仅提供默认范围；速度和力矩仍按模型校验。取消回复为空时会等待旧目标终态，确认超时仍停止。

滑块通过 `/control/joint_target` (`sensor_msgs/JointState`) 提交关节名和位置，不提交到达时间或 JTC goal。`follow_controller` 从上一周期指令位置、速度连续接续，沿用 URDF 速度上限，加速度上限为速度上限 / 0.5 s；松手后到达并保持最后目标。主机以 50 Hz 更新，下行/上行仍为 50/50 Hz、MCU 电机执行仍为 200 Hz。停止、客户端断连或切换控制器会丢弃旧目标，重新激活从实测位置开始。

工作台整条轨迹最多 8192 个展开后的电机路点，每个传输分段最多 256 点；全部分段在 2 秒内收齐并通过时间戳、限位校验后才提交一个 JTC goal。缺段、总点数超限或停止会丢弃未提交轨迹，不执行部分内容。

动作库 `MotorTrajectory` 沿用消息约定：位置与时间决定执行曲线，文件中的速度仅用于诊断，仍须为有限值。主机完成传动映射后，在关节空间按位置和不等时间间隔生成单调 Hermite 切矢，首末速度为零，随后仍检查整段位置、速度限位；原 JSON 不变。该关节空间插值不保证与旧固件在任意耦合电机空间的插值等价。原生 `JointTrajectory` 和手动 P/V/E 继续使用显式速度；轨迹非零 effort 仍拒绝。

原始采样对比动作可显式设置 `interpolation: "linear"`，经 `/control/trajectory_raw` 使用 JTC 原生位置线性插值，保留位置和时刻，不生成平滑切矢。该模式仍校验每段两端位置和割线速度限值，保留共用的组包、取消、停止与 MCU 保护；路点处速度可以不连续。未指定该字段的现有动作保持默认单调插值。

原始线性模式每条 JTC goal 单独使用 `0.2 rad` 路径位置误差，普通播放仍继承默认 `0.1 rad`。末点到位 `0.05 rad`、额外等待 `1 s`、停止速度判据及关节位置/速度限位保持原值。

修改上位机位置限位须先停止主机关节控制。新范围同时提交到规划与硬件接口，收到硬件确认后才更新状态；允许覆盖 URDF 默认位置范围，不改模型文件或固件限位。提交失败继续显示原生效范围，不自动恢复浏览器旧草稿；通信结果未确认时，须显式重新提交限位后才能控制。测量接口故障后输出无效值，页面不会把旧测量刷新成实时反馈。

## 官方模拟硬件与标准轨迹 Action

```bash
ros2 launch motor_hardware_interface mock_control.launch.py
```

默认加载官方 `mock_components/GenericSystem`、`joint_state_broadcaster` 和 `joint_trajectory_controller/JointTrajectoryController`，提供 `joint_trajectory_controller/follow_joint_trajectory`。默认模型是无物理动力学的合成关节树，只验证控制器接口，不能用于判断真实机械运动或碰撞。

`motor_config_file` 仅用于读取共享传动模型中的关节清单。该配置中的设备路径不会传给模拟插件；launch 不声明 CAN/串口覆盖参数。可以使用 `model_xacro` 指定更完整的模拟描述，但渲染完成后必须满足：仅一个 `GenericSystem`，控制关节与配置完全一致，且每个控制关节存在于 URDF。任何真实或未知硬件插件都会在创建进程前被拒绝，没有开启实机插件的开关。

可选 `control_mode:=position` 使用标准 `ForwardCommandController`。同一次启动只激活一个命令控制器。`controller_manager_name:=/my_robot/controller_manager` 同时作用于 manager、控制器、反馈和 Action 的命名空间。

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_components --controller-manager /controller_manager
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
```

## 轨迹容差

模拟 JTC 的启动参数按实际关节清单注入：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `trajectory_path_tolerance` | `0.2` | 路径位置误差 rad，移动关节为 m |
| `trajectory_goal_tolerance` | `0.02` | 终点位置误差 |
| `trajectory_goal_time` | `1.0` | 终点额外等待时间 s |
| `trajectory_stopped_velocity_tolerance` | `0.01` | 终点静止速度容差 |

这些值必须有限且大于零；目标必须覆盖完整关节清单。实机使用上文的主机 JTC 容差，MCU 保留断流、反馈和本地保护。

型号库的唯一源文件位于 `robot_description/config/motor.yaml`。本包的旧路径仅是源码符号链接及由同一源安装的兼容文件。

测试以真实官方 JTC 和 GenericSystem 验证资源认领、目标反馈、成功、取消以及退出，同时验证真实插件和旧直连入口在启动前拒绝；不会打开 CAN、串口或 Agent。
