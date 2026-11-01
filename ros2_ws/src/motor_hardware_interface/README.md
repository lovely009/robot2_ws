# MCU 反馈适配与离线控制器测试

实机电机统一由 micro-ROS MCU 控制。请使用 `robot_bringup/managed_robot.launch.py`；主机不直接打开电机 CAN 或串口设备。

旧 `hardware.launch.py` 在创建任何节点前拒绝启动，并给出迁移提示。已删除停用的 `CanFdSystem` 实现、专属配置解析与 xacro；pluginlib 只导出 `MicroRosSystem`。构建后需重新加载当前工作区；不要用旧安装目录启动设备。

本包继续提供 `firmware_topic_bridge.py`、控制状态跟踪和传动适配。标准实机轨迹入口属于受监督的 MCU 执行链；下面的 JTC 入口用于离线验证。

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

这些值必须有限且大于零；目标必须覆盖完整关节清单。实机执行的容差和约束以 MCU 托管入口为准。

型号库的唯一源文件位于 `robot_description/config/motor.yaml`。本包的旧路径仅是源码符号链接及由同一源安装的兼容文件。

测试以真实官方 JTC 和 GenericSystem 验证资源认领、目标反馈、成功、取消以及退出，同时验证真实插件和旧直连入口在启动前拒绝；不会打开 CAN、串口或 Agent。
