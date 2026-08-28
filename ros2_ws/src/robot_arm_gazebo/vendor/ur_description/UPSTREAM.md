# UR5e 上游资产

- Repository: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- Commit: `18e6f603b3ebc2ec479fecb62d6be544b15755e9`
- License: BSD-3-Clause，见同目录 `LICENSE`

只保留 UR5e 模型所需的 xacro、参数与 mesh。未引入上游 launch、MoveIt、
driver、CI、其他机器人型号或 `ur.ros2_control.xacro`。

本地修改仅限于：package lookup 与 mesh URI 指向 `robot_arm_gazebo` 内的
`vendor/ur_description`；本地 wrapper 固定传入
`generate_ros2_control_tag:=false`，控制接口由本工程另行定义。

这些物理参数和网格仅作为 Gazebo 仿真名义数据，不代表真机性能或安全边界。
