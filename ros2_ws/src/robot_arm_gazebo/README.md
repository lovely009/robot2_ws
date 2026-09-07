# robot_arm_gazebo

肘部关节机械臂（`zhoubuguanjie0`）的 Ignition Gazebo 仿真集成包。模型由 SolidWorks
URDF 导出器生成（4 个 revolute 关节 + base_link，STL 网格），已移植为 ROS 2
Humble ament 包，仿真后端为 Ignition（`ros_gz_sim` + `gz_ros2_control`）。

## 目录结构

```
robot_arm_gazebo/
├── config/
│   ├── controllers.yaml        # controller_manager / 关节状态广播 / 位置控制器
│   └── display.rviz            # RViz 预览配置
├── launch/
│   ├── display.launch.py       # RViz 模型预览（无 Gazebo）
│   └── gazebo.launch.py        # Ignition 仿真 + 控制器
├── meshes/                     # SolidWorks 导出 STL
└── urdf/
    ├── zhoubuguanjie0.urdf         # 原始 SolidWorks 导出（mesh 路径已改指本包）
    └── zhoubuguanjie0.urdf.xacro   # 仿真薄壳：注入 ros2_control 与 gz 插件
```

## 构建与使用

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash

# RViz 预览（滑杆拖动关节）
ros2 launch robot_arm_gazebo display.launch.py

# Ignition 仿真（空世界 + 生成模型 + 控制器）
ros2 launch robot_arm_gazebo gazebo.launch.py
ros2 launch robot_arm_gazebo gazebo.launch.py world_file:=/path/to/world.sdf
```

## 控制机械臂

`gazebo.launch.py` 启动后自动激活 `joint_state_broadcaster` 与
`arm_position_controller`（forward position）：

```bash
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 1.0, 0.2]}"

ros2 topic echo /joint_states
```

关节顺序：`Joint_Link1`（0~1.57 rad）、`Joint_Link2`（±1.57）、
`Joint_Link3`（-0.09~2.09）、`Joint_Link4`（±1.57）。

## 移植说明

- mesh 路径已从 `package://zhoubuguanjie0/` 改写为 `package://robot_arm_gazebo/`。
- SolidWorks 导出的关节限位 `effort="0" velocity="0"` 已补为
  `effort="10.0" Nm` / `velocity="1.0" rad/s`（名义占位值，按真实执行器参数调整）。
- ros2_control 硬件接口为 `gz_ros2_control/GazeboSimSystem`（position 命令 +
  position/velocity 状态），控制器参数见 `config/controllers.yaml`。
