# robot_dynamics

独立的 Pinocchio 依赖边界，为后续机器人运动学/动力学功能提供 C++ 包入口。

## 当前范围

当前版本只验证 Pinocchio 可由 ROS 2 Humble / ament_cmake 找到、编译和链接：

- 不提供 ROS 节点、topic、service 或参数。
- 不读取 URDF。
- 不参与 ros2_control、`motor_drive`、`motor_sim` 或 Gazebo 的运行路径。
- 不实现正逆运动学、逆动力学、重力补偿或轨迹前馈。

## 安装前提

```bash
source /opt/ros/humble/setup.bash
sudo -A apt install ros-humble-pinocchio
```

## 构建与测试

```bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_dynamics
source install/setup.bash
colcon test --packages-select robot_dynamics
colcon test-result --verbose
```

测试只构造一个空的 `pinocchio::Model`，不启动 Gazebo，也不连接真实硬件。
