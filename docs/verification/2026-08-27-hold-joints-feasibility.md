# Gazebo `hold_joints` 抱闸可行性验证

日期：2026-08-27

环境：ROS 2 Humble，Gazebo Sim 6.18.0，`dev-sim` 分支，SocketCAN `vcan0`

## 验证目的

确认重力保持开启时，停用 `effort_controller` 释放六轴 effort command interface 后，
`gz_ros2_control` 的 `<hold_joints>true</hold_joints>` 能作为仿真抱闸基础。该结果只验证
Gazebo 仿真机制，不代表 UR5e 厂家制动器性能或真机安全等级。

## 启动与切换

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo gazebo.launch.py \
  transport_mode:=socket_can arm_port:=vcan0 fault_latching_enabled:=false

ros2 service call /robot_arm/controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{deactivate_controllers: [effort_controller], strictness: 2, activate_asap: true, timeout: {sec: 2, nanosec: 0}}"
```

切换服务返回 `ok=True`。控制器状态为：

```text
joint_state_broadcaster  active
effort_controller       inactive
```

## 十秒漂移结果

停用 controller 后采集的起止关节角如下，单位均为 rad：

| 关节 | 起始值 | 10 秒后 | 绝对漂移 |
|---|---:|---:|---:|
| `shoulder_pan_joint` | 0.759975167238 | 0.759975094064 | 0.000000073175 |
| `shoulder_lift_joint` | 0.243720058428 | 0.243720058428 | 0.000000000000 |
| `elbow_joint` | -3.141592653942 | -3.141592654319 | 0.000000000376 |
| `wrist_1_joint` | 2.040551044946 | 2.040557924888 | 0.000006879942 |
| `wrist_2_joint` | -1.073364124370 | -1.073355225769 | 0.000008898601 |
| `wrist_3_joint` | 0.903442682749 | 0.903439606453 | 0.000003076297 |

六轴最大漂移为 `0.000008898601 rad`，小于设计门槛 `0.01 rad`，且未观察到持续振荡
或模型飞散。因此 `hold_joints` 可用于后续“两阶段 gate + controller 切换”仿真抱闸设计。

## 观察边界

本次现有 launch 会先激活 `effort_controller`，motor node 在未 enable 时输出零 effort，
所以停用 controller 前机械臂已经受重力塌落。本验证只证明在停用 controller 后能稳定保持
当时姿态；后续实施必须让 effort controller 从启动阶段保持 inactive，才能避免初始塌落。
