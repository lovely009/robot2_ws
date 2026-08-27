# 真实抱闸与启动时序验收

日期：2026-08-27；环境：ROS 2 Humble、Gazebo Fortress、`dev-sim`。

## 自动验收

```text
colcon build --symlink-install
Summary: 2 packages finished

colcon test && colcon test-result --verbose
Summary: 127 tests, 0 errors, 0 failures, 0 skipped

colcon list --names-only
motor_sim
robot_arm_gazebo
```

测试覆盖暂停启动编排、三个 controller 的加载顺序、STRICT 双向互斥切换、控制 gate、
五态驱动生命周期、最终 SDF 自碰撞开关和六轴 effort ABI。

## Gazebo 真实运行

统一 launch 实际运行确认：Gazebo 暂停加载模型，`joint_state_broadcaster`、
`effort_controller`、`brake_controller` 配置完成后，先提交抱闸激活请求，再恢复物理；
随后状态为 `brake_engaged`，外部 effort controller 为 inactive，闭环抱闸为 active。

关闭模型级自碰撞以隔离控制问题时，原始初始姿态在连续 5 秒采样间的最大关节漂移约
`2.3e-5 rad`，未再出现启动后整体折叠。开启自碰撞时机械臂同样不再塌落，但当前 UR
碰撞网格会在腕部产生约 `0.03 rad` 的接触偏置，因此碰撞与控制增益应分开调试。

## 边界

这里的 PID、惯量、限幅和碰撞结果仅是仿名义行为，不是 UR5e 厂家规格，也不能用于
推导真机断电下坠距离或人员安全结论。完整 vcan 运动必须持续提供六轴使能和新鲜命令；
单轴 `cansend` 仅验证协议收发，不会释放整机抱闸。
