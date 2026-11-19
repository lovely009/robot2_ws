# 关节目标跟随与重力补偿

## 连续目标

网页滑块通过 `/control/joint_target` 提交关节名与位置。`workbench_control` 校验模型、反馈和目标范围，在跟随控制器激活后转发目标。

`follow_controller` 从上一周期的位置与速度继续生成指令。速度上限来自 URDF，加速度上限为速度上限除以 0.5 s。连续拖动保持指令连续，松手后到达最后目标并保持。

首次激活从实测位置开始。停止、断连、模型切换和重新激活会清除旧目标。动作播放由 JTC 执行，控制器切换时先停止已有运动。

## 重力补偿

3D 底栏开关调用 `/bench/start_gravity_feedforward` 和 `/bench/stop_gravity_feedforward`，由管理器设置 `gravity_controller` 参数。开启后的补偿系数为 1.0。

跟随控制器和 JTC 使用位置、速度接口，`gravity_controller` 使用力矩接口。管理器同时激活运动所需的控制器，任何一个异常退出都会停止该控制会话。

计算需要完整模型姿态、新鲜反馈和已配置的传动参数。页面等待参数回执和状态回读，显示请求失败或超时。网页重连后读取实际状态，开关值按当前主机会话处理。

停止运动后保留会话内的重力配置，页面显示“已开启 · 待运动”。下一次显式运动启动后显示“补偿中”。

## 测试

- `test_position_profile.cpp` 验证快速反向、速度和加速度边界、到位保持。
- `test_joint_follow_controller.cpp` 验证激活、目标接续、停止和重新激活。
- `test_workbench_control.py` 验证入口校验、控制器切换和迟到消息。
- 前端重力补偿与浏览器测试验证回执、超时、断连和模型变化。

控制器参数及轨迹容差见 [motor_hardware_interface](../../ros2_ws/src/motor_hardware_interface/README.md)。
