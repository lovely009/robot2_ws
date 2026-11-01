# 受监督的主机标准轨迹执行

`robot_supervisor` 加载 `HostMotionActionAdapter`，在
`/robot/follow_joint_trajectory` 提供标准 `FollowJointTrajectory` Action，
将经过授权和模型校验的完整计划交给主机 JTC。执行链为：

`主机 JTC/MicroRosSystem → supervisor → micro_ros_bridge → MCU → 电机`。

主机 JTC 插值，MCU 执行周期目标。本包不打开 CAN/串口、不自动使能或续租。
已删除未被生产入口调用的旧 MCU Action 适配器与轨迹编译器。

## 调用顺序

1. 显式申请控制权、武装并提交使能策略，等待 MCU 授权确认与新鲜反馈。
2. 生成 UUID，调用 `/robot/authorize_motion`，绑定当前租约、epoch、session、
   sequence 和目标 UUID；许可短期有效且只能消费一次。
3. 用相同 UUID 发送标准 Action。SDK 的 `follow_joint_trajectory` 封装授权与提交，
   不会代替调用者申请控制、使能或续租。
4. 调用者在运行期间续租并驱动 executor；取消使用标准 Action cancel。

模型校验在工作线程执行，周期转发继续由 supervisor 执行器处理；校验结束后重新
核验控制权。同一时刻仅接受一个目标。新的所有权清退旧控制器；同一有效所有权
在目标结束后可以继续提交，不强制重启 JTC。

## 轨迹与停止

计划通过共享 `WorkbenchModel` 检查关节、限位、反馈新鲜度及起点。支持位置和
速度路点；加速度、非零 effort 和预约起点被拒绝。轨迹由主机保留，不受旧 MCU
缓冲上传的 256 点容量限制，也不表示 MCU 支持执行中流式续段。

反馈和结果来自 JTC；取消请求转发给 JTC，保持驱动使能策略独立。显式停止、
控制权丢失、反馈或目标过期由 supervisor 与 MCU 授权期限处理。恢复需要显式
重新申请和武装，不会自动恢复旧目标。

## 验证

`test/test_host_validation.py` 覆盖校验期间执行器继续服务、并发目标拒绝、校验后
重新检查授权、同所有权连续目标及准备阶段取消。完整 ROS/JTC/原生固件运行
用例位于 `robot_bringup/test/sil`、`robot_supervisor/test/test_wire_runtime.py` 和
`robot_bringup/test/test_host_trajectory_integration.py`；当前仍有失败，不能视为
完整运行验收通过。测试中的模型与标定为虚构 fixture，不是硬件验收。
