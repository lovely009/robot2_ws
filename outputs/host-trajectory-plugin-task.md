# 上位机轨迹迁移：MicroRosSystem 插件任务报告

## 结果

已在 `motor_hardware_interface` 中实现 `motor_hardware_interface/MicroRosSystem` ros2_control 插件。主机执行边界为：

- 发布 `/motor/host_command_array`，QoS reliable depth 1。
- 订阅 `/motor/motion_states`（best_effort）与 `/motor/control_states`（reliable）。
- 使用 `robot_transmission::Model::load(config_file)` 做 actuator 与 ROS 关节的双向映射。
- 暴露 position/velocity command 与 position/velocity/effort state。
- 内部 rclcpp Node 使用 `use_global_arguments(false)`，避免 `ros2_control_node` 的全局重映射污染内部节点。
- 无独立重发 timer 或 worker thread；每次 `write` 最多产生一份新 timestamp/seq 的 command 快照。
- 未授权、故障、停用和未武装时发布空快照；只有完整 policy ACTIVE、健康、在线且 execution 允许时才可发布非空目标。
- `EXECUTION_ENABLING` 不触发撤权闭锁，首次目标允许 policy ACTIVE 但 `motion_enabled=false`，不把 `motion_enabled` 冒充为本地物理确认。
- 硬件 active 后，只要已有 control session，则无论是否曾经 armed，session 变化都会闭锁并返回 ERROR；只能经显式 deactivate/activate 建立新基线。
- motion 实测年龄严格限制为 100 ms；control state 流按现有桥约束允许 300 ms，10 Hz status 不会错误否决持续新鲜的 motion。
- command interface 非空 min/max 必须在 `on_init` 通过 finite 与 min<=max 校验；`write` 对 joint 坐标越位置/速度边界整包停止并闭锁，不做静默 clamp。
- `prepare_command_mode_switch`/`perform_command_mode_switch` 要求完整覆盖本模型的全部 P/V 接口；controller stop 后 writer 关闭且只发空快照，restart 从当拍新鲜实测初始化，不重放旧目标。

## 红灯到绿灯

1. `ZPKG-RED-01`：初始插件桩的公共 `on_init` 失败，`export_command_interfaces` 返回 0 项，无法满足 ros2_control 公共接口。新增激活、import/export、command 发布和 state 映射测试后转为绿灯。
2. 首次目标死锁回归：真实 JTC 链路证明“永不发首目标”会形成死锁。公开测试 `PolicyActiveWithoutMotionEnabledCanEmitFirstTarget` 曾因空 command 快照失败；移除错误的 `motion_enabled` 授权门限后通过。
3. `EXECUTION_ENABLING` 回归：首目标后进入 ENABLING 时曾因子健康判定被误撤权并闭锁；公开测试 `EnablingAfterFirstTargetDoesNotRevokeAuthority` 曾因空 targets 失败，纳入 ENABLING 的允许执行态后通过。
4. mode-switch 回归：restart 场景曾因 fixture 把旧目标误当新实测目标而失败；修正 fixture 后，公开测试锁定 controller stop 后继续 read/write 只产生空快照，restart 不重放旧目标。
5. 未武装 session 回归：spec 审计发现旧判断只在 `armed_` 时检查 session 变化。新增 `SessionChangeWhileActiveButUnarmedLatchesUntilReactivation` 后，现实现以 `active_ && control_session_ != 0` 强制闭锁，并验证显式重新激活后恢复。

## 最终测试

在隔离构建 `/tmp/robot2-host-trajectory-plugin-green`、`ROS_DOMAIN_ID=178` 下，使用 `micro_ros_agent/install` 与 `ros2_ws/install` 环境，并将 `LD_LIBRARY_PATH` 指向当前构建目录，避免旧安装 `.so` 抢先加载：

```text
cmake --build ... --target test_micro_ros_system -j2
Built target motor_hardware_interface
Built target test_micro_ros_system

ctest --output-on-failure -R '^test_micro_ros_system$'
100% tests passed, 0 tests failed out of 1

test_micro_ros_system --gtest_color=no
Running 14 tests from 1 test suite
[ PASSED ] 14 tests
```

覆盖包括：配置与 ID/限位校验、激活实测初始化、控制器 claim/mode switch、首次目标、ENABLING、每周期 command timestamp/seq、joint 越限整包停止、撤权闭锁、旧 session motion 拒绝、active 未 armed 时 session 闭锁、100 ms motion 过期、10 Hz control stream 与新鲜 motion、deactivate 空快照。

主代理已另行验证真实 JTC -> bridge 集成四阶段全部通过：完成、取消、JTC 停用且 manager 继续循环、控制循环停止。插件本身不执行硬件动作，也不宣称物理停止或物理确认。

## 文件

- `ros2_ws/src/motor_hardware_interface/include/motor_hardware_interface/micro_ros_system.hpp`
- `ros2_ws/src/motor_hardware_interface/src/micro_ros_system.cpp`
- `ros2_ws/src/motor_hardware_interface/test/test_micro_ros_system.cpp`
- `ros2_ws/src/motor_hardware_interface/CMakeLists.txt`
- `ros2_ws/src/motor_hardware_interface/package.xml`
- `ros2_ws/src/motor_hardware_interface/hardware_plugins.xml`

## 重要局限

- 本轮未操作硬件、CAN 或 live domain；测试 I/O 边界为真实 ROS topics 与直接 lifecycle/公共 SystemInterface API。
- 插件发布空快照仅表示撤销主机非空租约；没有新的物理确认协议，不能据 topic 快照宣称执行器已在物理上停止。
- 没有独立重发 timer/worker thread。若桥或节点循环停止，不会由插件旁路续租或重发。
- `src/micro_ros_system.cpp` 当前 755 行，高于期望的 200--350 行。按本轮“不要扩大其余重构”的要求未在收尾阶段拆分。
- 未修改 launch、bridge、固件或既有 CAN-FD 插件；未提交 git。
