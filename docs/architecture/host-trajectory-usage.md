# 工作台 ros2_control 路线

工作台运动统一由主机 ros2_control 执行，`motor_command` 已删除。软件修改和隔离验证不等于运行服务已切换，未烧录、未执行真机动作。

## 接线

`工作台低频请求 → workbench_control → JTC / 标准手动控制器 → MicroRosSystem::write → /motor/host_command_array → micro_ros_bridge → MCU → CAN`。

- 轨迹插值、误差检查和完成结果来自 JTC；action 为 `/host_control/trajectory_controller/follow_joint_trajectory`。
- 电机 P/V/E 调试通过每关节一个标准 MultiInterfaceForwardCommandController，控制器名为 `manual_controller_<index>`，顺序来自传动模型的 joint_names。手动与轨迹控制器由严格资源切换互斥。
- 重力补偿由 `robot_dynamics/GravityCompensationController` 在 ros2_control 更新周期内计算并写 effort，不再改道到外部命令发布者。
- 参数、使能和故障接口仍在 `/motor/*`。MCU 继续处理设备协议、反馈年龄、命令超时和本地保护。
- workbench_control 仅转换/校验请求、调用 action/切换服务和发布状态，没有重发运动目标的定时器。

## 启动与模型

默认 `robot.launch.py` 启动一个 host 模式的主机桥、Web 服务和台架管理器。工作台选定模型且 Agent 已运行后，台架管理器启动对应 ros2_control/TF 会话。运动控制器默认加载为 inactive，等待新的操作请求；不会自动使能电机或恢复旧目标。

显式提供经过核对的配置也可以启动：

```bash
ros2 launch robot_bringup robot.launch.py \
  host_motor_config_file:=/absolute/path/to/motors.yaml \
  host_urdf_file:=/absolute/path/to/expanded_robot.urdf \
  firmware_model:=model_name
```

两个路径必须成对提供。`firmware_model` 保留为初始模型身份参数；`trajectory_execution` 仅接受 `host`，旧 `firmware_joint_control` 不再提供。单独启动主机控制栈用于标准 ROS action 客户端时，可显式指定 `start_controllers:=true`，台架验收和部署仍遵循专项授权。

配置中的裸 motor_id 必须在 MCU 全局唯一（1..64，最多 48 台）。URDF 须展开且包含全部配置活动关节、合法 P/V/E 边界。重复 ID、缺失限位或映射不一致直接拒绝，不回退到旧运动路径。没有完整配置组反馈或授权时不能运动，不自动武装 policy。

HIL 仿真由仿真会话组合相同主机控制栈；电机后端 simulator_only 不启动控制器。现有直接 CAN 的独立 ros2_control 测试入口继续独立使用。

## 工作台低频接口

- `/control/setpoint`：MotorCommandArray，转换为手动控制器 P/V/E 输入。
- `/control/trajectory`：MotorTrajectory，作为动作文件的低频兼容格式收齐后提交 JTC，不再上传给 MCU；空轨迹表示取消。
- `/arm_controller/joint_trajectory`：JointTrajectory，提交 JTC action。
- `/control/stop`：std_msgs/Empty，停止全部运动控制器。
- `/control/status`：真实控制器模式、模型、反馈和错误；`/control/trajectory_state`：由 JTC 反馈及终态转换的动作状态。

动作仍可任意长度串行分段，单消息容量不构成动作总点数上限。Humble JTC 不支持轨迹 effort 字段：零 effort 被归一为空，非零力矩路点明确拒绝；手动 P/V/E 的力矩输入与实时重力补偿均有独立支持，不静默忽略力矩。

## 停止和失效语义

- JTC 完成保持末点；取消 action 后保持当前位置。取消与撤权软停是不同操作，界面须区分。
- 显式全部软停、已确认客户端数归零：同时发独立硬件停止门禁与全部失能策略，再取消待执行请求并停用运动控制器。硬件只发空快照，即使 manager 服务故障也不能持续旧目标；MCU 执行本地撤权。恢复前须显式重新使能，重连不恢复旧目标。
- 控制器失活不能继续重发旧命令；controller manager 停止后没有独立续租源。
- 反馈过期、授权撤销、设备故障或会话变化仍由硬件插件闭锁，恢复需要显式重新激活和新目标。
- 保留 30 ms 命令软停、100 ms 撤权与运动反馈保护。不能用平均 100 Hz 代替最大调度间隔验证。
- 重力默认 disabled，增益范围 0..1。不完整或 ArmModel 不支持的构型允许普通 P/V/E 路径运行，但启用重力会明确拒绝。

## 验收边界

软件验证使用隔离 ROS_DOMAIN_ID 与仅本机 DDS、真实 controller manager/JTC、Topic 假设备，不启动 Agent、不打开 CAN 或串口。部署前仍须验证 USB 调度抖动、真实授权/撤权、CAN 故障及带重力负载的停机效果。
