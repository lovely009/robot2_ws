# Robot2 工程

ROS 2 Humble 整机平台。实机只有一条电机执行链：`robot_supervisor → micro_ros_bridge → micro-ROS MCU → 电机`。主机负责模型、控制权、标准运动接口与诊断；MCU 负责轨迹插值、执行与本地授权期限。

## 功能与职责

| 包 | 职责 |
| --- | --- |
| `robot_description` | 真机、TF、动力学与仿真共享的模型、传动配置和型号库 |
| `robot_transmission` | 关节与执行器换算，包括差动机构 |
| `robot_calibration` | 机器实例验收记录、配置摘要与只读启动核验 |
| `robot_system_msgs` | 整机状态、独占租约、受控意图与一次性运动许可 |
| `robot_supervisor` | 就绪、显式武装、控制权、MCU 授权续期及故障锁存 |
| `robot_motion` | 标准 `control_msgs/action/FollowJointTrajectory`，由 MCU 执行与反馈 |
| `micro_ros_bridge` | 主机消息与 MCU 线上协议之间的编码、会话及授权校验 |
| `motor_hardware_interface` | MCU 实测关节反馈与传动映射；官方模拟硬件的离线控制器测试 |
| `robot_sim` / `motor_sim` | 仿真、协议级模拟和 HIL 支撑 |
| `robot_bringup` / `web_host` | 托管启动和网页控制客户端 |

模型已经从 `robot_sim/models` 迁移到 `robot_description/models`；仿真与真机引用同一份资产。原 `CanFdSystem` 实机插件不再导出，`hardware.launch.py` 拒绝启动；`mock_control.launch.py` 仅允许官方 `mock_components/GenericSystem`，用于验证标准控制器。

## 构建与启动

线上协议为 **v3**。`can_bridge_msgs`、主机 bridge、监督器与 MCU 固件必须配套更新。旧固件无法通过新入口的就绪核验。模型迁移也改变配置摘要，需要核对新配置并更新该台机器的标定验收记录。

首次检出时先生成网页资产（在仓库根目录执行，需要 Node.js/npm）：

```bash
cd ros2_ws/src/web_host/frontend
npm ci
npm run build
cd ../../../..
```

再构建并加载 `micro_ros_agent` 工作区中的接口与 Agent，然后构建主工作区：

```bash
cd micro_ros_agent
colcon build --packages-up-to can_bridge_msgs micro_ros_agent
source install/setup.bash
cd ../ros2_ws
colcon build --packages-up-to robot_bringup
source install/setup.bash
```

不连接设备的启动检查：

```bash
ros2 launch robot_bringup robot.launch.py \
  model:=half_body_robot robot_id:=arm-01
```

默认不启动串口 Agent、不开放轨迹、不使能电机；缺少机器实例验收记录与新鲜 MCU 状态时，仅显示未就绪。网页默认在 `http://localhost:8080`。具体配置、标定、真实设备启动与接口用法见 [robot_bringup](ros2_ws/src/robot_bringup/README.md)。

`robot.launch.py` 是默认托管入口；`managed_robot.launch.py` 默认无网页。`bench.launch.py` 保留维护和仿真管理、只读展示。`firmware_tf.launch.py` 只发布实测关节与 TF。

## ROS 2 接口

- 运动：标准 `FollowJointTrajectory`，由一次性 `AuthorizeMotion` 将目标 UUID 绑定已有控制会话；申请、武装和使能均需显式操作。
- 状态：`sensor_msgs/JointState`、TF2、`sensor_msgs/Imu` 和整机 `RobotState`。
- 诊断：`/diagnostics` 上的 `diagnostic_msgs/DiagnosticArray`。
- 验收：`launch_testing`、真实 DDS Action/bridge 回归、官方模拟硬件与固件 host CTest。
- 记录：`rosbag2` 记录公开状态、诊断与必要的下行输出，避免保存控制令牌。

首版标准轨迹接口要求完整关节清单、实测 t=0 起点、整数毫秒位置路点，最多 256 个展开后的电机路点、默认 2 秒。速度、加速度、力矩路点和预约起点暂不支持，明确拒绝而不静默改变其语义。反馈、容差、完成与取消均关联 MCU 会话和轨迹标识，详见 [robot_motion](ros2_ws/src/robot_motion/README.md)。

MCU 授权使用本地单调截止时间，覆盖缓冲轨迹；旧租约、重复心跳和旧命令不能恢复运动。网页隐藏、断线或状态过期会清除意图并停止续租；重新进入页面需要重新取得控制权。

## 验证

```bash
colcon test --packages-select robot_description motor_hardware_interface \
  micro_ros_bridge robot_system_msgs robot_calibration robot_motion \
  robot_supervisor robot_bringup web_host
colcon test-result --verbose
```

固件 host 测试位于 `micro_ros_firmware/test/host`，前端测试位于 `ros2_ws/src/web_host/frontend`。模拟测试不接设备；实机控制周期、总线负载、停止时延、抱闸与机构支撑需要另外测量。当前不包含 MoveIt 规划、碰撞规划或足式平衡控制。
