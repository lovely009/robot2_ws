# 领域术语与边界（dev）

运行方式以 README 为准，架构取舍以 docs/adr/ 为准。

## 分层

| 术语 | 含义 |
|---|---|
| 底层驱动 | `motor_drive`：传输、线协议、通用电机映射（无 ROS） |
| ROS 适配层 | `motor_hardware_interface`：ros2_control lifecycle、joint 映射、YAML 解析、真机 bring-up |
| 仿真层 | `motor_sim`（共享仿真库：协议 / 传输 / 传动 / plant / 帧路由 / 锁存安全）+ `robot_arm_gazebo`（机械臂模型库 `models/<模型名>` / launch / Gazebo 集成） |
| 运动学/动力学层 | `robot_dynamics`：独立 Pinocchio 计算边界，不属于电机驱动实时链路 |

## 配置术语

| 术语 | 含义 |
|---|---|
| 电机拓扑配置 | `config/motor_driver.yaml`（**唯一拓扑源**：数量/类型/ID/关节名/总线/传输；当前只有真机侧持有） |
| 型号库 | `config/motor.yaml`（协议运行参数 / 安全阈值，与拓扑无关、可跨机器人复用；完整规格见 `motor_drive/docs/`） |
| 逻辑总线 | 与设备路径解耦的总线标识，如 `left_arm` |
| 传输类型 | `BusConfig.transport`：`socketcan`（内核接口）或 `serial_slcan`（CANable2 串口），缺省 `socketcan` |
| 设备路径 | 可覆盖目标：socketcan 为接口名（`can0`），serial_slcan 为 tty（`/dev/ttyACM0`） |
| 总线覆盖 | `bus_override.<逻辑总线名>`（launch `interface_<总线名>`）：只覆盖设备路径，不切传输类型 |
| 传输工厂注册 | `CanTransportRegistry`：按 `transport` 创建后端；新增后端实现并注册工厂 |
| 协议策略 | `MotorProtocol`：电机家族帧格式/ID 规则/控制语义；`MotorProtocolRegistry` 注册 |
| joints 单一来源 | controller joints 与 URDF ros2_control 清单由 launch 从配置生成，不手抄 |
| 覆盖路径一致性 | `motor_config_file:=` 覆盖后，controller joints 与 URDF 清单以实际配置为准 |

## 控制链路

- 驱动侧 controller：`/driver/controller_manager` 下的 position controller。
- 真机 bring-up：`motor_hardware_interface` 提供，默认 serial_slcan 双总线。
- 仿真链路：`MotorDriver -> vcan -> motor_sim -> Gazebo effort controller`（`motor_sim` 独立回路）；
  `robot_arm_gazebo` 的 `gazebo.launch.py` 用 `gz_ros2_control` 位置控制器直接驱动模型，
  不经 vcan 也不经 `motor_drive`（两个仿真回路尚未联动）。
- fail-fast 停机：激活重试 3 次（每次 1 s 窗口）或运行期反馈超时即终止，人工诊断后重跑。

## 命名

- 区分逻辑总线、传输类型与设备路径，不混称（`left_arm` ≠ `socketcan` ≠ `/dev/ttyACM0`）。
- 协议名、类名、ROS 2 API、Topic 保留英文原名。
