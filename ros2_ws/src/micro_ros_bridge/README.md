# micro_ros_bridge

本包的 `micro_ros_bridge_node` 是主机 facade 消息与 micro-ROS 线上契约之间的边界节点。线上话题名由 [`WireContract.msg`](../can_bridge_msgs/msg/WireContract.msg) 定义；固件侧接线与发布节奏见 [`micro_ros_firmware/README.md`](../../../micro_ros_firmware/README.md)。**节点目前没有接入任何 launch**，构建本包不会切换现有话题流。

| 方向 | 主机话题 | 线上话题 | 线上消息 | QoS |
|---|---|---|---|---|
| 主机 → 线上 | `/motor/command_array` | `/micro_ros/do/low_cmd` | `LowCommand` | stream |
| 主机 → 线上 | `/motor/param_set` | `/micro_ros/do/config` | `ConfigRequest` | reliable |
| 主机 → 线上 | `/motor/system_command` | `/micro_ros/do/config` | `ConfigRequest` | reliable |
| 主机 → 线上 | `/motor/policy_array` | `/micro_ros/do/policy` | `MotorPolicyArray` | reliable |
| 主机 → 线上 | `/motor/trajectory` | `/micro_ros/do/trajectory` | `MotorTrajectory` | reliable |
| 线上 → 主机 | `/motor/motion_states` | `/micro_ros/up/low_state` | `LowState` | stream |
| 线上 → 主机 | `/motor/status_array` | `/micro_ros/up/motor_status` | `MotorStatus` | reliable |
| 线上 → 主机 | `/motor/trajectory_state` | `/micro_ros/up/motor_status` | `MotorStatus` | reliable |
| 线上 → 主机 | `/motor/system_status` | `/micro_ros/up/motor_status` | `MotorStatus` | reliable |
| 线上 → 主机 | `/motor/diagnostics` | `/micro_ros/up/motor_status` | `MotorStatus` | reliable |
| 线上 → 主机 | `/motor/topology` | `/micro_ros/up/topology` | `MotorTopology` | reliable |
| 线上 → 主机 | `/motor/events` | `/micro_ros/up/events` | `MotorEvent` | reliable |
| 线上 → 主机 | `/motor/device_param_report` | `/micro_ros/up/device_param` | `DeviceParamReport` | reliable |
| 线上 → 主机 | `/imu/data` | `/micro_ros/up/imu/data` | `sensor_msgs/Imu` | stream |
| 线上 → 主机 | `/imu/temp` | `/micro_ros/up/imu/temp` | `std_msgs/Float32` | stream |

`stream` 是高频、下一拍可替代的消息，使用 best-effort、depth 1；`reliable` 是低频声明、配置或事件，使用 reliable、depth 10。`/motor/command_array` 主机订阅端单独使用 reliable、depth 1，线上 `low_cmd` 发布端为 stream。每条透传链路的订阅和发布两端使用相同的 QoS 类别。

命令和状态的定点换算为 `q = clamp(round(x × 32767 / scale), -32767, 32767)`，反算为 `x = q × scale / 32767`。位置量程为 ±12.5 rad，速度为 ±64 rad/s，力矩为 ±64 N·m；`-32768` 是缺失测量哨兵。超量程命令钳位并报告饱和，NaN/Inf 命令整帧拒绝。低状态帧的电机 ID 必须在 `[0, 63]`，否则整帧拒绝。

在收到首条有效 `MotorStatus` 前，schema 门禁允许下行命令。之后只接受与本机 `SCHEMA_VERSION` 相同的有效状态帧；版本为 0 或其他值时拒绝命令与配置，畸形状态帧不改变门禁。
