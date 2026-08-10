# 领域术语与边界

本文统一仓库中的领域术语。运行方式以根目录及各包 `README.md` 为准，架构取舍以 `docs/adr/` 为准。

## 分层术语

| 推荐术语 | 含义 | 避免使用 |
|---|---|---|
| 底层驱动 | `motor_canfd_drive`：SocketCAN、线协议和通用电机映射 | 硬件接口、ROS 驱动 |
| ROS 适配层 | `motor_hardware_interface`：`ros2_control` lifecycle、joint 映射和 YAML 解析 | 底层接口 |
| 电机侧模拟器 | `motor_canfd_sim`：经 vcan 模拟真实电机命令与反馈 | Python CAN 桥、仿真驱动 |
| 模型集成包 | `upper_body_canfd_gazebo`：模型、Gazebo、launch 和动作工具 | 电机模拟器 |
| 虚拟 CAN 电机设备 | 电机侧模拟器依据配置挂载到 vcan 的电机实例 | 固定双臂模拟器 |

## 配置术语

| 推荐术语 | 含义 |
|---|---|
| 电机拓扑配置 | `motor_canfd.yaml`；总线、电机类型和 motor-to-joint 映射的单一配置源 |
| 逻辑总线 | 与接口名解耦的总线标识，例如 `left_arm` |
| SocketCAN 接口 | Linux 网络接口，例如 `can0` 或 `vcan0` |
| 总线覆盖 | `bus_override.<逻辑总线名>`；启动时替换逻辑总线对应的接口 |
| 协议策略 | 实现某一电机家族帧格式、ID 规则和控制语义的 `MotorProtocol` |

## 控制链路术语

- **驱动侧 controller**：`/driver/controller_manager` 下的 position controller，输出电机目标。
- **Gazebo 侧 controller**：`/controller_manager` 下的 effort controller，驱动仿真关节。
- **完整回环**：position command 经硬件接口、底层驱动、vcan、电机侧模拟器和 Gazebo 后，再以 CAN-FD 反馈返回底层驱动。
- **HIL**：Hardware-in-the-Loop。允许模拟器访问物理 CAN 接口的专项测试模式，不是常规仿真配置。

## 命名原则

- 使用“仿真”表示完整系统行为，使用“电机侧模拟器”表示具体进程。
- 使用“电机拓扑”表示总线、电机和关节关系；“参数”仅表示增益、超时等数值。
- 区分逻辑总线和 SocketCAN 接口，避免将 `left_arm` 与 `can0` 混称为 CAN 口。
- 协议名、类名、函数名、ROS 2 API 和 Topic 保留英文原名。
