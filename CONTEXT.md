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
| 电机拓扑配置 | 总线、电机类型和 motor-to-joint 映射；真机侧为 `motor_hardware_interface/config/motor_canfd.yaml`（单一配置源），仿真侧为 `motor_canfd_sim/config/motor_canfd.yaml`（来源于仿真模型，两侧解耦） |
| 真机参数 | `motor_hardware_interface/config/motor_canfd.yaml` 中经标定的数值（如反馈换算扭矩常数、安全时序），只服务驱动侧/真机 |
| 仿真参数 | `motor_canfd_sim/config` 中描述模拟电机与物理行为的假设数值（扭矩常数、限幅、模式增益），只影响仿真，不得直接作为真机标定值 |
| 逻辑总线 | 与接口名解耦的总线标识，例如 `left_arm` |
| SocketCAN 接口 | Linux 网络接口，例如 `can0` 或 `vcan0` |
| 总线覆盖 | `bus_override.<逻辑总线名>`；启动时替换逻辑总线对应的接口（HIL 时把 vcan 重定向到真实 can 接口） |
| 协议策略 | 实现某一电机家族帧格式、ID 规则和控制语义的 `MotorProtocol` |
| position joints 单一来源 | `driver_controllers.yaml` 不手抄 joints；launch 用 `motor_joints_from_config()` 从电机拓扑配置生成并注入 position controller，避免关节顺序多处副本 |
| 接口参数 | `interface_<逻辑总线名>`：launch 按配置 `buses` 段生成的参数（例如 `interface_left_arm`），经 `bus_override.*` 传给硬件 xacro |

## 控制链路术语

- **驱动侧 controller**：`/driver/controller_manager` 下的 position controller，输出电机目标。
- **Gazebo 侧 controller**：`/controller_manager` 下的 effort controller，驱动仿真关节。
- **完整回环**：position command 经硬件接口、底层驱动、vcan、电机侧模拟器和 Gazebo 后，再以 CAN-FD 反馈返回底层驱动。
- **真机 bring-up**：由 `motor_hardware_interface` 提供的真机启动入口（launch 与控制器配置），不依赖仿真栈；模型描述由模型包提供。
- **fail-fast 停机**：激活阶段最多重试 3 次（每次 1 s 窗口），仍失败或运行期反馈超时时驱动侧立即终止（安全语义），人工诊断后重跑；回环在模拟器退出时随之中止。
- **HIL**：Hardware-in-the-Loop。允许模拟器访问物理 CAN 接口的专项测试模式，不是常规仿真配置。
- **家族仿真处理器**：`SimMotorHandler`，电机侧模拟器中与驱动侧 `MotorProtocol` 一一对应的家族策略；新增电机家族时在模拟器侧注册 handler，节点本身无家族分支。
- **动作库限位**：`motion_library.JOINT_LIMITS`，与 URDF joint limit 一致的双写副本；调整限位时需同步两处，由 `test_config_topology.py` 的跨源断言逐对校验。

## 命名原则

- 使用“仿真”表示完整系统行为，使用“电机侧模拟器”表示具体进程。
- 使用“电机拓扑”表示总线、电机和关节关系；“参数”仅表示增益、超时等数值。
- 区分逻辑总线和 SocketCAN 接口，避免将 `left_arm` 与 `can0` 混称为 CAN 口。
- 协议名、类名、函数名、ROS 2 API 和 Topic 保留英文原名。
