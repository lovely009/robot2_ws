# 领域术语与边界

本文统一仓库中的领域术语（`dev-real` 真机分支）。运行方式以根目录及各包 `README.md`
为准，架构取舍以 `docs/adr/` 为准。

## 分层术语

| 推荐术语 | 含义 | 避免使用 |
|---|---|---|
| 底层驱动 | `motor_drive`：SocketCAN/CAN-FD 传输、线协议和通用电机映射 | 硬件接口、ROS 驱动 |
| ROS 适配层 | `motor_hardware_interface`：`ros2_control` lifecycle、joint 映射、YAML 解析与真机 bring-up | 底层接口 |

## 配置术语

| 推荐术语 | 含义 |
|---|---|
| 电机拓扑配置 | 总线、电机类型和 motor-to-joint 映射；真机侧为 `motor_hardware_interface/config/motor_canfd.yaml`（单一配置源） |
| 真机参数 | `motor_canfd.yaml` 中经标定的数值（如反馈换算扭矩常数、命令增益、安全时序），只服务驱动侧/真机 |
| 逻辑总线 | 与接口名解耦的总线标识，例如 `left_arm` |
| SocketCAN 接口 | Linux 网络接口，例如 `can0` 或 `vcan0` |
| 总线覆盖 | `bus_override.<逻辑总线名>`；启动时替换逻辑总线对应的 SocketCAN 接口（HIL 时把 vcan 重定向到真实 can 接口） |
| 协议策略 | 实现某一电机家族帧格式、ID 规则和控制语义的 `MotorProtocol` |
| position joints 单一来源 | `driver_controllers.yaml` 不手抄 joints；launch 用 `motor_joints_from_config()` 从电机拓扑配置生成并注入 position controller，避免关节顺序多处副本 |
| 接口参数 | `interface_<逻辑总线名>`：launch 按配置 `buses` 段生成的参数（例如 `interface_left_arm`），经 `bus_override.*` 传给硬件 xacro |

## 控制链路术语

- **驱动侧 controller**：`/driver/controller_manager` 下的 position controller，输出电机目标。
- **真机 bring-up**：由 `motor_hardware_interface` 提供的真机启动入口（launch 与控制器
  配置），模型默认来自本包 `urdf/upper_body.urdf.xacro`，不依赖仿真栈。
- **fail-fast 停机**：激活阶段最多重试 3 次（每次 1 s 窗口），仍失败或运行期反馈超时时
  驱动侧立即终止（安全语义），人工诊断后重跑。

## 命名原则

- 区分逻辑总线和 SocketCAN 接口，避免将 `left_arm` 与 `can0` 混称为 CAN 口。
- 协议名、类名、函数名、ROS 2 API 和 Topic 保留英文原名。
