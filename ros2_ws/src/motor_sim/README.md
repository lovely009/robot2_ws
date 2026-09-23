# motor_sim

共享电机仿真库（与机器人无关），提供协议、传输、传动、plant 模型、帧路由与锁存安全功能。

## 定位

作为 `ros2_ws` 仿真栈的核心组件，为机器人侧提供与协议兼容的电机节点。
它保持机器人无关，可单独接入外部 `/joint_states`；`robot_arm_gazebo` 的
`simulation.launch.py` 负责组合 Gazebo、CAN 驱动端和本节点，组成端到端机械臂仿真。
实现不依赖真实电机硬件。

## 核心组件

### 协议层 (`protocol/`)

| 模块 | 职责 |
|---|---|
| `MotorProtocol` | 协议策略基类，定义帧格式与编解码接口 |
| `VaxorProtocol` | VAXOR 25/30mm 腕部电机协议（反馈电流 A） |
| `CerbProtocol` | CERB 协议（扩展支持） |

### 传输层 (`transport/`)

| 模块 | 职责 |
|---|---|
| `SocketCanTransport` | Linux SocketCAN 传输后端 |
| `SerialTransport` | USB 串口与 SLCAN CAN 适配器后端 |
| `FrameRecord` | 帧记录与回放 |

### 核心层 (`core/`)

| 模块 | 职责 |
|---|---|
| `Config` | 配置解析（与 `motor_drive` 兼容格式） |
| `DriveLifecycle` | 驱动生命周期管理（激活/运行/停机） |
| `FrameRouter` | 帧路由（多总线/多电机分发） |
| `Transmission` | 传动模型（减速比/效率/背隙） |
| `PlantModel` | 电机动力学模型（电流→扭矩→位置） |
| `SafetyMonitor` | 安全监控（超限检测/锁存保护） |

### ROS 层 (`ros/`)

| 模块 | 职责 |
|---|---|
| `MotorSimNode` | ROS 2 节点封装，提供仿真服务与话题 |

## 依赖关系

```
motor_sim
├── motor_protocol（协议编解码）
├── motor_transport（传输抽象）
├── motor_sim_core（核心仿真逻辑）
│   ├── motor_protocol
│   └── yaml-cpp（配置解析）
└── motor_sim_node（ROS 节点）
    ├── motor_sim_core
└── ROS 2（rclcpp, sensor_msgs, std_msgs, std_srvs）
```

## Gazebo 组合仿真

在 `robot_arm_gazebo` 包中可用单一入口启动完整回路：各模型目录的
`models/<模型名>/config/simulation_topology.yaml` 维护参与仿真的电机映射和 plant 参数；电机 ID、协议、
总线、类型和增益直接读取 `motor_hardware_interface` 的权威配置：

```bash
ros2 launch robot_arm_gazebo simulation.launch.py                 # vcan 默认闭环
ros2 launch robot_arm_gazebo simulation.launch.py transport:=socketcan \
  driver_interface:=can0 sim_interface:=can1
ros2 launch robot_arm_gazebo simulation.launch.py transport:=slcan \
  driver_tty:=/dev/serial/by-id/DRIVER sim_tty:=/dev/serial/by-id/SIM
```

`vcan` 与 `socketcan` 都使用 `SocketCanTransport`；`vcan` 是无需物理 CAN 适配器的
纯软件回环。SocketCAN host 闭环可用两个接口分开连接驱动与模拟器；SLCAN host 闭环
需要两只适配器接同一物理总线。单个 SocketCAN 接口或 CANable 均可通过
`controller_mode:=firmware` 接入 STM32 固件 HIL。模型映射、命令方式和注意事项见
[`robot_arm_gazebo/README.md`](../robot_arm_gazebo/README.md)。

## 与真机栈对比

| 特性 | motor_sim（仿真） | motor_drive（真机） |
|---|---|---|
| 协议 | 相同帧格式 | 相同帧格式 |
| 传输 | 模拟传输 | 真实 CAN/串口 |
| Plant | 数学模型 | 真实电机动力学 |
| 安全 | 软件模拟 | 硬件+软件 |
| 依赖 | ROS 2 | 无 ROS 依赖 |

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_sim
colcon test --packages-select motor_sim && colcon test-result --verbose
```

## 扩展指南

### 新增协议

1. 继承 `MotorProtocol` 实现新协议类
2. 在 `MotorProtocolRegistry` 注册
3. 在 `motor.yaml` 型号库声明新协议参数
4. 补充测试

### 新增传输

1. 继承 `Transport` 实现新传输类
2. 在 `TransportRegistry` 注册
3. 在 YAML 配置中声明新传输类型
4. 补充测试

## 测试覆盖

- 协议编解码测试（字段级）
- 传输模拟测试（帧级）
- 帧路由测试（多总线分发）
- Plant 模型测试（动力学验证）
- 安全监控测试（超限场景）

## 注意事项

1. **与机器人无关**：本库不包含任何机器人特定配置或拓扑
2. **接口兼容**：与 `motor_drive` 保持相同协议帧格式
3. **配置格式**：使用相同的 `motor.yaml` / `motor_driver.yaml` 格式
4. **测试隔离**：测试不启动 Gazebo 或真实硬件
