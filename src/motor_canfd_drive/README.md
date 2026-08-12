# motor_canfd_drive

C++17 CAN-FD 电机驱动库，提供可插拔的 CAN 传输（SocketCAN / CANable2 串口）、
协议编解码、协议策略注册和动态电机映射。

## 职责边界

负责：

  - 管理 `DriverConfig` 中声明的任意数量总线和电机（每条总线可独立选择传输后端）；
- 按 `(bus, type)` 分组并调用对应`MotorProtocol`；
- 发送使能、失能和控制命令，接收并解析反馈；
- 提供 RP 与 `motor_2530` 协议实现。

不负责：

- 解析 YAML 或 ROS 参数；
- 管理 ROS 2 lifecycle、Topic、Action 或 Service；
- 定义机器人关节数量、左右臂拓扑和 controller 行为。

## 核心模型

```text
DriverConfig
  buses[]  -> 逻辑总线名 + 传输后端（socketcan / serial_slcan）
  motors[] -> motor + joint + bus + type + id + parameters
       |
       v
MotorCanfdDriver
  -> 按 (bus, type) 分组
  -> MotorProtocol
  -> CanTransport（SocketCan / SerialSlcanTransport）
```

| 组件 | 职责 |
|---|---|
| `MotorCanfdDriver` | 配置校验、总线生命周期、电机分组和统一收发 |
| `MotorProtocol` | 电机家族的使能、控制、反馈和 ID 规则 |
| `protocol.*` | 物理量与线协议字段转换 |
| `CanTransport` | 传输抽象：`send()` / `receive()`，驱动不感知底层实现 |
| `SocketCan` | Linux CAN-FD socket 传输（`can0`/`vcan0` 等） |
| `SerialSlcanTransport` | CANable2 串口传输（`/dev/ttyACM0`，slcan-FD 方言） |
| `slcan.*` | slcan 帧编解码（纯函数，可独立单测） |

RP 单条广播命令最多容纳 8 个槽位，这是 RP 线协议限制，仅由 RP 策略校验，不限制其他协议或整机电机数量。

## 传输层

`BusConfig.transport` 选择总线后端：

```cpp
// SocketCAN（默认）
BusConfig{"left_arm", "can0", "socketcan", ""};

// CANable2 串口（slcan 固件，1M 仲裁 / 5M 数据段）
BusConfig{"bench", "", "serial_slcan", "/dev/ttyACM0"};
```

`SerialSlcanTransport` 构造时自动下发 `C → S8 → Y5 → O` 配置总线；
`send()`/`receive()` 与 SocketCAN 语义一致（非阻塞、按帧编解码）。
串口传输只需 tty 写权限（dialout 或 root），不需要 SocketCAN 接口或 NET_ADMIN。

## 公共接口

公共头文件位于 `include/motor_canfd_drive/`：

- `motor_types.hpp`：驱动配置、命令与反馈数据类型；
- `motor_protocol.hpp`：协议策略接口与 registry；
- `motor_canfd_driver.hpp`：通用驱动入口；
- `can_transport.hpp`：CAN 帧结构与传输抽象；
- `serial_slcan_transport.hpp`：CANable2 串口传输；
- `slcan.hpp`：slcan 帧编解码；
- `protocol.hpp`：协议字段编解码；
- `socketcan.hpp`：SocketCAN 封装。

调用方构造 `DriverConfig` 后传入 `MotorCanfdDriver`。本工作空间由
`motor_hardware_interface::load_driver_config()` 将 YAML 转换为该配置。

## 扩展电机协议

1. 实现 `MotorProtocol`。
2. 在 `MotorProtocolRegistry` 注册工厂。
3. 在 `motor_canfd.yaml` 的 `motor_types` 和 `motors` 中声明类型与实例。
4. 增加协议编解码、边界值、错误 ID 和分组行为测试。

不得在通用发送循环、接收循环或 ROS 关节映射中增加针对具体类型的分支。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_canfd_drive
colcon test --packages-select motor_canfd_drive
colcon test-result --verbose
```

测试入口：

- `test/test_protocol.cpp`：协议字段和帧编解码；
- `test/test_driver.cpp`：配置校验、分组和驱动行为。
