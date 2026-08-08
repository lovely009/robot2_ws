# motor_canfd_drive

`motor_canfd_drive` 是一个不依赖 ROS 运行时的 C++ CAN-FD 电机底层驱动库。它负责 SocketCAN 收发、线协议编解码、电机协议策略以及通用的“总线—电机”映射。

这个包刻意不知道“机械臂”是什么，也不知道机器人有多少个关节，更不知道 ROS 2 控制器如何组织。这样做的目的，是让同一套底层驱动既能被 `ros2_control` 使用，也能被独立 C++ 调试程序或测试程序直接使用。

## 运行结构

```text
DriverConfig
  buses[]  -> 逻辑总线名 + SocketCAN 接口
  motors[] -> 电机名 + joint + bus + type + id + parameters
        |
        v
MotorCanfdDriver
        |
        +-- 按 (bus, type) 分组 --> MotorProtocol 协议策略
        |                           +-- rp
        |                           +-- motor_2530
        |                           +-- 未来新增协议
        v
     SocketCAN
```

`MotorCanfdDriver` 使用动态容器管理总线和电机，所以数量完全由 `DriverConfig` 决定，不存在“机械臂固定 7 个关节”的限制。

RP 协议本身的一条广播命令最多包含 8 个槽位，这是 **RP 协议的物理限制**，只由 RP 协议策略负责检查；它不是整个驱动、机械臂或机器人的关节数量限制。其他电机协议可以拥有完全不同的 ID、帧格式和数量限制，而不需要修改通用驱动循环。

## 各层职责

| 层 | 主要职责 | 不应该知道的内容 |
|---|---|---|
| `MotorCanfdDriver` | 管理任意数量总线/电机、按 `(bus,type)` 分组、统一收发 | 具体机器人拓扑、ROS 2 |
| `MotorProtocol` | 定义某一电机家族的使能、命令、反馈规则 | ROS joint、controller |
| `protocol.cpp` | 把物理量与协议字节相互转换 | 总线拓扑、ROS 2 |
| `SocketCan` | Linux SocketCAN/CAN-FD 帧收发 | 电机类型、关节含义 |

## 新增一种电机类型

1. 为新线协议实现一个 `MotorProtocol`；
2. 在 `MotorProtocolRegistry` 中注册一个工厂，或者由调用方传入已经注册好的 registry；
3. 在 YAML 中增加新的 `type` 及该类型所需参数。

已有类型只是增加更多电机时，只需要增加 YAML `motors` 项，不需要修改 C++。

新增电机家族时，通用的发送循环、接收循环、总线管理以及 ROS 2 关节映射都不应该再增加 `if (type == ...)` 分支；电机差异应收敛在协议策略内部。

## 配置边界

本包不直接解析 ROS 参数，也不解析 YAML。调用方先构造普通 C++ `DriverConfig`，再传给 `MotorCanfdDriver`。

当前工作空间由 `motor_hardware_interface` 在 ROS 侧读取 `motor_canfd.yaml`，转换为 `DriverConfig` 后交给底层库。因此 `motor_canfd_drive` 不依赖 `yaml-cpp`、`rclcpp`、`hardware_interface`、`pluginlib`、Topic、Action 或 Service。

这个边界很重要：以后即使更换 ROS 2 上层、编写独立电机测试工具，底层 CAN-FD 驱动仍可直接复用。
