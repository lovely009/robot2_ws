# motor_canfd_drive

纯 C++ CAN-FD 底层驱动库。它不依赖 `rclcpp`、`ros2_control`、Gazebo 或 Python，负责：

- Linux SocketCAN / CAN-FD 收发；
- 意优 RP MIT 广播、状态帧编解码；
- 25/30 mm 驱控一体 MIT、运行控制、回复/周期状态编解码；
- 单臂总线管理：RP ID 与腕部节点映射、使能/失能、命令下发、反馈归一化。

`ArmDriverConfig` 是驱动配置入口。当前机器人每臂 4 个 RP + 3 个腕部节点，但 ID/节点号不是
散落在协议实现里的魔法数字，而是在配置时注入并校验。ROS 2 参数/YAML 的解析不放在本包，
由 `motor_hardware_interface` 负责，因此本库也可以被独立 C++ 测试程序和 `motor_canfd_sim`
直接复用。

## 分层

```text
ArmDriver
  ├─ protocol.*     RP + 25/30 mm 物理量/字节编解码
  └─ SocketCan      Linux CAN_RAW / CAN-FD BRS 传输
```

协议测试保留了现有工程已经验证过的手册字节序列，并增加了模拟器发包→驱动解包的回环断言。
