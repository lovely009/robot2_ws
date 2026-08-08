# motor_hardware_interface

ROS 2 Humble `ros2_control` 适配层。该包只承担 ROS 关节接口/lifecycle 与
`motor_canfd_drive::ArmDriver` 之间的转换，不再拥有私有协议或 SocketCAN 实现。

```text
controller_manager
       │ SystemInterface
       ▼
motor_hardware_interface
       │ C++ API
       ▼
motor_canfd_drive
       │ SocketCAN / CAN-FD
       ▼
can0 + can1
```

## Python 边界

Humble 的硬件组件由 `controller_manager` 通过 `pluginlib` 加载 C++
`hardware_interface::SystemInterface` 动态库，因此本包没有 Python hardware plugin，也不为
`motor_canfd_drive` 提供 pybind11。Python 节点仍可正常通过 ROS 2 controller 的
Topic / Action / Service 控制机器人；Python launch 文件也可以正常启动这个 C++ 插件。

## 参数

电机映射和调参集中在 `config/motor_canfd.yaml`：

- 左/右 SocketCAN 接口；
- RP ID 与腕部节点；
- 14 个关节名及顺序；
- RP / 腕部 Kp、Kd；
- 腕部 Nm/A 标定参数；
- 初始反馈等待与运行反馈超时。

`ros2_control` 的硬件插件参数仍通过 URDF `<ros2_control><hardware><param>` 进入
`HardwareInfo`；这里只传入 YAML 的 `config_file` 路径以及可选的左右接口覆盖值。控制器自身的
参数继续使用标准 ROS 2 controller YAML。

激活时会先显式失能，随后用零增益指令获取 14 路真实当前位置；只有全部收到反馈后才以当前
姿态为初始目标并使能。不会假定机械零位。
