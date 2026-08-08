# motor_canfd_sim

新 C++ CAN-FD/Gazebo 电机仿真路径。现有 `upper_body_canfd_gazebo` 的 Python bridge 不删除、
不改协议实现；本包提供一个可并行维护的 C++ bridge，并直接链接 `motor_canfd_drive`，因此
真机驱动与新模拟器只有一份 C++ 协议编解码代码。

```text
CAN master / motor_hardware_interface
              │
          vcan0/vcan1
              │
      motor_canfd_sim (C++)
              │ effort command + joint state
              ▼
      Gazebo Fortress / gz_ros2_control
```

行为与原 Python `canfd_gazebo_bridge.py` 对齐：500 Hz 力矩更新、50 Hz 腕部状态、0.15 s
命令超时、RP MIT 力矩计算、腕部 MIT/位置/速度模式、失能置零、腕部零位偏移和同一组仿真限幅。

参数使用标准 ROS 2 YAML：`config/sim.yaml`。默认只允许 `vcan*`，避免误把电机模拟器挂到
真机 `can0/can1`；只有明确设置 `allow_hardware_interfaces:=true` 才能用于有意的 HIL。

## 两条路径

- `gazebo_cpp_sim.launch.py`：现有 Gazebo 模型 + 新 C++ bridge；
- `driver_loopback.launch.py`：在上面基础上再启动命名空间隔离的
  `motor_hardware_interface`，形成 position controller → C++ driver → vcan → C++ motor sim →
  Gazebo 的完整回环。

旧 Python bridge 仍使用同一 SocketCAN 线协议，所以新 C++ driver 不需要 Python 绑定即可与
它通信；旧 `sim.launch.py` 保持可独立运行。
