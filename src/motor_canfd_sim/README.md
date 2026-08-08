# motor_canfd_sim

`motor_canfd_sim` 是电机侧 C++ CAN-FD 模拟器，目前内置 `rp` 和 `motor_2530` 两种电机模型。它读取与 `motor_hardware_interface` 相同的 YAML 结构，并复用 `motor_canfd_drive::DriverConfig` 的配置校验、SocketCAN 传输和协议编解码。

旧的独立 `can_motor_sim` 包已经删除；`upper_body_canfd_gazebo` 中原有 Python Gazebo/CAN 桥接器继续保留，因此原仿真路径不受影响。

## 为什么它也是动态配置

模拟器内部没有写死“2 条机械臂 × 每臂 7 关节”。总线、电机数量以及 joint 映射都来自真机同一份 `motor_canfd.yaml`。这样新增关节或调整 ID 后，真机驱动和电机侧模拟器不会维护两份容易漂移的拓扑。

仿真时通过 ROS 参数 `bus_override.<逻辑总线名>` 把 YAML 中的物理 CAN 接口安全地重定向到 vcan，例如：

```yaml
motor_canfd_sim:
  ros__parameters:
    bus_override.left_arm: vcan0
    bus_override.right_arm: vcan1
```

默认情况下，模拟器拒绝打开名称不是 `vcan` 开头的接口。原因是这个进程会“扮演真实电机”并主动发送反馈帧；如果误连到真实 `can0/can1`，可能与真实电机同时发送相同协议的状态帧。

只有明确进行 HIL（Hardware-in-the-Loop，硬件在环）测试时，才允许显式设置 `allow_hardware_interfaces:=true`。

## 全回环数据流

```text
ros2_control position controller
          -> motor_hardware_interface
          -> MotorCanfdDriver
          -> vcan
          -> motor_canfd_sim（模拟电机）
          -> Gazebo effort controller
          -> /joint_states
          -> 模拟反馈 CAN-FD 帧
          -> MotorCanfdDriver
```

启动完整回环：

```bash
ros2 launch motor_canfd_sim driver_loopback.launch.py
```

当前半身机器人 Gazebo effort controller 的输出数组顺序仍与 `motor_canfd.yaml` 中的 motor 顺序一致；如果以后修改 Gazebo 机器人本身的 controller joint 列表，必须同步检查这两个顺序。这个限制属于当前 Gazebo 控制器接口，不是 `MotorCanfdDriver` 的关节数量限制。
