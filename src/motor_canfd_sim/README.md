# motor_canfd_sim

`motor_canfd_sim` 是电机侧 C++ CAN-FD 模拟器，目前内置 `rp` 和 `motor_2530` 两种电机模型。它读取与 `motor_hardware_interface` 相同的 YAML 结构，并复用 `motor_canfd_drive::DriverConfig` 的配置校验、SocketCAN 传输和协议编解码。

本包是仓库唯一的电机仿真后端。Python Gazebo/CAN 桥和直接向 vcan 发送动作命令的路径已经移除，避免仿真绕过真实驱动架构或维护两套电机模型。

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

也可以只用 C++ 模拟器替换旧 Gazebo/CAN 桥接器：

```bash
ros2 launch motor_canfd_sim gazebo_cpp_sim.launch.py
```

该入口会直接读取真机驱动使用的
`motor_hardware_interface/config/motor_canfd.yaml`，因此电机 ID、关节和逻辑总线映射与真机保持一致；启动文件不会修改这份 YAML，而是通过
`bus_override.left_arm` 和 `bus_override_right_arm` 将其中的物理总线分别映射到
`left_interface`/`right_interface`。两个 launch 参数默认是 `vcan0`/`vcan1`，也可以在启动命令中显式覆盖。

动作和其他上层应用应向 `/driver/position_controller/commands` 发布 ROS 2 position command，不应直接占用 vcan。仓库提供的参考动作入口为：

```bash
ros2 run upper_body_canfd_gazebo ros_actions
```
当前半身机器人 Gazebo effort controller 的输出数组顺序仍与 `motor_canfd.yaml` 中的 motor 顺序一致；如果以后修改 Gazebo 机器人本身的 controller joint 列表，必须同步检查这两个顺序。这个限制属于当前 Gazebo 控制器接口，不是 `MotorCanfdDriver` 的关节数量限制。
