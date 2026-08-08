# motor_hardware_interface

`motor_hardware_interface` 是 ROS 2 Humble `ros2_control` 到 `motor_canfd_drive` 的 C++ 适配层。

插件实现标准 `hardware_interface::SystemInterface`。Python ROS 2 节点仍然可以通过 controller 的 Topic / Action / Service 正常控制机器人，不需要为底层驱动额外制作 `pybind11` 或 Python wrapper。

## 分层边界

```text
controller_manager / ros2_control
              |
              v
motor_hardware_interface
  - lifecycle 生命周期
  - ROS joint 状态/命令接口
  - YAML 参数与超时
  - joint_name <-> motor_index 映射
              |
              v
motor_canfd_drive::MotorCanfdDriver
  - 按 bus/type 动态分组
  - 协议编解码
  - SocketCAN
```

这里没有固定“几条机械臂”或“每条机械臂几个关节”。插件会检查 `<ros2_control>` 中声明的每个 joint 是否恰好映射到一个 YAML 电机，并且按 **joint 名称** 建立映射，而不是依赖数组下标或“前 4 个是 RP、后 3 个是腕部”之类的固定顺序。

## 配置文件

`config/motor_canfd.yaml` 的主要结构：

- `buses`：任意数量的逻辑总线名和 SocketCAN 接口；
- `motor_types`：某一电机类型共享的默认控制增益与驱动参数；
- `motors`：任意数量的电机项，每项指定 `joint`、`bus`、`type`、`id`；
- `timing`：首次反馈等待时间和运行时反馈超时。

示例：

```yaml
motor_hardware:
  buses:
    left_arm: {interface: can0}
  motor_types:
    rp:
      command: {kp: 30.0, kd: 1.0}
  motors:
    shoulder_yaw:
      joint: left_shoulder_yaw_joint
      bus: left_arm
      type: rp
      id: 1
```

同一种电机可以在 `motor_types` 下设置公共默认值，再在某个具体 motor 的 `command` 或 `parameters` 中覆盖。覆盖顺序是“类型默认值 → 单电机值”，因此只有特殊关节需要重复写参数。

机器人自己的 launch 还可以通过 `bus_override.<逻辑总线名>` 覆盖 SocketCAN 接口。例如 YAML 真机写 `can0`，仿真启动时可以映射成 `vcan0`，无需维护第二份电机拓扑。

## 激活安全逻辑

硬件插件激活时不会假定机器人当前位于零位。它会先保持失能，用零增益命令读取每个关节的真实反馈；只有所有关节都收到有效位置后，才把“当前姿态”作为初始 position target 并统一使能。

运行期间 `read()` 会持续检查每个 joint 的反馈时间戳；超过 `feedback_timeout_s` 会向 `controller_manager` 返回错误。停用插件时先发送失能，再关闭 SocketCAN。
