# motor_hardware_interface

将 `motor_canfd_drive` 接入 ROS 2 Humble `ros2_control` 的 C++ 适配包。

## 职责

- 实现 `hardware_interface::SystemInterface` lifecycle；
- 导出 joint position command，以及 position、velocity、effort state interface；
- 解析并校验 `motor_canfd.yaml`；
- 按 joint 名称建立 ROS 关节与电机索引映射；
- 管理首次反馈、运行期反馈超时和安全失能；
- 将配置转换为纯 C++ `motor_canfd_drive::DriverConfig`。
- 提供真机 bring-up 入口：`launch/hardware.launch.py`、`config/driver_controllers.yaml`
  与通用的 CAN-FD ros2_control 硬件 xacro 片段，真机部署不依赖仿真栈。

本包不实现线协议或 SocketCAN 帧编解码，也不限定机械臂数量和每臂关节数。

## 配置

默认配置为 `config/motor_canfd.yaml`，结构如下：

```yaml
motor_hardware:
  timing:
    feedback_timeout_s: 0.10
    initial_feedback_wait_s: 0.10
  buses:
    left_arm: {interface: can0}
  motor_types:
    rp:
      command: {kp: 30.0, kd: 1.0}
  motors:
    left_shoulder_yaw:
      joint: left_shoulder_yaw_joint
      bus: left_arm
      type: rp
      id: 1
```

| 节点 | 含义 |
|---|---|
| `timing` | 首次反馈等待和运行期反馈超时 |
| `buses` | 逻辑总线到 SocketCAN 接口的映射 |
| `motor_types` | 同类型电机的命令和协议参数默认值 |
| `motors` | motor、joint、bus、type 与 ID 的拓扑映射 |

单电机参数覆盖同类型默认值。Launch 可用 `bus_override.<逻辑总线名>` 覆盖接口
（真机场景把逻辑总线指向 `can0/can1`）。仿真侧不再复用本配置：仿真拓扑位于
`motor_canfd_sim/config/motor_canfd.yaml`，本文件只服务驱动侧/真机。

公开解析接口为：

```cpp
motor_canfd_drive::DriverConfig load_driver_config(const std::string & path);
```

`motor_canfd_sim` 也调用该接口，避免重复实现 YAML 解析。

## 真机 bring-up

```bash
ros2 launch motor_hardware_interface hardware.launch.py \
  model_xacro:=src/upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro \
  interface_left_arm:=can0 interface_right_arm:=can1
```

`model_xacro` 是必填参数：指向能渲染 CAN-FD 后端 ros2_control 块的机器人模型 xacro
（要求暴露 `hardware_backend`/`bus_overrides`/`motor_config_file`
参数）。真机部署只需底层驱动 + 本包 + 模型描述；启动前须完成 CAN-FD 总线配置并逐关节
验证零位、方向、限位、增益与急停。

## Lifecycle 与安全行为

激活流程：

1. 打开配置中的 SocketCAN 接口。
2. 保持电机失能，以零增益命令等待全部关节的有效反馈。
3. 将实测位置设为初始 position target。
4. 全部关节就绪后统一使能。

运行时 `read()` 检查每个 joint 的反馈时间戳；超过 `feedback_timeout_s` 时向
`controller_manager` 返回错误。停用流程先发送失能命令，再关闭总线。

> 激活失败或运行期反馈超时会让 `controller_manager` 立即终止——这是预期的
> fail-fast 安全语义（电机不可达就不能运行），不是可恢复错误；重新 bring-up 前
> 先诊断总线、电源与标定。安全逻辑不能替代硬件急停、机械限位和真机标定。

## 插件

插件描述文件为 `hardware_plugins.xml`。`urdf/canfd_hardware.xacro` 提供与机器人拓扑
无关的 CAN-FD ros2_control 硬件片段（`canfd_hardware` 与 `canfd_control_joint` 宏），
机器人模型 xacro include 后通过 `motor_config_file` 和 `bus_override.*` 参数加载插件；
上层应用只使用标准 ros2_control controller 接口。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to motor_hardware_interface
colcon test --packages-select motor_hardware_interface
colcon test-result --verbose
```

`test/test_motor_config.cpp` 使用 `test/fixtures/` 中的配置覆盖正常解析和错误配置校验；
`test/test_hardware_launch.py` 静态检查真机 bring-up launch 的参数与节点组成。
