# motor_hardware_interface

将 `motor_drive` 接入 ROS 2 Humble `ros2_control` 的 C++ 适配包，并提供
真机 bring-up 入口。本包是 `dev-real`（向真机转移）分支的适配层，不依赖
仿真栈；仿真能力保留在旧 `develop` 分支。

## 职责

- 实现 `hardware_interface::SystemInterface` lifecycle；
- 导出 joint position command，以及 position、velocity、effort state interface；
- 解析并校验 `config/motor_canfd.yaml`；
- 按 joint 名称建立 ROS 关节与电机索引映射；
- 管理首次反馈、运行期反馈超时和安全失能；
- 将配置转换为纯 C++ `motor_drive::DriverConfig`；
- 提供真机 bring-up：`launch/hardware.launch.py`、`config/driver_controllers.yaml`
  与 `urdf/canfd_hardware.xacro` 硬件片段，以及内置的 `urdf/upper_body.urdf.xacro`
  机器人模型；position controller 的 joints 由 launch 从电机拓扑配置生成并注入
  （单一来源），不在 `driver_controllers.yaml` 手抄。

本包不实现线协议或 SocketCAN 帧编解码，也不限定机械臂数量和每臂关节数。

## 配置

默认配置为 `config/motor_canfd.yaml`，结构如下：

```yaml
motor_hardware:
  timing:
    feedback_timeout_s: 0.10
    initial_feedback_wait_s: 1.0
  buses:
    left_arm: {interface: can0}
    right_arm: {interface: can1}
  motor_types:
    rp:
      command: {kp: 30.0, kd: 1.0}
    motor_2530:
      command: {kp: 0.008, kd: 1.0}
      parameters: {torque_constant_nm_per_amp: 0.35}
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

单电机参数覆盖同类型默认值。Launch 可用 `bus_override.<逻辑总线名>` 覆盖
SocketCAN 接口名（例如 HIL 时把 `vcan0` 重定向到真实 `can0`）。真机位率由
Linux `ip link` 配置（仲裁 1 Mbit/s、数据 5 Mbit/s），本文件不负责物理位率。

公开解析接口为：

```cpp
motor_drive::DriverConfig load_driver_config(const std::string &path);
```

## 真机 bring-up

```bash
ros2 launch motor_hardware_interface hardware.launch.py \
  interface_left_arm:=can0 interface_right_arm:=can1
```

`model_xacro` 默认指向本包 `urdf/upper_body.urdf.xacro`（只渲染 CAN-FD 后端
ros2_control 块，不含 Gazebo 内容），可用参数覆盖为其他模型。`interface_<逻辑总线名>`
参数由配置的 `buses` 段自动生成。启动前须完成 CAN-FD 总线配置，并逐关节验证
零位、方向、限位、增益与急停。

## Lifecycle 与安全行为

激活流程：

1. 打开配置中的 SocketCAN 接口。
2. 保持电机失能，以零增益命令在 `initial_feedback_wait_s`（默认 1 s）窗口内轮询
   全部关节反馈；未收齐则重试，最多 3 次，每次失败用中文日志列出缺失反馈的关节清单。
3. 将实测位置设为初始 position target。
4. 全部关节就绪后统一使能。

运行时 `read()` 检查每个 joint 的反馈时间戳；超过 `feedback_timeout_s` 时向
`controller_manager` 返回错误。停用流程先发送失能命令，再关闭总线。

> 激活失败或运行期反馈超时会让 `controller_manager` 立即终止——这是预期的
> fail-fast 安全语义（电机不可达就不能运行），不是可恢复错误；重新 bring-up 前
> 先诊断总线、电源与标定。安全逻辑不能替代硬件急停、机械限位和真机标定。

## 插件与模型

- `hardware_plugins.xml`：插件描述文件。
- `urdf/canfd_hardware.xacro`：与机器人拓扑无关的 CAN-FD ros2_control 硬件片段
  （`canfd_hardware` 与 `canfd_control_joint` 宏），通过 `motor_config_file` 与
  `bus_override.*` 参数加载插件。
- `urdf/upper_body.urdf.xacro`：14 自由度上半身模型，关节清单与 `motor_canfd.yaml`
  一一对应；新增/删除关节时需同步两处，静态测试会兜底校验。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to motor_hardware_interface
colcon test --packages-select motor_hardware_interface
colcon test-result --verbose
```

`test/test_motor_config.cpp` 使用 `test/fixtures/` 覆盖正常解析和错误配置校验；
`test/test_hardware_launch.py` 静态检查真机 bring-up launch 的参数与节点组成。
