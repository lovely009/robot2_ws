# motor_canfd_sim

C++ CAN-FD 电机侧模拟器。它通过 vcan 接收真实驱动命令、驱动 Gazebo effort controller，并将 Gazebo joint state 编码为电机反馈帧。

## 定位

本包是仓库唯一支持的电机仿真后端，当前实现 `rp` 和 `motor_2530` 两类虚拟电机。

```text
MotorCanfdDriver -> vcan -> motor_canfd_sim
                                  |
                     effort command / joint state
                                  |
                               Gazebo
```

模拟器默认使用本包 `config/motor_canfd.yaml`（仿真侧拓扑，来源于仿真模型），
上层 launch 需要不同拓扑时用 `motor_config_file` 覆盖；配置解析统一复用
`motor_hardware_interface::load_driver_config()`，与真机配置
（`motor_hardware_interface/config/motor_canfd.yaml`）解耦。

帧级处理按电机家族封装为 `SimMotorHandler`（与驱动侧 `MotorProtocol` 一一对应），
节点只做路由与 ROS 编排；新增家族时在模拟器侧注册 handler，不在节点内增加类型分支。

## 安全限制

模拟器会主动发送电机反馈帧，默认只允许打开名称以 `vcan` 开头的接口，防止与真实电机产生重复 CAN ID。

仅在明确执行 HIL 测试时，才可设置：

```text
allow_hardware_interfaces:=true
```

启用该参数前必须完成总线隔离、CAN ID 冲突检查和急停验证。

## 启动

完整 Gazebo 回环由顶层集成包启动：

```bash
ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py
```

只启动 Gazebo 模型和电机模拟器，不启动驱动侧 controller：

```bash
ros2 launch upper_body_canfd_gazebo model_sim.launch.py
```

只启动虚拟 CAN 电机设备：

```bash
ros2 launch motor_canfd_sim motor_canfd_sim.launch.py \
  motor_config_file:=$(ros2 pkg prefix motor_hardware_interface --share)/config/motor_canfd.yaml \
  interface_left_arm:=vcan0 interface_right_arm:=vcan1
```

`motor_config_file` 默认指向本包 `config/motor_canfd.yaml`，调用方需要不同
拓扑时覆盖。总线接口参数由配置的 buses 段生成（`interface_<总线名>`），
默认分别为 `vcan0`、`vcan1`。

## 参数

运行参数默认值位于 `config/sim.yaml`。主要参数包括：

- `motor_config_file`：电机拓扑配置路径；
- `bus_override.<bus>`：逻辑总线到 vcan 的覆盖；
- `allow_hardware_interfaces`：是否允许非 vcan 接口；
- `control_rate_hz`、`status_rate_hz`、`command_timeout_s`：循环频率与命令超时；
- `rp_torque_limit_nm`、`motor_2530_*`：仿真模型限幅与简化模型参数。

上层控制命令必须发送到 ros2_control position controller，不应由应用直接占用 vcan。

## 已知约束

Gazebo effort controller 的数组顺序必须与仿真侧 `config/motor_canfd.yaml` 的
motor 顺序一致。修改仿真模型 joint 列表、controller 配置或仿真拓扑时，应
同步检查三者；该约束由 `upper_body_canfd_gazebo` 的 `test_config_topology.py`
静态测试兜底。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to motor_canfd_sim
colcon test --packages-select motor_canfd_sim
colcon test-result --verbose
```

`test/test_plant_model.cpp` 覆盖 RP MIT 公式、腕部三模式、限幅与扭矩常数换算；
`test/test_motor_canfd_sim_launch.py` 对独立 launch 的参数和安全默认值进行静态检查。
两者都不启动 Gazebo 或硬件。
