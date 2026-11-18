# 仿真

本包统一提供 Gazebo / MuJoCo 启动、协议级电机模拟和 HIL 辅助工具。模型与传动配置位于 `robot_description`，不在本包复制。

## 入口

- `simulation.launch.py`：工作台使用的组合仿真入口。
- `gazebo.launch.py` / `mujoco.launch.py`：对应物理引擎。
- `display.launch.py`：模型显示。
- `motor_sim.launch.py`：独立电机协议模拟，参数由 YAML 配置提供。

```bash
colcon build --packages-select robot_sim
source install/setup.bash
ros2 launch robot_sim motor_sim.launch.py motor_config_file:=/path/to/motors.yaml
```

MuJoCo HIL 位置模式可按输入关节调整位置刚度 `kp` 和速度阻尼 `kv`。参数初值取自当前 MJCF，在线修改在下一控制拍生效，未注册轴在重新注册后采用新值。以下命令读写本次仿真的右踝横滚增益，重启仿真后恢复模型初值。

```bash
ros2 param get /mujoco_effort_bridge position_servo.right_ankle_roll
ros2 param set /mujoco_effort_bridge position_servo.right_ankle_roll '[48.0, 1.2]'
```

原 `motor_sim` 发行包已合入本包；旧命令中的包名改为 `robot_sim`，可执行文件仍为 `motor_sim_node`。节点名、话题、参数和 `motor_sim/` 头文件路径不变。CMake 消费者使用 `find_package(robot_sim REQUIRED)`，导出目标为 `robot_sim::motor_protocol`、`robot_sim::motor_transport`、`robot_sim::motor_sim_core`、`robot_sim::motor_sim_ros`。

独立电机节点需要外部关节反馈和 effort 接收端；它通过 YAML 指定的总线端点发送协议帧。串口或物理 CAN 配置可能接入真实设备，应先确认端点与物理总线隔离。

测试：`colcon test --packages-select robot_sim`，然后 `colcon test-result --verbose`。单元与传动 HIL 测试使用替身或伪终端，不需要物理 CAN、串口或电机；不等同于完整 Gazebo / MuJoCo 闭环验收。

## 多路模拟电机后端

工作台「仿真 → CAN 接口」选择“多路 CAN（含创芯 USB-CAN FD）”，在电机后端中启用 MCU CAN1～CAN3，逐路选择端点和节点 ID。节点可填 `1-9,12`，只加载填写的模型节点，未分配的节点不创建；`*` 加载其他路未显式分配的模型节点，留空只开通道。同一节点不能跨路重复，填写的 ID 必须属于所选模型。

创芯双通道示例（全身模型仅加载 18 个节点，1～9 走 CAN1、10～18 走 CAN2，19～37 不加载）：

```bash
ros2 launch robot_sim simulation.launch.py \
  simulator_only:=true controller_mode:=firmware transport:=multi \
  model:=full_body_robot start_web_host:=false start_micro_ros_agent:=false \
  'sim_channels_json:=[{"bus":0,"transport":"usbcanfd","device":0,"channel":0,"ids":"1-9"},{"bus":1,"transport":"usbcanfd","device":0,"channel":1,"ids":"10-18"}]'
```

`bus` 是 MCU 总线索引 0～2；`channel` 是分析仪通道索引 0～1；`device` 对应设备枚举中的索引。SocketCAN/SLCAN 通道分别使用 `transport: "socketcan"` / `"slcan"` 和 `endpoint` 字段，可以混用。它们是模拟后端的接线配置，不会改写模型的硬件标定。

创芯采用原厂 Linux SDK，仲裁 1 Mbps、数据 5 Mbps、ISO FD、BRS，启用分析仪的 120 Ω 终端。将适配本机平台的原厂 `libcontrolcanfd.so` 放到 `~/.local/lib/robot2/`，或在命令行通过 `ROBOT2_CONTROLCANFD_LIB` / `--usbcanfd-library` 指定；原厂二进制不纳入仓库。可安装本包 `config/99-robot2-usbcanfd.rules` 到 `/etc/udev/rules.d/`，重载 udev 后重插设备，为 plugdev 用户提供读写权限并关闭该设备的自动挂起。

Linux SDK 在打开设备时重复发送 `SET_CONFIGURATION(1)`，实测会使固件 V2.13 的设备握手超时，USB 仍可枚举。使用仓库中的兼容加载库保留已生效的配置（需要 `cc` 编译器，在工作区根目录执行）：

```bash
python3 micro_ros_agent/tools/build_usbcanfd_compat.py
```

它在原厂库旁生成 `libcontrolcanfd_robot2.so`，后端默认优先加载；原厂库不被覆盖。自定义 SDK 路径可作为脚本参数传入，并将生成的兼容库路径传给 `ROBOT2_CONTROLCANFD_LIB` / `--usbcanfd-library`。兼容层只在查询确认当前配置与请求相同时跳过重复设置；真实配置变更仍调用原厂实现，查询失败则中止打开。它以本地动态库作用域加载，不使用全局 `LD_PRELOAD`。如果此前已经握手超时，安装兼容层后需拔掉分析仪 USB 约 5 秒再插回，清除设备异常状态；单纯重启后端不能恢复。

后端保持一份控制 socket。`status` 返回逐节点 `bus` 和逐路状态、SYNC 计数、丢帧计数及错误；多路 `add` 命令须带 `bus`，`remove/link` 按全局唯一 ID 路由。一路运行期 I/O 故障只停止该路；其他路继续服务，故障端点在后端整体停止时统一关闭，重新启用故障路需停止并重新启动后端。旧单路参数保持兼容。
