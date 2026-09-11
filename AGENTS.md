# Repository Guidelines

`robot_ws` 是**三区域工作空间**（`dev` 开发分支），各区域独立构建。权威文档优先：
各区域/各包 `README.md`；本文件给 Agent 即用指南；架构取舍见 `docs/adr/`。

| 区域 | 内容 | 权威文档 |
|---|---|---|
| `ros2_ws/` | ROS 2 Humble 六包工作区：消息、真机、仿真与 Web 上位机 | 本文件 + `CONTEXT.md` + 各包 `README.md` |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS，serial over USB CDC） | `micro_ros_firmware/README.md` + `micro_ros_firmware/AGENTS.md` |
| `micro_ros_agent/` | micro-ROS Agent 独立 colcon 工作区（serial ↔ 固件，与电机栈同域互通） | `micro_ros_agent/README.md` |

## ros2_ws（ROS 2 电机栈）

真机 `motor_drive`（无 ROS 底层驱动，双传输 `socketcan`/`serial_slcan`，协议 `vaxor`/`cerb`）
+ `motor_hardware_interface`（ros2_control 适配 + 真机入口）+ 仿真 `motor_sim` +
`robot_arm_gazebo`；`can_bridge_msgs` 提供固件/ROS 2 共用消息，`web_host` 提供
rosbridge Web 上位机。**电机拓扑唯一来源是 YAML**：数量/类型/ID/关节名/总线/传输由
`motor_hardware_interface/config/motor_driver.yaml` 决定，增删改不修改 C++、launch
或 controller 配置。

## 结构与入口

- `ros2_ws/src/<package>` 标准 ament 布局（`src/`/`include/`/`config/`/`launch/`/`test/`，
  按需 `urdf/`、`scripts/`）；各包 `README.md` 是权威文档，修改前先读。
- launch 与测试共用 `motor_hardware_interface/motor_hardware_interface/helpers.py`
  （buses 读取、joints 生成、参数文件写入）。
- 协议 PDF 在 `ros2_ws/src/motor_drive/docs/`。

## 命令

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
ros2 launch motor_hardware_interface hardware.launch.py  # 真机，默认 tty 双总线
ros2 launch robot_arm_gazebo gazebo.launch.py           # 仿真（vcan）
```

- 覆盖设备路径：`interface_<逻辑总线名>:=<路径>`；传输类型切换改 YAML `transport`。
- serial_slcan 需要 tty 写权限（dialout/sudo）；socketcan 启动前
  `ip link ... fd on` 配仲裁 1M/数据 5M；首次接入逐关节验证零位/方向/限位/急停。

## 代码规范

- C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类）、`k` 前缀常量；Python snake_case 且测试带类型注解。
- 中文注释写意图（数据流/单位/安全边界）；协议名、ROS 2 API 保持英文。

## 测试规范

- C++ GoogleTest（`test/test_*.cpp`）；Python pytest（`test/test_*.py`，
  函数名 `test_<behavior>()`）。
- 测试不得启动 Gazebo 或真实硬件；统一 `colcon test && colcon test-result --verbose`。

## 提交与 PR

- conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因。
- 功能经 PR 合入 `dev`；`main` 仅发布；真机/仿真线分别同步 `dev-real` / `dev-sim`；
  PR 附验证方式（如 `colcon test` 结果）。

## Agent 注意事项

- 优先找现成方案，不自己造轮子：动手前先查已有 skill/工具/库/依赖（如 `find-skills`、
  各区域 `README.md`、已装依赖），确认没有现成再实现；能用成熟库就别手写核心逻辑。
- 不写死电机数量/类型/总线：拓扑只来自 YAML；扩展协议走 `MotorProtocolRegistry`、
  扩展传输走 `CanTransportRegistry`，不改通用收发循环。
- `bus_override` 只覆盖设备路径，传输类型切换须改 YAML。
- URDF 物理模型（几何/惯性）人工维护；ros2_control 关节清单由 launch 自动生成，
  不手写 `canfd_control_joint`。
- 软件安全逻辑不能替代硬件急停、机械限位与真机标定。

## micro_ros_firmware（STM32 固件）

STM32H723VGT6 固件使用 PlatformIO + STM32duino + FreeRTOS，经 USB CDC 对接
micro-ROS Agent。BMI088 使用阻塞 SPI2；三路 FDCAN 由单一 `motor_rt_task` 管理。
电机运行时按 `motor_id` 发现和控制，不保存安装表或 `joint_name`；命令是最多
64 个目标的完整快照。详细约束见 `micro_ros_firmware/README.md`、
`micro_ros_firmware/AGENTS.md` 和运行时 HANDOFF
（`micro_ros_firmware/docs/handoffs/HANDOFF-motor-runtime-refactor.md`）。

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure
```

- 固件不是 ament 包，不能用 colcon；修改 `platformio.ini` 编译宏后先执行
  `pio run -t clean`，不要在常规验证中运行 `clean_microros`。
- `motor_rt_task` 是 RX ring 唯一消费者、运行时唯一写者和正常 CAN TX 所有者；
  ROS 桥只使用 mailbox、只读状态副本和事件中继。
- FDCAN 内核时钟为 PLL2Q=80MHz；仲裁段 1Mbps、数据段 5Mbps。
- BMI088 当前不占用 DMA；WS2812 使用 SPI6 + BDMA Channel0，两个设备库各自
  持有句柄和中断入口。
- 自动测试不得烧录或驱动电机；软件停机不能替代硬件急停、机械限位与供电隔离。

## micro_ros_agent（micro-ROS Agent）

独立 colcon 工作区（ROS 2 humble），把固件（serial USB CDC `/dev/ttyACM0`）桥进 ROS 2 域：
`micro_ros_msgs` + `can_bridge_msgs` + `micro-ROS-Agent` 快照，构建/联调详见
`micro_ros_agent/README.md`。要点：

```bash
source /opt/ros/humble/setup.bash
cd micro_ros_agent
colcon build --symlink-install                # 缺测试依赖可加 --cmake-args -DBUILD_TESTING=OFF
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

无客户端时 agent 节点不出现在 `ros2 node list`，属正常（节点在客户端建立会话后才注册）。
