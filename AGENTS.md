# Repository Guidelines

`robot_ws` 是**三区域工作空间**（`dev` 开发分支），各区域独立构建。权威文档优先：
各区域/各包 `README.md`；本文件给 Agent 即用指南；架构取舍见 `docs/adr/`。

| 区域 | 内容 | 权威文档 |
|---|---|---|
| `ros2_ws/` | ROS 2 Humble 电机栈：真机 `motor_drive` + `motor_hardware_interface`，仿真 `motor_sim` + `robot_arm_gazebo` | 本文件 + `CONTEXT.md` + 各包 `README.md` |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS，serial over USB CDC） | `micro_ros_firmware/README.md` + `micro_ros_firmware/AGENTS.md` |
| `micro_ros_agent/` | micro-ROS Agent 独立 colcon 工作区（serial ↔ 固件，与电机栈同域互通） | `micro_ros_agent/README.md` |

## ros2_ws（ROS 2 电机栈）

真机 `motor_drive`（无 ROS 底层驱动，双传输 `socketcan`/`serial_slcan`，协议 `rp`/`vaxor`）
+ `motor_hardware_interface`（ros2_control 适配 + 真机入口）+ 仿真 `motor_sim` +
`robot_arm_gazebo`。**电机拓扑唯一来源是 YAML**：数量/类型/ID/关节名/总线/传输由
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

STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS），serial over
USB CDC（PA11=USB_DM / PA12=USB_DP）进 ROS 2（humble）。业务为 **BMI088 IMU**
（`micro_ros/imu/data`、`micro_ros/imu/temp`）+ **三路 FDCAN**（FDCAN1/2/3，
例程测试帧默认关闭、RX 回调收帧）+ **电机协议编解码**（RP/25-30mm，来自
`ros2_ws/src/motor_drive`；`micro_ros/motor_command` 驱动两个默认测试电机，
消息为 `sensor_msgs/JointState`；完整拓扑后续由参数服务下发）。
**详细决策与踩坑见 `micro_ros_firmware/README.md` 与 `micro_ros_firmware/AGENTS.md`**，
这里只列即用要点：

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run            # 构建（pio 不在 PATH；首次需外网拉 micro-ROS 源码）
~/.platformio/penv/bin/pio run -t upload  # ST-Link 烧录
~/.platformio/penv/bin/pio device monitor # 串口监视 115200
```

- 非 ROS 2 工程，**不要用 colcon**；改构建宏等不触发 SrcWrapper 重编译，
  须 `pio run -t clean` 全量重建。
- H723 无独立 `USB_OTG_FS`：USB 靠 `-DPIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED_FULLMODE`；
  HAL 模块使能走命令行 `-D` 宏，SrcWrapper 编译不含项目 `include/`。
- FreeRTOS：调度器接管 SysTick；micro-ROS 建链/收发在独立 `micro_ros` 任务，
  IMU 与 FDCAN 各有独立业务任务；`loop()` 被 idle hook 调用只能非阻塞。
- 节点名 `micro_ros_node`（`src/ros_bridge.cpp` 顶部常量，改一处即可）。
- FDCAN 需 `-DHAL_FDCAN_MODULE_ENABLED`，内核时钟为 PLL2Q=80MHz；
  改 HAL 构建宏后必须 `pio run -t clean` 全量重建。
- 状态灯是 **WS2812 彩灯**（SPI6 MOSI=PA7），`digitalWrite` 无效，用 `ws2812_set_rgb()`。
- 烧录/串口权限：stlink 与 USB CDC（0483:5740）udev 规则；
  `libusb_open ... LIBUSB_ERROR_ACCESS` 即缺 stlink 规则。

## micro_ros_agent（micro-ROS Agent）

独立 colcon 工作区（ROS 2 humble），把固件（serial USB CDC `/dev/ttyACM0`）桥进 ROS 2 域：
`micro_ros_msgs` + `can_bridge_msgs` + `micro-ROS-Agent` 快照，构建/联调详见
`micro_ros_agent/README.md`。要点：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install                # 缺测试依赖可加 --cmake-args -DBUILD_TESTING=OFF
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

无客户端时 agent 节点不出现在 `ros2 node list`，属正常（节点在客户端建立会话后才注册）。
