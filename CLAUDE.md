# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 这是什么

ROS 2 Humble 工作空间（`dev` 开发分支），`ros2_ws/` 下六个 ament 包：

- `can_bridge_msgs` —— 固件与 ROS 2 共用的 ID 化有界消息定义
- `motor_drive` —— 无 ROS 依赖的 C++17 底层驱动库（CAN 传输、线协议、电机映射）
- `motor_hardware_interface` —— ros2_control 适配层 + 真机 bring-up 入口
- `motor_sim` —— 与机器人无关的共享仿真库（协议 / 传输 / 传动 / plant / 帧路由 / 锁存安全）
- `robot_arm_gazebo` —— UR5e 模型与 Gazebo 集成（URDF / world / controller / launch）
- `web_host` —— rosbridge Web 上位机（姿态、IMU、电机状态与命令快照）

另有两个独立工程区域：`micro_ros_firmware`（PlatformIO，非 ament 包）与
`micro_ros_agent`（独立 colcon 工作区）。

分支约定：`dev/robot_ws` 开发（本分支，真机 + 仿真 6 包）、
`dev-sim` 仿真线、`main` 仅发布。

权威文档（修改代码前先读）：各包 `README.md`（`ros2_ws/src/<pkg>/README.md`）、
术语与命名 `CONTEXT.md`、架构决策 `docs/adr/`。配置为 `motor.yaml`（型号库）+
`motor_driver.yaml`（拓扑），协议 `vaxor`/`cerb`。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose      # 全量测试
```

按包构建：

```bash
colcon build --symlink-install --packages-select motor_drive
colcon build --symlink-install --packages-up-to motor_hardware_interface
```

运行单个测试：

```bash
# 单个 C++ gtest（按 ctest 名过滤）
colcon test --packages-select motor_drive --ctest-args -R test_protocol
# 单个 gtest 用例（先 build，再直接跑二进制）
./build/motor_drive/test_protocol --gtest_filter='ProtocolTest.*'

# 单个 Python pytest
colcon test --packages-select motor_hardware_interface --pytest-args "-k <表达式>"
```

测试目标：`motor_drive` 有 `test_protocol`/`test_motor_protocol`/`test_driver`/`test_slcan`；`motor_hardware_interface` 有 `test_motor_config`/`test_canfd_logic`（gtest）+ `test_hardware_launch`/`test_xacro_consistency`（pytest）；`motor_sim` 有协议/传输/帧路由/plant 等 gtest；`robot_arm_gazebo` 有 launch/拓扑 pytest。测试不得启动 Gazebo 或真实硬件。

真机 bring-up（默认 serial_slcan 双总线 `/dev/ttyACM0`、`/dev/ttyACM1`）：

```bash
ros2 launch motor_hardware_interface hardware.launch.py
ros2 launch motor_hardware_interface hardware.launch.py interface_left_arm:=can0  # 覆盖设备路径
```

仿真 bring-up（vcan）：

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py
```

与机器人无关的独立仿真节点（不启 Gazebo；拓扑完全由 `motor_config_file` 决定，
无任何默认值，bus→端口映射必须显式传入）：

```bash
ros2 launch motor_sim motor_sim.launch.py \
  motor_config_file:=/path/to/motors.yaml transport_mode:=socket_can arm_port:=vcan0
```

注意传输名两套：motor_drive YAML 用 `socketcan`/`serial_slcan`，motor_sim 节点参数用
`socket_can`/`usb_serial`/`serial_slcan`；`serial_slcan` 模式必须显式
`allow_external_canfd:=true`。

## micro_ros_firmware（STM32 固件，PlatformIO，非 ament 包）

STM32H723VGT6 控制板固件：micro-ROS（serial/USB CDC）+ FreeRTOS + 三路 FDCAN，经同仓库 `micro_ros_agent/` 对接 ROS 2。构建体系是 **PlatformIO，不能用 colcon**；权威文档是该目录 `AGENTS.md`，运行时设计与后续台架项见 `micro_ros_firmware/docs/architecture/ARD-0007-micro-ros-motor-runtime.md` 和 `micro_ros_firmware/docs/handoffs/HANDOFF-motor-runtime-refactor.md`（台架报告与指南在 `docs/reports/`、`docs/guides/`）。

```bash
# 均在 micro_ros_firmware/ 内执行
~/.platformio/penv/bin/pio run                 # 编译（pio 不在 PATH）；-t upload 烧录
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure   # host 测试：CTest + cassert 断言（无 gtest，另含一个 python 线协议测试）
```

- 改 `platformio.ini` 编译宏或 `ldscript/*.ld` 后必须 `pio run -t clean`（HAL SrcWrapper 对象不重编；增量构建不监听链接脚本改动、会静默跳过重新链接）；勿跑 `-t clean_microros`（联网重下环境）。
- motor runtime 已完成固件侧 ID 驱动化；当前固件应同时通过 host 测试与 `pio run`。剩余上层迁移和真机台架项见 `micro_ros_firmware/docs/handoffs/HANDOFF-motor-runtime-refactor.md`。

## micro_ros_agent（独立 colcon 工作区，勿在 ros2_ws 内构建）

固件 USB CDC 与 ROS 2 之间的 micro-ROS 桥；也放主机侧台架工具。构建在该目录内独立进行：

```bash
cd micro_ros_agent && colcon build --symlink-install
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0   # ttyACM0=板子 CDC
```

- 消息定义唯一来源是 `micro_ros_agent/src/can_bridge_msgs`；固件 `extra_packages/can_bridge_msgs` 是指向它的符号链接。
- 改消息后普通 `pio run` 不会重生成类型（PlatformIO 只检查 `libmicroros.a` 是否存在）：用 `micro_ros_agent/tools/refresh_microros_messages.py` 增量刷新，勿跑 `-t clean_microros`。
- `micro_ros_agent/tools/fdcan_motor_sim.py`：FDCAN1 九节点（CERB 1–6 + VAXOR 7–9）主机模拟器，经 `/dev/ttyACM1` CANable2 SLCAN 跑全通路台架；流程与启动顺序（CANable 必须先成为 ACK 节点再复位控制板，否则总线 bus-off 不自动恢复）见 `docs/guides/fdcan1-bench-guide.md`。

## 架构总览

核心四个电机包单向依赖，职责边界清晰：

```
motor_drive（纯 C++，无 ROS）              motor_hardware_interface（ROS 2 适配）
  CanTransport / CanTransportRegistry         CanFdSystem (SystemInterface lifecycle)
  MotorProtocol / MotorProtocolRegistry       motor_config.cpp（YAML 解析，唯一 YAML 边界）
  MotorDriver（分组/校验/统一收发）           canfd_logic.cpp（纯逻辑，可单测）
  protocol.cpp（物理量 <-> 线协议字段）       launch + xacro（真机 bring-up）

motor_sim（共享仿真库）                      robot_arm_gazebo（集成）
  SimMotorHandler / plant_model / 帧路由        urdf / launch / Gazebo 集成
  （协议 / 传输 / 传动 / 锁存安全）            gazebo.launch.py 统一后端入口
```

核心是「三自适应」闭环，全部由配置驱动、编译代码不含机器人拓扑：

1. **传输自适应** —— `BusConfig.transport` 决定后端：`socketcan`（内核接口，需 `interface`）或 `serial_slcan`（CANable2 串口，需 `tty`，驱动内固定 S8/Y5 位率）。新增后端实现 `CanTransport` 并注册到 `CanTransportRegistry`。
2. **协议自适应** —— `MotorProtocol` 策略 + `MotorProtocolRegistry` 按 type 字符串工厂注册。内置 `vaxor`（25/30 mm 腕部电机，反馈电流 A）与 `cerb`（CERB Lite CAN FD 关节模组，反馈输出轴位置/电机端速度/Iq）。新增电机家族实现 `MotorProtocol` 并注册，不改 `MotorDriver` 收发循环。
3. **数量自适应** —— `DriverConfig` 用 `vector` 描述 buses/motors，无固定 7/14 关节假设。

数据流：YAML → `load_system_config()`（yaml-cpp 解析，字符串参数 map）→ 纯 C++ `DriverConfig` → `MotorDriver`（按 `(bus, type)` 分组，每组一个 `MotorProtocol`）→ `CanTransport` 收发帧。`MotorDriver` 不接触 YAML，协议/传输都不感知 ROS。

## 配置（单一来源）

拓扑是唯一真相，增删改电机零代码改动：

- **`config/motor_driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 + `timing`（反馈超时/激活窗口）+ `buses`（逻辑总线/传输）+ `motors`（`type` 引用型号、`joint`、`bus`、`id`、`command.kp/kd` 动态增益）。
- **`config/motor.yaml`（型号库）**：`motor_types.<型号>.protocol` 声明所属协议，`parameters` 是协议运行所需静态参数（如 `torque_constant_nm_per_amp`、`gear_ratio`）与安全阈值。与拓扑无关、可跨机器人复用。

解析层把型号名经 `protocol` 字段映射为协议类型；参数默认值来自型号库，单电机 `parameters` 可覆盖。`bus_override`（launch 的 `interface_<总线名>`）只覆盖设备路径，**切换传输类型必须改 YAML**。

## launch 与 URDF 模型

- `launch/hardware.launch.py` 用 `OpaqueFunction` 延迟求值：运行时读取实际 `motor_config_file`，生成 position controller 的 joints、URDF ros2_control 关节清单（`control_joints`）与 `bus_overrides`——覆盖配置时自动一致。
- `urdf/canfd_hardware.xacro`：与拓扑无关的 ros2_control 硬件片段（`canfd_hardware`/`emit_bus_overrides`/`emit_control_joints` 宏），不写死任何关节。
- `urdf/upper_body.urdf.xacro`：物理模型（几何/惯性）**人工维护**；其 ros2_control 关节清单由 launch 注入，不手抄。两者一致性由 `test_xacro_consistency.py` 兜底。
- `motor_hardware_interface/helpers.py`：launch 与测试共用（读取 buses/joints、生成参数文件）。
- 仿真 launch：`robot_arm_gazebo` 的 `gazebo.launch.py` 统一仿真后端入口，默认挂 `robot_arm_gazebo/config/motors.yaml`。

## 关键约定与坑

- **命名混淆点**：`vaxor` 协议族对应 25/30 mm 腕部电机；`protocol.hpp` 内部函数与常量仍用 `wrist_*` / `kWrist*` 命名，注释写「25/30 mm」。registry 的 type 字符串是 `vaxor`（见 `motor_protocol.hpp` 的 `kVaxorType`）。
- **单位边界**：所有物理量↔线协议字段换算集中在 `protocol.cpp`；上层统一用 rad、rad/s、A、Nm，协议单位不得渗透进 ROS 控制逻辑。`vaxor` 反馈帧的 `effort` 在 `protocol` 层暂存「电流 A」，转 Nm 由策略完成。
- **不写死拓扑**：电机数量/类型/ID/关节名/总线/传输只来自 YAML；扩展走 `MotorProtocolRegistry` / `CanTransportRegistry`，不在通用收发循环或 ROS 关节映射里加类型分支。
- **安全 fail-fast**：激活时零增益轮询反馈（窗口内最多重试 3 次）→ 实测位置作初始目标 → 统一使能；运行期反馈超时返回错误，`controller_manager` 立即终止。软件兜底不能替代硬件急停、机械限位与真机标定。
- **serial_slcan 权限**：需 tty 写权限（dialout/sudo）；`socketcan` 启动前 `ip link` 配置仲裁 1M/数据 5M。
- **代码规范**：C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进、snake_case（文件/函数/变量）、PascalCase（类）、`k` 前缀常量；中文注释写意图（数据流/单位/安全边界），协议名与 ROS 2 API 保留英文。
- **提交**：conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），功能经 PR 合入 `dev`；真机/仿真线分别同步 `dev-real` / `dev-sim`。
