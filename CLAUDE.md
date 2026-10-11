# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 这是什么

`robot_ws` 是**三个独立构建区域**，不是一个工作区（`dev/robot_ws` 开发分支）：

| 区域 | 内容 | 构建系统 |
|---|---|---|
| `ros2_ws/` | ROS 2 Humble 七个 ament 包：真机 + 仿真 + 动力学 + 消息 + Web 上位机 | colcon |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件 | PlatformIO（非 ament 包） |
| `micro_ros_agent/` | micro-ROS Agent 与主机侧台架工具 | colcon（在该目录内独立构建） |

ros2_ws 七包：

- `can_bridge_msgs` —— 固件与 ROS 2 共用的 ID 化有界消息定义（21 个 `.msg`，无 README、无测试）
- `motor_drive` —— 无 ROS 依赖的 C++17 底层驱动库（CAN 传输、线协议、电机映射）
- `motor_hardware_interface` —— ros2_control 适配层 + 真机 bring-up 入口 + 固件话题桥
- `motor_sim` —— 与机器人无关的共享仿真库（协议 / 传输 / 传动 / plant / 帧路由 / 锁存安全）
- `robot_arm_gazebo` —— 机械臂 URDF 模型库（`models/<模型名>/`）与 Ignition Gazebo 仿真集成
- `robot_dynamics` —— 独立 Pinocchio 运动学/动力学计算边界（当前仅验证依赖，无节点/话题）
- `web_host` —— rosbridge Web 上位机（臂视图、IMU、设备参数、使能/增益、轨迹、仿真启动器）

分支：`dev/robot_ws` 开发（本分支）、`dev-sim` 仿真线、`main` 仅发布、`origin/dev-real` 真机线。

权威文档（改代码前先读）：根 `AGENTS.md`（Agent 即用指南）、`CONTEXT-MAP.md` → 各区域
`CONTEXT.md`（术语与命名边界）、各包 `README.md`（`ros2_ws/src/<pkg>/README.md`）、
架构决策 `docs/adr/`、台架流程 `docs/guides/fdcan1-bench-guide.md`、历史实测
`docs/reports/`。配置为 `motor.yaml`（型号库）+ `motor_driver.yaml`（拓扑），协议 `vaxor`/`cerb`。

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

测试目标：`motor_drive` —— `test_protocol`/`test_motor_protocol`/`test_driver`/`test_slcan`（gtest）
+ `test_motor_probe`（pytest）；`motor_hardware_interface` —— `test_motor_config`/`test_canfd_logic`（gtest）
+ `test_hardware_launch`/`test_xacro_consistency`/`test_topic_bridge`（pytest）；
`motor_sim` —— 协议/cerb 协议/传输/帧路由/frame record/plant/传动/安全/配置/节点等 gtest；
`robot_arm_gazebo` —— `test_model_consistency`/`test_launch_models`（pytest，模型目录自洽与 launch 契约）；
`robot_dynamics` —— `test_pinocchio_dependency`；`web_host` —— `test/test_agent_manager.py`。
测试不得启动 Gazebo 或真实硬件。

真机 bring-up（默认 serial_slcan 双总线 `/dev/ttyACM0`、`/dev/ttyACM1`）：

```bash
ros2 launch motor_hardware_interface hardware.launch.py
ros2 launch motor_hardware_interface hardware.launch.py interface_left_arm:=can0  # 覆盖设备路径
```

仿真 bring-up —— 两个入口，用途不同：

```bash
# 纯 Gazebo（Ignition + gz_ros2_control 位置控制），不经 motor_sim 的 CAN 回路
ros2 launch robot_arm_gazebo gazebo.launch.py [model:=<模型名>]

# 端到端电机—机械臂仿真：Gazebo + motor_sim + effort 门控 + motor_hardware_interface
ros2 launch robot_arm_gazebo simulation.launch.py [transport:=vcan|socketcan|slcan] \
  [controller_mode:=host|external|firmware]
```

`simulation.launch.py` 默认用本机自动创建/清理的 `robot_vcan0` 闭环（需 `CAP_NET_ADMIN`，
否则预先 `sudo ip link add/set ... type vcan`）；拓扑由
`models/<模型名>/config/simulation_topology.yaml`（电机名 → Gazebo 关节映射 + plant 参数，
只记参与仿真的子集）叠加 `motor_hardware_interface` 的权威配置生成。`controller_mode:=firmware`
走 STM32 HIL，需要 `micro_ros_agent` 工作区已 source。

与机器人无关的独立仿真节点（不启 Gazebo；拓扑完全由 `motor_config_file` 决定，
无任何默认值）：

```bash
ros2 launch motor_sim motor_sim.launch.py \
  motor_config_file:=/path/to/motors.yaml transport_mode:=socket_can arm_port:=vcan0
```

`arm_port` 是 `<总线名>_port`，总线名取自该 YAML（launch 不声明具体 bus 参数，节点运行时按需读取）。

注意传输名两套：motor_drive YAML 用 `socketcan`/`serial_slcan`，motor_sim 节点参数用
`socket_can`/`usb_serial`/`serial_slcan`；`serial_slcan` 模式必须显式
`allow_external_canfd:=true`。

## web_host（rosbridge Web 上位机）

一个 launch 起两个节点：`rosbridge_shim`（先给 `rosbridge_library` 补 `string<=31` 原语再
runpy 官方入口，否则 `can_bridge_msgs` 的有界字符串字段序列化报错）+ `static_server`
（纯标准库托管 `web/`，无 Node 构建链）。启动前必须同时 source 两个工作区：

```bash
source /opt/ros/humble/setup.bash
source micro_ros_agent/install/setup.bash
source ros2_ws/install/setup.bash
ros2 launch web_host web_host.launch.py      # rosbridge_port:=9090、web_port:=8080、model_dir:=
```

浏览器开 `http://<主机IP>:8080`。页面只管理自己启动的一个 micro-ROS Agent 子进程，
不改动 rosbridge 连接；运行用户需对所选串口有读写权限（`dialout`）。

- `--symlink-install` **不能**兜住 `setup.py` 的清单改动：新增/改名 `console_scripts`（如
  `rosbridge_shim`）或 `web/` 下**新增文件**（如 `arm3d.js`、`vendor/`）都必须重跑 colcon build，
  否则分别报 `executable not found` 与页面 404（构建"成功"）。
- 臂视图几何（URDF + 约 20 MB STL）不在 `web/` 里，由 `static_server` 把
  `robot_arm_gazebo` share 目录挂到 `/robot_arm_gazebo`；`web/` 内不能放目录符号链接
  （`os.walk` 与 setuptools 都会漏）。
- 页面职守点在**固件话题**（如 `micro_ros_up/motion_states`），不是 `/joint_states`——
  `/joint_states` 在 Gazebo 台架上会被 gz 自己的 broadcaster 抢同名话题。

## micro_ros_firmware（STM32 固件，PlatformIO，非 ament 包）

STM32H723VGT6 控制板固件：micro-ROS（serial/USB CDC）+ FreeRTOS + 三路 FDCAN，经同仓库 `micro_ros_agent/` 对接 ROS 2。构建体系是 **PlatformIO，不能用 colcon**；**权威操作文档是该目录 `AGENTS.md`**（构建/台架顺序/坑），工程事实以 `micro_ros_firmware/README.md` 为准；设计决策见 `micro_ros_firmware/docs/adr/0007-...`（含勘误），冻结的 ARD-0007 完整规范已归档 `micro_ros_firmware/docs/archive/`，开放台架项唯一清单是 `micro_ros_firmware/docs/plans/active/motor-runtime-validation.md`。

```bash
# 均在 micro_ros_firmware/ 内执行
~/.platformio/penv/bin/pio run                 # 编译（pio 不在 PATH）；-t upload 烧录
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure   # host 测试：CTest + cassert 断言（无 gtest，另含一个 python 线协议测试）
```

- 改 `platformio.ini` 编译宏或 `ldscript/*.ld` 后必须 `pio run -t clean`（HAL SrcWrapper 对象不重编；增量构建不监听链接脚本改动、会静默跳过重新链接）；勿跑 `-t clean_microros`（联网重下环境）。
- host 测试直接编译固件真实 `.c`、无桩无复制；新增内核 `.c` 只改 `test/host/CMakeLists.txt` 一处（`fw_host_test()` 宏即注册 CTest）。
- 当前固件应同时通过 host 测试（22 项 CTest = 21 个 gtest + 1 个 python 线协议用例）与 `pio run`；剩余上层迁移与真机台架项见 validation plan。

## micro_ros_agent（独立 colcon 工作区，勿在 ros2_ws 内构建）

固件 USB CDC 与 ROS 2 之间的 micro-ROS 桥；也放主机侧台架工具（`tools/`）。构建在该目录内独立进行：

```bash
cd micro_ros_agent && colcon build --symlink-install
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0   # ttyACM0=板子 CDC
```

- 消息定义唯一来源是 `micro_ros_agent/src/can_bridge_msgs`；固件 `extra_packages/can_bridge_msgs` 是指向它的符号链接。
- 改消息后普通 `pio run` 不会重生成类型（PlatformIO 只检查 `libmicroros.a` 是否存在）：用 `micro_ros_agent/tools/refresh_microros_messages.py` 增量刷新，勿跑 `-t clean_microros`。
- `tools/fdcan_motor_sim.py`：FDCAN1 默认四节点主机模拟器（CERB 1=R58、2–4=R48），经 CANable2 SLCAN 跑全通路台架；完整九节点需显式传入节点参数。设备路径一律按 `/dev/serial/by-id/` 区分（`ttyACM` 编号会交换）。
- 台架启动**一律走 `tools/bench_up.sh`**（模拟器先起当 ACK 节点 → 再复位板子）。CANable 必须先成为 ACK 节点再复位控制板，否则总线 bus-off 不自动恢复；**烧录前先停模拟器**——MCU 挂起期间模拟器仍在发帧而无人 ACK，会把 CANable 打进错误态。完整流程见 `docs/guides/fdcan1-bench-guide.md`。
- 无客户端时 agent 节点不出现在 `ros2 node list`，属正常（客户端建立会话后才注册）。

## 架构总览

核心四个电机包单向依赖，职责边界清晰：

```
motor_drive（纯 C++，无 ROS）              motor_hardware_interface（ROS 2 适配）
  CanTransport / CanTransportRegistry         CanFdSystem (SystemInterface lifecycle)
  MotorProtocol / MotorProtocolRegistry       motor_config.cpp（YAML 解析，唯一 YAML 边界）
  MotorDriver（分组/校验/统一收发）           canfd_logic.cpp（纯逻辑，可单测）
  protocol.cpp（物理量 <-> 线协议字段）       launch + xacro（真机 bring-up）
                                              topic_bridge（固件 ID 话题 <-> ROS 关节话题）

motor_sim（共享仿真库）                      robot_arm_gazebo（模型库 + 仿真集成）
  SimMotorHandler / plant_model / 帧路由        models/<模型名> / launch / Gazebo 集成
  （协议 / 传输 / 传动 / 锁存安全）            gazebo.launch.py（纯 Gazebo）
                                              simulation.launch.py（端到端组合入口）
```

`web_host` 是纯消费方：直连 rosbridge 订阅固件话题，不依赖 `motor_drive`。

独立计算边界：`robot_dynamics -> Pinocchio`。它不依赖电机驱动、ros2_control 适配、
`motor_sim` 或 Gazebo，也不进入现有实时控制路径；跨包集成须另立 ADR。

核心是「三自适应」闭环，全部由配置驱动、编译代码不含机器人拓扑：

1. **传输自适应** —— `BusConfig.transport` 决定后端：`socketcan`（内核接口，需 `interface`）或 `serial_slcan`（CANable2 串口，需 `tty`，驱动内固定 S8/Y5 位率）。新增后端实现 `CanTransport` 并注册到 `CanTransportRegistry`。
2. **协议自适应** —— `MotorProtocol` 策略 + `MotorProtocolRegistry` 按 type 字符串工厂注册。内置 `vaxor`（25/30 mm 腕部电机，反馈电流 A）与 `cerb`（CERB Lite CAN FD 关节模组，反馈输出轴位置/电机端速度/Iq）。新增电机家族实现 `MotorProtocol` 并注册，不改 `MotorDriver` 收发循环。
3. **数量自适应** —— `DriverConfig` 用 `vector` 描述 buses/motors，无固定 7/14 关节假设。

数据流：YAML → `load_system_config()`（yaml-cpp 解析，字符串参数 map）→ 纯 C++ `DriverConfig` → `MotorDriver`（按 `(bus, type)` 分组，每组一个 `MotorProtocol`）→ `CanTransport` 收发帧。`MotorDriver` 不接触 YAML，协议/传输都不感知 ROS。

### 固件 ↔ ROS 2 接口边界

固件侧接口是 **ID 化、无关节名**的私有话题（ARD-0007 禁止 MCU 维护 `joint_name`），
ROS 生态侧按关节名索引，映射只存在于主机侧：

- 上行 `micro_ros_up/*`：`motion_states`（50 Hz）、`motor_status_array`（20 Hz）、
  `motor_topology`、`motor_events`、`trajectory_state`、`device_param_report`、
  `system_status`、`diagnostics`、`imu/data`、`imu/temp`。
- 下行 `micro_ros_do/*`：`motor_command_array`（完整快照）、`motor_trajectory`（≤256 路点）、
  `motor_policy`（enable/kp/kd）、`param_set`、`system_command`（SAVE/REFRESH/SET_ZERO/CLEAR_FAULT）。
- 桥：`motor_hardware_interface/scripts/firmware_topic_bridge.py` —— 上行
  `motion_states` → `/joint_states`，下行标准 `JointTrajectory` → `motor_trajectory`（桥侧负责
  拆分与超限拒绝；固件对非法轨迹**静默丢弃**，所以校验只能在桥上做）。
- 消息定义单源：`micro_ros_agent/src/can_bridge_msgs`（`ros2_ws/src/can_bridge_msgs` 与固件 `extra_packages/can_bridge_msgs` 都是指向它的符号链接）。

## 配置（单一来源）

拓扑是唯一真相，增删改电机零代码改动：

- **`config/motor_driver.yaml`（拓扑，唯一入口）**：`motor_specs_file` 引用型号库 + `timing`（反馈超时/激活窗口）+ `buses`（逻辑总线/传输）+ `motors`（`type` 引用型号、`joint`、`bus`、`id`、`command.kp/kd` 动态增益）。
- **`config/motor.yaml`（型号库）**：`motor_types.<型号>.protocol` 声明所属协议，`parameters` 是协议运行所需静态参数（如 `torque_constant_nm_per_amp`、`gear_ratio`）与安全阈值。与拓扑无关、可跨机器人复用。
- **`models/<模型名>/config/simulation_topology.yaml`（仿真叠加）**：只记参与该 Gazebo 模型仿真的电机子集、电机名 → Gazebo 关节映射与 plant 参数，引用上面两份的权威定义，不复制它们。

解析层把型号名经 `protocol` 字段映射为协议类型；参数默认值来自型号库，单电机 `parameters` 可覆盖。`bus_override`（launch 的 `interface_<总线名>`）只覆盖设备路径，**切换传输类型必须改 YAML**。

## launch 与 URDF 模型

- `launch/hardware.launch.py` 用 `OpaqueFunction` 延迟求值：运行时读取实际 `motor_config_file`，生成 position controller 的 joints、URDF ros2_control 关节清单（`control_joints`）与 `bus_overrides`——覆盖配置时自动一致。
- `urdf/canfd_hardware.xacro`：与拓扑无关的 ros2_control 硬件片段（`canfd_hardware`/`emit_bus_overrides`/`emit_control_joints` 宏），不写死任何关节。
- `urdf/upper_body.urdf.xacro`：物理模型（几何/惯性）**人工维护**；其 ros2_control 关节清单由 launch 注入，不手抄。两者一致性由 `test_xacro_consistency.py` 兜底。
- `motor_hardware_interface/helpers.py`：launch 与测试共用（读取 buses/joints、生成参数文件）。
- `robot_arm_gazebo/launch/gazebo.launch.py` 与 `display.launch.py` 用 `model:=<模型名>` 选
  `models/` 下的模型（默认 `chest_upper_00`），控制器参数挂该模型自己的
  `config/controllers.yaml`。`simulation.launch.py` 的 `model` 默认同为 `chest_upper_00`
  （`motor_config_file` 留空时优先用模型目录自带的 `config/motor_driver.yaml`）。
- `robot_arm_gazebo/launch/simulation.launch.py` 是端到端组合入口，配
  `scripts/simulation_gate.py`（全轴就绪才开放 effort）与 `scripts/agent_after_sim_ready.py`；
  `controller_mode:=firmware` 时默认连带启动 WebHost。模型是仿真资产：几何/惯性取自 CAD
  未标定，关节限位/力矩是占位值。

## 关键约定与坑

- **命名混淆点**：`vaxor` 协议族对应 25/30 mm 腕部电机；`protocol.hpp` 内部函数与常量仍用 `wrist_*` / `kWrist*` 命名，注释写「25/30 mm」。registry 的 type 字符串是 `vaxor`（见 `motor_protocol.hpp` 的 `kVaxorType`）。
- **单位边界**：所有物理量↔线协议字段换算集中在 `protocol.cpp`；上层统一用 rad、rad/s、A、Nm，协议单位不得渗透进 ROS 控制逻辑。`vaxor` 反馈帧的 `effort` 在 `protocol` 层暂存「电流 A」，转 Nm 由策略完成。Web 参数面以固件记录为准，力矩仍按 A 表达。
- **不写死拓扑**：电机数量/类型/ID/关节名/总线/传输只来自 YAML；扩展走 `MotorProtocolRegistry` / `CanTransportRegistry`，不在通用收发循环或 ROS 关节映射里加类型分支。
- **安全 fail-fast**：激活时零增益轮询反馈（窗口内最多重试 3 次）→ 实测位置作初始目标 → 统一使能；运行期反馈超时返回错误，`controller_manager` 立即终止。软件兜底不能替代硬件急停、机械限位与真机标定。
- **serial_slcan 权限**：需 tty 写权限（dialout/sudo）；`socketcan` 启动前 `ip link` 配置仲裁 1M/数据 5M；`vcan` 创建/启用需 `CAP_NET_ADMIN`。
- **代码规范**：C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进、snake_case（文件/函数/变量）、PascalCase（类）、`k` 前缀常量；Python snake_case 且测试带类型注解；中文注释写意图（数据流/单位/安全边界），协议名与 ROS 2 API 保留英文。
- **提交**：conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因；功能经 PR 合入 `dev/robot_ws`，`main` 仅发布，真机/仿真线分别同步 `dev-real` / `dev-sim`，PR 附验证方式。
