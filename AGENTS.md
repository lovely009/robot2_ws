# Repository Guidelines

`robot_ws` 是**三区域工作空间**，各区域独立构建。本文件是 **Agent 即用指南**：
区域 → 权威文档的映射、命令、以及改代码时的规范。区域事实只在各区域文档里写一份，
本文件不复述。

| 区域 | 内容 | 权威文档 |
|---|---|---|
| `ros2_ws/` | ROS 2 Humble 七包：消息、真机、仿真、动力学与 Web 上位机 | [ros2_ws/README.md](ros2_ws/README.md) + 各包 `README.md` + [docs/adr/](docs/adr/) |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS） | [README.md](micro_ros_firmware/README.md)（工程事实/开放项）、[AGENTS.md](micro_ros_firmware/AGENTS.md)（构建/烧录/台架顺序与坑）、[decision-log](micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md)（设计理由）、[lib/README.md](micro_ros_firmware/lib/README.md)（库分层与契约） |
| `micro_ros_agent/` | micro-ROS Agent 独立 colcon 工作区与主机侧台架工具 | [README.md](micro_ros_agent/README.md) |

文档分类规则与「每份事实的唯一宿主」见 [docs/README.md](docs/README.md)。

## 命令

```bash
# ros2_ws
source /opt/ros/humble/setup.bash && cd ros2_ws
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
ros2 launch motor_hardware_interface hardware.launch.py       # 真机，默认 tty 双总线
ros2 launch robot_sim gazebo.launch.py [model:=<模型名>]       # 纯 Gazebo
ros2 launch robot_sim mujoco.launch.py [model:=<模型名>]       # 纯 MuJoCo
ros2 launch robot_sim simulation.launch.py backend:=mujoco    # 端到端闭环

# micro_ros_firmware（非 ament 包，不能用 colcon）
cd micro_ros_firmware
~/.platformio/penv/bin/pio run
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure

# micro_ros_agent（在该目录内独立构建）
cd micro_ros_agent && colcon build --symlink-install
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v
```

无客户端时 agent 节点不出现在 `ros2 node list`，属正常（客户端建立会话后才注册）。

## 结构与入口

- `ros2_ws/src/<package>` 是标准 ament 布局（按需 `urdf/`、`scripts/`、`models/<模型名>/`）；
  launch 与测试共用 `motor_hardware_interface` 的 `helpers.py`（buses 读取、joints 生成、
  参数文件写入）。
- 协议 PDF 与型号摘要（`cerb-series.md`、`motor-2530-series.md`）在
  `ros2_ws/src/motor_drive/docs/`；固件与板卡的厂商原件在 `micro_ros_firmware/docs/`。

## 代码规范

- C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进；snake_case（文件/函数/变量）、PascalCase
  （类）、`k` 前缀常量。Python snake_case，测试带类型注解。
- 中文注释只写意图（数据流 / 单位 / 安全边界 / 非直觉陷阱），不写设计论证、变更沿革或实测
  过程；协议名与 ROS 2 API 保持英文。单个注释块不超过 5 行。

## 测试规范

- C++ 用 GoogleTest（`test/test_*.cpp`）；Python 用 pytest（`test/test_*.py`，函数名
  `test_<behavior>()`）。固件 host 测试是 CTest + `cassert`（无 gtest）。
- 测试不得启动 Gazebo、开 MuJoCo 窗口、占 vcan/串口或真实硬件；统一
  `colcon test && colcon test-result --verbose`。

## 提交与 PR

- conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因。
- 功能经 PR 合入 `dev/robot_ws`；`main` 仅发布；真机/仿真线分别同步 `dev-real` / `dev-sim`；
  PR 附验证方式（如 `colcon test` 结果）。

## Agent 注意事项

- 优先找现成方案：动手前先查已有 skill/工具/依赖与各区域 README，确认没有现成再实现；
  能用成熟库就别手写核心逻辑。
- 不写死电机数量/类型/总线：拓扑只来自 YAML；扩展协议走 `MotorProtocolRegistry`、扩展传输走
  `CanTransportRegistry`，不改通用收发循环。`bus_override` 只覆盖设备路径，切换传输类型必须
  改 YAML。
- URDF 物理模型（几何/惯性）人工维护；ros2_control 关节清单由 launch 自动生成，不手写。
- 软件安全逻辑不能替代硬件急停、机械限位与真机标定；固件的自动测试不得烧录或驱动电机。
- 一份事实只写一处（宿主表见 `docs/README.md`）；过程性文档不进仓库。
