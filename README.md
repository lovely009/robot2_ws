# robot_ws

机器人的三区域工作空间（开发分支 `dev/robot_ws`）。三个区域**各自独立构建**，不是一个
colcon 工作区；改哪个区域就读哪个区域的文档。

| 区域 | 内容 | 构建系统 | 权威文档 |
|---|---|---|---|
| `ros2_ws/` | ROS 2 Humble 七个 ament 包：真机 + 仿真 + 动力学 + 消息 + Web 上位机 | colcon | [ros2_ws/README.md](ros2_ws/README.md) + 各包 `README.md` |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件 | PlatformIO | [micro_ros_firmware/README.md](micro_ros_firmware/README.md)（工程事实）+ [AGENTS.md](micro_ros_firmware/AGENTS.md)（操作手册） |
| `micro_ros_agent/` | micro-ROS Agent 独立 colcon 工作区与主机侧台架工具 | colcon（在该目录内独立构建） | [micro_ros_agent/README.md](micro_ros_agent/README.md) |

三个区域的构建与运行边界互相独立；改动前先读对应区域的文档。固件与 ROS 2 之间的
ID 化话题桥、消息单源位置见 `micro_ros_firmware/README.md`。

## 快速开始

```bash
# ROS 2 侧
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# 固件侧（PlatformIO，不能用 colcon）
cd micro_ros_firmware && ~/.platformio/penv/bin/pio run

# micro-ROS Agent（独立工作区）
cd micro_ros_agent && colcon build --symlink-install
```

## 分支

- `dev/robot_ws`：开发分支（本分支，真机 + 仿真 + 动力学 7 包）
- `dev-sim`：仿真线、`dev-real`：真机线、`main`：仅发布

## 文档地图

| 文档 | 内容 |
|---|---|
| [ros2_ws/README.md](ros2_ws/README.md) | ROS 2 区域：十二包、构建测试、bring-up、配置契约、扩展指南 |
| [micro_ros_firmware/README.md](micro_ros_firmware/README.md) | 固件区域：工程事实、话题契约、术语与所有权、开放项与开发计划 |
| [micro_ros_firmware/AGENTS.md](micro_ros_firmware/AGENTS.md) | 固件操作手册：构建、烧录、台架顺序与坑 |
| [micro_ros_agent/README.md](micro_ros_agent/README.md) | Agent 工作区与台架工具 |
| [docs/adr/](docs/adr/) | 架构决策记录（ros2_ws 与仓库级） |
| [docs/guides/fdcan1-bench-guide.md](docs/guides/fdcan1-bench-guide.md) | FDCAN 台架操作与只读验收流程 |
| [docs/research/](docs/research/) | 外部调研与参考 |
| [AGENTS.md](AGENTS.md) | Agent 即用指南：区域 × 权威文档映射、命令、代码与测试规范 |
| [CLAUDE.md](CLAUDE.md) | 仓库约定（Claude Code 自动加载） |

文档分类规则与状态词表见 [docs/README.md](docs/README.md)。
