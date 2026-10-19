# CLAUDE.md

本文件给 Claude Code 用的仓库约定。**区域事实不在本文件复述**——每个区域一份权威文档，
本文件只写"改动时的规矩"与"去哪找事实"。

## 这是什么

`robot_ws` 是**三个独立构建区域**，不是一个 colcon 工作区：

| 区域 | 内容 | 构建 | 权威文档 |
|---|---|---|---|
| `ros2_ws/` | ROS 2 Humble 十一包（真机 + 仿真 + 动力学 + 消息 + 台架管理 + Web 上位机） | colcon | [ros2_ws/README.md](ros2_ws/README.md) + 各包 `README.md` + [docs/adr/](docs/adr/) |
| `micro_ros_firmware/` | STM32H723VGT6 micro-ROS 固件 | PlatformIO | [README.md](micro_ros_firmware/README.md)（事实）、[AGENTS.md](micro_ros_firmware/AGENTS.md)（操作）、[decision-log](micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md)（理由）、[lib/README.md](micro_ros_firmware/lib/README.md)（库契约） |
| `micro_ros_agent/` | micro-ROS Agent 与主机侧台架工具 | colcon（目录内独立） | [README.md](micro_ros_agent/README.md) |

分支：`dev/robot_ws` 开发（本分支）、`dev-sim` 仿真线、`dev-real` 真机线、`main` 仅发布。
文档地图与"每份事实的唯一宿主"见 [docs/README.md](docs/README.md)；Agent 即用指南见
[AGENTS.md](AGENTS.md)。

## 构建与测试

```bash
# ROS 2（十一包）
source /opt/ros/humble/setup.bash && cd ros2_ws
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
colcon build --symlink-install --packages-select motor_drive      # 单包
colcon test --packages-select motor_drive --ctest-args -R test_protocol   # 单个 gtest
colcon test --packages-select motor_hardware_interface --pytest-args "-k <表达式>"

# 固件（PlatformIO，不能用 colcon）
cd micro_ros_firmware
~/.platformio/penv/bin/pio run                    # 编译；-t upload 烧录（ST-Link）
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure  # host 测试：CTest + cassert，无 gtest
```

- 改 `platformio.ini` 编译宏或 `ldscript/*.ld` 后必须 `pio run -t clean`（HAL SrcWrapper
  对象不重编；增量构建不监听链接脚本改动，会静默跳过重新链接）；**勿跑 `-t clean_microros`**
  （联网重下环境）。
- 测试不得启动 Gazebo、不得开 MuJoCo 窗口、不得占 vcan/串口或真实硬件。
- 固件 host 测试直接编译固件真实 `.c`、无桩无复制；新增内核 `.c` 只改
  `test/host/CMakeLists.txt` 一处（`fw_host_test()` 宏即注册 CTest）。

## 改动规矩

- **不写死拓扑**：电机数量/类型/ID/关节名/总线/传输只来自 YAML。扩展协议走
  `MotorProtocolRegistry`、扩展传输走 `CanTransportRegistry`，不在通用收发循环或 ROS 关节
  映射里加类型分支。
- **单位边界**：协议单位不得渗透进 ROS 控制逻辑。物理量↔线协议字段的换算集中在
  `motor_drive` 的 `protocol.cpp`；固件侧集中在 `lib/motor_protocol` 与型号 Profile。
  上层统一用 rad、rad/s、A、Nm。
- **安全 fail-fast**：激活时零增益轮询反馈（窗口内最多重试 3 次）→ 实测位置作初始目标 →
  统一使能；运行期反馈超时返回错误，`controller_manager` 立即终止。软件兜底**不能**替代
  硬件急停、机械限位与真机标定。
- **单写者所有权**：固件侧电机运行时、正常 CAN TX、W25Q64 各只有一个所有者；ROS 侧
  controller joints 与 URDF 清单由 launch 从配置生成，不手抄。
- **文档**：一份事实只写一处（宿主表见 [docs/README.md](docs/README.md)）。过程性产物
  （实施计划、任务台账、逐次运行报告）不进仓库，结论并入宿主文档。
- **提交**：conventional 前缀（`feat:`/`fix:`/`refactor:`/`docs:`/`test:`），正文写原因；
  功能经 PR 合入 `dev/robot_ws`，`main` 仅发布；PR 附验证方式。
- **注释**：只写意图（数据流 / 单位 / 安全边界 / 非直觉陷阱），**不写**设计论证、变更沿革、
  实测排查过程或与他处重复的事实——那些归 ADR、decision-log 与区域 README。叙述性注释块
  控制在 5 行内；枚举/字段/话题这类纯契约清单可略长，但同样不写散文。

## 环境坑

- `serial_slcan` 需 tty 写权限（`dialout`/sudo）；`socketcan` 启动前 `ip link` 配仲裁
  1M/数据 5M；`vcan` 创建/启用需 `CAP_NET_ADMIN`。设备路径一律按 `/dev/serial/by-id/` 区分
  （`ttyACM` 编号会交换）。
- `web_host` 的 `--symlink-install` **不能**兜住 `setup.py` 清单改动：新增/改名
  `console_scripts` 或 `web/` 下**新增文件**都必须重跑 colcon build，否则分别报
  `executable not found` 与页面 404。
- 固件改 `.msg` 后 `pio run` 不会重生成类型，用
  `micro_ros_agent/tools/refresh_microros_messages.py` 增量刷新，勿跑 `-t clean_microros`。
