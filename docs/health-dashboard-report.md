# robot_ws 代码健康度报告（Health Dashboard）

**Mode:** Health Dashboard
**Scope:** 整个仓库（4 个 ament 包，ROS 2 Humble 工作空间）；PR 维度 = develop 相对 main 的分支差异（40 个文件）
**Health Score:** 98/100
**Trend:** 95 → 98 (+3)（第二次运行；上一轮主要发现已在 develop 分支落地）
**生成日期:** 2026-08-11

## 给执行 AI 的说明

本报告是只读审查结果。每一条发现都遵循 Iron Law（Symptom → Source → Consequence → Remedy），
`Remedy` 字段可直接作为实现任务，报告结尾附有建议执行顺序与验证方式。

- 修改前先读根目录 `README.md`、`AGENTS.md` 与受影响包的 `README.md`。
- 遵守仓库规范：C++17、`-Wall -Wextra -Wpedantic`、2 空格缩进、注释用中文说明设计意图。
- 每完成一项修改后用 `colcon test && colcon test-result --verbose` 验证；测试不得启动 Gazebo 或硬件。
- 本报告不包含 Critical 级发现，剩余问题全部是低严重度重复与耦合，可按 Remedy 小步修改。

## 评分总览

| Dimension | Score | Top Finding |
|-----------|-------|------------|
| Code Quality | 99/100 | 默认配置路径在多个 launch/测试中重复拼写 |
| Architecture | 100/100 | Clean：单向四层、无环、bring-up 归属有 ADR 0003 支撑 |
| Tech Debt | 93/100 | 关节顺序仍有未受测试保护的副本（`driver_controllers.yaml`、`motion_library.py`） |
| Test Quality | 99/100 | 42 个测试全绿；3 个 launch 静态测试耦合 launch_ros 私有属性 |

## Module Dependency Graph

```mermaid
graph TD
    UBG[upper_body_canfd_gazebo<br/>模型 / URDF / launch / 动作库 (Python)]
    SIM[motor_canfd_sim<br/>C++ 模拟器 + 植物模型]
    HWI[motor_hardware_interface<br/>ros2_control 适配 + 真机 bring-up]
    DRV[motor_canfd_drive<br/>纯 C++ SocketCAN / 协议策略]
    UBG -->|exec_depend + xacro include 硬件片段| HWI
    UBG -->|exec_depend| SIM
    SIM -->|load_driver_config| HWI
    SIM -->|MotorCanfdDriver / SocketCan| DRV
    HWI -->|SystemInterface 插件 + YAML 解析| DRV
```

依赖全部单向、无环，且 ADR 0001–0004 对每条边都有记录（含 `motor_canfd_sim → motor_hardware_interface`
复用解析 API、真机入口下沉到适配层等权衡）。`upper_body.urdf.xacro` 通过
`$(find motor_hardware_interface)` include 适配层提供的 `canfd_hardware.xacro` 片段，方向符合 ADR 0003。

## Findings

### 🟡 Warning — Knowledge Duplication（Debt R3）: 关节清单仍有不受测试保护的副本

**Symptom:** 14 关节的有序清单存在于 5 处：真机与仿真两份
`motor_canfd.yaml`（`src/motor_hardware_interface/config/` 与 `src/motor_canfd_sim/config/`）、
`src/upper_body_canfd_gazebo/config/controllers.yaml`（effort 列表）、
`src/motor_hardware_interface/config/driver_controllers.yaml`（position 列表，含根命名空间与 `/driver` 命名空间两份）、
URDF（`src/upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro`），外加
`src/upper_body_canfd_gazebo/upper_body_canfd_gazebo/motion_library.py` 的 7 关节 `JOINT_NAMES`。
`src/upper_body_canfd_gazebo/test/test_config_topology.py` 已断言真机↔仿真拓扑、effort controller 顺序、
URDF 顺序三者一致，但 **`driver_controllers.yaml` 的 position 列表和 `motion_library.JOINT_NAMES` 不在任何测试覆盖内**。

**Source:** Fowler — Refactoring: Duplicate Code; Hunt & Thomas — The Pragmatic Programmer: DRY（重复的是决策）

**Consequence:** 新增/重排一个关节时，前 5 处漏改会被测试抓住，但 position 控制器或动作库漏改仍会静默——命令发给错误关节或动作越限到运行时才暴露。

**Remedy:** 在 `test_config_topology.py` 增加两条断言：
1. `driver_controllers.yaml` 的 `position_controller.ros__parameters.joints` 列表 == 仿真配置 `motor_hardware.motors` 的 `joint` 顺序；
2. `motion_library.JOINT_NAMES` == 配置中任一臂的关节前缀顺序（左臂 7 个）。

### 🟢 Suggestion — Knowledge Duplication（Debt R3）: 模拟器帧处理仍手写“按电机类型分发”

**Symptom:** 植物模型已抽为 `MotorPlantModel`
（`src/motor_canfd_sim/include/motor_canfd_sim/plant_model.hpp`），但
`src/motor_canfd_sim/src/motor_canfd_sim_node.cpp` 的 CAN 帧路径仍有 6 处
`type == kRpMotorType / kMotor2530Type` 分支（构造校验、`handle_frame` 三处、`send_rp_feedback`、
`send_motor_2530_reply`、`status_tick`），而 `motor_canfd_drive` 早已用 `MotorProtocol` 策略封装同一分发。

**Source:** Fowler — Refactoring: Parallel Inheritance Hierarchies; DRY

**Consequence:** 新增电机家族时，driver 侧注册一个策略即可，但仿真节点仍要同步改 6 个分支，两处对“电机类型”的知识继续各自演进。

**Remedy:** 将帧级处理也按家族封装（例如每个家族一个 `SimMotorHandler`，负责命令帧解析与反馈帧构造），与 driver 的 `MotorProtocol` 一一对应；节点只做路由与 ROS 编排。

### 🟢 Suggestion — Change Propagation（Debt R2）: `motor_canfd.yaml` 被解析两次

**Symptom:** `src/motor_hardware_interface/src/canfd_system.cpp::load_config` 先调用
`load_driver_config(path)`（内部 `YAML::LoadFile` 一次），随后又对同一路径 `YAML::LoadFile` 一次
读 `timing` 与 `command` 段，且 `document["motor_hardware"]` 根节点校验在
`src/motor_hardware_interface/src/motor_config.cpp` 与 `canfd_system.cpp` 两处各写一遍。

**Source:** Ousterhout — A Philosophy of Software Design: Information Leakage（同一文件格式知识编码在两个模块）

**Consequence:** 每次启动多付一次 YAML 解析；若某段 schema 只在一处演进，另一处会静默读到空值。

**Remedy:** 让 `load_driver_config` 返回携带完整解析结果的结构（或提供“一次加载、按需取段”的 API），适配层不再二次加载同一文件。

### 🟢 Suggestion — Change Propagation（PR R2）: 默认配置路径在多个 launch/测试中重复拼写

**Symptom:** 仿真默认配置路径
`PathJoinSubstitution([FindPackageShare("motor_canfd_sim"), "config", "motor_canfd.yaml"])`
在 `src/motor_canfd_sim/launch/motor_canfd_sim.launch.py`、
`src/upper_body_canfd_gazebo/launch/model_sim.launch.py`、
`src/upper_body_canfd_gazebo/launch/driver_loopback.launch.py` 与对应测试
（`src/motor_canfd_sim/test/test_motor_canfd_sim_launch.py`、
`src/upper_body_canfd_gazebo/test/test_model_sim_launch.py`）中各自拼写；
真机默认路径同理出现在 `src/motor_hardware_interface/launch/hardware.launch.py` 与
`src/motor_hardware_interface/test/test_hardware_launch.py`。

**Source:** Fowler — Refactoring: Shotgun Surgery; Brooks — The Mythical Man-Month: 变更传播半径

**Consequence:** 下次切换配置归属或新增 launch 时仍需逐处复制路径，漏改一处即产生 launch 间不一致。

**Remedy:** 提供公共 helper（例如 `motor_canfd_sim` 导出“默认仿真配置路径”的 Python 函数或 launch 片段），launch 与测试统一引用。

### 🟢 Suggestion — Test Brittleness（Test T2）: launch 静态测试依赖 launch_ros 私有成员

**Symptom:** `src/motor_canfd_sim/test/test_motor_canfd_sim_launch.py`、
`src/upper_body_canfd_gazebo/test/test_model_sim_launch.py` 通过 `simulator._Node__parameters`，
`src/upper_body_canfd_gazebo/test/test_driver_loopback_launch.py` 通过
`_OnActionEventBase__action_matcher`、`_Node__arguments` 等私有属性断言参数注入与事件链。

**Source:** Osherove — The Art of Unit Testing: 测试隔离（实现耦合）

**Consequence:** launch_ros 上游内部改名时，行为未变测试也会批量失败，套件对 Humble 补丁升级敏感。
（这些测试验证的是可观察契约，私有 API 是当前唯一可行的静态 seam，属于合理权衡——问题只在它散落在三个文件里。）

**Remedy:** 把私有 API 访问集中到仓库内一个测试辅助模块（如 `test/launch_helpers.py`），
集中标注升级注意事项，未来一次性替换为公开 API。

## Test Suite Map

| 层 | 测试 | 数量 |
|---|---|---|
| Unit（C++ gtest） | `motor_canfd_drive` 8 + `motor_hardware_interface` 4 + `motor_canfd_sim` 植物模型 16 | 28 |
| 静态/轻量（pytest） | `upper_body_canfd_gazebo` 12 + `motor_canfd_sim` 1 + `motor_hardware_interface` 1 | 14 |

最新一轮 `colcon test`（2026-08-11 12:31）共 42 个测试全部通过。`test_plant_model.cpp`
覆盖 MIT 扭矩、限幅、失能/超时归零、前馈、腕部三模式、置零重基、错误参数等 16 个场景。

**剩余覆盖缺口：** `src/motor_hardware_interface/src/canfd_system.cpp`（380 行，激活时序与反馈超时等
安全关键路径）和 `src/motor_canfd_drive/src/motor_protocol.cpp` 的真实协议策略仍无直接单元测试。

## 已解决项（相对上一轮报告）

- 植物模型从 442 行节点中抽出为 `MotorPlantModel`，并新增 16 个单测。
- 真机入口下沉到 `motor_hardware_interface`（ADR 0003），`driver_loopback` 与真机共用同一份控制器配置。
- 仿真/真机配置解耦（ADR 0002）并有 `test_config_topology.py` 兜底拓扑一致性。
- 激活失败与回环终止采用 fail-fast 语义（ADR 0004），launch 事件链有时序测试。

## Summary

仓库整体健康：四层单向依赖清晰、文档与 ADR 齐备、42 个测试全绿，无 Critical 发现。剩余问题集中在
“新增一个关节或电机家族时仍要同步改多个文件”这一类重复耦合上，且大部分已有测试部分兜底。
最值得先做的是把 `test_config_topology.py` 扩成完整防线，其次补 `canfd_system.cpp` 的安全关键路径单测。

## 建议执行顺序与验证

| 优先级 | 任务 | 验证方式 |
|---|---|---|
| P0 | 扩展 `test_config_topology.py`：断言 `driver_controllers.yaml` position 列表与 `motion_library.JOINT_NAMES` 顺序 | `colcon test` 中 `upper_body_canfd_gazebo` pytest 通过 |
| P1 | 为 `canfd_system.cpp` 的激活时序/反馈超时/关节映射补 gtest（提取可测 seam，如把超时判断与映射抽成纯函数） | 新增 gtest 通过 |
| P1 | 抽取 `SimMotorHandler` 帧处理策略，消除模拟器 6 处类型分支 | 新增/原有 gtest 通过，`motor_canfd_sim` 构建通过 |
| P2 | 统一 `load_driver_config` 一次解析，消除双 LoadFile | `test_motor_config` 与 `test_hardware_launch` 通过 |
| P2 | 默认配置路径收敛到公共 helper | 三个 launch 静态测试通过 |
| P3 | launch 测试私有 API 集中到辅助模块 | 全部 pytest 通过 |

统一验证命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```
