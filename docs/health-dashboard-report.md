# Brooks-Lint Health Dashboard

**Mode:** Health Dashboard
**Scope:** 整个仓库（4 个 ament 包，ROS 2 Humble 工作空间）；PR 维度 = develop 相对 main 的分支差异（48 个文件，+1861/−381）
**Composite Score:** 100/100
**Trend:** 98 → 100 (+2)（第三次运行；上一轮 Warning 与 Suggestion 已全部在 develop 落地）
**生成日期:** 2026-08-11

## 给执行 AI 的说明

本报告是只读审查快照，不直接修改代码。若实施 `Remedy`，请遵守仓库规范（C++17、`-Wall -Wextra -Wpedantic`、中文注释、TDD）并运行 `colcon test && colcon test-result --verbose` 验证。测试不得启动 Gazebo 或硬件。

## 评分总览

| Dimension | Score | Top Finding |
|-----------|-------|------------|
| Code Quality (PR) | 100/100 | Clean：近四批改动均带测试与 ADR，无 R1/R2 新发现 |
| Architecture | 100/100 | Clean：单向四层、无环、家族扩展 seam 完整 |
| Tech Debt | 99/100 | 动作库限位与 URDF 双写，无跨源测试 |
| Test Quality | 99/100 | 94 项全绿；内置协议帧生成仍以字段级测试间接覆盖 |

Composite = 100×0.25 + 100×0.30 + 99×0.25 + 99×0.20 = 99.55 → **100/100**。

## Module Dependency Graph

```mermaid
graph TD
    UBG[upper_body_canfd_gazebo<br/>模型 / URDF / launch / 动作库 (Python)]
    SIM[motor_canfd_sim<br/>C++ 模拟器 + plant model]
    HWI[motor_hardware_interface<br/>ros2_control 适配 + 真机 bring-up]
    DRV[motor_canfd_drive<br/>纯 C++ SocketCAN / 协议策略]
    UBG -->|exec_depend + xacro include 硬件片段| HWI
    UBG -->|exec_depend| SIM
    SIM -->|load_driver_config 解析复用| HWI
    SIM -->|MotorCanfdDriver / SocketCan| DRV
    HWI -->|SystemInterface 插件 + YAML 解析| DRV

    classDef clean fill:#51cf66,stroke:#2b8a3e,color:#fff
    class UBG,SIM,HWI,DRV clean
```

依赖全部单向、无环；`motor_canfd_sim → motor_hardware_interface` 仅复用解析 API，
`upper_body.urdf.xacro` 通过 `$(find motor_hardware_interface)` include 适配层提供的
`canfd_hardware.xacro` 片段，方向均符合 ADR 0001–0004。

## Top Findings

### 🟢 Suggestion — Knowledge Duplication（R3）: 动作库限位与 URDF 双写，无跨源测试

**Symptom:** `upper_body_canfd_gazebo/upper_body_canfd_gazebo/motion_library.py` 的
`JOINT_LIMITS` 与 `urdf/upper_body.urdf.xacro` 的七个关节限位逐对相同（-1.57…2.80 rad），
注释声明“与 URDF 保持一致”，但 `test_motion_library.py` 只用 `validate_pose` 做自校验，
`test_config_topology.py` 只比对关节名，没有跨源断言限位相等。

**Source:** Fowler — Refactoring: Duplicate Code; Hunt & Thomas — The Pragmatic Programmer: DRY（重复的是决策）

**Consequence:** 调整 URDF 限位（如放开肩关节行程）后，动作库仍在旧限位处提前拦截，姿态越限或动作不可达要到运行时才暴露。

**Remedy:** 在 `test_config_topology.py` 增加跨源断言：从 URDF xacro 提取各 `canfd_control_joint` 对应的 limit，与 `motion_library.JOINT_LIMITS` 逐对相等。

### 🟢 Suggestion — Coverage Illusion（T5）: 内置协议策略帧生成缺少直接单测

**Symptom:** `test_protocol.cpp` 覆盖字段级编解码与一次“模拟器回包 → 驱动解码”回环，但
`RpProtocol` / `Motor2530Protocol` 的 `make_enable_frames()` / `make_command_frames()` 没有
“给定 motors + indices + commands → 断言帧 ID / 槽位布局 / 载荷”的直接用例；
`test_driver.cpp` 的 `DummyProtocol` 只验证注册机制与配置校验。

**Source:** Feathers — Working Effectively with Legacy Code: 未覆盖路径; Osherove — The Art of Unit Testing: 边界路径完整性

**Consequence:** 使能广播槽位、命令帧 ID 或字节布局回归时，单元层无法发现，只能靠手动回环或真机暴露。

**Remedy:** 为两种内置协议补 gtest：构造 `motors + indices + commands` 后调用帧生成接口，按协议手册例值断言帧数、CAN ID、槽位布局，并做反馈解码回环。

## Recommendation

98 → 100 的跃升来自近四批重构全部落地：joints 单一来源、YAML 一次解析、模拟器家族策略化
（`SimMotorHandler`）与拓扑一致性测试。剩余两条均为低严重度，建议先补协议帧生成单测
（涉及安全使能路径、风险最高），再补动作库限位跨源断言；均可在 1 个 PR 内完成。
