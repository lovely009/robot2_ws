# 仿真重力补偿前馈实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `simulation.launch.py` 的 host/external 组合能按开关把 Pinocchio 计算的重力矩叠加到 Gazebo 的 effort 命令上，用于观察 effort 闭环在重力下的稳态落差。

**Architecture:** `robot_dynamics` 新增 `ArmModel`（URDF → `pinocchio::Model`，给出关节侧重力矩）与 `gravity_torque_node`（订阅 `/joint_states`，发布 `sensor_msgs/JointState` 形式的 τ_g）；`robot_arm_gazebo` 新增 `gravity_feedforward_mixer.py`，在 `motor_sim` 的力矩与 τ_g 之间求和后送 effort 控制器，并按控制门与输入超时归零。默认关闭，现有接线不变。

**Tech Stack:** C++17、ROS 2 Humble / rclcpp、Pinocchio 4.0.0（`ros-humble-pinocchio`）、`sensor_msgs`、`std_msgs`、Python 3 / rclpy、pytest、ament_cmake / ament_cmake_gtest / ament_cmake_pytest。

**Spec:** `docs/superpowers/specs/2026-10-08-gravity-feedforward-design.md`
**ADR:** `docs/adr/0009-simulation-gravity-feedforward-boundary.md`

## Global Constraints

- 功能只在 `simulation.launch.py` 的 `gravity_feedforward:=true` 下生效；默认 `false`，`motor_sim` 的 `effort_topic` 与节点集合保持现状。
- 只支持 `controller_mode ∈ {host, external}` 且 `gravity:=true`；`firmware` 模式与 `gazebo.launch.py` 不在本轮。
- `robot_dynamics` 不在编译期依赖 `robot_arm_gazebo` 之外的任何机器人包，也不参与 ros2_control 实时路径；`motor_drive`、`motor_sim`、`motor_hardware_interface`、模型 URDF/xacro、`gazebo.launch.py`、`web_host`、固件与 Agent 一律不改。
- 单位边界：rad、Nm，关节侧；不做减速比与方向换算，不把协议单位带进叠加层。
- 前馈只在控制门开启时叠加；门关、输入超时、长度不符一律输出零。任何降级都不得放大输出。
- 自动测试不得启动 Gazebo、不得连接真实硬件、不得占用 vcan 或串口。
- C++ 保持 C++17、2 空格缩进、`-Wall -Wextra -Wpedantic`、snake_case / PascalCase / `k` 前缀常量；Python snake_case 且测试带类型注解；新增说明用中文。
- 修改前后都不得覆盖、暂存或回滚工作区里已有的用户改动。

---

### Task 1: `robot_dynamics` 新增 `ArmModel` 重力矩计算

**Files:**
- Create: `ros2_ws/src/robot_dynamics/include/robot_dynamics/arm_model.hpp`
- Create: `ros2_ws/src/robot_dynamics/src/arm_model.cpp`
- Create: `ros2_ws/src/robot_dynamics/test/test_arm_model.cpp`
- Create: `ros2_ws/src/robot_dynamics/test/models/pendulum.urdf`
- Create: `ros2_ws/src/robot_dynamics/test/models/pendulum_continuous.urdf`
- Modify: `ros2_ws/src/robot_dynamics/CMakeLists.txt`

**Interfaces:**
- Consumes: Pinocchio 的 `pinocchio::urdf::buildModel`、`pinocchio::rnea`、`pinocchio::centerOfMass`。
- Produces: `class robot_dynamics::ArmModel`：`explicit ArmModel(const std::string &urdf_path)`；`const std::vector<std::string> &joint_names() const`；`std::size_t nv() const`；`std::vector<double> gravity_torque(const std::vector<double> &q)`。构造失败抛 `std::runtime_error`，`q` 长度不符抛 `std::invalid_argument`。

- [ ] **Step 1: 写单摆夹具 URDF**

`test/models/pendulum.urdf`（已离线核对：Pinocchio 解析得 `nq = nv = 1`，关节名 `pivot`，
`τ_g` 与解析式在 −1.0 / −0.3 / 0.0 / 0.4 / 1.2 rad 上逐位一致）：

```xml
<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.001"/>
      <inertia ixx="1e-06" ixy="0" ixz="0" iyy="1e-06" iyz="0" izz="1e-06"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="pivot" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="10.0" velocity="10.0"/>
  </joint>
</robot>
```

两个 link 都写 `<inertial>`：`base` 是根连杆、质心不随 q 变化，给个微小质量即可，缺 inertial
会让解析行为依赖 Pinocchio 版本。按此几何 `z_com(q) = −0.2·sin q`、`U = m·g·z_com`，解析
重力矩为 **`τ_g(q) = −m·g·l·cos(q) = −0.981·cos(q)` Nm**（`g = 9.81`、`l = 0.2`；实测
`q = 0` 时 `τ_g = −0.981`）。

`test/models/pendulum_continuous.urdf`：同一个摆，关节改成 `type="continuous"`、去掉
`<limit>`。Pinocchio 把它建成 `JointModelRUBY`，`nq = 2`、`nv = 1`（已核对），用于验证拒绝
路径。

```
- [ ] **Step 2: 写失败的测试**

`test/test_arm_model.cpp` 至少覆盖：

```cpp
TEST(ArmModel, MatchesAnalyticPendulumTorque) {
  robot_dynamics::ArmModel model(fixture_path("pendulum.urdf"));
  ASSERT_EQ(model.joint_names().size(), 1U);
  EXPECT_EQ(model.joint_names().front(), "pivot");
  for (const double q : {-1.0, -0.3, 0.0, 0.4, 1.2}) {
    const auto tau = model.gravity_torque({q});
    ASSERT_EQ(tau.size(), 1U);
    EXPECT_NEAR(tau[0], -0.5 * 9.81 * 0.2 * std::cos(q), 1e-9);
  }
}

TEST(ArmModel, GravityTorqueMatchesPotentialGradient) { /* 中心差分 ∂U/∂q，容差 1e-7 */ }

TEST(ArmModel, RejectsWrongConfigurationSize) { /* 长度不符 → std::invalid_argument */ }
TEST(ArmModel, RejectsMissingFile) { /* std::runtime_error */ }
TEST(ArmModel, RejectsNonSingleDofJoint) { /* continuous 夹具 → std::runtime_error */ }
```

符号约定以解析式为准：如果实测符号与 `-m·g·l·cos(q)` 相反，**不要改测试期望值**，先查夹具的
关节轴与质心方向（`∂U/∂q` 的有限差分是仲裁者）。

- [ ] **Step 3: 让 `robot_dynamics` 从 INTERFACE 库变成可编译的库**

`CMakeLists.txt`：保留 `find_package(ament_cmake REQUIRED)`、`find_package(pinocchio REQUIRED)` 与
现有的 `test_pinocchio_dependency`；把 `add_library(${PROJECT_NAME} INTERFACE)` 换成带
`src/arm_model.cpp` 的普通库，`target_include_directories(... PUBLIC include ...)`，
`target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)`，
`target_compile_options(... PRIVATE -Wall -Wextra -Wpedantic)`；`install(TARGETS ... EXPORT ...)`
与 `ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)`、
`ament_export_dependencies(pinocchio)` 保持；新增
`ament_add_gtest(test_arm_model test/test_arm_model.cpp)`（`cxx_std_17`、同样的 warning 选项、
链接 `${PROJECT_NAME}`）。

- [ ] **Step 4: 实现 `ArmModel`**

`src/arm_model.cpp` 要点：

- 构造时 `pinocchio::urdf::buildModel(urdf_path, model_)`；文件不存在或解析抛错时转成
  `std::runtime_error`，带上路径。
- 逐关节检查：**从下标 1 开始**，每个关节必须 `nq == nv == 1`，否则抛
  `std::runtime_error`。已核对的实测：本模型 4 个关节全部满足（`joint_link2` 是
  `JointModelRZ`，其余为 `JointModelRevoluteUnaligned`）；`pendulum_continuous` 的
  `JointModelRUBY` 是 `nq = 2 / nv = 1`，必须被拒。注意 `model.joints[0]` 是占位项
  （实测其 `shortname()` 为 `JointModelRX`，与真实首关节无关），不要用它做判断。
- `joint_names()` 用 `model_.names` 里除 universe 之外的名字；`nv()` 返回 `model_.nv`。
- `gravity_torque(q)` 校验 `q.size() == model_.nv`，构造 `Eigen::VectorXd`，跑
  `pinocchio::rnea(model_, data_, q, zero, zero)`，拷回 `std::vector<double>`。
- `data_` 是成员，`gravity_torque` 不是 `const`；头文件注释写明「一个实例一个线程」。

- [ ] **Step 5: 跑测试**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install --packages-select robot_dynamics && source install/setup.bash
colcon test --packages-select robot_dynamics && colcon test-result --verbose
```

---

### Task 2: `robot_dynamics` 新增 `gravity_torque_node`

**Files:**
- Create: `ros2_ws/src/robot_dynamics/src/gravity_torque_node.cpp`
- Modify: `ros2_ws/src/robot_dynamics/CMakeLists.txt`
- Modify: `ros2_ws/src/robot_dynamics/package.xml`
- Modify: `ros2_ws/src/robot_dynamics/README.md`

**Interfaces:**
- Consumes: `robot_dynamics::ArmModel`；参数 `model_file`（必填）、`joint_states_topic`
  （默认 `/joint_states`）、`gravity_torque_topic`（默认 `/robot_dynamics/gravity_torque`）。
- Produces: 可执行 `robot_dynamics/gravity_torque_node`；话题 `gravity_torque_topic`，
  类型 `sensor_msgs/msg/JointState`，`name` = 模型关节名（固定），`effort` = τ_g（Nm），
  `header.stamp` = 输入时间戳，`position` / `velocity` 留空。

- [ ] **Step 1: 实现节点**

要点：

- 启动即读 `model_file`：为空、文件不存在或解析失败 → 记 ERROR 并抛出/退出（launch 随之失败），
  不做静默降级。
- 订阅 `joint_states_topic`；回调里按 `name → position` 建映射，取每个模型关节：
  缺任一关节或值非有限 → WARN（同一条原因限频，例如 1 秒一次），**本周期不发布**。
- 计算 `τ_g`，全部有限才发布；不发布零，也不做超时。
- 节点不订阅 `motor_sim` 的任何话题，不引用 Gazebo 或 ros2_control 相关类型。
- 日志与注释用中文，说明数据流与单位。

- [ ] **Step 2: 构建与手工冒烟（不启 Gazebo）**

```bash
cd ros2_ws && colcon build --symlink-install --packages-select robot_dynamics && source install/setup.bash
# 另开终端：发布一个位姿，检查输出
ros2 run robot_dynamics gravity_torque_node --ros-args \
  -p model_file:=$(pwd)/src/robot_arm_gazebo/models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.standalone.urdf
ros2 topic echo /robot_dynamics/gravity_torque
```

用 `[0.5, 0, 0, 0]` 的 `/joint_states` 输入核对输出与设计文档中的实测值
（`joint_link1 ≈ 0.896 Nm`、`joint_link3 ≈ 0.130 Nm`）。这一步只手动跑，不作为自动化测试。

---

### Task 3: `robot_arm_gazebo` 新增重力前馈叠加节点

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/scripts/gravity_feedforward_mixer.py`
- Create: `ros2_ws/src/robot_arm_gazebo/test/test_gravity_feedforward_mixer.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/CMakeLists.txt`

**Interfaces:**
- Consumes: 参数 `joints`（必填，有序）、`effort_input_topic`（默认 `/motor_sim/effort`）、
  `effort_output_topic`（默认 `/arm_effort_controller/commands`）、`gravity_torque_topic`
  （默认 `/robot_dynamics/gravity_torque`）、`drive_state_topic`（默认 `/motor_sim/drive_state`）、
  `input_timeout_ms`（默认 100）、`publish_rate_hz`（默认 100）；话题
  `std_msgs/msg/Float64MultiArray`（输入、输出）、`sensor_msgs/msg/JointState`（τ_g）、
  `std_msgs/msg/String`（驱动状态）。
- Produces: 纯函数

```python
class MixOutcome(NamedTuple):
    efforts: list[float]
    reason: str  # 'ok' | 'gate_closed' | 'effort_stale' | 'size_mismatch' | 'gravity_stale'

def mix_efforts(
    effort: Sequence[float] | None,
    gravity: Mapping[str, float] | None,
    joints: Sequence[str],
    gate_open: bool,
) -> MixOutcome: ...
```

`effort is None` / `gravity is None` 表示对应输入已超时（由节点判定）。

- [ ] **Step 1: 先写纯函数测试**

覆盖：正常叠加（含 `gravity` 里多出的关节名被忽略）、门控关闭 → 全零、`effort is None` →
全零、`len(effort) != len(joints)` → 全零、`gravity is None` → 直通、`gravity` 缺某个关节 →
该关节直通其余照常、`joints` 为空 → 直接报错（`ValueError`，属于配置错误）。

- [ ] **Step 2: 实现节点与纯函数**

- 纯函数只做上表的规则，不做 ROS 调用、不读时间。
- 节点：`joints` 为空或字段缺失 → 启动即抛错（配置错误，不猜）；订阅三个输入，记录各自的
  到达时间，用 `steady_clock` 判超时；`drive_state` 取值 `ready_for_control` /
  `motor_control_active` 为开，其余为关，初值关。
- 定时器按 `publish_rate_hz` 发布（不依赖输入到达，保证输入停止后能归零）。
- `reason` 变化时记一次日志（中文），正常叠加回到 `ok` 时也记一次。
- 退出时不做「最后一次发布」以外的补救；由 launch 在节点退出时整体关闭。

- [ ] **Step 3: `CMakeLists.txt` 与 `package.xml`**

`install(PROGRAMS ... scripts/gravity_feedforward_mixer.py ...)`；新增
`ament_add_pytest_test(test_gravity_feedforward_mixer test/test_gravity_feedforward_mixer.py)`；
`package.xml` 增加 `<exec_depend>robot_dynamics</exec_depend>`。

- [ ] **Step 4: 跑测试**

```bash
colcon build --symlink-install --packages-select robot_arm_gazebo
colcon test --packages-select robot_arm_gazebo && colcon test-result --verbose
```

---

### Task 4: `simulation.launch.py` 接线与前置校验

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/launch/simulation.launch.py`

**Interfaces:**
- Consumes: 既有 `joint_order`（`[motor["joint"] for motor in simulation["motors"]]`）、
  `gazebo_launch`、`motor_sim` 节点构造处、`controller_mode`、`gravity`。
- Produces: launch 参数 `gravity_feedforward`（默认 `false`）；为 `true` 时
  `motor_sim` 的 `effort_topic` 变为 `/motor_sim/effort`，并额外启动
  `gravity_feedforward_mixer.py` 与 `robot_dynamics/gravity_torque_node`。

- [ ] **Step 1: 加参数与前置校验**

`DeclareLaunchArgument("gravity_feedforward", default_value="false")`。在读参数的
`OpaqueFunction` 里按顺序校验并抛 `RuntimeError`（文案点名触发条件与修正方式）：

1. `gravity_feedforward` 为真但 `gravity` 为假 → 报错（前馈会在无重力时把臂抬起）；
2. `gravity_feedforward` 为真但 `controller_mode` 不是 `host` / `external` → 报错
   （`firmware` 走 STM32 与台架模拟器，本轮不支持）；
3. `models/<模型>/urdf/<模型>.standalone.urdf` 不存在 → 报错。

- [ ] **Step 2: 接线**

- `motor_sim` 的 `effort_topic` 按开关在 `/motor_sim/effort` 与
  `/arm_effort_controller/commands` 之间取值（后者为默认，保持现状）。
- 启动 mixer：`scripts/gravity_feedforward_mixer.py`，参数 `joints=joint_order`，其余用默认值。
- 启动 `robot_dynamics` 的 `gravity_torque_node`：`model_file` 指向第 1 步校验过的
  standalone URDF，`joint_states_topic` 用 `/joint_states`。
- 与既有 `simulation_gate.py` 一致：mixer 或重力节点退出时 `EmitEvent(Shutdown(...))`，
  避免留下「输出停在最后一次力矩」的状态。
- 校验必须发生在创建 vcan 与临时目录之前（沿用现有 `try` 结构，失败时不留下残留接口）。

- [ ] **Step 3: 静态验证（不启 Gazebo）**

```bash
ros2 launch robot_arm_gazebo simulation.launch.py --show-args | grep gravity_feedforward
# 三个前置条件分别构造一次，确认在启动任何进程前就报错：
ros2 launch robot_arm_gazebo simulation.launch.py gravity_feedforward:=true gravity:=false
ros2 launch robot_arm_gazebo simulation.launch.py gravity_feedforward:=true controller_mode:=firmware
```

同时确认默认参数下 launch 生成的动作集合与改动前一致（可对比 `git stash` 前的输出，或逐项核对
节点与话题清单）。

---

### Task 5: 同步文档与描述

**Files:**
- Modify: `ros2_ws/src/robot_dynamics/README.md`、`ros2_ws/src/robot_arm_gazebo/README.md`
- Modify: `README.md`（第 12 行 `robot_dynamics` 一行）、`CLAUDE.md`（第 22 行与测试清单）

- [ ] **Step 1: 包 README**

`robot_dynamics/README.md`：写清新的计算 API、`gravity_torque_node` 的参数与话题、测试命令，
以及「不进入 ros2_control 实时路径、自动化测试不启 Gazebo」的边界。
`robot_arm_gazebo/README.md`：写清 `gravity_feedforward` 参数、适用组合、两个新节点的话题，
以及 A/B 验证命令与「本模型落差 ≈ 0.03 rad」的预期量级。

- [ ] **Step 2: 顶层文档**

把「`robot_dynamics` 当前仅验证依赖、无节点/话题」改成实际能力描述（含 ADR 0009 链接）；
测试清单补上 `test_arm_model` 与 `test_gravity_feedforward_mixer`。

---

### Task 6: 构建、测试与范围复核

- [ ] **Step 1: 全量构建与相关包测试**

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select robot_dynamics robot_arm_gazebo motor_sim motor_hardware_interface
colcon test-result --verbose
```

- [ ] **Step 2: 范围复核**

`git status` / `git diff --stat` 确认：`motor_drive`、`motor_sim`、`motor_hardware_interface`、
模型 URDF/xacro、`gazebo.launch.py`、`web_host`、固件与 Agent 无改动；没有把 `build/`、
`install/`、`log/` 或临时配置纳入变更。

---

### Task 7: 仿真 A/B 实测与报告（需要 Gazebo 与 vcan 权限）

**Files:**
- Create: `docs/reports/2026-10-08-gravity-feedforward-simulation.md`（日期按实际执行日）

- [ ] **Step 1: 跑两次仿真**

```bash
ros2 launch robot_arm_gazebo simulation.launch.py
ros2 launch robot_arm_gazebo simulation.launch.py gravity_feedforward:=true
```

每次启动后给同一组位姿目标（建议 `[0.5, 0.0, 0.0, 0.0]`，该位姿 τ_g 最大项 ≈ 0.90 Nm）：

```bash
ros2 topic pub --once /driver/position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, 0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states
ros2 topic echo /robot_dynamics/gravity_torque
```

- [ ] **Step 2: 记录并对照**

报告至少包含：两种模式的稳态关节误差、对应的 `joint_link1` 落差变化、`gravity_torque` 与
Pinocchio 离线值的一致性、以及 `τ_g/kp` 的预测值与实测值的差异。

已核对：本机 `ign gazebo -s`（无显示）能正常起服务并发布 `/world/empty/*` 话题，
`ros_gz_sim`、`gz_ros2_control` 均已安装。若 vcan 权限或其他条件仍不满足，报告改为记录
命令、预期值与未执行的原因，并把该验收项挂到活动计划里，不得写成已通过。
