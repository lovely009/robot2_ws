# UR5e RP/Vaxor Gazebo Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不回退现有 30 轴人形仿真的前提下，抽取通用 `motor_sim` 包，并新增一套可由 ROS 2、USB 或 USB-CANFD 控制的 6 轴 UR5e RP/Vaxor Gazebo 仿真。

**Architecture:** `motor_sim` 导出与机器人拓扑无关的协议、transport、plant、路由、安全和 ROS 节点组件。`humanoid_gazebo` 和 `robot_arm_gazebo` 各自提供固定 `TopologyContract`、模型、配置、launch 和薄 executable；通用核心不写死轴数或 bus 数。UR5e 只 vendor 固定上游提交的必要描述资产，控制层完全使用本项目 RP/Vaxor 语义。

**Tech Stack:** Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress / `ros_gz_sim`、`gz_ros2_control`、C++17、Python 3、ament_cmake、GoogleTest、pytest、YAML-CPP、POSIX PTY。

**Spec:** `docs/superpowers/specs/2026-08-25-ur5e-motor-simulation-design.md`

## Global Constraints

- 直接在 `dev-sim` 实施；每个任务只暂存明确列出的文件，不使用 `git add .` 或 `git add -A`。
- 工作空间最终只含仿真代码，不增加真机 bring-up、UR 真机驱动、MoveIt 2 或外部硬件插件。
- C++17，2 空格缩进，`-Wall -Wextra -Wpedantic`；Python 使用类型注解。
- 新增和修改行为必须遵循 red-green-refactor：先运行新测试并确认按预期失败，再实现最小代码。
- `humanoid_gazebo` 最终保持三个现有 launch、30 轴 ABI、topic、三端口和安全行为；其现有测试全部通过。
- UR5e 资产固定到 `18e6f603b3ebc2ec479fecb62d6be544b15755e9`，保留 BSD-3-Clause `LICENSE` 和本地修改记录。
- UR5e 驱动 ABI 固定为 6 轴：RP ID 1–3，Vaxor ID 4–6，全部位于 `arm` bus。
- ROS 与 external control 互斥；超时、越限、错 bus、竞争命令、断线或关键进程退出都导致整臂锁存零 effort。
- 正常时序为 controller 100 Hz、RP 反馈 100 Hz、Vaxor 反馈 50 Hz、已 enable 电机命令超时 150 ms。
- 最终验收不需真实 USB/USB-CANFD 设备，不做 10/30 分钟持续运行；必须完成 PTY 故障路径和无 GUI Gazebo 0.05 rad 动作容差。

---

## File and Interface Map

### `motor_sim`

- `include/motor_sim/protocol/*`：RP/Vaxor 帧 API，namespace `motor_sim::protocol`。
- `include/motor_sim/transport/*`：帧记录和 tty API，namespace `motor_sim::transport`。
- `include/motor_sim/core/config.hpp`：`SystemConfig`、`TopologyContract` 和 YAML loader。
- `include/motor_sim/core/{transmission,plant_model,safety_monitor,frame_router}.hpp`：无 ROS 的通用 plant 与安全核心。
- `include/motor_sim/ros/motor_sim_node.hpp`：动态 bus/port 的 ROS 节点组件。
- `src/{protocol,transport,core,ros}/`：上述接口的实现。
- `test/`：从人形包迁移的通用单测，以及新的动态拓扑/逐电机超时测试。

### `humanoid_gazebo`

- `include/humanoid_gazebo/topology.hpp` + `src/topology.cpp`：唯一的 30 轴拓扑常量。
- `src/motor_sim_main.cpp`：构造 30 轴 `TopologyContract` 并实例化共享 node。
- `CMakeLists.txt` / `package.xml`：删除内嵌核心 target，改为依赖 `motor_sim`。
- 现有 config/URDF/launch/Python 动作保留，只做必需的 namespace/include 迁移。

### `robot_arm_gazebo`

- `vendor/ur_description/`：固定上游描述、mesh、`LICENSE` 和 `UPSTREAM.md`。
- `urdf/ur5e.urdf.xacro`：官方 UR5e wrapper + 本项目 Fortress effort control tag。
- `config/{motors,controllers}.yaml`：6 轴 ABI、plant 和 controller 配置。
- `include/robot_arm_gazebo/topology.hpp` + `src/{topology,motor_sim_main}.cpp`：6 轴契约与薄 executable。
- `robot_arm_gazebo/{launch_support,motion_library,ros_action_player}.py`：launch 合成和有界动作。
- `launch/{gazebo,usb_sim,canfd_sim}.launch.py`、`worlds/ur5e_world.sdf`、`scripts/robot_arm_actions`。
- `test/`：资产、模型、ABI、launch、动作、PTY 和 diagnostics 测试。

---

### Task 1: Extract Protocol and Serial Transport into `motor_sim`

**Files:**
- Create: `src/motor_sim/package.xml`
- Create: `src/motor_sim/CMakeLists.txt`
- Create: `src/motor_sim/include/motor_sim/protocol/*.hpp`
- Create: `src/motor_sim/include/motor_sim/transport/*.hpp`
- Create: `src/motor_sim/src/protocol/*.cpp`
- Create: `src/motor_sim/src/transport/*.cpp`
- Create: `src/motor_sim/test/test_motor_protocol.cpp`
- Create: `src/motor_sim/test/test_frame_record.cpp`
- Create: `src/motor_sim/test/test_serial_transport.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`
- Modify: `src/humanoid_gazebo/package.xml`
- Delete after green: corresponding protocol/transport headers, sources and tests under `src/humanoid_gazebo/`

**Interfaces:**
- Produces: `motor_sim::protocol::{CanFrame,MotorProtocol,MotorCommand,MotorFeedback}`
- Produces: `motor_sim::transport::{FrameTransport,SerialTransport,ReceivedFrame,ReceiveResult}`
- Preserves: every existing RP/Vaxor golden vector and `C\rS8\rY5\rO\r` initialization byte sequence.

- [ ] **Step 1: Add a compile-failing public-surface test and minimal test-only package manifest**

```cpp
#include "motor_sim/protocol/motor_protocol.hpp"
#include "motor_sim/transport/serial_transport.hpp"

#include <gtest/gtest.h>

TEST(MotorSimPackageSurface, ExposesProtocolAndTransportNamespaces) {
  motor_sim::protocol::CanFrame frame;
  EXPECT_EQ(frame.size, 0U);
  EXPECT_EQ(motor_sim::transport::ReceiveResult::NoData,
            motor_sim::transport::ReceiveResult::NoData);
}
```

Add only enough `package.xml`/`CMakeLists.txt` to register this test target; do not add the missing headers yet.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_sim --cmake-args -DBUILD_TESTING=ON
```

Expected: compile failure containing `motor_sim/protocol/motor_protocol.hpp: No such file or directory`.

- [ ] **Step 3: Move protocol/transport code and update namespaces/includes**

Use the existing implementation verbatim except for package include paths and namespaces:

```cpp
namespace motor_sim::protocol {
// existing protocol implementation
}

namespace motor_sim::transport {
// existing transport implementation
}
```

Export CMake targets `motor_protocol` and `motor_transport`; `motor_transport` links `motor_protocol` and `Threads::Threads`. Update `humanoid_gazebo` temporarily to link the exported targets while its core/node still lives in that package.

- [ ] **Step 4: Move all three existing test suites and verify GREEN**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to humanoid_gazebo
source install/setup.bash
colcon test --packages-select motor_sim humanoid_gazebo
colcon test-result --verbose
```

Expected: both packages build; every moved golden-vector, fragmentation, baud-rate, SLCAN initialization and disconnect test passes with zero failures.

- [ ] **Step 5: Remove old duplicate files and commit**

Stage only the new `motor_sim` files and the explicit protocol/transport deletions and dependency edits.

```bash
git commit -m "refactor: extract motor protocol and transport"
```

---

### Task 2: Extract the Generic Core and Add Model-Owned Topology Contracts

**Files:**
- Create: `src/motor_sim/include/motor_sim/core/config.hpp`
- Create: `src/motor_sim/include/motor_sim/core/{transmission,plant_model,safety_monitor,frame_router}.hpp`
- Create: `src/motor_sim/src/core/{config,transmission,plant_model,safety_monitor,frame_router}.cpp`
- Create: `src/motor_sim/test/test_{config,transmission,plant_model,frame_router}.cpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/topology.hpp`
- Create: `src/humanoid_gazebo/src/topology.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`
- Modify: `src/humanoid_gazebo/src/motor_sim_main.cpp`
- Delete after green: old `include/humanoid_gazebo/motor_sim/`, `src/motor_sim/*.cpp` except main, and moved tests

**Interfaces:**
- Produces:

```cpp
namespace motor_sim::core {
struct RequiredMotorAddress {
  std::string joint;
  std::string bus;
  std::string protocol;
  unsigned id;
};

struct TopologyContract {
  std::vector<std::string> buses;
  std::vector<RequiredMotorAddress> motors;
};

SystemConfig load_system_config(const std::string &path,
                                const TopologyContract &contract);
}
```

- Produces: `humanoid_gazebo::humanoid_topology_contract() -> motor_sim::core::TopologyContract`.
- Preserves: exact 30-axis/bus/protocol/ID rejection semantics for the humanoid config.

- [ ] **Step 1: Write generic topology tests before moving the loader**

```cpp
TEST(SystemConfig, AcceptsContractOwnedByCaller) {
  const motor_sim::core::TopologyContract contract{
      {"arm"},
      {{"joint_a", "arm", "rp", 1U},
       {"joint_b", "arm", "motor_2530", 4U}}};
  const auto config = motor_sim::core::load_system_config(
      fixture_path("two_axis.yaml"), contract);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].joint, "joint_b");
}

TEST(SystemConfig, RejectsOrderDifferentFromContract) {
  const motor_sim::core::TopologyContract contract{
      {"arm"}, {{"joint_b", "arm", "motor_2530", 4U}}};
  EXPECT_THROW(motor_sim::core::load_system_config(
                   fixture_path("two_axis.yaml"), contract),
               std::runtime_error);
}
```

- [ ] **Step 2: Run the focused config test and verify RED**

Run:

```bash
colcon build --packages-select motor_sim --cmake-args -DBUILD_TESTING=ON
```

Expected: compile failure because `TopologyContract` and the two-argument loader do not exist.

- [ ] **Step 3: Move core types and replace the hard-coded 30-axis array**

Move the existing implementations into namespace `motor_sim::core`. In `load_system_config`, validate:

```cpp
if (parsed_buses != contract.buses) {
  throw std::runtime_error("buses violate the model topology contract");
}
if (motors.size() != contract.motors.size()) {
  throw std::runtime_error("motors violate the model topology contract");
}
```

For each index, compare joint, bus, protocol and ID with `contract.motors[index]`. Keep duplicate joint/address, finite parameter, hard/command limit and Vaxor Kt/current validation.

- [ ] **Step 4: Define the humanoid contract outside the shared core**

```cpp
namespace humanoid_gazebo {
const motor_sim::core::TopologyContract &humanoid_topology_contract();
}
```

`topology.cpp` owns the exact 30 entries previously held by `kRequiredMotorOrder`; no other human-specific constant remains in `motor_sim`.

- [ ] **Step 5: Move core tests and verify both generic and 30-axis behavior**

Run:

```bash
colcon build --symlink-install --packages-up-to humanoid_gazebo
source install/setup.bash
colcon test --packages-select motor_sim humanoid_gazebo
colcon test-result --verbose
```

Expected: generic 2-axis tests pass, all prior 30-axis config/transmission/plant/router tests pass, and wrong-bus synthetic router tests remain green.

- [ ] **Step 6: Commit**

```bash
git commit -m "refactor: make motor simulation topology generic"
```

---

### Task 3: Make Safety State Per-Enabled-Motor

**Files:**
- Modify: `src/motor_sim/include/motor_sim/core/safety_monitor.hpp`
- Modify: `src/motor_sim/src/core/safety_monitor.cpp`
- Modify: `src/motor_sim/src/core/frame_router.cpp`
- Modify: `src/motor_sim/test/test_safety_monitor.cpp`
- Modify: `src/motor_sim/test/test_frame_router.cpp`

**Interfaces:**
- Produces:

```cpp
struct MotorSafetyStatus {
  bool enabled{false};
  bool timed_out{false};
  std::optional<std::chrono::milliseconds> command_age;
};

struct SafetySnapshot {
  bool latched{false};
  Fault fault{Fault::None};
  std::vector<double> efforts;
  std::vector<MotorSafetyStatus> motor_status;
};

void set_enabled(std::size_t motor_index, bool enabled, TimePoint now);
void observe_valid_command(std::size_t motor_index,
                           const std::string &bus, uint32_t can_id,
                           const std::vector<uint8_t> &payload,
                           TimePoint now);
```

- Preserves: first-fault latch, 2 ms conflicting-payload heuristic, disconnect latch, zero effort snapshot.

- [ ] **Step 1: Add failing tests for disabled idle and one stale enabled axis**

```cpp
TEST(SafetyMonitor, DisabledMotorsMayRemainIdle) {
  SafetyMonitor safety(2U, 150ms, 2ms);
  EXPECT_FALSE(safety.tick(TimePoint{} + 10s).latched);
}

TEST(SafetyMonitor, OneStaleEnabledMotorLatchesWholeRobot) {
  SafetyMonitor safety(2U, 150ms, 2ms);
  const auto start = TimePoint{} + 1s;
  safety.set_enabled(0U, true, start);
  safety.set_enabled(1U, true, start);
  safety.observe_valid_command(0U, "arm", 1U, {1U}, start + 100ms);
  safety.observe_valid_command(1U, "arm", 2U, {2U}, start + 10ms);
  const auto snapshot = safety.tick(start + 170ms);
  EXPECT_TRUE(snapshot.latched);
  EXPECT_EQ(snapshot.fault, Fault::CommandTimeout);
  EXPECT_TRUE(snapshot.motor_status[1].timed_out);
}
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
colcon test --packages-select motor_sim --ctest-args -R 'test_safety_monitor|test_frame_router' --output-on-failure
```

Expected: compile failure for missing per-motor API or behavioral failure because the current monitor has one global timestamp.

- [ ] **Step 3: Implement per-motor timestamps and wire `FrameRouter`**

Use vectors sized by `axis_count`; enabling initializes the deadline at `now`, disabling clears that motor's timestamp, and each decoded non-control command refreshes only its target index. `tick()` latches the first enabled motor whose age is greater than 150 ms and populates all `MotorSafetyStatus` entries.

- [ ] **Step 4: Add red-green coverage for conflict timestamps**

Keep the key `bus + CAN ID`; use the `ReceivedFrame.received_at` supplied by the reader thread. Two different payloads at 1 ms must fault; the same pair at 3 ms must not fault.

- [ ] **Step 5: Run core regression and commit**

```bash
colcon test --packages-select motor_sim --event-handlers console_cohesion+
colcon test-result --verbose
git commit -m "fix: track command timeout per enabled motor"
```

---

### Task 4: Extract the Generic ROS Node and Preserve Humanoid Runtime Behavior

**Files:**
- Create: `src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`
- Create: `src/motor_sim/src/ros/motor_sim_node.cpp`
- Create: `src/motor_sim/test/test_motor_sim_node.cpp`
- Create: `src/motor_sim/test/test_pty_integration.cpp`
- Modify: `src/motor_sim/CMakeLists.txt`
- Modify: `src/motor_sim/package.xml`
- Modify: `src/humanoid_gazebo/src/motor_sim_main.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`
- Modify: `src/humanoid_gazebo/package.xml`
- Delete after green: old node header/source and moved node/PTY tests from `humanoid_gazebo`

**Interfaces:**
- Produces:

```cpp
struct BusPort {
  std::string bus;
  std::string device;
};

struct NodeParameters {
  motor_sim::transport::TransportMode transport_mode;
  bool allow_external_canfd{false};
  int baud_rate{3000000};
  std::string config_file;
  std::vector<BusPort> ports;
};

struct NodeDefaults {
  std::string config_file;
  std::string joint_states_topic{"joint_states"};
  std::string effort_topic{"effort_controller/commands"};
  std::string diagnostics_topic{"diagnostics"};
};

class MotorSimNode : public rclcpp::Node {
public:
  MotorSimNode(const rclcpp::NodeOptions &options,
               motor_sim::core::TopologyContract contract,
               NodeDefaults defaults);
};

void validate_node_parameters(
    const NodeParameters &parameters,
    const motor_sim::core::TopologyContract &contract);
```

- Parameters: `transport_mode`, `allow_external_canfd`, `baud_rate`, `config_file`, and one `<bus>_port` parameter for every `contract.buses` entry.
- Port validation: nonempty path, exists, opens as a tty; stable `/dev/serial/by-id` is recommended but not required.

- [ ] **Step 1: Write dynamic-port parameter tests**

```cpp
TEST(MotorSimNodeParameters, AcceptsSingleArmPtyAndThreeHumanoidPorts) {
  PseudoTerminal arm;
  PseudoTerminal left;
  PseudoTerminal right;
  PseudoTerminal legs;
  EXPECT_NO_THROW(validate_node_parameters(
      parameters({{"arm", arm.slave_path()}}), arm_contract()));
  EXPECT_NO_THROW(validate_node_parameters(
      parameters({{"left_arm", left.slave_path()},
                  {"right_arm", right.slave_path()},
                  {"legs", legs.slave_path()}}),
      humanoid_contract()));
}
```

Also test missing bus, duplicate device, unsupported baud, SLCAN without opt-in, and regular non-tty file rejection.

- [ ] **Step 2: Run and verify RED**

Run:

```bash
colcon test --packages-select motor_sim --ctest-args -R test_motor_sim_node --output-on-failure
```

Expected: compile failure because the generic node/parameter validator does not exist.

- [ ] **Step 3: Move the node and replace three fixed ports with contract-driven ports**

Declare `<bus>_port` in contract order, create one `SerialTransport` per bus, and keep relative ROS topic names so the caller's namespace determines final names. Resolve the configured device with `std::filesystem::canonical` only after existence/open validation; diagnostics retain both requested and resolved strings.

- [ ] **Step 4: Preserve disconnect zero delivery and launch shutdown semantics**

The fault path must:

```cpp
publish_zero_effort();
effort_publisher_->wait_for_all_acked(std::chrono::milliseconds(100));
request_process_shutdown_after(std::chrono::milliseconds(100));
```

Use the existing bounded shutdown behavior; do not block indefinitely. Keep `ReceivedFrame.received_at` from the reader thread through routing.

- [ ] **Step 5: Migrate the humanoid executable and PTY test**

`humanoid_motor_sim_node` constructs `humanoid_topology_contract()` and root-relative topics, preserving `/joint_states`, `/effort_controller/commands` and `/diagnostics`. Re-run the three-bus PTY round trip and disconnect assertion without weakening it.

- [ ] **Step 6: Verify all existing human tests and commit**

```bash
colcon build --symlink-install --packages-up-to humanoid_gazebo
source install/setup.bash
colcon test --packages-select motor_sim humanoid_gazebo
colcon test-result --verbose
git commit -m "refactor: share the generic motor simulation node"
```

---

### Task 5: Vendor the Pinned UR5e Description Assets

**Files:**
- Create: `src/robot_arm_gazebo/vendor/ur_description/LICENSE`
- Create: `src/robot_arm_gazebo/vendor/ur_description/UPSTREAM.md`
- Create: `src/robot_arm_gazebo/vendor/ur_description/urdf/ur_macro.xacro`
- Create: `src/robot_arm_gazebo/vendor/ur_description/urdf/inc/{ur_common,ur_transmissions}.xacro`
- Create: `src/robot_arm_gazebo/vendor/ur_description/config/ur5e/*.yaml`
- Create: `src/robot_arm_gazebo/vendor/ur_description/config/initial_positions.yaml`
- Create: 7 DAE visual and 7 STL collision files under `meshes/ur5e/`
- Create: `src/robot_arm_gazebo/test/test_vendor_assets.py`

**Interfaces:**
- Produces: an offline, package-local UR5e description tree with no runtime `ur_description` dependency.
- Preserves: upstream file content except package-lookup replacements recorded in `UPSTREAM.md`.

- [ ] **Step 1: Write the failing asset manifest test**

```python
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_SHA = "18e6f603b3ebc2ec479fecb62d6be544b15755e9"

def test_exact_ur5e_vendor_manifest() -> None:
    vendor = PACKAGE_ROOT / "vendor" / "ur_description"
    assert (vendor / "LICENSE").is_file()
    assert UPSTREAM_SHA in (vendor / "UPSTREAM.md").read_text()
    assert len(list((vendor / "meshes/ur5e/visual").glob("*.dae"))) == 7
    assert len(list((vendor / "meshes/ur5e/collision").glob("*.stl"))) == 7
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
python3 -m pytest src/robot_arm_gazebo/test/test_vendor_assets.py -v
```

Expected: FAIL because the vendor tree does not exist.

- [ ] **Step 3: Retrieve only the approved files from the pinned commit**

Use a temporary sparse checkout of the official repository at the exact SHA. Copy only the manifest listed in the spec. Do not copy upstream launch, MoveIt, driver, `ur.ros2_control.xacro`, unrelated robot types, CI or documentation.

Record in `UPSTREAM.md`:

```markdown
Repository: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
Commit: 18e6f603b3ebc2ec479fecb62d6be544b15755e9
Local changes: package lookups and mesh URIs point to robot_arm_gazebo;
generate_ros2_control_tag remains false in the local wrapper.
```

- [ ] **Step 4: Patch package paths and verify no external package lookup remains**

Run:

```bash
rg -n 'ur_description|\$\(find ' src/robot_arm_gazebo/vendor/ur_description
```

Expected: every remaining occurrence is either provenance text or a deliberate path replaced with `robot_arm_gazebo`; no runtime lookup requires an installed `ur_description` package.

- [ ] **Step 5: Run asset tests and commit**

```bash
python3 -m pytest src/robot_arm_gazebo/test/test_vendor_assets.py -v
git commit -m "feat: vendor pinned UR5e description assets"
```

---

### Task 6: Build the `robot_arm_gazebo` Model, Config and Fixed Six-Axis ABI

**Files:**
- Create: `src/robot_arm_gazebo/package.xml`
- Create: `src/robot_arm_gazebo/CMakeLists.txt`
- Create: `src/robot_arm_gazebo/resource/robot_arm_gazebo`
- Create: `src/robot_arm_gazebo/include/robot_arm_gazebo/topology.hpp`
- Create: `src/robot_arm_gazebo/src/topology.cpp`
- Create: `src/robot_arm_gazebo/src/motor_sim_main.cpp`
- Create: `src/robot_arm_gazebo/config/motors.yaml`
- Create: `src/robot_arm_gazebo/config/controllers.yaml`
- Create: `src/robot_arm_gazebo/urdf/ur5e.urdf.xacro`
- Create: `src/robot_arm_gazebo/worlds/ur5e_world.sdf`
- Create: `src/robot_arm_gazebo/test/test_{package_identity,config_topology,ur5e_model}.py`
- Create: `src/robot_arm_gazebo/test/test_motor_config.cpp`

**Interfaces:**
- Produces: `robot_arm_gazebo::robot_arm_topology_contract()` with six exact entries.
- Produces: executable `robot_arm_motor_sim_node` in `lib/robot_arm_gazebo`.
- Produces: URDF effort interfaces in official joint order and a namespaced controller configuration.

- [ ] **Step 1: Write the six-axis contract tests**

```python
EXPECTED = [
    ("shoulder_pan_joint", "arm", "rp", 1),
    ("shoulder_lift_joint", "arm", "rp", 2),
    ("elbow_joint", "arm", "rp", 3),
    ("wrist_1_joint", "arm", "motor_2530", 4),
    ("wrist_2_joint", "arm", "motor_2530", 5),
    ("wrist_3_joint", "arm", "motor_2530", 6),
]

def test_motor_controller_and_urdf_order_match() -> None:
    assert motor_rows() == EXPECTED
    assert controller_joints() == [row[0] for row in EXPECTED]
    assert ros2_control_joints() == [row[0] for row in EXPECTED]
```

Add failure cases for a seventh motor, swapped wrist IDs, bus not equal to `arm`, and command limits outside hard limits.

- [ ] **Step 2: Run and verify RED**

Run:

```bash
python3 -m pytest src/robot_arm_gazebo/test/test_config_topology.py -v
```

Expected: FAIL because package config and URDF do not exist.

- [ ] **Step 3: Implement package scaffolding and topology constants**

`topology.cpp` returns:

```cpp
const motor_sim::core::TopologyContract contract{
    {"arm"},
    {{"shoulder_pan_joint", "arm", "rp", 1U},
     {"shoulder_lift_joint", "arm", "rp", 2U},
     {"elbow_joint", "arm", "rp", 3U},
     {"wrist_1_joint", "arm", "motor_2530", 4U},
     {"wrist_2_joint", "arm", "motor_2530", 5U},
     {"wrist_3_joint", "arm", "motor_2530", 6U}}};
```

- [ ] **Step 4: Implement motor profiles and official model limits**

Use official UR5e hard/velocity/URDF effort limits. Set 5° command margins. Set RP effort limit 60 Nm; set Vaxor ratio 80, efficiency 0.8, Kt 0.012 Nm/A, 11 A and 10 Nm. Start plant gains at `kp: 30.0`, `kd: 1.0`; do not tune before Gazebo measurements.

- [ ] **Step 5: Implement the wrapper URDF and world**

The wrapper calls the pinned `ur_robot` macro with local parameter paths and `generate_ros2_control_tag:=false`, fixes `base_link` to `world`, disables self-collision, and appends six `command_interface name="effort"` entries for `ign_ros2_control/IgnitionSystem`. The world uses `<max_step_size>0.001</max_step_size>` and only physics, scene, ground and sensors systems.

- [ ] **Step 6: Verify xacro, model and ABI GREEN**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo
source install/setup.bash
xacro src/robot_arm_gazebo/urdf/ur5e.urdf.xacro > /tmp/robot_arm_ur5e.urdf
colcon test --packages-select robot_arm_gazebo --ctest-args -R 'package_identity|config_topology|ur5e_model|motor_config' --output-on-failure
colcon test-result --verbose
```

Expected: exactly three packages are listed; xacro succeeds; model contains six revolute driven joints, fixed base, `tool0`, positive finite inertia and the exact controller order.

- [ ] **Step 7: Commit**

```bash
git commit -m "feat: add six-axis UR5e motor model"
```

---

### Task 7: Add Namespaced Gazebo Launch and ROS Effort Actions

**Files:**
- Create: `src/robot_arm_gazebo/robot_arm_gazebo/__init__.py`
- Create: `src/robot_arm_gazebo/robot_arm_gazebo/launch_support.py`
- Create: `src/robot_arm_gazebo/robot_arm_gazebo/motion_library.py`
- Create: `src/robot_arm_gazebo/robot_arm_gazebo/ros_action_player.py`
- Create: `src/robot_arm_gazebo/launch/gazebo.launch.py`
- Create: `src/robot_arm_gazebo/scripts/robot_arm_actions`
- Create: `src/robot_arm_gazebo/test/test_{launch_contract,motion_library,ros_action_player}.py`
- Modify: `src/robot_arm_gazebo/CMakeLists.txt`
- Modify: `src/robot_arm_gazebo/package.xml`

**Interfaces:**
- Produces: `/robot_arm/joint_states`, `/robot_arm/effort_controller/commands`, `/robot_arm/controller_manager`.
- Produces actions: `home`, `joint_scan`, `combined`, `zero`.
- Produces launch argument `initial_positions` as a comma-separated six-radian vector validated before process creation.

- [ ] **Step 1: Write failing launch and action tests**

```python
def test_gazebo_launch_uses_robot_arm_namespace() -> None:
    text = launch_text("gazebo.launch.py")
    assert 'namespace="robot_arm"' in text or "PushRosNamespace" in text
    assert "ur5e" in text
    assert "/robot_arm/controller_manager" in text

def test_motion_library_has_bounded_six_axis_actions() -> None:
    for name in ("home", "joint_scan", "combined", "zero"):
        for target in action(name).targets:
            assert len(target) == 6
            assert inside_command_limits(target)
```

- [ ] **Step 2: Run and verify RED**

Run:

```bash
python3 -m pytest src/robot_arm_gazebo/test/test_launch_contract.py src/robot_arm_gazebo/test/test_motion_library.py -v
```

Expected: FAIL because launch and action modules do not exist.

- [ ] **Step 3: Implement fail-fast namespaced Gazebo launch**

Use `PushRosNamespace("robot_arm")` for robot-state/controller resources, entity name `ur5e`, controller manager `/robot_arm/controller_manager`, and relative topics. Spawn `joint_state_broadcaster` first and `effort_controller` only after success. Any Gazebo/create/spawner failure emits launch shutdown; do not continue the chain after a nonzero exit.

- [ ] **Step 4: Implement strict initial-position parsing**

```python
def parse_initial_positions(raw: str, limits: Sequence[tuple[float, float]]) -> list[float]:
    values = [float(item.strip()) for item in raw.split(",")]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("initial_positions must contain six finite radians")
    if any(not lower <= value <= upper for value, (lower, upper) in zip(values, limits)):
        raise ValueError("initial_positions exceed UR5e hard limits")
    return values
```

Default: `0,-1.57,0,-1.57,0,0`.

- [ ] **Step 5: Implement the six-axis bounded PD action player**

Default topics are `/robot_arm/effort_controller/commands` and `/robot_arm/joint_states`; default rate is 100 Hz and gains start at 30/1. Reject a joint-state name set/order mismatch, stale feedback, nonfinite state, missing controller subscription, or a second effort publisher. Always publish a final six-zero array on normal completion and exceptions.

- [ ] **Step 6: Run tests and commit**

```bash
colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash
colcon test --packages-select robot_arm_gazebo
colcon test-result --verbose
git commit -m "feat: launch and exercise the UR5e Gazebo arm"
```

---

### Task 8: Add USB and USB-CANFD External-Control Launches and Arm PTY Coverage

**Files:**
- Create: `src/robot_arm_gazebo/launch/usb_sim.launch.py`
- Create: `src/robot_arm_gazebo/launch/canfd_sim.launch.py`
- Modify: `src/robot_arm_gazebo/robot_arm_gazebo/launch_support.py`
- Create: `src/robot_arm_gazebo/test/test_pty_integration.cpp`
- Create: `src/robot_arm_gazebo/test/test_external_launch_contract.py`
- Modify: `src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`
- Modify: `src/motor_sim/src/ros/motor_sim_node.cpp`
- Modify: `src/motor_sim/package.xml`
- Modify: `src/robot_arm_gazebo/CMakeLists.txt`

**Interfaces:**
- Launch arguments: `arm_port`, `baud_rate:=3000000`, `allow_external_canfd`.
- Diagnostics: `/robot_arm/diagnostics` with mode, requested/resolved port, connection, per-motor enable/age/timeout, first fault and feedback counters.
- External feedback begins while disabled; effort remains zero until explicit per-motor enable.

- [ ] **Step 1: Add failing launch-contract tests**

```python
def test_external_launch_defaults_and_opt_in() -> None:
    usb = defaults("usb_sim.launch.py")
    canfd = defaults("canfd_sim.launch.py")
    assert usb["transport_mode"] == "usb_serial"
    assert canfd["transport_mode"] == "serial_slcan"
    assert usb["baud_rate"] == canfd["baud_rate"] == "3000000"
    assert canfd["allow_external_canfd"] == "false"
    assert "arm_port" in usb and "arm_port" in canfd
```

- [ ] **Step 2: Add a failing six-axis PTY test**

The test must:

1. open one PTY and start `robot_arm_motor_sim_node` in `/robot_arm`;
2. publish six official-order joint states;
3. observe disabled RP/Vaxor feedback at 100/50 Hz and six zero efforts;
4. enable RP 1–3 and Vaxor 4–6, then send valid commands;
5. verify nonzero finite efforts in exact order;
6. stop commands only for motor 2 while continuing the other five past 150 ms;
7. verify a six-zero array, `command_timeout` diagnostic and bounded shutdown.

Run the new test before implementation and expect failure because external launch and diagnostics are absent.

- [ ] **Step 3: Implement the two external launch wrappers**

Each wrapper includes `gazebo.launch.py`, starts exactly one `robot_arm_motor_sim_node` with the matching transport mode, and uses launch event handlers so motor node, Gazebo, create or controller failure shuts down the whole launch. `serial_slcan` refuses to construct processes unless opt-in is true.

- [ ] **Step 4: Implement structured diagnostics and publisher arbitration**

Publish one `DiagnosticStatus` for the arm and one per motor. The arm status includes transport and requested/resolved port; each motor includes protocol/ID, enabled, command age and timeout. Counters are monotonically increasing for process lifetime. If `count_publishers(effort_topic) > 1`, latch `duplicate_controller`.

- [ ] **Step 5: Add remaining PTY fault cycles**

Use fresh node/PTY instances per fault because faults only reset by process restart:

- command outside the 5°-guarded limit;
- same bus/CAN ID, different payload at recorded arrival times 1 ms apart;
- peer hangup and zero delivery;
- SLCAN exact `C\rS8\rY5\rO\r` bytes before frame traffic;
- `set-zero` changes Vaxor process-local feedback and resets after node recreation.

Wrong-bus remains a direct `FrameRouter` unit test: a one-port PTY cannot encode a source bus label.

- [ ] **Step 6: Run focused integration and commit**

```bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo
source install/setup.bash
colcon test --packages-select motor_sim robot_arm_gazebo --ctest-args -R 'pty|external_launch|motor_sim_node' --output-on-failure
colcon test-result --verbose
git commit -m "feat: add UR5e USB and CAN-FD control loops"
```

---

### Task 9: Tune Actions Against Headless Gazebo and Freeze Acceptance Values

**Files:**
- Modify: `src/robot_arm_gazebo/config/motors.yaml`
- Modify: `src/robot_arm_gazebo/robot_arm_gazebo/motion_library.py`
- Modify: `src/robot_arm_gazebo/robot_arm_gazebo/ros_action_player.py`
- Modify: `src/robot_arm_gazebo/test/test_motion_library.py`
- Create: `docs/verification/2026-08-25-ur5e-gazebo.md`

**Interfaces:**
- Acceptance: every `home`, `joint_scan`, `combined`, `zero` target reaches stable absolute error ≤ 0.05 rad without nonfinite state, limit violation, controller failure, process exit or sustained effort saturation.

- [ ] **Step 1: Launch the unmodified 30/1 baseline headlessly and record RED evidence if tolerance fails**

Run in one terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo gazebo.launch.py gui:=false
```

Run each action in another terminal:

```bash
ros2 run robot_arm_gazebo robot_arm_actions --action home
ros2 run robot_arm_gazebo robot_arm_actions --action joint_scan
ros2 run robot_arm_gazebo robot_arm_actions --action combined
ros2 run robot_arm_gazebo robot_arm_actions --action zero
```

Record per-joint maximum/stable error and saturation duration. If all actions already pass, retain 30/1; do not tune merely to change numbers.

- [ ] **Step 2: Use the action exit status as RED and apply the smallest fixed YAML gain changes**

`robot_arm_actions` must return nonzero when a target times out, exceeds 0.05 rad stable error, observes nonfinite state or remains saturated beyond its bounded settle window. For each baseline failure, retain that nonzero run as RED evidence, adjust only the failing joint class in `motors.yaml`, rebuild, and rerun the same action until it returns zero. Keep the 60 Nm RP and 10 Nm Vaxor caps unchanged; never relax the 0.05 rad threshold. If the 30/1 baseline is already green, make no gain change.

- [ ] **Step 3: Re-run all actions and controller failure checks**

Verify:

```bash
ros2 control list_controllers --controller-manager /robot_arm/controller_manager
ros2 topic echo /robot_arm/joint_states --once
ros2 topic info /robot_arm/effort_controller/commands --verbose
```

Expected: both controllers active, six exact joint names, only the current expected publisher, and all action exit codes zero.

- [ ] **Step 4: Record fixed values and measured evidence**

Document the retained `kp/kd`, action exit codes, maximum/stable error and saturation duration in YAML comments and the verification report. The report must state that these are simulation acceptance values, not real-motor tuning.

- [ ] **Step 5: Commit tuned values and evidence**

```bash
git commit -m "test: verify UR5e Gazebo arm actions"
```

---

### Task 10: Update Repository Rules and User Documentation

**Files:**
- Modify: `AGENTS.md`
- Modify: `README.md`
- Modify: `CONTEXT.md`
- Modify: `src/humanoid_gazebo/README.md`
- Create: `src/motor_sim/README.md`
- Create: `src/robot_arm_gazebo/README.md`
- Create: `src/robot_arm_gazebo/docs/usb-wiring.md`
- Modify: `docs/verification/2026-08-25-ur5e-gazebo.md`
- Create or modify tests: `src/robot_arm_gazebo/test/test_documentation_contract.py`

**Interfaces:**
- Documents exactly three packages and all six launch commands.
- Documents nonstable TTY risk, default/allowed baud rates, SLCAN opt-in, no physical CAN fidelity, nominal motor parameters, fault restart semantics and omitted endurance/physical-device tests.

- [ ] **Step 1: Write a failing documentation-contract test**

```python
def test_root_docs_describe_three_simulation_packages() -> None:
    readme = ROOT_README.read_text()
    agents = AGENTS.read_text()
    for package in ("motor_sim", "humanoid_gazebo", "robot_arm_gazebo"):
        assert package in readme
    assert "不要新增第二个 ROS package" not in agents
    assert "最终合入 `dev-sim`" in agents
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest src/robot_arm_gazebo/test/test_documentation_contract.py -v
```

Expected: FAIL because current root docs still state that only `humanoid_gazebo` exists and still require fixed `dev/smi` development.

- [ ] **Step 3: Update all documentation without overstating verification**

Include exact ROS, USB and CANFD launch examples, `/robot_arm` topics, six-axis mapping, fault/reset behavior and upstream attribution. State that PTY/headless Gazebo were tested; state that physical USB adapters and endurance tests were not executed. Keep human instructions valid after the package extraction.

- [ ] **Step 4: Run documentation/static tests and commit**

```bash
colcon test --packages-select motor_sim humanoid_gazebo robot_arm_gazebo --ctest-args -R 'documentation|package_identity|launch_contract' --output-on-failure
colcon test-result --verbose
git commit -m "docs: document shared motor and UR5e simulation"
```

---

### Task 11: Full Clean Build, Regression, Gazebo Smoke Test and Remote Delivery

**Files:**
- Modify only if evidence is inaccurate: `docs/verification/2026-08-25-ur5e-gazebo.md`
- Do not stage: `build/`, `install/`, `log/`, caches or IDE files

**Interfaces:**
- Completion evidence: three-package clean build, complete automated suite, human launch regression, arm PTY suite, arm headless Gazebo actions, clean Git tree and remote `dev-sim` at the verified commit.

- [ ] **Step 1: Inspect the exact final diff before destructive cleanup**

```bash
git status --short --branch
git diff --check
git diff --stat HEAD~10..HEAD
git ls-files --others --exclude-standard
```

Resolve any unrelated/untracked source before proceeding. Move only explicit generated `build/`, `install/`, `log/` and cache paths to the desktop trash; do not delete or overwrite source paths.

- [ ] **Step 2: Run a fresh complete build**

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --symlink-install
```

Expected package list, in any colcon display order:

```text
humanoid_gazebo
motor_sim
robot_arm_gazebo
```

Expected build summary: three packages finished, zero failed.

- [ ] **Step 3: Run the complete automated suite**

```bash
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

Expected: zero errors, zero failures, zero skipped unless a skip is explicitly documented and approved before completion.

- [ ] **Step 4: Run human and arm headless smoke tests**

Human:

```bash
ros2 launch humanoid_gazebo gazebo.launch.py gui:=false
```

Verify 30 joint states and active human controllers, then stop cleanly. Arm:

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py gui:=false
ros2 run robot_arm_gazebo robot_arm_actions --action joint_scan
```

Verify six exact joint states, active `/robot_arm` controllers, action tolerance and clean shutdown. Do not run both launch files simultaneously in this task.

- [ ] **Step 5: Verify final repository cleanliness and evidence**

```bash
git diff --check
git status --short --branch
git ls-files --others --exclude-standard
colcon test-result --verbose
```

Expected: no unstaged/staged changes, no untracked nonignored files, and fresh zero-failure test summary.

- [ ] **Step 6: Synchronize only the verified commit to `dev-sim`**

Confirm local and remote base/head before the write. Use the available authenticated GitHub connector if terminal HTTPS credentials remain unavailable. After publishing, query the remote `dev-sim` SHA and verify it equals local `HEAD`; do not create a PR because the user explicitly chose direct `dev-sim` delivery.
