# Elbow Joint Gazebo Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the four-axis elbow model from the standalone `elbow_joint_description` package into `robot_arm_gazebo` and provide a fully controlled Gazebo Fortress launch using YAML-driven `motor_sim` topology.

**Architecture:** Keep SolidWorks exports as immutable vendor assets inside `robot_arm_gazebo`, wrap them with a simulation-specific Xacro, and expose a separate `elbow_joint.launch.py` orchestration path. Remove the compiled six-axis topology requirement from the generic motor simulator entry point so the selected YAML remains the only motor topology source, while retaining the strict-contract loader overload for callers that need it.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress 6, URDF/Xacro, `ros_gz_sim`, `gz_ros2_control`, ros2_control controllers, C++17, Python pytest, YAML.

**Spec:** `docs/superpowers/specs/2026-09-02-elbow-joint-gazebo-integration-design.md`

## Global Constraints

- Preserve the existing `robot_arm_gazebo/gazebo.launch.py` UR5e interface and behavior.
- Motor count, joint names, bus names, protocol, IDs, and profiles come from the selected YAML; do not hard-code an elbow topology in C++ or launch.
- Elbow controlled joints are exactly `1J`, `2J`, `3J`, and `4J`.
- Elbow simulation topology uses logical bus `vcan0`, `cerb`, `rp_elbow`, and IDs 1 through 4.
- Simulation joint limits are 11 N·m and 3.141592653589793 rad/s; damping is 0.1 N·m·s/rad; these are not real-hardware ratings.
- Preserve all exported mesh geometry, inertial tensors, joint origins, axes, and mechanical position limits.
- Delete `ros2_ws/src/urdf` after its reusable assets and tests have moved; leave no runtime reference to `elbow_joint_description`.
- Tests must not start Gazebo or real hardware; final integration verification may start Gazebo explicitly after automated tests pass.

---

### Task 1: Make `motor_sim` topology YAML-driven

**Files:**
- Modify: `ros2_ws/src/motor_sim/include/motor_sim/core/config.hpp`
- Modify: `ros2_ws/src/motor_sim/src/core/config.cpp`
- Modify: `ros2_ws/src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`
- Modify: `ros2_ws/src/motor_sim/src/ros/motor_sim_node.cpp`
- Modify: `ros2_ws/src/motor_sim/test/test_config.cpp`
- Create: `ros2_ws/src/motor_sim/test/fixtures/four_axis.yaml`
- Modify: `ros2_ws/src/robot_arm_gazebo/src/motor_sim_main.cpp`
- Modify: `ros2_ws/src/robot_arm_gazebo/CMakeLists.txt`
- Modify: `ros2_ws/src/robot_arm_gazebo/test/test_motor_config.cpp`
- Delete: `ros2_ws/src/robot_arm_gazebo/include/robot_arm_gazebo/topology.hpp`
- Delete: `ros2_ws/src/robot_arm_gazebo/src/topology.cpp`

**Interfaces:**
- Consumes: existing `motor_sim::core::SystemConfig` YAML schema and `MotorSimNode::NodeDefaults::config_file`.
- Produces: `SystemConfig load_system_config(const std::string &path)` for self-validating arbitrary YAML topology; existing `load_system_config(path, contract)` remains available and delegates to the same parser before applying an optional strict contract; `MotorSimNode(NodeOptions, NodeDefaults)` uses the YAML-only overload.

- [ ] **Step 1: Write failing core tests for YAML-only topology loading**

Add tests that load a temporary four-axis fixture without a `TopologyContract` and assert its exact ordered addresses:

```cpp
TEST(MotorSimConfig, LoadsTopologyDirectlyFromYaml) {
  const auto config = motor_sim::core::load_system_config(kFourAxisConfig);
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses[0].name, "vcan0");
  ASSERT_EQ(config.motors.size(), 4U);
  EXPECT_EQ(config.motors[0].joint, "1J");
  EXPECT_EQ(config.motors[3].id, 4U);
}
```

Also assert the YAML-only overload rejects duplicate `(bus, protocol, id)`, unknown buses, duplicate joint names, empty motor lists, and missing profiles.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_sim --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select motor_sim --ctest-args -R test_config --output-on-failure
```

Expected: compile failure because `load_system_config(const std::string &)` does not exist.

- [ ] **Step 3: Add the YAML-only parser interface**

Refactor the current parser into one implementation that always performs intrinsic schema and uniqueness validation. Implement:

```cpp
SystemConfig load_system_config(const std::string &path);

SystemConfig load_system_config(const std::string &path,
                                const TopologyContract &contract) {
  auto config = load_system_config(path);
  validate_contract(config, contract);
  return config;
}
```

Do not weaken existing strict-contract behavior or duplicate YAML parsing.

- [ ] **Step 4: Verify core GREEN**

Run the Task 1 Step 2 commands. Expected: all `motor_sim` config tests pass.

- [ ] **Step 5: Write a failing node/entry-point test for configurable topology**

Replace `RobotArmMotorConfig.LoadsExactSixAxisContract` with tests that call the YAML-only overload for both `config/motors.yaml` and `motor_sim/test/fixtures/four_axis.yaml`, whose exact addresses are `1J` through `4J`, bus `vcan0`, protocol `cerb`, and IDs 1 through 4. Add a source-contract test that fails while `motor_sim_main.cpp` still calls `robot_arm_topology_contract()`.

- [ ] **Step 6: Add the contract-free `MotorSimNode` constructor and simplify the entry point**

Add this overload while retaining the existing strict constructor:

```cpp
MotorSimNode(const rclcpp::NodeOptions &options, NodeDefaults defaults);
```

It must resolve `config_file` through the existing ROS parameter flow, call the YAML-only loader, and initialize identically to the strict constructor. Change `motor_sim_main.cpp` to construct the node with only `NodeOptions` and defaults. Remove `topology.hpp`, `topology.cpp`, their CMake source references, and their hard-coded tests.

- [ ] **Step 7: Build and test both affected packages**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select motor_sim robot_arm_gazebo --event-handlers console_direct+
colcon test-result --test-result-base build/motor_sim --verbose
colcon test-result --test-result-base build/robot_arm_gazebo --verbose
```

Expected: zero failures.

- [ ] **Step 8: Commit the YAML-driven topology seam**

```bash
git add ros2_ws/src/motor_sim ros2_ws/src/robot_arm_gazebo/CMakeLists.txt \
  ros2_ws/src/robot_arm_gazebo/src/motor_sim_main.cpp \
  ros2_ws/src/robot_arm_gazebo/include/robot_arm_gazebo/topology.hpp \
  ros2_ws/src/robot_arm_gazebo/src/topology.cpp \
  ros2_ws/src/robot_arm_gazebo/test/test_motor_config.cpp
git commit -m "refactor: load simulation topology from yaml"
```

---

### Task 2: Move elbow vendor assets and description tests

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/urdf/joint.urdf`
- Create: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/urdf/肘部关节.csv`
- Create: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/meshes/*.STL`
- Create: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/config/joint_names_肘部关节.yaml`
- Create: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/export.log`
- Create: `ros2_ws/src/robot_arm_gazebo/test/test_elbow_description.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/test/test_vendor_assets.py`
- Delete: `ros2_ws/src/urdf/`

**Interfaces:**
- Consumes: corrected source assets and semantic checks from the standalone package.
- Produces: one canonical elbow vendor asset root owned and installed by `robot_arm_gazebo`; mesh URI prefix `package://robot_arm_gazebo/vendor/elbow_joint_description/meshes/`.

- [ ] **Step 1: Write failing migration tests**

Create `test_elbow_description.py` with explicit paths under the future vendor directory. Test XML parsing, positive masses, positive-definite inertia matrices, non-empty material names, fixed joints without axes, and existence of every referenced mesh. Assert that each URI begins with the new literal prefix and no source file contains `package://elbow_joint_description/`.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_description.py
```

Expected: failure because the vendor asset directory does not exist.

- [ ] **Step 3: Move assets and rewrite only package URIs**

Move the URDF, CSV, six STL files, joint-name YAML, and export log into the paths listed above. Update all twelve visual/collision mesh URIs to:

```xml
<mesh filename="package://robot_arm_gazebo/vendor/elbow_joint_description/meshes/link0.STL"/>
```

with the corresponding case-sensitive filename for each link. Do not modify numeric geometry, inertial, origin, axis, or position-limit values.

- [ ] **Step 4: Remove standalone package scaffolding**

Delete the old package manifest, CMake file, Classic Gazebo/RViz launch files, old tests, and now-empty source directories. Confirm:

```bash
test ! -e /home/ly/robot_ws/ros2_ws/src/urdf
! rg -n 'elbow_joint_description' /home/ly/robot_ws/ros2_ws/src --glob '!robot_arm_gazebo/vendor/**'
```

- [ ] **Step 5: Verify GREEN and installation**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_description.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_gazebo --cmake-args -DBUILD_TESTING=ON
test -f install/robot_arm_gazebo/share/robot_arm_gazebo/vendor/elbow_joint_description/meshes/link0.STL
```

Expected: all description tests pass and installed assets exist.

- [ ] **Step 6: Commit the asset migration**

```bash
git add -A ros2_ws/src/urdf ros2_ws/src/robot_arm_gazebo/vendor \
  ros2_ws/src/robot_arm_gazebo/test/test_elbow_description.py \
  ros2_ws/src/robot_arm_gazebo/test/test_vendor_assets.py
git commit -m "refactor: move elbow model into gazebo package"
```

---

### Task 3: Add the elbow simulation Xacro

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/urdf/elbow_joint.urdf.xacro`
- Modify: `ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/urdf/joint.urdf`
- Modify: `ros2_ws/src/robot_arm_gazebo/test/test_elbow_description.py`

**Interfaces:**
- Consumes: vendor `joint.urdf`, controller file argument, robot namespace argument.
- Produces: Xacro arguments `controllers_file`, `robot_namespace`, `joint_damping`, and four controlled joints with effort command plus position/velocity/effort state interfaces.

- [ ] **Step 1: Write failing Xacro behavior tests**

Render `urdf/elbow_joint.urdf.xacro` using the real `xacro` command. Assert:

```python
assert joints["world_to_link0"].attrib["type"] == "fixed"
assert [joint.attrib["name"] for joint in control.findall("joint")] == [
    "1J", "2J", "3J", "4J"
]
assert all(joint.find("command_interface").attrib["name"] == "effort"
           for joint in control.findall("joint"))
assert all(joints[name].find("dynamics").attrib["damping"] == "0.1"
           for name in ("1J", "2J", "3J", "4J"))
```

Also assert each revolute limit has `effort=11` and `velocity=3.141592653589793`, the plugin namespace can be overridden, and `ign sdf -p` accepts the rendered document.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_description.py
```

Expected: failure because `elbow_joint.urdf.xacro` does not exist.

- [ ] **Step 3: Make the vendor description includable**

Do not add Xacro syntax to the vendor document. The Humble probe confirmed that the wrapper namespace is sufficient and the complete vendor `<robot>` contents expand correctly. Keep the vendor file valid under direct `check_urdf` parsing.

- [ ] **Step 4: Implement the wrapper**

The wrapper must include the vendor description, add `world` and `world_to_link0`, declare the four ros2_control joints, and configure:

```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
  <!-- 1J through 4J: effort command, position/velocity/effort state -->
</ros2_control>
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <ros><namespace>/$(arg robot_namespace)</namespace></ros>
    <parameters>$(arg controllers_file)</parameters>
    <hold_joints>true</hold_joints>
  </plugin>
</gazebo>
```

Humble Xacro has been probed with the real source file and expands a complete included `<robot>` document into the wrapper. Use that include behavior directly; do not copy the model into a second maintained file.

- [ ] **Step 5: Verify GREEN**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_description.py
xacro src/robot_arm_gazebo/urdf/elbow_joint.urdf.xacro > /tmp/elbow_joint.urdf
check_urdf /tmp/elbow_joint.urdf
ign sdf -p /tmp/elbow_joint.urdf > /tmp/elbow_joint.sdf
```

Expected: all commands exit zero.

- [ ] **Step 6: Commit the simulation description**

```bash
git add ros2_ws/src/robot_arm_gazebo/urdf/elbow_joint.urdf.xacro \
  ros2_ws/src/robot_arm_gazebo/vendor/elbow_joint_description/urdf/joint.urdf \
  ros2_ws/src/robot_arm_gazebo/test/test_elbow_description.py
git commit -m "feat: add elbow gazebo control description"
```

---

### Task 4: Add synchronized elbow controller and motor YAML

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/config/elbow_controllers.yaml`
- Create: `ros2_ws/src/robot_arm_gazebo/config/elbow_motors.yaml`
- Create: `ros2_ws/src/robot_arm_gazebo/test/test_elbow_config_topology.py`

**Interfaces:**
- Consumes: controlled joint order `1J`, `2J`, `3J`, `4J`; existing `motor_simulation` and wildcard controller schemas.
- Produces: controller manager parameters and motor simulation topology with identical ordered joints.

- [ ] **Step 1: Write failing cross-file topology tests**

Assert the exact motor rows:

```python
EXPECTED = [
    ("1J", "vcan0", "cerb", 1, "rp_elbow"),
    ("2J", "vcan0", "cerb", 2, "rp_elbow"),
    ("3J", "vcan0", "cerb", 3, "rp_elbow"),
    ("4J", "vcan0", "cerb", 4, "rp_elbow"),
]
```

Assert `motor_simulation.buses == ["vcan0"]`; controller `effort_controller` and `brake_controller` joints equal the same ordered list; IDs and joint names are unique; profile hard limits contain the original URDF position limits; and the Xacro ros2_control set equals the YAML sets.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_config_topology.py
```

Expected: failure because both YAML files are missing.

- [ ] **Step 3: Create the motor YAML**

Copy only the schema shape from `motors.yaml`. Define timing, bus `[vcan0]`, one `rp_elbow` profile with `velocity_rad_s=3.141592653589793`, `effort_nm=11`, and four `cerb` motor rows. Convert the four URDF radian limits to degrees for profile hard/command bounds without widening beyond the mechanical bounds. Add a top comment stating all protocol/profile/ID/gain values are simulation defaults, not hardware calibration.

- [ ] **Step 4: Create the controller YAML**

Use the existing wildcard node keys. Configure `joint_state_broadcaster`, `effort_controller`, and `brake_controller` for exactly `1J` through `4J`; use effort command and position/velocity state interfaces. Use explicit simulation holding gains:

```yaml
gains:
  1J: {p: 40.0, i: 0.0, d: 0.0}
  2J: {p: 40.0, i: 0.0, d: 0.0}
  3J: {p: 40.0, i: 0.0, d: 0.0}
  4J: {p: 40.0, i: 0.0, d: 0.0}
```

- [ ] **Step 5: Verify GREEN through Python and C++ loaders**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_elbow_config_topology.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select motor_sim robot_arm_gazebo --event-handlers console_direct+
```

Expected: zero failures, including YAML-only C++ loading of `elbow_motors.yaml`.

- [ ] **Step 6: Commit synchronized configuration**

```bash
git add ros2_ws/src/robot_arm_gazebo/config/elbow_controllers.yaml \
  ros2_ws/src/robot_arm_gazebo/config/elbow_motors.yaml \
  ros2_ws/src/robot_arm_gazebo/test/test_elbow_config_topology.py \
  ros2_ws/src/robot_arm_gazebo/test/test_motor_config.cpp
git commit -m "feat: configure four-axis elbow simulation"
```

---

### Task 5: Add the independent elbow launch entry point

**Files:**
- Create: `ros2_ws/src/robot_arm_gazebo/launch/elbow_joint.launch.py`
- Create: `ros2_ws/src/robot_arm_gazebo/test/test_elbow_launch_contract.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/robot_arm_gazebo/launch_support.py`
- Modify: `ros2_ws/src/robot_arm_gazebo/test/test_launch_contract.py`

**Interfaces:**
- Consumes: `elbow_joint.urdf.xacro`, `elbow_controllers.yaml`, `elbow_motors.yaml`, existing motor simulator and brake coordinator executables.
- Produces: `ros2 launch robot_arm_gazebo elbow_joint.launch.py`; launch arguments `initial_positions`, `world_file`, `world_name`, `gz_verbosity`, `transport_mode`, `arm_port`, `baud_rate`, `allow_external_canfd`, `fault_latching_enabled`, `motor_config_file`, `robot_namespace`, topic/service names, `controller_switch_timeout`, and `joint_damping`.

- [ ] **Step 1: Generalize initial-position parsing with failing tests**

Change `parse_initial_positions` to derive count exclusively from `limits` and update its error text from “six” to the expected count. Add tests for four-element elbow vectors while keeping all current six-element UR5e tests green.

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/ly/robot_ws/ros2_ws
python3 -m pytest -q src/robot_arm_gazebo/test/test_launch_contract.py \
  src/robot_arm_gazebo/test/test_elbow_launch_contract.py
```

Expected: failure because four-axis parsing and the elbow launch do not exist.

- [ ] **Step 3: Implement the independent launch**

Follow the existing fail-fast stage order, substituting:

```python
ELBOW_LIMITS = (
    (-2.6179938779914944, 0.17453292519943295),
    (-1.5707963267948966, 1.5707963267948966),
    (-0.08726646259971647, 2.0943951023931953),
    (-1.5707963267948966, 1.5707963267948966),
)
```

Use defaults `initial_positions="0,0,0,0"`, `robot_namespace="elbow_joint"`, `arm_port="vcan0"`, `motor_config_file="elbow_motors.yaml"`, entity name `elbow_joint`, Xacro `elbow_joint.urdf.xacro`, and controllers `elbow_controllers.yaml`. Keep Gazebo paused until broadcasters/controllers and brake controller are configured, then start the generic `robot_arm_motor_sim_node` with the selected YAML.

- [ ] **Step 4: Assert the launch contract and fail-fast chain**

Test exact defaults, entity name, namespace, four initial positions, generated file substitutions, controller-manager path, motor config parameter, remappings, inactive controller stages, paused-world stepping, and shutdown behavior when model spawning or either backend exits.

- [ ] **Step 5: Verify GREEN without Gazebo**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/robot_arm_gazebo/test/test_launch_contract.py \
  src/robot_arm_gazebo/test/test_elbow_launch_contract.py
ros2 launch robot_arm_gazebo elbow_joint.launch.py --show-args
```

Expected: tests pass and all documented launch arguments are listed.

- [ ] **Step 6: Commit the launch entry point**

```bash
git add ros2_ws/src/robot_arm_gazebo/launch/elbow_joint.launch.py \
  ros2_ws/src/robot_arm_gazebo/robot_arm_gazebo/launch_support.py \
  ros2_ws/src/robot_arm_gazebo/test/test_launch_contract.py \
  ros2_ws/src/robot_arm_gazebo/test/test_elbow_launch_contract.py
git commit -m "feat: launch controlled elbow simulation"
```

---

### Task 6: Document, verify, and run the integrated simulation

**Files:**
- Modify: `ros2_ws/src/robot_arm_gazebo/README.md`
- Modify: `ros2_ws/src/robot_arm_gazebo/package.xml`

**Interfaces:**
- Consumes: completed elbow launch and configs.
- Produces: documented build, vcan setup, launch, command, state, and safety instructions.

- [ ] **Step 1: Add documentation checks before editing README**

Extend an existing package/launch contract test to assert the installed package has `ros_gz_sim`, `gz_ros2_control`, `xacro`, controller dependencies, and both launch entries. This should fail only if a newly required dependency or installed file is missing.

- [ ] **Step 2: Update README and manifest**

Document these exact commands:

```bash
sudo ip link add dev vcan0 type vcan  # only when vcan0 does not exist
sudo ip link set up vcan0
ros2 launch robot_arm_gazebo elbow_joint.launch.py
ros2 topic echo /elbow_joint/joint_states
```

Explain that `elbow_motors.yaml` values are simulation defaults, list `1J` through `4J`, and retain the existing UR5e usage. Add only missing runtime/test dependencies to `package.xml`.

- [ ] **Step 3: Run focused static verification**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select motor_sim robot_arm_gazebo --event-handlers console_direct+
colcon test-result --test-result-base build/motor_sim --verbose
colcon test-result --test-result-base build/robot_arm_gazebo --verbose
ament_flake8 src/robot_arm_gazebo/test src/robot_arm_gazebo/launch \
  src/robot_arm_gazebo/robot_arm_gazebo
```

Expected: builds succeed, all focused tests pass, and flake8 reports no errors.

- [ ] **Step 4: Run full non-hardware regression tests**

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon test
colcon test-result --verbose
```

Expected: zero new failures. If pre-existing failures remain, record their exact package/test names and verify their result timestamps predate or are independent of this change.

- [ ] **Step 5: Start Gazebo Fortress and verify the model**

Ensure `vcan0` exists, then run:

```bash
cd /home/ly/robot_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo elbow_joint.launch.py
```

In separate sourced terminals verify:

```bash
ign model --list
ros2 control list_controllers -c /elbow_joint/controller_manager
ros2 topic echo --once /elbow_joint/joint_states
```

Expected: `elbow_joint` exists; `joint_state_broadcaster` and `brake_controller` are active; all four joints appear; logs contain no mesh URI, plugin, controller, or topology mismatch errors.

- [ ] **Step 6: Verify controlled motion within limits**

After switching from brake to effort control through the existing coordinator workflow, send a bounded nonzero command to one axis and observe its position change without exceeding its URDF limit. Stop the launch with Ctrl-C after collecting evidence.

- [ ] **Step 7: Inspect the final change set**

```bash
cd /home/ly/robot_ws
git diff --check
git status --short
rg -n 'elbow_joint_description' ros2_ws/src --glob '!robot_arm_gazebo/vendor/**'
test ! -e ros2_ws/src/urdf
```

Expected: no whitespace errors, no standalone package, and no runtime reference to the deleted package name. Preserve and report unrelated pre-existing worktree changes.

- [ ] **Step 8: Commit documentation and final integration adjustments**

```bash
git add ros2_ws/src/robot_arm_gazebo/README.md \
  ros2_ws/src/robot_arm_gazebo/package.xml
git commit -m "docs: document elbow gazebo simulation"
```
