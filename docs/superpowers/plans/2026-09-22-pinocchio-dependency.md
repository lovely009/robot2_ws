# Pinocchio Dependency Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone `robot_dynamics` ROS 2 package that discovers and links the ROS Humble Pinocchio package without changing existing control or simulation behavior.

**Architecture:** `robot_dynamics` is an ament C++ dependency boundary. It exposes a minimal CMake interface target linked to `pinocchio::pinocchio` and contains one smoke test that constructs a basic `pinocchio::Model`; it has no ROS node, runtime path, URDF loading, or controller integration.

**Tech Stack:** ROS 2 Humble, ament_cmake, C++17, CMake, GoogleTest, ROS package `ros-humble-pinocchio` 4.0.0-2jammy.20260606.100000.

**Spec:** `docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md`

## Global Constraints

- Use the system/ROS-distribution Pinocchio package; do not vendor or auto-download Pinocchio source.
- Keep `motor_drive`, `motor_hardware_interface`, `motor_sim`, `robot_arm_gazebo`, `web_host`, and all existing launch/YAML/URDF/controller files unchanged.
- Do not add a ROS node, topic, service, parameter, controller, launch action, or URDF parsing path.
- Use C++17 with `-Wall -Wextra -Wpedantic` for the new compiled test target.
- Do not commit generated `build/`, `install/`, or `log/` files.
- Do not start Gazebo or connect real hardware during verification.

---

### Task 1: Create the standalone package manifest

**Files:**
- Create: `ros2_ws/src/robot_dynamics/package.xml`

**Interfaces:**
- Produces the ROS package name `robot_dynamics` and declares the rosdep key `pinocchio` for the CMake package `pinocchio`.
- Produces a package that can be discovered by `colcon` without depending on any existing workspace package.

- [x] **Step 1: Write the package manifest**

Create the following package metadata, matching the existing ament package style:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_dynamics</name>
  <version>0.1.0</version>
  <description>Standalone Pinocchio dependency boundary for robot kinematics and dynamics.</description>
  <maintainer email="robot@example.com">Robot Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pinocchio</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [x] **Step 2: Validate the manifest syntax**

Run:

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('ros2_ws/src/robot_dynamics/package.xml')
print('package.xml: valid XML')
PY
```

Expected: `package.xml: valid XML`.

- [x] **Step 3: Commit the manifest**

```bash
git add ros2_ws/src/robot_dynamics/package.xml
git commit -m "feat: add robot dynamics package manifest"
```

---

### Task 2: Add the Pinocchio interface target and CMake test registration

**Files:**
- Create: `ros2_ws/src/robot_dynamics/CMakeLists.txt`
- Create: `ros2_ws/src/robot_dynamics/test/test_pinocchio_dependency.cpp`

**Interfaces:**
- Consumes the `pinocchio` CMake package exported by `ros-humble-pinocchio`.
- Produces the exported CMake target `robot_dynamics` and the dependency export `pinocchio`.
- Registers the test executable `test_pinocchio_dependency` when `BUILD_TESTING` is enabled.

- [x] **Step 1: Add CMake dependency discovery and target definition**

Create:

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_dynamics)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pinocchio_dependency test/test_pinocchio_dependency.cpp)
  target_compile_features(test_pinocchio_dependency PRIVATE cxx_std_17)
  target_compile_options(test_pinocchio_dependency PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_pinocchio_dependency ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)
ament_package()
```

The interface target intentionally has no runtime implementation or public wrapper header; its purpose is to make the Pinocchio dependency discoverable and linkable from the future package API without introducing speculative code.

- [x] **Step 2: Add the dependency smoke test source**

Create `ros2_ws/src/robot_dynamics/test/test_pinocchio_dependency.cpp` with:

```cpp
#include <gtest/gtest.h>

#include <pinocchio/multibody/model.hpp>

TEST(PinocchioDependencyTest, ConstructsEmptyModel) {
  pinocchio::Model model;

  EXPECT_EQ(model.nq, 0);
  EXPECT_EQ(model.nv, 0);
}
```

This verifies that the exported headers and target are usable, while avoiding any URDF or robot-specific behavior.

- [x] **Step 3: Configure and run only the new package test**

If the system dependency is missing, install the already-available ROS package through the project’s approved sudo flow:

```bash
sudo -A apt install ros-humble-pinocchio
```

Then run:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_dynamics
colcon test --packages-select robot_dynamics --event-handlers console_direct+
colcon test-result --verbose
```

Expected: the package configures, builds, and `test_pinocchio_dependency` passes. No Gazebo or hardware process should start.

- [x] **Step 4: Commit the package build target and test**

```bash
git add ros2_ws/src/robot_dynamics/CMakeLists.txt ros2_ws/src/robot_dynamics/test/test_pinocchio_dependency.cpp
git commit -m "feat: link Pinocchio in robot dynamics package"
```

---

### Task 3: Document the dependency boundary and verify workspace isolation

**Files:**
- Create: `ros2_ws/src/robot_dynamics/README.md`

**Interfaces:**
- Documents the prerequisite package, package-only build/test commands, and explicit non-goals for future maintainers.
- Does not alter any launch or runtime configuration.

- [x] **Step 1: Write the package README**

Create:

```markdown
# robot_dynamics

独立的 Pinocchio 依赖边界，为后续机器人运动学/动力学功能提供 C++ 包入口。

## 当前范围

当前版本只验证 Pinocchio 可由 ROS 2 Humble / ament_cmake 找到、编译和链接：

- 不提供 ROS 节点、topic、service 或参数。
- 不读取 URDF。
- 不参与 ros2_control、`motor_drive`、`motor_sim` 或 Gazebo 的运行路径。
- 不实现正逆运动学、逆动力学、重力补偿或轨迹前馈。

## 安装前提

```bash
source /opt/ros/humble/setup.bash
sudo -A apt install ros-humble-pinocchio
```

## 构建与测试

```bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_dynamics
source install/setup.bash
colcon test --packages-select robot_dynamics
colcon test-result --verbose
```

测试只构造一个空的 `pinocchio::Model`，不启动 Gazebo，也不连接真实硬件。
```

- [x] **Step 2: Run package-only and full workspace checks**

Run:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select robot_dynamics
colcon test --packages-select robot_dynamics
colcon test-result --verbose
```

Then run the existing package test suite without hardware or Gazebo:

```bash
colcon test
colcon test-result --verbose
```

Expected: the new package test passes and existing tests retain their prior behavior. If an unrelated pre-existing test fails, report its exact failure instead of changing unrelated packages.

- [x] **Step 3: Verify the final diff boundary**

Run:

```bash
git diff HEAD~2..HEAD --name-only
git status --short
```

Expected tracked changes are limited to the spec commit plus:

```text
ros2_ws/src/robot_dynamics/package.xml
ros2_ws/src/robot_dynamics/CMakeLists.txt
ros2_ws/src/robot_dynamics/test/test_pinocchio_dependency.cpp
ros2_ws/src/robot_dynamics/README.md
```

Generated workspace directories must remain untracked/ignored.

- [x] **Step 4: Commit the package documentation**

```bash
git add ros2_ws/src/robot_dynamics/README.md
git commit -m "docs: document Pinocchio dependency boundary"
```

---

## Execution Record

- 2026-09-22：已安装系统包 `ros-humble-pinocchio`（4.0.0-2jammy.20260606.100000）。
- 2026-09-22：`colcon build --symlink-install --packages-select robot_dynamics` 成功；`test_pinocchio_dependency` 通过。
- 2026-09-22：完整工作区 `colcon test` 通过，7 个包共 214 项测试，0 errors、0 failures、0 skipped。
- Pinocchio 的 CMake 配置会输出外部 `FindBoost` 对 `python310` header 的非致命警告；不影响构建或测试。

## Self-Review Checklist

- **Spec coverage:** the standalone package boundary, ROS-distribution dependency, smoke test, unchanged existing packages, installation prerequisite, package-only verification, and explicit non-goals are covered by Tasks 1–3.
- **No placeholders:** no `TBD`, `TODO`, or unspecified implementation step is required.
- **Type/target consistency:** the package is named `robot_dynamics`; the CMake interface target, exported target, and test link all use `${PROJECT_NAME}` / `robot_dynamics`; the test executable is consistently `test_pinocchio_dependency`.
- **Safety boundary:** verification never starts Gazebo or real hardware, and system installation uses the approved `sudo -A` flow if needed.
