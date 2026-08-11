# 历史实现计划（已被后续决策取代）

> 本文是 2026-08-10 四包分层重构的**历史计划**，仅供追溯。其中"真机与仿真共享
> 同一份 motor_canfd.yaml"等表述已被 [ADR 0002](../adr/0002-sim-topology-config-decoupled-from-hardware.md)、
> [ADR 0003](../adr/0003-hardware-bringup-entry-in-motor-hardware-interface.md) 与
> [ADR 0004](../adr/0004-fail-fast-on-activation-and-loopback-loss.md) 取代，请勿按本文执行。

# 四包分层重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把工程落成已确认的 4 包单向依赖链 `motor_canfd_drive ← motor_hardware_interface ← motor_canfd_sim ← upper_body_canfd_gazebo`：公开 YAML 解析 API、让 sim 复用解析并哑设备化、重组 gazebo 包 launch/config、反转依赖并更新文档。

**Architecture:** 底层 `motor_canfd_drive` 保持零 ROS 依赖、只收纯 C++ `DriverConfig`。`motor_hardware_interface` 持有 `motor_canfd.yaml` 并公开 `load_driver_config()` 作为 YAML→`DriverConfig` 的唯一边界。`motor_canfd_sim` 是"虚拟 CAN 电机设备"，只接收 `motor_config_file` 路径（由上层决定挂哪些电机），不再自带 YAML 解析。`upper_body_canfd_gazebo` 拥有全部仿真 launch（`gazebo.launch.py ← model_sim.launch.py ← driver_loopback.launch.py`）与控制器配置。

**Tech Stack:** ROS 2 Humble（C++17 / Python 3 launch）、ament_cmake / ament_python、ros2_control、yaml-cpp（经 `yaml_cpp_vendor`）、GoogleTest、pytest。

## Global Constraints

- 工作空间根：`/home/love009/robot_ws`；构建命令统一先 `source /opt/ros/humble/setup.bash`。
- C++17，编译选项 `-Wall -Wextra -Wpedantic`，缩进 2 空格；类 `PascalCase`，函数/变量 `snake_case`，常量 `k` 前缀。
- 中文注释说明设计意图；协议名、ROS 2 API 保持英文。
- 测试：C++ 用 `ament_cmake_gtest`，Python 用 pytest；launch 静态检查不得启动 Gazebo 或硬件。
- `motor_canfd_sim` 是唯一仿真后端，不得重新引入 Python CAN-FD 仿真路径。
- `motor_canfd_drive` 不得解析 YAML，不得新增 ROS 依赖。
- 提交用 conventional 前缀（`refactor:` / `docs:` 等），正文说明原因。
- 每个任务结束时必须能独立通过构建与本任务相关测试；Task 4 做全量验证。

---

### Task 1: motor_hardware_interface 公开 YAML→DriverConfig 解析 API

把 `canfd_system.cpp` 里的 YAML 拓扑解析提升为包级公开 API，供 `motor_canfd_sim` 复用，并补上解析单测。

**Files:**
- Create: `src/motor_hardware_interface/include/motor_hardware_interface/motor_config.hpp`
- Create: `src/motor_hardware_interface/src/motor_config.cpp`
- Create: `src/motor_hardware_interface/test/test_motor_config.cpp`
- Create: `src/motor_hardware_interface/test/fixtures/sample_motor_config.yaml`
- Create: `src/motor_hardware_interface/test/fixtures/bad_motor_config.yaml`
- Modify: `src/motor_hardware_interface/src/canfd_system.cpp`
- Modify: `src/motor_hardware_interface/CMakeLists.txt`

**Interfaces:**
- Consumes: `motor_canfd_drive::DriverConfig`、`motor_canfd_drive::MotorConfig`、`motor_canfd_drive::MotorParameterMap`（来自 `motor_canfd_drive/motor_canfd_driver.hpp`）。
- Produces: `motor_hardware_interface::load_driver_config(const std::string &path) -> motor_canfd_drive::DriverConfig`；空路径/文件不可读/schema 非法时抛 `std::runtime_error`。Task 2 的 sim 与 Task 1 改造后的 `canfd_system.cpp` 都调用它。

- [ ] **Step 1: 先写失败的单测与 fixture**

`src/motor_hardware_interface/test/fixtures/sample_motor_config.yaml`：

```yaml
motor_hardware:
  timing:
    feedback_timeout_s: 0.10
    initial_feedback_wait_s: 0.10
  buses:
    test_bus:
      interface: vcan0
  motor_types:
    rp:
      command:
        kp: 30.0
        kd: 1.0
      parameters:
        torque_constant_nm_per_amp: 0.35
    motor_2530:
      command:
        kp: 0.008
        kd: 1.0
      parameters:
        torque_constant_nm_per_amp: 0.35
  motors:
    shoulder_yaw:
      joint: shoulder_yaw_joint
      bus: test_bus
      type: rp
      id: 1
    wrist_pitch:
      joint: wrist_pitch_joint
      bus: test_bus
      type: motor_2530
      id: 2
      parameters:
        torque_constant_nm_per_amp: 0.55
        extra: per_motor
```

`src/motor_hardware_interface/test/fixtures/bad_motor_config.yaml`：

```yaml
not_motor_hardware:
  foo: bar
```

`src/motor_hardware_interface/test/test_motor_config.cpp`：

```cpp
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "motor_hardware_interface/motor_config.hpp"

namespace {

const std::string kSampleConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/sample_motor_config.yaml";
const std::string kBadConfig =
    std::string(TEST_SOURCE_DIR) + "/fixtures/bad_motor_config.yaml";

TEST(LoadDriverConfig, ParsesTopologyFromYaml) {
  const auto config = motor_hardware_interface::load_driver_config(kSampleConfig);
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses[0].name, "test_bus");
  EXPECT_EQ(config.buses[0].interface_name, "vcan0");

  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[0].name, "shoulder_yaw");
  EXPECT_EQ(config.motors[0].joint_name, "shoulder_yaw_joint");
  EXPECT_EQ(config.motors[0].bus, "test_bus");
  EXPECT_EQ(config.motors[0].type, "rp");
  EXPECT_EQ(config.motors[0].id, 1U);
  EXPECT_EQ(config.motors[0].parameters.at("torque_constant_nm_per_amp"), "0.35");
}

TEST(LoadDriverConfig, MotorOverridesTypeDefaults) {
  const auto config = motor_hardware_interface::load_driver_config(kSampleConfig);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].parameters.at("torque_constant_nm_per_amp"), "0.55");
  EXPECT_EQ(config.motors[1].parameters.at("extra"), "per_motor");
}

TEST(LoadDriverConfig, EmptyPathThrows) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(""),
               std::runtime_error);
}

TEST(LoadDriverConfig, MissingRootThrows) {
  EXPECT_THROW(motor_hardware_interface::load_driver_config(kBadConfig),
               std::runtime_error);
}

} // namespace
```

- [ ] **Step 2: 构建测试，确认因 API 不存在而失败**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/love009/robot_ws
colcon build --packages-select motor_hardware_interface
colcon test --packages-select motor_hardware_interface
```
Expected: 编译失败，`motor_config.hpp` 不存在（找不到头文件）。

- [ ] **Step 3: 实现公开 API 并改造 canfd_system.cpp 复用**

`src/motor_hardware_interface/include/motor_hardware_interface/motor_config.hpp`：

```cpp
#pragma once

#include <string>

#include "motor_canfd_drive/motor_canfd_driver.hpp"

namespace motor_hardware_interface {

// 读取电机拓扑配置（motor_canfd.yaml）并解析为纯 C++ DriverConfig。
// 本函数是 YAML 世界与底层驱动之间的唯一边界：motor_canfd_drive
// 不接触 YAML，只消费本函数产出的 DriverConfig。
// 空路径、文件不可读或内容不符合 schema 时抛 std::runtime_error。
motor_canfd_drive::DriverConfig load_driver_config(const std::string &path);

} // namespace motor_hardware_interface
```

`src/motor_hardware_interface/src/motor_config.cpp`：

```cpp
#include "motor_hardware_interface/motor_config.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace motor_hardware_interface {
namespace {

motor_canfd_drive::MotorParameterMap
parse_driver_parameters(const YAML::Node &node, const std::string &context) {
  // 底层 DriverConfig 不依赖 yaml-cpp，所以在这里把协议私有参数统一转换成
  // string->string。具体协议策略再按需要解释数值和单位。
  motor_canfd_drive::MotorParameterMap parameters;
  if (!node) {
    return parameters;
  }
  if (!node.IsMap()) {
    throw std::runtime_error(context + " parameters must be a YAML map");
  }
  for (const auto &entry : node) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error(context +
                               " parameters must contain scalar key/value pairs");
    }
    parameters[entry.first.Scalar()] = entry.second.Scalar();
  }
  return parameters;
}

void merge_parameters(motor_canfd_drive::MotorParameterMap &target,
                      const motor_canfd_drive::MotorParameterMap &overrides) {
  // 后写入的单电机参数覆盖 motor_types 默认值，使同型号电机可以只配置差异项。
  for (const auto &entry : overrides) {
    target[entry.first] = entry.second;
  }
}

motor_canfd_drive::DriverConfig parse_driver_config(const YAML::Node &hardware) {
  // 输出纯 C++ DriverConfig 后，底层驱动不再接触 YAML。
  motor_canfd_drive::DriverConfig config;
  const auto buses = hardware["buses"];
  if (!buses || !buses.IsMap() || buses.size() == 0U) {
    throw std::runtime_error("motor_hardware.buses must be a non-empty map");
  }
  for (const auto &entry : buses) {
    config.buses.push_back({entry.first.as<std::string>(),
                            entry.second["interface"].as<std::string>()});
  }

  std::unordered_map<std::string, motor_canfd_drive::MotorParameterMap>
      type_parameters;
  const auto motor_types = hardware["motor_types"];
  if (motor_types) {
    if (!motor_types.IsMap()) {
      throw std::runtime_error("motor_hardware.motor_types must be a map");
    }
    for (const auto &entry : motor_types) {
      const auto type = entry.first.as<std::string>();
      type_parameters[type] = parse_driver_parameters(
          entry.second["parameters"], "motor type '" + type + "'");
    }
  }

  const auto motors = hardware["motors"];
  if (!motors || !motors.IsMap() || motors.size() == 0U) {
    throw std::runtime_error("motor_hardware.motors must be a non-empty map");
  }
  for (const auto &entry : motors) {
    motor_canfd_drive::MotorConfig motor;
    motor.name = entry.first.as<std::string>();
    const auto node = entry.second;
    motor.joint_name = node["joint"].as<std::string>();
    motor.bus = node["bus"].as<std::string>();
    motor.type = node["type"].as<std::string>();
    motor.id = node["id"].as<unsigned>();
    const auto defaults = type_parameters.find(motor.type);
    if (defaults != type_parameters.end()) {
      motor.parameters = defaults->second;
    }
    merge_parameters(motor.parameters,
                     parse_driver_parameters(node["parameters"],
                                             "motor '" + motor.name + "'"));
    config.motors.push_back(std::move(motor));
  }
  return config;
}

} // namespace

motor_canfd_drive::DriverConfig load_driver_config(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("motor_config_file is required");
  }
  const auto document = YAML::LoadFile(path); // 文件不可读时抛 YAML::BadFile
  const auto hardware = document["motor_hardware"];
  if (!hardware || !hardware.IsMap()) {
    throw std::runtime_error("YAML root 'motor_hardware' is required");
  }
  return parse_driver_config(hardware);
}

} // namespace motor_hardware_interface
```

`src/motor_hardware_interface/src/canfd_system.cpp`：

- 删除匿名命名空间中的 `parse_driver_parameters`、`merge_parameters`、`parse_driver_config` 三个函数（它们整体已迁入 `motor_config.cpp`），保留 `has_state_interface` 与 `parse_command_config`。
- 增加头文件包含：`#include "motor_hardware_interface/motor_config.hpp"`（放在 `canfd_system.hpp` 包含之后）。
- 把 `CanFdSystem::load_config` 改为复用公开 API，仅保留 ROS 侧专属的 timing/命令增益解析：

```cpp
bool CanFdSystem::load_config(const std::string &path, std::string &error) {
  try {
    if (path.empty()) {
      throw std::runtime_error("hardware parameter 'config_file' is required");
    }
    // 拓扑（buses/motors/参数）由共享解析 API 产出，真机与模拟器共用一份 schema。
    config_.driver = load_driver_config(path);
    // 以下 ROS 侧专属段（反馈超时、命令增益）仍在本适配层解析。
    const auto document = YAML::LoadFile(path);
    const auto hardware = document["motor_hardware"];
    if (!hardware || !hardware.IsMap()) {
      throw std::runtime_error("YAML root 'motor_hardware' is required");
    }
    const auto timing = hardware["timing"];
    if (!timing || !timing.IsMap()) {
      throw std::runtime_error("motor_hardware.timing is required");
    }
    config_.feedback_timeout_s = timing["feedback_timeout_s"].as<double>();
    config_.initial_feedback_wait_s =
        timing["initial_feedback_wait_s"].as<double>();
    if (!std::isfinite(config_.feedback_timeout_s) ||
        !std::isfinite(config_.initial_feedback_wait_s) ||
        config_.feedback_timeout_s <= 0.0 ||
        config_.initial_feedback_wait_s <= 0.0) {
      throw std::runtime_error("feedback timeouts must be finite positive values");
    }

    const auto types = hardware["motor_types"];
    const auto motors = hardware["motors"];
    config_.motor_commands.clear();
    config_.motor_commands.reserve(config_.driver.motors.size());
    for (const auto &motor : config_.driver.motors) {
      const auto motor_node = motors[motor.name];
      const auto type_node = types ? types[motor.type] : YAML::Node{};
      const auto gains = parse_command_config(type_node, motor_node, motor.name);
      config_.motor_commands.push_back({gains.first, gains.second});
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}
```

`src/motor_hardware_interface/CMakeLists.txt`：库源增加 `src/motor_config.cpp`，并加测试：

```cmake
add_library(${PROJECT_NAME} SHARED
  src/canfd_system.cpp
  src/motor_config.cpp)
```

在 `ament_package()` 前追加：

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_config test/test_motor_config.cpp)
  target_compile_definitions(test_motor_config PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_motor_config ${PROJECT_NAME})
endif()
```

- [ ] **Step 4: 构建并跑测试，确认通过**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/love009/robot_ws
colcon build --packages-select motor_hardware_interface
colcon test --packages-select motor_hardware_interface
colcon test-result --verbose
```
Expected: `test_motor_config` 4 个用例全部 PASS；`colcon test-result` 无 FAILED。

- [ ] **Step 5: 提交**

```bash
git add src/motor_hardware_interface
git commit -m "refactor(motor_hardware_interface): 公开 YAML→DriverConfig 解析 API 并补单测"
```

---

### Task 2: motor_canfd_sim 复用共享解析 + 独立 launch

删除 sim 自己的 YAML 解析副本，改用 `motor_hardware_interface::load_driver_config`；新增只启动模拟器的独立 launch（`motor_config_file` 必须显式传入）。

**Files:**
- Modify: `src/motor_canfd_sim/src/motor_canfd_sim_node.cpp`
- Modify: `src/motor_canfd_sim/CMakeLists.txt`
- Modify: `src/motor_canfd_sim/package.xml`
- Create: `src/motor_canfd_sim/launch/motor_canfd_sim.launch.py`
- Create: `src/motor_canfd_sim/test/test_motor_canfd_sim_launch.py`

**Interfaces:**
- Consumes: `motor_hardware_interface::load_driver_config(const std::string &)`（Task 1 产出）。
- Produces: `motor_canfd_sim` 的 `motor_canfd_sim_node` 行为不变（仍从 `motor_config_file`、`bus_override.*`、`sim.yaml` 取参数）；独立 launch `motor_canfd_sim.launch.py` 声明 `motor_config_file`/`left_interface`/`right_interface`/`allow_hardware_interfaces` 四个参数。Task 3 的 `model_sim.launch.py` 会以 `motor_canfd_sim_node` 为启动目标。

- [ ] **Step 1: 改造 sim 节点，移除重复解析**

`src/motor_canfd_sim/src/motor_canfd_sim_node.cpp`：

- 删除 `#include <yaml-cpp/yaml.h>`。
- 增加 `#include "motor_hardware_interface/motor_config.hpp"`。
- 删除匿名命名空间中的 `parse_driver_parameters`、`merge_parameters`、`load_driver_config_yaml` 三个函数。
- 构造函数里把 `driver_config_ = load_driver_config_yaml(config_file);` 改为：

```cpp
    // 设备清单由调用方决定：本节点只接收配置路径，解析复用硬件接口的公开 API。
    driver_config_ = motor_hardware_interface::load_driver_config(config_file);
```

`src/motor_canfd_sim/CMakeLists.txt`：

- `find_package` 删除 `yaml_cpp_vendor`，新增 `motor_hardware_interface`。
- `target_link_libraries(motor_canfd_sim_node motor_hardware_interface motor_canfd_drive)`，删除 `yaml-cpp`。
- pytest 注册改为新测试：`ament_add_pytest_test(test_motor_canfd_sim_launch test/test_motor_canfd_sim_launch.py)`。

`src/motor_canfd_sim/package.xml`：删除 `<depend>yaml_cpp_vendor</depend>`、`<exec_depend>controller_manager</exec_depend>`、`<exec_depend>upper_body_canfd_gazebo</exec_depend>`、`<exec_depend>xacro</exec_depend>`；新增 `<depend>motor_hardware_interface</depend>`。保留 `motor_canfd_drive`、`rclcpp`、`sensor_msgs`、`std_msgs`、`launch`、`launch_ros`。

- [ ] **Step 2: 新增独立 launch 与其静态测试**

`src/motor_canfd_sim/launch/motor_canfd_sim.launch.py`：

```python
"""只启动 C++ 电机模拟器（虚拟 CAN 电机设备），不启动 Gazebo 与驱动控制器。

挂多少、挂哪些电机由调用方决定：必须显式传入 motor_config_file
（例如 upper_body_canfd_gazebo 的 launch 传入共享 motor_canfd.yaml）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    motor_config_file = LaunchConfiguration("motor_config_file")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")

    simulator = Node(
        package="motor_canfd_sim",
        executable="motor_canfd_sim_node",
        name="motor_canfd_sim",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("motor_canfd_sim"), "config", "sim.yaml"]
            ),
            {
                "motor_config_file": motor_config_file,
                "bus_override.left_arm": left_interface,
                "bus_override.right_arm": right_interface,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motor_config_file",
                default_value="",
                description="电机拓扑配置路径；模拟器不预设机器人拓扑，必须由调用方指定",
            ),
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            simulator,
        ]
    )
```

`src/motor_canfd_sim/test/test_motor_canfd_sim_launch.py`：

```python
"""静态检查 motor_canfd_sim 独立 launch，不启动 Gazebo 或任何进程。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "motor_canfd_sim.launch.py"
    spec = importlib.util.spec_from_file_location("motor_canfd_sim_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_requires_explicit_motor_config_file():
    description = _load_launch_description()
    context = LaunchContext()

    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    for declaration in declarations.values():
        declaration.execute(context)

    # 独立 launch 不预设机器人拓扑：motor_config_file 默认必须为空。
    assert context.launch_configurations["motor_config_file"] == ""
    assert context.launch_configurations["left_interface"] == "vcan0"
    assert context.launch_configurations["right_interface"] == "vcan1"

    simulator = next(action for action in description.entities if isinstance(action, Node))
    parameters = evaluate_parameters(context, simulator._Node__parameters)
    injected = next(parameter for parameter in parameters if isinstance(parameter, dict))
    assert injected["motor_config_file"] == ""
    assert injected["bus_override.left_arm"] == "vcan0"
    assert injected["bus_override.right_arm"] == "vcan1"
```

- [ ] **Step 3: 构建并跑 motor_canfd_sim 测试**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/love009/robot_ws
colcon build --packages-select motor_hardware_interface motor_canfd_sim
colcon test --packages-select motor_canfd_sim
colcon test-result --verbose
```
Expected: 编译通过；`test_motor_canfd_sim_launch` PASS；无 FAILED。

- [ ] **Step 4: 提交**

```bash
git add src/motor_canfd_sim
git commit -m "refactor(motor_canfd_sim): 复用共享解析 API 并新增独立 launch"
```

---

### Task 3: upper_body_canfd_gazebo launch/config 重组与依赖反转

把仿真 launch 全部收进 `upper_body_canfd_gazebo`，形成 `gazebo.launch.py ← model_sim.launch.py ← driver_loopback.launch.py` 嵌套链；合并控制器配置；`motor_canfd_sim` 不再 exec 依赖 gazebo 包。

**Files:**
- Rename: `src/upper_body_canfd_gazebo/launch/sim.launch.py` → `src/upper_body_canfd_gazebo/launch/gazebo.launch.py`
- Create: `src/upper_body_canfd_gazebo/launch/model_sim.launch.py`
- Create: `src/upper_body_canfd_gazebo/launch/driver_loopback.launch.py`
- Delete: `src/motor_canfd_sim/launch/gazebo_cpp_sim.launch.py`
- Delete: `src/motor_canfd_sim/launch/driver_loopback.launch.py`
- Delete: `src/motor_canfd_sim/config/loopback_controllers.yaml`
- Delete: `src/motor_canfd_sim/test/test_gazebo_cpp_sim_launch.py`
- Rename: `src/upper_body_canfd_gazebo/config/hardware_controllers.yaml` → `src/upper_body_canfd_gazebo/config/driver_controllers.yaml`（内容替换为合并版）
- Modify: `src/upper_body_canfd_gazebo/launch/hardware.launch.py`
- Create: `src/upper_body_canfd_gazebo/test/test_model_sim_launch.py`
- Modify: `src/upper_body_canfd_gazebo/package.xml`

**Interfaces:**
- Consumes: `motor_canfd_sim` 的 `motor_canfd_sim_node` 可执行文件与 `config/sim.yaml`（Task 2 保留）；`gazebo.launch.py` 的 `gui`/`render_engine` 参数。
- Produces: `model_sim.launch.py` 声明 `left_interface`/`right_interface`/`gui`/`render_engine`/`motor_config_file`；`driver_loopback.launch.py` 声明同样的五个参数并把它们透传给 `model_sim.launch.py`。`driver_controllers.yaml` 同时提供根命名空间（真机）与 `/driver` 命名空间（回环）的控制器配置。

- [ ] **Step 1: 重命名 Gazebo 基础 launch 并更新文档字符串**

```bash
git mv src/upper_body_canfd_gazebo/launch/sim.launch.py \
       src/upper_body_canfd_gazebo/launch/gazebo.launch.py
```

把文件顶部 docstring 改为：

```python
"""启动 Gazebo Fortress、机器人模型和 Gazebo 侧 effort controller。

本文件是模型侧基础 launch，不启动电机模拟器；由 model_sim.launch.py
（+ 模拟器）与 driver_loopback.launch.py（+ 驱动控制器）嵌套引入。
"""
```

文件其余内容保持不变。

- [ ] **Step 2: 新增 model_sim.launch.py（原 gazebo_cpp_sim，清理 start_bridge）**

`src/upper_body_canfd_gazebo/launch/model_sim.launch.py`：

```python
"""Gazebo 模型 + C++ 电机模拟器，不含 ros2_control 驱动控制器。

供模型侧开发/测试使用；完整回环请用 driver_loopback.launch.py。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")
    motor_config_file = LaunchConfiguration("motor_config_file")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "render_engine": render_engine,
        }.items(),
    )

    # 模拟器只接收配置路径；挂哪些电机由本包（上层集成）决定。
    simulator = Node(
        package="motor_canfd_sim",
        executable="motor_canfd_sim_node",
        name="motor_canfd_sim",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("motor_canfd_sim"), "config", "sim.yaml"]
            ),
            {
                "motor_config_file": motor_config_file,
                "bus_override.left_arm": left_interface,
                "bus_override.right_arm": right_interface,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("motor_hardware_interface"), "config", "motor_canfd.yaml"]
                ),
                description="电机拓扑配置路径；默认使用与真机相同的 motor_canfd.yaml",
            ),
            gazebo,
            simulator,
        ]
    )
```

- [ ] **Step 3: 新增 driver_loopback.launch.py（完整回环）**

`src/upper_body_canfd_gazebo/launch/driver_loopback.launch.py`：

```python
"""完整回环：gazebo.launch.py 基础 + C++ 电机模拟器 + /driver ros2_control。

模拟器（model_sim.launch.py 内的 motor_canfd_sim 节点）启动后才拉起
/driver/controller_manager，避免硬件接口激活时因模拟器未就绪收不到反馈。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_simulator(action):
    return (
        isinstance(action, Node)
        and action.node_package == "motor_canfd_sim"
        and action.node_executable == "motor_canfd_sim_node"
    )


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")
    motor_config_file = LaunchConfiguration("motor_config_file")

    model_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "model_sim.launch.py"]
            )
        ),
        launch_arguments={
            "left_interface": left_interface,
            "right_interface": right_interface,
            "gui": gui,
            "render_engine": render_engine,
            "motor_config_file": motor_config_file,
        }.items(),
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("upper_body_canfd_gazebo"), "urdf", "upper_body.urdf.xacro"]
                ),
                " hardware_backend:=canfd left_interface:=",
                left_interface,
                " right_interface:=",
                right_interface,
                " motor_config_file:=",
                motor_config_file,
            ]
        ),
        value_type=str,
    )
    # 真机与回环共用同一份控制器配置，区别只有 controller_manager 命名空间。
    controller_config = PathJoinSubstitution(
        [FindPackageShare("upper_body_canfd_gazebo"), "config", "driver_controllers.yaml"]
    )
    # /driver 命名空间避免与 Gazebo 自己的 /controller_manager 重名；
    # 二者通过 CAN-FD/vcan 和关节状态形成完整回环。
    driver_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="driver",
        output="screen",
        parameters=[{"robot_description": robot_description}, controller_config],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )

    # 事件处理器必须先注册，再执行 model_sim（其中的模拟器节点才会触发
    # OnProcessStart(_is_simulator)），否则会漏掉模拟器启动事件。
    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("motor_hardware_interface"), "config", "motor_canfd.yaml"]
                ),
                description="电机拓扑配置路径；默认与真机一致",
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=_is_simulator, on_start=[driver_manager])
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=driver_manager, on_start=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[position_controller])
            ),
            model_sim,
        ]
    )
```

- [ ] **Step 4: 合并控制器配置并改真机 launch 引用**

```bash
git mv src/upper_body_canfd_gazebo/config/hardware_controllers.yaml \
       src/upper_body_canfd_gazebo/config/driver_controllers.yaml
```

`src/upper_body_canfd_gazebo/config/driver_controllers.yaml` 内容替换为：

```yaml
# 真机与 /driver 回环共用同一套 position 控制器配置，区别只有命名空间：
# - 真机：ros2_control_node 在根命名空间运行 → 匹配 controller_manager 键；
# - 回环：ros2_control_node 在 /driver 命名空间运行 → 匹配 /driver/* 键。
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    position_controller:
      type: forward_command_controller/ForwardCommandController

/driver/controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    position_controller:
      type: forward_command_controller/ForwardCommandController

position_controller:
  ros__parameters:
    joints:
      - left_shoulder_yaw_joint
      - left_shoulder_roll_joint
      - left_shoulder_pitch_joint
      - left_elbow_pitch_joint
      - left_wrist_yaw_joint
      - left_wrist_pitch_joint
      - left_wrist_roll_joint
      - right_shoulder_yaw_joint
      - right_shoulder_roll_joint
      - right_shoulder_pitch_joint
      - right_elbow_pitch_joint
      - right_wrist_yaw_joint
      - right_wrist_pitch_joint
      - right_wrist_roll_joint
    interface_name: position

/driver/position_controller:
  ros__parameters:
    joints:
      - left_shoulder_yaw_joint
      - left_shoulder_roll_joint
      - left_shoulder_pitch_joint
      - left_elbow_pitch_joint
      - left_wrist_yaw_joint
      - left_wrist_pitch_joint
      - left_wrist_roll_joint
      - right_shoulder_yaw_joint
      - right_shoulder_roll_joint
      - right_shoulder_pitch_joint
      - right_elbow_pitch_joint
      - right_wrist_yaw_joint
      - right_wrist_pitch_joint
      - right_wrist_roll_joint
    interface_name: position
```

`src/upper_body_canfd_gazebo/launch/hardware.launch.py`：把
`PathJoinSubstitution([share, "config", "hardware_controllers.yaml"])` 改为
`PathJoinSubstitution([share, "config", "driver_controllers.yaml"])`，其余不变。

- [ ] **Step 5: 删除 motor_canfd_sim 中已迁移的旧文件，迁移 launch 静态测试**

```bash
git rm src/motor_canfd_sim/launch/gazebo_cpp_sim.launch.py
git rm src/motor_canfd_sim/launch/driver_loopback.launch.py
git rm src/motor_canfd_sim/config/loopback_controllers.yaml
git rm src/motor_canfd_sim/test/test_gazebo_cpp_sim_launch.py
```

创建 `src/upper_body_canfd_gazebo/test/test_model_sim_launch.py`：

```python
"""静态检查 model_sim 入口（Gazebo 模型 + C++ 模拟器），不启动 Gazebo 或任何进程。"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "model_sim.launch.py"
    spec = importlib.util.spec_from_file_location("model_sim_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_motor_configuration_and_bus_overrides_are_injected():
    description = _load_launch_description()
    context = LaunchContext()

    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    for declaration in declarations.values():
        declaration.execute(context)

    assert context.launch_configurations["left_interface"] == "vcan0"
    assert context.launch_configurations["right_interface"] == "vcan1"

    simulator = next(action for action in description.entities if isinstance(action, Node))
    parameters = evaluate_parameters(context, simulator._Node__parameters)
    injected = next(parameter for parameter in parameters if isinstance(parameter, dict))

    expected_config = Path(get_package_share_directory("motor_hardware_interface")) / (
        "config/motor_canfd.yaml"
    )
    assert Path(injected["motor_config_file"]).resolve() == expected_config.resolve()
    assert injected["bus_override.left_arm"] == "vcan0"
    assert injected["bus_override.right_arm"] == "vcan1"
```

- [ ] **Step 6: 更新 upper_body_canfd_gazebo 依赖**

`src/upper_body_canfd_gazebo/package.xml`：新增 `<exec_depend>motor_canfd_sim</exec_depend>`（放在 `<exec_depend>motor_hardware_interface</exec_depend>` 之前），其余不变。`setup.py` 用 glob 自动收新 launch/config 文件，无需改动。

- [ ] **Step 7: 构建并跑三个包的相关测试**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/love009/robot_ws
colcon build --packages-select motor_hardware_interface motor_canfd_sim upper_body_canfd_gazebo
colcon test --packages-select motor_canfd_sim upper_body_canfd_gazebo
colcon test-result --verbose
```
Expected: 三个包构建通过；`test_motor_canfd_sim_launch` 与 `test_model_sim_launch` PASS；gazebo 包原有 pytest（motion library / ros action player）仍 PASS；无 FAILED。

- [ ] **Step 8: 提交**

```bash
git add src/motor_canfd_sim src/upper_body_canfd_gazebo
git commit -m "refactor(upper_body_canfd_gazebo): 重组 launch 链与控制器配置并反转依赖"
```

---

### Task 4: 更新文档并全量验证

把 README/AGENTS 里的启动命令与结构描述更新到新布局，然后全量构建与测试。

**Files:**
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `src/motor_canfd_sim/README.md`
- Modify: `src/upper_body_canfd_gazebo/README.md`
- Modify: `src/motor_hardware_interface/README.md`

**Interfaces:**
- Consumes: Task 1–3 的文件布局与 launch 名。

- [ ] **Step 1: 更新根 README.md**

`README.md`：
- 把 `ros2 launch motor_canfd_sim driver_loopback.launch.py` 改为：
  ```bash
  ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py
  ```
- 把 `motor_canfd_sim` 一行改为 `C++ 电机侧 CAN-FD 模拟器（虚拟 CAN 电机设备，清单由上层决定）`。

- [ ] **Step 2: 更新 AGENTS.md**

`AGENTS.md`：
- 把 `ros2 launch motor_canfd_sim driver_loopback.launch.py` 改为 `ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py`。
- 把"launch 文件静态检查用 `ament_add_pytest_test`（见 `motor_canfd_sim`）"改为：
  ```
  launch 文件静态检查：motor_canfd_sim 用 ament_add_pytest_test，
  upper_body_canfd_gazebo 用 pytest 自动发现；测试不得启动 Gazebo 或硬件。
  ```

- [ ] **Step 3: 更新 motor_canfd_sim/README.md**

`src/motor_canfd_sim/README.md`：
- 首段"它读取与 `motor_hardware_interface` 相同的 YAML 结构"改为"它通过 `motor_hardware_interface` 的公开解析 API 复用同一份 `motor_canfd.yaml`（拓扑由上层决定）"。
- 把 `ros2 launch motor_canfd_sim driver_loopback.launch.py` 改为 `ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py`。
- 把 `ros2 launch motor_canfd_sim gazebo_cpp_sim.launch.py` 改为 `ros2 launch upper_body_canfd_gazebo model_sim.launch.py`。
- 增加独立启动说明：
  ```bash
  # 只启动模拟器（虚拟 CAN 电机设备）：必须显式指定电机拓扑配置路径
  ros2 launch motor_canfd_sim motor_canfd_sim.launch.py \
    motor_config_file:=$(ros2 pkg prefix motor_hardware_interface --share)/config/motor_canfd.yaml
  ```

- [ ] **Step 4: 更新 upper_body_canfd_gazebo/README.md**

`src/upper_body_canfd_gazebo/README.md`：
- 把"由 `motor_canfd_sim/launch/driver_loopback.launch.py` 作为基础 launch 引入"改为"由 `launch/driver_loopback.launch.py` 作为完整回环入口嵌套引入"。
- 把 `ros2 launch motor_canfd_sim driver_loopback.launch.py` 改为 `ros2 launch upper_body_canfd_gazebo driver_loopback.launch.py`。
- "主要文件"清单改为：
  ```text
  launch/gazebo.launch.py                 Gazebo 基础仿真（模型+世界+effort）
  launch/model_sim.launch.py              Gazebo + C++ 电机模拟器（无驱动控制器）
  launch/driver_loopback.launch.py        完整回环（+ /driver ros2_control）
  launch/hardware.launch.py               真机 ros2_control bring-up
  urdf/upper_body.urdf.xacro              14-DoF 模型与硬件后端选择
  worlds/upper_body_world.sdf             Gazebo Fortress 世界
  config/controllers.yaml                 Gazebo effort controller
  config/driver_controllers.yaml          真机与 /driver 回环共用的 position controller
  upper_body_canfd_gazebo/motion_library.py
  upper_body_canfd_gazebo/ros_action_player.py
  scripts/setup_vcan.sh
  ```

- [ ] **Step 5: 更新 motor_hardware_interface/README.md**

`src/motor_hardware_interface/README.md`：在"配置文件"一节末尾追加：

```markdown
`config/motor_canfd.yaml` 是电机拓扑的单一配置源。解析入口是公开 API
`motor_hardware_interface::load_driver_config(path)`（`motor_config.hpp`），
它产出纯 C++ `DriverConfig` 后交给 `motor_canfd_drive`；`motor_canfd_sim`
通过同一 API 复用解析，不再自带 YAML 解析副本。
```

- [ ] **Step 6: 全量构建与测试**

Run:
```bash
source /opt/ros/humble/setup.bash
cd /home/love009/robot_ws
colcon build --symlink-install
colcon test
colcon test-result --verbose
```
Expected: 全部包构建成功，所有测试 PASS，无 FAILED。另外确认旧入口已消失：
```bash
ls src/motor_canfd_sim/launch src/upper_body_canfd_gazebo/launch
```
Expected: `motor_canfd_sim/launch` 只有 `motor_canfd_sim.launch.py`；gazebo 包有 `gazebo.launch.py`、`model_sim.launch.py`、`driver_loopback.launch.py`、`hardware.launch.py`。

- [ ] **Step 7: 提交**

```bash
git add README.md AGENTS.md src
git commit -m "docs: 更新四包分层后的 README、AGENTS 与启动命令"
```
