# 单包 30 轴人形机器人 Gazebo 仿真实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将仓库收敛为唯一 ROS 2 包 `humanoid_gazebo`，提供可由三路 USB 串口或 USB-CANFD 串口控制的 170 cm、45 kg、30 轴悬空人形机器人 Gazebo 仿真。

**Architecture:** `humanoid_gazebo` 是唯一 ament package，包内用 `humanoid_protocol`、`humanoid_transport`、`humanoid_motor_sim_core` 三个 C++ 深模块隔离协议、传输与电机仿真，ROS 节点只负责把三路串口帧、Gazebo joint state 和 effort controller 命令连接起来。机械 YAML 与电机 YAML 分别作为机械参数和电气/传动参数的单一来源，静态契约测试保证 30 个 joint 一一对应。

**Tech Stack:** ROS 2 Humble、C++17、ament_cmake、ament_cmake_python、Python 3、GoogleTest、pytest、yaml-cpp、ros2_control、gz_ros2_control、Gazebo Fortress、xacro、POSIX tty、CANable2 SLCAN-FD。

**Spec:** `docs/superpowers/specs/2026-08-25-humanoid-canfd-simulation-design.md`

## Global Constraints

- 开发分支为 `dev/smi`；不修改或提交未跟踪的 `ros2_ws/`。
- 最终 `colcon list --names-only` 只能发现 `humanoid_gazebo`，不得保留旧 ROS package、兼容 package 或真机 bring-up。
- 不保留 vcan、SocketCAN、本机 driver loopback 或外部 ros2_control hardware plugin。
- C++17，`-Wall -Wextra -Wpedantic`，2 空格缩进；Python 使用类型注解。
- 行为改动执行 RED → GREEN → REFACTOR；测试不得依赖 Gazebo GUI 或真实串口硬件。
- 两种运行模式固定为 `usb_serial` 与 `serial_slcan`；单进程只能选择一种，三条总线必须各绑定唯一的 `/dev/serial/by-id/...`。
- 30 轴顺序固定为左臂 9、右臂 9、左腿 6、右腿 6；总线固定为 `left_arm`、`right_arm`、`legs`。
- 默认控制 100 Hz、RP 反馈 100 Hz、Vaxor 状态 50 Hz、command timeout 150 ms、Gazebo physics 目标 1 kHz。
- `serial_slcan` 只有显式 `allow_external_canfd:=true` 才能启动。
- 协议与执行器参数中的仿名义值不得写成厂家保证值。

---

### Task 1: 建立唯一的混合 C++/Python 包壳

**Files:**
- Move: `src/humanoid_canfd_gazebo/` → `src/humanoid_gazebo/`
- Create: `src/humanoid_gazebo/CMakeLists.txt`
- Modify: `src/humanoid_gazebo/package.xml`
- Move: `src/humanoid_gazebo/humanoid_canfd_gazebo/` → `src/humanoid_gazebo/humanoid_gazebo/`
- Move: `src/humanoid_gazebo/resource/humanoid_canfd_gazebo` → `src/humanoid_gazebo/resource/humanoid_gazebo`
- Delete: `src/humanoid_gazebo/setup.py`, `src/humanoid_gazebo/setup.cfg`
- Create: `src/humanoid_gazebo/scripts/ros_actions`
- Modify: `src/humanoid_gazebo/test/test_package_identity.py`
- Modify: all existing Python imports and launch package references under `src/humanoid_gazebo/`

**Interfaces:**
- Consumes: existing `ament_python` package and its six pytest files.
- Produces: ament package `humanoid_gazebo`; Python module `humanoid_gazebo`; CMake targets can be added under the same package in later tasks.

- [ ] **Step 1: Write the failing package identity test**

```python
def test_integration_package_has_final_identity() -> None:
    package_xml = ROOT / "src/humanoid_gazebo/package.xml"
    assert package_xml.exists()
    assert ElementTree.parse(package_xml).findtext("name") == "humanoid_gazebo"
    assert (ROOT / "src/humanoid_gazebo/CMakeLists.txt").exists()
    assert not (ROOT / "src/humanoid_canfd_gazebo").exists()
```

- [ ] **Step 2: Run the identity test and verify RED**

Run: `python3 -m pytest src/humanoid_canfd_gazebo/test/test_package_identity.py -v`

Expected: FAIL because `src/humanoid_gazebo/package.xml` does not exist.

- [ ] **Step 3: Move the package and convert it to `ament_cmake`**

Move the directory and Python module/resource marker, replace all `humanoid_canfd_gazebo` identifiers with `humanoid_gazebo`, and use this build skeleton:

```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ros_actions DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB PYTEST_FILES CONFIGURE_DEPENDS test/test_*.py)
  foreach(PYTEST_FILE ${PYTEST_FILES})
    get_filename_component(TEST_NAME ${PYTEST_FILE} NAME_WE)
    ament_add_pytest_test(${TEST_NAME} ${PYTEST_FILE})
  endforeach()
endif()

ament_package()
```

Set `package.xml` build type to `ament_cmake`, add `ament_cmake_python`, and remove dependencies on `motor_sim` and `motor_hardware_interface` from this package boundary.
`scripts/ros_actions` is an executable Python wrapper that imports and calls
`humanoid_gazebo.ros_action_player:main`, preserving `ros2 run humanoid_gazebo ros_actions`.

- [ ] **Step 4: Build and run the renamed package tests**

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select humanoid_gazebo`

Run: `source install/setup.bash && colcon test --packages-select humanoid_gazebo && colcon test-result --verbose`

Expected: build succeeds and all retained Python tests pass.

- [ ] **Step 5: Commit the package shell**

```bash
git add src/humanoid_canfd_gazebo src/humanoid_gazebo
git commit -m "refactor: rename Gazebo package"
```

---

### Task 2: 迁入 RP/Vaxor 协议与帧记录编解码

**Files:**
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/protocol/can_frame.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/protocol/motor_protocol.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/protocol/rp_protocol.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/protocol/vaxor_protocol.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/transport/frame_record.hpp`
- Create: `src/humanoid_gazebo/src/protocol/motor_protocol.cpp`
- Create: `src/humanoid_gazebo/src/protocol/rp_protocol.cpp`
- Create: `src/humanoid_gazebo/src/protocol/vaxor_protocol.cpp`
- Create: `src/humanoid_gazebo/src/transport/frame_record.cpp`
- Create: `src/humanoid_gazebo/docs/protocol/16-30mm外置驱动器使用说明手册_20260720.pdf`
- Create: `src/humanoid_gazebo/docs/protocol/25-30mm驱控一体使用说明手册_20260204.pdf`
- Create: `src/humanoid_gazebo/docs/protocol/意优RP系列行星关节CAN-FD通信协议.pdf`
- Create: `src/humanoid_gazebo/test/test_motor_protocol.cpp`
- Create: `src/humanoid_gazebo/test/test_frame_record.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`, `src/humanoid_gazebo/package.xml`

**Interfaces:**
- Consumes: verified protocol behavior from `src/motor_drive/src/protocol.cpp`, `motor_protocol.cpp`, `slcan.cpp` and their tests.
- Produces: `CanFrame {uint32_t id; bool extended; bool fd; bool brs; std::array<uint8_t, 64> data; size_t size;}`; `MotorProtocol::decode_command(const CanFrame&)`; `MotorProtocol::encode_feedback(const MotorFeedback&)`; `encode_frame_record(const CanFrame&) -> std::string`; `FrameRecordParser::push(std::string_view) -> std::vector<CanFrame>`.

- [ ] **Step 1: Write failing golden-vector protocol tests**

```cpp
TEST(RpProtocol, DecodesPositionCommandGoldenVector) {
  const CanFrame frame = frame_from_bytes(0x141, {0xA4, 0, 0, 0, 0x10, 0, 0, 0});
  const auto command = RpProtocol{}.decode_command(frame);
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->motor_id, 1U);
  EXPECT_EQ(command->mode, CommandMode::Position);
}

TEST(VaxorProtocol, FeedbackRoundTripsAtMotorSide) {
  const MotorFeedback input{1, 0.25, -0.5, 2.0, 36.0};
  const auto decoded = VaxorProtocol{}.decode_feedback(
      VaxorProtocol{}.encode_feedback(input));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_NEAR(decoded->position_rad, input.position_rad, 1e-4);
}
```

- [ ] **Step 2: Write failing fragmented frame-record tests**

```cpp
TEST(FrameRecordParser, ReassemblesFragmentedFdBrsRecord) {
  FrameRecordParser parser;
  EXPECT_TRUE(parser.push("b1233AABB").empty());
  const auto frames = parser.push("CC\r");
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x123U);
  EXPECT_TRUE(frames[0].fd);
  EXPECT_TRUE(frames[0].brs);
  EXPECT_EQ(frames[0].size, 3U);
}

TEST(FrameRecordParser, RejectsInvalidHexWithoutLosingNextRecord) {
  FrameRecordParser parser;
  const auto frames = parser.push("d1231GG\rd1231AA\r");
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].data[0], 0xAA);
}
```

- [ ] **Step 3: Run the focused tests and verify RED**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo --cmake-args -DBUILD_TESTING=ON`

Expected: FAIL because the protocol and frame-record headers do not exist.

- [ ] **Step 4: Implement the package-internal protocol libraries**

Port only RP, Vaxor and SLCAN-FD frame-record logic. Move the three protocol PDFs from `src/motor_sim/` into `src/humanoid_gazebo/docs/protocol/` without modifying their bytes. Do not port `SocketCan`, `MotorCanfdDriver`, CAN interface discovery or public ament exports. Register exactly these protocol names:

```cpp
std::unique_ptr<MotorProtocol> make_motor_protocol(std::string_view name) {
  if (name == "rp") return std::make_unique<RpProtocol>();
  if (name == "motor_2530") return std::make_unique<VaxorProtocol>();
  throw std::invalid_argument("unsupported motor protocol");
}
```

`FrameRecordParser` accepts `t/T/d/D/b/B`, buffers until `\r`, rejects malformed records one record at a time, caps the pending record at 256 bytes, and never treats adapter status lines as CAN frames.

- [ ] **Step 5: Run protocol tests and verify GREEN**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo && source install/setup.bash && colcon test --packages-select humanoid_gazebo --ctest-args -R 'test_motor_protocol|test_frame_record' && colcon test-result --verbose`

Expected: both new GoogleTest targets pass.

- [ ] **Step 6: Commit protocol migration**

```bash
git add src/humanoid_gazebo
git commit -m "feat: embed motor protocols in humanoid simulation"
```

---

### Task 3: 实现统一 POSIX tty 传输

**Files:**
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/transport/frame_transport.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/transport/serial_transport.hpp`
- Create: `src/humanoid_gazebo/src/transport/serial_transport.cpp`
- Create: `src/humanoid_gazebo/test/test_serial_transport.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`

**Interfaces:**
- Consumes: `CanFrame`, `encode_frame_record`, `FrameRecordParser` from Task 2.
- Produces: `enum class ReceiveResult { Frame, NoData, Disconnected }; enum class TransportMode { UsbSerial, SerialSlcan }; class FrameTransport { open, close, send, receive }; SerialTransport constructor `(std::string device, TransportMode mode, int baud_rate)`.

- [ ] **Step 1: Write failing pseudo-terminal transport tests**

```cpp
TEST(SerialTransport, UsbSerialDoesNotSendAdapterCommands) {
  PseudoTerminal pty;
  SerialTransport transport(pty.slave_path(), TransportMode::UsbSerial, 3000000);
  transport.open();
  EXPECT_EQ(pty.read_with_timeout(20ms), "");
}

TEST(SerialTransport, SerialSlcanInitializesCanableInOrder) {
  PseudoTerminal pty;
  SerialTransport transport(pty.slave_path(), TransportMode::SerialSlcan, 3000000);
  transport.open();
  EXPECT_EQ(pty.read_exact(12), "C\rS8\rY5\rO\r");
}

TEST(SerialTransport, ReportsHangupAsDisconnected) {
  PseudoTerminal pty;
  SerialTransport transport(pty.slave_path(), TransportMode::UsbSerial, 3000000);
  transport.open();
  pty.close_master();
  CanFrame frame;
  EXPECT_EQ(transport.receive(frame), ReceiveResult::Disconnected);
}
```

- [ ] **Step 2: Build and verify RED**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo`

Expected: FAIL because `SerialTransport` is undefined.

- [ ] **Step 3: Implement nonblocking tty lifecycle**

Open with `O_RDWR | O_NOCTTY | O_NONBLOCK`, configure raw mode, validate baud rate, and keep parser state per transport. In `SerialSlcan` mode, write and drain exactly `C\rS8\rY5\rO\r`; in `UsbSerial` mode, write no initialization bytes. Map `POLLHUP`, `POLLERR`, zero-byte read and fatal `read/write` errors to `Disconnected`; map `EAGAIN/EWOULDBLOCK` to `NoData`. `close()` must be idempotent and `send()` must handle partial writes.

- [ ] **Step 4: Run tty and frame tests and verify GREEN**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo && source install/setup.bash && colcon test --packages-select humanoid_gazebo --ctest-args -R 'test_serial_transport|test_frame_record' && colcon test-result --verbose`

Expected: pseudo-terminal tests pass without physical USB hardware.

- [ ] **Step 5: Commit transport layer**

```bash
git add src/humanoid_gazebo
git commit -m "feat: add USB serial transports"
```

---

### Task 4: 建立 30 轴电机配置、传动换算和 plant

**Files:**
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/config.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/transmission.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/plant_model.hpp`
- Create: `src/humanoid_gazebo/src/motor_sim/config.cpp`
- Create: `src/humanoid_gazebo/src/motor_sim/transmission.cpp`
- Create: `src/humanoid_gazebo/src/motor_sim/plant_model.cpp`
- Create: `src/humanoid_gazebo/config/motors.yaml`
- Create: `src/humanoid_gazebo/test/test_motor_config.cpp`
- Create: `src/humanoid_gazebo/test/test_transmission.cpp`
- Create: `src/humanoid_gazebo/test/test_plant_model.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`, `src/humanoid_gazebo/package.xml`

**Interfaces:**
- Consumes: motor types `rp` and `motor_2530` from Task 2.
- Produces: `SystemConfig load_system_config(const std::string&)`; `Transmission::motor_to_joint_position(double)`; `joint_to_motor_position(double)`; `motor_torque_to_joint_effort(double)`; `MotorPlantModel::step(const MotorCommand&, const JointSample&, double dt_s) -> PlantOutput`.

- [ ] **Step 1: Write failing schema and topology tests**

```cpp
TEST(MotorConfig, LoadsExactThirtyAxisContract) {
  const auto config = load_system_config(kMotorsYaml);
  ASSERT_EQ(config.buses.size(), 3U);
  ASSERT_EQ(config.motors.size(), 30U);
  EXPECT_EQ(count_type(config, "rp"), 20U);
  EXPECT_EQ(count_type(config, "motor_2530"), 10U);
  EXPECT_EQ(config.motors.front().joint, "left_scapula_protraction_joint");
  EXPECT_EQ(config.motors.back().joint, "right_ankle_roll_joint");
}

TEST(MotorConfig, RejectsCommandLimitOutsideHardLimit) {
  EXPECT_THROW(load_system_config(kBadLimitsYaml), std::runtime_error);
}
```

Required order is left arm 9, right arm 9, left leg 6, right leg 6. Validate unique joint names and unique `(bus, protocol, motor_id)` tuples; buses must be exactly `left_arm`, `right_arm`, `legs`.

- [ ] **Step 2: Write failing transmission and plant tests**

```cpp
TEST(Transmission, VaxorRoundTripsMotorAndJointPosition) {
  Transmission tx({36.0, 0.80, -1.0, 0.1});
  const double motor = tx.joint_to_motor_position(0.4);
  EXPECT_NEAR(tx.motor_to_joint_position(motor), 0.4, 1e-12);
  EXPECT_NEAR(tx.motor_torque_to_joint_effort(0.1), -2.88, 1e-12);
}

TEST(PlantModel, SaturatesEffortAndRejectsNonPositiveDt) {
  MotorPlantModel plant({30.0, 1.0, 8.0});
  EXPECT_NEAR(plant.step(position_command(1.0), {0.0, 0.0}, 0.01).effort_nm,
              8.0, 1e-12);
  EXPECT_THROW(plant.step(position_command(1.0), {0.0, 0.0}, 0.0),
               std::invalid_argument);
}
```

- [ ] **Step 3: Build and verify RED**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo`

Expected: FAIL because configuration, transmission and plant APIs do not exist.

- [ ] **Step 4: Implement strict YAML parsing and nominal actuator data**

Use `yaml-cpp`; reject missing/non-finite values, `gear_ratio <= 0`, efficiency outside `(0, 1]`, direction outside `{-1, 1}`, non-positive velocity/effort limits, and any ordering other than `hard_lower < command_lower < command_upper < hard_upper`. Encode these nominal values in `motors.yaml` with comments marking them simulation-only:

```yaml
timing: {control_hz: 100, rp_feedback_hz: 100, vaxor_feedback_hz: 50, command_timeout_ms: 150}
vaxor_defaults: {efficiency: 0.80, torque_constant_nm_per_a: 0.012}
transmission_ratio: {forearm_and_wrist: 36.0, ankle: 80.0}
```

RP uses joint-side ratio 1.0. Populate the exact IDs and hard limits from the approved spec; command limits shrink each hard-limit endpoint by 5 degrees.

- [ ] **Step 5: Implement transmission and plant equations**

Use:

```text
motor_position = direction * gear_ratio * (joint_position - zero_offset)
joint_position = zero_offset + direction * motor_position / gear_ratio
joint_effort = direction * motor_torque * gear_ratio * efficiency
position plant effort = clamp(kp * position_error - kd * joint_velocity,
                              -effort_limit, effort_limit)
```

Keep every motor's plant state independent; do not infer joint count from motor type.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo && source install/setup.bash && colcon test --packages-select humanoid_gazebo --ctest-args -R 'test_motor_config|test_transmission|test_plant_model' && colcon test-result --verbose`

Expected: config, transmission and plant tests pass.

- [ ] **Step 7: Commit motor simulation core**

```bash
git add src/humanoid_gazebo
git commit -m "feat: add thirty-axis motor simulation core"
```

---

### Task 5: 构建 170 cm、45 kg、30 轴悬空模型

**Files:**
- Create: `src/humanoid_gazebo/config/mechanics.yaml`
- Replace: `src/humanoid_gazebo/urdf/upper_body.urdf.xacro` → `src/humanoid_gazebo/urdf/humanoid.urdf.xacro`
- Replace: `src/humanoid_gazebo/worlds/upper_body_world.sdf` → `src/humanoid_gazebo/worlds/humanoid_world.sdf`
- Modify: `src/humanoid_gazebo/config/controllers.yaml`
- Replace: `src/humanoid_gazebo/test/test_config_topology.py`
- Create: `src/humanoid_gazebo/test/test_humanoid_model.py`

**Interfaces:**
- Consumes: 30-joint order and limits from `config/motors.yaml`.
- Produces: xacro robot `humanoid`; fixed world-to-torso support; `/effort_controller/commands` with exactly 30 joints; mechanical YAML as geometry/inertia source.

- [ ] **Step 1: Write failing 30-axis cross-source contract tests**

```python
def test_motor_urdf_and_controller_joint_order_matches() -> None:
    motors = load_motor_joints(MOTORS_YAML)
    urdf = expand_xacro(HUMANOID_XACRO)
    actuated = actuated_joint_names(urdf)
    controlled = controller_joint_names(CONTROLLERS_YAML)
    assert len(motors) == 30
    assert actuated == motors
    assert controlled == motors

def test_mass_height_and_support_contract() -> None:
    urdf = expand_xacro(HUMANOID_XACRO)
    assert total_dynamic_mass(urdf) == pytest.approx(45.0, abs=1e-6)
    assert model_height(urdf) == pytest.approx(1.70, abs=0.01)
    assert foot_bottom_height(urdf) == pytest.approx(0.10, abs=0.005)
    assert fixed_support_joint(urdf).type == "fixed"
```

- [ ] **Step 2: Run model tests and verify RED**

Run: `python3 -m pytest src/humanoid_gazebo/test/test_config_topology.py src/humanoid_gazebo/test/test_humanoid_model.py -v`

Expected: FAIL because the current model has 14 joints and the new files do not exist.

- [ ] **Step 3: Implement mechanics YAML and xacro generation**

Set dynamic link masses to pelvis 6 kg, abdomen 4 kg, chest 8 kg, head 3 kg, each arm 4.8 kg and each leg 7.2 kg. Use REP-103 axes, neutral arms-down/legs-straight pose, rigid head and hands, industrial primitive geometry, positive diagonal inertias, joint damping/friction, and mirrored left/right origins. Generate five shoulder-complex joints, two elbow/forearm joints and two wrist joints per arm; generate hip yaw/roll/pitch, knee pitch and ankle pitch/roll per leg.

The world contains a fixed support whose mass is excluded from 45 kg. Model dimensions must put the foot bottoms at `z=0.10 m`, hip center near `z=1.04 m` and head top at `z=1.80 m`.

- [ ] **Step 4: Add internal `gz_ros2_control` effort controller**

Declare effort command and position/velocity state interfaces for all 30 actuated joints. Configure:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    effort_controller:
      type: forward_command_controller/ForwardCommandController
effort_controller:
  ros__parameters:
    interface_name: effort
```

- [ ] **Step 5: Run model and topology tests and verify GREEN**

Run: `python3 -m pytest src/humanoid_gazebo/test/test_config_topology.py src/humanoid_gazebo/test/test_humanoid_model.py -v`

Expected: all topology, mass, height, limit, mirror and controller-order assertions pass.

- [ ] **Step 6: Commit the humanoid model**

```bash
git add src/humanoid_gazebo
git commit -m "feat: add suspended thirty-axis humanoid model"
```

---

### Task 6: 实现三总线 ROS 节点与锁存安全状态机

**Files:**
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/safety_monitor.hpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/frame_router.hpp`
- Create: `src/humanoid_gazebo/src/motor_sim/safety_monitor.cpp`
- Create: `src/humanoid_gazebo/src/motor_sim/frame_router.cpp`
- Create: `src/humanoid_gazebo/include/humanoid_gazebo/motor_sim/motor_sim_node.hpp`
- Create: `src/humanoid_gazebo/src/motor_sim_node.cpp`
- Create: `src/humanoid_gazebo/test/test_safety_monitor.cpp`
- Create: `src/humanoid_gazebo/test/test_frame_router.cpp`
- Modify: `src/humanoid_gazebo/CMakeLists.txt`, `src/humanoid_gazebo/package.xml`

**Interfaces:**
- Consumes: `FrameTransport`, `SystemConfig`, `MotorProtocol`, `Transmission`, `MotorPlantModel`.
- Produces: executable `humanoid_motor_sim_node`; injectable `MotorSimNode(NodeOptions, SystemConfig, TransportMap)` for integration tests; `SafetyMonitor::observe_command(bus, id, payload, steady_time)`; `SafetyMonitor::tick(steady_time)`; `FrameRouter::handle(bus, frame, steady_time) -> RouterResult`; ROS topics `/joint_states`, `/effort_controller/commands`, `/humanoid_gazebo/diagnostics`.

- [ ] **Step 1: Write failing safety-state tests**

```cpp
TEST(SafetyMonitor, TimeoutZerosEveryAxisAndLatches) {
  SafetyMonitor safety(30, 150ms, 2ms);
  safety.observe_valid_command("left_arm", 1, bytes("01"), at_ms(0));
  EXPECT_TRUE(safety.tick(at_ms(151)).latched);
  EXPECT_EQ(safety.tick(at_ms(151)).efforts, std::vector<double>(30, 0.0));
}

TEST(SafetyMonitor, DifferentDuplicatePayloadWithinWindowLatches) {
  SafetyMonitor safety(30, 150ms, 2ms);
  safety.observe_valid_command("legs", 3, bytes("01"), at_us(0));
  safety.observe_valid_command("legs", 3, bytes("02"), at_us(1500));
  EXPECT_EQ(safety.fault(), Fault::DuplicateController);
}

TEST(SafetyMonitor, ReconnectDoesNotClearFault) {
  SafetyMonitor safety(30, 150ms, 2ms);
  safety.observe_disconnect("right_arm");
  safety.observe_reconnect("right_arm");
  EXPECT_EQ(safety.fault(), Fault::TransportDisconnected);
}
```

- [ ] **Step 2: Write failing router tests**

```cpp
TEST(FrameRouter, RejectsFrameOnWrongBus) {
  FrameRouter router(load_system_config(kMotorsYaml));
  EXPECT_EQ(router.handle("legs", rp_position_frame(1), at_ms(0)).status,
            RouterStatus::WrongBus);
}

TEST(FrameRouter, OutOfCommandRangeLatchesWholeRobot) {
  FrameRouter router(load_system_config(kMotorsYaml));
  EXPECT_EQ(router.handle("left_arm", command_beyond_limit(), at_ms(0)).status,
            RouterStatus::SafetyFault);
  EXPECT_TRUE(router.safety_state().latched);
}
```

- [ ] **Step 3: Build and verify RED**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo`

Expected: FAIL because safety and routing APIs are undefined.

- [ ] **Step 4: Implement deterministic routing and safety**

Index motors by `(bus, protocol, motor_id)`. Decode only frames assigned to the receiving bus, reject non-finite and out-of-range commands before modifying any plant, and publish one 30-element effort vector per 10 ms control tick. Use `std::chrono::steady_clock` for 150 ms timeout and 2 ms duplicate detection. Any limit, timeout, disconnect or conflicting duplicate fault latches all efforts to zero; reconnect never clears the latch.

- [ ] **Step 5: Implement the ROS node**

Declare parameters `transport_mode`, `left_arm_port`, `right_arm_port`, `legs_port`, `baud_rate`, `config_file`, `allow_external_canfd`. Validate nonempty, distinct `/dev/serial/by-id/` paths before constructing three transports. Reject mixed modes by the single `transport_mode` enum; reject `serial_slcan` unless `allow_external_canfd` is true. Subscribe to `/joint_states`, publish 30 efforts, encode RP feedback at 100 Hz and Vaxor feedback at 50 Hz, and fail the process after publishing zero effort when any transport returns `Disconnected`.

- [ ] **Step 6: Run core and node contract tests and verify GREEN**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo && source install/setup.bash && colcon test --packages-select humanoid_gazebo --ctest-args -R 'test_safety_monitor|test_frame_router' && colcon test-result --verbose`

Expected: safety and router tests pass; node links without old packages.

- [ ] **Step 7: Commit node and safety behavior**

```bash
git add src/humanoid_gazebo
git commit -m "feat: route external motor frames into Gazebo"
```

---

### Task 7: 提供三个 launch 入口和 30 轴动作验收工具

**Files:**
- Replace: `src/humanoid_gazebo/launch/gazebo.launch.py`
- Replace: `src/humanoid_gazebo/launch/model_sim.launch.py` → `src/humanoid_gazebo/launch/usb_sim.launch.py`
- Replace: `src/humanoid_gazebo/launch/driver_loopback.launch.py` → `src/humanoid_gazebo/launch/canfd_sim.launch.py`
- Modify: `src/humanoid_gazebo/humanoid_gazebo/motion_library.py`
- Modify: `src/humanoid_gazebo/humanoid_gazebo/ros_action_player.py`
- Create: `src/humanoid_gazebo/test/test_launch_contract.py`
- Modify: `src/humanoid_gazebo/test/test_motion_library.py`
- Modify: `src/humanoid_gazebo/test/test_ros_action_player.py`

**Interfaces:**
- Consumes: `humanoid.urdf.xacro`, `humanoid_motor_sim_node`, 30-joint ordered config.
- Produces: `gazebo.launch.py`, `usb_sim.launch.py`, `canfd_sim.launch.py`; action names `joint_scan`, `shoulder_rhythm`, `arm_combo`, `suspended_legs`, `mirror_check`.

- [ ] **Step 1: Write failing launch safety tests**

```python
def test_usb_launch_requires_three_stable_unique_ports() -> None:
    launch = load_launch("usb_sim.launch.py")
    assert launch.argument("transport_mode").default == "usb_serial"
    assert launch.required_ports == {"left_arm_port", "right_arm_port", "legs_port"}

def test_canfd_launch_is_opt_in() -> None:
    launch = load_launch("canfd_sim.launch.py")
    assert launch.argument("transport_mode").default == "serial_slcan"
    assert launch.argument("allow_external_canfd").default == "false"
```

- [ ] **Step 2: Write failing shoulder-rhythm and action-order tests**

```python
def test_shoulder_rhythm_is_off_by_default() -> None:
    assert apply_shoulder_rhythm(60.0, enabled=False) == (0.0, 0.0)
    assert apply_shoulder_rhythm(60.0, enabled=True) == pytest.approx((7.5, 3.6))

def test_every_action_has_exact_thirty_joint_values() -> None:
    for action in ACTIONS.values():
        assert tuple(action.positions) == JOINT_ORDER
        assert len(action.keyframes[0]) == 30
```

- [ ] **Step 3: Run launch/action tests and verify RED**

Run: `python3 -m pytest src/humanoid_gazebo/test/test_launch_contract.py src/humanoid_gazebo/test/test_motion_library.py src/humanoid_gazebo/test/test_ros_action_player.py -v`

Expected: FAIL because final launch files and 30-axis actions do not exist.

- [ ] **Step 4: Implement launch composition and validation**

`gazebo.launch.py` starts the world, xacro robot, `robot_state_publisher`, bridge, controller manager and both controller spawners. `usb_sim.launch.py` includes Gazebo and starts the node with `usb_serial`; `canfd_sim.launch.py` does the same with `serial_slcan`. Launch-time `OpaqueFunction` validates all three paths start with `/dev/serial/by-id/`, are distinct, and CAN-FD opt-in is true before adding the simulator node.

- [ ] **Step 5: Implement the 30-axis actions**

Keep the 30 joints independent. Shoulder rhythm defaults off; when enabled and shoulder raise exceeds 30 degrees, compute `scapula_elevation = 0.25 * (raise - 30)` and for forward raise compute `scapula_protraction = 0.12 * (shoulder_pitch - 30)`, then clamp both to command limits. Add actions for joint-by-joint scanning, coordinated raise, arm combination, suspended legs and left/right mirror checking.

The action player is a model-visualization tool, not a second serial controller: it subscribes to `/joint_states`, applies bounded joint-space PD to each target pose, and publishes the resulting 30-element effort vector directly to `/effort_controller/commands`. It is not started by `usb_sim.launch.py` or `canfd_sim.launch.py`, so it cannot coexist accidentally with the external motor simulator node.

- [ ] **Step 6: Run launch/action tests and verify GREEN**

Run: `python3 -m pytest src/humanoid_gazebo/test/test_launch_contract.py src/humanoid_gazebo/test/test_motion_library.py src/humanoid_gazebo/test/test_ros_action_player.py -v`

Expected: all static launch and action tests pass.

- [ ] **Step 7: Commit launch and actions**

```bash
git add src/humanoid_gazebo
git commit -m "feat: add USB simulation launch workflows"
```

---

### Task 8: 删除旧包并重写仓库文档

**Files:**
- Delete: `src/motor_drive/`
- Delete: `src/motor_sim/`
- Delete: `src/motor_hardware_interface/`
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `CONTEXT.md`
- Modify: `docs/adr/0001-driver-architecture.md`
- Modify: `docs/adr/0002-simulation-config-independence.md`
- Modify: `docs/adr/0003-driver-transport-abstraction.md`
- Modify: `docs/adr/0004-simulator-fail-fast-lifecycle.md`
- Create: `src/humanoid_gazebo/docs/usb-wiring.md`
- Modify: `src/humanoid_gazebo/README.md`
- Modify: `src/humanoid_gazebo/test/test_package_identity.py`

**Interfaces:**
- Consumes: all migrated code and final launch commands from Tasks 1–7.
- Produces: a repository whose source tree and documentation describe only `humanoid_gazebo` and the two supported external serial modes.

- [ ] **Step 1: Strengthen the package and forbidden-reference tests**

```python
def test_colcon_discovers_only_humanoid_gazebo() -> None:
    result = subprocess.run(
        ["colcon", "list", "--names-only"], cwd=ROOT,
        check=True, text=True, capture_output=True,
    )
    assert result.stdout.splitlines() == ["humanoid_gazebo"]

def test_runtime_sources_have_no_removed_transport_or_package_names() -> None:
    forbidden = ("SocketCAN", "vcan", "motor_hardware_interface",
                 "motor_drive", "humanoid_canfd_gazebo",
                 "find_package(motor_sim", 'package="motor_sim"')
    text = read_runtime_and_user_docs()
    assert not any(token in text for token in forbidden)
```

- [ ] **Step 2: Run identity tests and verify RED**

Run: `python3 -m pytest src/humanoid_gazebo/test/test_package_identity.py -v`

Expected: FAIL because the three old package directories still exist.

- [ ] **Step 3: Remove migrated packages and stale generated package artifacts**

Delete only the tracked source directories `src/motor_drive`, `src/motor_sim`, `src/motor_hardware_interface`. Do not touch `ros2_ws/`. Move stale generated directories for removed packages from `build/`, `install/` and `log/` into one `mktemp -d` recovery directory before the clean build, and report that recovery path.

- [ ] **Step 4: Rewrite user and agent documentation**

Document only:

```text
ros2 launch humanoid_gazebo gazebo.launch.py
ros2 launch humanoid_gazebo usb_sim.launch.py \
  left_arm_port:=/dev/serial/by-id/... \
  right_arm_port:=/dev/serial/by-id/... \
  legs_port:=/dev/serial/by-id/...
ros2 launch humanoid_gazebo canfd_sim.launch.py \
  allow_external_canfd:=true \
  left_arm_port:=/dev/serial/by-id/... \
  right_arm_port:=/dev/serial/by-id/... \
  legs_port:=/dev/serial/by-id/...
```

The wiring guide must distinguish USB CDC/UART from USB-CANFD-CANFD-USB and STM32 USB-CANFD, show CANH/CANL/GND and termination requirements, warn about duplicate motor IDs, and state that no physical CAN behavior is modeled in `usb_serial` mode. Update ADRs as superseded historical records rather than silently presenting removed architecture as current.

- [ ] **Step 5: Build from the one-package source tree and verify GREEN**

Run: `source /opt/ros/humble/setup.bash && colcon list --names-only`

Expected: exactly `humanoid_gazebo`.

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && colcon test && colcon test-result --verbose`

Expected: the single package builds and every test passes.

Run: `rg -n 'motor_drive|motor_hardware_interface|humanoid_canfd_gazebo|SocketCAN|vcan|find_package\(motor_sim|package="motor_sim"' README.md AGENTS.md CONTEXT.md src/humanoid_gazebo --glob '!docs/protocol/**'`

Expected: no output except explicit historical migration notes approved in ADRs.

- [ ] **Step 6: Commit the single-package cleanup**

```bash
git add README.md AGENTS.md CONTEXT.md docs/adr src
git commit -m "refactor: keep only humanoid Gazebo simulation"
```

---

### Task 9: 全量验证、无头运行验收与交付证据

**Files:**
- Create: `src/humanoid_gazebo/test/test_pty_integration.cpp`
- Create: `docs/verification/2026-08-25-humanoid-gazebo.md`
- Create: `docs/verification/2026-08-25-humanoid-gazebo.png` when a renderer is available
- Modify: `src/humanoid_gazebo/README.md`

**Interfaces:**
- Consumes: final single-package build, USB launch, node diagnostics and all automated tests.
- Produces: reproducible PTY integration test using dependency-injected transports, build/test transcript summary, measured headless runtime data, and Gazebo screenshot or an explicit environment limitation.

- [ ] **Step 1: Write the end-to-end PTY integration test**

```cpp
TEST(PtyIntegration, ThreeBusRoundTripAndDisconnect) {
  ThreeBusPtyHarness harness;
  auto node = harness.make_node(load_system_config(kMotorsYaml));
  harness.send("left_arm", rp_position_command(1, 0.1));
  harness.send("right_arm", vaxor_position_command(1, -0.1));
  harness.send("legs", rp_position_command(1, 0.2));
  ASSERT_TRUE(harness.spin_until_feedback_from_all_buses(*node, 2s));
  harness.disconnect("legs");
  EXPECT_TRUE(harness.spin_until_zero_effort(*node, 200ms));
  EXPECT_TRUE(node->shutdown_requested());
}
```

- [ ] **Step 2: Run the PTY test and verify RED before completing its harness**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select humanoid_gazebo`

Expected: FAIL until the C++ harness can create three PTYs, inject three `SerialTransport` instances, publish joint states, observe feedback and close one PTY.

- [ ] **Step 3: Complete the deterministic PTY harness and verify GREEN**

Implement bounded executor spins, PTY cleanup and captured diagnostics. Construct `MotorSimNode` through the dependency-injection constructor from Task 6, so production `/dev/serial/by-id/` validation remains unchanged and no test-only runtime parameter exists.

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && colcon test --packages-select humanoid_gazebo --ctest-args -R test_pty_integration && colcon test-result --verbose`

Expected: PASS, including feedback on all three buses and fail-fast zero effort on disconnect.

- [ ] **Step 4: Run the complete automated verification**

Run: `source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && colcon test && colcon test-result --verbose`

Expected: build succeeds and failures are 0.

Run: `git diff --check && git status --short`

Expected: no whitespace errors; only intended verification report/image changes plus the user-owned untracked `ros2_ws/` are present.

- [ ] **Step 5: Run headless Gazebo acceptance and record actual measurements**

Run the USB PTY harness for 30 minutes with `gui:=false`, execute all five action families, and record actual RTF, p99 control-period jitter, feedback timeout count and cross-bus routing error count. Acceptance is RTF at least 0.95, p99 jitter within ±2 ms, zero feedback timeouts and zero cross-bus errors. Exercise illegal command, 150 ms timeout and cable-disconnect cases separately.

If the environment can render Gazebo, capture the neutral 30-axis suspended robot as `docs/verification/2026-08-25-humanoid-gazebo.png`. If Gazebo, rendering or USB-CANFD hardware is unavailable, write the exact unexecuted check and reason in the verification report; do not mark it passed.

- [ ] **Step 6: Commit verification evidence**

```bash
git add src/humanoid_gazebo/test/test_pty_integration.cpp \
  src/humanoid_gazebo/README.md docs/verification
git commit -m "test: verify humanoid Gazebo simulation"
```

---

## Completion Gate

- `git branch --show-current` prints `dev/smi`.
- `colcon list --names-only` prints only `humanoid_gazebo`.
- Full build and test suite reports zero failures.
- Source and current user documentation contain no old package, vcan, SocketCAN,真机 bring-up or local driver-loopback path.
- The model has exactly 30 actuated joints, total dynamic mass 45 kg, body height 1.70 m and foot-bottom height 0.10 m.
- USB PTY integration proves three-bus routing, feedback, timeout, limit rejection, duplicate-source latch and disconnect fail-fast.
- USB-CANFD physical acceptance is either supported by captured evidence or explicitly marked unexecuted.
