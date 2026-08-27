# UR5e Realistic Brake and Self-Collision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让统一 `gazebo.launch.py` 启动的 UR5e 在无有效控制、故障和超时时可靠抱闸，在全轴新鲜命令握手后释放抱闸，并阻止非相邻 link 相互穿透。

**Architecture:** `motor_sim` 用与机器人无关的安全生命周期生成严格限定的 `drive_state`，并用 `control_gate` 阻止 controller 切换期间输出非零 effort。`robot_arm_gazebo` 内新增 `brake_coordinator`，通过 Controller Manager 的 STRICT switch 服务释放或重新占用 effort command interface；模型级 `self_collide` 交给 SDFormat/Gazebo 物理引擎处理。

**Tech Stack:** ROS 2 Humble、C++17、rclcpp、controller_manager_msgs、gz_ros2_control、Gazebo Fortress、SDFormat、ament_cmake、GoogleTest、pytest、SocketCAN/vcan。

**Spec:** `docs/superpowers/specs/2026-08-27-realistic-brake-and-self-collision-design.md`

## Global Constraints

- 仓库仍只包含 `motor_sim` 与 `robot_arm_gazebo` 两个 ament package；不得新增真机 bring-up、外部硬件插件、第三个 ROS package 或 MoveIt。
- 保持现有 CAN identifier、payload、六轴顺序和 `config/motors.yaml` 格式兼容。
- 保持显式 enable、150 ms 命令超时、断线停机、越限/错总线/重复控制器故障语义。
- `fault_latching_enabled=false` 仅放宽恢复方式；故障仍必须清零、抱闸，并等待全轴 disable 后的新 enable 周期。
- 动作播放器不得由 launch 启动，也不得与外部电机仿真节点并行控制。
- 自动测试不得要求 Gazebo GUI、真实 tty 或真实 CAN；运行验收使用 `vcan0`。
- 不关闭重力，不用无条件高阻尼掩盖保持失败；仿真参数不得描述为厂家规格或用于真机安全结论。
- `src/robot_arm_gazebo/launch/gazebo.launch.py` 中用户当前未提交的连接默认值逐行保留，不纳入本功能提交。

## File Map

- `src/motor_sim/include/motor_sim/core/drive_lifecycle.hpp`、`src/motor_sim/src/core/drive_lifecycle.cpp`：机器人无关的全轴 enable、首帧、故障、rearm、gate 状态机。
- `src/motor_sim/include/motor_sim/core/frame_router.hpp`、`src/motor_sim/src/core/frame_router.cpp`：把协议帧事件送入生命周期，并以 gate 约束 effort 输出。
- `src/motor_sim/include/motor_sim/core/safety_monitor.hpp`、`src/motor_sim/src/core/safety_monitor.cpp`：故障检测、锁存/reset 条件与非锁存 disable-rearm 条件。
- `src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`、`src/motor_sim/src/ros/motor_sim_node.cpp`：`drive_state`、`control_gate`、`reset_fault`、diagnostics ROS 接口和零 effort 确认顺序。
- `src/robot_arm_gazebo/include/robot_arm_gazebo/brake_coordinator.hpp`、`src/robot_arm_gazebo/src/brake_coordinator.cpp`、`src/robot_arm_gazebo/src/brake_coordinator_main.cpp`：STRICT controller 切换协调器及其可单测状态机。
- `src/robot_arm_gazebo/urdf/ur5e.urdf.xacro`：可配置模型级 self-collision。
- `src/robot_arm_gazebo/launch/gazebo.launch.py`：统一参数、inactive controller 启动编排、节点 fail-fast。
- `README.md`、`CONTEXT.md`、`docs/verification/2026-08-27-realistic-brake-and-self-collision.md`：用户接口、架构语义和实机式仿真验收证据。

---

### Task 1: Verify `hold_joints` Feasibility Under Gravity

**Files:**
- Create after a passing run: `docs/verification/2026-08-27-hold-joints-feasibility.md`
- Do not modify source code in this task.

**Interfaces:**
- Consumes: `/robot_arm/controller_manager/switch_controller`, `/robot_arm/joint_states`, existing `<hold_joints>true</hold_joints>`.
- Produces: a measured go/no-go decision: every joint drifts at most `0.01 rad` over `10 s`, without visible sustained oscillation.

- [ ] **Step 1: Record and protect the dirty launch file**

Run:

```bash
git diff -- src/robot_arm_gazebo/launch/gazebo.launch.py > /tmp/robot_arm_launch_user.patch
git status --short
```

Expected: the saved patch contains the user's `world_file`、`socket_can`、`vcan0`、`fault_latching_enabled=false` and `motors.yaml` defaults; no source file is changed by this step.

- [ ] **Step 2: Build and create the test CAN interface**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_gazebo
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
source install/setup.bash
```

Expected: build succeeds and `ip -details link show vcan0` reports `UP`.

- [ ] **Step 3: Launch the existing backend and deactivate effort control**

Terminal A:

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py transport_mode:=socket_can arm_port:=vcan0 fault_latching_enabled:=false
```

Terminal B:

```bash
ros2 service call /robot_arm/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: [effort_controller], strictness: 2, activate_asap: true, timeout: {sec: 2, nanosec: 0}}"
ros2 control list_controllers -c /robot_arm/controller_manager
```

Expected: `effort_controller` is `inactive` and `joint_state_broadcaster` remains `active`.

- [ ] **Step 4: Measure ten-second joint drift**

Run this after the controller is inactive:

```bash
timeout 12s ros2 topic echo /robot_arm/joint_states --field position > /tmp/ur5e_hold_positions.txt
```

Extract the first and last six-value samples and calculate `abs(last[i] - first[i])` for all axes. Expected: each value is `<= 0.01`; Gazebo shows no sustained oscillation or model flight.

- [ ] **Step 5: Apply the hard gate**

If any axis exceeds `0.01 rad` or oscillates, stop this plan without changing implementation files and return to the design spec to select a dedicated parameterized hold controller. If it passes, create the verification file with the exact launch command, first/last vectors, six deltas, Gazebo version, and controller states.

- [ ] **Step 6: Commit only the feasibility evidence**

```bash
git add docs/verification/2026-08-27-hold-joints-feasibility.md
git commit -m "test: verify gazebo hold joints feasibility"
```

### Task 2: Add a Robot-Agnostic Drive Lifecycle

**Files:**
- Create: `src/motor_sim/include/motor_sim/core/drive_lifecycle.hpp`
- Create: `src/motor_sim/src/core/drive_lifecycle.cpp`
- Create: `src/motor_sim/test/test_drive_lifecycle.cpp`
- Modify: `src/motor_sim/CMakeLists.txt`

**Interfaces:**
- Consumes: an arbitrary non-zero axis count and safety events; no UR5e names, IDs, buses, or fixed axis count.
- Produces: `enum class DriveState { BrakeEngaged, AwaitingFirstCommand, ReadyForControl, MotorControlActive, FaultLatched };`, `std::string_view drive_state_name(DriveState)`, and class `DriveLifecycle` with `set_enabled(index, enabled)`, `observe_fresh_command(index)`, `set_control_gate(bool)`, `enter_fault(bool latched)`, `reset_to_brake()`, `state()`, `all_disabled()`.

- [ ] **Step 1: Write lifecycle tests first**

Create GoogleTests that assert these exact transitions for a two-axis instance:

```cpp
TEST(DriveLifecycle, RequiresEveryAxisFreshCommandInCurrentEnableCycle) {
  DriveLifecycle lifecycle(2U);
  EXPECT_EQ(lifecycle.state(), DriveState::BrakeEngaged);
  lifecycle.set_enabled(0U, true);
  lifecycle.observe_fresh_command(0U);
  lifecycle.set_enabled(1U, true);
  EXPECT_EQ(lifecycle.state(), DriveState::AwaitingFirstCommand);
  lifecycle.observe_fresh_command(1U);
  EXPECT_EQ(lifecycle.state(), DriveState::ReadyForControl);
  lifecycle.set_control_gate(true);
  EXPECT_EQ(lifecycle.state(), DriveState::MotorControlActive);
}

TEST(DriveLifecycle, DisableInvalidatesEveryOldCommandAndEngagesBrake) {
  DriveLifecycle lifecycle(2U);
  for (std::size_t index = 0; index < 2; ++index) {
    lifecycle.set_enabled(index, true);
    lifecycle.observe_fresh_command(index);
  }
  lifecycle.set_control_gate(true);
  lifecycle.set_enabled(0U, false);
  EXPECT_EQ(lifecycle.state(), DriveState::BrakeEngaged);
  lifecycle.set_enabled(0U, true);
  EXPECT_EQ(lifecycle.state(), DriveState::AwaitingFirstCommand);
}
```

Also test: gate cannot make an unready arm active; disabling any axis clears gate; non-latched fault enters `BrakeEngaged`; latched fault enters `FaultLatched`; reset returns to `BrakeEngaged`; invalid indices throw; enum strings exactly match the five values in the spec.

- [ ] **Step 2: Run the new test and confirm red**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select motor_sim --ctest-args -R test_drive_lifecycle --output-on-failure
```

Expected: compilation fails because `drive_lifecycle.hpp` does not exist.

- [ ] **Step 3: Implement the minimal lifecycle**

Store `enabled_` and `fresh_command_` vectors. On every `false -> true` enable transition clear that axis's fresh bit; on any disable clear all fresh bits and `control_gate_`; allow `MotorControlActive` only when all axes are enabled, all fresh bits are true, no fault state is present, and gate is true. `enter_fault()` clears gate and all fresh bits.

- [ ] **Step 4: Register and run the focused test**

Add `drive_lifecycle.cpp` to `motor_sim_core`, add `ament_add_gtest(test_drive_lifecycle test/test_drive_lifecycle.cpp)`, link `motor_sim::motor_sim_core`, then rerun the command from Step 2.

Expected: all lifecycle tests pass.

- [ ] **Step 5: Run the existing core tests**

```bash
colcon test --packages-select motor_sim --ctest-args -R 'test_(safety_monitor|plant_model|motor_protocol)' --output-on-failure
```

Expected: all selected tests pass unchanged.

- [ ] **Step 6: Commit**

```bash
git add src/motor_sim/include/motor_sim/core/drive_lifecycle.hpp src/motor_sim/src/core/drive_lifecycle.cpp src/motor_sim/test/test_drive_lifecycle.cpp src/motor_sim/CMakeLists.txt
git commit -m "feat: add generic drive lifecycle"
```

### Task 3: Make Faults Brake and Support Explicit Rearm

**Files:**
- Modify: `src/motor_sim/include/motor_sim/core/safety_monitor.hpp`
- Modify: `src/motor_sim/src/core/safety_monitor.cpp`
- Modify: `src/motor_sim/test/test_safety_monitor.cpp`

**Interfaces:**
- Consumes: existing timeout, limit, duplicate-controller, wrong-bus and disconnect detections.
- Produces: `enum class ResetFaultResult { Cleared, NotLatched, AxesStillEnabled, TransportDisconnected, ConditionActive };`, `reset_fault()`, `rearm_required()`, `ready_for_new_enable_cycle()`, and snapshots that report detected faults even when latching is disabled.

- [ ] **Step 1: Replace permissive debug-mode expectations with safe recovery tests**

Add tests asserting:

```cpp
TEST(SafetyMonitorDebugMode, TimeoutRequiresAllAxisDisableBeforeRearm) {
  SafetyMonitor safety(2U, 150ms, 2ms, false);
  safety.set_enabled(0U, true, at_ms(0));
  safety.set_enabled(1U, true, at_ms(0));
  safety.observe_valid_command(0U, "arm", 1U, {1U}, at_ms(0));
  safety.observe_valid_command(1U, "arm", 2U, {2U}, at_ms(0));
  EXPECT_TRUE(safety.tick(at_ms(151)).rearm_required);
  safety.set_enabled(0U, false, at_ms(152));
  EXPECT_FALSE(safety.ready_for_new_enable_cycle());
  safety.set_enabled(1U, false, at_ms(152));
  EXPECT_TRUE(safety.ready_for_new_enable_cycle());
}
```

Add locked-mode tests for reset rejection while any axis is enabled, rejection after disconnect, success after all-axis disable for a recoverable timeout/limit fault, and reset not reusing old command timestamps. Update the former “does not latch or zero” tests: non-latching mode must report the detected fault and `rearm_required=true` instead of continuing output.

- [ ] **Step 2: Run the safety test and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select motor_sim --ctest-args -R test_safety_monitor --output-on-failure
```

Expected: compilation fails on the new reset/rearm API.

- [ ] **Step 3: Implement fault and reset bookkeeping**

Keep first-fault-wins for locked mode. Track `detected_fault_`, `rearm_required_`, `transport_connected_`, and the all-axis-disable observation. Every fault sets `rearm_required_`; locked mode additionally sets `fault_`. A complete all-axis disable clears a non-latched recoverable detected fault and begins a new enable epoch. `reset_fault()` clears only a recoverable locked fault when transport is connected and every axis is disabled; it clears timestamps and duplicate-frame history.

- [ ] **Step 4: Run focused safety tests**

Run the Step 2 command.

Expected: locked faults cannot be cleared by normal frames; debug faults brake until all-axis disable; disconnect cannot be reset.

- [ ] **Step 5: Commit**

```bash
git add src/motor_sim/include/motor_sim/core/safety_monitor.hpp src/motor_sim/src/core/safety_monitor.cpp src/motor_sim/test/test_safety_monitor.cpp
git commit -m "feat: add explicit fault reset and safe rearm"
```

### Task 4: Integrate Lifecycle and Gate into `FrameRouter`

**Files:**
- Modify: `src/motor_sim/include/motor_sim/core/frame_router.hpp`
- Modify: `src/motor_sim/src/core/frame_router.cpp`
- Create: `src/motor_sim/test/test_frame_router.cpp`
- Modify: `src/motor_sim/CMakeLists.txt`

**Interfaces:**
- Consumes: `DriveLifecycle`, `SafetyMonitor`, decoded protocol control/motion commands.
- Produces: `DriveState drive_state() const`, `void set_control_gate(bool)`, `ResetFaultResult reset_fault()`, and guaranteed all-zero `compute_efforts()` unless state is `MotorControlActive`.

- [ ] **Step 1: Add two-axis router tests using the fixture YAML**

Use `test/fixtures/two_axis.yaml` and the existing protocol encoders to prove: startup zero effort and `BrakeEngaged`; one enabled/fresh axis remains `AwaitingFirstCommand`; both axes enabled plus current-cycle motion commands becomes `ReadyForControl`; efforts remain zero before gate; gate true changes state to `MotorControlActive` and permits non-zero effort; disable/timeout/fault immediately returns all-zero effort; reset/rearm cannot reuse pre-fault commands.

- [ ] **Step 2: Run and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select motor_sim --ctest-args -R test_frame_router --output-on-failure
```

Expected: compilation fails on `drive_state()` and `set_control_gate()`.

- [ ] **Step 3: Route protocol events into lifecycle state**

Call `lifecycle_.set_enabled()` for control frames and CERB embedded enable, and call `observe_fresh_command()` only after a valid, in-limit motion command is accepted while that motor is enabled. After every safety tick or immediate fault, call `enter_fault(fault_latching_enabled_)`. Reject control and motion frames while locked except disable frames needed to satisfy reset prerequisites.

- [ ] **Step 4: Gate plant output**

In `compute_efforts()`, tick safety first, synchronize lifecycle, and return a vector of configured-axis zeros unless `lifecycle_.state() == DriveState::MotorControlActive`. Never preserve or replay a non-zero vector across a gate transition.

- [ ] **Step 5: Run router and existing protocol tests**

```bash
colcon test --packages-select motor_sim --ctest-args -R 'test_(frame_router|motor_protocol|cerb_protocol|plant_model)' --output-on-failure
```

Expected: all selected tests pass and golden protocol vectors are unchanged.

- [ ] **Step 6: Commit**

```bash
git add src/motor_sim/include/motor_sim/core/frame_router.hpp src/motor_sim/src/core/frame_router.cpp src/motor_sim/test/test_frame_router.cpp src/motor_sim/CMakeLists.txt
git commit -m "feat: gate motor effort with drive lifecycle"
```

### Task 5: Expose Drive State, Gate, Reset, and Diagnostics in `MotorSimNode`

**Files:**
- Modify: `src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp`
- Modify: `src/motor_sim/src/ros/motor_sim_node.cpp`
- Modify: `src/motor_sim/test/test_motor_sim_node.cpp`
- Modify: `src/motor_sim/CMakeLists.txt`
- Modify: `src/motor_sim/package.xml`

**Interfaces:**
- Consumes: `FrameRouter::drive_state()`, `set_control_gate(bool)`, `reset_fault()`.
- Produces: relative `std_msgs/msg/String drive_state`; relative `std_msgs/msg/Bool control_gate` subscription; relative `std_srvs/srv/Trigger reset_fault`; diagnostics key-values `drive_state`, `brake_engaged`, `control_gate`, `fault`, `fault_latched`, `last_transition_reason`, `reset_rejection_reason`.

- [ ] **Step 1: Add testable node defaults and ROS interface tests**

Extend `NodeDefaults` with:

```cpp
std::string drive_state_topic{"drive_state"};
std::string control_gate_topic{"control_gate"};
std::string reset_fault_service{"reset_fault"};
```

Using the existing injected-config/injected-transport constructor, add tests with a `SingleThreadedExecutor` that subscribe to effort and drive state, publish gate values, and invoke reset. Assert that every published non-active state is accompanied by a six/configured-axis zero effort sample before any later active sample, and reset failures return `success=false` with a stable reason.

- [ ] **Step 2: Run node tests and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select motor_sim --ctest-args -R test_motor_sim_node --output-on-failure
```

Expected: compilation fails because the new ROS members and `std_srvs` dependency are absent.

- [ ] **Step 3: Implement ROS interfaces and publication order**

Declare `drive_state_topic`, `control_gate_topic`, and `reset_fault_service` parameters. In each control tick: compute efforts, publish efforts, then publish the current drive state. On `control_gate=false`, update the router before the next effort calculation so the next published vector is zero. Publish drive state every control tick so the coordinator can distinguish the post-gate zero-effort confirmation from the original fault notification.

- [ ] **Step 4: Implement reset and structured diagnostics**

Map every `ResetFaultResult` to a deterministic Chinese service message. Populate `diagnostic_msgs/msg/KeyValue` entries for all fields listed in **Produces**; use ERROR for locked/unrecoverable faults, WARN for brake/rearm states, and OK only for ready/active states.

- [ ] **Step 5: Add dependencies and run tests**

Add `find_package(std_srvs REQUIRED)`, `ament_target_dependencies(... std_srvs)`, and `<depend>std_srvs</depend>`. Run:

```bash
colcon test --packages-select motor_sim --event-handlers console_direct+
colcon test-result --verbose
```

Expected: every `motor_sim` test passes, including PTY and SocketCAN parameter tests.

- [ ] **Step 6: Commit**

```bash
git add src/motor_sim/include/motor_sim/ros/motor_sim_node.hpp src/motor_sim/src/ros/motor_sim_node.cpp src/motor_sim/test/test_motor_sim_node.cpp src/motor_sim/CMakeLists.txt src/motor_sim/package.xml
git commit -m "feat: expose motor drive safety interfaces"
```

### Task 6: Add the Robot-Side Brake Coordinator

**Files:**
- Create: `src/robot_arm_gazebo/include/robot_arm_gazebo/brake_coordinator.hpp`
- Create: `src/robot_arm_gazebo/src/brake_coordinator.cpp`
- Create: `src/robot_arm_gazebo/src/brake_coordinator_main.cpp`
- Create: `src/robot_arm_gazebo/test/test_brake_coordinator.cpp`
- Modify: `src/robot_arm_gazebo/CMakeLists.txt`
- Modify: `src/robot_arm_gazebo/package.xml`

**Interfaces:**
- Consumes: `std_msgs/msg/String drive_state`, `/controller_manager/switch_controller`, parameters `controller_name`, `controller_manager_name`, `controller_switch_timeout`.
- Produces: `std_msgs/msg/Bool control_gate`, coordinator diagnostics, process exit failure after unavailable/failed/timed-out STRICT switch.

- [ ] **Step 1: Test a pure coordinator decision state machine**

Define `CoordinatorAction { None, PublishGateFalse, ActivateController, DeactivateController, FailFast }` and a pure `BrakeCoordinatorStateMachine`. Tests must prove:

- repeated identical states do not issue duplicate switch requests;
- `ready_for_control` requests activation exactly once;
- gate becomes true only after a successful activation response;
- brake/fault first emits gate false, then waits for a subsequent same non-active drive-state sample (the motor node's post-zero confirmation), then requests deactivation;
- deactivation success records brake closed; timeout or rejected STRICT response returns `FailFast`;
- a stale activation response after a brake request cannot reopen the gate.

- [ ] **Step 2: Run coordinator tests and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R test_brake_coordinator --output-on-failure
```

Expected: compilation fails because coordinator files do not exist.

- [ ] **Step 3: Implement the ROS node around the tested state machine**

Use `controller_manager_msgs/srv/SwitchController` with `strictness=STRICT`, `activate_asap=true`, and the configured timeout. Activation request contains only `effort_controller`; deactivation request contains only `effort_controller`. Make service availability and response timeout timers explicit; set `shutdown_requested()` on failure so `main()` returns exit code `1` after `rclcpp::shutdown()`.

- [ ] **Step 4: Add diagnostics and idempotency tests**

Diagnostics must expose `drive_state`, `brake_engaged`, actual controller state last confirmed by the service, `last_transition_reason`, and switch failure text. Test duplicate input, delayed response, timeout, and failure with a fake switch service.

- [ ] **Step 5: Add build/package dependencies and run focused tests**

Add executable `robot_arm_brake_coordinator`, `controller_manager_msgs` dependency, and link/install rules. Run the Step 2 command.

Expected: coordinator state-machine and ROS service tests pass without Gazebo.

- [ ] **Step 6: Commit**

```bash
git add src/robot_arm_gazebo/include/robot_arm_gazebo/brake_coordinator.hpp src/robot_arm_gazebo/src/brake_coordinator.cpp src/robot_arm_gazebo/src/brake_coordinator_main.cpp src/robot_arm_gazebo/test/test_brake_coordinator.cpp src/robot_arm_gazebo/CMakeLists.txt src/robot_arm_gazebo/package.xml
git commit -m "feat: coordinate simulated arm brakes"
```

### Task 7: Enable Configurable Model Self-Collision

**Files:**
- Modify: `src/robot_arm_gazebo/urdf/ur5e.urdf.xacro`
- Modify: `src/robot_arm_gazebo/test/test_ur5e_model.py`

**Interfaces:**
- Consumes: xacro argument `self_collision_enabled` with string boolean values.
- Produces: final generated SDFormat model with `<self_collide>true</self_collide>` by default and false only when explicitly overridden.

- [ ] **Step 1: Add source and final-SDF tests**

Extend the xacro helper to accept `self_collision_enabled:=true/false`. Assert seven collision geometries remain. Run `gz sdf -p <expanded-urdf-file>` in a temporary directory and parse the result to assert model-level `self_collide` is true by default and false when overridden. Assert there are no explicit adjacent-link collision pairs that counteract SDFormat's direct-joint exclusion.

- [ ] **Step 2: Run and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R test_ur5e_model --output-on-failure
```

Expected: the new assertion fails because final SDF lacks model-level self-collision.

- [ ] **Step 3: Add the xacro argument and Gazebo model property**

Declare `<xacro:arg name="self_collision_enabled" default="true"/>` and emit `<self_collide>$(arg self_collision_enabled)</self_collide>` in the model's Gazebo block. Do not alter collision meshes, gravity, inertia, joint damping, or adjacency rules in this step.

- [ ] **Step 4: Run model and vendor tests**

```bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R 'test_(ur5e_model|vendor_assets)' --output-on-failure
```

Expected: final SDF reports self-collision true, all seven meshes remain, and vendor integrity tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/robot_arm_gazebo/urdf/ur5e.urdf.xacro src/robot_arm_gazebo/test/test_ur5e_model.py
git commit -m "feat: enable ur5e self collision"
```

### Task 8: Rework Unified Launch for Inactive Effort Control and Brake Handshake

**Files:**
- Modify: `src/robot_arm_gazebo/launch/gazebo.launch.py`
- Modify: `src/robot_arm_gazebo/test/test_launch_contract.py`

**Interfaces:**
- Consumes: motor node and coordinator interfaces from Tasks 5-6, xacro argument from Task 7.
- Produces launch arguments: `drive_state_topic=drive_state`, `control_gate_topic=control_gate`, `reset_fault_service=reset_fault`, `controller_switch_timeout=2.0`, `self_collision_enabled=true`; effort controller initially `inactive`; both backend nodes started exactly once and fail-fast monitored.

- [ ] **Step 1: Save the user's current launch diff again**

```bash
git diff -- src/robot_arm_gazebo/launch/gazebo.launch.py > /tmp/robot_arm_launch_user_before_brake.patch
```

Expected: this captures only the pre-existing default changes so they can be distinguished from feature edits during staging.

- [ ] **Step 2: Update static launch tests first**

Require all five new arguments and non-empty Chinese descriptions. Assert the effort spawner includes `--inactive`; after its successful exit, one grouped stage starts exactly one `robot_arm_motor_sim_node` and one `robot_arm_brake_coordinator`; both share the configured namespace/topics; either process exiting returns a `Shutdown`. Assert no action-player process exists.

Update override tests to resolve these exact parameters:

```python
{
    "drive_state_topic": "/test/drive_state",
    "control_gate_topic": "/test/control_gate",
    "reset_fault_service": "/test/reset_fault",
    "controller_switch_timeout": 3.5,
    "self_collision_enabled": False,
}
```

- [ ] **Step 3: Run launch tests and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R test_launch_contract --output-on-failure
```

Expected: failures report missing arguments, active effort spawner, and missing coordinator.

- [ ] **Step 4: Implement launch parameters with Chinese comments**

Add every new value to `LAUNCH_DEFAULTS`, a preceding Chinese comment explaining units/safety purpose, and a Chinese `description`. Pass `self_collision_enabled` into xacro. Remap/parameterize motor state, gate and reset interfaces; give coordinator the fully namespaced controller manager and configured timeout.

- [ ] **Step 5: Change startup chain**

Start `joint_state_broadcaster` active, then invoke the effort spawner with `--inactive`, then start motor and coordinator together. Register fail-fast exit handlers for both long-running nodes. Do not let the coordinator activate effort control before `ready_for_control`.

- [ ] **Step 6: Verify the user's defaults survived and stage selectively**

```bash
git diff -- src/robot_arm_gazebo/launch/gazebo.launch.py
git diff --check
```

Expected: feature changes coexist with the user's original defaults. Stage the launch file interactively (`git add -p`) so the pre-existing connection-default hunks are not included in the feature commit; stage the test file normally.

- [ ] **Step 7: Run launch/model tests**

```bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R 'test_(launch_contract|ur5e_model)' --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 8: Commit**

```bash
git add src/robot_arm_gazebo/test/test_launch_contract.py
git add -p src/robot_arm_gazebo/launch/gazebo.launch.py
git diff --cached --check
git commit -m "feat: launch arm backend with brake coordination"
```

After commit, `git diff -- src/robot_arm_gazebo/launch/gazebo.launch.py` must still show the user's uncommitted default changes.

### Task 9: Run Full Automated and Gazebo/vcan Acceptance

**Files:**
- Create: `docs/verification/2026-08-27-realistic-brake-and-self-collision.md`
- Modify only if acceptance reveals a defect: files owned by Tasks 2-8 plus their focused tests.

**Interfaces:**
- Consumes: the complete unified backend.
- Produces: reproducible evidence for brake hold, handshake, timeout/recovery, vcan motion, self-contact, package boundary, and regression suite.

- [ ] **Step 1: Run repository boundary, build, and full tests**

```bash
source /opt/ros/humble/setup.bash
colcon list --names-only
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: package list is exactly `motor_sim` and `robot_arm_gazebo`; build succeeds with warning flags; every test passes.

- [ ] **Step 2: Verify launch arguments and startup brake**

```bash
ros2 launch robot_arm_gazebo gazebo.launch.py --show-args
ros2 launch robot_arm_gazebo gazebo.launch.py transport_mode:=socket_can arm_port:=vcan0 fault_latching_enabled:=true
```

In another terminal record `/robot_arm/drive_state`, `/robot_arm/joint_states`, `/robot_arm/diagnostics`, and controller states. Expected: startup state is `brake_engaged`, effort controller is inactive, and every joint drifts at most `0.01 rad` over 10 seconds.

- [ ] **Step 3: Verify partial preparation remains braked**

Send valid enable and motion frames to only five configured axes through `vcan0`. Expected: `awaiting_first_command`, effort controller inactive, gate false, zero effort, and no drift beyond the threshold.

- [ ] **Step 4: Verify all-axis release and actual motion**

Send the sixth axis's current-cycle valid frame. Expected sequence: `ready_for_control` -> controller active -> gate true -> `motor_control_active`. Send a known Vaxor position command and record joint movement toward the target plus CAN reply `0x711`, retaining the existing end-to-end protocol expectations.

- [ ] **Step 5: Verify timeout and both recovery modes**

Stop commands for more than `150 ms`. In locked mode expect zero effort, brake hold at the fault pose, `fault_latched`, ordinary commands unable to resume, successful `/robot_arm/reset_fault` only after all-axis disable, and re-enable plus six fresh commands restoring control without restarting Gazebo. Relaunch with `fault_latching_enabled:=false`; after another timeout, verify stale commands cannot resume and explicit all-axis disable followed by a new enable/fresh-command cycle can recover.

- [ ] **Step 6: Verify self-collision in Gazebo**

Drive a reproducible pose where two non-adjacent links contact. Record pose, link pair, contact behavior, and real-time factor. Expected: no visible interpenetration, sustained explosion, model flight, or unacceptable persistent real-time-factor degradation; adjacent joint-connected links remain usable under SDFormat exclusion.

- [ ] **Step 7: Verify fail-fast**

Bring `vcan0` down during a run and separately test an invalid controller-manager endpoint. Expected: gate false/zero effort is issued, diagnostics reports the cause, and the unified launch shuts down rather than claiming brake/controller success.

- [ ] **Step 8: Write evidence and fix only proven defects test-first**

Record exact commands, timestamps, state sequences, joint deltas, CAN IDs, reset responses, collision pair and full test summary in the verification document. If a check fails, add a focused failing regression test, implement the smallest correction in the owning task's files, rerun focused plus full tests, and record the corrected evidence.

- [ ] **Step 9: Commit verification**

```bash
git add docs/verification/2026-08-27-realistic-brake-and-self-collision.md
git commit -m "test: verify realistic arm brake backend"
```

### Task 10: Synchronize User and Architecture Documentation

**Files:**
- Modify: `README.md`
- Modify: `CONTEXT.md`
- Verify: `docs/research/2026-08-27-robot-arm-simulation-debugging-practices.md`
- Verify: `docs/superpowers/specs/2026-08-27-realistic-brake-and-self-collision-design.md`

**Interfaces:**
- Consumes: verified behavior and exact launch/service/topic names from Tasks 2-9.
- Produces: one-launch backend usage and debugging/recovery instructions consistent with actual implementation.

- [ ] **Step 1: Add documentation contract tests**

Extend the existing package-surface or launch-contract tests to require README references to `drive_state`, `control_gate`, `reset_fault`, `fault_latching_enabled`, `self_collision_enabled`, `vcan0`, and the warning that simulated brake/self-collision parameters are not manufacturer safety specifications.

- [ ] **Step 2: Run the documentation contract and confirm red**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_arm_gazebo --ctest-args -R 'test_(package_identity|launch_contract)' --output-on-failure
```

Expected: documentation assertions fail until the new interfaces are documented.

- [ ] **Step 3: Update README**

Document: single-launch command; all launch arguments and Chinese meanings; startup/enable/first-command/brake sequence; locked reset procedure; unlocked disable-enable procedure; `/dev/ttyUSB0`、`/dev/ttyACM0` are common transient serial names but `/dev/serial/by-id/...` is required for a stable serial binding; `vcan0` setup and CAN control example; `--show-args`; diagnostics interpretation; self-collision override and limitations.

- [ ] **Step 4: Update CONTEXT.md**

Describe ownership boundaries: generic lifecycle remains in `motor_sim`; controller switching and UR5e model policy remain in `robot_arm_gazebo`; document five drive-state values, gate ordering, reset preconditions, fail-fast behavior, and why `hold_joints` is a simulation mechanism rather than a real brake model.

- [ ] **Step 5: Run full verification one final time**

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
git diff --check
git status --short
```

Expected: all tests pass; only the user's intentionally uncommitted launch-default hunks remain outside planned commits.

- [ ] **Step 6: Commit docs without the user's launch defaults**

```bash
git add README.md CONTEXT.md src/robot_arm_gazebo/test
git diff --cached --check
git commit -m "docs: explain simulated brake backend workflow"
```
