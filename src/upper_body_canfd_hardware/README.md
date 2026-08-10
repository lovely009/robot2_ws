# upper_body_canfd_hardware

C++ ros2_control hardware layer for the upper-body CAN-FD motors: YiYou
RP-series joints (MIT broadcast, CAN ID `0x80`) and 25/30 mm wrist drives
(per-node MIT frames, `0x501+4N` / `0x701+4N` / `0x702+4N`).

## Architecture

One `CanFdMotorSystem` instance owns **one** CAN-FD bus and implements the
`hardware_interface::SystemInterface` lifecycle:

- `on_init` — parses per-joint URDF parameters and validates them.
- `on_configure` — opens the SocketCAN bus, puts wrist drives into MIT mode,
  and sends disable frames. Refuses `can*` interfaces unless
  `allow_hardware_interfaces:=true`.
- `on_activate` — sends enable frames, arms the feedback watchdog.
- `read()` — drains the bus, decodes RP state frames (CAN ID = motor ID) and
  25/30 mm MIT replies/status frames into `position`/`velocity`/`effort`
  state interfaces. Missing feedback for more than `feedback_timeout_cycles`
  cycles returns `ERROR` (safe stop + disable).
- `write()` — maps each joint `effort` command to MIT frames: RP broadcast
  with kp/kd from parameters and effort as feed-forward torque; per-node
  wrist frames with effort converted to current. Commands clamp to the URDF
  effort limits; with no feedback yet a zero-gain coast command is sent.
  A command watchdog zeroes the bus when controllers stop writing.
- `on_deactivate` / `on_error` / `on_cleanup` — zero commands, disable, close.

## URDF parameters

Hardware block:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `interface` | `vcan0` | SocketCAN interface for this bus |
| `allow_hardware_interfaces` | `false` | Permit physical `can*` interfaces |
| `feedback_timeout_cycles` | `50` | Missing-feedback error threshold (500 Hz loop ⇒ 100 ms) |
| `command_timeout_ms` | `150` | Zero commands after controller silence |

Per joint:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `motor_type` | `wrist` | `rp` or `wrist` |
| `can_node_id` | `1` | RP 1..8, wrist 1..60 |
| `direction` | `1.0` | Sign of the joint↔motor mapping |
| `zero_offset` | `0.0` | Motor position offset (rad) |
| `kp` / `kd` | RP 36/2.0, wrist 0.008/1.0 | MIT gains |
| `effort_scale` | `1.0` | Joint Nm → motor FF units (A for wrist) |

The robot description in `upper_body_canfd_gazebo/urdf/upper_body.urdf.xacro`
switches between Gazebo and this plugin with
`use_real_hardware:=true|false`.

## Build and test

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --symlink-install --packages-select upper_body_canfd_hardware
source install/setup.bash
```

Unit tests cover the protocol codecs (known byte sequences from the manuals,
round-trips) and the plugin against vcan with an in-process fake slave.
They need `vcan0`/`vcan1` and raw-socket permission (run with sudo-capable
shell if sandboxed):

```bash
bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh
colcon test --packages-select upper_body_canfd_hardware
colcon test-result --verbose
```

## Run against the emulator (no Gazebo)

```bash
ros2 launch upper_body_canfd_gazebo hardware_sim.launch.py
ros2 topic echo /joint_states
```

## Real hardware bring-up

1. Configure the physical buses (arbitration 1 Mbps, data 5 Mbps):

   ```bash
   sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
   sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
   ```

2. Launch with explicit hardware permission:

   ```bash
   ros2 launch upper_body_canfd_gazebo real_hardware.launch.py \
       left_interface:=can0 right_interface:=can1 allow_hardware:=true
   ```

3. Bring-up order: verify feedback on `/joint_states` with zero commands,
   then move one joint with a small effort command before enabling the full
   arm. `upper_body_canfd_hardware` refuses `can*` interfaces unless
   `allow_hardware:=true` is passed on purpose.
