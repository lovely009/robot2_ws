# motor_hardware_interface

ROS 2 Humble `ros2_control` adapter for `motor_canfd_drive`.

The plugin is a C++ `hardware_interface::SystemInterface`; Python ROS 2 nodes are
still fully compatible through normal controller Topic / Action / Service APIs.
No pybind11 or Python wrapper is used for the low-level driver.

## Boundary

```text
controller_manager / ros2_control
              |
              v
motor_hardware_interface
  - lifecycle
  - ROS joint interfaces
  - YAML command/timing parameters
  - joint-name <-> motor-index mapping
              |
              v
motor_canfd_drive::MotorCanfdDriver
  - bus/type grouping
  - protocol encode/decode
  - SocketCAN
```

There is no fixed arm count or fixed joint count. The plugin validates that every
joint declared in its `<ros2_control>` block has exactly one YAML motor mapping,
then maps by **joint name**, not by array position.

## Configuration

`config/motor_canfd.yaml` contains:

- `buses`: arbitrary logical bus names and SocketCAN interfaces;
- `motor_types`: shared command gains and driver parameters per type;
- `motors`: arbitrary motor entries with `joint`, `bus`, `type` and `id`;
- `timing`: feedback/activation timeouts.

Example:

```yaml
motor_hardware:
  buses:
    left_arm: {interface: can0}
  motor_types:
    rp:
      command: {kp: 30.0, kd: 1.0}
  motors:
    shoulder_yaw:
      joint: left_shoulder_yaw_joint
      bus: left_arm
      type: rp
      id: 1
```

Per-motor `command` and `parameters` maps override the defaults for that motor
type. Robot-specific launch files may override any bus using a hardware
parameter named `bus_override.<logical_bus_name>`.
