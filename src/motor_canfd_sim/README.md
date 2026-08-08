# motor_canfd_sim

C++ motor-side CAN-FD simulator for the built-in `rp` and `motor_2530` families.
It reads the same YAML schema as `motor_hardware_interface`, then reuses
`motor_canfd_drive::DriverConfig` validation, SocketCAN transport and wire
codecs. The original Python Gazebo/CAN simulation remains untouched.

The simulator has no compiled-in `2 arms x 7 joints` topology. It builds buses
and motor state vectors from the same `motor_canfd.yaml` used by the physical
`ros2_control` hardware plugin.

For simulation, logical buses are safely redirected to virtual CAN with ROS
parameters named `bus_override.<logical_bus_name>`, for example:

```yaml
motor_canfd_sim:
  ros__parameters:
    bus_override.left_arm: vcan0
    bus_override.right_arm: vcan1
```

By default the emulator refuses to open interfaces whose names do not start with
`vcan`. `allow_hardware_interfaces:=true` must be set explicitly for intentional
HIL testing.

The current upper-body Gazebo effort controller still determines the order of
the outgoing effort array, so its controller joint list and the YAML motor map
must stay in the same order when the Gazebo robot model itself is changed.
