# upper_body_canfd_hardware

`ros2_control` hardware plugin for the physical 14-axis upper body. It is the only layer that
knows both the ROS joint model and the two private motor protocols; controllers remain independent
of SocketCAN and can therefore run unchanged against simulation or hardware.

## Architecture

```text
ROS controller -> position interfaces -> CanFdSystem -> can0/can1 -> drives
ROS controller <- state interfaces    <- CanFdSystem <- can0/can1 <- feedback
```

Each bus owns RP IDs 1..4 and wrist nodes 1..3. The joint order is validated from the URDF.
RP commands are sent as one neutral-padded 64-byte broadcast. Wrist commands use one 12-byte MIT
frame per node. Activation holds the observed pose before enabling drives; deactivation always
sends disable frames. Missing feedback makes `read()` fail after `feedback_timeout_s`.

## Physical bus setup

Configure both interfaces before starting ROS (the project correction is 1 Mbit/s arbitration,
5 Mbit/s data, CAN-FD with BRS):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up
ros2 launch upper_body_canfd_gazebo hardware.launch.py
```

Send 14 positions in the documented joint order to `/position_controller/commands`. Do not run the
legacy direct-CAN demo/action processes at the same time: a bus must have exactly one command owner.
Before first hardware motion, verify each drive ID, zero, direction, limit, emergency stop and gain
with the mechanism unloaded. Defaults are commissioning values, not a safety certification.
