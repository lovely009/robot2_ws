# motor_canfd_drive

ROS-independent C++ CAN-FD motor driver. This package owns SocketCAN transport,
wire codecs, motor-family protocol strategies and the generic motor/bus mapping.
It does **not** know what an arm is, how many joints the robot has, or how ROS 2
controllers are arranged.

## Runtime model

```text
DriverConfig
  buses[]  -> logical bus name + SocketCAN interface
  motors[] -> motor name + joint + bus + type + id + parameters
        |
        v
MotorCanfdDriver
        |
        +-- group by (bus, type) --> MotorProtocol strategy
        |                            +-- rp
        |                            +-- motor_2530
        |                            +-- future protocol
        v
     SocketCAN
```

`MotorCanfdDriver` uses `std::vector`, so motor/bus counts are configuration
driven. RP's 8-slot broadcast limit remains a property of the RP protocol
strategy, not a robot-wide seven-joint limit. Other protocols can have their own
limits without changing the generic driver.

## Adding another motor family

1. Implement `MotorProtocol` for the new wire protocol.
2. Register one factory in `MotorProtocolRegistry` (or provide a registry with
   the factory already registered).
3. Add the new `type` and its parameters to YAML.

The generic send/receive loops, bus management and ROS 2 hardware adapter do not
need a new `if (type == ...)` branch.

## YAML ownership

`load_driver_config_yaml()` reads only `buses`, `motors`, and type/per-motor
driver parameters. ROS-specific timing and command gains are intentionally not
interpreted here. `yaml-cpp` is the only configuration dependency; the library
has no `rclcpp`, `hardware_interface`, `pluginlib`, Topic, Action or Service
dependency.
