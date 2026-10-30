"""Retired host-to-motor entry point.

Every real motor is controlled by the micro-ROS MCU. This compatibility file
fails before constructing or launching a node, including when old callers pass
CAN device overrides. Offline controller tests use mock_control.launch.py.
"""


def generate_launch_description():
    raise RuntimeError(
        "Host-to-motor hardware.launch.py has been retired. Real robots must use "
        "ros2 launch robot_bringup managed_robot.launch.py through micro-ROS MCU. "
        "For offline GenericSystem controller tests use "
        "ros2 launch motor_hardware_interface mock_control.launch.py.")
