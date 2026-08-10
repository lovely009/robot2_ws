"""静态检查 Gazebo C++ 模拟器入口，不启动 Gazebo 或任何进程。"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "gazebo_cpp_sim.launch.py"
    spec = importlib.util.spec_from_file_location("gazebo_cpp_sim_launch", launch_file)
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
