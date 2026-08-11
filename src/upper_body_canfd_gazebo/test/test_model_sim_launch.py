"""静态检查 model_sim 入口（Gazebo 模型 + C++ 模拟器），不启动 Gazebo 或任何进程。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from motor_canfd_sim.helpers import evaluate_node_parameters, expected_default_config_path


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
    parameters = evaluate_node_parameters(context, simulator)
    injected = next(parameter for parameter in parameters if isinstance(parameter, dict))

    expected_config = expected_default_config_path()
    assert Path(injected["motor_config_file"]).resolve() == expected_config.resolve()
    assert injected["bus_override.left_arm"] == "vcan0"
    assert injected["bus_override.right_arm"] == "vcan1"
